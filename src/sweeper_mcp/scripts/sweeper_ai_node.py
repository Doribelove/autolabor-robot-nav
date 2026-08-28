#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Qt-authorised cloud planning and sequential MCP execution node."""

from collections import deque
import hmac
import json
import os
import secrets
import stat
import threading
import time
import uuid

import rospy
import yaml

from sweeper_mcp.agent import AgentRunner
from sweeper_mcp.asr_client import AsrClient
from sweeper_mcp.msg import AiControlStatus, AiEvent
from sweeper_mcp.srv import (
    CancelAiTask, CancelAiTaskResponse,
    SetAiAuthorization, SetAiAuthorizationResponse,
    SetAsrModel, SetAsrModelResponse,
    SetAsrRecording, SetAsrRecordingResponse,
    SetSmartVoice, SetSmartVoiceResponse,
    SubmitAiText, SubmitAiTextResponse,
)


SMART_VOICE_QUEUE_LIMIT = 8
SMART_VOICE_QUEUE_TTL_S = 30.0
DEFAULT_ASR_MODEL = "medium"
ASR_MODEL_SPECS = {
    "small": {
        "filename": "small.pt",
        "sha256": "9ecf779972d90ba49c06d968637d720dd632c55bbf19d441fb42bf17a411e794",
    },
    "medium": {
        "filename": "medium.pt",
        "sha256": "345ae4da62f9b3d59415adc60127b97c714f32e89e936602e85993674d08dcb1",
    },
    # The Qt label is deliberately short; this is OpenAI Whisper large-v3.
    "large": {
        "filename": "large-v3.pt",
        "sha256": "e5b1a55b89c1367dacf97e3e19bfd829a01529dbfdeefa8caeb59b3f1b81dadb",
    },
}


def _canonical_asr_model(value):
    name = str(value or "").strip().lower()
    if name == "large-v3":
        name = "large"
    return name if name in ASR_MODEL_SPECS else ""


class SweeperAiNode:
    def __init__(self):
        self._lock = threading.RLock()
        self._session_token = os.environ.get("SWEEPER_AI_SESSION_TOKEN", "")
        if not self._session_token:
            # A standalone node remains locked because no UI knows this token.
            self._session_token = secrets.token_urlsafe(32)
            rospy.logwarn("SWEEPER_AI_SESSION_TOKEN missing; control services remain UI-locked")
        self._control_capability = secrets.token_urlsafe(32)
        self._voice_authorized = False
        self._parse_authorized = False
        self._control_authorized = False
        self._last_heartbeat = 0.0
        self._task_active = False
        self._request_id = ""
        self._phase = "IDLE"
        self._current_step = 0
        self._total_steps = 0
        self._last_cloud_rtt_ms = 0.0
        self._last_total_latency_ms = 0.0
        self._last_http_status = 0
        self._last_error = ""
        self._final_text = ""
        self._event_sequence = 0

        # ASR lives in a separate, unprivileged subprocess.  The epoch makes
        # every result revocable: inference may finish after microphone
        # authorization was removed, but such a result is never displayed or
        # sent to the cloud.
        self._asr_client = None
        self._asr_epoch = 0
        self._asr_worker_generation = 0
        self._asr_switching = False
        self._asr_capture_id = ""
        self._asr_available = False
        self._asr_model_loaded = False
        self._asr_recording = False
        self._asr_phase = "DISABLED"
        self._asr_audio_duration_s = 0.0
        self._asr_latency_ms = 0.0
        self._asr_last_error = ""

        # Smart voice is an explicit, revocable continuous-listening session.
        # Worker session IDs reject cross-session late events; the local epoch
        # also invalidates queued/transcribing results on every stop/revoke.
        self._smart_voice_enabled = False
        self._smart_voice_listening = False
        self._smart_voice_session_id = ""
        self._smart_voice_epoch = 0
        self._smart_voice_utterance_count = 0
        self._smart_voice_pending = deque()
        self._smart_voice_seen = set()

        config, config_error = self._load_config()
        agent = config.get("agent", {}) if isinstance(config, dict) else {}
        asr = config.get("asr", {}) if isinstance(config, dict) else {}
        self._model = str(agent.get("model", "deepseek-v4-flash"))
        api_key = os.environ.get("DEEPSEEK_API_KEY", "") or str(
            agent.get("api_key", ""))
        self._cloud_configured = bool(api_key and not config_error)
        if config_error:
            self._last_error = config_error

        self._asr_enabled = bool(asr.get("enabled", False)) and not config_error
        requested_asr_model = _canonical_asr_model(
            asr.get("model", DEFAULT_ASR_MODEL))
        self._asr_model = requested_asr_model or DEFAULT_ASR_MODEL
        self._asr_device = str(asr.get("device", "cuda"))
        self._asr_python = os.environ.get(
            "SWEEPER_ASR_PYTHON", str(asr.get("python", ""))).strip()
        self._asr_worker_script = os.environ.get(
            "SWEEPER_ASR_WORKER", str(asr.get("worker_script", ""))).strip()
        if not self._asr_worker_script:
            self._asr_worker_script = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "whisper_asr_worker.py")
        legacy_model_path = os.environ.get(
            "SWEEPER_ASR_MODEL_PATH", str(asr.get("model_path", ""))).strip()
        self._asr_model_dir = os.environ.get(
            "SWEEPER_ASR_MODEL_DIR", str(asr.get("model_dir", ""))).strip()
        if not self._asr_model_dir and legacy_model_path:
            self._asr_model_dir = os.path.dirname(
                os.path.abspath(legacy_model_path))
        self._asr_common_config = {
            "device": self._asr_device,
            "arecord_path": str(asr.get("arecord_path", "/usr/bin/arecord")),
            "input_device": str(asr.get("input_device", "")),
            "sample_rate": int(asr.get("sample_rate", 16000)),
            "max_record_seconds": float(asr.get(
                "max_audio_seconds", asr.get("max_record_seconds", 30.0))),
            "min_record_seconds": float(asr.get(
                "min_audio_seconds", asr.get("min_record_seconds", 0.4))),
            "min_rms": float(asr.get("min_rms", 0.003)),
            "language": str(asr.get("language", "zh")),
            "initial_prompt": str(asr.get(
                "initial_prompt", "简体中文，无人清扫车语音指令")),
            "startup_timeout_s": float(asr.get("startup_timeout_s", 20.0)),
            "smart_chunk_ms": float(asr.get("smart_chunk_ms", 100.0)),
            "smart_speech_start_ms": float(asr.get(
                "smart_speech_start_ms", 200.0)),
            "smart_silence_ms": float(asr.get(
                "smart_silence_ms", 800.0)),
            "smart_pre_roll_ms": float(asr.get(
                "smart_pre_roll_ms", 300.0)),
            "smart_min_utterance_s": float(asr.get(
                "smart_min_utterance_s", 0.4)),
            "smart_max_utterance_s": float(asr.get(
                "smart_max_utterance_s", 15.0)),
            "smart_queue_limit": int(asr.get("smart_queue_limit", 4)),
            "smart_vad_rms": float(asr.get(
                "smart_vad_rms", asr.get("min_rms", 0.003))),
        }
        self._asr_config = self._asr_config_for_model(self._asr_model)
        self._asr_model_path = self._asr_config["model_path"]

        self._status_pub = rospy.Publisher(
            "/sweeper_ai/status", AiControlStatus, queue_size=1, latch=True)
        self._event_pub = rospy.Publisher(
            "/sweeper_ai/events", AiEvent, queue_size=100)
        self._auth_service = rospy.Service(
            "/sweeper_ai/set_authorization", SetAiAuthorization,
            self._set_authorization)
        self._submit_service = rospy.Service(
            "/sweeper_ai/submit_text", SubmitAiText, self._submit_text)
        self._cancel_service = rospy.Service(
            "/sweeper_ai/cancel_task", CancelAiTask, self._cancel_task)
        self._asr_recording_service = rospy.Service(
            "/sweeper_ai/set_asr_recording", SetAsrRecording,
            self._set_asr_recording)
        self._asr_model_service = rospy.Service(
            "/sweeper_ai/set_asr_model", SetAsrModel,
            self._set_asr_model)
        self._smart_voice_service = rospy.Service(
            "/sweeper_ai/set_smart_voice", SetSmartVoice,
            self._set_smart_voice)

        backend = os.environ.get("SWEEPER_MCP_BACKEND", "ros").strip().lower()
        if backend not in ("ros", "mock"):
            raise RuntimeError("SWEEPER_MCP_BACKEND must be ros or mock")
        self._backend = backend
        self._runner = AgentRunner(
            base_url=str(agent.get("base_url", "https://api.deepseek.com")),
            model=self._model,
            api_key=api_key,
            temperature=float(agent.get("temperature", 0.2)),
            timeout_s=float(agent.get("timeout_s", 20.0)),
            max_retries=int(agent.get("max_retries", 3)),
            max_plan_steps=int(agent.get("max_plan_steps", 8)),
            nav_wait_timeout=float(agent.get("nav_wait_timeout", 240.0)),
            navigation_cleanup_timeout=float(agent.get(
                "navigation_cleanup_timeout", 30.0)),
            coverage_wait_timeout=float(agent.get(
                "coverage_wait_timeout", 3600.0)),
            visual_wait_timeout=float(agent.get("visual_wait_timeout", 300.0)),
            poll_interval=float(agent.get("poll_interval", 0.5)),
            tool_text_limit=int(agent.get("tool_text_limit", 1500)),
            backend=backend,
            control_token=self._control_capability,
            event_callback=self._agent_event,
            authorization_checker=self._execution_authorized,
            cloud_authorization_checker=self._cloud_authorized,
        )
        self._timer = rospy.Timer(rospy.Duration(1.0), self._watchdog)
        rospy.on_shutdown(self._shutdown)
        self._publish_status()
        if self._asr_enabled:
            self._asr_phase = "STARTING"
            threading.Thread(
                target=self._initialize_asr,
                name="sweeper-asr-initialize", daemon=True).start()
        else:
            self._asr_last_error = (
                config_error or "ASR 未在本机配置中启用")
        rospy.loginfo(
            "sweeper_ai ready; model=%s backend=%s cloud_configured=%s "
            "asr_model=%s asr_enabled=%s gates=off",
            self._model, backend, self._cloud_configured,
            self._asr_model, self._asr_enabled)

    @staticmethod
    def _config_path():
        explicit = os.environ.get("SWEEPER_AI_CONFIG", "").strip()
        if explicit:
            return explicit
        import rospkg
        return os.path.join(rospkg.RosPack().get_path("sweeper_mcp"),
                            "config", "sweeper_mcp.yaml")

    def _load_config(self):
        path = self._config_path()
        try:
            info = os.lstat(path)
        except OSError as exc:
            return {}, "AI 配置不可读: %s" % exc
        if stat.S_ISLNK(info.st_mode):
            return {}, "AI 配置不能是符号链接"
        if info.st_uid != os.geteuid() or stat.S_IMODE(info.st_mode) != 0o600:
            return {}, "AI 配置必须由当前用户持有且权限为 0600"
        try:
            with open(path, "r", encoding="utf-8") as stream:
                value = yaml.safe_load(stream) or {}
        except (OSError, yaml.YAMLError) as exc:
            return {}, "AI 配置解析失败: %s" % exc
        if not isinstance(value, dict):
            return {}, "AI 配置根节点必须是对象"
        return value, ""

    def _asr_config_for_model(self, model):
        canonical = _canonical_asr_model(model)
        if not canonical:
            raise ValueError("ASR 模型必须是 small、medium 或 large")
        spec = ASR_MODEL_SPECS[canonical]
        config = dict(self._asr_common_config)
        config.update({
            "model": canonical,
            "model_path": (
                os.path.join(self._asr_model_dir, spec["filename"])
                if self._asr_model_dir else ""),
            "model_sha256": spec["sha256"],
        })
        return config

    def _asr_event_for_generation(self, generation, event):
        # Closing/replacing a worker emits a final EOF status.  Keep the lock
        # for the complete callback so an old worker can never overwrite the
        # status or submit a late smart transcript after a successful swap.
        with self._lock:
            if generation != self._asr_worker_generation:
                return
            if self._asr_switching:
                return
            self._asr_event(event)

    def _new_asr_client(self, config, generation):
        return AsrClient(
            self._asr_python, self._asr_worker_script, dict(config),
            event_callback=lambda event: self._asr_event_for_generation(
                generation, event))

    def _initialize_asr(self):
        """Start the isolated worker without opening the microphone."""
        client = None
        diagnostic = ""
        try:
            if not self._asr_python:
                raise RuntimeError("ASR Python 未配置")
            if not os.path.isfile(self._asr_worker_script):
                raise RuntimeError(
                    "ASR worker 不存在: %s" % self._asr_worker_script)
            with self._lock:
                self._asr_worker_generation += 1
                generation = self._asr_worker_generation
            client = self._new_asr_client(self._asr_config, generation)
            status = client.start()
            with self._lock:
                self._apply_asr_status_locked(status or {})
                if not self._asr_available:
                    raise RuntimeError(
                        self._asr_last_error or "ASR worker 未通过就绪检查")
                self._asr_client = client
                self._asr_phase = str(
                    (status or {}).get("state", "READY")).upper()
                self._asr_last_error = ""
            client = None  # ownership moved to self._asr_client
            rospy.loginfo(
                "ASR worker ready; model=%s device=%s checkpoint=%s",
                self._asr_model, self._asr_device, self._asr_model_path)
        except Exception as exc:
            rospy.logerr("ASR worker unavailable: %s", exc)
            diagnostic = str(exc)
            with self._lock:
                if self._asr_client is not None:
                    failed_client = self._asr_client
                    self._asr_client = None
                else:
                    failed_client = None
                self._asr_available = False
                self._asr_recording = False
                self._asr_phase = "UNAVAILABLE"
                self._asr_last_error = str(exc)
            if failed_client is not None:
                try:
                    failed_client.close()
                except Exception:
                    pass
        finally:
            if client is not None:
                try:
                    client.close()
                except Exception:
                    pass
            # close() intentionally reports worker EOF through the callback.
            # Restore the concrete configuration diagnostic after the reader
            # thread has joined so Qt does not replace it with a generic EOF.
            if diagnostic:
                with self._lock:
                    self._asr_available = False
                    self._asr_recording = False
                    self._asr_phase = "UNAVAILABLE"
                    self._asr_last_error = diagnostic
            self._publish_status()

    def _apply_asr_status_locked(self, status):
        if not isinstance(status, dict):
            return
        if "ready" in status:
            self._asr_available = bool(status.get("ready"))
        if status.get("model"):
            model = _canonical_asr_model(status.get("model"))
            self._asr_model = model or str(status.get("model"))
        if status.get("device"):
            self._asr_device = str(status.get("device"))
        if "model_loaded" in status:
            self._asr_model_loaded = bool(status.get("model_loaded"))
        if "recording" in status and not self._smart_voice_enabled:
            self._asr_recording = bool(status.get("recording"))
        if status.get("state"):
            self._asr_phase = str(status.get("state")).upper()
        if "audio_duration_s" in status:
            self._asr_audio_duration_s = float(
                status.get("audio_duration_s") or 0.0)
        if "asr_latency_ms" in status:
            self._asr_latency_ms = float(status.get("asr_latency_ms") or 0.0)
        if "error" in status:
            self._asr_last_error = str(status.get("error") or "")

    def _set_asr_model(self, request):
        requested_model = _canonical_asr_model(request.model)
        if not self._token_valid(request.session_token):
            return SetAsrModelResponse(
                False, "无效的 NVIDIA UI 会话", self._asr_model)
        if not requested_model:
            return SetAsrModelResponse(
                False, "ASR 模型必须是 small、medium 或 large",
                self._asr_model)
        with self._lock:
            if not self._heartbeat_fresh():
                return SetAsrModelResponse(
                    False, "Qt 会话心跳未就绪", self._asr_model)
            if not self._asr_enabled:
                return SetAsrModelResponse(
                    False, "ASR 已在配置中禁用", self._asr_model)
            if self._asr_switching:
                return SetAsrModelResponse(
                    False, "ASR 模型切换正在进行", self._asr_model)
            busy = bool(
                self._asr_capture_id or self._asr_recording or
                self._smart_voice_enabled or self._smart_voice_listening or
                self._asr_phase in (
                    "STARTING", "RECORDING", "RECORDED", "STOPPING",
                    "TRANSCRIBING", "RECOGNIZING", "CANCELLING",
                    "SMART_STARTING", "SMART_LISTENING", "SMART_STOPPING"))
            if busy:
                return SetAsrModelResponse(
                    False, "请先停止录音/智能语音并等待当前识别完成",
                    self._asr_model)
            if (requested_model == self._asr_model and
                    self._asr_available and self._asr_client is not None):
                return SetAsrModelResponse(
                    True, "ASR 已使用 %s 模型" % requested_model,
                    self._asr_model)
            try:
                new_config = self._asr_config_for_model(requested_model)
            except Exception as exc:
                return SetAsrModelResponse(False, str(exc), self._asr_model)
            old_client = self._asr_client
            old_generation = self._asr_worker_generation
            old_state = {
                "model": self._asr_model,
                "model_path": self._asr_model_path,
                "config": dict(self._asr_config),
                "available": self._asr_available,
                "model_loaded": self._asr_model_loaded,
                "phase": self._asr_phase,
                "error": self._asr_last_error,
            }
            self._asr_epoch += 1
            self._asr_worker_generation += 1
            generation = self._asr_worker_generation
            self._asr_switching = True
            self._asr_phase = "SWITCHING"
            self._asr_last_error = ""
        self._publish_status()

        candidate = None
        try:
            candidate = self._new_asr_client(new_config, generation)
            status = candidate.start() or {}
            if not bool(status.get("ready")):
                raise RuntimeError(
                    str(status.get("error") or "目标 ASR worker 未通过就绪检查"))
            with self._lock:
                if (not self._asr_switching or
                        generation != self._asr_worker_generation):
                    raise RuntimeError("ASR 模型切换已被撤销")
                self._asr_client = candidate
                self._asr_config = dict(new_config)
                self._asr_model_path = new_config["model_path"]
                self._apply_asr_status_locked(status)
                self._asr_model = requested_model
                self._asr_model_loaded = bool(status.get("model_loaded", False))
                self._asr_phase = str(status.get("state", "READY")).upper()
                self._asr_last_error = ""
                self._asr_switching = False
            candidate = None
            if old_client is not None:
                old_client.close()
            message = "ASR 模型已切换为 %s" % requested_model
            self._emit("ASR_MODEL", state="READY", description=message)
            self._publish_status()
            return SetAsrModelResponse(True, message, requested_model)
        except Exception as exc:
            if candidate is not None:
                try:
                    candidate.close()
                except Exception:
                    pass
            with self._lock:
                if generation == self._asr_worker_generation:
                    self._asr_worker_generation = old_generation
                    self._asr_client = old_client
                    self._asr_model = old_state["model"]
                    self._asr_model_path = old_state["model_path"]
                    self._asr_config = old_state["config"]
                    self._asr_available = old_state["available"]
                    self._asr_model_loaded = old_state["model_loaded"]
                    self._asr_phase = old_state["phase"]
                    self._asr_last_error = old_state["error"]
                    self._asr_switching = False
                active_model = self._asr_model
            message = "切换 ASR 模型失败: %s" % exc
            self._emit("ASR_MODEL", state="ERROR", description=message)
            self._publish_status()
            return SetAsrModelResponse(False, message, active_model)

    def _asr_event(self, event):
        if isinstance(event, dict) and event.get("event") == "smart_transcript":
            self._handle_smart_transcript(event)
            return
        if (isinstance(event, dict) and
                event.get("event") in ("smart_stopped", "smart_error")):
            self._handle_smart_terminal(event)
            return
        if (isinstance(event, dict) and
                event.get("event") == "smart_queue_drop"):
            self._handle_smart_queue_drop(event)
            return
        if not isinstance(event, dict):
            payload = {}
        elif isinstance(event.get("status"), dict):
            payload = event["status"]
        elif isinstance(event.get("result"), dict):
            payload = event["result"]
        elif isinstance(event.get("params"), dict):
            payload = event["params"]
        else:
            payload = event
        smart_unavailable = None
        with self._lock:
            self._apply_asr_status_locked(payload)
            status_session = str(payload.get(
                "smart_session_id", payload.get("session_id", "")))
            if ("smart_listening" in payload and
                    self._smart_voice_enabled and
                    status_session == self._smart_voice_session_id):
                self._smart_voice_listening = bool(
                    payload.get("smart_listening"))
            unavailable = bool(
                ("ready" in payload and not bool(payload.get("ready"))) or
                str(payload.get("state", "")).upper() == "UNAVAILABLE")
            if unavailable and self._smart_voice_enabled:
                smart_unavailable = {
                    "session_id": self._smart_voice_session_id,
                    "dropped_pending": len(self._smart_voice_pending),
                    "error": str(payload.get("error") or
                                 "ASR worker 已离线"),
                }
                self._smart_voice_epoch += 1
                self._smart_voice_enabled = False
                self._smart_voice_listening = False
                self._smart_voice_session_id = ""
                self._smart_voice_pending.clear()
                self._smart_voice_seen.clear()
            if (isinstance(event, dict) and event.get("error") and
                    not payload.get("error")):
                self._asr_last_error = str(event.get("error"))
        if smart_unavailable is not None:
            self._emit(
                "SMART_VOICE", state="ERROR",
                description=(
                    "ASR worker 离线，智能监听和待处理队列已关闭: %s" %
                    smart_unavailable["error"]),
                result_json=json.dumps(
                    smart_unavailable, ensure_ascii=False))
        self._publish_status()

    def _handle_smart_queue_drop(self, event):
        payload = event.get("result") if isinstance(event, dict) else None
        if not isinstance(payload, dict):
            return
        session_id = str(payload.get("session_id", ""))
        try:
            utterance_index = int(payload.get("utterance_index", 0))
        except (TypeError, ValueError):
            utterance_index = 0
        with self._lock:
            if (not self._smart_voice_enabled or
                    not self._smart_voice_listening or
                    session_id != self._smart_voice_session_id or
                    not self._voice_authorized or
                    not self._heartbeat_fresh()):
                return
            result = {
                "session_id": session_id,
                "utterance_id": str(payload.get("utterance_id", "")),
                "utterance_index": utterance_index,
                "reason": str(payload.get(
                    "reason", "ASR worker 智能语音队列已满")),
            }
            self._emit(
                "SMART_VOICE", state="QUEUE_FULL",
                description="本地 ASR 队列已满，本句已丢弃且不会发送云端",
                result_json=json.dumps(result, ensure_ascii=False))
            self._publish_status()

    def _handle_smart_terminal(self, event):
        payload = event.get("result") if isinstance(event, dict) else None
        if not isinstance(payload, dict):
            return
        session_id = str(payload.get("session_id", ""))
        event_name = str(event.get("event", ""))
        state = str(payload.get(
            "state", "ERROR" if event_name == "smart_error" else
            "SMART_STOPPED")).upper()
        error = str(payload.get("error", ""))
        with self._lock:
            if (not session_id or
                    session_id != self._smart_voice_session_id or
                    not self._smart_voice_enabled):
                return
            dropped = len(self._smart_voice_pending)
            self._smart_voice_epoch += 1
            self._smart_voice_enabled = False
            self._smart_voice_listening = False
            self._smart_voice_session_id = ""
            self._smart_voice_pending.clear()
            self._smart_voice_seen.clear()
            self._asr_phase = state
            self._asr_last_error = error
        self._emit(
            "SMART_VOICE",
            state="ERROR" if event_name == "smart_error" else "STOPPED",
            description=(error or "ASR worker 已停止智能语音监听"),
            result_json=json.dumps({
                "session_id": session_id,
                "worker_state": state,
                "dropped_pending": dropped,
            }, ensure_ascii=False))
        self._publish_status()

    def _set_smart_voice(self, request):
        if not self._token_valid(request.session_token):
            return SetSmartVoiceResponse(
                False, "无效的 NVIDIA UI 会话", "")
        if request.enabled:
            return self._start_smart_voice()
        return self._stop_smart_voice("操作员关闭智能语音")

    def _start_smart_voice(self):
        with self._lock:
            if not self._heartbeat_fresh():
                return SetSmartVoiceResponse(
                    False, "Qt 会话心跳未就绪", "")
            if not self._voice_authorized:
                return SetSmartVoiceResponse(
                    False, "语音输入尚未授权", "")
            if not self._asr_available or self._asr_client is None:
                return SetSmartVoiceResponse(
                    False, self._asr_last_error or "ASR 不可用", "")
            if self._smart_voice_enabled:
                return SetSmartVoiceResponse(
                    True, "智能语音已经启用", self._smart_voice_session_id)
            if (self._asr_capture_id or self._asr_recording or
                    self._asr_switching or
                    self._asr_phase in (
                        "STARTING", "RECORDING", "RECORDED", "STOPPING",
                        "TRANSCRIBING", "RECOGNIZING", "CANCELLING",
                        "SWITCHING")):
                return SetSmartVoiceResponse(
                    False, "手动录音或识别正在进行，不能启用智能语音", "")
            self._smart_voice_epoch += 1
            epoch = self._smart_voice_epoch
            session_id = str(uuid.uuid4())
            client = self._asr_client
            self._smart_voice_enabled = True
            self._smart_voice_listening = False
            self._smart_voice_session_id = session_id
            self._smart_voice_utterance_count = 0
            self._smart_voice_pending.clear()
            self._smart_voice_seen.clear()
            self._asr_phase = "SMART_STARTING"
            self._asr_last_error = ""
        self._publish_status()
        try:
            status = client.request(
                "start_smart_listening", {"session_id": session_id},
                timeout=8.0) or {}
            returned_session = str(status.get(
                "smart_session_id", status.get("session_id", session_id)))
            listening = bool(status.get("smart_listening", False))
            with self._lock:
                if (epoch != self._smart_voice_epoch or
                        session_id != self._smart_voice_session_id or
                        not self._smart_voice_enabled or
                        not self._voice_authorized or
                        not self._heartbeat_fresh()):
                    raise RuntimeError("智能语音启动期间授权已撤销")
                if returned_session != session_id:
                    raise RuntimeError("ASR worker 智能语音会话不匹配")
                if not listening:
                    raise RuntimeError(
                        str(status.get("error") or
                            "ASR worker 未进入智能监听状态"))
                self._apply_asr_status_locked(status)
                self._smart_voice_listening = True
                self._asr_phase = "SMART_LISTENING"
                self._asr_last_error = ""
            self._emit(
                "SMART_VOICE", state="LISTENING",
                description="智能语音监听已开启",
                result_json=json.dumps(
                    {"session_id": session_id}, ensure_ascii=False))
            self._publish_status()
            return SetSmartVoiceResponse(
                True, "智能语音监听已开启", session_id)
        except Exception as exc:
            with self._lock:
                if (epoch == self._smart_voice_epoch and
                        session_id == self._smart_voice_session_id):
                    self._smart_voice_epoch += 1
                    stop_epoch = self._smart_voice_epoch
                    self._smart_voice_enabled = False
                    self._smart_voice_listening = False
                    self._smart_voice_session_id = ""
                    self._smart_voice_pending.clear()
                    self._smart_voice_seen.clear()
                    self._asr_phase = "ERROR"
                    self._asr_last_error = str(exc)
                else:
                    stop_epoch = self._smart_voice_epoch
            self._stop_smart_worker_async(
                (client, session_id, stop_epoch, True),
                "智能语音启动失败")
            self._emit("SMART_VOICE", state="ERROR", description=str(exc))
            self._publish_status()
            return SetSmartVoiceResponse(False, str(exc), "")

    def _revoke_smart_voice_locked(self, reason):
        client = self._asr_client
        session_id = self._smart_voice_session_id
        active = bool(
            self._smart_voice_enabled or self._smart_voice_listening or
            session_id or self._smart_voice_pending)
        self._smart_voice_epoch += 1
        epoch = self._smart_voice_epoch
        self._smart_voice_enabled = False
        self._smart_voice_listening = False
        self._smart_voice_session_id = ""
        self._smart_voice_pending.clear()
        self._smart_voice_seen.clear()
        if active:
            self._asr_phase = "SMART_STOPPING"
            self._asr_last_error = reason
        return client, session_id, epoch, active

    def _stop_smart_worker(self, stop_request, reason):
        client, session_id, epoch, active = stop_request
        if not active or client is None or not session_id:
            return True, ""
        try:
            status = client.request(
                "stop_smart_listening", {"session_id": session_id},
                timeout=8.0) or {}
            returned_session = str(status.get(
                "smart_session_id", status.get("session_id", session_id)))
            if returned_session and returned_session != session_id:
                raise RuntimeError("ASR worker 停止了错误的智能语音会话")
            with self._lock:
                if (epoch == self._smart_voice_epoch and
                        not self._smart_voice_enabled):
                    self._apply_asr_status_locked(status)
                    self._smart_voice_listening = False
                    self._asr_phase = "READY" if self._asr_available else "UNAVAILABLE"
                    self._asr_last_error = ""
            return True, ""
        except Exception as exc:
            # A failed graceful stop must still make a best-effort exact-worker
            # cancellation; it never re-enables smart voice or robot control.
            try:
                client.request(
                    "cancel", {"reason": reason, "unload_model": False},
                    timeout=5.0)
            except Exception:
                pass
            with self._lock:
                if (epoch == self._smart_voice_epoch and
                        not self._smart_voice_enabled):
                    self._smart_voice_listening = False
                    self._asr_phase = "ERROR"
                    self._asr_last_error = "停止智能语音失败: %s" % exc
            return False, str(exc)

    def _stop_smart_worker_async(self, stop_request, reason):
        if not stop_request[3]:
            return

        def stop_worker():
            self._stop_smart_worker(stop_request, reason)
            self._publish_status()

        threading.Thread(
            target=stop_worker, name="sweeper-smart-voice-stop",
            daemon=True).start()

    def _stop_smart_voice(self, reason):
        with self._lock:
            session_id = self._smart_voice_session_id
            stop_request = self._revoke_smart_voice_locked(reason)
        ok, error = self._stop_smart_worker(stop_request, reason)
        self._emit(
            "SMART_VOICE", state="STOPPED" if ok else "ERROR",
            description=("智能语音监听已关闭" if ok else
                         "智能语音监听关闭失败: %s" % error),
            result_json=json.dumps(
                {"session_id": session_id}, ensure_ascii=False))
        self._publish_status()
        return SetSmartVoiceResponse(
            ok, "智能语音监听已关闭" if ok else
            "智能语音监听关闭失败: %s" % error, session_id)

    def _handle_smart_transcript(self, event):
        payload = event.get("result") if isinstance(event, dict) else None
        if not isinstance(payload, dict):
            return
        session_id = str(payload.get("session_id", ""))
        utterance_id = str(payload.get("utterance_id", ""))
        try:
            utterance_index = int(payload.get(
                "utterance_index", payload.get("index", 0)))
        except (TypeError, ValueError):
            utterance_index = 0
        transcript = str(payload.get("transcript", "")).strip()
        with self._lock:
            if (not self._smart_voice_enabled or
                    not self._smart_voice_listening or
                    session_id != self._smart_voice_session_id or
                    not self._voice_authorized or
                    not self._heartbeat_fresh()):
                return
            epoch = self._smart_voice_epoch
            utterance_key = utterance_id or "index:%d" % utterance_index
            if utterance_key in self._smart_voice_seen:
                return
            if len(self._smart_voice_seen) >= 4096:
                self._smart_voice_seen.clear()
            self._smart_voice_seen.add(utterance_key)
            if not transcript or not bool(payload.get("accepted", True)):
                self._emit(
                    "SMART_VOICE", state="EMPTY",
                    description=str(payload.get("error") or
                                    "智能语音未识别到可用文本"))
                return
            if len(transcript) > 4000:
                self._emit(
                    "SMART_VOICE", state="REJECTED",
                    description="智能语音文本超过 4000 字符，已拒绝")
                return
            self._smart_voice_utterance_count += 1
            metrics = {
                "session_id": session_id,
                "utterance_id": utterance_id,
                "utterance_index": utterance_index,
                "audio_duration_s": float(
                    payload.get("audio_duration_s") or 0.0),
                "rms": float(payload.get("rms") or 0.0),
                "asr_latency_ms": float(
                    payload.get("asr_latency_ms") or 0.0),
            }
            self._asr_audio_duration_s = metrics["audio_duration_s"]
            self._asr_latency_ms = metrics["asr_latency_ms"]
            self._asr_model_loaded = bool(
                payload.get("model_loaded", self._asr_model_loaded))
            self._emit(
                "TRANSCRIPT", state="SMART_ASR", description=transcript,
                result_json=json.dumps(metrics, ensure_ascii=False),
                duration_ms=self._asr_latency_ms)
            if not self._parse_authorized:
                self._emit(
                    "SMART_VOICE", state="LOCAL_ONLY",
                    description="识别文本仅本地显示；AI 语义解析未授权")
                self._publish_status()
                return
            if len(self._smart_voice_pending) >= SMART_VOICE_QUEUE_LIMIT:
                self._emit(
                    "SMART_VOICE", state="QUEUE_FULL",
                    description="智能语音队列已满，本句未发送云端")
                self._publish_status()
                return
            self._smart_voice_pending.append({
                "epoch": epoch,
                "session_id": session_id,
                "utterance_id": utterance_id,
                "utterance_index": utterance_index,
                "transcript": transcript,
                "enqueued_at": time.monotonic(),
            })
        self._publish_status()
        self._dispatch_next_smart_voice()

    def _dispatch_next_smart_voice(self):
        with self._lock:
            if self._task_active:
                return False
            while self._smart_voice_pending:
                item = self._smart_voice_pending.popleft()
                age_s = max(
                    0.0, time.monotonic() - item.get("enqueued_at", 0.0))
                if age_s > SMART_VOICE_QUEUE_TTL_S:
                    self._emit(
                        "SMART_VOICE", state="DROPPED",
                        description=(
                            "智能语音等待 %.1f 秒，已超过 %.0f 秒安全时限，"
                            "不会执行陈旧口令" %
                            (age_s, SMART_VOICE_QUEUE_TTL_S)),
                        result_json=json.dumps({
                            "session_id": item["session_id"],
                            "utterance_id": item["utterance_id"],
                            "utterance_index": item["utterance_index"],
                            "age_s": age_s,
                        }, ensure_ascii=False))
                    continue
                if (not self._smart_voice_enabled or
                        not self._smart_voice_listening or
                        item["epoch"] != self._smart_voice_epoch or
                        item["session_id"] != self._smart_voice_session_id or
                        not self._voice_authorized or
                        not self._parse_authorized or
                        not self._heartbeat_fresh()):
                    continue
                accepted, request_id, message = self._begin_request(
                    item["transcript"], "ASR", emit_transcript=False)
                if accepted:
                    self._emit(
                        "SMART_VOICE", state="DISPATCHED",
                        description="智能语音已进入串行 AI 处理",
                        result_json=json.dumps({
                            "session_id": item["session_id"],
                            "utterance_id": item["utterance_id"],
                            "utterance_index": item["utterance_index"],
                            "request_id": request_id,
                        }, ensure_ascii=False))
                    self._publish_status()
                    return True
                self._emit(
                    "SMART_VOICE", state="DROPPED",
                    description="智能语音未进入 AI 处理: %s" % message)
            self._publish_status()
            return False

    def _cancel_asr_async(self, epoch, reason):
        with self._lock:
            client = self._asr_client
        if client is None:
            return

        def cancel_worker():
            try:
                status = client.request(
                    "cancel", {"reason": reason, "unload_model": True},
                    timeout=5.0) or {}
                with self._lock:
                    if epoch == self._asr_epoch:
                        self._apply_asr_status_locked(status)
                        self._asr_recording = False
            except Exception as exc:
                with self._lock:
                    if epoch == self._asr_epoch:
                        self._asr_recording = False
                        self._asr_phase = "ERROR"
                        self._asr_last_error = "停止 ASR 失败: %s" % exc
            self._publish_status()

        threading.Thread(
            target=cancel_worker, name="sweeper-asr-cancel", daemon=True).start()

    def _revoke_asr_locked(self, reason):
        self._asr_epoch += 1
        epoch = self._asr_epoch
        active = bool(self._asr_recording or self._asr_capture_id or
                      self._asr_model_loaded or
                      self._asr_phase in (
                          "STARTING", "RECORDING", "STOPPING",
                          "TRANSCRIBING", "RECOGNIZING"))
        self._asr_capture_id = ""
        self._asr_recording = False
        if active:
            self._asr_phase = "CANCELLING"
            self._asr_last_error = reason
        return epoch, active

    def _set_asr_recording(self, request):
        if not self._token_valid(request.session_token):
            return SetAsrRecordingResponse(
                False, "无效的 NVIDIA UI 会话", "")
        if request.recording:
            return self._start_asr_recording()
        return self._stop_asr_recording()

    def _start_asr_recording(self):
        with self._lock:
            if not self._heartbeat_fresh():
                return SetAsrRecordingResponse(
                    False, "Qt 会话心跳未就绪", "")
            if not self._voice_authorized:
                return SetAsrRecordingResponse(
                    False, "语音输入尚未授权", "")
            if not self._asr_available or self._asr_client is None:
                return SetAsrRecordingResponse(
                    False, self._asr_last_error or "ASR 不可用", "")
            if self._smart_voice_enabled or self._smart_voice_listening:
                return SetAsrRecordingResponse(
                    False, "智能语音监听已启用，不能开始手动录音", "")
            if (self._asr_switching or self._asr_capture_id or
                    self._asr_recording or self._asr_phase in (
                    "STARTING", "STOPPING", "TRANSCRIBING", "RECOGNIZING",
                    "CANCELLING", "SWITCHING")):
                return SetAsrRecordingResponse(
                    False, "ASR 当前正忙", self._asr_capture_id)
            self._asr_epoch += 1
            epoch = self._asr_epoch
            capture_id = str(uuid.uuid4())
            self._asr_capture_id = capture_id
            self._asr_phase = "STARTING"
            self._asr_audio_duration_s = 0.0
            self._asr_latency_ms = 0.0
            self._asr_last_error = ""
            client = self._asr_client
        self._publish_status()
        try:
            status = client.request(
                "start_recording", {"capture_id": capture_id}, timeout=8.0) or {}
            with self._lock:
                if (epoch != self._asr_epoch or
                        not self._voice_authorized or
                        not self._heartbeat_fresh()):
                    raise RuntimeError("录音启动期间授权已撤销")
                self._apply_asr_status_locked(status)
                if not self._asr_recording:
                    raise RuntimeError(
                        self._asr_last_error or "录音设备未进入采集状态")
                self._asr_phase = "RECORDING"
            self._emit("ASR", state="RECORDING",
                       description="本地麦克风录音已开始")
            self._publish_status()
            return SetAsrRecordingResponse(True, "录音已开始", capture_id)
        except Exception as exc:
            with self._lock:
                if epoch == self._asr_epoch:
                    self._asr_capture_id = ""
                    self._asr_recording = False
                    self._asr_phase = "ERROR"
                    self._asr_last_error = str(exc)
            self._cancel_asr_async(epoch, "录音启动失败")
            self._emit("ASR", state="ERROR", description=str(exc))
            self._publish_status()
            return SetAsrRecordingResponse(False, str(exc), "")

    def _stop_asr_recording(self):
        with self._lock:
            if not self._heartbeat_fresh():
                return SetAsrRecordingResponse(
                    False, "Qt 会话心跳未就绪", "")
            if not self._voice_authorized:
                return SetAsrRecordingResponse(
                    False, "语音输入尚未授权", "")
            if self._asr_phase not in ("RECORDING", "RECORDED") or \
                    not self._asr_capture_id:
                return SetAsrRecordingResponse(False, "当前没有录音", "")
            if self._asr_client is None:
                return SetAsrRecordingResponse(False, "ASR worker 已离线", "")
            epoch = self._asr_epoch
            capture_id = self._asr_capture_id
            client = self._asr_client
            self._asr_phase = "STOPPING"
        threading.Thread(
            target=self._finish_asr_recording,
            args=(client, epoch, capture_id),
            name="sweeper-asr-transcribe", daemon=True).start()
        self._publish_status()
        return SetAsrRecordingResponse(
            True, "录音已停止，正在使用 %s 本地识别" % self._asr_model,
            capture_id)

    def _finish_asr_recording(self, client, epoch, capture_id):
        try:
            result = client.request(
                "stop_recording", {"capture_id": capture_id},
                timeout=600.0) or {}
            transcript = str(result.get("transcript", "")).strip()
            returned_capture = str(result.get("capture_id", capture_id))
            with self._lock:
                still_authorized = bool(
                    epoch == self._asr_epoch and
                    capture_id == self._asr_capture_id and
                    returned_capture == capture_id and
                    self._voice_authorized and self._heartbeat_fresh())
                if not still_authorized:
                    return
                self._apply_asr_status_locked(result)
                self._asr_capture_id = ""
                self._asr_recording = False
                self._asr_phase = "READY"
                parse_now = bool(
                    self._parse_authorized and self._heartbeat_fresh())
                metrics = {
                    "capture_id": capture_id,
                    "audio_duration_s": self._asr_audio_duration_s,
                    "rms": float(result.get("rms", 0.0)),
                    "asr_latency_ms": self._asr_latency_ms,
                    "model": self._asr_model,
                    "device": self._asr_device,
                }
                recognized = bool(result.get("accepted", bool(transcript)))
                # Keep the epoch lock until both local events are published so
                # a simultaneous revoke wins cleanly instead of displaying a
                # result after authorization has already been removed.
                self._emit(
                    "ASR", state="SUCCEEDED" if recognized else "REJECTED",
                    description=("本地 %s 识别完成" % self._asr_model
                                 if recognized else
                                 (self._asr_last_error or "本地音频未通过识别条件")),
                    result_json=json.dumps(metrics, ensure_ascii=False),
                    duration_ms=self._asr_latency_ms)
                self._emit("TRANSCRIPT", state="ASR", description=transcript)
                self._publish_status()
            if not transcript:
                self._emit("ASR", state="EMPTY",
                           description="未识别到可用文本，不会发送云端")
                return
            if not parse_now:
                self._emit(
                    "ASR", state="LOCAL_ONLY",
                    description="识别文本仅本地显示；AI 语义解析未授权")
                return
            accepted, _request_id, message = self._begin_request(
                transcript, "ASR", emit_transcript=False)
            if not accepted:
                self._emit(
                    "ASR", state="LOCAL_ONLY",
                    description="识别文本未发送云端: %s" % message)
        except Exception as exc:
            with self._lock:
                if epoch != self._asr_epoch:
                    return
                self._asr_capture_id = ""
                self._asr_recording = False
                self._asr_phase = "ERROR"
                self._asr_last_error = str(exc)
            self._emit("ASR", state="ERROR", description=str(exc))
            self._publish_status()

    def _token_valid(self, token):
        return bool(token and hmac.compare_digest(token, self._session_token))

    def _heartbeat_fresh(self):
        return time.monotonic() - self._last_heartbeat <= 3.0

    def _execution_authorized(self):
        with self._lock:
            return bool(self._parse_authorized and self._control_authorized and
                        self._heartbeat_fresh())

    def _cloud_authorized(self):
        with self._lock:
            return bool(self._parse_authorized and self._heartbeat_fresh())

    def _set_authorization(self, request):
        if not self._token_valid(request.session_token):
            return self._auth_response(False, "无效的 NVIDIA UI 会话")
        gate = request.gate.strip().lower()
        should_cancel = False
        asr_cancel = None
        smart_stop = None
        cleared_pending = 0
        cleared_reason = ""
        with self._lock:
            if gate == "heartbeat":
                self._last_heartbeat = time.monotonic()
                return self._auth_response(True, "heartbeat")
            if gate == "voice":
                if request.enabled and not self._heartbeat_fresh():
                    return self._auth_response(False, "Qt 会话心跳未就绪")
                self._voice_authorized = bool(request.enabled)
                if request.enabled:
                    message = ("语音输入已授权；麦克风仍保持关闭，需另行点击"
                               "开始录音或启用智能语音")
                else:
                    smart_stop = self._revoke_smart_voice_locked(
                        "语音输入授权已关闭")
                    asr_cancel = self._revoke_asr_locked(
                        "语音输入授权已关闭")
                    message = (
                        "语音输入授权已关闭；智能监听、录音、队列和迟到识别结果"
                        "已撤销")
            elif gate == "parse":
                if request.enabled and not self._cloud_configured:
                    return self._auth_response(False,
                                               self._last_error or "云端配置不可用")
                if request.enabled and not self._heartbeat_fresh():
                    return self._auth_response(False, "Qt 会话心跳未就绪")
                self._parse_authorized = bool(request.enabled)
                if not request.enabled:
                    self._control_authorized = False
                    should_cancel = self._task_active
                    cleared_pending = len(self._smart_voice_pending)
                    self._smart_voice_pending.clear()
                    cleared_reason = "AI 解析授权关闭"
                message = "AI 语义解析已%s" % ("授权" if request.enabled else "关闭")
            elif gate == "control":
                if request.enabled and not self._parse_authorized:
                    return self._auth_response(False, "请先授权 AI 语义解析")
                if request.enabled and not self._heartbeat_fresh():
                    return self._auth_response(False, "Qt 会话心跳未就绪")
                control_changed = (
                    bool(request.enabled) != self._control_authorized)
                self._control_authorized = bool(request.enabled)
                if control_changed:
                    cleared_pending = len(self._smart_voice_pending)
                    self._smart_voice_pending.clear()
                    cleared_reason = "AI 控制授权状态变化"
                should_cancel = not request.enabled and self._task_active
                message = "AI 控制已%s" % ("授权" if request.enabled else "关闭")
            else:
                return self._auth_response(False, "未知授权门: %s" % gate)
        if should_cancel:
            self._runner.cancel()
        if smart_stop is not None and smart_stop[3]:
            stopped, stop_error = self._stop_smart_worker(
                smart_stop, "语音输入授权已关闭")
            if not stopped:
                self._emit(
                    "SMART_VOICE", state="ERROR",
                    description="语音撤权后停止智能监听失败: %s" % stop_error)
        if asr_cancel is not None and asr_cancel[1]:
            self._cancel_asr_async(asr_cancel[0], message)
        if cleared_pending:
            self._emit(
                "SMART_VOICE", state="DROPPED",
                description="%s，已清除 %d 条待处理语音" %
                            (cleared_reason, cleared_pending))
        self._emit("AUTH", state=gate.upper(), description=message)
        self._publish_status()
        return self._auth_response(True, message)

    def _auth_response(self, accepted, message):
        with self._lock:
            return SetAiAuthorizationResponse(
                accepted=accepted,
                message=message,
                voice_authorized=self._voice_authorized,
                parse_authorized=self._parse_authorized,
                control_authorized=self._control_authorized,
            )

    def _submit_text(self, request):
        if not self._token_valid(request.session_token):
            return SubmitAiTextResponse(
                accepted=False, request_id="", message="无效的 NVIDIA UI 会话")
        source = request.source.strip().upper()
        text = request.text.strip()
        if source not in ("MANUAL", "ASR"):
            return SubmitAiTextResponse(False, "", "source 必须是 MANUAL 或 ASR")
        if not text:
            return SubmitAiTextResponse(False, "", "输入文本为空")
        if len(text) > 4000:
            return SubmitAiTextResponse(False, "", "输入文本超过 4000 字符")
        accepted, request_id, message = self._begin_request(text, source)
        return SubmitAiTextResponse(accepted, request_id, message)

    def _begin_request(self, text, source, emit_transcript=True):
        with self._lock:
            if not self._heartbeat_fresh():
                return False, "", "Qt 会话心跳未就绪"
            if not self._parse_authorized:
                return False, "", "AI 语义解析尚未授权"
            if source == "ASR" and not self._voice_authorized:
                return False, "", "语音输入尚未授权"
            if self._task_active:
                return False, "", "已有 AI 请求正在处理"
            request_id = str(uuid.uuid4())
            self._request_id = request_id
            self._task_active = True
            self._phase = "PARSING"
            self._current_step = 0
            self._total_steps = 0
            self._last_error = ""
            self._final_text = ""
            execute = self._control_authorized and self._heartbeat_fresh()
        if emit_transcript:
            self._emit("TRANSCRIPT", state=source, description=text)
        self._publish_status()
        threading.Thread(
            target=self._run_request,
            args=(request_id, text, execute),
            name="sweeper-ai-request", daemon=True,
        ).start()
        return True, request_id, "请求已接收"

    def _run_request(self, request_id, text, execute):
        try:
            if execute and not self._execution_authorized():
                execute = False
            result = self._runner.run(text, execute=execute)
            with self._lock:
                if request_id != self._request_id:
                    return
                self._phase = result["state"]
                self._last_total_latency_ms = float(result.get("total_ms", 0.0))
                self._final_text = result.get("answer", "")
                self._total_steps = len(result.get("plan", {}).get("steps", []))
                self._current_step = len(result.get("results", []))
                if result["state"] == "FAILED" and result.get("results"):
                    self._last_error = result["results"][-1].get("detail", "")
        except Exception as exc:
            rospy.logerr("AI request failed: %s", exc)
            with self._lock:
                self._phase = "FAILED"
                self._last_error = str(exc)
                self._final_text = "AI 请求处理失败：%s" % exc
        finally:
            with self._lock:
                self._task_active = False
            self._emit("FINAL", state=self._phase,
                       description=self._final_text,
                       duration_ms=self._last_total_latency_ms)
            self._publish_status()
            self._dispatch_next_smart_voice()

    def _cancel_task(self, request):
        if not self._token_valid(request.session_token):
            return CancelAiTaskResponse(False, "无效的 NVIDIA UI 会话")
        with self._lock:
            if (request.request_id and self._request_id and
                    request.request_id != self._request_id):
                return CancelAiTaskResponse(False, "request_id 与当前任务不一致")
            task_active = self._task_active
            smart_stop = self._revoke_smart_voice_locked(
                "操作员取消 AI 任务")
            if not task_active and not smart_stop[3]:
                return CancelAiTaskResponse(True, "当前没有活动 AI 任务")
            if task_active:
                self._phase = "CANCELLING"
        if task_active:
            self._runner.cancel()
        smart_stopped, smart_error = self._stop_smart_worker(
            smart_stop, "操作员取消 AI 任务")
        description = "操作员请求停止 AI 任务"
        if smart_stop[3]:
            description += "；智能监听和待处理队列已关闭"
        if not smart_stopped:
            description += "；但停止智能监听失败: %s" % smart_error
        self._emit("CANCEL", state="REQUESTED", description=description)
        self._publish_status()
        if not smart_stopped:
            return CancelAiTaskResponse(False, description)
        return CancelAiTaskResponse(True, description)

    def _agent_event(self, event):
        kind = event.get("kind", "EVENT")
        if kind == "CLOUD":
            try:
                payload = json.loads(event.get("result_json") or "{}")
            except ValueError:
                payload = {}
            with self._lock:
                self._last_cloud_rtt_ms = float(event.get("duration_ms", 0.0))
                self._last_http_status = int(payload.get("http_status", 0))
        elif kind == "PLAN":
            try:
                payload = json.loads(event.get("result_json") or "{}")
                steps = payload.get("steps", [])
            except (TypeError, ValueError):
                steps = []
            with self._lock:
                self._phase = "PREVIEW" if not self._control_authorized else "EXECUTING"
                self._total_steps = len(steps)
        elif kind == "STEP":
            with self._lock:
                self._phase = "EXECUTING"
                self._current_step = max(0, int(event.get("step_index", 0)) + 1)
        self._emit(kind, **{key: value for key, value in event.items()
                            if key != "kind"})
        self._publish_status()

    def _emit(self, kind, **fields):
        with self._lock:
            self._event_sequence += 1
            request_id = self._request_id
            sequence = self._event_sequence
        message = AiEvent()
        message.header.stamp = rospy.Time.now()
        message.request_id = request_id
        message.sequence = sequence
        message.kind = str(kind)
        message.step_index = int(fields.get("step_index", -1))
        message.tool = str(fields.get("tool", ""))
        message.state = str(fields.get("state", ""))
        message.description = str(fields.get("description", ""))
        message.arguments_json = str(fields.get("arguments_json", ""))
        message.result_json = str(fields.get("result_json", ""))
        message.duration_ms = float(fields.get("duration_ms", 0.0))
        self._event_pub.publish(message)

    def _publish_status(self):
        with self._lock:
            message = AiControlStatus()
            message.header.stamp = rospy.Time.now()
            message.voice_authorized = self._voice_authorized
            message.parse_authorized = self._parse_authorized
            message.control_authorized = self._control_authorized
            message.ui_session_alive = self._heartbeat_fresh()
            message.cloud_configured = self._cloud_configured
            message.task_active = self._task_active
            message.request_id = self._request_id
            message.phase = self._phase
            message.current_step = self._current_step
            message.total_steps = self._total_steps
            message.model = self._model
            message.backend = self._backend
            message.last_cloud_rtt_ms = self._last_cloud_rtt_ms
            message.last_total_latency_ms = self._last_total_latency_ms
            message.last_http_status = self._last_http_status
            message.last_error = self._last_error
            message.final_text = self._final_text
            message.asr_available = self._asr_available
            message.asr_model = self._asr_model
            message.asr_device = self._asr_device
            message.asr_model_loaded = self._asr_model_loaded
            message.asr_recording = self._asr_recording
            message.asr_phase = self._asr_phase
            message.asr_audio_duration_s = self._asr_audio_duration_s
            message.asr_latency_ms = self._asr_latency_ms
            message.asr_last_error = self._asr_last_error
            message.smart_voice_enabled = self._smart_voice_enabled
            message.smart_voice_listening = self._smart_voice_listening
            message.smart_voice_utterance_count = (
                self._smart_voice_utterance_count)
            message.smart_voice_pending_count = len(self._smart_voice_pending)
        self._status_pub.publish(message)

    def _watchdog(self, _event):
        should_cancel = False
        revoked = False
        asr_cancel = None
        smart_stop = None
        with self._lock:
            if ((self._voice_authorized or self._parse_authorized or
                 self._control_authorized) and
                    not self._heartbeat_fresh()):
                self._voice_authorized = False
                self._parse_authorized = False
                self._control_authorized = False
                should_cancel = self._task_active
                self._last_error = "Qt 会话心跳超时，授权已全部撤销"
                smart_stop = self._revoke_smart_voice_locked(self._last_error)
                asr_cancel = self._revoke_asr_locked(self._last_error)
                revoked = True
        if should_cancel:
            self._runner.cancel()
        if asr_cancel is not None and asr_cancel[1]:
            self._cancel_asr_async(asr_cancel[0], self._last_error)
        if smart_stop is not None and smart_stop[3]:
            self._stop_smart_worker_async(smart_stop, self._last_error)
        if revoked:
            self._emit("AUTH", state="REVOKED", description=self._last_error)
        self._publish_status()

    def _shutdown(self):
        # Revoke every capability and close the microphone before waiting for
        # MCP subprocess teardown.  mcp.close() may wait for a child process;
        # it must never extend the lifetime of an authorised recorder.
        try:
            self._runner.cancel()
        except Exception:
            pass
        with self._lock:
            self._voice_authorized = False
            self._parse_authorized = False
            self._control_authorized = False
            smart_stop = self._revoke_smart_voice_locked(
                "AI 节点正在关闭")
            self._revoke_asr_locked("AI 节点正在关闭")
            self._asr_worker_generation += 1
            self._asr_switching = False
            client = self._asr_client
            self._asr_client = None
        self._stop_smart_worker(smart_stop, "AI 节点正在关闭")
        if client is not None:
            try:
                client.request(
                    "cancel",
                    {"reason": "parent shutdown", "unload_model": True},
                    timeout=2.0)
            except Exception:
                pass
            try:
                client.close()
            except Exception:
                pass
        try:
            self._runner.close()
        except Exception:
            pass


def main():
    rospy.init_node("sweeper_ai", anonymous=False, disable_signals=False)
    SweeperAiNode()
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
