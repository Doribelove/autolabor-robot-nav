#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Local, capability-free selectable Whisper ASR worker.

Transport is one JSON object per stdin/stdout line.  The worker has no ROS
imports, cloud client, MCP client, or robot-control interface.  Audio is
captured by one exact ``arecord`` child launched with an argv list (never a
shell), and every exit path closes that child.
"""

from array import array
import argparse
from collections import deque
import hashlib
import importlib.util
import json
import logging
import math
import os
import re
import signal
import stat
import subprocess
import sys
import tempfile
import threading
import time


DEFAULT_MODEL_NAME = "medium"
MODEL_SPECS = {
    "small": {
        "n_mels": 80,
        "n_audio_state": 768,
        "n_audio_layer": 12,
        "n_text_state": 768,
        "n_text_layer": 12,
    },
    "medium": {
        "n_mels": 80,
        "n_audio_state": 1024,
        "n_audio_layer": 24,
        "n_text_state": 1024,
        "n_text_layer": 24,
    },
    # The operator-facing ``large`` option intentionally uses OpenAI's
    # multilingual large-v3 checkpoint.
    "large": {
        "n_mels": 128,
        "n_audio_state": 1280,
        "n_audio_layer": 32,
        "n_text_state": 1280,
        "n_text_layer": 32,
    },
}
DEVICE_NAME = "cuda"
SAMPLE_RATE = 16000
CHANNELS = 1
SAMPLE_BYTES = 2
MAX_AUDIO_SECONDS_HARD_LIMIT = 30.0
HEARTBEAT_TIMEOUT_SECONDS = 3.0
WATCHDOG_INTERVAL_SECONDS = 0.1

logger = logging.getLogger("whisper_asr_worker")


_ALSA_CAPTURE_LINE = re.compile(
    r"^\s*card\s+(\d+):\s*([^\s\[]+)\s*\[(.*?)\],\s*"
    r"device\s+(\d+):\s*(.*?)\s*\[",
    re.IGNORECASE,
)


def _parse_alsa_capture_devices(listing):
    """Parse ``arecord -l`` without opening any capture device."""
    devices = []
    for line in str(listing or "").splitlines():
        match = _ALSA_CAPTURE_LINE.match(line)
        if match is None:
            continue
        devices.append({
            "card_index": int(match.group(1)),
            "card_id": match.group(2).strip(),
            "card_name": match.group(3).strip(),
            "device_index": int(match.group(4)),
            "device_name": match.group(5).strip(),
        })
    return devices


def _default_capture_device_lister(arecord_path):
    """Enumerate ALSA capture endpoints; this never starts recording."""
    try:
        completed = subprocess.run(
            [arecord_path, "-l"],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
            timeout=3.0,
            check=False,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise RuntimeError("unable to enumerate ALSA capture devices: %s" % exc)
    if completed.returncode != 0:
        detail = (completed.stderr or "").strip()
        raise RuntimeError(
            "unable to enumerate ALSA capture devices%s" %
            ((": %s" % detail) if detail else ""))
    return _parse_alsa_capture_devices(completed.stdout)


def _automatic_capture_device(candidates):
    """Choose a real capture endpoint, preferring USB and stable ALSA IDs."""
    usable = []
    virtual_tokens = (
        "auto_null", "monitor", "null", "ape", "tegra-dlink", "adsp-fe",
    )
    for candidate in candidates or ():
        try:
            card_index = int(candidate["card_index"])
            device_index = int(candidate["device_index"])
            card_id = str(candidate.get("card_id", "")).strip()
            description = " ".join((
                card_id,
                str(candidate.get("card_name", "")),
                str(candidate.get("device_name", "")),
            )).lower()
        except (KeyError, TypeError, ValueError):
            continue
        if card_index < 0 or device_index < 0:
            continue
        if any(token in description for token in virtual_tokens):
            continue
        stable_id = (
            card_id if re.match(r"^[A-Za-z0-9_-]{1,64}$", card_id)
            else str(card_index)
        )
        # USB capture devices are normally external microphones. Other real
        # capture cards remain valid fallbacks for built-in/analogue inputs.
        priority = 0 if "usb" in description else 1
        usable.append((priority, card_index, device_index, stable_id))
    if not usable:
        raise ValueError(
            "no usable physical ALSA capture device was found; "
            "connect a microphone or configure input_device explicitly")
    _, _, device_index, stable_id = sorted(usable)[0]
    return "plughw:CARD=%s,DEV=%d" % (stable_id, device_index)


class WorkerRequestError(Exception):
    """Error returned through the JSON-lines response envelope."""

    def __init__(self, code, message, data=None):
        super().__init__(message)
        self.code = code
        self.message = message
        self.data = data


def _default_cuda_probe():
    try:
        import torch
    except Exception as exc:
        raise RuntimeError("CUDA PyTorch import failed: %s" % exc)
    if not torch.cuda.is_available() or getattr(torch.version, "cuda", None) is None:
        raise RuntimeError(
            "CUDA is unavailable in the ASR Python environment; CPU fallback is forbidden")
    return True


def _default_model_loader(checkpoint_path):
    # ``checkpoint_path`` is an absolute existing file.  OpenAI Whisper's
    # load_model path branch loads it directly and cannot invoke its model-name
    # download branch.
    _default_cuda_probe()
    try:
        import whisper
    except Exception as exc:
        raise RuntimeError("OpenAI Whisper import failed: %s" % exc)
    return whisper.load_model(checkpoint_path, device=DEVICE_NAME)


def _default_text_converter():
    try:
        from opencc import OpenCC
    except Exception as exc:
        raise RuntimeError("OpenCC t2s import failed: %s" % exc)
    converter = OpenCC("t2s")
    return converter.convert


def _validate_model(model, model_name):
    dims = getattr(model, "dims", None)
    expected = MODEL_SPECS.get(model_name)
    if expected is None:
        raise RuntimeError("unsupported Whisper model: %s" % model_name)
    if dims is None:
        raise RuntimeError("loaded checkpoint has no Whisper model dimensions")
    mismatches = []
    for name, value in expected.items():
        actual = getattr(dims, name, None)
        if actual != value:
            mismatches.append("%s=%r (expected %r)" % (name, actual, value))
    if mismatches:
        raise RuntimeError(
            "local checkpoint does not match OpenAI Whisper %s: %s" %
            (model_name, ", ".join(mismatches)))


def _numeric_config(config, name, default, minimum, maximum):
    value = config.get(name, default)
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ValueError("%s must be numeric" % name)
    if not math.isfinite(number) or number < minimum or number > maximum:
        raise ValueError(
            "%s must be between %s and %s" % (name, minimum, maximum))
    return number


class WhisperAsrWorker(object):
    """Own one recorder and one lazily loaded local Whisper model."""

    def __init__(self, config, cuda_probe=None, model_loader=None,
                 popen_factory=None, event_callback=None, text_converter=None,
                 capture_device_lister=None):
        self._config = dict(config or {})
        self._cuda_probe = cuda_probe or _default_cuda_probe
        self._using_default_model_loader = model_loader is None
        self._model_loader = model_loader or _default_model_loader
        self._popen_factory = popen_factory or subprocess.Popen
        self._capture_device_lister = (
            capture_device_lister or _default_capture_device_lister)
        self._injected_text_converter = text_converter
        self._text_converter = None
        self._event_callback = event_callback
        self._lock = threading.RLock()
        self._operation_lock = threading.Lock()
        self._model_lock = threading.Lock()
        self._shutdown_event = threading.Event()
        self._record_proc = None
        self._capture_dir = ""
        self._raw_path = ""
        self._capture_id = ""
        self._capture_generation = 0
        self._record_started_at = 0.0
        self._last_heartbeat = time.monotonic()
        self._recording = False
        self._inference_active = False
        self._smart_inference_active = False
        self._model = None
        self._unload_requested = False
        self._ready = False
        self._state = "INITIALIZING"
        self._error = ""
        self._audio_duration_s = 0.0
        self._last_rms = 0.0
        self._asr_latency_ms = 0.0
        self._model_name = DEFAULT_MODEL_NAME
        self._checkpoint_path = ""
        self._checkpoint_sha256 = ""
        self._checkpoint_identity = None
        self._arecord_path = ""
        self._input_device = ""
        self._max_audio_seconds = MAX_AUDIO_SECONDS_HARD_LIMIT
        self._min_audio_seconds = 0.3
        self._min_rms = 0.003
        self._language = "zh"
        self._initial_prompt = "简体中文"
        self._smart_listening = False
        self._smart_session_id = ""
        self._smart_proc = None
        self._smart_capture_thread = None
        self._smart_generation = 0
        self._smart_queue = deque()
        self._smart_condition = threading.Condition(self._lock)
        self._smart_utterance_count = 0
        self._smart_dropped_utterances = 0
        self._smart_terminal_state = "SMART_STOPPED"
        self._smart_chunk_ms = 100.0
        self._smart_speech_start_ms = 200.0
        self._smart_silence_ms = 800.0
        self._smart_pre_roll_ms = 300.0
        self._smart_min_utterance_s = 0.4
        self._smart_max_utterance_s = 15.0
        self._smart_queue_limit = 4
        self._smart_vad_rms = self._min_rms
        self._configure()
        self._smart_inference_thread = threading.Thread(
            target=self._smart_inference_loop,
            name="asr-smart-inference",
            daemon=True,
        )
        self._smart_inference_thread.start()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop,
            name="asr-recording-watchdog",
            daemon=True,
        )
        self._watchdog_thread.start()

    def set_event_callback(self, callback):
        self._event_callback = callback

    def _configure(self):
        try:
            model_name = str(
                self._config.get("model", DEFAULT_MODEL_NAME)).strip().lower()
            device_name = str(self._config.get("device", DEVICE_NAME))
            if model_name not in MODEL_SPECS:
                raise ValueError("ASR model must be one of small, medium, large")
            if device_name != DEVICE_NAME:
                raise ValueError("ASR device must be exactly cuda; CPU fallback is forbidden")

            sample_rate = self._config.get("sample_rate", SAMPLE_RATE)
            try:
                sample_rate = int(sample_rate)
            except (TypeError, ValueError):
                raise ValueError("sample_rate must be exactly 16000")
            if sample_rate != SAMPLE_RATE:
                raise ValueError("sample_rate must be exactly 16000")
            language = str(self._config.get("language", "zh")).strip()
            if language != "zh":
                raise ValueError("ASR language must be exactly zh")
            initial_prompt = str(
                self._config.get("initial_prompt", "简体中文")).strip()
            if "\x00" in initial_prompt or len(initial_prompt) > 200:
                raise ValueError("initial_prompt is invalid")

            checkpoint_path = os.path.abspath(
                str(self._config.get("model_path", "")))
            if not checkpoint_path or checkpoint_path == os.path.abspath(""):
                raise ValueError("model_path is required")
            checkpoint_info = os.lstat(checkpoint_path)
            if stat.S_ISLNK(checkpoint_info.st_mode):
                raise ValueError("Whisper checkpoint must not be a symlink")
            if not stat.S_ISREG(checkpoint_info.st_mode) or checkpoint_info.st_size <= 0:
                raise ValueError("Whisper checkpoint must be a non-empty regular file")
            checkpoint_sha256 = str(
                self._config.get("model_sha256", "")).strip().lower()
            if (len(checkpoint_sha256) != 64 or
                    any(char not in "0123456789abcdef"
                        for char in checkpoint_sha256)):
                raise ValueError("model_sha256 must be 64 hexadecimal characters")
            arecord_path = os.path.abspath(
                str(self._config.get("arecord_path", "/usr/bin/arecord")))
            # Ubuntu's /usr/bin/arecord is intentionally a symlink to the
            # ALSA aplay multi-call binary.  Validate the final target while
            # preserving argv[0]=/usr/bin/arecord so ALSA selects capture mode.
            arecord_info = os.stat(arecord_path)
            if not stat.S_ISREG(arecord_info.st_mode) or not os.access(
                    arecord_path, os.X_OK):
                raise ValueError("arecord_path must be an executable regular file")

            input_device = str(self._config.get("input_device", "")).strip()
            if "\x00" in input_device or len(input_device) > 256:
                raise ValueError("input_device is invalid")
            if not input_device or input_device.lower() == "auto":
                input_device = _automatic_capture_device(
                    self._capture_device_lister(arecord_path))
            lowered_device = input_device.lower()
            unsafe_device_tokens = ("auto_null", "monitor", "ape")
            if (lowered_device in ("default", "sysdefault", "pulse", "null") or
                    any(token in lowered_device for token in unsafe_device_tokens)):
                raise ValueError(
                    "input_device resolves to a null, monitor, APE, or default source")
            max_seconds = _numeric_config(
                self._config, "max_record_seconds",
                MAX_AUDIO_SECONDS_HARD_LIMIT, 0.1,
                MAX_AUDIO_SECONDS_HARD_LIMIT)
            min_seconds = _numeric_config(
                self._config, "min_record_seconds", 0.3, 0.0,
                MAX_AUDIO_SECONDS_HARD_LIMIT)
            if min_seconds > max_seconds:
                raise ValueError("min_record_seconds cannot exceed max_record_seconds")
            min_rms = _numeric_config(
                self._config, "min_rms", 0.003, 0.0, 1.0)

            smart_chunk_ms = _numeric_config(
                self._config, "smart_chunk_ms", 100.0, 10.0, 1000.0)
            smart_speech_start_ms = _numeric_config(
                self._config, "smart_speech_start_ms", 200.0,
                smart_chunk_ms, 5000.0)
            smart_silence_ms = _numeric_config(
                self._config, "smart_silence_ms", 800.0,
                smart_chunk_ms, 10000.0)
            smart_pre_roll_ms = _numeric_config(
                self._config, "smart_pre_roll_ms", 300.0, 0.0, 5000.0)
            smart_min_utterance_s = _numeric_config(
                self._config, "smart_min_utterance_s", 0.4, 0.05,
                MAX_AUDIO_SECONDS_HARD_LIMIT)
            smart_max_utterance_s = _numeric_config(
                self._config, "smart_max_utterance_s", 15.0, 0.1,
                MAX_AUDIO_SECONDS_HARD_LIMIT)
            if smart_min_utterance_s > smart_max_utterance_s:
                raise ValueError(
                    "smart_min_utterance_s cannot exceed smart_max_utterance_s")
            smart_queue_limit_value = self._config.get("smart_queue_limit", 4)
            if (isinstance(smart_queue_limit_value, bool) or
                    not isinstance(smart_queue_limit_value, int) or
                    smart_queue_limit_value < 1 or smart_queue_limit_value > 32):
                raise ValueError("smart_queue_limit must be an integer between 1 and 32")
            smart_vad_rms = _numeric_config(
                self._config, "smart_vad_rms", min_rms, 0.000001, 1.0)

            # Hash the multi-gigabyte checkpoint only after all inexpensive
            # capture/configuration checks have passed.
            digest = hashlib.sha256()
            with open(checkpoint_path, "rb") as checkpoint_stream:
                hashed_info = os.fstat(checkpoint_stream.fileno())
                while True:
                    chunk = checkpoint_stream.read(4 * 1024 * 1024)
                    if not chunk:
                        break
                    digest.update(chunk)
            actual_sha256 = digest.hexdigest()
            if actual_sha256 != checkpoint_sha256:
                raise ValueError(
                    "%s checkpoint SHA-256 mismatch: expected %s, got %s" %
                    (model_name, checkpoint_sha256, actual_sha256))
            current_info = os.lstat(checkpoint_path)
            identity = (
                hashed_info.st_dev, hashed_info.st_ino, hashed_info.st_size,
                getattr(hashed_info, "st_mtime_ns",
                        int(hashed_info.st_mtime * 1000000000)),
            )
            current_identity = (
                current_info.st_dev, current_info.st_ino, current_info.st_size,
                getattr(current_info, "st_mtime_ns",
                        int(current_info.st_mtime * 1000000000)),
            )
            if current_identity != identity:
                raise ValueError("Whisper checkpoint changed during verification")

            # Probe only.  The multi-gigabyte model remains lazy until a valid
            # utterance reaches stop_recording.
            if not self._cuda_probe():
                raise RuntimeError(
                    "CUDA is unavailable; CPU fallback is forbidden")
            text_converter = (
                self._injected_text_converter
                if self._injected_text_converter is not None
                else _default_text_converter())
            if not callable(text_converter):
                raise RuntimeError("OpenCC t2s converter is not callable")
            if self._using_default_model_loader:
                if importlib.util.find_spec("whisper") is None:
                    raise RuntimeError("OpenAI Whisper is not installed")
                if importlib.util.find_spec("numpy") is None:
                    raise RuntimeError("NumPy is not installed")

            self._model_name = model_name
            self._checkpoint_path = checkpoint_path
            self._checkpoint_sha256 = checkpoint_sha256
            self._checkpoint_identity = identity
            self._arecord_path = arecord_path
            self._input_device = input_device
            self._max_audio_seconds = max_seconds
            self._min_audio_seconds = min_seconds
            self._min_rms = min_rms
            self._language = language
            self._initial_prompt = initial_prompt
            self._smart_chunk_ms = smart_chunk_ms
            self._smart_speech_start_ms = smart_speech_start_ms
            self._smart_silence_ms = smart_silence_ms
            self._smart_pre_roll_ms = smart_pre_roll_ms
            self._smart_min_utterance_s = smart_min_utterance_s
            self._smart_max_utterance_s = smart_max_utterance_s
            self._smart_queue_limit = smart_queue_limit_value
            self._smart_vad_rms = smart_vad_rms
            self._text_converter = text_converter
            self._ready = True
            self._state = "IDLE"
            self._error = ""
        except Exception as exc:
            self._ready = False
            self._state = "ERROR"
            self._error = str(exc)

    def status(self):
        with self._lock:
            now = time.monotonic()
            duration = self._audio_duration_s
            if self._recording and self._record_started_at > 0.0:
                duration = min(
                    self._max_audio_seconds,
                    max(0.0, now - self._record_started_at))
            return {
                "ready": bool(self._ready),
                "state": self._state,
                "model": self._model_name,
                "device": DEVICE_NAME,
                "model_loaded": self._model is not None,
                "input_device": self._input_device,
                "recording": bool(self._recording),
                "recognizing": bool(self._inference_active),
                "smart_listening": bool(self._smart_listening),
                "smart_session_id": self._smart_session_id,
                "session_id": self._smart_session_id,
                "pending_utterances": len(self._smart_queue),
                "utterance_count": int(self._smart_utterance_count),
                "dropped_utterances": int(self._smart_dropped_utterances),
                "capture_id": self._capture_id,
                "audio_duration_s": float(duration),
                "rms": float(self._last_rms),
                "asr_latency_ms": float(self._asr_latency_ms),
                "max_record_seconds": float(self._max_audio_seconds),
                "error": self._error,
            }

    def heartbeat(self):
        with self._lock:
            if self._shutdown_event.is_set():
                raise WorkerRequestError("SHUTDOWN", "ASR worker is shutting down")
            self._last_heartbeat = time.monotonic()
        return {"alive": True}

    def start_recording(self, params):
        capture_id = str((params or {}).get("capture_id", "")).strip()
        if not capture_id or len(capture_id) > 128 or "\x00" in capture_id:
            raise WorkerRequestError(
                "INVALID_CAPTURE_ID", "capture_id must be 1-128 characters")

        with self._operation_lock:
            with self._lock:
                if not self._ready:
                    raise WorkerRequestError("ASR_UNAVAILABLE", self._error)
                if self._shutdown_event.is_set():
                    raise WorkerRequestError("SHUTDOWN", "ASR worker is shutting down")
                if self._recording:
                    raise WorkerRequestError(
                        "ALREADY_RECORDING", "ASR is already recording")
                if self._smart_listening:
                    raise WorkerRequestError(
                        "SMART_LISTENING", "smart listening is already active")
                if self._state == "CANCELLING":
                    raise WorkerRequestError(
                        "ASR_BUSY", "ASR cancellation is still closing the recorder")
                if self._inference_active:
                    raise WorkerRequestError(
                        "ASR_BUSY", "ASR inference is still active")
                if time.monotonic() - self._last_heartbeat > HEARTBEAT_TIMEOUT_SECONDS:
                    raise WorkerRequestError(
                        "HEARTBEAT_STALE", "ASR client heartbeat is stale")

            self._cleanup_capture_files()
            capture_dir = tempfile.mkdtemp(prefix="sweeper_asr_")
            os.chmod(capture_dir, 0o700)
            raw_path = os.path.join(capture_dir, "audio.raw")
            argv = [
                self._arecord_path, "--quiet",
                "--device", self._input_device,
            ]
            argv.extend([
                "--file-type", "raw",
                "--format", "S16_LE",
                "--channels", str(CHANNELS),
                "--rate", str(SAMPLE_RATE),
                "--duration", str(int(math.ceil(self._max_audio_seconds))),
                raw_path,
            ])
            try:
                process = self._popen_factory(
                    argv,
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                    shell=False,
                    close_fds=True,
                    start_new_session=True,
                )
            except Exception as exc:
                self._remove_capture_dir(capture_dir, raw_path)
                with self._lock:
                    self._state = "ERROR"
                    self._error = "unable to start arecord: %s" % exc
                raise WorkerRequestError("ARECORD_START_FAILED", self._error)

            with self._lock:
                self._capture_generation += 1
                self._capture_id = capture_id
                self._capture_dir = capture_dir
                self._raw_path = raw_path
                self._record_proc = process
                self._record_started_at = time.monotonic()
                self._recording = True
                self._audio_duration_s = 0.0
                self._last_rms = 0.0
                self._asr_latency_ms = 0.0
                self._state = "RECORDING"
                self._error = ""
        status = self.status()
        self._emit_status(status)
        return status

    def stop_recording(self, params=None):
        requested_capture_id = str(
            (params or {}).get("capture_id", "")).strip()
        with self._operation_lock:
            with self._lock:
                if self._shutdown_event.is_set():
                    raise WorkerRequestError("SHUTDOWN", "ASR worker is shutting down")
                if not self._recording and self._state != "RECORDED":
                    raise WorkerRequestError(
                        "NOT_RECORDING", "ASR is not recording")
                if requested_capture_id and requested_capture_id != self._capture_id:
                    raise WorkerRequestError(
                        "CAPTURE_MISMATCH", "capture_id does not match current capture")
                if self._inference_active:
                    raise WorkerRequestError("ASR_BUSY", "ASR inference is already active")
                process = self._record_proc
                self._record_proc = None
                self._recording = False
                self._state = "STOPPING"
                capture_id = self._capture_id
                capture_dir = self._capture_dir
                raw_path = self._raw_path
                generation = self._capture_generation

            if process is not None:
                self._terminate_exact_process(process)

            try:
                pcm = self._read_bounded_pcm(raw_path)
            except Exception as exc:
                self._cleanup_capture_files()
                with self._lock:
                    if self._capture_id == capture_id:
                        self._capture_id = ""
                    self._state = "ERROR"
                    self._error = "unable to read recorded PCM: %s" % exc
                raise WorkerRequestError("AUDIO_READ_FAILED", self._error)

            audio_duration_s = len(pcm) / float(
                SAMPLE_RATE * CHANNELS * SAMPLE_BYTES)
            rms = self._pcm_rms(pcm)
            with self._lock:
                self._audio_duration_s = audio_duration_s
                self._last_rms = rms
            self._remove_capture_dir(capture_dir, raw_path)
            with self._lock:
                if self._capture_dir == capture_dir:
                    self._capture_dir = ""
                    self._raw_path = ""

            rejection_reason = ""
            if audio_duration_s < self._min_audio_seconds:
                rejection_reason = (
                    "audio is shorter than %.3f seconds" %
                    self._min_audio_seconds)
            elif rms < self._min_rms:
                rejection_reason = (
                    "audio RMS %.6f is below %.6f" % (rms, self._min_rms))

            if not rejection_reason:
                with self._lock:
                    if generation != self._capture_generation:
                        raise WorkerRequestError(
                            "CANCELLED", "ASR capture was cancelled")
                    self._inference_active = True
                    self._state = "RECOGNIZING"
                    self._error = ""
        if rejection_reason:
            return self._rejected_result(
                capture_id, audio_duration_s, rms, rejection_reason)
        self._emit_status()

        started_at = time.monotonic()
        try:
            model = self._load_model()
            import numpy as np
            audio = np.frombuffer(pcm, dtype="<i2").astype(np.float32)
            audio /= 32768.0
            output = model.transcribe(
                audio,
                language=self._language,
                task="transcribe",
                fp16=True,
                verbose=False,
                initial_prompt=self._initial_prompt or None,
                condition_on_previous_text=False,
                temperature=0.0,
            )
            transcript = self._text_converter(
                str((output or {}).get("text", ""))).strip()
            latency_ms = (time.monotonic() - started_at) * 1000.0
            with self._lock:
                cancelled = generation != self._capture_generation
                self._inference_active = False
                if self._capture_id == capture_id:
                    self._capture_id = ""
                self._asr_latency_ms = latency_ms
                if cancelled:
                    self._state = "CANCELLED"
                    self._error = "ASR capture was cancelled"
                else:
                    self._capture_id = ""
                    self._state = "READY"
                    self._error = "" if transcript else "Whisper returned empty text"
            if cancelled:
                self._unload_if_requested()
                self._emit_status()
                raise WorkerRequestError("CANCELLED", "ASR capture was cancelled")
            result = {
                "accepted": bool(transcript),
                "capture_id": capture_id,
                "transcript": transcript,
                "audio_duration_s": float(audio_duration_s),
                "rms": float(rms),
                "asr_latency_ms": float(latency_ms),
                "model": self._model_name,
                "device": DEVICE_NAME,
                "model_loaded": self._model is not None,
                "error": "" if transcript else "Whisper returned empty text",
            }
            self._emit({"event": "result", "result": result})
            self._emit_status()
            return result
        except WorkerRequestError:
            raise
        except Exception as exc:
            with self._lock:
                cancelled = generation != self._capture_generation
                self._inference_active = False
                self._asr_latency_ms = (time.monotonic() - started_at) * 1000.0
                self._state = "CANCELLED" if cancelled else "ERROR"
                self._error = (
                    "ASR capture was cancelled" if cancelled else str(exc))
                error = self._error
            self._unload_if_requested()
            self._emit_status()
            if cancelled:
                raise WorkerRequestError("CANCELLED", error)
            raise WorkerRequestError("TRANSCRIPTION_FAILED", error)

    def _rejected_result(self, capture_id, audio_duration_s, rms, reason):
        with self._lock:
            if self._capture_id == capture_id:
                self._capture_id = ""
            self._state = "REJECTED"
            self._error = reason
            self._asr_latency_ms = 0.0
        result = {
            "accepted": False,
            "capture_id": capture_id,
            "transcript": "",
            "audio_duration_s": float(audio_duration_s),
            "rms": float(rms),
            "asr_latency_ms": 0.0,
            "model": self._model_name,
            "device": DEVICE_NAME,
            "model_loaded": self._model is not None,
            "error": reason,
        }
        self._emit({"event": "result", "result": result})
        self._emit_status()
        return result

    def start_smart_listening(self, params):
        session_id = str((params or {}).get("session_id", "")).strip()
        if not session_id or len(session_id) > 128 or "\x00" in session_id:
            raise WorkerRequestError(
                "INVALID_SESSION_ID", "session_id must be 1-128 characters")

        with self._operation_lock:
            with self._lock:
                if not self._ready:
                    raise WorkerRequestError("ASR_UNAVAILABLE", self._error)
                if self._shutdown_event.is_set():
                    raise WorkerRequestError(
                        "SHUTDOWN", "ASR worker is shutting down")
                if self._smart_listening:
                    raise WorkerRequestError(
                        "ALREADY_SMART_LISTENING",
                        "smart listening is already active")
                if self._state == "CANCELLING":
                    raise WorkerRequestError(
                        "ASR_BUSY", "previous ASR cancellation is still closing")
                if (self._recording or self._capture_id or
                        self._state == "RECORDED"):
                    raise WorkerRequestError(
                        "MANUAL_RECORDING_ACTIVE",
                        "manual recording and smart listening are mutually exclusive")
                if self._inference_active:
                    raise WorkerRequestError(
                        "ASR_BUSY", "ASR inference is still active")
                if time.monotonic() - self._last_heartbeat > HEARTBEAT_TIMEOUT_SECONDS:
                    raise WorkerRequestError(
                        "HEARTBEAT_STALE", "ASR client heartbeat is stale")

            argv = [
                self._arecord_path, "--quiet",
                "--device", self._input_device,
                "--file-type", "raw",
                "--format", "S16_LE",
                "--channels", str(CHANNELS),
                "--rate", str(SAMPLE_RATE),
            ]
            try:
                process = self._popen_factory(
                    argv,
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    shell=False,
                    close_fds=True,
                    start_new_session=True,
                )
            except Exception as exc:
                with self._lock:
                    self._state = "ERROR"
                    self._error = "unable to start smart arecord: %s" % exc
                raise WorkerRequestError("ARECORD_START_FAILED", self._error)

            with self._smart_condition:
                self._capture_generation += 1
                generation = self._capture_generation
                self._smart_generation = generation
                self._smart_session_id = session_id
                self._smart_proc = process
                self._smart_listening = True
                self._smart_queue.clear()
                self._smart_utterance_count = 0
                self._smart_dropped_utterances = 0
                self._audio_duration_s = 0.0
                self._last_rms = 0.0
                self._asr_latency_ms = 0.0
                self._state = "SMART_LISTENING"
                self._error = ""
                self._smart_condition.notify_all()

            capture_thread = threading.Thread(
                target=self._smart_capture_loop,
                args=(process, generation, session_id),
                name="asr-smart-capture",
                daemon=True,
            )
            stderr_thread = threading.Thread(
                target=self._drain_smart_stderr,
                args=(process,),
                name="asr-smart-stderr",
                daemon=True,
            )
            with self._lock:
                self._smart_capture_thread = capture_thread
            try:
                stderr_thread.start()
                capture_thread.start()
            except BaseException as exc:
                cleanup_process = None
                message = "unable to start smart capture thread: %s" % exc
                with self._smart_condition:
                    if (generation == self._capture_generation and
                            self._smart_listening and
                            process is self._smart_proc):
                        cleanup_process = self._invalidate_smart_locked(
                            "ERROR", message)
                    self._smart_capture_thread = None
                if cleanup_process is not None:
                    self._terminate_exact_process(
                        cleanup_process, communicate=False)
                    self._complete_smart_invalidation()
                if isinstance(exc, Exception):
                    raise WorkerRequestError("THREAD_START_FAILED", message)
                raise
        status = self.status()
        self._emit_status(status)
        return status

    def stop_smart_listening(self, params=None):
        requested_session = str(
            (params or {}).get("session_id", "")).strip()
        with self._operation_lock:
            with self._smart_condition:
                if not self._smart_listening:
                    raise WorkerRequestError(
                        "NOT_SMART_LISTENING", "smart listening is not active")
                if (requested_session and
                        requested_session != self._smart_session_id):
                    raise WorkerRequestError(
                        "SESSION_MISMATCH",
                        "session_id does not match current smart session")
                stopped_session = self._smart_session_id
                stopped_generation = self._smart_generation
                process = self._invalidate_smart_locked(
                    "SMART_STOPPED", "")
            if process is not None:
                self._terminate_exact_process(process, communicate=False)
                self._complete_smart_invalidation()
        self._emit_smart_terminal(
            "smart_stopped", stopped_session, stopped_generation,
            "SMART_STOPPED", "")
        status = self.status()
        status["session_id"] = stopped_session
        status["stopped_session_id"] = stopped_session
        self._emit_status(status)
        return status

    def _invalidate_smart_locked(self, state, error):
        """Invalidate the current smart generation; caller holds _smart_condition."""
        self._capture_generation += 1
        self._smart_generation = self._capture_generation
        process = self._smart_proc
        self._smart_proc = None
        self._smart_listening = False
        self._smart_session_id = ""
        self._smart_queue.clear()
        self._smart_terminal_state = state
        self._state = (
            "CANCELLING" if (
                self._smart_inference_active or process is not None) else state)
        self._error = error
        self._smart_condition.notify_all()
        return process

    def _smart_capture_loop(self, process, generation, session_id):
        samples_per_chunk = max(
            1, int(round(SAMPLE_RATE * self._smart_chunk_ms / 1000.0)))
        chunk_bytes = samples_per_chunk * CHANNELS * SAMPLE_BYTES
        pre_roll_limit = int(math.ceil(
            self._smart_pre_roll_ms / self._smart_chunk_ms))
        speech_start_chunks = max(1, int(math.ceil(
            self._smart_speech_start_ms / self._smart_chunk_ms)))
        silence_end_chunks = max(1, int(math.ceil(
            self._smart_silence_ms / self._smart_chunk_ms)))
        max_utterance_frames = max(
            1, int(SAMPLE_RATE * self._smart_max_utterance_s))
        max_utterance_bytes = (
            max_utterance_frames * CHANNELS * SAMPLE_BYTES)

        pre_roll = deque(maxlen=pre_roll_limit or None)
        speech_run = []
        utterance = None
        silence_run = []
        buffered = b""
        stream = process.stdout
        try:
            while stream is not None:
                data = stream.read(max(1, chunk_bytes - len(buffered)))
                if not data:
                    break
                buffered += data
                if len(buffered) < chunk_bytes:
                    continue
                chunk = buffered[:chunk_bytes]
                buffered = buffered[chunk_bytes:]
                with self._lock:
                    if (generation != self._capture_generation or
                            not self._smart_listening or
                            session_id != self._smart_session_id or
                            process is not self._smart_proc):
                        return
                is_speech = self._pcm_rms(chunk) >= self._smart_vad_rms
                if utterance is None:
                    if is_speech:
                        speech_run.append(chunk)
                        if len(speech_run) >= speech_start_chunks:
                            utterance = list(pre_roll) + speech_run
                            speech_run = []
                            silence_run = []
                    else:
                        speech_run = []
                        if pre_roll_limit:
                            pre_roll.append(chunk)
                    continue

                utterance.append(chunk)
                if is_speech:
                    silence_run = []
                else:
                    silence_run.append(chunk)
                reached_silence = len(silence_run) >= silence_end_chunks
                reached_limit = sum(len(item) for item in utterance) >= \
                    max_utterance_bytes
                if not reached_silence and not reached_limit:
                    continue

                pcm = b"".join(utterance)[:max_utterance_bytes]
                self._queue_smart_utterance(generation, session_id, pcm)
                if reached_silence and pre_roll_limit:
                    pre_roll = deque(
                        silence_run[-pre_roll_limit:], maxlen=pre_roll_limit)
                else:
                    pre_roll = deque(maxlen=pre_roll_limit or None)
                speech_run = []
                utterance = None
                silence_run = []
        except Exception as exc:
            self._fail_smart_capture(
                generation, session_id, process,
                "smart audio capture failed: %s" % exc)
            return
        finally:
            if stream is not None:
                try:
                    stream.close()
                except Exception:
                    pass

        with self._lock:
            expected_exit = bool(
                generation != self._capture_generation or
                not self._smart_listening or
                process is not self._smart_proc)
        if not expected_exit:
            return_code = process.poll()
            self._fail_smart_capture(
                generation, session_id, process,
                "smart arecord exited unexpectedly%s" % (
                    "" if return_code is None else " with status %s" % return_code))

    def _queue_smart_utterance(self, generation, session_id, pcm):
        audio_duration_s = len(pcm) / float(
            SAMPLE_RATE * CHANNELS * SAMPLE_BYTES)
        rms = self._pcm_rms(pcm)
        # The consecutive start gate already proved real speech.  Do not
        # compare whole-utterance RMS with the VAD threshold again: configured
        # trailing silence intentionally lowers that aggregate RMS and would
        # otherwise reject quiet but valid speech.
        if audio_duration_s < self._smart_min_utterance_s:
            return False
        dropped = None
        with self._smart_condition:
            if (generation != self._capture_generation or
                    not self._smart_listening or
                    session_id != self._smart_session_id):
                return False
            self._smart_utterance_count += 1
            utterance_index = self._smart_utterance_count
            utterance_id = "%s:%d" % (session_id, utterance_index)
            if len(self._smart_queue) >= self._smart_queue_limit:
                self._smart_dropped_utterances += 1
                self._error = "smart utterance queue is full"
                dropped = {
                    "session_id": session_id,
                    "utterance_id": utterance_id,
                    "utterance_index": utterance_index,
                    "reason": self._error,
                }
            else:
                self._smart_queue.append({
                    "generation": generation,
                    "session_id": session_id,
                    "utterance_id": utterance_id,
                    "utterance_index": utterance_index,
                    "pcm": pcm,
                    "audio_duration_s": audio_duration_s,
                    "rms": rms,
                })
                self._smart_condition.notify_all()
        if dropped is not None:
            self._emit({"event": "smart_queue_drop", "result": dropped})
        self._emit_status()
        return dropped is None

    def _smart_inference_loop(self):
        while True:
            with self._smart_condition:
                while not self._smart_queue and not self._shutdown_event.is_set():
                    self._smart_condition.wait(timeout=0.5)
                if self._shutdown_event.is_set():
                    return
                item = self._smart_queue.popleft()
                if (item["generation"] != self._capture_generation or
                        not self._smart_listening or
                        item["session_id"] != self._smart_session_id):
                    continue
                self._inference_active = True
                self._smart_inference_active = True
                self._state = "SMART_RECOGNIZING"
                self._error = ""
            self._emit_status()
            self._recognize_smart_utterance(item)

    def _recognize_smart_utterance(self, item):
        started_at = time.monotonic()
        transcript = ""
        failure = ""
        try:
            model = self._load_model()
            with self._lock:
                if (item["generation"] != self._capture_generation or
                        not self._smart_listening or
                        item["session_id"] != self._smart_session_id or
                        time.monotonic() - self._last_heartbeat >
                        HEARTBEAT_TIMEOUT_SECONDS):
                    raise WorkerRequestError(
                        "CANCELLED", "smart utterance was cancelled")
            import numpy as np
            audio = np.frombuffer(item["pcm"], dtype="<i2").astype(np.float32)
            audio /= 32768.0
            output = model.transcribe(
                audio,
                language=self._language,
                task="transcribe",
                fp16=True,
                verbose=False,
                initial_prompt=self._initial_prompt or None,
                condition_on_previous_text=False,
                temperature=0.0,
            )
            transcript = self._text_converter(
                str((output or {}).get("text", ""))).strip()
        except WorkerRequestError:
            pass
        except Exception as exc:
            failure = str(exc)

        latency_ms = (time.monotonic() - started_at) * 1000.0
        process_to_stop = None
        terminal_event = None
        transcript_event = None
        with self._smart_condition:
            session_current = bool(
                item["generation"] == self._capture_generation and
                self._smart_listening and
                item["session_id"] == self._smart_session_id)
            heartbeat_stale = bool(
                session_current and
                time.monotonic() - self._last_heartbeat >
                HEARTBEAT_TIMEOUT_SECONDS)
            valid = bool(session_current and not heartbeat_stale and
                         not self._shutdown_event.is_set())
            self._inference_active = False
            self._smart_inference_active = False
            if heartbeat_stale:
                reason = "ASR client heartbeat timed out"
                process_to_stop = self._invalidate_smart_locked(
                    "CANCELLED", reason)
                terminal_event = (
                    "smart_stopped", item["session_id"],
                    item["generation"], "CANCELLED", reason)
            elif valid:
                self._audio_duration_s = item["audio_duration_s"]
                self._last_rms = item["rms"]
                self._asr_latency_ms = latency_ms
                self._state = "SMART_LISTENING"
                self._error = failure or (
                    "" if transcript else "Whisper returned empty text")
                if failure:
                    process_to_stop = self._invalidate_smart_locked(
                        "ERROR", "smart transcription failed: %s" % failure)
                    terminal_event = (
                        "smart_error", item["session_id"],
                        item["generation"], "ERROR",
                        "smart transcription failed: %s" % failure)
                else:
                    result = {
                        "session_id": item["session_id"],
                        "generation": int(item["generation"]),
                        "utterance_id": item["utterance_id"],
                        "utterance_index": int(item["utterance_index"]),
                        "transcript": transcript,
                        "audio_duration_s": float(item["audio_duration_s"]),
                        "rms": float(item["rms"]),
                        "asr_latency_ms": float(latency_ms),
                        "accepted": bool(transcript),
                        "model": self._model_name,
                        "device": DEVICE_NAME,
                        "model_loaded": self._model is not None,
                        "error": (
                            "" if transcript else "Whisper returned empty text"),
                    }
                    transcript_event = {
                        "event": "smart_transcript", "result": result}
            elif self._state == "CANCELLING":
                self._state = self._smart_terminal_state
            self._smart_condition.notify_all()
        # Protocol output can block indefinitely under pipe backpressure.  It
        # must never run under the generation/state lock: stop, cancel and the
        # heartbeat watchdog must remain able to close the exact recorder.
        # The node also verifies session_id/generation before forwarding a
        # transcript, so a transport frame delayed by a stalled reader is
        # harmless after the session has been revoked.
        if transcript_event is not None:
            self._emit(transcript_event)
        if process_to_stop is not None:
            self._terminate_exact_process(process_to_stop, communicate=False)
            self._complete_smart_invalidation()
        if terminal_event is not None:
            self._emit_smart_terminal(*terminal_event)
        self._unload_if_requested()
        self._emit_status()

    def _fail_smart_capture(self, generation, session_id, process, reason):
        process_to_stop = None
        with self._smart_condition:
            if (generation == self._capture_generation and
                    self._smart_listening and
                    session_id == self._smart_session_id and
                    process is self._smart_proc):
                process_to_stop = self._invalidate_smart_locked("ERROR", reason)
        if process_to_stop is not None and process_to_stop.poll() is None:
            self._terminate_exact_process(process_to_stop, communicate=False)
        if process_to_stop is not None:
            self._complete_smart_invalidation()
            self._emit_smart_terminal(
                "smart_error", session_id, generation, "ERROR", reason)
            self._emit_status()

    def _emit_smart_terminal(self, event_name, session_id, generation,
                             state, error):
        if not session_id:
            return
        self._emit({
            "event": event_name,
            "result": {
                "session_id": str(session_id),
                "generation": int(generation),
                "state": str(state),
                "error": str(error or ""),
                "smart_listening": False,
                "pending_utterances": 0,
            },
        })

    def _complete_smart_invalidation(self):
        with self._smart_condition:
            if (not self._smart_listening and
                    not self._smart_inference_active and
                    self._state == "CANCELLING"):
                self._state = self._smart_terminal_state
                self._smart_condition.notify_all()

    @staticmethod
    def _drain_smart_stderr(process):
        stream = getattr(process, "stderr", None)
        if stream is None:
            return
        try:
            while stream.read(4096):
                pass
        except Exception:
            pass
        finally:
            try:
                stream.close()
            except Exception:
                pass

    def cancel(self, params=None):
        unload_model = (params or {}).get("unload_model", False)
        if not isinstance(unload_model, bool):
            raise WorkerRequestError(
                "INVALID_PARAMS", "unload_model must be a boolean")
        with self._operation_lock:
            with self._smart_condition:
                smart_session = self._smart_session_id
                smart_generation = self._smart_generation
                self._capture_generation += 1
                self._unload_requested = bool(unload_model)
                self._capture_id = ""
                process = self._record_proc
                self._record_proc = None
                self._recording = False
                smart_process = self._smart_proc
                self._smart_proc = None
                self._smart_listening = False
                self._smart_session_id = ""
                self._smart_queue.clear()
                self._smart_terminal_state = "CANCELLED"
                self._state = (
                    "CANCELLING" if self._inference_active else "CANCELLED")
                self._error = "ASR capture was cancelled"
                capture_dir = self._capture_dir
                raw_path = self._raw_path
                self._capture_dir = ""
                self._raw_path = ""
                self._smart_condition.notify_all()
            if process is not None:
                self._terminate_exact_process(process)
            if smart_process is not None:
                self._terminate_exact_process(
                    smart_process, communicate=False)
            self._remove_capture_dir(capture_dir, raw_path)
            if not self._inference_active:
                self._unload_if_requested()
        if smart_session:
            self._emit_smart_terminal(
                "smart_stopped", smart_session, smart_generation,
                "CANCELLED", "ASR capture was cancelled")
        status = self.status()
        self._emit_status(status)
        return status

    def shutdown(self):
        self._shutdown_event.set()
        with self._operation_lock:
            with self._smart_condition:
                smart_session = self._smart_session_id
                smart_generation = self._smart_generation
                self._capture_generation += 1
                self._capture_id = ""
                process = self._record_proc
                self._record_proc = None
                self._recording = False
                smart_process = self._smart_proc
                self._smart_proc = None
                self._smart_listening = False
                self._smart_session_id = ""
                self._smart_queue.clear()
                self._smart_terminal_state = "SHUTDOWN"
                capture_dir = self._capture_dir
                raw_path = self._raw_path
                self._capture_dir = ""
                self._raw_path = ""
                self._state = "SHUTDOWN"
                self._error = ""
                self._smart_condition.notify_all()
            if process is not None:
                self._terminate_exact_process(process)
            if smart_process is not None:
                self._terminate_exact_process(
                    smart_process, communicate=False)
            self._remove_capture_dir(capture_dir, raw_path)
        if smart_session:
            self._emit_smart_terminal(
                "smart_stopped", smart_session, smart_generation,
                "SHUTDOWN", "ASR worker is shutting down")
        status = self.status()
        self._emit_status(status)
        return status

    def _load_model(self):
        with self._model_lock:
            if self._model is None:
                # Recheck CUDA at the moment of allocation.  A failure is
                # terminal for this request and never falls back to CPU.
                if not self._cuda_probe():
                    raise RuntimeError(
                        "CUDA is unavailable; CPU fallback is forbidden")
                checkpoint_info = os.lstat(self._checkpoint_path)
                checkpoint_identity = (
                    checkpoint_info.st_dev, checkpoint_info.st_ino,
                    checkpoint_info.st_size,
                    getattr(checkpoint_info, "st_mtime_ns",
                            int(checkpoint_info.st_mtime * 1000000000)),
                )
                if (stat.S_ISLNK(checkpoint_info.st_mode) or
                        checkpoint_identity != self._checkpoint_identity):
                    raise RuntimeError(
                        "Whisper checkpoint changed after startup verification")
                model = self._model_loader(self._checkpoint_path)
                _validate_model(model, self._model_name)
                self._model = model
            return self._model

    def _unload_if_requested(self):
        with self._lock:
            should_unload = self._unload_requested and not self._inference_active
            if should_unload:
                self._unload_requested = False
        if not should_unload:
            return False
        with self._model_lock:
            self._model = None
        try:
            import torch
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
        except Exception:
            pass
        return True

    def _watchdog_loop(self):
        while not self._shutdown_event.wait(WATCHDOG_INTERVAL_SECONDS):
            try:
                self._watchdog_tick(time.monotonic())
            except Exception as exc:
                logger.warning("recording watchdog error: %s", exc)

    def _watchdog_tick(self, now):
        process = None
        smart_process = None
        capture_dir = ""
        raw_path = ""
        invalidate = False
        reason = ""
        smart_session = ""
        smart_generation = 0
        with self._smart_condition:
            if (self._smart_listening and
                    now - self._last_heartbeat > HEARTBEAT_TIMEOUT_SECONDS):
                smart_session = self._smart_session_id
                smart_generation = self._smart_generation
                smart_process = self._invalidate_smart_locked(
                    "CANCELLED", "ASR client heartbeat timed out")
            if not self._recording or self._record_proc is None:
                process = None
            else:
                elapsed = max(0.0, now - self._record_started_at)
                process_status = self._record_proc.poll()
                if now - self._last_heartbeat > HEARTBEAT_TIMEOUT_SECONDS:
                    reason = "ASR client heartbeat timed out"
                    invalidate = True
                elif elapsed >= self._max_audio_seconds:
                    reason = "recording reached %.1f second limit" % self._max_audio_seconds
                elif process_status is not None:
                    if process_status == 0:
                        reason = "arecord completed"
                    else:
                        reason = "arecord exited with status %s" % process_status
                        invalidate = True
                if reason:
                    process = self._record_proc
                    self._record_proc = None
                    self._recording = False
                    self._audio_duration_s = min(elapsed, self._max_audio_seconds)
                    if invalidate:
                        self._capture_generation += 1
                        self._capture_id = ""
                        capture_dir = self._capture_dir
                        raw_path = self._raw_path
                        self._capture_dir = ""
                        self._raw_path = ""
                        self._state = "CANCELLED" if "heartbeat" in reason else "ERROR"
                        self._error = reason
                    else:
                        self._state = "RECORDED"
                        self._error = ""
        if smart_process is not None:
            self._terminate_exact_process(
                smart_process, communicate=False)
            self._complete_smart_invalidation()
            self._emit_smart_terminal(
                "smart_stopped", smart_session, smart_generation,
                "CANCELLED", "ASR client heartbeat timed out")
        if process is not None and process.poll() is None:
            self._terminate_exact_process(process)
        elif process is not None:
            try:
                process.communicate(timeout=0.1)
            except Exception:
                pass
        if invalidate:
            self._remove_capture_dir(capture_dir, raw_path)
        if smart_process is not None or process is not None:
            self._emit_status()

    @staticmethod
    def _terminate_exact_process(process, communicate=True):
        if process is None:
            return
        if process.poll() is None:
            try:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=1.5)
            except subprocess.TimeoutExpired:
                try:
                    process.terminate()
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=1.0)
            except ProcessLookupError:
                pass
        if communicate:
            try:
                process.communicate(timeout=0.1)
            except Exception:
                pass

    def _read_bounded_pcm(self, raw_path):
        if not raw_path:
            raise OSError("recording file is unavailable")
        maximum = int(
            SAMPLE_RATE * CHANNELS * SAMPLE_BYTES * self._max_audio_seconds)
        with open(raw_path, "rb") as stream:
            pcm = stream.read(maximum + SAMPLE_BYTES)
        pcm = pcm[:maximum]
        if len(pcm) % SAMPLE_BYTES:
            pcm = pcm[:-1]
        return pcm

    @staticmethod
    def _pcm_rms(pcm):
        if not pcm:
            return 0.0
        samples = array("h")
        samples.frombytes(pcm)
        if sys.byteorder != "little":
            samples.byteswap()
        total = sum(float(value) * float(value) for value in samples)
        return math.sqrt(total / len(samples)) / 32768.0

    def _cleanup_capture_files(self):
        with self._lock:
            capture_dir = self._capture_dir
            raw_path = self._raw_path
            self._capture_dir = ""
            self._raw_path = ""
        self._remove_capture_dir(capture_dir, raw_path)

    @staticmethod
    def _remove_capture_dir(capture_dir, raw_path):
        if raw_path:
            try:
                os.unlink(raw_path)
            except FileNotFoundError:
                pass
            except OSError:
                pass
        if capture_dir:
            try:
                os.rmdir(capture_dir)
            except FileNotFoundError:
                pass
            except OSError:
                pass

    def _emit_status(self, status=None):
        self._emit({"event": "status", "status": status or self.status()})

    def _emit(self, event):
        callback = self._event_callback
        if callback is None:
            return
        try:
            callback(event)
        except Exception:
            pass


class JsonLineServer(object):
    """Concurrent request dispatcher with stdout reserved for JSON frames."""

    def __init__(self, worker, input_stream=None, output_stream=None):
        self.worker = worker
        self.input_stream = input_stream or sys.stdin
        self.output_stream = output_stream or sys.stdout
        self._write_lock = threading.Lock()
        self._shutdown_lock = threading.Lock()
        self._worker_shutdown_started = False
        self._worker_shutdown_done = threading.Event()
        self._shutdown_result = None
        self._shutdown_error = None
        self._stopping = False
        self.worker.set_event_callback(self._write_event)

    def serve(self):
        try:
            for line in self.input_stream:
                line = line.strip()
                if not line:
                    continue
                try:
                    request = json.loads(line)
                    self._dispatch(request)
                except ValueError as exc:
                    self._write_error(None, "INVALID_JSON", str(exc))
                if self._stopping:
                    break
        finally:
            # EOF, a normal shutdown request, and a broken stdout pipe all
            # converge here.  shutdown() is idempotent and must always run:
            # arecord owns a separate session and cannot be left behind merely
            # because writing the final protocol frame failed.
            self._shutdown_worker_once()

    def _dispatch(self, request):
        if not isinstance(request, dict):
            self._write_error(None, "INVALID_REQUEST", "request must be an object")
            return
        request_id = request.get("id")
        method = request.get("method")
        params = request.get("params", {})
        if params is None:
            params = {}
        if request_id is None or not isinstance(method, str) or not method:
            self._write_error(
                request_id, "INVALID_REQUEST", "id and method are required")
            return
        if not isinstance(params, dict):
            self._write_error(
                request_id, "INVALID_PARAMS", "params must be an object")
            return
        if method == "stop_recording":
            threading.Thread(
                target=self._run_request,
                args=(request_id, self.worker.stop_recording, params),
                name="asr-stop-transcribe",
                daemon=True,
            ).start()
            return
        if method == "status":
            self._run_request(request_id, self.worker.status)
        elif method == "heartbeat":
            self._run_request(request_id, self.worker.heartbeat)
        elif method == "start_recording":
            self._run_request(request_id, self.worker.start_recording, params)
        elif method == "start_smart_listening":
            self._run_request(
                request_id, self.worker.start_smart_listening, params)
        elif method == "stop_smart_listening":
            self._run_request(
                request_id, self.worker.stop_smart_listening, params)
        elif method == "cancel":
            self._run_request(request_id, self.worker.cancel, params)
        elif method == "shutdown":
            self._run_request(request_id, self._shutdown_worker_once)
            self._stopping = True
        else:
            self._write_error(
                request_id, "METHOD_NOT_FOUND", "unknown method: %s" % method)

    def _run_request(self, request_id, function, *args):
        try:
            result = function(*args)
            if not isinstance(result, dict):
                raise RuntimeError("worker method result must be an object")
            self._write({"id": request_id, "ok": True, "result": result})
        except WorkerRequestError as exc:
            self._write_error(request_id, exc.code, exc.message, exc.data)
        except Exception as exc:
            logger.warning("request failed: %s", exc)
            self._write_error(request_id, "INTERNAL_ERROR", str(exc))

    def _write_event(self, event):
        if isinstance(event, dict):
            self._write(event)

    def _write_error(self, request_id, code, message, data=None):
        error = {"code": code, "message": message}
        if data is not None:
            error["data"] = data
        self._write({"id": request_id, "ok": False, "error": error})

    def _write(self, frame):
        line = json.dumps(frame, ensure_ascii=False, separators=(",", ":")) + "\n"
        try:
            with self._write_lock:
                self.output_stream.write(line)
                self.output_stream.flush()
        except (BrokenPipeError, OSError):
            first_failure = not self._stopping
            self._stopping = True
            if first_failure:
                # A background smart-transcript write can discover parent
                # loss while the main thread is blocked reading stdin.  Cut
                # the callback first to avoid recursive writes, then close the
                # exact microphone process immediately instead of waiting for
                # another request to wake serve().
                self.worker.set_event_callback(None)
                threading.Thread(
                    target=self._shutdown_worker_once,
                    name="asr-broken-pipe-shutdown",
                    daemon=True,
                ).start()

    def _shutdown_worker_once(self):
        with self._shutdown_lock:
            owner = not self._worker_shutdown_started
            if owner:
                self._worker_shutdown_started = True
        if owner:
            try:
                result = self.worker.shutdown()
            except BaseException as exc:
                with self._shutdown_lock:
                    self._shutdown_error = exc
                self._worker_shutdown_done.set()
                raise
            with self._shutdown_lock:
                self._shutdown_result = result
            self._worker_shutdown_done.set()
            return result

        # A broken-pipe callback may start shutdown on a daemon thread just
        # before serve() reaches its finally block.  Do not mistake "started"
        # for "finished": waiting here guarantees the exact arecord child was
        # reaped before the JSON server/main thread is allowed to exit.
        self._worker_shutdown_done.wait()
        with self._shutdown_lock:
            error = self._shutdown_error
            result = self._shutdown_result
        if error is not None:
            raise error
        return result or {"state": "SHUTDOWN"}


def _parse_config(argv=None):
    parser = argparse.ArgumentParser(description="isolated selectable Whisper ASR worker")
    parser.add_argument("--config-json", required=True)
    args = parser.parse_args(argv)
    try:
        config = json.loads(args.config_json)
    except ValueError as exc:
        raise SystemExit("invalid --config-json: %s" % exc)
    if not isinstance(config, dict):
        raise SystemExit("--config-json must decode to an object")
    return config


def main(argv=None):
    logging.basicConfig(
        level=logging.INFO,
        stream=sys.stderr,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    config = _parse_config(argv)
    worker = WhisperAsrWorker(config)
    server = JsonLineServer(worker)
    server.serve()
    return 0


if __name__ == "__main__":
    sys.exit(main())
