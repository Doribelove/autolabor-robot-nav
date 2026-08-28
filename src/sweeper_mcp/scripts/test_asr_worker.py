#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline tests for the isolated ASR worker and JSON-lines client."""

import hashlib
import importlib.util
import json
import os
import stat
import subprocess
import sys
import tempfile
import textwrap
import threading
import time
from types import SimpleNamespace
from unittest import mock


_THIS = os.path.dirname(os.path.abspath(__file__))
_PKG_SRC = os.path.abspath(os.path.join(_THIS, "..", "src"))
if _PKG_SRC not in sys.path:
    sys.path.insert(0, _PKG_SRC)

from sweeper_mcp.asr_client import AsrClient  # noqa: E402

_WORKER_PATH = os.path.join(_THIS, "whisper_asr_worker.py")
_SPEC = importlib.util.spec_from_file_location("whisper_asr_worker_test", _WORKER_PATH)
_WORKER_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_WORKER_MODULE)
WhisperAsrWorker = _WORKER_MODULE.WhisperAsrWorker
JsonLineServer = _WORKER_MODULE.JsonLineServer
parse_alsa_capture_devices = _WORKER_MODULE._parse_alsa_capture_devices


class _FakeDims(object):
    n_mels = 80
    n_audio_state = 1024
    n_audio_layer = 24
    n_text_state = 1024
    n_text_layer = 24


class _FakeModel(object):
    dims = _FakeDims()

    def __init__(self, text="向前一米", dims=None):
        self.dims = dims or _FakeDims()
        self.text = text
        self.calls = []

    def transcribe(self, audio, **kwargs):
        self.calls.append((audio, kwargs))
        return {"text": self.text}


class _SequenceModel(_FakeModel):
    def __init__(self, texts):
        super().__init__(text="")
        self.texts = list(texts)
        self._active = 0
        self.max_active = 0
        self._lock = threading.Lock()

    def transcribe(self, audio, **kwargs):
        with self._lock:
            self._active += 1
            self.max_active = max(self.max_active, self._active)
            index = len(self.calls)
            self.calls.append((audio, kwargs))
        try:
            return {"text": self.texts[index]}
        finally:
            with self._lock:
                self._active -= 1


def _write_executable(path, body):
    with open(path, "w", encoding="utf-8") as stream:
        stream.write(body)
    os.chmod(path, stat.S_IRUSR | stat.S_IWUSR | stat.S_IXUSR)


def _fake_arecord(path, amplitude):
    body = textwrap.dedent("""\
        #!{python}
        import signal
        import struct
        import sys
        import time

        running = [True]
        def stop(_signum, _frame):
            running[0] = False
        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)
        output = sys.argv[-1]
        chunk = struct.pack("<h", {amplitude}) * 1600
        with open(output, "wb", buffering=0) as stream:
            while running[0]:
                stream.write(chunk)
                time.sleep(0.01)
    """).format(python=sys.executable, amplitude=int(amplitude))
    _write_executable(path, body)


def _fake_smart_arecord(path, amplitudes, chunk_ms=20, delay_s=0.003):
    body = textwrap.dedent("""\
        #!{python}
        import signal
        import struct
        import sys
        import time

        running = [True]
        def stop(_signum, _frame):
            running[0] = False
        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)
        samples = int(16000 * {chunk_ms} / 1000)
        try:
            for amplitude in {amplitudes!r}:
                if not running[0]:
                    break
                sys.stdout.buffer.write(struct.pack("<h", amplitude) * samples)
                sys.stdout.buffer.flush()
                time.sleep({delay_s})
            silence = struct.pack("<h", 0) * samples
            while running[0]:
                sys.stdout.buffer.write(silence)
                sys.stdout.buffer.flush()
                time.sleep({delay_s})
        except BrokenPipeError:
            pass
    """).format(
        python=sys.executable,
        amplitudes=[int(value) for value in amplitudes],
        chunk_ms=int(chunk_ms),
        delay_s=float(delay_s),
    )
    _write_executable(path, body)


def _worker_config(directory, arecord_path, model="medium"):
    filenames = {
        "small": "small.pt",
        "medium": "medium.pt",
        "large": "large-v3.pt",
    }
    model_path = os.path.join(directory, filenames.get(model, "invalid.pt"))
    payload = ("offline-%s-checkpoint-placeholder" % model).encode("ascii")
    with open(model_path, "wb") as stream:
        stream.write(payload)
    return {
        "model": model,
        "model_path": model_path,
        "model_sha256": hashlib.sha256(payload).hexdigest(),
        "device": "cuda",
        "arecord_path": arecord_path,
        "input_device": "plughw:offline,0",
        "sample_rate": 16000,
        "max_record_seconds": 30.0,
        "min_record_seconds": 0.05,
        "min_rms": 0.003,
        "language": "zh",
        "initial_prompt": "简体中文",
    }


def _smart_worker_config(directory, arecord_path, queue_limit=4):
    config = _worker_config(directory, arecord_path)
    config.update({
        "smart_chunk_ms": 20,
        "smart_speech_start_ms": 40,
        "smart_silence_ms": 40,
        "smart_pre_roll_ms": 20,
        "smart_min_utterance_s": 0.06,
        "smart_max_utterance_s": 0.3,
        "smart_queue_limit": int(queue_limit),
        "smart_vad_rms": 0.01,
    })
    return config


def _wait_for(predicate, timeout=3.0, description="condition"):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("timed out waiting for %s" % description)


def test_worker_uses_exact_bounded_arecord_argv_and_returns_metrics():
    with tempfile.TemporaryDirectory() as directory:
        arecord_path = os.path.join(directory, "fake_arecord")
        _fake_arecord(arecord_path, amplitude=1200)
        config = _worker_config(directory, arecord_path)
        commands = []
        model = _FakeModel(text="向前壹米")

        def popen(argv, **kwargs):
            commands.append((list(argv), dict(kwargs)))
            return subprocess.Popen(argv, **kwargs)

        worker = WhisperAsrWorker(
            config,
            cuda_probe=lambda: True,
            model_loader=lambda _path: model,
            popen_factory=popen,
            text_converter=lambda text: text.replace("壹", "一"),
        )
        try:
            worker.heartbeat()
            started = worker.start_recording({"capture_id": "capture-1"})
            assert started["recording"] is True
            time.sleep(0.12)
            result = worker.stop_recording({"capture_id": "capture-1"})
            assert result["accepted"] is True
            assert result["capture_id"] == "capture-1"
            assert result["transcript"] == "向前一米"
            assert 0.05 <= result["audio_duration_s"] <= 30.0
            assert result["rms"] > config["min_rms"]
            assert result["asr_latency_ms"] >= 0.0
            assert result["model_loaded"] is True
            assert len(model.calls) == 1
            assert model.calls[0][1]["language"] == "zh"
            assert model.calls[0][1]["fp16"] is True

            argv, kwargs = commands[0]
            assert argv[0] == arecord_path
            assert argv[1:] == [
                "--quiet",
                "--device", "plughw:offline,0",
                "--file-type", "raw",
                "--format", "S16_LE",
                "--channels", "1",
                "--rate", "16000",
                "--duration", "30",
                argv[-1],
            ]
            assert kwargs["shell"] is False
            assert kwargs["stdin"] is subprocess.DEVNULL
            assert kwargs["stdout"] is subprocess.DEVNULL
        finally:
            worker.shutdown()


def test_short_or_quiet_audio_is_rejected_before_model_load():
    with tempfile.TemporaryDirectory() as directory:
        arecord_path = os.path.join(directory, "fake_arecord")
        _fake_arecord(arecord_path, amplitude=0)
        config = _worker_config(directory, arecord_path)
        loaded = []
        worker = WhisperAsrWorker(
            config,
            cuda_probe=lambda: True,
            model_loader=lambda _path: loaded.append(True),
            text_converter=lambda text: text,
        )
        try:
            worker.heartbeat()
            worker.start_recording({"capture_id": "quiet"})
            time.sleep(0.10)
            result = worker.stop_recording({"capture_id": "quiet"})
            assert result["accepted"] is False
            assert result["transcript"] == ""
            assert result["rms"] == 0.0
            assert "RMS" in result["error"]
            assert loaded == []
            assert result["model_loaded"] is False
        finally:
            worker.shutdown()


def test_normal_arecord_multicall_symlink_is_accepted_without_realpath_rewrite():
    with tempfile.TemporaryDirectory() as directory:
        target_path = os.path.join(directory, "aplay")
        arecord_path = os.path.join(directory, "arecord")
        _fake_arecord(target_path, amplitude=1000)
        os.symlink(target_path, arecord_path)
        config = _worker_config(directory, arecord_path)
        commands = []

        def popen(argv, **kwargs):
            commands.append(list(argv))
            return subprocess.Popen(argv, **kwargs)

        worker = WhisperAsrWorker(
            config,
            cuda_probe=lambda: True,
            model_loader=lambda _path: _FakeModel(),
            popen_factory=popen,
            text_converter=lambda text: text,
        )
        try:
            assert worker.status()["ready"] is True
            worker.heartbeat()
            worker.start_recording({"capture_id": "symlink"})
            assert commands[0][0] == arecord_path
        finally:
            worker.shutdown()


def test_smart_listening_segments_multiple_utterances_in_order():
    amplitudes = [
        0, 0, 1200, 1200, 1200, 0, 0,
        0, 0, 1600, 1600, 1600, 0, 0,
    ]
    with tempfile.TemporaryDirectory() as directory:
        arecord_path = os.path.join(directory, "fake_smart_arecord")
        _fake_smart_arecord(arecord_path, amplitudes)
        config = _smart_worker_config(directory, arecord_path)
        events = []
        commands = []
        model = _SequenceModel(["第一句", "第二句"])

        def popen(argv, **kwargs):
            commands.append((list(argv), dict(kwargs)))
            return subprocess.Popen(argv, **kwargs)

        worker = WhisperAsrWorker(
            config,
            cuda_probe=lambda: True,
            model_loader=lambda _path: model,
            popen_factory=popen,
            event_callback=events.append,
            text_converter=lambda text: text,
        )
        try:
            worker.heartbeat()
            started = worker.start_smart_listening({"session_id": "smart-1"})
            assert started["smart_listening"] is True
            assert started["session_id"] == "smart-1"
            _wait_for(
                lambda: len([event for event in events
                             if event.get("event") == "smart_transcript"]) >= 2,
                description="two smart transcripts")
            transcripts = [
                event["result"] for event in events
                if event.get("event") == "smart_transcript"]
            assert [item["transcript"] for item in transcripts[:2]] == [
                "第一句", "第二句"]
            assert [item["utterance_index"] for item in transcripts[:2]] == [1, 2]
            assert all(item["session_id"] == "smart-1" for item in transcripts[:2])
            assert all(item["audio_duration_s"] >= 0.06 for item in transcripts[:2])
            assert all(item["rms"] >= config["smart_vad_rms"]
                       for item in transcripts[:2])
            assert all(item["asr_latency_ms"] >= 0.0 for item in transcripts[:2])
            assert model.max_active == 1
            status = worker.status()
            assert status["utterance_count"] == 2
            assert status["pending_utterances"] == 0

            argv, kwargs = commands[0]
            assert argv == [
                arecord_path, "--quiet", "--device", "plughw:offline,0",
                "--file-type", "raw", "--format", "S16_LE",
                "--channels", "1", "--rate", "16000",
            ]
            assert kwargs["stdout"] is subprocess.PIPE
            assert kwargs["shell"] is False
            process = worker._smart_proc
            stopped = worker.stop_smart_listening({"session_id": "smart-1"})
            assert stopped["smart_listening"] is False
            assert stopped["stopped_session_id"] == "smart-1"
            assert process.poll() is not None
        finally:
            worker.shutdown()


def test_smart_vad_does_not_reject_quiet_speech_after_trailing_silence():
    # 340/32768 is just over the 0.01 active-frame threshold.  Once the
    # pre-roll and endpoint silence are included, aggregate RMS is below 0.01;
    # that must not cause a second rejection after speech-start already fired.
    with tempfile.TemporaryDirectory() as directory:
        arecord_path = os.path.join(directory, "quiet_smart_arecord")
        _fake_smart_arecord(
            arecord_path, [0, 340, 340, 340, 0, 0])
        config = _smart_worker_config(directory, arecord_path)
        events = []
        worker = WhisperAsrWorker(
            config,
            cuda_probe=lambda: True,
            model_loader=lambda _path: _FakeModel(text="轻声指令"),
            event_callback=events.append,
            text_converter=lambda text: text,
        )
        try:
            worker.heartbeat()
            worker.start_smart_listening({"session_id": "quiet-smart"})
            _wait_for(
                lambda: any(event.get("event") == "smart_transcript"
                            for event in events),
                description="quiet smart transcript")
            result = [event["result"] for event in events
                      if event.get("event") == "smart_transcript"][-1]
            assert result["transcript"] == "轻声指令"
            assert result["accepted"] is True
            assert 0.0 < result["rms"] < config["smart_vad_rms"]
            worker.stop_smart_listening({"session_id": "quiet-smart"})
        finally:
            worker.shutdown()


def test_smart_stop_invalidates_inflight_transcript():
    class BlockingModel(_FakeModel):
        def __init__(self):
            super().__init__(text="停用后不得发布")
            self.started = threading.Event()
            self.release = threading.Event()

        def transcribe(self, audio, **kwargs):
            self.calls.append((audio, kwargs))
            self.started.set()
            self.release.wait(3.0)
            return {"text": self.text}

    with tempfile.TemporaryDirectory() as directory:
        arecord_path = os.path.join(directory, "fake_smart_arecord")
        _fake_smart_arecord(
            arecord_path, [1200, 1200, 1200, 0, 0])
        config = _smart_worker_config(directory, arecord_path)
        events = []
        model = BlockingModel()
        worker = WhisperAsrWorker(
            config,
            cuda_probe=lambda: True,
            model_loader=lambda _path: model,
            event_callback=events.append,
            text_converter=lambda text: text,
        )
        try:
            worker.heartbeat()
            worker.start_smart_listening({"session_id": "cancel-smart"})
            assert model.started.wait(2.0)
            worker.stop_smart_listening({"session_id": "cancel-smart"})
            model.release.set()
            _wait_for(
                lambda: not worker.status()["recognizing"],
                description="cancelled smart inference")
            assert not any(event.get("event") == "smart_transcript"
                           for event in events)
            stopped_events = [
                event["result"] for event in events
                if event.get("event") == "smart_stopped"]
            assert stopped_events[-1]["session_id"] == "cancel-smart"
            assert stopped_events[-1]["state"] == "SMART_STOPPED"
            assert stopped_events[-1]["smart_listening"] is False
            status = worker.status()
            assert status["smart_listening"] is False
            assert status["pending_utterances"] == 0
            assert status["state"] == "SMART_STOPPED"
        finally:
            model.release.set()
            worker.shutdown()


def test_blocked_transcript_callback_cannot_block_stop_or_recorder_cleanup():
    callback_started = threading.Event()
    release_callback = threading.Event()
    events = []

    def blocking_callback(event):
        if event.get("event") == "smart_transcript":
            callback_started.set()
            release_callback.wait(3.0)
        events.append(event)

    with tempfile.TemporaryDirectory() as directory:
        arecord_path = os.path.join(directory, "fake_smart_arecord")
        _fake_smart_arecord(
            arecord_path, [1200, 1200, 1200, 0, 0])
        worker = WhisperAsrWorker(
            _smart_worker_config(directory, arecord_path),
            cuda_probe=lambda: True,
            model_loader=lambda _path: _FakeModel(text="阻塞输出"),
            event_callback=blocking_callback,
            text_converter=lambda text: text,
        )
        try:
            worker.heartbeat()
            worker.start_smart_listening({"session_id": "blocked-output"})
            assert callback_started.wait(2.0)
            with worker._lock:
                process = worker._smart_proc

            started_at = time.monotonic()
            stopped = worker.stop_smart_listening(
                {"session_id": "blocked-output"})
            elapsed = time.monotonic() - started_at

            assert elapsed < 1.0
            assert stopped["smart_listening"] is False
            assert process.poll() is not None
        finally:
            release_callback.set()
            worker.shutdown()


def test_manual_recording_and_smart_listening_are_mutually_exclusive():
    with tempfile.TemporaryDirectory() as directory:
        smart_arecord = os.path.join(directory, "fake_smart_arecord")
        _fake_smart_arecord(smart_arecord, [0])
        smart_worker = WhisperAsrWorker(
            _smart_worker_config(directory, smart_arecord),
            cuda_probe=lambda: True,
            model_loader=lambda _path: _FakeModel(),
            text_converter=lambda text: text,
        )
        try:
            smart_worker.heartbeat()
            smart_worker.start_smart_listening({"session_id": "exclusive"})
            try:
                smart_worker.start_recording({"capture_id": "manual"})
                assert False, "manual recording must be rejected"
            except _WORKER_MODULE.WorkerRequestError as exc:
                assert exc.code == "SMART_LISTENING"
            smart_worker.stop_smart_listening({"session_id": "exclusive"})
        finally:
            smart_worker.shutdown()

    with tempfile.TemporaryDirectory() as directory:
        manual_arecord = os.path.join(directory, "fake_manual_arecord")
        _fake_arecord(manual_arecord, amplitude=1000)
        manual_worker = WhisperAsrWorker(
            _smart_worker_config(directory, manual_arecord),
            cuda_probe=lambda: True,
            model_loader=lambda _path: _FakeModel(),
            text_converter=lambda text: text,
        )
        try:
            manual_worker.heartbeat()
            manual_worker.start_recording({"capture_id": "manual"})
            try:
                manual_worker.start_smart_listening({"session_id": "smart"})
                assert False, "smart listening must be rejected"
            except _WORKER_MODULE.WorkerRequestError as exc:
                assert exc.code == "MANUAL_RECORDING_ACTIVE"
            manual_worker.cancel()
        finally:
            manual_worker.shutdown()


def test_smart_queue_is_bounded_while_model_is_busy():
    class BlockingModel(_FakeModel):
        def __init__(self):
            super().__init__(text="不会发布")
            self.started = threading.Event()
            self.release = threading.Event()

        def transcribe(self, audio, **kwargs):
            self.calls.append((audio, kwargs))
            self.started.set()
            self.release.wait(5.0)
            return {"text": self.text}

    utterance = [1200, 1200, 1200, 0, 0, 0]
    with tempfile.TemporaryDirectory() as directory:
        arecord_path = os.path.join(directory, "fake_smart_arecord")
        _fake_smart_arecord(arecord_path, utterance * 10, delay_s=0.001)
        config = _smart_worker_config(directory, arecord_path, queue_limit=2)
        events = []
        model = BlockingModel()
        worker = WhisperAsrWorker(
            config,
            cuda_probe=lambda: True,
            model_loader=lambda _path: model,
            event_callback=events.append,
            text_converter=lambda text: text,
        )
        try:
            worker.heartbeat()
            worker.start_smart_listening({"session_id": "bounded"})
            assert model.started.wait(2.0)
            _wait_for(
                lambda: worker.status()["dropped_utterances"] >= 2,
                description="bounded queue drops")
            status = worker.status()
            assert status["pending_utterances"] <= 2
            assert status["utterance_count"] >= 5
            worker.stop_smart_listening({"session_id": "bounded"})
            model.release.set()
            _wait_for(lambda: not worker.status()["recognizing"],
                      description="bounded inference cancellation")
            assert worker.status()["pending_utterances"] == 0
            assert any(event.get("event") == "smart_queue_drop"
                       for event in events)
            assert not any(event.get("event") == "smart_transcript"
                           for event in events)
        finally:
            model.release.set()
            worker.shutdown()


def test_smart_cancel_invalidates_inflight_transcript_and_closes_recorder():
    class BlockingModel(_FakeModel):
        def __init__(self):
            super().__init__(text="cancel 后不得发布")
            self.started = threading.Event()
            self.release = threading.Event()

        def transcribe(self, audio, **kwargs):
            self.started.set()
            self.release.wait(3.0)
            return {"text": self.text}

    with tempfile.TemporaryDirectory() as directory:
        arecord_path = os.path.join(directory, "fake_smart_arecord")
        _fake_smart_arecord(arecord_path, [1200, 1200, 1200, 0, 0])
        events = []
        model = BlockingModel()
        worker = WhisperAsrWorker(
            _smart_worker_config(directory, arecord_path),
            cuda_probe=lambda: True,
            model_loader=lambda _path: model,
            event_callback=events.append,
            text_converter=lambda text: text,
        )
        try:
            worker.heartbeat()
            worker.start_smart_listening({"session_id": "cancel-method"})
            assert model.started.wait(2.0)
            with worker._lock:
                process = worker._smart_proc
            worker.cancel({"unload_model": True})
            assert process.poll() is not None
            model.release.set()
            _wait_for(lambda: not worker.status()["recognizing"],
                      description="smart cancel completion")
            assert not any(event.get("event") == "smart_transcript"
                           for event in events)
            terminal = [event["result"] for event in events
                        if event.get("event") == "smart_stopped"][-1]
            assert terminal["session_id"] == "cancel-method"
            assert terminal["state"] == "CANCELLED"
            assert worker.status()["model_loaded"] is False
        finally:
            model.release.set()
            worker.shutdown()


def test_smart_capture_failure_emits_session_scoped_error_event():
    with tempfile.TemporaryDirectory() as directory:
        arecord_path = os.path.join(directory, "exiting_arecord")
        _write_executable(
            arecord_path,
            "#!%s\nimport sys\nraise SystemExit(7)\n" % sys.executable)
        events = []
        worker = WhisperAsrWorker(
            _smart_worker_config(directory, arecord_path),
            cuda_probe=lambda: True,
            model_loader=lambda _path: _FakeModel(),
            event_callback=events.append,
            text_converter=lambda text: text,
        )
        try:
            worker.heartbeat()
            started = worker.start_smart_listening({"session_id": "capture-error"})
            generation = worker._smart_generation
            assert started["smart_listening"] is True
            _wait_for(
                lambda: any(event.get("event") == "smart_error"
                            for event in events),
                description="session-scoped smart capture error")
            failure = [event["result"] for event in events
                       if event.get("event") == "smart_error"][-1]
            assert failure["session_id"] == "capture-error"
            assert failure["generation"] == generation
            assert failure["state"] == "ERROR"
            assert failure["smart_listening"] is False
            assert failure["error"]
            assert worker.status()["smart_listening"] is False
        finally:
            worker.shutdown()


def test_smart_thread_start_failure_closes_exact_recorder():
    for failing_thread_name in ("asr-smart-stderr", "asr-smart-capture"):
        with tempfile.TemporaryDirectory() as directory:
            arecord_path = os.path.join(directory, "fake_smart_arecord")
            _fake_smart_arecord(arecord_path, [0])
            processes = []

            def popen(argv, **kwargs):
                process = subprocess.Popen(argv, **kwargs)
                processes.append(process)
                return process

            worker = WhisperAsrWorker(
                _smart_worker_config(directory, arecord_path),
                cuda_probe=lambda: True,
                model_loader=lambda _path: _FakeModel(),
                popen_factory=popen,
                text_converter=lambda text: text,
            )
            original_start = threading.Thread.start

            def fail_selected_start(thread):
                if thread.name == failing_thread_name:
                    raise RuntimeError("synthetic thread start failure")
                return original_start(thread)

            try:
                worker.heartbeat()
                with mock.patch.object(
                        threading.Thread, "start", new=fail_selected_start):
                    try:
                        worker.start_smart_listening(
                            {"session_id": failing_thread_name})
                        assert False, "thread start failure must reject start"
                    except _WORKER_MODULE.WorkerRequestError as exc:
                        assert exc.code == "THREAD_START_FAILED"
                assert len(processes) == 1
                assert processes[0].poll() is not None
                status = worker.status()
                assert status["smart_listening"] is False
                assert status["state"] == "ERROR"
            finally:
                worker.shutdown()


def test_smart_inference_failure_emits_session_scoped_error_event():
    class FailingModel(_FakeModel):
        def transcribe(self, audio, **kwargs):
            raise RuntimeError("synthetic large-v3 failure")

    with tempfile.TemporaryDirectory() as directory:
        arecord_path = os.path.join(directory, "fake_smart_arecord")
        _fake_smart_arecord(arecord_path, [1200, 1200, 1200, 0, 0])
        events = []
        worker = WhisperAsrWorker(
            _smart_worker_config(directory, arecord_path),
            cuda_probe=lambda: True,
            model_loader=lambda _path: FailingModel(),
            event_callback=events.append,
            text_converter=lambda text: text,
        )
        try:
            worker.heartbeat()
            worker.start_smart_listening({"session_id": "inference-error"})
            _wait_for(
                lambda: any(event.get("event") == "smart_error"
                            for event in events),
                description="session-scoped smart inference error")
            failure = [event["result"] for event in events
                       if event.get("event") == "smart_error"][-1]
            assert failure["session_id"] == "inference-error"
            assert failure["state"] == "ERROR"
            assert "synthetic" in failure["error"]
            assert not any(event.get("event") == "smart_transcript"
                           for event in events)
            assert worker.status()["smart_listening"] is False
        finally:
            worker.shutdown()


def test_smart_heartbeat_timeout_closes_exact_recorder_and_discards_queue():
    with tempfile.TemporaryDirectory() as directory:
        arecord_path = os.path.join(directory, "fake_smart_arecord")
        _fake_smart_arecord(arecord_path, [0])
        config = _smart_worker_config(directory, arecord_path)
        events = []
        worker = WhisperAsrWorker(
            config,
            cuda_probe=lambda: True,
            model_loader=lambda _path: _FakeModel(),
            event_callback=events.append,
            text_converter=lambda text: text,
        )
        try:
            worker.heartbeat()
            worker.start_smart_listening({"session_id": "heartbeat"})
            with worker._lock:
                process = worker._smart_proc
                worker._last_heartbeat = time.monotonic() - 4.0
            worker._watchdog_tick(time.monotonic())
            status = worker.status()
            assert status["smart_listening"] is False
            assert status["pending_utterances"] == 0
            assert status["state"] == "CANCELLED"
            assert "heartbeat" in status["error"]
            assert process.poll() is not None
            assert not any(event.get("event") == "smart_transcript"
                           for event in events)
            terminal = [event["result"] for event in events
                        if event.get("event") == "smart_stopped"][-1]
            assert terminal["session_id"] == "heartbeat"
            assert terminal["state"] == "CANCELLED"
        finally:
            worker.shutdown()


def test_smart_heartbeat_timeout_discards_inflight_transcript():
    class BlockingModel(_FakeModel):
        def __init__(self):
            super().__init__(text="心跳失联后不得发布")
            self.started = threading.Event()
            self.release = threading.Event()

        def transcribe(self, audio, **kwargs):
            self.started.set()
            self.release.wait(3.0)
            return {"text": self.text}

    with tempfile.TemporaryDirectory() as directory:
        arecord_path = os.path.join(directory, "fake_smart_arecord")
        _fake_smart_arecord(arecord_path, [1200, 1200, 1200, 0, 0])
        events = []
        model = BlockingModel()
        worker = WhisperAsrWorker(
            _smart_worker_config(directory, arecord_path),
            cuda_probe=lambda: True,
            model_loader=lambda _path: model,
            event_callback=events.append,
            text_converter=lambda text: text,
        )
        try:
            worker.heartbeat()
            worker.start_smart_listening({"session_id": "heartbeat-inflight"})
            assert model.started.wait(2.0)
            with worker._lock:
                worker._last_heartbeat = time.monotonic() - 4.0
            worker._watchdog_tick(time.monotonic())
            model.release.set()
            _wait_for(lambda: not worker.status()["recognizing"],
                      description="heartbeat-invalidated inference")
            assert not any(event.get("event") == "smart_transcript"
                           for event in events)
            terminal = [event["result"] for event in events
                        if event.get("event") == "smart_stopped"][-1]
            assert terminal["session_id"] == "heartbeat-inflight"
            assert terminal["state"] == "CANCELLED"
        finally:
            model.release.set()
            worker.shutdown()


def test_stale_heartbeat_watchdog_closes_exact_recorder_and_invalidates_capture():
    with tempfile.TemporaryDirectory() as directory:
        arecord_path = os.path.join(directory, "fake_arecord")
        _fake_arecord(arecord_path, amplitude=1000)
        config = _worker_config(directory, arecord_path)
        worker = WhisperAsrWorker(
            config,
            cuda_probe=lambda: True,
            model_loader=lambda _path: _FakeModel(),
            text_converter=lambda text: text,
        )
        try:
            worker.heartbeat()
            worker.start_recording({"capture_id": "watchdog"})
            with worker._lock:
                process = worker._record_proc
                worker._last_heartbeat = time.monotonic() - 4.0
            worker._watchdog_tick(time.monotonic())
            status = worker.status()
            assert status["recording"] is False
            assert status["state"] == "CANCELLED"
            assert "heartbeat" in status["error"]
            assert process.poll() is not None
            try:
                worker.stop_recording({"capture_id": "watchdog"})
                assert False, "invalidated capture must not be transcribed"
            except _WORKER_MODULE.WorkerRequestError as exc:
                assert exc.code == "NOT_RECORDING"
        finally:
            worker.shutdown()


def test_cuda_failure_is_explicit_and_never_falls_back_to_cpu():
    with tempfile.TemporaryDirectory() as directory:
        arecord_path = os.path.join(directory, "fake_arecord")
        _fake_arecord(arecord_path, amplitude=1000)
        config = _worker_config(directory, arecord_path)

        def no_cuda():
            raise RuntimeError("CUDA unavailable in offline test")

        worker = WhisperAsrWorker(config, cuda_probe=no_cuda)
        try:
            status = worker.status()
            assert status["ready"] is False
            assert status["device"] == "cuda"
            assert status["model_loaded"] is False
            assert "CUDA" in status["error"]
        finally:
            worker.shutdown()


def test_empty_and_auto_input_select_usb_capture_without_opening_microphone():
    listing = textwrap.dedent("""\
        **** CAPTURE Hardware Devices ****
        card 1: APE [NVIDIA Jetson AGX Orin APE], device 0: tegra-dlink-0 XBAR-ADMAIF1-0 []
        card 2: Audio [AB17X USB Audio], device 0: USB Audio [USB Audio]
    """)
    devices = parse_alsa_capture_devices(listing)
    assert [device["card_id"] for device in devices] == ["APE", "Audio"]

    with tempfile.TemporaryDirectory() as directory:
        arecord_path = os.path.join(directory, "fake_arecord")
        _fake_arecord(arecord_path, amplitude=1000)
        for input_device in ("", "auto", "AUTO"):
            config = _worker_config(directory, arecord_path)
            config["input_device"] = input_device
            worker = WhisperAsrWorker(
                config,
                cuda_probe=lambda: True,
                model_loader=lambda _path: _FakeModel(),
                text_converter=lambda text: text,
                capture_device_lister=lambda _path: devices,
            )
            try:
                status = worker.status()
                assert status["ready"] is True, status
                assert status["input_device"] == "plughw:CARD=Audio,DEV=0"
            finally:
                worker.shutdown()


def test_auto_input_rejects_virtual_or_missing_capture_endpoints():
    virtual_devices = [{
        "card_index": 1,
        "card_id": "APE",
        "card_name": "NVIDIA Jetson AGX Orin APE",
        "device_index": 0,
        "device_name": "tegra-dlink-0 XBAR-ADMAIF1-0",
    }]
    with tempfile.TemporaryDirectory() as directory:
        arecord_path = os.path.join(directory, "fake_arecord")
        _fake_arecord(arecord_path, amplitude=1000)
        for devices in ([], virtual_devices):
            config = _worker_config(directory, arecord_path)
            config["input_device"] = "auto"
            worker = WhisperAsrWorker(
                config,
                cuda_probe=lambda: True,
                capture_device_lister=lambda _path, value=devices: value,
            )
            try:
                status = worker.status()
                assert status["ready"] is False
                assert "no usable physical ALSA capture device" in status["error"]
            finally:
                worker.shutdown()


def test_explicit_null_monitor_and_ape_capture_devices_are_rejected():
    with tempfile.TemporaryDirectory() as directory:
        arecord_path = os.path.join(directory, "fake_arecord")
        _fake_arecord(arecord_path, amplitude=1000)
        for input_device in (
                "default", "auto_null", "auto_null.monitor",
                "hw:APE,0", "plughw:CARD=APE,DEV=1"):
            config = _worker_config(directory, arecord_path)
            config["input_device"] = input_device
            worker = WhisperAsrWorker(config, cuda_probe=lambda: True)
            try:
                status = worker.status()
                assert status["ready"] is False, input_device
                assert "input_device" in status["error"], status
            finally:
                worker.shutdown()


def test_missing_opencc_t2s_converter_makes_worker_unavailable():
    with tempfile.TemporaryDirectory() as directory:
        arecord_path = os.path.join(directory, "fake_arecord")
        _fake_arecord(arecord_path, amplitude=1000)
        config = _worker_config(directory, arecord_path)
        with mock.patch.object(
                _WORKER_MODULE, "_default_text_converter",
                side_effect=RuntimeError("OpenCC unavailable")):
            worker = WhisperAsrWorker(
                config,
                cuda_probe=lambda: True,
                model_loader=lambda _path: _FakeModel(),
            )
        try:
            status = worker.status()
            assert status["ready"] is False
            assert "OpenCC" in status["error"]
        finally:
            worker.shutdown()


def test_model_cuda_rate_duration_and_hash_contract_is_fail_closed():
    with tempfile.TemporaryDirectory() as directory:
        arecord_path = os.path.join(directory, "fake_arecord")
        _fake_arecord(arecord_path, amplitude=1000)
        bad_values = (
            ("model", "tiny"),
            ("device", "cpu"),
            ("sample_rate", 44100),
            ("max_record_seconds", 30.1),
            ("model_sha256", "0" * 64),
        )
        for name, value in bad_values:
            config = _worker_config(directory, arecord_path)
            config[name] = value
            worker = WhisperAsrWorker(
                config,
                cuda_probe=lambda: True,
                model_loader=lambda _path: _FakeModel(),
                text_converter=lambda text: text,
            )
            try:
                status = worker.status()
                assert status["ready"] is False, (name, status)
                assert status["model_loaded"] is False
            finally:
                worker.shutdown()


def test_small_medium_and_large_have_distinct_dimension_contracts():
    expected = {
        "small": (80, 768, 12, 768, 12),
        "medium": (80, 1024, 24, 1024, 24),
        "large": (128, 1280, 32, 1280, 32),
    }
    with tempfile.TemporaryDirectory() as directory:
        arecord_path = os.path.join(directory, "fake_arecord")
        _fake_arecord(arecord_path, amplitude=1000)
        for model_name, values in expected.items():
            dims = SimpleNamespace(
                n_mels=values[0], n_audio_state=values[1],
                n_audio_layer=values[2], n_text_state=values[3],
                n_text_layer=values[4])
            config = _worker_config(directory, arecord_path, model=model_name)
            worker = WhisperAsrWorker(
                config,
                cuda_probe=lambda: True,
                model_loader=lambda _path, value=dims: _FakeModel(dims=value),
                text_converter=lambda text: text,
            )
            try:
                assert worker.status()["ready"] is True
                assert worker.status()["model"] == model_name
                assert worker._load_model().dims is dims
            finally:
                worker.shutdown()


def test_cancel_can_release_a_loaded_model_when_inference_is_idle():
    with tempfile.TemporaryDirectory() as directory:
        arecord_path = os.path.join(directory, "fake_arecord")
        _fake_arecord(arecord_path, amplitude=1000)
        config = _worker_config(directory, arecord_path)
        worker = WhisperAsrWorker(
            config, cuda_probe=lambda: True,
            model_loader=lambda _path: _FakeModel(),
            text_converter=lambda text: text)
        try:
            with worker._lock:
                worker._model = _FakeModel()
            assert worker.status()["model_loaded"] is True
            status = worker.cancel({"unload_model": True})
            assert status["model_loaded"] is False
        finally:
            worker.shutdown()


def test_cancel_during_inference_discards_result_then_unloads_model():
    class BlockingModel(_FakeModel):
        def __init__(self):
            super().__init__(text="不得返回")
            self.started = threading.Event()
            self.release = threading.Event()

        def transcribe(self, audio, **kwargs):
            self.calls.append((audio, kwargs))
            self.started.set()
            self.release.wait(2.0)
            return {"text": self.text}

    with tempfile.TemporaryDirectory() as directory:
        arecord_path = os.path.join(directory, "fake_arecord")
        _fake_arecord(arecord_path, amplitude=1000)
        config = _worker_config(directory, arecord_path)
        model = BlockingModel()
        worker = WhisperAsrWorker(
            config,
            cuda_probe=lambda: True,
            model_loader=lambda _path: model,
            text_converter=lambda text: text,
        )
        outcome = {}
        try:
            worker.heartbeat()
            worker.start_recording({"capture_id": "cancel-inference"})
            time.sleep(0.10)

            def stop_request():
                try:
                    outcome["result"] = worker.stop_recording(
                        {"capture_id": "cancel-inference"})
                except Exception as exc:
                    outcome["error"] = exc

            thread = threading.Thread(target=stop_request)
            thread.start()
            assert model.started.wait(2.0)
            cancelling = worker.cancel({"unload_model": True})
            assert cancelling["state"] == "CANCELLING"
            assert cancelling["recognizing"] is True
            model.release.set()
            thread.join(timeout=5.0)
            assert not thread.is_alive()
            assert "result" not in outcome
            assert outcome["error"].code == "CANCELLED"
            status = worker.status()
            assert status["state"] == "CANCELLED"
            assert status["model_loaded"] is False
            assert status["recording"] is False
        finally:
            model.release.set()
            worker.shutdown()


def _fake_json_worker(path):
    body = textwrap.dedent("""\
        import json
        import os
        import sys
        import threading
        import time

        lock = threading.Lock()
        secret_names = [
            "SWEEPER_AI_SESSION_TOKEN", "DEEPSEEK_API_KEY",
            "SWEEPER_ASR_CAPABILITY", "SWEEPER_ASR_MODEL_PATH",
            "SWEEPER_ASR_MODEL_DIR",
            "SWEEPER_MCP_CONTROL_TOKEN", "ROS_MASTER_URI", "ROS_IP",
            "ROS_PACKAGE_PATH",
        ]
        def write(frame):
            with lock:
                sys.stdout.write(json.dumps(frame) + "\\n")
                sys.stdout.flush()
        def delayed(request_id):
            time.sleep(0.35)
            write({"id": request_id, "ok": True, "result": {
                "capture_id": "parallel", "transcript": "done",
                "audio_duration_s": 0.5, "rms": 0.1,
                "asr_latency_ms": 350.0, "model_loaded": True}})
        for line in sys.stdin:
            request = json.loads(line)
            request_id = request["id"]
            method = request["method"]
            if method == "status":
                write({"id": request_id, "ok": True, "result": {
                    "ready": True, "state": "IDLE", "model": "medium",
                    "device": "cuda", "model_loaded": False,
                    "recording": False, "recognizing": False,
                    "audio_duration_s": 0.0, "asr_latency_ms": 0.0,
                    "error": "", "leaked": [name for name in secret_names
                    if name in os.environ]}})
            elif method == "stop_recording":
                threading.Thread(target=delayed, args=(request_id,), daemon=True).start()
            elif method == "heartbeat":
                write({"id": request_id, "ok": True, "result": {"alive": True}})
            elif method == "shutdown":
                write({"id": request_id, "ok": True, "result": {"state": "SHUTDOWN"}})
                break
            else:
                write({"id": request_id, "ok": True, "result": {"ok": True}})
    """)
    with open(path, "w", encoding="utf-8") as stream:
        stream.write(body)


def test_client_strips_control_and_ros_environment_and_routes_parallel_requests():
    with tempfile.TemporaryDirectory() as directory:
        worker_path = os.path.join(directory, "fake_worker.py")
        _fake_json_worker(worker_path)
        sensitive = {
            "SWEEPER_AI_SESSION_TOKEN": "ui-secret",
            "DEEPSEEK_API_KEY": "cloud-secret",
            "SWEEPER_ASR_CAPABILITY": "asr-secret",
            "SWEEPER_ASR_MODEL_PATH": "/secret/model/path",
            "SWEEPER_ASR_MODEL_DIR": "/secret/model/directory",
            "SWEEPER_MCP_CONTROL_TOKEN": "control-secret",
            "ROS_MASTER_URI": "http://robot:11311",
            "ROS_IP": "192.168.10.50",
            "ROS_PACKAGE_PATH": "/robot/packages",
        }
        events = []
        with mock.patch.dict(os.environ, sensitive, clear=False):
            client = AsrClient(sys.executable, worker_path, {}, events.append)
            try:
                status = client.start()
                assert status["leaked"] == []
                output = {}

                def long_request():
                    output["result"] = client.request(
                        "stop_recording", timeout=2.0)

                thread = threading.Thread(target=long_request)
                thread.start()
                time.sleep(0.05)
                started = time.monotonic()
                parallel_status = client.request("status", timeout=1.0)
                elapsed = time.monotonic() - started
                assert parallel_status["ready"] is True
                assert elapsed < 0.25
                thread.join(timeout=2.0)
                assert output["result"]["transcript"] == "done"
            finally:
                client.close()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not any(
                event.get("event") == "status" and
                event.get("status", {}).get("state") == "UNAVAILABLE"
                for event in events):
            time.sleep(0.01)
        unavailable = [
            event for event in events
            if event.get("event") == "status" and
            event.get("status", {}).get("state") == "UNAVAILABLE"]
        assert unavailable
        assert unavailable[-1]["status"]["ready"] is False
        assert unavailable[-1]["status"]["recording"] is False
        assert unavailable[-1]["status"]["smart_listening"] is False
        assert unavailable[-1]["status"]["pending_utterances"] == 0


def test_json_server_dispatches_smart_listening_ipc_methods():
    class FakeWorker(object):
        def __init__(self):
            self.callback = None
            self.calls = []

        def set_event_callback(self, callback):
            self.callback = callback

        def start_smart_listening(self, params):
            self.calls.append(("start", dict(params)))
            return {
                "smart_listening": True,
                "session_id": params["session_id"],
            }

        def stop_smart_listening(self, params):
            self.calls.append(("stop", dict(params)))
            return {
                "smart_listening": False,
                "session_id": params["session_id"],
            }

        def shutdown(self):
            return {"state": "SHUTDOWN"}

    import io
    worker = FakeWorker()
    input_stream = io.StringIO(
        json.dumps({
            "id": 1, "method": "start_smart_listening",
            "params": {"session_id": "ipc-smart"},
        }) + "\n" +
        json.dumps({
            "id": 2, "method": "stop_smart_listening",
            "params": {"session_id": "ipc-smart"},
        }) + "\n")
    output_stream = io.StringIO()
    JsonLineServer(
        worker, input_stream=input_stream,
        output_stream=output_stream).serve()
    frames = [json.loads(line) for line in output_stream.getvalue().splitlines()]
    assert [frame["id"] for frame in frames] == [1, 2]
    assert all(frame["ok"] for frame in frames)
    assert frames[0]["result"]["smart_listening"] is True
    assert frames[1]["result"]["smart_listening"] is False
    assert worker.calls == [
        ("start", {"session_id": "ipc-smart"}),
        ("stop", {"session_id": "ipc-smart"}),
    ]


def test_json_server_treats_parent_stdin_eof_as_shutdown():
    class FakeWorker(object):
        def __init__(self):
            self.shutdown_called = False
            self.callback = None

        def set_event_callback(self, callback):
            self.callback = callback

        def shutdown(self):
            self.shutdown_called = True
            return {"state": "SHUTDOWN"}

    import io
    worker = FakeWorker()
    server = JsonLineServer(
        worker, input_stream=io.StringIO(""), output_stream=io.StringIO())
    server.serve()
    assert worker.shutdown_called is True


def test_json_server_broken_stdout_still_shuts_down_exact_worker():
    class FakeWorker(object):
        def __init__(self):
            self.shutdown_calls = 0
            self.callback = None

        def set_event_callback(self, callback):
            self.callback = callback

        def status(self):
            return {"ready": True}

        def shutdown(self):
            self.shutdown_calls += 1
            return {"state": "SHUTDOWN"}

    class BrokenOutput(object):
        def write(self, _value):
            raise BrokenPipeError("parent stdout closed")

        def flush(self):
            pass

    import io
    worker = FakeWorker()
    request = json.dumps({"id": 1, "method": "status"}) + "\n"
    server = JsonLineServer(
        worker, input_stream=io.StringIO(request), output_stream=BrokenOutput())
    server.serve()
    assert worker.shutdown_calls == 1


def test_json_server_waits_for_concurrent_broken_pipe_shutdown_completion():
    class BlockingShutdownWorker(object):
        def __init__(self):
            self.shutdown_calls = 0
            self.shutdown_started = threading.Event()
            self.shutdown_release = threading.Event()
            self.callback = None

        def set_event_callback(self, callback):
            self.callback = callback

        def status(self):
            return {"ready": True}

        def shutdown(self):
            self.shutdown_calls += 1
            self.shutdown_started.set()
            self.shutdown_release.wait(3.0)
            return {"state": "SHUTDOWN"}

    class BrokenOutput(object):
        def write(self, _value):
            raise BrokenPipeError("parent stdout closed")

        def flush(self):
            pass

    import io
    worker = BlockingShutdownWorker()
    request = json.dumps({"id": 1, "method": "status"}) + "\n"
    server = JsonLineServer(
        worker, input_stream=io.StringIO(request), output_stream=BrokenOutput())
    serve_thread = threading.Thread(target=server.serve)
    serve_thread.start()
    try:
        assert worker.shutdown_started.wait(1.0)
        time.sleep(0.05)
        assert serve_thread.is_alive(), (
            "serve() returned before the exact worker shutdown completed")
    finally:
        worker.shutdown_release.set()
        serve_thread.join(2.0)
    assert not serve_thread.is_alive()
    assert worker.shutdown_calls == 1


if __name__ == "__main__":
    tests = [value for name, value in sorted(globals().items())
             if name.startswith("test_")]
    for test in tests:
        test()
        print("PASS", test.__name__)
    print("%d ASR worker tests passed" % len(tests))
