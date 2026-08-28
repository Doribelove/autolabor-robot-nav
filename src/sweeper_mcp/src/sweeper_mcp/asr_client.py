# -*- coding: utf-8 -*-
"""Isolated JSON-lines client for the local Whisper ASR worker.

The worker deliberately receives no ROS or robot-control credentials.  This
module only manages one local child process and routes request IDs to waiting
threads; it never imports ROS and never opens a network connection.
"""

from collections import deque
import json
import os
import subprocess
import threading


_STRIPPED_ENV_KEYS = {
    "DEEPSEEK_API_KEY",
    "SWEEPER_AI_SESSION_TOKEN",
    "SWEEPER_ASR_CAPABILITY",
    "SWEEPER_MCP_CONTROL_TOKEN",
}
_STRIPPED_ENV_PREFIXES = (
    "ROS_", "SWEEPER_AI_", "SWEEPER_ASR_", "SWEEPER_MCP_")


class AsrClientError(Exception):
    """Worker transport or request error."""

    def __init__(self, code, message, data=None):
        super().__init__("%s (code=%s)" % (message, code))
        self.code = code
        self.message = message
        self.data = data


def _sanitized_worker_environment(source=None):
    """Return an environment without cloud, MCP-control, or ROS capability."""
    child_env = dict(os.environ if source is None else source)
    for name in list(child_env):
        if (name in _STRIPPED_ENV_KEYS or
                any(name.startswith(prefix)
                    for prefix in _STRIPPED_ENV_PREFIXES)):
            child_env.pop(name, None)
    return child_env


class AsrClient(object):
    """Thread-safe client for ``whisper_asr_worker.py``.

    Args:
        python_executable: Python from the isolated ASR virtual environment.
        worker_script: Absolute path to ``whisper_asr_worker.py``.
        config: Worker configuration dict.  It is serialized as one argv item,
            never interpreted by a shell.
        event_callback: Optional callable receiving unsolicited event dicts on
            the reader thread.
    """

    def __init__(self, python_executable, worker_script, config,
                 event_callback=None):
        self._python_executable = os.path.abspath(str(python_executable))
        self._worker_script = os.path.abspath(str(worker_script))
        self._config = dict(config or {})
        self._event_callback = event_callback
        self._proc = None
        self._reader = None
        self._stderr_reader = None
        self._heartbeat_thread = None
        self._heartbeat_stop = threading.Event()
        self._lifecycle_lock = threading.RLock()
        self._pending_lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._pending = {}
        self._next_id = 0
        self._closing = False
        self._stderr_tail = deque(maxlen=40)

    def start(self):
        """Start the worker, verify its status, and begin a 1 Hz heartbeat."""
        with self._lifecycle_lock:
            if self._proc is not None and self._proc.poll() is None:
                return self.request("status", timeout=self._startup_timeout())
            if not os.path.isfile(self._python_executable):
                raise AsrClientError(
                    "EXECUTABLE_MISSING",
                    "ASR Python executable does not exist: %s" %
                    self._python_executable)
            if not os.access(self._python_executable, os.X_OK):
                raise AsrClientError(
                    "EXECUTABLE_NOT_EXECUTABLE",
                    "ASR Python is not executable: %s" %
                    self._python_executable)
            if not os.path.isfile(self._worker_script):
                raise AsrClientError(
                    "WORKER_MISSING",
                    "ASR worker does not exist: %s" % self._worker_script)
            try:
                config_json = json.dumps(
                    self._config, ensure_ascii=False, separators=(",", ":"))
            except (TypeError, ValueError) as exc:
                raise AsrClientError(
                    "INVALID_CONFIG", "ASR config is not JSON serializable: %s" % exc)

            command = [
                self._python_executable,
                self._worker_script,
                "--config-json",
                config_json,
            ]
            self._closing = False
            self._heartbeat_stop.clear()
            self._stderr_tail.clear()
            try:
                self._proc = subprocess.Popen(
                    command,
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    encoding="utf-8",
                    bufsize=1,
                    env=_sanitized_worker_environment(),
                    close_fds=True,
                )
            except OSError as exc:
                self._proc = None
                raise AsrClientError(
                    "SPAWN_FAILED", "Unable to start ASR worker: %s" % exc)

            self._reader = threading.Thread(
                target=self._read_loop, name="asr-worker-reader", daemon=True)
            self._stderr_reader = threading.Thread(
                target=self._stderr_loop, name="asr-worker-stderr", daemon=True)
            self._reader.start()
            self._stderr_reader.start()

        try:
            status = self.request("status", timeout=self._startup_timeout())
            # Make the first heartbeat synchronous so start_recording can be
            # called immediately after start() returns without a race.
            self.request("heartbeat", timeout=1.0)
        except Exception:
            self.close()
            raise

        with self._lifecycle_lock:
            self._heartbeat_thread = threading.Thread(
                target=self._heartbeat_loop,
                name="asr-worker-heartbeat",
                daemon=True,
            )
            self._heartbeat_thread.start()
        return status

    def _startup_timeout(self):
        try:
            value = float(self._config.get("startup_timeout_s", 20.0))
        except (TypeError, ValueError):
            value = 20.0
        return min(120.0, max(1.0, value))

    def request(self, method, params=None, timeout=30.0):
        """Send one request and return its result dict.

        Calls from multiple threads are supported.  In particular a long
        ``stop_recording``/transcription request does not block ``status`` or
        ``cancel`` requests.
        """
        if not isinstance(method, str) or not method:
            raise AsrClientError("INVALID_METHOD", "ASR method must be non-empty")
        if params is not None and not isinstance(params, dict):
            raise AsrClientError("INVALID_PARAMS", "ASR params must be an object")
        try:
            timeout = float(timeout)
        except (TypeError, ValueError):
            raise AsrClientError("INVALID_TIMEOUT", "ASR timeout must be numeric")
        if timeout <= 0.0:
            raise AsrClientError("INVALID_TIMEOUT", "ASR timeout must be positive")

        with self._lifecycle_lock:
            process = self._proc
            if process is None or process.poll() is not None:
                raise AsrClientError(
                    "WORKER_NOT_RUNNING", self._worker_exit_message())

        with self._pending_lock:
            self._next_id += 1
            request_id = self._next_id
            entry = {
                "event": threading.Event(),
                "message": None,
                "error": None,
            }
            self._pending[request_id] = entry

        frame = {"id": request_id, "method": method}
        if params is not None:
            frame["params"] = params
        line = json.dumps(frame, ensure_ascii=False, separators=(",", ":")) + "\n"
        try:
            with self._write_lock:
                if process.poll() is not None or process.stdin is None:
                    raise BrokenPipeError("ASR worker exited")
                process.stdin.write(line)
                process.stdin.flush()
        except Exception as exc:
            with self._pending_lock:
                self._pending.pop(request_id, None)
            raise AsrClientError(
                "WRITE_FAILED", "Unable to write to ASR worker: %s" % exc)

        if not entry["event"].wait(timeout):
            with self._pending_lock:
                self._pending.pop(request_id, None)
            raise AsrClientError(
                "TIMEOUT", "ASR request timed out: %s" % method)
        with self._pending_lock:
            self._pending.pop(request_id, None)
        if entry["error"] is not None:
            raise entry["error"]
        message = entry["message"] or {}
        if not message.get("ok", False):
            error = message.get("error") or {}
            raise AsrClientError(
                error.get("code", "WORKER_ERROR"),
                error.get("message", "ASR worker request failed"),
                error.get("data"),
            )
        result = message.get("result")
        if not isinstance(result, dict):
            raise AsrClientError(
                "INVALID_RESPONSE", "ASR worker result must be an object")
        return result

    def _read_loop(self):
        process = self._proc
        try:
            for line in process.stdout:
                line = line.strip()
                if not line:
                    continue
                try:
                    message = json.loads(line)
                except ValueError:
                    self._deliver_event({
                        "event": "protocol_error",
                        "error": "ASR worker emitted non-JSON stdout",
                    })
                    continue
                if not isinstance(message, dict):
                    continue
                request_id = message.get("id")
                if request_id is None:
                    self._deliver_event(message)
                    continue
                with self._pending_lock:
                    entry = self._pending.get(request_id)
                if entry is None:
                    continue
                entry["message"] = message
                entry["event"].set()
        finally:
            exit_message = self._worker_exit_message()
            self._deliver_event({
                "event": "status",
                "status": {
                    "ready": False,
                    "state": "UNAVAILABLE",
                    "model": str(self._config.get("model", "medium")),
                    "device": "cuda",
                    "model_loaded": False,
                    "recording": False,
                    "recognizing": False,
                    "smart_listening": False,
                    "smart_session_id": "",
                    "session_id": "",
                    "pending_utterances": 0,
                    "utterance_count": 0,
                    "dropped_utterances": 0,
                    "capture_id": "",
                    "audio_duration_s": 0.0,
                    "rms": 0.0,
                    "asr_latency_ms": 0.0,
                    "error": exit_message,
                },
            })
            error = AsrClientError("WORKER_EOF", exit_message)
            with self._pending_lock:
                entries = list(self._pending.values())
            for entry in entries:
                entry["error"] = error
                entry["event"].set()

    def _stderr_loop(self):
        process = self._proc
        if process is None or process.stderr is None:
            return
        for line in process.stderr:
            value = line.rstrip()
            if value:
                self._stderr_tail.append(value)

    def _deliver_event(self, message):
        callback = self._event_callback
        if callback is None:
            return
        try:
            callback(message)
        except Exception:
            # A UI callback must never be able to stop the protocol reader.
            pass

    def _heartbeat_loop(self):
        while not self._heartbeat_stop.wait(1.0):
            try:
                self.request("heartbeat", timeout=0.75)
            except AsrClientError as exc:
                if self._heartbeat_stop.is_set() or self._closing:
                    return
                self._deliver_event({
                    "event": "heartbeat_error",
                    "error": exc.message,
                })

    def _worker_exit_message(self):
        process = self._proc
        code = process.poll() if process is not None else None
        message = "ASR worker is not running"
        if code is not None:
            message += " (exit=%s)" % code
        if self._stderr_tail:
            message += ": " + self._stderr_tail[-1]
        return message

    def close(self):
        """Close the microphone through shutdown, then reap the exact child."""
        with self._lifecycle_lock:
            process = self._proc
            if process is None:
                return
            self._closing = True
            self._heartbeat_stop.set()

        if process.poll() is None:
            try:
                self.request("shutdown", timeout=3.0)
            except AsrClientError:
                pass
        try:
            if process.stdin is not None:
                process.stdin.close()
        except Exception:
            pass
        try:
            process.wait(timeout=4.0)
        except subprocess.TimeoutExpired:
            # The target is the exact worker Popen created above.
            try:
                process.terminate()
                process.wait(timeout=2.0)
            except Exception:
                try:
                    process.kill()
                    process.wait(timeout=2.0)
                except Exception:
                    pass
        current = threading.current_thread()
        for thread in (self._reader, self._stderr_reader,
                       self._heartbeat_thread):
            if thread is not None and thread is not current:
                thread.join(timeout=1.0)
        with self._lifecycle_lock:
            self._proc = None
            self._closing = False

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.close()
