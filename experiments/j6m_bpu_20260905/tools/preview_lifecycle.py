"""Bounded diagnostics and ownership checks for display-only preview sessions."""
import json
import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path
import re
import threading
from xmlrpc.client import ServerProxy, Transport


def within_lifetime(started, now, seconds):
    return seconds is None or now - started < seconds


class StrictRotatingHandler(RotatingFileHandler):
    def shouldRollover(self, record):
        if self.stream is None:
            self.stream = self._open()
        size = len((self.format(record) + '\n').encode('utf-8'))
        return self.stream.tell() + size > self.maxBytes

    def handleError(self, record):
        # Disk errors must reach the caller, not silently discard diagnostics.
        raise


class BoundedLog:
    """Keep the current log and two backups, each at most four MiB."""
    def __init__(self, path, max_bytes=4 * 1024 * 1024, backups=2):
        self.path = Path(path)
        self.max_bytes = max_bytes
        for candidate in [self.path] + [Path(str(self.path) + '.' + str(i))
                                       for i in range(1, backups + 1)]:
            if candidate.is_symlink():
                raise ValueError('Refuse symlinked preview log')
        self.handler = StrictRotatingHandler(str(self.path), maxBytes=max_bytes,
                                             backupCount=backups, encoding='utf-8')
        self.handler.setFormatter(logging.Formatter('%(message)s'))

    def write(self, value):
        value = value.rstrip('\n')
        if len(value.encode('utf-8')) + 1 > self.max_bytes:
            raise ValueError('Preview log record exceeds bounded size')
        record = logging.LogRecord('bpu_preview', logging.INFO, '', 0, value, (), None)
        self.handler.handle(record)

    def flush(self):
        self.handler.flush()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.handler.close()


def drain_stderr(worker, log):
    worker.preview_stderr_error = None
    def drain():
        try:
            while True:
                chunk = worker.stderr.read(4096)
                if not chunk:
                    break
                log.write(chunk.decode('utf-8', errors='replace'))
        except Exception as error:
            worker.preview_stderr_error = repr(error)
    worker.preview_stderr_thread = threading.Thread(target=drain, daemon=True)
    worker.preview_stderr_thread.start()


class TimeoutTransport(Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 2.0
        return connection


def session_identity(master):
    """A restarted master or GUI ends this bridge; no automatic control restart."""
    pid = master.getPid('/bpu_preview_lifecycle')
    gui = master.lookupNode('/bpu_preview_lifecycle', '/autolabor_operator_gui')
    if (pid[0] != 1 or type(pid[2]) is not int or pid[2] <= 0 or
            gui[0] != 1 or not isinstance(gui[2], str) or not gui[2].startswith('http://')):
        raise RuntimeError('Navigation master/Qt session is absent')
    return pid[2], gui[2]


def master_proxy(uri):
    return ServerProxy(uri, transport=TimeoutTransport(), allow_none=True)


def request_stop(lab):
    """Only create the selected private session's marker; never signal a PID."""
    status_path = lab / 'qt_bridge_current.json'
    if not status_path.exists():
        return 'No preview session exists'
    status = json.loads(status_path.read_text())
    if not status.get('active'):
        return 'Preview is already stopped'
    run_id = status.get('run_id', '')
    if not re.fullmatch(r'qt_[0-9_]{1,60}', run_id):
        raise ValueError('Invalid preview session identifier')
    directory = lab / 'live' / run_id
    if (str(directory) != status.get('directory') or directory.resolve() != directory or
            not directory.is_dir()):
        raise ValueError('Refuse stop marker outside the private preview session')
    marker = directory / 'STOP'
    try:
        with marker.open('x') as stream:
            stream.write('operator_stop\n')
    except FileExistsError:
        if marker.is_symlink() or not marker.is_file():
            raise ValueError('Invalid preview stop marker')
    return 'Preview stop requested: ' + run_id
