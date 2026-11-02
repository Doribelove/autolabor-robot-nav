"""Stop only this preview after checking its exact script path."""
import os
from pathlib import Path
import signal

root = Path(__file__).resolve().parent
pid_file = root/'server.pid'
if pid_file.exists():
    pid = int(pid_file.read_text())
    proc = Path('/proc')/str(pid)
    if proc.exists():
        args = (proc/'cmdline').read_bytes().split(b'\0')
        if str(root/'server.py').encode() not in args:
            raise RuntimeError('Preview PID identity mismatch')
        os.kill(pid, signal.SIGTERM)
