#!/usr/bin/env python3
"""Stop only this session's PID namespace when its controlling SSH pipe ends."""
import json
import os
from pathlib import Path
import selectors
import signal
import subprocess
import sys
import time

root = Path(sys.argv[1]).resolve()
if root.parent != Path('/map/conbine_navigation') or not root.name.startswith('session-'):
    raise RuntimeError('Unexpected session root')
stopping = False
def stop(*_):
    global stopping
    stopping = True
for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
    signal.signal(sig, stop)
child = subprocess.Popen(['unshare', '--mount', '--pid', '--fork', '--kill-child=SIGTERM',
                          'bash', str(root/'namespace.sh'), str(root)], start_new_session=True)
(root/'guardian.json').write_text(json.dumps({'pid': os.getpid(), 'namespace_pid': child.pid}))
sel = selectors.DefaultSelector()
sel.register(sys.stdin.buffer, selectors.EVENT_READ)
last = time.monotonic()
try:
    while not stopping and child.poll() is None:
        if sel.select(.25):
            data = os.read(sys.stdin.fileno(), 4096)
            if not data:
                break
            last = time.monotonic()
        if time.monotonic()-last > 8:
            break
finally:
    sel.close()
    for sig, timeout in ((signal.SIGINT, 12), (signal.SIGTERM, 5), (signal.SIGKILL, 3)):
        if child.poll() is not None:
            break
        os.killpg(child.pid, sig)
        try:
            child.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            pass
    (root/'guardian_exit.json').write_text(json.dumps({'returncode': child.poll(), 'ended': time.time()}))
