"""N05 owned-process cleanup; original algorithm, new exact port pair.

No global kill. Actual source marker and PID start-time checks remain inherited.
"""
import os
import signal
import socket
import subprocess
import time
from pathlib import Path
from run_smoke_a02 import owned_processes, atomic, utc

def stop_group(proc,root):
    if proc.poll() is None:
        try:os.killpg(proc.pid,signal.SIGINT)
        except ProcessLookupError:pass
        try:proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            try:os.killpg(proc.pid,signal.SIGTERM)
            except ProcessLookupError:pass
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:os.killpg(proc.pid,signal.SIGKILL)
                except ProcessLookupError:pass
                proc.wait(timeout=5)
    # roslaunch starts nodes with setsid: descendant PGIDs differ. Exact
    # per-run ROS_LOG_DIR plus PID start ticks establishes our ownership.
    for sig,seconds in ((signal.SIGTERM,5),(signal.SIGKILL,5)):
        owned=owned_processes(root)
        if not owned:break
        for pid,pgid,ticks in owned:
            try:
                current=Path('/proc/{}/stat'.format(pid)).read_text().rsplit(')',1)[1].split()
                if current[19]==ticks:os.kill(pid,sig)
            except (FileNotFoundError,ProcessLookupError):pass
        deadline=time.monotonic()+seconds
        while owned_processes(root) and time.monotonic()<deadline:time.sleep(.1)
    survivors=owned_processes(root)
    if survivors:raise RuntimeError('owned descendants did not terminate: '+str(survivors))
    for port in (14830,14831):
        with socket.socket() as sock:
            sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            sock.bind(('127.0.0.1',port))
    atomic(root/'cleanup.json',{'utc':utc(),'owned_live_processes':[],
                              'ports_14830_14831_released':True,'scope':'exact run environment markers, not global process names'})
