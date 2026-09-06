#!/usr/bin/env python3
"""Private persistent worker ownership; no system service or broad process kill."""
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import time

LAB=Path('/map/robot_j6m_optimized_20260905/bpu_perception_20260906')


def write_state(path,value):
    temporary=path.with_name(path.name+'.tmp.'+str(os.getpid()))
    temporary.write_text(json.dumps(value)+'\n');temporary.replace(path)


def identity(pid):
    try:
        raw=Path('/proc/%d/stat'%pid).read_text()
        start=raw[raw.rfind(')')+2:].split()[19]
        command=Path('/proc/%d/cmdline'%pid).read_bytes().split(b'\0')
        if str(LAB/'tools/inference_service.py').encode() not in command:return None
        return str(pid)+':'+start
    except (OSError,IndexError):return None


def main():
    if Path(__file__).resolve().parents[1]!=LAB:raise RuntimeError('Wrong private directory')
    operation=sys.argv[1] if len(sys.argv)>1 else 'status'
    if operation not in ('start','stop','status'):raise ValueError('Expected start|stop|status')
    run=LAB/'run';run.mkdir(parents=True,exist_ok=True);(LAB/'logs').mkdir(exist_ok=True)
    with (run/'manager.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        owner=json.loads((run/'owner.json').read_text()) if (run/'owner.json').exists() else {}
        alive=bool(owner.get('identity')) and identity(owner.get('pid',0))==owner['identity']
        if operation=='status':
            ready=json.loads((run/'ready.json').read_text()) if (run/'ready.json').exists() else {}
            print(json.dumps(dict(owner=owner,alive=alive,ready=ready)));return
        if operation=='stop':
            if not alive:return
            (run/'STOP').touch()
            for _ in range(50):
                if identity(owner['pid'])!=owner['identity']:return
                time.sleep(.2)
            raise RuntimeError('Owned service did not stop; inspect its log')
        if alive:
            ready=json.loads((run/'ready.json').read_text())
            if not ready.get('ready') or ready.get('pid')!=owner['pid']:
                raise RuntimeError('Owned service exists but is not ready')
            print(json.dumps(ready));return
        (run/'STOP').unlink(missing_ok=True)
        write_state(run/'ready.json',dict(ready=False))
        stamp=time.strftime('%Y%m%d_%H%M%S')
        with (LAB/'logs'/('service_'+stamp+'.log')).open('ab') as log:
            child=subprocess.Popen([sys.executable,'-u',str(LAB/'tools/inference_service.py')],
                stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,close_fds=True)
        record=None
        for _ in range(20):
            record=identity(child.pid)
            if record:break
            time.sleep(.05)
        if not record:raise RuntimeError('Could not identify the service child')
        write_state(run/'owner.json',dict(pid=child.pid,identity=record))
        for _ in range(50):
            if child.poll() is not None:raise RuntimeError('BPU service failed; inspect its private log')
            ready=json.loads((run/'ready.json').read_text())
            if ready.get('ready') and ready.get('pid')==child.pid:
                print(json.dumps(ready));return
            time.sleep(.2)
        raise RuntimeError('BPU service startup deadline')


if __name__=='__main__':main()
