"""Private desktop session with explicit, reversible BPU ownership handoff."""
import argparse
import ctypes as C
import fcntl
import json
import os
from pathlib import Path
import re
import shlex
import signal
import socket
import subprocess
import sys
import threading
import time
import urllib.request

WS=Path(__file__).resolve().parents[3]
RUNTIME=WS/'runtime/surround_operator'
UNIT='j6m-b-surround-operator.service'
SUPERVISOR='j6m-b-camera-link.service'
RECEIVER='j6m-b-camera-receiver.service'
SSH=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=5','-o','ServerAliveInterval=5',
     '-o','ServerAliveCountMax=3','root@192.168.10.100']


def command(args,timeout=20,check=True):
    return subprocess.run(args,text=True,capture_output=True,timeout=timeout,check=check)


def remote(args,**kwargs):
    return command(SSH+[' '.join(shlex.quote(str(arg)) for arg in args)],**kwargs)


def close_previous_preview():
    root=WS/'runtime/surround_gdc_preview_20261023'
    pidfile=root/'viewer.pid'
    if not pidfile.exists():return
    pid=int(pidfile.read_text())
    proc=Path('/proc/%d'%pid)
    if not proc.exists():return
    if str(root/'viewer.py').encode() not in (proc/'cmdline').read_bytes().split(b'\0'):
        raise RuntimeError('Previous preview PID belongs to another program')
    tree=command(['xwininfo','-root','-tree']).stdout
    candidates=re.findall(r'(0x[0-9a-f]+) "J6M 三路 GDC 透视预览[^\n]+',tree)
    owned=[]
    for window in candidates:
        prop=command(['xprop','-id',window,'_NET_WM_PID']).stdout
        if re.search(r'=\s*'+str(pid)+r'\s*$',prop):owned.append(int(window,16))
    if len(owned)!=1:raise RuntimeError('Cannot identify the previous GDC preview window')
    class Data(C.Union):
        _fields_=[('b',C.c_char*20),('s',C.c_short*10),('l',C.c_long*5)]
    class Message(C.Structure):
        _fields_=[('type',C.c_int),('serial',C.c_ulong),('send_event',C.c_int),
                  ('display',C.c_void_p),('window',C.c_ulong),('message_type',C.c_ulong),
                  ('format',C.c_int),('data',Data)]
    class Event(C.Union):
        _fields_=[('message',Message),('pad',C.c_long*24)]
    x=C.CDLL('libX11.so.6')
    x.XOpenDisplay.argtypes=[C.c_char_p];x.XOpenDisplay.restype=C.c_void_p
    x.XInternAtom.argtypes=[C.c_void_p,C.c_char_p,C.c_int];x.XInternAtom.restype=C.c_ulong
    x.XSendEvent.argtypes=[C.c_void_p,C.c_ulong,C.c_int,C.c_long,C.POINTER(Event)]
    x.XFlush.argtypes=[C.c_void_p];x.XCloseDisplay.argtypes=[C.c_void_p]
    display=x.XOpenDisplay(None)
    if not display:raise RuntimeError('Desktop is unavailable')
    event=Event()
    event.message.type=33;event.message.display=display;event.message.window=owned[0]
    event.message.message_type=x.XInternAtom(display,b'WM_PROTOCOLS',False)
    event.message.format=32
    event.message.data.l[0]=x.XInternAtom(display,b'WM_DELETE_WINDOW',False)
    x.XSendEvent(display,owned[0],False,0,C.byref(event));x.XFlush(display);x.XCloseDisplay(display)
    deadline=time.monotonic()+15
    while proc.exists() and time.monotonic()<deadline:time.sleep(.1)
    if proc.exists():raise RuntimeError('Previous GDC viewer has not released its camera')


def run():
    config=json.loads((RUNTIME/'session_config.json').read_text())
    # A user-reviewed ownership handoff is required; never ignore the BPU lock.
    if not config.get('take_over_existing_bpu'):
        raise RuntimeError('Existing ZED BPU handoff has not been authorized for this session')
    lock=(RUNTIME/'session.lock').open('a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    root=Path(config['board_root'])
    script=root/'code/distribution/j6m_bpu_navigation/surround/operator_board_session.py'
    board_args=['python3','-B',str(script),'run','--config',str(root/'config.json')]
    remote(['python3','-B',str(script),'check','--config',str(root/'config.json')])
    prior_supervisor=command(['systemctl','--user','is-active','--quiet',SUPERVISOR],check=False).returncode==0
    prior_receiver=remote(['systemctl','is-active','--quiet',RECEIVER],check=False).returncode==0
    supervisor_exec=command(['systemctl','--user','show',SUPERVISOR,'-p','ExecStart','--value']).stdout
    receiver_exec=remote(['systemctl','show',RECEIVER,'-p','ExecStart','--value']).stdout
    if prior_supervisor and '/home/slam/j6m-dev-b/distribution/j6m_bpu_navigation/camera/agx_services.py' not in supervisor_exec:
        raise RuntimeError('BPU supervisor ownership mismatch')
    if prior_receiver and '/map/j6m-dev-b/camera/releases/20261021_operator_qt/receiver.py' not in receiver_exec:
        raise RuntimeError('Existing BPU receiver ownership mismatch')
    for port in [11342,18781]:
        with socket.socket() as probe:
            probe.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            probe.bind(('127.0.0.1',port))
    stop=threading.Event()
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,lambda *_:stop.set())
    ssh=gui=None
    paused_supervisor=paused_receiver=False
    log=(RUNTIME/'session.log').open('a')
    (RUNTIME/'restore_state.json').write_text(json.dumps(dict(prior_supervisor=prior_supervisor,
        prior_receiver=prior_receiver,board_root=str(root),started_at=time.time(),pid=os.getpid()),indent=2)+'\n')
    try:
        close_previous_preview()
        if prior_supervisor:
            command(['systemctl','--user','stop',SUPERVISOR]);paused_supervisor=True
        if prior_receiver:
            remote(['systemctl','stop',RECEIVER]);paused_receiver=True
        remote_command='cd '+shlex.quote(str(root))+' && exec '+' '.join(shlex.quote(arg) for arg in board_args)
        ssh=subprocess.Popen(SSH[:-1]+['-o','ExitOnForwardFailure=yes',
            '-L','127.0.0.1:18781:127.0.0.1:18781',SSH[-1],remote_command],stdout=log,stderr=log)
        deadline=time.monotonic()+35
        state=None
        while not stop.is_set() and time.monotonic()<deadline:
            if ssh.poll() is not None:raise RuntimeError('Board camera/BPU session exited')
            try:
                with urllib.request.urlopen('http://127.0.0.1:18781/state',timeout=.5) as response:state=json.load(response)
                if all(state['ports'][str(p)]['fresh'] for p in range(3)):break
            except (OSError,ValueError):pass
            stop.wait(.2)
        if stop.is_set():return
        if not state or not all(state['ports'][str(p)]['fresh'] for p in range(3)):
            raise RuntimeError('Three GDC/BPU streams not ready: '+json.dumps(state))
        (RUNTIME/'startup_state.json').write_text(json.dumps(state,indent=2)+'\n')
        gui=subprocess.Popen(['/bin/bash',str(WS/'distribution/j6m_bpu_navigation/surround/operator_qt.sh'),'run-gui'],
                             stdout=log,stderr=log,start_new_session=True)
        print('THREE_PORT_QT_STARTED',gui.pid,flush=True)
        while not stop.wait(.25) and gui.poll() is None:
            if ssh.poll() is not None:raise RuntimeError('Board session lost; closing owned GUI')
    finally:
        if gui and gui.poll() is None:
            os.killpg(gui.pid,signal.SIGINT)
            try:gui.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(gui.pid,signal.SIGTERM);gui.wait(timeout=5)
        if ssh:
            try:
                remote(['python3','-B',str(script),'stop','--config',str(root/'config.json')])
                ssh.wait(timeout=15)
            except Exception as exc:
                print('PRIVATE_BOARD_CLEANUP_ERROR',str(exc),flush=True)
                ssh.terminate()
        # Restore only services that this invocation found active and paused.
        try:
            if paused_receiver:
                # Do not start the old receiver while a lost SSH session still owns BPU.
                wait_code = "import fcntl,time; from pathlib import Path\nf=Path('/map/robot_j6m_navigation_20260907/bpu_lab_20260905/preview_worker.lock').open('a')\nfor i in range(50):\n try:\n  fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB); break\n except BlockingIOError: time.sleep(.2)\nelse: raise RuntimeError('Surround BPU owner still present; original receiver not restarted')\n"
                remote(['python3','-B','-c',wait_code],timeout=15)
            if paused_receiver:remote(['systemctl','start',RECEIVER])
            if paused_supervisor:command(['systemctl','--user','start',SUPERVISOR])
        finally:
            log.close()
        print('SURROUND_QT_CLOSED_PREVIOUS_BPU_RESTORED',flush=True)


def main(action):
    if WS != Path('/home/slam/j6m-dev-b-surround'):
        raise RuntimeError('Surround worktree only')
    if action=='run':return run()
    existing=command(['systemctl','--user','show',UNIT,'-p','ExecStart','--value'],check=False).stdout
    if existing.strip() and str(Path(__file__).resolve()) not in existing:
        raise RuntimeError('Foreign Qt session unit')
    if action=='start':
        if command(['systemctl','--user','is-active','--quiet',UNIT],check=False).returncode==0:
            return main('status')
        args=['systemd-run','--user','--unit='+UNIT,'--collect',
            '--property=WorkingDirectory='+str(WS),'--property=KillMode=mixed','--property=TimeoutStopSec=50',
            '--setenv=DISPLAY='+os.environ.get('DISPLAY',':0'),
            '--setenv=XAUTHORITY='+os.environ.get('XAUTHORITY','/run/user/1000/gdm/Xauthority'),
            sys.executable,'-B',str(Path(__file__).resolve()),'run']
        print(command(args).stdout)
    elif action=='stop':
        print(command(['systemctl','--user','stop',UNIT],timeout=55).stdout)
    else:
        print(command(['systemctl','--user','show',UNIT,'-p','ActiveState,SubState,MainPID,ExecStart'],check=False).stdout)
        try:
            with urllib.request.urlopen('http://127.0.0.1:18781/state',timeout=1) as response:
                print(json.dumps(json.load(response),indent=2))
        except OSError:pass


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('action',choices=['start','run','stop','status'])
    main(parser.parse_args().action)
