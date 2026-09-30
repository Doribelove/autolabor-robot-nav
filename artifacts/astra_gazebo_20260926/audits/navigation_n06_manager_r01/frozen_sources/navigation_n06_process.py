"""Per-PID cleanup with actual ancestry, child-only nonce and start ticks.

Never signal a process group. Linux subreaper keeps orphaned descendants inside
the manager ancestry; uncertain ownership is an integrity failure, not a kill.
"""
import ctypes
from contextlib import contextmanager
import errno
import os
from pathlib import Path
import platform
import signal
import socket
import subprocess
import time

from navigation_n06_integrity import publish

NONCE_KEY='ASTRA_N06_CHILD_NONCE'
SYSCALL_HEADER=Path('/usr/include/x86_64-linux-gnu/asm/unistd_64.h')

def signal_pinned(pid,start_ticks,sig,authorize):
    """Pin kernel process identity before the final ownership check/signal."""
    if platform.machine()!='x86_64':raise RuntimeError('reviewed Linux x86_64 pidfd backend required')
    header=SYSCALL_HEADER.read_text()
    if '#define __NR_pidfd_open 434' not in header or '#define __NR_pidfd_send_signal 424' not in header:raise RuntimeError('pidfd syscall source differs')
    libc=ctypes.CDLL(None,use_errno=True);libc.syscall.restype=ctypes.c_long
    fd=libc.syscall(434,pid,0)
    if fd<0:
        if ctypes.get_errno()==errno.ESRCH:return False
        raise OSError(ctypes.get_errno(),'pidfd_open')
    try:
        row=proc_row(pid)
        if row is None or row['start_ticks']!=str(start_ticks) or not authorize(row):return False
        if libc.syscall(424,fd,int(sig),None,0)<0:
            if ctypes.get_errno()==errno.ESRCH:return False
            raise OSError(ctypes.get_errno(),'pidfd_send_signal')
        return True
    finally:os.close(fd)

def proc_row(pid):
    try:
        raw=(Path('/proc')/str(pid)/'stat').read_text().rsplit(')',1)[1].split()
        return {'pid':int(pid),'ppid':int(raw[1]),'pgid':int(raw[2]),'sid':int(raw[3]),'start_ticks':raw[19],'state':raw[0]}
    except (FileNotFoundError,ProcessLookupError):return None

def descendant_chain(pid,ancestor,reader=proc_row):
    chain=[];seen=set();current=pid
    while current not in seen and current>1:
        seen.add(current);row=reader(current)
        if row is None:return None
        chain.append(row)
        if row['ppid']==ancestor:return chain
        current=row['ppid']
    return None

def child_marker(pid,nonce,root):
    try:
        fields=(Path('/proc')/str(pid)/'environ').read_bytes().split(b'\0')
        return (os.fsencode(NONCE_KEY+'='+nonce) in fields and b'ROS_LOG_DIR='+os.fsencode(Path(root)/'roslog') in fields)
    except (FileNotFoundError,ProcessLookupError,PermissionError):return False

def eligible(row,owner_pid,nonce_matches,reader=proc_row):
    if row is None or row['pid'] in (owner_pid,os.getpid(),1) or row['state']=='Z' or not nonce_matches:return False
    current=reader(row['pid'])
    return current is not None and current['start_ticks']==row['start_ticks'] and descendant_chain(row['pid'],owner_pid,reader) is not None

def scan(owner_pid,nonce,root):
    result=[]
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():continue
        pid=int(entry.name)
        if pid==owner_pid:continue
        row=proc_row(pid)
        if row is None or row['state']=='Z':continue
        chain=descendant_chain(pid,owner_pid)
        marker=child_marker(pid,nonce,root)
        if chain is not None or marker:
            row.update(marker_match=marker,ancestry=chain,eligible=eligible(row,owner_pid,marker))
            result.append(row)
    return sorted(result,key=lambda r:r['pid'])

def enable_subreaper():
    libc=ctypes.CDLL(None,use_errno=True)
    if libc.prctl(36,1,0,0,0)!=0:raise OSError(ctypes.get_errno(),'PR_SET_CHILD_SUBREAPER failed')
    value=ctypes.c_int()
    if libc.prctl(37,ctypes.byref(value),0,0,0)!=0 or value.value!=1:raise RuntimeError('subreaper not verified')

def assert_ports_free(ports=(14830,14831)):
    for port in ports:
        with socket.socket() as sock:
            sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);sock.bind(('127.0.0.1',port))

@contextmanager
def assessment_child_environment(root,nonce):
    """Single-threaded manager: mark spawned assessment children, then restore."""
    values={NONCE_KEY:nonce,'ROS_LOG_DIR':str(Path(root)/'roslog')}
    previous={key:os.environ.get(key) for key in values}
    os.environ.update(values)
    try:yield
    finally:
        for key,value in previous.items():
            if value is None:os.environ.pop(key,None)
            else:os.environ[key]=value

def cleanup(proc,root,nonce,owner_pid=None,grace=(15.,5.,5.),audit_name='cleanup.json'):
    if audit_name not in ('cleanup.json','cleanup_after_assessment.json','cleanup_supervisor.json'):raise ValueError('closed cleanup audit filename required')
    owner_pid=os.getpid() if owner_pid is None else owner_pid
    events=[];uncertain=[];vanished=[]
    for sig,seconds in zip((signal.SIGINT,signal.SIGTERM,signal.SIGKILL),grace):
        deadline=time.monotonic()+seconds
        while True:
            current=scan(owner_pid,nonce,root)
            for row in current:
                fresh=proc_row(row['pid'])
                if fresh is None or fresh['state']=='Z':vanished.append({'pid':row['pid'],'reason':'vanished_or_zombie_before_check'});continue
                if not eligible(row,owner_pid,child_marker(row['pid'],nonce,root)):
                    latest=proc_row(row['pid'])
                    if latest is None or latest['state']=='Z':
                        vanished.append({'pid':row['pid'],'reason':'vanished_or_zombie_during_ownership_check'});continue
                    uncertain.append(row);continue
                # Each signal is preceded by a fresh real ancestry/nonce/tick
                # check; no cached PID or session/group ownership shortcut.
                before=proc_row(row['pid'])
                if before is None or before['start_ticks']!=row['start_ticks']:continue
                sent=signal_pinned(row['pid'],row['start_ticks'],sig,
                    lambda latest:eligible(latest,owner_pid,child_marker(latest['pid'],nonce,root)))
                if not sent:continue
                events.append({'pid':row['pid'],'start_ticks':row['start_ticks'],'signal':int(sig),'ancestry':row['ancestry']})
            if proc is not None:proc.poll()
            if not current or time.monotonic()>=deadline:break
            time.sleep(min(.05,max(0.,deadline-time.monotonic())))
        if not scan(owner_pid,nonce,root):break
    if proc is not None:
        try:proc.wait(timeout=.5)
        except subprocess.TimeoutExpired:pass
    # Reap adopted zombies without stealing the Popen child's status.
    for entry in Path('/proc').iterdir():
        if entry.name.isdigit():
            row=proc_row(int(entry.name))
            if row and row['ppid']==owner_pid and row['state']=='Z' and (proc is None or row['pid']!=proc.pid):
                try:os.waitpid(row['pid'],os.WNOHANG)
                except ChildProcessError:pass
    survivors=scan(owner_pid,nonce,root);port_error=None
    try:assert_ports_free()
    except OSError as exc:port_error=repr(exc)
    record={'schema':'astra_navigation_n06_cleanup_v1','owner_pid':owner_pid,'signals':events,'uncertain_ownership_observed':uncertain,
        'survivors':survivors,'benign_vanished_or_reaped':vanished,'port_error':port_error,'pass':not survivors and port_error is None and not uncertain,
        'killpg_used':False,'scope':'real descendants plus child-only nonce and start ticks; external host exit is not observed'}
    publish(Path(root)/audit_name,record)
    if not record['pass']:raise RuntimeError('N06 cleanup integrity failed')
    return record

class StopRequest:
    def __init__(self):self.signal=None;self.old={}
    def __enter__(self):
        def mark(sig,_frame):self.signal=sig
        for sig in (signal.SIGINT,signal.SIGTERM):self.old[sig]=signal.signal(sig,mark)
        return self
    def __exit__(self,*_):
        for sig,handler in self.old.items():signal.signal(sig,handler)

def checked_parent(pid,start_ticks):
    row=proc_row(pid)
    return row is not None and row['start_ticks']==str(start_ticks)
