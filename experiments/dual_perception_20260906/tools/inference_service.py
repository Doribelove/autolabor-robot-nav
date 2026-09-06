#!/usr/bin/env python3
"""Two resident models, one bounded inference lane; listens on J6M loopback only."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
import fcntl
import hashlib
import json
from pathlib import Path
import signal
import socket
import sys
import threading
import time

LAB=Path('/map/robot_j6m_optimized_20260905/bpu_perception_20260906')
REFERENCE=Path('/map/robot_j6m_optimized_20260905/bpu_lab_20260905')
RUNTIME=str(REFERENCE/'vendor/runtime_4_9_2/lib')+':/usr/hobot/lib'
sys.path[:0]=[str(LAB/'python'),str(REFERENCE/'vendor/python311'),str(REFERENCE/'tools')]
FCOS_SHA='1daacc7dfde181b65872c47f4fe5db84fd31f19d3ad56aa1fa1d902bb5ffd7fa'
CENTERPOINT_SHA='7a12188054b4f5a05c112dd6d8611436d84cda3e060e11f72d1f1e739040593c'


def write_ready(value):
    path=LAB/'run/ready.json'
    temporary=path.with_name(path.name+'.tmp.'+str(os.getpid()))
    temporary.write_text(json.dumps(value)+'\n');temporary.replace(path)


def main():
    if Path(__file__).resolve().parents[1]!=LAB:
        raise RuntimeError('Use the private J6M perception directory')
    if os.environ.get('LD_LIBRARY_PATH')!=RUNTIME:
        os.execve(sys.executable,[sys.executable,'-u']+sys.argv,dict(os.environ,LD_LIBRARY_PATH=RUNTIME))
    import numpy as np
    from autolabor_bpu_perception.protocol import receive,send,validate_request,PORT
    from autolabor_bpu_perception.centerpoint import decode as decode_centerpoint
    from resident_fcos import ResidentFCOS
    from decode_fcos_demo import decode as decode_fcos
    from centerpoint_binding import ResidentCenterPoint
    models={'fcos':REFERENCE/'models/fcos_nash_m/model.hbm',
            'centerpoint':LAB/'models/centerpoint_nash_m/model.hbm'}
    hashes={'fcos':FCOS_SHA,'centerpoint':CENTERPOINT_SHA}
    for key,path in models.items():
        if hashlib.sha256(path.read_bytes()).hexdigest()!=hashes[key]:
            raise RuntimeError('Model SHA mismatch: '+key)
    # Share the existing ownership lock: an old preview cannot run concurrently.
    lock=(REFERENCE/'preview_worker.lock').open('a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    stop=threading.Event()
    signal.signal(signal.SIGTERM,lambda *_:stop.set())
    signal.signal(signal.SIGINT,lambda *_:stop.set())
    engines={}
    lane=threading.Lock(); slots=threading.BoundedSemaphore(2)
    last_run={'fcos':0.0,'centerpoint':0.0}
    connections=set();connection_lock=threading.Lock();threads=[]
    try:
        engines['fcos']=ResidentFCOS(REFERENCE,models['fcos'])
        engines['centerpoint']=ResidentCenterPoint(LAB,models['centerpoint'])
        ready=dict(pid=os.getpid(),ready=True,models=hashes,port=PORT,motion_eligible=False)
        client_errors=[0]

        def handle(connection):
            previous=0
            try:
                while not stop.is_set():
                    try:header,payload=receive(connection,timeout=10)
                    except (EOFError,TimeoutError):break
                    received=time.monotonic();validate_request(header,payload)
                    if header['sequence']<=previous:raise ValueError('Replayed request sequence')
                    previous=header['sequence'];key=header['model']
                    result={k:header[k] for k in ('model','sequence','stamp_ns')}
                    result.update(ok=False,motion_eligible=False,model_sha256=hashes[key])
                    if not lane.acquire(timeout=.10):
                        result['error']='BPU inference lane busy'
                        send(connection,result);continue
                    try:
                        period=1/30 if key=='fcos' else 1/5
                        if received-last_run[key]<period*.90:
                            raise ValueError('Model request frequency exceeds budget')
                        if time.monotonic()-received>.15:
                            raise TimeoutError('Request expired before BPU dispatch')
                        last_run[key]=time.monotonic()
                        if key=='fcos':
                            rows=header['rows'];n_y=896*rows
                            tensor=np.empty(896*896*3//2,dtype=np.uint8)
                            tensor[:896*896]=16;tensor[896*896:]=128
                            raw=np.frombuffer(payload,dtype=np.uint8)
                            tensor[:n_y]=raw[:n_y];tensor[896*896:896*896+len(raw)-n_y]=raw[n_y:]
                            outputs,timing=engines[key].infer(tensor.tobytes())
                            detections=decode_fcos(outputs)
                        else:
                            points=np.frombuffer(payload,dtype='<f4').reshape(-1,5)
                            if not np.isfinite(points).all():raise ValueError('Nonfinite point input')
                            outputs,timing=engines[key].infer(points)
                            detections=decode_centerpoint(outputs)
                        result.update(ok=True,detections=detections,timing=timing,
                                      service_ms=(time.monotonic()-received)*1000)
                    except Exception as error:
                        result['error']=str(error)
                    finally:lane.release()
                    send(connection,result)
            except Exception as error:
                client_errors[0]+=1
                if client_errors[0]<=20:
                    print('CLIENT_CLOSED '+str(error),flush=True)
            finally:
                with connection_lock:connections.discard(connection)
                connection.close();slots.release()

        with socket.socket() as server:
            server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            server.bind(('127.0.0.1',PORT));server.listen(2);server.settimeout(.25)
            write_ready(ready)
            print('PERCEPTION_READY '+json.dumps(ready),flush=True)
            while not stop.is_set() and not (LAB/'run/STOP').exists():
                try:connection,_=server.accept()
                except socket.timeout:continue
                connection.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
                if not slots.acquire(blocking=False):connection.close();continue
                with connection_lock:connections.add(connection)
                worker=threading.Thread(target=handle,args=(connection,),daemon=True)
                worker.start();threads=[t for t in threads if t.is_alive()]+[worker]
    finally:
        stop.set()
        with connection_lock:
            for connection in connections:
                try:connection.shutdown(socket.SHUT_RDWR)
                except OSError:pass
        for thread in threads:thread.join(timeout=3)
        if any(t.is_alive() for t in threads):
            # Never free device buffers while an inference thread may use them.
            print('Inference thread did not terminate; process exit releases resources',flush=True)
            os._exit(3)
        for engine in engines.values():engine.close()
        write_ready(dict(ready=False,pid=os.getpid()))
        lock.close()


if __name__=='__main__':main()
