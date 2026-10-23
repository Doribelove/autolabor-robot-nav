"""Own only this candidate's camera and BPU children; never stop other services."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import hashlib


def preflight(config):
    """Resolve the full import/contract graph before touching camera/BPU ownership."""
    root = Path(config['runtime_root']).resolve(strict=True)
    for name, expected in json.loads((root/'MANIFEST.json').read_text()).items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest() != expected:
            raise RuntimeError('Candidate hash mismatch: '+name)
    code = root/'code'
    env = dict(os.environ, LD_LIBRARY_PATH=config['bpu_ld_library_path'],
        PYTHONPATH=':'.join([str(code), str(code/'distribution/j6m_bpu_navigation/tools'),
                            str(code/'src/application/autolabor_fod_vision/src')]+config['python_dependencies']),
        OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', PYTHONDONTWRITEBYTECODE='1')
    program = ('import cv2,numpy,PIL; import native_tensor,board_preprocess,production_io; '
        'from autolabor_fod_vision.two_stage import context_crop; '
        'from distribution.j6m_bpu_navigation.surround.inference import BpuEngine; '
        'c=production_io.contract(); assert c["target"]=="nash-m"; '
        'print("PREFLIGHT_OK",cv2.__version__,numpy.__version__,PIL.__version__)')
    subprocess.run([sys.executable,'-B','-c',program],env=env,check=True,timeout=15)


def stop_owned(root):
    record = root/'session.json'
    if not record.exists():
        return
    data = json.loads(record.read_text())
    pid = data['pid']
    proc = Path('/proc/%d'%pid)
    expected = str(root/'code/distribution/j6m_bpu_navigation/surround/operator_board_session.py').encode()
    if (proc.exists() and expected in (proc/'cmdline').read_bytes().split(b'\0')
            and (proc/'cwd').resolve() == root
            and (proc/'stat').read_text().split()[21] == data['proc_start_ticks']):
        os.kill(pid, signal.SIGTERM)


def run(config):
    root = Path(config['runtime_root']).resolve(strict=True)
    if not str(root).startswith('/map/j6m-dev-b/vision-bpu-surround/operator_') or Path.cwd() != root:
        raise RuntimeError('Candidate session path mismatch')
    lock = (root/'session.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    # Check exclusivity before camera initialization. The actual owners lock again.
    for name in ['/map/j6m-dev-b/vision-bpu-surround/camera.lock', config['inference']['exclusive_lock']]:
        with Path(name).open('a') as probe:
            fcntl.flock(probe, fcntl.LOCK_EX | fcntl.LOCK_NB)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: stop.set())
    children, logs = [], []
    record = root/'session.json'
    try:
        stamp = time.strftime('%Y%m%d_%H%M%S')
        code = root/'code'
        for kind in ['gdc', 'inference']:
            log = (root/('%s_%s.log'%(kind,stamp))).open('a')
            logs.append(log)
            env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', CAM_LOGLEVEL='2', LOGLEVEL='2')
            if kind == 'gdc':
                env['PYTHONPATH'] = str(code)
                command = [sys.executable,'-B','-m','distribution.j6m_bpu_navigation.surround.gdc_diagnostic',
                           '--assets',str(root/'assets')]
            else:
                env.update(LD_LIBRARY_PATH=config['bpu_ld_library_path'],
                    PYTHONPATH=':'.join([str(code)]+config['python_dependencies']),
                    OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
                command = [sys.executable,'-B','-m','distribution.j6m_bpu_navigation.surround.operator_server',
                           '--config',str(root/'config.json')]
            children.append(subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT))
        data = dict(pid=os.getpid(),proc_start_ticks=Path('/proc/self/stat').read_text().split()[21],
                    children=[p.pid for p in children],started_at=time.time(),runtime_root=str(root))
        record.write_text(json.dumps(data,indent=2)+'\n')
        print('SURROUND_SESSION_STARTED', json.dumps(data), flush=True)
        while not stop.wait(.25):
            if any(child.poll() is not None for child in children):
                raise RuntimeError('A camera/inference child exited; see candidate logs')
    finally:
        for child in reversed(children):
            if child.poll() is None:
                child.terminate()
            try:
                child.wait(timeout=12)
            except subprocess.TimeoutExpired:
                child.kill()  # Only this Popen-owned child, never a process-name kill.
                child.wait(timeout=3)
        for log in logs:
            log.close()
        if record.exists() and json.loads(record.read_text()).get('pid') == os.getpid():
            record.unlink()
        print('SURROUND_SESSION_STOPPED',flush=True)


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('action',choices=['run','stop','check'])
    parser.add_argument('--config',required=True)
    args=parser.parse_args()
    config=json.loads(Path(args.config).read_text())
    if args.action=='stop':
        stop_owned(Path(config['runtime_root']).resolve(strict=True))
    elif args.action=='check':
        preflight(config)
    else:
        preflight(config)
        run(config)
