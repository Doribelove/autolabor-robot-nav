#!/usr/bin/env python3
"""Control one isolated ZED candidate; never change the existing robot stack."""
import argparse
import configparser
import datetime
import fcntl
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time
import uuid

RELEASE = Path(__file__).resolve().parent
BASE = RELEASE.parent.parent


def read_json(path):
    return json.loads(path.read_text())


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(1048576), b''):
            h.update(chunk)
    return h.hexdigest()


def require_owned():
    if BASE != Path('/map/j6m-dev-b/zed-direct') or RELEASE.parent != BASE/'releases':
        raise RuntimeError('Unexpected candidate path')
    meta = read_json(RELEASE/'deployment.json')
    if meta['release_path'] != str(RELEASE) or meta['owner'] != 'j6m-dev-b/vision-bpu':
        raise RuntimeError('Candidate ownership mismatch')
    if read_json(BASE/'owner.json')['owner'] != meta['owner']:
        raise RuntimeError('Base ownership mismatch')
    return meta


def run(args, timeout=10, env=None):
    child = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                             text=True, env=env, start_new_session=True)
    try:
        out, err = child.communicate(timeout=timeout)
        return {'command':args, 'returncode':child.returncode, 'stdout':out, 'stderr':err}
    except (subprocess.TimeoutExpired, KeyboardInterrupt):
        os.killpg(child.pid, signal.SIGTERM)
        try:
            out, err = child.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGKILL)
            out, err = child.communicate(timeout=3)
        return {'command':args, 'returncode':124, 'stdout':out, 'stderr':err,
                'timed_out_or_interrupted':True}


def current_target():
    link = BASE/'current'
    if link.is_symlink():
        return os.readlink(link)
    if link.exists():
        raise RuntimeError('current is not a symlink')
    return None


def installed():
    manifest_path = RELEASE/'installed_manifest.json'
    if not manifest_path.is_file():
        return {'ok':False, 'reason':'Build has not been validated'}
    changed = []
    rows = read_json(manifest_path)
    for row in rows:
        rel = Path(row['path'])
        if rel.is_absolute() or '..' in rel.parts:
            raise RuntimeError('Invalid installed manifest path')
        path = RELEASE/rel
        if not path.is_file() or digest(path) != row['sha256']:
            changed.append(str(rel))
    return {'ok':bool(rows) and not changed, 'changed':changed, 'files':len(rows)}


def baseline_check():
    baseline = read_json(RELEASE/'baseline.json')
    changed = []
    for name, expected in baseline['baseline_hashes'].items():
        path = Path(name)
        actual = digest(path) if path.is_file() else None
        if actual != expected:
            changed.append(name)
    for row in baseline['current_links']:
        path = Path(row['path'])
        if not path.is_symlink() or os.readlink(path) != row['link']:
            changed.append(row['path'])
    with gzip.open('/proc/config.gz','rb') as f:
        config_hash = hashlib.sha256(f.read()).hexdigest()
    if config_hash != baseline['kernel_config_sha256']:
        changed.append('/proc/config.gz (decompressed)')
    return {'ok':not changed, 'changed':changed}


def text(path):
    try:
        return path.read_text().strip()
    except OSError:
        return None


def hardware():
    with gzip.open('/proc/config.gz','rt') as f:
        config = f.read().splitlines()
    usb = []
    for path in Path('/sys/bus/usb/devices').glob('*'):
        if (path/'idVendor').exists():
            usb.append(dict(path=str(path), **{
                key:text(path/key) for key in ['idVendor','idProduct','serial','product','speed']}))
    videos = []
    for entry in Path('/sys/class/video4linux').glob('video*'):
        parents = [entry.resolve()] + list(entry.resolve().parents)
        parent = next((p for p in parents if (p/'idVendor').exists()), None)
        if parent and text(parent/'idVendor') == '2b03':
            videos.append({'path':'/dev/'+entry.name, 'usb':str(parent),
                           'serial':text(parent/'serial'), 'speed':text(parent/'speed')})
    controller_modes = []
    for directory, _, names in os.walk('/proc/device-tree'):
        if 'dr_mode' in names:
            path = Path(directory)/'dr_mode'
            controller_modes.append({'path':str(path), 'mode':path.read_bytes().rstrip(b'\x00').decode(errors='replace')})
    reasons = []
    if not usb:
        reasons.append('NO_USB_HOST_DEVICES_ENUMERATED')
        if controller_modes and all(x['mode']=='peripheral' for x in controller_modes):
            reasons.append('USB_CONTROLLER_IN_PERIPHERAL_MODE')
    if '# CONFIG_MEDIA_SUPPORT is not set' in config:
        reasons.append('KERNEL_MEDIA_SUPPORT_DISABLED')
    if not any(line in config for line in ['CONFIG_USB_VIDEO_CLASS=y','CONFIG_USB_VIDEO_CLASS=m']):
        reasons.append('KERNEL_UVC_SUPPORT_MISSING')
    if not any(row['idVendor'] == '2b03' for row in usb):
        reasons.append('ZED_USB_NOT_ENUMERATED')
    if not videos:
        reasons.append('ZED_VIDEO_DEVICE_MISSING')
    return {'ready':not reasons, 'blockers':reasons, 'usb':usb, 'zed_video_devices':videos,
            'usb_controller_modes':controller_modes,
            'hidapi_backend':'libusb', 'hidraw_required_for_this_build':False}


def calibration(meta):
    name = 'SN'+str(meta['expected_serial'])+'.conf'
    path = RELEASE/'calibration'/name
    config = configparser.ConfigParser()
    if not path.is_file() or digest(path) != meta['calibration_sha256']:
        return {'ok':False, 'reason':'Calibration identity/hash mismatch'}
    config.read(path)
    baseline = config.getfloat('STEREO','Baseline')
    focal = config.getfloat('LEFT_CAM_HD','fx')
    return {'ok':0 < baseline < 1000 and focal > 0, 'file':str(path),
            'expected_serial':meta['expected_serial'], 'baseline_mm':baseline,
            'camera_identity_confirmed':False, 'real_depth_accuracy_validated':False}


def status(meta):
    return {'release':str(RELEASE), 'active_candidate':current_target(),
            'software':installed(), 'original_version':baseline_check(),
            'hardware':hardware(), 'calibration':calibration(meta),
            'production_camera_backend_changed':False,
            'production_ready':False, 'motion_eligible':False}


def switch(meta, rollback=False):
    own = 'releases/'+RELEASE.name
    old = meta['previous_candidate_current']
    current = current_target()
    expected, target = (own, old) if rollback else (old, own)
    if current == target:
        return {'changed':False, 'current':target}
    if current != expected:
        raise RuntimeError('current belongs to another deployment; refusing to change it')
    if not rollback and (not installed()['ok'] or not baseline_check()['ok']):
        raise RuntimeError('Candidate or baseline verification failed')
    if target is None:
        (BASE/'current').unlink()
    else:
        # A previous candidate must remain inside this experiment's release tree.
        target_path = (BASE/target).resolve()
        if target_path.parent != BASE/'releases' or not target_path.is_dir():
            raise RuntimeError('Invalid candidate target')
        temporary = BASE/('.current-'+uuid.uuid4().hex)
        os.symlink(target,temporary)
        os.replace(temporary,BASE/'current')
    return {'changed':True, 'current':target, 'original_robot_version_changed':False}


def device_busy(device):
    owners=[]
    for directory in Path('/proc').glob('[0-9]*'):
        for fd in (directory/'fd').glob('*'):
            try:
                if os.readlink(fd) == device:
                    owners.append(int(directory.name)); break
            except OSError:
                continue
    return owners


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=['status','check','self-test','probe','activate','rollback'])
    parser.add_argument('--seconds',type=int,default=10)
    parser.add_argument('--device',help='An enumerated Stereolabs /dev/videoN path')
    args=parser.parse_args()
    meta=require_owned()
    if os.geteuid()!=0:
        raise RuntimeError('Run this board-local controller as root')
    if not 1 <= args.seconds <= 60:
        raise RuntimeError('seconds must be between 1 and 60')
    # Serialize candidate capture, switching, and isolated validation. No daemon is started.
    with (BASE/'operation.lock').open('a') as lock:
        try: fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError: raise RuntimeError('Another owned candidate operation is running')
        report={'command':args.command,'time':datetime.datetime.now().astimezone().isoformat()}
        rc=0
        if args.command in ['activate','rollback']:
            report['result']=switch(meta,rollback=args.command=='rollback')
        else:
            report.update(status(meta))
            if not report['software']['ok'] or not report['original_version']['ok'] or not report['calibration']['ok']:
                rc=10
            elif args.command=='check':
                rc=0 if report['hardware']['ready'] else 20
            elif args.command=='self-test':
                report['native_version']=run([str(RELEASE/'install/bin/zed_capture_probe'),'--version'])
                report['native_enumeration']=run([str(RELEASE/'install/bin/zed_capture_probe'),'--list'])
                report['isolated_opencv_ros']=run(['unshare','--mount','--net','--propagation','private',
                    '/bin/bash',str(RELEASE/'namespace.sh'),'self-test'],timeout=45)
                rc=0 if all(report[k]['returncode']==0 for k in ['native_version','native_enumeration','isolated_opencv_ros']) else 11
            elif args.command=='probe':
                if not report['hardware']['ready']:
                    rc=20
                else:
                    devices=report['hardware']['zed_video_devices']
                    if args.device:
                        devices=[d for d in devices if d['path']==args.device]
                    if len(devices)!=1:
                        raise RuntimeError('Specify one of the enumerated ZED video devices with --device')
                    device=devices[0]
                    if float(device['speed'] or 0)<5000:
                        raise RuntimeError('HD720 validation requires a USB 3 connection')
                    if device_busy(device['path']):
                        raise RuntimeError('Camera is already open by another process')
                    if time.time()<1767225600:
                        raise RuntimeError('Board system time must be synchronized first')
                    number=re.fullmatch(r'/dev/video([0-9]+)',device['path'])
                    if not number: raise RuntimeError('Unexpected video device path')
                    report['capture']=run([str(RELEASE/'install/bin/zed_capture_probe'),'--capture',
                        number[1],str(meta['expected_serial']),str(args.seconds)],timeout=args.seconds+15)
                    rc=report['capture']['returncode']
        report['exit_code']=rc
        # A separate directory per invocation preserves failed and successful runs.
        log_dir=Path(tempfile.mkdtemp(prefix=args.command+'_',dir=str(RELEASE/'logs')))
        (log_dir/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        report['evidence']=str(log_dir/'result.json')
        print(json.dumps(report,ensure_ascii=False,indent=2))
        return rc


if __name__=='__main__':
    try:
        sys.exit(main())
    except Exception as exc:
        print(json.dumps({'error':str(exc)},ensure_ascii=False),file=sys.stderr)
        sys.exit(2)
