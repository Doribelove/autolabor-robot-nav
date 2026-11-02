#!/usr/bin/env python3
"""Capture the current BEV and test the existing trash HBM on J6M, then restore.

The model has different weights from best6.pt. This command only saves reports;
it does not select the live detector, generate map points, or control the car.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shlex
import signal
import struct
import subprocess
import time
import urllib.request

from pixel_bpu_worker import CURRENT_CHECKPOINT_SHA

ROOT = Path(__file__).resolve().parents[2]
SSH = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', 'root@192.168.10.100']
SERVICE = 'j6m-b-camera-receiver.service'
LINK = 'j6m-b-camera-link.service'


def remote(command, **kwargs):
    return subprocess.run(SSH+[command], check=True, timeout=60, **kwargs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--frames', type=int, default=20)
    parser.add_argument('--pause-camera-receiver', action='store_true',
                        help='Pause the old BPU receiver and its restart supervisor, then restore both')
    args = parser.parse_args()
    if not 1 <= args.frames <= 100:
        parser.error('--frames must be 1..100')
    current_model = ROOT/'src/application/autolabor_fod_vision/models/best6.pt'
    if hashlib.sha256(current_model.read_bytes()).hexdigest() != CURRENT_CHECKPOINT_SHA:
        raise ValueError('Current CUDA checkpoint changed; update the comparison identity first')
    args.output.mkdir(parents=True, exist_ok=True)
    samples, seen = [], set()
    deadline = time.monotonic()+max(10, args.frames)
    while len(samples) < args.frames and time.monotonic() < deadline:
        with urllib.request.urlopen('http://127.0.0.1:18883/frame', timeout=2) as response:
            data = response.read()
        magic, nj, ni = struct.unpack('!4sII', data[:12])
        if magic != b'BG01' or len(data) != 12+nj+ni:
            raise ValueError('Invalid BEV packet')
        meta = json.loads(data[12:12+nj])
        key = (meta['fusion_session'], meta['fusion_sequence'])
        if time.time()-meta['produced_epoch_s'] > 1.5:
            raise ValueError('Stale BEV capture')
        if key not in seen:
            seen.add(key)
            name = '%04d.jpg'%len(samples)
            jpeg = data[12+nj:]
            (args.output/name).write_bytes(jpeg)
            samples.append(dict(file=name, jpeg_sha256=hashlib.sha256(jpeg).hexdigest(),
                                source_id=dict(session=key[0], sequence=key[1]), metadata=meta))
        time.sleep(.05)
    if len(samples) != args.frames:
        raise RuntimeError('Not enough distinct live frames')
    (args.output/'samples.json').write_text(json.dumps(samples, indent=2)+'\n')
    board = remote('mktemp -d /tmp/conbine-bev-pixel-XXXXXXXX', capture_output=True, text=True).stdout.strip()
    if not re.fullmatch(r'/tmp/conbine-bev-pixel-[A-Za-z0-9]+', board):
        raise ValueError('Unexpected board temporary directory')
    active = remote('systemctl is-active '+SERVICE+' || true', capture_output=True, text=True).stdout.strip() == 'active'
    link_active = subprocess.run(['systemctl', '--user', 'is-active', '--quiet', LINK]).returncode == 0
    if (active or link_active) and not args.pause_camera_receiver:
        raise RuntimeError('Existing BPU receiver is active; rerun with --pause-camera-receiver to restore it after testing')
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        files = [ROOT/'tools/bev_feature/pixel_bpu_worker.py', args.output/'samples.json']
        files += [args.output/s['file'] for s in samples]
        subprocess.run(['scp', '-q', *map(str, files), 'root@192.168.10.100:'+board+'/'], check=True, timeout=30)
        if link_active:
            subprocess.run(['systemctl', '--user', 'stop', LINK], check=True, timeout=30)
        if active:
            remote('systemctl stop '+SERVICE)
        result = remote(shlex.join(['python3', '-B', board+'/pixel_bpu_worker.py', board]),
                        capture_output=True, text=True)
        (args.output/'board.log').write_text(result.stdout+result.stderr)
        subprocess.run(['scp', '-q', 'root@192.168.10.100:'+board+'/report.json',
                        str(args.output)], check=True, timeout=30)
        subprocess.run(['scp', '-q', 'root@192.168.10.100:'+board+'/*_bpu.jpg',
                        str(args.output)], check=True, timeout=30)
        print(result.stdout)
    except subprocess.CalledProcessError as exc:
        (args.output/'board_error.log').write_text(str(exc)+'\n'+str(exc.stdout)+'\n'+str(exc.stderr))
        raise
    finally:
        try:
            if active:
                remote('systemctl start '+SERVICE)
        finally:
            if link_active:
                subprocess.run(['systemctl', '--user', 'start', LINK], check=True, timeout=30)
        restored = remote('systemctl show '+SERVICE+' -p ActiveState -p MainPID',
                          capture_output=True, text=True).stdout
        link_restored = subprocess.run(['systemctl', '--user', 'show', LINK, '-p', 'ActiveState',
                                       '-p', 'MainPID'], check=True, capture_output=True, text=True).stdout
        (args.output/'restoration.json').write_text(json.dumps(dict(
            receiver_was_active=active, final_service=restored,
            link_was_active=link_active, final_link=link_restored,
            board_evidence=board), indent=2)+'\n')


if __name__ == '__main__':
    main()
