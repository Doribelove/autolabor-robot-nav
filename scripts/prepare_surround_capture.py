#!/usr/bin/env python3
"""Prepare a private hardware-GDC capture bundle without replacing older assets.

Reads current camera flash parameters with the existing diagnostic SDK adapter,
then generates per-port LUTs using the existing board-side GDC compiler.
Does not start BEV, BPU, ROS, navigation, or modify system libraries.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys

PROJECT = Path(__file__).resolve().parents[1]
SSH = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
       '-o', 'StrictHostKeyChecking=yes', 'root@192.168.10.100']


def run(args, **kwargs):
    return subprocess.run(args, check=True, timeout=60, **kwargs)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--assets', required=True)
    parser.add_argument('--board-root', required=True)
    parser.add_argument('--ports', type=int, nargs='+', default=[0, 1, 2])
    args = parser.parse_args()
    root = Path(args.assets).resolve()
    if (root.parent != PROJECT/'runtime' or root.exists() or
            not re.fullmatch(r'/map/conbine_camera/[a-zA-Z0-9_-]+', args.board_root) or
            len(args.ports) != len(set(args.ports)) or not set(args.ports) <= set(range(4))):
        raise ValueError('Expected a new private runtime directory and distinct RX1 ports 0..3')
    remote = args.board_root
    template = PROJECT/'runtime/camera_bev_check_20261025'
    root.mkdir()
    for name in ('libcapture_pair.so', 'libj6m_surround_capture_raw.so', 'libcamera_info.so',
                 'gen_gdc', 'build_maps.py', 'stop.py', 'probe.py', 'vpm_probe.json'):
        shutil.copy2(template/name, root/name)
    shutil.copy2(PROJECT/'scripts/ground_camera_server.py', root/'server.py')
    camera = json.loads((template/'hb_probe.json').read_text())
    camera = {k: v for k, v in camera.items() if not k.startswith('port_') or int(k[5:]) in args.ports}
    camera['comment'] = 'Private RX1 GDC inputs; vehicle directions belong to the BEV config'
    for name in ('hb_j6dev.json', 'hb_probe.json'):
        (root/name).write_text(json.dumps(camera, indent=2)+'\n')
    original = json.loads((template/'vpm_gdc.json').read_text())['pipeline0']
    pipelines = {}
    for port in args.ports:
        # Node indices are local to each pipeline in the vendor VIO parser.
        pipeline = copy.deepcopy(original)
        pipeline['vin_node0']['cim']['input']['mipi']['vc_index'] = port
        pipeline['pym_node0']['common']['slot_id'] = 4+port
        pipelines['pipeline%d'%port] = pipeline
    (root/'vpm_gdc.json').write_text(json.dumps(pipelines, indent=2)+'\n')
    # Do not overwrite any earlier private candidate, even if it has the same name.
    run(SSH+['test ! -e '+shlex.quote(remote)+' && mkdir '+shlex.quote(remote)])
    initial = ['libcapture_pair.so', 'libj6m_surround_capture_raw.so', 'libcamera_info.so',
               'gen_gdc', 'probe.py', 'vpm_probe.json', 'hb_probe.json', 'hb_j6dev.json', 'vpm_gdc.json']
    run(['scp', '-q']+[str(root/n) for n in initial]+['root@192.168.10.100:'+remote+'/'])
    result = run(SSH+['env CAM_LOGLEVEL=2 LOGLEVEL=2 python3 -B '+remote+'/probe.py'],
                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    (root/'probe.log').write_text(result.stdout)
    print(result.stdout, end='', flush=True)
    run(['scp', '-q', 'root@192.168.10.100:'+remote+'/sensor_intrinsics.json', str(root)])
    rows = json.loads((root/'sensor_intrinsics.json').read_text())
    if {r['port'] for r in rows} != set(args.ports):
        raise RuntimeError('Not all requested cameras initialized; inspect '+str(root/'probe.log'))
    run([sys.executable, '-B', str(root/'build_maps.py')])
    run(['scp', '-q']+[str(root/('port%d.grid'%p)) for p in args.ports]+
        ['root@192.168.10.100:'+remote+'/'])
    for port in args.ports:
        run(SSH+[remote+'/gen_gdc '+remote+'/port%d.grid '%port+remote+'/port%d.bin'%port])
        run(['scp', '-q', 'root@192.168.10.100:'+remote+'/port%d.bin'%port, str(root)])
    names = ['libcapture_pair.so', 'hb_j6dev.json', 'vpm_gdc.json', 'sensor_intrinsics.json',
             'perspective_candidates.json', 'server.py', 'stop.py']+['port%d.bin'%p for p in args.ports]
    manifest = dict(purpose='four_camera_surround_rear_extension', accuracy_verified=False,
        ports=args.ports, sha256={n: hashlib.sha256((root/n).read_bytes()).hexdigest() for n in names})
    (root/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    run(['scp', '-q']+[str(root/n) for n in ('sensor_intrinsics.json', 'perspective_candidates.json',
                                          'server.py', 'stop.py', 'manifest.json')]+
        ['root@192.168.10.100:'+remote+'/'])
    check = ('import hashlib,json; from pathlib import Path; r=Path('+repr(remote)+'); '
             'm=json.loads((r/"manifest.json").read_text()); '
             'assert all(hashlib.sha256((r/n).read_bytes()).hexdigest()==s for n,s in m["sha256"].items()); '
             'print("CAPTURE_BUNDLE_VERIFIED",m["ports"])')
    run(SSH+['python3 -B -c '+shlex.quote(check)])
    print('Assets:', root, 'Board:', remote, flush=True)


if __name__ == '__main__':
    main()
