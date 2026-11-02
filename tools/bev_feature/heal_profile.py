#!/usr/bin/env python3
"""Profile the exact recorded real-view tensors using official hrt_model_exec."""
import argparse
import json
from pathlib import Path
import shlex
import subprocess


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', type=Path, required=True)
    a = p.parse_args()
    d = json.loads((a.run/'deployment.json').read_text())
    ssh = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', 'root@192.168.10.100']
    def remote(args, **kwargs):
        return subprocess.run(ssh+[shlex.join(args)], check=True, timeout=60, **kwargs)
    files = [a.run/('input_%02d.bin'%i) for i in range(14)]
    subprocess.run(['scp', '-q', *map(str, files), 'root@192.168.10.100:'+d['board']+'/'],
                   check=True, timeout=60)
    reference = '/map/robot_j6m_navigation_20260907/bpu_lab_20260905'
    prefix = ['flock', '-n', reference+'/preview_worker.lock', 'env',
              'LD_LIBRARY_PATH='+d['runtime']+'/lib:/usr/hobot/lib',
              reference+'/vendor/runtime_4_9_2/bin/hrt_model_exec']
    inputs = ','.join(d['board']+'/'+f.name for f in files)
    with (a.run/'model_info.log').open('w') as out:
        remote(prefix+['model_info', '--model_file', d['runtime']+'/model.hbm'],
               stdout=out, stderr=subprocess.STDOUT)
    with (a.run/'perf.log').open('w') as out:
        remote(prefix+['perf', '--model_file', d['runtime']+'/model.hbm',
               '--input_file', inputs, '--core_id', '1', '--thread_num', '1',
               '--frame_count', '100', '--profile_path', d['board']+'/profile'],
               stdout=out, stderr=subprocess.STDOUT)
    subprocess.run(['scp', '-qr', 'root@192.168.10.100:'+d['board']+'/profile', str(a.run)],
                   check=True, timeout=30)


if __name__ == '__main__':
    main()
