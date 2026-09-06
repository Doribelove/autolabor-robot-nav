#!/usr/bin/env python3
"""Own one installed ROS launch process group until the SSH heartbeat ends."""
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time

LAB=Path('/map/robot_j6m_optimized_20260905/bpu_perception_20260906')
ROOT=LAB.parent/'rootfs'


def main():
    if Path(__file__).resolve().parents[1]!=LAB:raise RuntimeError('Wrong private directory')
    mode=sys.argv[1] if len(sys.argv)==2 else ''
    if mode not in ('companion','sensors','replay'):raise ValueError('Expected companion|sensors|replay')
    master='http://192.168.10.100:11311' if mode=='companion' else 'http://192.168.10.50:11571'
    launch='mid360_test.launch' if mode=='sensors' else 'centerpoint.launch'
    if not os.path.ismount(ROOT/'proc'):raise RuntimeError('Candidate chroot must be mounted by the unified lifecycle')
    command='source /opt/ros/noetic/setup.bash && source /opt/autolabor/ros/install/setup.bash && source /opt/autolabor/dual_host/current/setup.bash && exec roslaunch autolabor_bpu_perception '+launch
    child=subprocess.Popen(['chroot',str(ROOT),'/usr/bin/env','-u','ROS_HOSTNAME',
        'ROS_MASTER_URI='+master,'ROS_IP=192.168.10.100','ROS_HOME=/var/lib/autolabor/ros-home',
        'ROS_LOG_DIR=/var/log/autolabor/perception','/bin/bash','-c',command],
        stdin=subprocess.DEVNULL,start_new_session=True)
    stopped=[False]
    signal.signal(signal.SIGINT,lambda *_:stopped.__setitem__(0,True))
    signal.signal(signal.SIGTERM,lambda *_:stopped.__setitem__(0,True))
    last=time.monotonic();pending=b''
    try:
        while child.poll() is None and not stopped[0] and time.monotonic()-last<4:
            if select.select([sys.stdin],[],[],.2)[0]:
                chunk=os.read(sys.stdin.fileno(),4096)
                if not chunk:break
                pending+=chunk
                if len(pending)>128:raise ValueError('Oversized lifecycle input')
                while b'\n' in pending:
                    line,pending=pending.split(b'\n',1)
                    if line!=b'tick':raise ValueError('Unexpected lifecycle input')
                    last=time.monotonic()
    finally:
        if child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
    return child.returncode


if __name__=='__main__':sys.exit(main())
