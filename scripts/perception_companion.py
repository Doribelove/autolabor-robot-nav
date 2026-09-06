#!/usr/bin/env python3
"""Attach both perception chains to an already owned, running candidate graph."""
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

WS=Path(__file__).resolve().parents[1]
REMOTE='/map/robot_j6m_optimized_20260905/bpu_perception_20260906/tools/'
SSH=['ssh','-T','-o','BatchMode=yes','-o','ConnectTimeout=4','root@192.168.10.100']


def main():
    if os.environ.get('ROBOT_OPTIMIZED_SANDBOX')!='1' or os.environ.get('ROS_MASTER_URI')!='http://192.168.10.100:11311':
        raise RuntimeError('Use candidate setup_env and the configured running ROS graph')
    subprocess.run([str(WS/'scripts/verify_master_owner.sh')],check=True)
    socket.setdefaulttimeout(3)
    lock=(WS/'runtime/perception_companion.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    import rosgraph
    master=rosgraph.Master('/perception_companion_guard')
    publishers=master.getSystemState()[0]
    for topic,nodes in publishers:
        if topic in ('/fod/bpu_preview/image','/perception/fcos/objects','/perception/centerpoint/objects') and nodes:
            raise RuntimeError('Perception topic already owned: '+topic)
    state=json.loads(subprocess.check_output(SSH+['python3',REMOTE+'service_manager.py','status'],text=True))
    started_service=not state['alive'];children=[];stopping=[False]
    signal.signal(signal.SIGINT,lambda *_:stopping.__setitem__(0,True))
    signal.signal(signal.SIGTERM,lambda *_:stopping.__setitem__(0,True))
    try:
        subprocess.run(SSH+['python3',REMOTE+'service_manager.py','start'],check=True)
        remote=subprocess.Popen(SSH+['python3','-u',REMOTE+'run_ros_client.py','companion'],stdin=subprocess.PIPE)
        children.append(remote)
        local=subprocess.Popen(['roslaunch','autolabor_fod_vision','fcos_rgbd.launch'],start_new_session=True)
        children.append(local)
        while not stopping[0] and all(c.poll() is None for c in children):
            master.getPid()
            remote.stdin.write(b'tick\n');remote.stdin.flush()
            time.sleep(1)
    finally:
        if children:
            children[0].stdin.close()
        if len(children)>1 and children[1].poll() is None:
            os.killpg(children[1].pid,signal.SIGINT)
        for child in reversed(children):
            try:child.wait(timeout=18)
            except subprocess.TimeoutExpired:
                # Remote stdin lease already stops its owned ROS group.
                child.terminate();child.wait(timeout=5)
        if started_service:subprocess.run(SSH+['python3',REMOTE+'service_manager.py','stop'],check=True)


if __name__=='__main__':main()
