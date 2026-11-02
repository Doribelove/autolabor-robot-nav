#!/usr/bin/env python3
"""Own the original operator Qt, private local ROS master and BEV YOLO worker."""
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import xmlrpc.client

PROJECT = Path(__file__).resolve().parents[1]
ROOT = PROJECT/'runtime/bev_qt'
STOP = threading.Event()


def main():
    ROOT.mkdir(parents=True, exist_ok=True)
    (ROOT/'yolo-config').mkdir(exist_ok=True)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: STOP.set())
    children, streams = [], []

    def spawn(name, args, env):
        stream = (ROOT/(name+'.log')).open('a')
        streams.append(stream)
        child = subprocess.Popen(args, env=env, stdout=stream, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        children.append(child)
        return child

    env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:11587', ROS_IP='127.0.0.1',
               ROS_HOME=str(ROOT/'ros-home'), ROS_LOG_DIR=str(ROOT/'ros-logs'),
               XDG_CONFIG_HOME=str(ROOT/'qt-config'), XDG_CACHE_HOME=str(ROOT/'cache'),
               BEV_PREVIEW_URL='http://127.0.0.1:18884', BEV_QT_SCREENSHOT=str(ROOT/'qt_bev.png'))
    env.pop('ROS_HOSTNAME', None)
    env.pop('SURROUND_PREVIEW_URL', None)
    for name in ['ros-home', 'ros-logs', 'qt-config', 'cache']:
        (ROOT/name).mkdir(exist_ok=True)
    try:
        # The video display does not require starting a navigation stack.
        master = spawn('roscore', ['/opt/ros/noetic/bin/roscore', '-p', '11587'], env)
        yolo_env = dict(env, YOLO_CONFIG_DIR=str(ROOT/'yolo-config'), OMP_NUM_THREADS='2',
                        OPENBLAS_NUM_THREADS='2')
        yolo = spawn('yolo', ['/home/slam/robot_ws/.venv/fod_yolo/bin/python3', '-s',
                              str(PROJECT/'scripts/bev_yolo_server.py')], yolo_env)
        deadline = time.monotonic()+12
        while time.monotonic() < deadline and not STOP.is_set():
            if master.poll() is not None:
                raise RuntimeError('Private Qt ROS master exited')
            try:
                if xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/bev_qt')[0] == 1:
                    break
            except OSError:
                pass
            STOP.wait(.1)
        else:
            raise RuntimeError('Private Qt ROS master did not become ready')
        # Status and relative detections are available in this display session.
        # There is no navigator on this private master; real cleaning uses live-start.
        spawn('bev_cleaner', ['roslaunch', str(PROJECT/'src/application/autolabor_fod_control/launch/bev_cleaning.launch')], env)
        gui = spawn('qt', [str(PROJECT/'devel/lib/autolabor_operator_gui/autolabor_operator_gui_node'),
                          '_initial_vision_page:=true', '_enable_rviz:=false',
                          '_window_title:=Autolabor · 环视 BEV 视野清扫'], env)
        while not STOP.wait(.2):
            if gui.poll() is not None:
                if gui.returncode:
                    raise RuntimeError('Operator Qt exited with '+str(gui.returncode))
                break
            if yolo.poll() is not None:
                raise RuntimeError('BEV YOLO worker exited; see yolo.log')
            if master.poll() is not None:
                raise RuntimeError('Private Qt ROS master exited')
    finally:
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGTERM)
                    child.wait(timeout=5)
        for stream in streams:
            stream.close()


if __name__ == '__main__':
    main()
