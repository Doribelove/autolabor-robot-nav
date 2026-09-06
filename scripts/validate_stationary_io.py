#!/usr/bin/env python3
"""Isolated GUI subscription and stationary ZED test. No chassis/control nodes."""
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import time
import xmlrpc.client

from benchmark_cloud_enhancer import stats, stop

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--with-vision', action='store_true')
    args = parser.parse_args()
    prefix = 'stationary_vision_' if args.with_vision else 'stationary_'
    env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:11475', ROS_IP='127.0.0.1',
               QT_QPA_PLATFORM='offscreen')
    env.pop('ROS_HOSTNAME', None)
    os.environ.update(env)
    with socket.socket() as probe:
        if probe.connect_ex(('127.0.0.1', 11475)) == 0:
            raise RuntimeError('stationary test port is occupied')
    import rospy
    from sensor_msgs.msg import Image, CameraInfo, PointCloud2
    from std_msgs.msg import UInt64
    master_api = xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
    output = {}
    logs = []
    children = []

    def launch(name, command):
        log = (ROOT / 'validation' / (prefix + name + '.log')).open('w')
        logs.append(log)
        process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT)
        children.append(process)
        return process

    def subscribers(node):
        state = master_api.getSystemState('/stationary_verifier')[2]
        return sorted(topic for topic, nodes in state[1] if node in nodes)

    def ensure_no_chassis_publishers():
        state = master_api.getSystemState('/stationary_verifier')[2]
        active = {topic: nodes for topic, nodes in state[0]
                  if topic in ('/cmd_vel', '/cmd_vel_safe', '/canbus_cmd') and nodes}
        if active:
            raise RuntimeError('Unexpected motion publishers: ' + repr(active))

    try:
        launch('master', ['roscore', '-p', '11475'])
        deadline = time.monotonic() + 15
        while True:
            try:
                master_api.getPid('/stationary_verifier')
                break
            except OSError:
                if time.monotonic() > deadline:
                    raise
                time.sleep(.1)
        rospy.init_node('stationary_verifier', disable_signals=True)
        metadata = rospy.Publisher('/fast_lio/body_point_count', UInt64, queue_size=2)
        cloud = rospy.Publisher('/cloud_registered_body', PointCloud2, queue_size=2)
        gui_results = []
        for optimized in (() if args.with_vision else (False, True)):
            command = ['roslaunch', 'autolabor_operator_gui', 'operator_gui.launch', 'enable_rviz:=false']
            if optimized:
                command.append('cloud_point_count_topic:=/fast_lio/body_point_count')
            gui = launch('gui_' + str(optimized), command)
            deadline = time.monotonic() + 20
            expected = '/fast_lio/body_point_count' if optimized else '/cloud_registered_body'
            while expected not in subscribers('/autolabor_operator_gui'):
                if gui.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('GUI failed to subscribe: ' + expected)
                time.sleep(.1)
            for _ in range(30):
                metadata.publish(UInt64(data=6471))
                time.sleep(.1)
            topics = subscribers('/autolabor_operator_gui')
            assert ('/cloud_registered_body' in topics) == (not optimized)
            assert ('/fast_lio/body_point_count' in topics) == optimized
            ensure_no_chassis_publishers()
            gui_results.append(dict(optimized=optimized, subscriptions=topics, alive=gui.poll() is None))
            stop(gui)
            time.sleep(.5)
        output['gui_subscription_checks'] = gui_results

        samples = {}
        def callback(message, topic):
            record = samples.setdefault(topic, dict(count=0, first=time.monotonic(), last=0,
                                                     width=message.width, height=message.height))
            record['count'] += 1
            record['last'] = time.monotonic()
        subscriptions = [rospy.Subscriber(topic, kind, callback, callback_args=topic, queue_size=1)
                         for topic, kind in (('/fod_camera/image_raw', Image),
                                             ('/fod_camera/depth_registered', Image),
                                             ('/fod_camera/camera_info', CameraInfo))]
        tegra = launch('tegrastats', ['tegrastats', '--interval', '1000'])
        vision_received = []
        command = ['roslaunch', 'robot_bringup', 'zed2_camera.launch']
        if args.with_vision:
            from autolabor_fod_msgs.msg import FodDetectionArray
            subscriptions.append(rospy.Subscriber('/fod/detections', FodDetectionArray,
                                                  lambda message: vision_received.append(time.monotonic()), queue_size=1))
            command = ['roslaunch', 'autolabor_fod_vision', 'zed_fod_detection.launch',
                       'start_camera:=true', 'backend:=detect_and_classify',
                       'enable_image_quality_controller:=false', 'enable_clip_filter:=false',
                       'runtime_token:=isolated_stationary_validation']
            for argument, key in (
                ('serial_number', 'NVIDIA_ZED_SERIAL'),
                ('detector_python', 'NVIDIA_DETECTOR_PYTHON'),
                ('ultralytics_root', 'NVIDIA_DETECT_CLASSIFY_ULTRALYTICS_ROOT'),
                ('expected_model_sha256', 'NVIDIA_DETECT_CLASSIFY_DETECTOR_SHA256'),
                ('required_class_names', 'NVIDIA_DETECT_CLASSIFY_CLASSIFIER_CLASS_NAMES'),
                ('two_stage_detector_weights', 'NVIDIA_DETECT_CLASSIFY_DETECTOR_WEIGHTS'),
                ('two_stage_detector_sha256', 'NVIDIA_DETECT_CLASSIFY_DETECTOR_SHA256'),
                ('two_stage_classifier_weights', 'NVIDIA_DETECT_CLASSIFY_CLASSIFIER_WEIGHTS'),
                ('two_stage_classifier_sha256', 'NVIDIA_DETECT_CLASSIFY_CLASSIFIER_SHA256'),
            ):
                command.append(argument + ':=' + os.environ[key])
        camera = launch('zed', command)
        start = time.monotonic()
        status = 'STREAM_NOT_READY_WITHIN_TIMEOUT'
        camera_stats = []
        while time.monotonic() - start < (120 if args.with_vision else 45):
            ensure_no_chassis_publishers()
            if camera.poll() is not None:
                status = 'CAMERA_LAUNCH_EXITED'
                break
            try:
                uri = master_api.lookupNode('/stationary_verifier', '/zed2/zed_node')[2]
                pid = xmlrpc.client.ServerProxy(uri).getPid('/stationary_verifier')[2]
                cpu, rss = stats(pid)
                camera_stats.append(dict(t=time.monotonic()-start, cpu_seconds=cpu, rss_mib=rss))
            except (OSError, ValueError, xmlrpc.client.Error):
                pass
            now = time.monotonic()
            if len(samples) == 3:
                if any(now - value['last'] > 3 for value in samples.values()):
                    status = 'SENSOR_STALE_STOPPED'
                    break
                if all(now - value['first'] >= 15 for value in samples.values()) and (not args.with_vision or len(vision_received) >= 10):
                    status = 'RGB_DEPTH_INFO_LIVE_15_SECONDS'
                    break
            time.sleep(.5)
        output['camera'] = dict(status=status, launch_exit=camera.poll(), samples=samples,
                                process_samples=camera_stats, seconds=time.monotonic()-start)
        if args.with_vision:
            output['vision'] = dict(messages=len(vision_received), parameters={
                key: rospy.get_param('/fod_detector/' + key, None) for key in
                ('ready_token', 'backend', 'runtime_path', 'model_load_count_detector',
                 'model_load_count_classifier', 'detector_task', 'classifier_task')})
        stop(camera)
        stop(tegra)
        ensure_no_chassis_publishers()
        output['chassis_publishers_absent'] = True
    finally:
        for child in reversed(children):
            stop(child)
        for log in logs:
            log.close()
        (ROOT / 'validation' / (prefix + 'io.json')).write_text(json.dumps(output, indent=2) + '\n')
        print(json.dumps(output, indent=2))


if __name__ == '__main__':
    main()
