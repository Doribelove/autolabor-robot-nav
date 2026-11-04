#!/usr/bin/env python3
"""Isolated ROS recorder integration. Never connects to the vehicle's 11311 master."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import uuid
import xmlrpc.client

import three_party_test as core


def publisher(control):
    if os.environ.get('TRI_TEST_SIMULATION') != '1' or ':11311' in os.environ.get('ROS_MASTER_URI', ''):
        raise RuntimeError('Synthetic publishers require the explicitly isolated test master')
    import rospy
    from geometry_msgs.msg import Twist, TransformStamped
    from nav_msgs.msg import Odometry, OccupancyGrid
    from sensor_msgs.msg import NavSatFix, LaserScan, CompressedImage, CameraInfo, PointCloud2
    from tf2_msgs.msg import TFMessage
    from std_msgs.msg import String
    from autolabor_fod_msgs.msg import FodDetectionArray
    rospy.init_node('three_party_synthetic_sensors')
    pubs = {name: rospy.Publisher(name, typ, queue_size=1, latch=name.endswith('/status')) for name, typ in (
        ('/fod_navigation_mode/status', String), ('/fod_visual_servo/status', String), ('/gps/long_range/status', String),
        ('/fod/detections', FodDetectionArray), ('/gps/fix', NavSatFix), ('/odom', Odometry), ('/cmd_vel', Twist),
        ('/scan', LaserScan), ('/fod_camera/image_raw/compressed', CompressedImage),
        ('/fod_camera/camera_info', CameraInfo), ('/fod_navigation_mode/verification/status', String),
        ('/tf',TFMessage),('/cloud_filtered_for_scan',PointCloud2),('/move_base/local_costmap/costmap',OccupancyGrid))}
    import cv2
    import numpy as np
    noise = cv2.imencode('.jpg', np.random.RandomState(42).randint(0,256,(360,640,3),dtype=np.uint8))[1].tobytes()
    while not rospy.is_shutdown():
        values = core.read_json(control, {'state': 'FOD_COMPLETE_STOP', 'recovery_armed': False})
        values['reason'] = 'SIMULATION ONLY'
        for name, pub in pubs.items():
            if name == '/fod_navigation_mode/verification/status': msg=String(data=json.dumps(values.get('verification',{'phase':'IDLE'})))
            elif name == '/fod_navigation_mode/status': msg = String(data=json.dumps(values))
            elif name == '/gps/long_range/status': msg=String(data=json.dumps(values.get('gps',{'state':'IDLE','active':False})))
            elif name.endswith('/status'): msg = String(data=json.dumps({'state': 'DISABLED' if 'visual' in name else 'IDLE'}))
            else:
                msg = pub.data_class()
                if hasattr(msg, 'header'): msg.header.stamp = rospy.Time.now()
                if isinstance(msg, CompressedImage): msg.format = 'jpeg'; msg.data = noise
                if isinstance(msg, CameraInfo): msg.width=640;msg.height=360
                if isinstance(msg, NavSatFix): msg.latitude = 0.; msg.longitude = 0.
                if isinstance(msg, LaserScan): msg.ranges = [2., 3., 4.]; msg.angle_increment = .1
                if isinstance(msg, Odometry): msg.twist.twist.linear.x=.7
                if isinstance(msg, Twist): msg.linear.x=1.8
                if isinstance(msg, OccupancyGrid): msg.info.width=2;msg.info.height=2;msg.info.resolution=.1;msg.data=[0,0,100,0]
                if isinstance(msg, TFMessage):
                    tf=TransformStamped();tf.header.stamp=rospy.Time.now();tf.header.frame_id='map';tf.child_frame_id='base_link';tf.transform.rotation.w=1.;msg.transforms=[tf]
            pub.publish(msg)
        rospy.sleep(.1)


def integration(output):
    output.mkdir(parents=True, exist_ok=False)
    campaign = output / 'SIMULATION_ONLY'
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0)); port = sock.getsockname()[1]
    assert port != 11311
    uri = 'http://127.0.0.1:' + str(port)
    env = os.environ.copy()
    env.update(ROS_MASTER_URI=uri, ROS_HOME=str(output/'ros_home'), ROS_LOG_DIR=str(output/'ros_logs'), TRI_TEST_SIMULATION='1')
    env.pop('ROS_IP', None); env.pop('ROS_HOSTNAME', None)
    os.environ.update(env)
    os.environ.pop('ROS_IP', None); os.environ.pop('ROS_HOSTNAME', None)
    processes = []
    logs = []
    active_records = []
    try:
        for name, command in [('master', ['roscore','-p',str(port)])]:
            f=(output/(name+'.log')).open('wb');logs.append(f)
            p=subprocess.Popen(command,env=env,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p)
        deadline=time.monotonic()+15
        while True:
            try:
                if xmlrpc.client.ServerProxy(uri).getPid('/test')[0]==1:break
            except OSError: pass
            if time.monotonic()>deadline:raise RuntimeError('isolated master did not start')
            time.sleep(.1)
        control=output/'synthetic_control.json'
        core.write_json(control, {'state':'FOD_COMPLETE_STOP','recovery_armed':False})
        f=(output/'publisher.log').open('wb');logs.append(f)
        p=subprocess.Popen([sys.executable,__file__,'--publisher',str(control)],env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p)
        time.sleep(2)
        f=(output/'unrelated_recorder.log').open('wb');logs.append(f)
        other=subprocess.Popen(['rosbag','record','-O',str(output/'unrelated'),'/odom','__name:=unrelated_test_recorder'],env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append(other)
        config=core.read_json(core.CONFIG);config['simulation']=True
        core.write_json(output/'simulation_config.json',config)
        request=dict(core.parse_command('测试 白天 机坪 pt6'),raw_command='SIMULATED 测试 白天 机坪 pt6')
        result=core.start(campaign,request,output/'simulation_config.json')
        directory=core.resolve_session(campaign,result['session_id']);active_records.append(directory)
        assert result['result']=='recording',result
        first_pid=core.read_json(directory/'recorder_process.json')['pid']
        assert Path('/proc/%d/exe'%first_pid).resolve().name=='record', 'saved PID must be the actual C++ bag writer'
        assert core.start(campaign,request,output/'simulation_config.json')['result']=='already_recording'
        conflict=dict(request,point='pt1')
        assert core.start(campaign,conflict,output/'simulation_config.json')['result']=='conflict'
        assert core.start(campaign,dict(request,retest=True),output/'simulation_config.json')['result']=='conflict'
        assert core.read_json(directory/'recorder_process.json')['pid']==first_pid
        assert core.read_json(directory/'session.json')['task_started_at'] is None
        for state, armed in [('GPS_ACTIVE',False),('GPS_ACTIVE',True),('FOD_ACTIVE',False),('FOD_COMPLETE_STOP',False)]:
            core.write_json(control,{'state':state,'recovery_armed':armed})
            time.sleep(.5)
        state=core.read_json(directory/'session.json')
        assert state['task_started_at'] is not None
        assert state['software_completed_at'] is not None
        assert state['recovery_completed_at'] is None
        end=core.iso_ns(time.time_ns())
        time.sleep(.3)
        result=core.stop(campaign,directory,'complete','simulated completion',source='isolated simulation clock',actual_time=end)
        assert result['result']=='stopped',result
        check=core.read_json(directory/'check.json')
        assert check['recording_integrity_ok'],check
        assert check['task_time_covered'] is True,check
        assert all(check['critical_time_coverage'].values()),check
        assert (directory/'finalized.json').exists(), 'stop must wait for validation and report'
        assert other.poll() is None,'unrelated recorder was stopped'
        assert not list((directory/'bags').glob('*.active'))
        second=core.start(campaign,dict(request,retest=True),output/'simulation_config.json')
        d2=core.resolve_session(campaign,second['session_id']);active_records.append(d2)
        assert d2!=directory and core.read_json(d2/'session.json')['retest_of']==directory.name
        stopped=core.stop(campaign,d2,'abort','simulated aborted attempt')
        assert stopped['result']=='stopped',stopped
        assert core.read_json(d2/'session.json')['phase']=='aborted'
        demo=core.start(campaign,dict(core.parse_command('垃圾回收判定开始'),raw_command='SIMULATION ONLY 垃圾回收判定开始'))
        d3=core.resolve_session(campaign,demo['session_id']);active_records.append(d3)
        assert demo['result']=='recording',demo
        for phase in ('PREPARING','REVERSING','OBSERVING','PASSED'):
            core.write_json(control,{'state':'FOD_VERIFY','recovery_armed':False,
                                    'verification':{'phase':phase,'result':'PASSED' if phase=='PASSED' else None}})
            time.sleep(.4)
        assert core.read_json(d3/'session.json')['verification']['result']=='PASSED'
        assert core.read_json(d3/'session.json')['recovery_success'] is None
        stopped=core.stop(campaign,d3,'abort','SIMULATION ONLY; not a physical recovery')
        assert stopped['result']=='stopped',stopped
        cfg=core.read_json(d3/'capture_config.json');check=core.read_json(d3/'check.json')
        critical=[e['name'] for e in cfg['topics'] if e['critical']]
        for name in ('/fod_camera/image_raw/compressed','/fod_camera/camera_info','/fod/detections','/fod_navigation_mode/verification/status'):
            assert name in critical and check['counts'][name]>0,(name,check)
        assert check['recording_integrity_ok'],check
        assert other.poll() is None,'unrelated recorder was stopped during demo finalization'
        obstacle=core.start(campaign,dict(core.parse_command('避障测试'),raw_command='SIMULATION ONLY 避障测试'))
        d4=core.resolve_session(campaign,obstacle['session_id']);active_records.append(d4)
        assert obstacle['result']=='recording',obstacle
        assert core.read_json(d4/'session.json')['light'] is None
        core.set_info(d4,{'light':'白天','scene':'机坪'},'isolated simulation metadata')
        assert core.start(campaign,core.parse_command('避障测试'))['result']=='already_recording'
        assert core.start(campaign,core.parse_command('避障测试夜晚滑行道'))['result']=='conflict'
        core.write_json(control,{'state':'GPS_ACTIVE','gps':{'state':'ACTIVE','active':True,'route_id':'sim-obstacle'}})
        time.sleep(2.5)
        core.manual_event(d4,'obstacle_encounter','isolated simulation clock',actual_time=core.iso_ns(time.time_ns()))
        time.sleep(.5)
        core.write_json(control,{'state':'GPS_ACTIVE','gps':{'state':'COMPLETE','active':False,'route_id':'sim-obstacle'}})
        time.sleep(.5)
        end=core.iso_ns(time.time_ns());time.sleep(.3)
        stopped=core.stop(campaign,d4,'complete','SIMULATION ONLY end; physical bypass unconfirmed',source='isolated simulation clock',actual_time=end)
        assert stopped['result']=='stopped',stopped
        cfg=core.read_json(d4/'capture_config.json');check=core.read_json(d4/'check.json');state=core.read_json(d4/'session.json')
        for name in ('/fod_camera/image_raw/compressed','/scan','/cloud_filtered_for_scan','/move_base/local_costmap/costmap','/tf','/odom','/cmd_vel'):
            assert check['counts'].get(name,0)>0,(name,check)
        assert check['recording_integrity_ok'] and check['task_time_covered'],check
        assert abs(check['pre_obstacle_actual_speed']['last_before_mps']-.7)<1e-9
        assert state['recovery_success'] is None and state['obstacle'].get('bypassed') is None
        assert state['recovery_completed_at'] is None and state['obstacle']['completed_at']
        assert other.poll() is None,'unrelated recorder was stopped during obstacle finalization'
        from three_party_report import report
        summary=report(campaign)
        assert summary['recovery_attempts']==2 and summary['recovery_slots_with_attempts']==1
        assert other.poll() is None
        core.write_json(output/'result.json', {'passed':True,'simulation':True,'master_uri':uri,'sessions':[directory.name,d2.name,d3.name,d4.name],
            'verified':['real bag writes and key messages','old latched state ignored','activation timestamp from new state','software completion not physical pickup',
                        'duplicate command idempotency','different command conflict','retest preserved','graceful split-bag validation','task time coverage','unrelated recorder unaffected',
                        'obstacle profile records camera/scan/cloud/costmap/TF/feedback','bare obstacle command and later metadata','0.7 m/s actual feedback separated from 1.8 command','obstacle stop does not invent physical bypass'], 'summary':summary})
        print(json.dumps(core.read_json(output/'result.json'),ensure_ascii=False,indent=2))
    finally:
        for directory in active_records:
            if core.alive(core.read_json(directory/'recorder_process.json')):
                core.write_json(directory/'stop_request.json',{'disposition':'abort','reason':'simulation cleanup'})
                from three_party_capture import stop_owned_recorder
                stop_owned_recorder(directory)
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
        for f in logs:f.close()


if __name__=='__main__':
    if len(sys.argv)==3 and sys.argv[1]=='--publisher':publisher(Path(sys.argv[2]))
    elif len(sys.argv)==2:integration(Path(sys.argv[1]))
    else:raise SystemExit('Usage: test_three_party_capture_ros.py NEW_SIMULATION_OUTPUT_DIRECTORY')
