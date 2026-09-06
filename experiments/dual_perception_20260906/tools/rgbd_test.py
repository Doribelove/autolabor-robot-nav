#!/usr/bin/env python3
"""Owned RGB-D sensor test on a private ROS graph. No chassis/nav nodes."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time
import numpy as np

LAB=Path(__file__).resolve().parents[1];WS=LAB.parents[1]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--seconds',type=int,default=60)
    parser.add_argument('--centerpoint-load',action='store_true')
    parser.add_argument('--mid360',action='store_true');parser.add_argument('--replay',action='store_true')
    parser.add_argument('--qt',action='store_true');args=parser.parse_args()
    expected_master='http://192.168.10.50:11571' if (args.mid360 or args.replay) else 'http://127.0.0.1:11571'
    if os.environ.get('ROBOT_OPTIMIZED_SANDBOX')!='1' or os.environ.get('ROS_MASTER_URI')!=expected_master:
        raise RuntimeError('Use isolated setup_env and private ROS master 11571')
    if not 10<=args.seconds<=300:raise ValueError('Test duration must be 10..300 seconds')
    if sum((args.mid360,args.replay,args.centerpoint_load))>1:raise ValueError('Select only one point-cloud input')
    for unit in ('autolabor-dual-host.service','autolabor-optimized-20260905.service'):
        state=subprocess.check_output(['systemctl','--user','show',unit,'-p','ActiveState','--value'],text=True).strip()
        if state not in ('inactive','failed'):raise RuntimeError('Navigation active: '+unit)
    for host,port in [('127.0.0.1',11311),('127.0.0.1',11571),('192.168.10.100',11311)]:
        with socket.socket() as probe:
            probe.settimeout(.5)
            if probe.connect_ex((host,port))==0:raise RuntimeError('ROS master already active')
    subprocess.run([str(WS/'scripts/zed_camera_check.sh'),'--wait','0'],check=True)
    directory=LAB/'results'/time.strftime('rgbd_%Y%m%d_%H%M%S');directory.mkdir(parents=True)
    mount=subprocess.check_output(['python3',str(WS/'scripts/zed_mount_args.py')],text=True).splitlines()
    shutdown=threading.Event();signal.signal(signal.SIGINT,lambda *_:shutdown.set());signal.signal(signal.SIGTERM,lambda *_:shutdown.set())
    received=[];statuses=[];rgb=[];depth=[];cp=[];cp_errors=[];children=[];tf_pose=None
    live_cp=[];cloud_stamps=[];obstacle_counts=[];remote=None;published_topics=[];preview=[]
    cp_observations=[];control_messages=[];fixture=None;fixture_pub=None
    def centerpoint_load():
        from autolabor_bpu_perception.protocol import Client
        client=None
        try:
            client=Client(remote=True);rng=np.random.default_rng(42)
            points=np.zeros((10000,5),np.float32);points[:,:2]=rng.uniform(-4,4,(10000,2));points[:,2]=-1;points[:,3]=20
            sequence=0
            while not shutdown.wait(.205):
                sequence+=1;start=time.monotonic()
                try:
                    result=client.infer(dict(model='centerpoint',sequence=sequence,stamp_ns=time.time_ns(),points=len(points)),points.tobytes())
                    cp.append(dict(roundtrip_ms=(time.monotonic()-start)*1000,**result['timing']))
                except Exception as error:cp_errors.append(str(error));break
        finally:
            if client:client.close()
    worker=threading.Thread(target=centerpoint_load,daemon=True)
    with (directory/'roslaunch.log').open('w') as log:
        launch=subprocess.Popen(['roslaunch','-p','11571',str(LAB/'rgbd_test.launch'),
            'mid360:='+str(args.mid360).lower(),'qt:='+str(args.qt).lower()]+mount,
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True);children.append(launch)
        try:
            import rospy
            from autolabor_bpu_perception.msg import DetectedObjects
            from sensor_msgs.msg import Image,PointCloud2,PointField
            from costmap_converter.msg import ObstacleArrayMsg
            from std_msgs.msg import String
            from geometry_msgs.msg import Twist,PoseStamped
            import tf2_ros
            start=time.monotonic()
            while time.monotonic()-start<15:
                if launch.poll() is not None:raise RuntimeError('Sensor launch exited')
                with socket.socket() as probe:
                    if probe.connect_ex(('127.0.0.1',11571))==0:break
                time.sleep(.1)
            rospy.init_node('laserMapping' if args.replay else 'rgbd_static_observer',disable_signals=True)
            def objects(m):
                if m.valid:received.append(dict(t=time.monotonic(),stamp=m.header.stamp.to_sec(),age_ms=m.latency_ms,
                    frame=m.header.frame_id,objects=[dict(name=o.class_name,id=o.object_id,valid=o.position_valid,
                    depth=o.depth_m,point=[o.position.x,o.position.y,o.position.z]) for o in m.objects]))
            def status(m):
                try:statuses.append(json.loads(m.data))
                except ValueError:pass
            subs=[rospy.Subscriber('/perception/fcos/objects',DetectedObjects,objects,queue_size=10),
                  rospy.Subscriber('/perception/fcos/status',String,status,queue_size=10),
                  rospy.Subscriber('/fod_camera/image_raw',Image,lambda m:rgb.append(m.header.stamp.to_sec()),queue_size=1),
                  rospy.Subscriber('/fod_camera/depth_registered',Image,lambda m:depth.append(m.header.stamp.to_sec()),queue_size=1)]
            subs.append(rospy.Subscriber('/fod/bpu_preview/image',Image,lambda m:preview.append(m.header.stamp.to_sec()),queue_size=1))
            for topic,kind in [('/cmd_vel',Twist),('/cmd_vel_raw',Twist),('/move_base_simple/goal',PoseStamped)]:
                subs.append(rospy.Subscriber(topic,kind,lambda m,t=topic:control_messages.append(t),queue_size=10))
            if args.mid360 or args.replay:
                subs.extend([
                    rospy.Subscriber('/perception/centerpoint/status',String,lambda m:live_cp.append(json.loads(m.data)),queue_size=10),
                    rospy.Subscriber('/perception/centerpoint/objects',DetectedObjects,lambda m:cp_observations.append(dict(valid=m.valid,age=rospy.Time.now().to_sec()-m.header.stamp.to_sec(),objects=len(m.objects))),queue_size=10),
                    rospy.Subscriber('/cloud_registered_body',PointCloud2,lambda m:cloud_stamps.append(m.header.stamp.to_sec()),queue_size=1),
                    rospy.Subscriber('/perception/centerpoint/obstacles_candidate',ObstacleArrayMsg,lambda m:obstacle_counts.append(len(m.obstacles)),queue_size=1)])
                remote=subprocess.Popen(['ssh','-T','-o','BatchMode=yes','root@192.168.10.100','python3','-u',
                    '/map/robot_j6m_optimized_20260905/bpu_perception_20260906/tools/run_ros_client.py','replay' if args.replay else 'sensors'],
                    stdin=subprocess.PIPE,stdout=log,stderr=subprocess.STDOUT)
                children.append(remote)
            if args.replay:
                # Explicit synthetic flat scene: verifies transport and lease,
                # cannot establish MID360 accuracy or physical performance.
                rng=np.random.default_rng(42);points=np.zeros((10000,4),np.float32)
                points[:,:2]=rng.uniform(-4,4,(10000,2));points[:,2]=-1;points[:,3]=20
                points[:,:3]+=[-.011,-.02329,.04412]
                fixture=PointCloud2();fixture.header.frame_id='body';fixture.height=1;fixture.width=len(points)
                fixture.point_step=16;fixture.row_step=16*len(points);fixture.is_dense=True
                fixture.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z','intensity'))]
                fixture.data=points.astype('<f4').tobytes();fixture_pub=rospy.Publisher('/cloud_registered_body',PointCloud2,queue_size=1)
            buffer=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buffer)
            if args.centerpoint_load:worker.start()
            while time.monotonic()-start<args.seconds and not shutdown.wait(.2):
                if launch.poll() is not None:raise RuntimeError('Sensor launch exited early')
                if remote:
                    if remote.poll() is not None:raise RuntimeError('Remote sensor client exited')
                    remote.stdin.write(b'tick\n');remote.stdin.flush()
                if fixture_pub and time.monotonic()-start<args.seconds-4:
                    fixture.header.stamp=rospy.Time.now();fixture_pub.publish(fixture)
            published_topics=rospy.get_published_topics()
            try:
                tf=buffer.lookup_transform('base_link','perception_zed_left_camera_optical_frame',rospy.Time(0),rospy.Duration(.2)).transform
                tf_pose=dict(position=[tf.translation.x,tf.translation.y,tf.translation.z],quaternion=[tf.rotation.x,tf.rotation.y,tf.rotation.z,tf.rotation.w])
            except Exception as error:tf_pose=dict(error=str(error))
        finally:
            shutdown.set()
            if remote:
                remote.stdin.close()
                try:remote.wait(timeout=18)
                except subprocess.TimeoutExpired:remote.terminate();remote.wait(timeout=5)
                children.remove(remote)
            if worker.is_alive():worker.join(timeout=5)
            for child in reversed(children):
                if child.poll() is None:
                    os.killpg(child.pid,signal.SIGINT)
                    try:child.wait(timeout=15)
                    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
            def hz(values):return (len(values)-1)/(values[-1]-values[0]) if len(values)>1 and values[-1]>values[0] else 0
            ages=[v['age_ms'] for v in received]
            report=dict(seconds=args.seconds,depth_mode='QUALITY',camera_fps=15,rgb_hz=hz(rgb),depth_hz=hz(depth),
                fcos_hz=hz([v['t'] for v in received]),fcos_results=len(received),
                source_age_ms=dict(mean=float(np.mean(ages)),p95=float(np.percentile(ages,95))) if ages else {},
                metric_objects=sum(o['valid'] for r in received for o in r['objects']),tf_left_optical=tf_pose,
                last_status=statuses[-1] if statuses else {},centerpoint_synthetic_frames=len(cp),centerpoint_errors=cp_errors,
                centerpoint_infer_ms=float(np.mean([v['vendor_infer_ms'] for v in cp])) if cp else None,
                mid360_live=args.mid360,synthetic_ros_cloud=args.replay,cloud_hz=hz(cloud_stamps),centerpoint_live_results=len(live_cp),
                centerpoint_live_last=live_cp[-1] if live_cp else {},preview_frames=len(preview),
                candidate_obstacle_max=max(obstacle_counts) if obstacle_counts else 0,
                centerpoint_valid_results=sum(o['valid'] for o in cp_observations),
                centerpoint_tail_invalid=bool(cp_observations) and not any(o['valid'] for o in cp_observations[-10:]),
                control_messages=control_messages,
                control_topics_advertised=[p for p in published_topics if p[0] in ('/cmd_vel','/cmd_vel_raw','/move_base_simple/goal','/move_base/TebLocalPlannerROS/obstacles')])
            (directory/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
            (directory/'observations.json').write_text(json.dumps(dict(objects=received,statuses=statuses,centerpoint=cp,centerpoint_live=live_cp))+'\n')
            print(json.dumps(dict(directory=str(directory),**report),indent=2))
    if len(received)<20 or report['last_status'].get('depth_matched',0)<10:raise RuntimeError('RGB-D acceptance failed')
    if control_messages:raise RuntimeError('Unexpected control message in isolated test')
    if args.replay and (report['centerpoint_valid_results']<20 or not report['centerpoint_tail_invalid']):
        raise RuntimeError('Synthetic ROS CenterPoint/expiry acceptance failed')


if __name__=='__main__':main()
