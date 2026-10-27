#!/usr/bin/env python3
"""Exercise MID360-only scan loss and complete mapping in a loopback-only sandbox."""
import collections
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
import xmlrpc.client

if [name for _,name in socket.if_nameindex()]!=['lo']:
    raise RuntimeError('Synthetic mapping validation requires a loopback-only network namespace')
WS=Path(__file__).resolve().parents[1]
ROOT=WS/'validation/mid360_only_20261024'/('mapping-'+str(time.time_ns()))
ROOT.mkdir(parents=True)
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11593',ROS_IP='127.0.0.1',
                  ROS_HOME=str(ROOT/'ros-home'),ROS_LOG_DIR=str(ROOT/'ros-logs'))
children=[];stopping=threading.Event();publish_scans=threading.Event();publish_scans.set()
def spawn(name,args,env=None):
    stream=(ROOT/(name+'.log')).open('w')
    p=subprocess.Popen(args,env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    children.append((p,stream));return p
def wait(predicate,description,seconds=25):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        if predicate():return
        time.sleep(.05)
    raise RuntimeError('Timeout: '+description)
try:
    spawn('master',['roscore','-p','11593'])
    def master_ready():
        try:return xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/probe')[0]==1
        except OSError:return False
    wait(master_ready,'private master')
    import rospy,rosgraph,yaml
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan,PointCloud2,PointField
    from sensor_msgs import point_cloud2
    from std_msgs.msg import Bool,Header
    rospy.init_node('synthetic_mid360_mapping',disable_signals=True)
    scan_pub=rospy.Publisher('/mid360/scan',LaserScan,queue_size=5)
    odom_pub=rospy.Publisher('/Odometry',Odometry,queue_size=5)
    cloud_pub=rospy.Publisher('/cloud_registered',PointCloud2,queue_size=5)
    lock=threading.RLock();seen=[];original={};active=[False]
    def output(scan):
        with lock:seen.append((scan.header.seq,scan.header.stamp.to_nsec(),scan.header.frame_id))
    def health(msg):active[0]=msg.data
    rospy.Subscriber('/scan',LaserScan,output,queue_size=100)
    rospy.Subscriber('/avoidance/mid360_active',Bool,health,queue_size=10)
    guard=spawn('guard',[sys.executable,'-B',str(WS/'src/scripts/robot_bringup/scripts/mid360_scan_guard.py')])
    points=[(4.*math.cos(i*.05),4.*math.sin(i*.05),-.4,1.) for i in range(126)]
    fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1)
            for i,n in enumerate(('x','y','z','intensity'))]
    def publish():
        seq=0
        while not stopping.wait(.05):
            now=rospy.Time.now();seq+=1
            odom=Odometry();odom.header.stamp=now;odom.header.frame_id='camera_init'
            odom.child_frame_id='body';odom.pose.pose.orientation.w=1.;odom_pub.publish(odom)
            cloud_pub.publish(point_cloud2.create_cloud(Header(stamp=now,frame_id='camera_init'),fields,points))
            if publish_scans.is_set():
                scan=LaserScan();scan.header.seq=seq;scan.header.stamp=now;scan.header.frame_id='base_link'
                scan.angle_min=-math.pi;scan.angle_max=math.pi;scan.angle_increment=math.pi/720
                scan.range_min=.5;scan.range_max=12.;scan.ranges=[4.]*1441
                with lock:original[seq]=now.to_nsec()
                scan_pub.publish(scan)
    thread=threading.Thread(target=publish,daemon=True);thread.start()
    wait(lambda:active[0] and len(seen)>4,'MID360-only guard ready')
    env=dict(os.environ,STATIC_MAP_OUTPUT_ROOT=str(ROOT/'map_sets'),MAPPING_TOPIC_WAIT_SEC='5')
    mapper=spawn('mapping',['bash',str(WS/'scripts/global_mapping_session.sh')],env)
    master=rosgraph.Master(rospy.get_name())
    def mappers_ready():
        if mapper.poll() is not None:
            raise RuntimeError('Mapping exited before readiness: '+(ROOT/'mapping.log').read_text()[-2500:])
        try:
            subs=master.getSystemState()[1]
            return any(t=='/cloud_registered' and '/voxel_cloud_mapper' in ns for t,ns in subs) and any(t=='/mid360/scan' and '/mid360_grid_mapper' in ns for t,ns in subs)
        except OSError:return False
    wait(mappers_ready,'both real mapper implementations subscribe')
    time.sleep(3.)
    mapper.send_signal(signal.SIGINT)
    assert mapper.wait(timeout=30)==0,(ROOT/'mapping.log').read_text()[-3000:]
    manifests=list((ROOT/'map_sets').glob('map_*/manifest.yaml'));assert len(manifests)==1
    target=manifests[0].parent;manifest=yaml.safe_load(manifests[0].read_text())
    meta=yaml.safe_load((target/'map_2d/mapping_info.yaml').read_text())
    assert manifest['status']=='complete' and manifest['map_2d']['occupancy_source']=='mid360_only'
    assert meta['scan_topic']=='/mid360/scan' and meta['horizontal_fov_deg']==360.
    assert meta['integrated_scans']>=5 and meta['occupied_cells']>0
    for rel in ('map_3d/map.pcd','map_2d/map.pgm','map_fused_2d/map.pgm'):
        assert (target/rel).stat().st_size>20,rel
    # A missing MID360 never causes the guard to repeat old scans or use a fallback sensor.
    publish_scans.clear();time.sleep(.9)
    assert not active[0],'source lease stayed active after MID360 stopped'
    before=len(seen);time.sleep(.25);assert len(seen)==before,'old scan was republished'
    with lock:
        source_stamps=set(original.values())
        # ROS publishers assign their own header.seq; source time/frame must survive the relay.
        assert all(stamp in source_stamps and frame=='base_link' for _,stamp,frame in seen),'source time/frame was rewritten'
    assert not any(t.startswith('/dual_lidar/') for section in master.getSystemState()[:2] for t,_ in section)
    result={'network_interfaces':['lo'],'hardware_connected':False,'checks_passed':10,
        'map_set':str(target),'occupancy_source':manifest['map_2d']['occupancy_source'],
        'integrated_scans':meta['integrated_scans'],'occupied_cells':meta['occupied_cells'],
        'forwarded_scans':len(seen),'timestamp_identity_preserved':True,'missing_mid360_disables_source':True,
        'stale_scans_not_republished':True,'ld19_topics_registered':False}
    (ROOT/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))
finally:
    stopping.set()
    if 'rospy' in locals():rospy.signal_shutdown('Synthetic mapping validation complete')
    for p,stream in reversed(children):
        for sig,seconds in ((signal.SIGINT,8),(signal.SIGTERM,3),(signal.SIGKILL,2)):
            if p.poll() is not None:break
            os.killpg(p.pid,sig)
            try:p.wait(timeout=seconds)
            except subprocess.TimeoutExpired:pass
        stream.close()
