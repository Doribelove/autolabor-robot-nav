#!/usr/bin/env python3
"""Private loopback-only Qt preview using real planning/parameter backends."""
import argparse
import copy
import datetime
import json
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import threading
import time
import xmlrpc.client
import xml.etree.ElementTree as ET

WS=Path(__file__).resolve().parents[1]
ROOT=Path(os.environ.get('NAV_TUNING_PREVIEW_DIR',str(WS/'runtime/navigation_tuning_preview')))
PORT=11543
children=[]
stop=threading.Event()


def wait(predicate, description, timeout=40):
    deadline=time.monotonic()+timeout
    while not stop.is_set() and time.monotonic()<deadline:
        if predicate():return
        time.sleep(.1)
    raise RuntimeError('timeout: '+description)


def spawn(name,args):
    stream=(ROOT/(name+'.log')).open('a')
    child=subprocess.Popen(args,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    children.append((child,stream));return child


def terminate(child):
    if child.poll() is not None:return
    os.killpg(child.pid,signal.SIGINT)
    try:child.wait(timeout=12)
    except subprocess.TimeoutExpired:
        os.killpg(child.pid,signal.SIGTERM)
        try:child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=3)


def prepare():
    ROOT.mkdir(parents=True,exist_ok=True)
    config=ROOT/'coverage.yaml'
    if not config.exists():shutil.copy2(WS/'src/application/autolabor_coverage/config/coverage.yaml',config)
    map_set=ROOT/'map_set';map_set.mkdir(exist_ok=True)
    width,height=160,120
    pixels=bytes(0 if x in (0,width-1) or y in (0,height-1) else 254
                 for y in range(height) for x in range(width))
    (map_set/'map.pgm').write_bytes(('P5\n%d %d\n255\n'%(width,height)).encode()+pixels)
    (map_set/'map.yaml').write_text('image: map.pgm\nresolution: 0.1\norigin: [-8.0, -6.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    launch=ET.Element('launch')
    ET.SubElement(launch,'node',pkg='map_server',type='map_server',name='map_server',args=str(map_set/'map.yaml'),required='true')
    inc=ET.SubElement(launch,'include',file=str(WS/'src/application/autolabor_coverage/launch/coverage.launch'))
    ET.SubElement(inc,'arg',name='config_file',value=str(config))
    node=ET.SubElement(launch,'node',pkg='move_base',type='move_base',name='move_base',output='screen',required='true')
    for name,ns in [('costmap_common_static.yaml','global_costmap'),('costmap_common_static.yaml','local_costmap'),('global_costmap_static.yaml',''),('local_costmap_static.yaml','')]:
        attributes=dict(file=str(WS/'src/scripts/robot_bringup/config'/name),command='load')
        if ns:attributes['ns']=ns
        ET.SubElement(node,'rosparam',**attributes)
    ET.SubElement(node,'rosparam',file=str(WS/'src/navigation_arena/arena-rosnav-3D/arena_navigation/arena_local_planer/model_based/conventional/config/dingo/teb_local_planner_params_nomap.yaml'),command='load')
    original=ET.parse(WS/'src/scripts/robot_bringup/launch/navigation_j6m.launch').getroot()
    for item in original.find("node[@name='move_base']"):
        if item.tag=='param' and item.get('name','').startswith('CoverageGlobalPlanner/'):
            copied=copy.deepcopy(item);copied.attrib.pop('if',None);node.append(copied)
    params={'base_global_planner':'autolabor_coverage/CoverageGlobalPlanner',
            'base_local_planner':'teb_local_planner/TebLocalPlannerROS',
            'controller_frequency':10.,'planner_frequency':1.,'recovery_behavior_enabled':False,
            'TebLocalPlannerROS/odom_topic':'/odom','TebLocalPlannerROS/map_frame':'map',
            'TebLocalPlannerROS/treat_unknown_as_obstacle':True,
            'TebLocalPlannerROS/max_global_plan_lookahead_dist':4.}
    for key,value in params.items():
        ET.SubElement(node,'param',name=key,value=str(value).lower() if type(value)is bool else str(value))
    ET.SubElement(node,'remap',**{'from':'cmd_vel','to':'/preview/cmd_vel_teb'})
    ET.SubElement(node,'remap',**{'from':'/move_base_simple/goal','to':'/preview/navigation_input_disabled'})
    ET.ElementTree(launch).write(str(ROOT/'backend.launch'),encoding='unicode')


def main():
    args=argparse.ArgumentParser();args.add_argument('--self-test',action='store_true');options=args.parse_args()
    if [name for _,name in socket.if_nameindex()]!=['lo']:
        raise RuntimeError('Preview requires a private network namespace containing only loopback')
    assert os.environ['ROS_MASTER_URI']=='http://127.0.0.1:%d'%PORT
    prepare()
    for sig in (signal.SIGTERM,signal.SIGINT):signal.signal(sig,lambda *_:stop.set())
    try:
        spawn('roscore',['roscore','-p',str(PORT)])
        def master_ready():
            try:return xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/preview_probe')[0]==1
            except OSError:return False
        wait(master_ready,'private ROS master')
        import rospy
        import tf2_ros
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        rospy.init_node('navigation_tuning_preview',disable_signals=True)
        broadcaster=tf2_ros.TransformBroadcaster();odom=rospy.Publisher('/odom',Odometry,queue_size=1)
        def tick(_):
            transform=TransformStamped();transform.header.stamp=rospy.Time.now()
            transform.header.frame_id='map';transform.child_frame_id='base_link';transform.transform.rotation.w=1.
            broadcaster.sendTransform(transform)
            message=Odometry();message.header=copy.deepcopy(transform.header);message.child_frame_id='base_link';message.pose.pose.orientation.w=1.
            odom.publish(message)
        timer=rospy.Timer(rospy.Duration(.05),tick)
        def backend_start():
            child=spawn('backend',['roslaunch','--wait',str(ROOT/'backend.launch')])
            for endpoint in ('/move_base/CoverageGlobalPlanner/set_parameters','/coverage_manager/set_parameters','/coverage/set_planning_defaults'):
                rospy.wait_for_service(endpoint,timeout=45)
            wait(lambda:rospy.get_param('/coverage_manager/tuning_persistence_protocol',0)==1 and rospy.get_param('/move_base/CoverageGlobalPlanner/tuning_persistence_protocol',0)==1,'permanent tuning backends')
            return child
        backend=backend_start()
        if options.self_test:
            import dynamic_reconfigure.client
            from autolabor_coverage.srv import SetCoveragePlanningDefaults,SetCoveragePlanningDefaultsRequest
            def clients():return (dynamic_reconfigure.client.Client('/move_base/CoverageGlobalPlanner',timeout=5),dynamic_reconfigure.client.Client('/coverage_manager',timeout=5))
            hybrid,transit=clients()
            assert hybrid.update_configuration({'hybrid_motion_step':.22})['hybrid_motion_step']==.22
            assert transit.update_configuration({'hybrid_transit_weight_viapoint':23.})['hybrid_transit_weight_viapoint']==23.
            call=SetCoveragePlanningDefaultsRequest();p=call.parameters
            for key,value in dict(operation_width_m=1.05,overlap_ratio=.15,allow_reverse=True,max_forward_speed_mps=.8,max_reverse_speed_mps=.3,max_angular_speed_rps=.6,linear_accel_mps2=1.,angular_accel_rps2=.5,direction_change_penalty_sec=.5,segment_handoff_penalty_sec=.5,transit_replan_period_sec=.5).items():setattr(p,key,value)
            response=rospy.ServiceProxy('/coverage/set_planning_defaults',SetCoveragePlanningDefaults)(call)
            assert response.success,response.message
            state=Path(os.environ['ROS_HOME'])/'autolabor_navigation/hybrid_tuning.json'
            backup=state.with_suffix('.saved');state.rename(backup);state.mkdir()
            try:
                actual=hybrid.update_configuration({'hybrid_motion_step':.21})
                assert actual['hybrid_motion_step']==.22,'write failure changed runtime'
            finally:state.rmdir();backup.rename(state)
            hybrid.close();transit.close();terminate(backend)
            rospy.delete_param('/coverage_manager');rospy.delete_param('/move_base')
            backend=backend_start();hybrid,transit=clients()
            assert hybrid.get_configuration(timeout=5)['hybrid_motion_step']==.22
            assert transit.get_configuration(timeout=5)['hybrid_transit_weight_viapoint']==23.
            assert rospy.get_param('/coverage_manager/operation_width_m')==1.05
            result={'checks':['hybrid update acknowledged','transit update acknowledged','coverage defaults committed','hybrid actual save failure rejected','hybrid survives process restart and launch defaults','transit survives process restart and launch defaults','coverage width survives process restart'], 'passed':7,'interfaces':['lo'],'robot_connected':False}
            (ROOT/'restart_test.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
            return
        frame_socket=os.environ.get('CONBINE_FRAME_SOCKET')
        vision_args=[]
        if frame_socket:
            gateway=spawn('frame_reader',['python3','-B',str(WS/'scripts/conbine_preview_relay.py'),
                'inside','--socket',frame_socket])
            def gateway_ready():
                try:
                    with socket.create_connection(('127.0.0.1',18782),timeout=.2):return True
                except OSError:return False
            wait(gateway_ready,'isolated frame reader',5)
            os.environ['SURROUND_PREVIEW_URL']='http://127.0.0.1:18782'
            vision_args=['_configured_vision_backend:=j6m_surround_bpu','_vision_backend_switch_script:=']
        gui=spawn('qt',[str(WS/'devel/lib/autolabor_operator_gui/autolabor_operator_gui_node'),
            '__name:=autolabor_operator_gui','_static_map_mode:=true','_initial_tuning_page:=true',
            '_static_map_set:='+str(ROOT/'map_set'),'_navigation_mode_label:=OFFLINE_TUNING',
            '_tuning_screenshot:='+str(ROOT/'qt_tuning.png'),
            '_coverage_region_root:='+str(ROOT/'map_set'),
            '_rviz_config:='+str(WS/'src/application/autolabor_operator_gui/config/operator_navigation.rviz')]+vision_args)
        (ROOT/'state.json').write_text(json.dumps({'mode':'real_surround_and_isolated_navigation' if frame_socket else 'offline_tuning_preview','interfaces':['lo'],'robot_navigation_connected':False,'camera_bridge_configured':bool(frame_socket),'qt_pid':gui.pid,'backend_pid':backend.pid,'persistent_directory':str(Path(os.environ['ROS_HOME'])/'autolabor_navigation'),'coverage_config':str(ROOT/'coverage.yaml'),'started_at':datetime.datetime.now().isoformat()},indent=2)+'\n')
        while not stop.wait(.25):
            if gui.poll() is not None:
                if gui.returncode:
                    raise RuntimeError('Qt exited with code %d; see qt.log' % gui.returncode)
                break
            if backend.poll() is not None:raise RuntimeError('Preview backend exited')
            if frame_socket and gateway.poll() is not None:raise RuntimeError('Isolated frame reader exited')
    finally:
        for child,stream in reversed(children):
            terminate(child);stream.close()
        if 'rospy' in locals():rospy.signal_shutdown('preview closed')

if __name__=='__main__':main()
