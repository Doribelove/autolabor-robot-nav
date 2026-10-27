#!/usr/bin/env python3
"""Finite read-only observation of the real graph and final command/feedback."""
import collections
import json
import os
from pathlib import Path
import threading
import time

import rosgraph
import rospy
from autolabor_canbus_driver.msg import ChassisMonitorInfo, ChassisStatusInfo
from autolabor_coverage.msg import CoverageStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Float64, String

rospy.init_node('conbine_live_readonly_probe', anonymous=True)
counts=collections.Counter(); arrivals=collections.defaultdict(list)
peak=collections.defaultdict(float); latest={}; lock=threading.RLock()
def callback(topic,msg):
    with lock:
        counts[topic]+=1;arrivals[topic].append(time.monotonic())
        if isinstance(msg,Twist):
            peak[topic]=max(peak[topic],*(abs(x) for x in (msg.linear.x,msg.linear.y,msg.linear.z,
                msg.angular.x,msg.angular.y,msg.angular.z)))
        elif isinstance(msg,Float64):peak[topic]=max(peak[topic],abs(msg.data))
        elif isinstance(msg,Bool):latest[topic]=msg.data
        elif isinstance(msg,String):latest[topic]=msg.data
        elif isinstance(msg,CoverageStatus):
            latest[topic]={k:getattr(msg,k) for k in ('state','map_ready','localized','active',
                'watchdog_motion_enabled','chassis_ready','chassis_detail','avoidance_ready','avoidance_detail')}
        elif isinstance(msg,(ChassisMonitorInfo,ChassisStatusInfo)):
            latest[topic]={k:getattr(msg,k) for k in msg.__slots__ if k!='header'}

for topic,cls in [('/Odometry',Odometry),('/odom',Odometry),('/livox/imu',Imu),('/scan',LaserScan),
    ('/cmd_vel',Twist),('/cmd_vel_safe',Twist),('/m2_driver/left_wheel_vel',Float64),
    ('/m2_driver/right_wheel_vel',Float64),('/m2_driver/chassis_info',ChassisStatusInfo),
    ('/m2_driver/chassis_monitor',ChassisMonitorInfo),('/fast_lio/localization_status',String),
    ('/coverage/status',CoverageStatus),('/autolabor_operator_gui/map_display_status',String),
    ('/nvidia_cmd_vel_watchdog/status',String)]:
    rospy.Subscriber(topic,cls,lambda msg,t=topic:callback(t,msg),queue_size=100,tcp_nodelay=True)
for topic,cls in [('/avoidance/mid360_active',Bool),('/avoidance/source_mode',String)]:
    rospy.Subscriber(topic,cls,lambda msg,t=topic:callback(t,msg),queue_size=10,tcp_nodelay=True)
time.sleep(15)
master=rosgraph.Master(rospy.get_name());graph=master.getSystemState()
with lock:
    result={'master':os.environ['ROS_MASTER_URI'],'counts':dict(counts),
        'rates_hz':{t:round((len(v)-1)/(v[-1]-v[0]),3) for t,v in arrivals.items() if len(v)>1},
        'peak_abs_command_or_wheel_speed':dict(peak),'latest':latest,
        'motion_enabled_param':rospy.get_param('/nvidia_cmd_vel_watchdog/motion_enabled'),
        'cmd_vel_publishers':[nodes for topic,nodes in graph[0] if topic=='/cmd_vel'],
        'cmd_vel_subscribers':[nodes for topic,nodes in graph[1] if topic=='/cmd_vel']}
    result['auxiliary_lidar_topics']=[topic for part in graph[:2] for topic,_ in part
                                    if topic.startswith('/dual_lidar/') or topic=='/avoidance/dual_lidar_active']
Path(os.environ['CONBINE_PROBE_OUTPUT']).write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(result,ensure_ascii=False,indent=2))
assert counts['/cmd_vel']>200 and result['motion_enabled_param'] is True
assert result['cmd_vel_publishers']==[['/nvidia_cmd_vel_watchdog']]
assert counts['/Odometry']>80 and counts['/scan']>80 and counts['/odom']>80
assert counts['/m2_driver/left_wheel_vel']>200 and counts['/m2_driver/right_wheel_vel']>200
assert latest.get('/avoidance/mid360_active') is True and latest.get('/avoidance/source_mode')=='mid360'
assert latest['/coverage/status']['avoidance_ready'],latest['/coverage/status']
assert not result['auxiliary_lidar_topics']
rospy.signal_shutdown('Read-only observation complete')
