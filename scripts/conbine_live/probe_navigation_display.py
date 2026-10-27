#!/usr/bin/env python3
"""Verify actual costmap/footprint data and TF without publishing a goal."""
import collections
import json
import os
from pathlib import Path
import threading
import time
import rosgraph
import rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PolygonStamped,Twist
from nav_msgs.msg import OccupancyGrid

rospy.init_node('conbine_costmap_readonly_probe',anonymous=True)
counts=collections.Counter();latest={};lock=threading.RLock();peak=[0.]
def callback(topic,msg):
    with lock:
        counts[topic]+=1
        if isinstance(msg,OccupancyGrid):
            latest[topic]={'frame':msg.header.frame_id,'width':msg.info.width,'height':msg.info.height,
                          'resolution':msg.info.resolution,'cells':len(msg.data)}
        elif isinstance(msg,PolygonStamped):
            latest[topic]={'frame':msg.header.frame_id,'points':[[p.x,p.y,p.z] for p in msg.polygon.points]}
        elif isinstance(msg,Twist):
            peak[0]=max(peak[0],*(abs(x) for x in (msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z)))
        else:latest[topic]=[s.status for s in msg.status_list]
for topic,cls in [('/move_base/global_costmap/costmap',OccupancyGrid),
                  ('/move_base/local_costmap/costmap',OccupancyGrid),
                  ('/move_base/local_costmap/footprint',PolygonStamped),
                  ('/move_base/status',GoalStatusArray),('/cmd_vel',Twist)]:
    rospy.Subscriber(topic,cls,lambda msg,t=topic:callback(t,msg),queue_size=5,tcp_nodelay=True)
buffer=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buffer)
time.sleep(8.)
transform=buffer.lookup_transform('map','base_link',rospy.Time(0),rospy.Duration(2.))
master=rosgraph.Master(rospy.get_name())
with lock:
    result={'counts':dict(counts),'latest':latest,'map_to_base_link':{
        'xyz':[transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z],
        'quaternion':[transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w]},
        'hybrid_protocol':rospy.get_param('/move_base/CoverageGlobalPlanner/tuning_persistence_protocol',0),
        'transit_protocol':rospy.get_param('/coverage_manager/tuning_persistence_protocol',0),
        'teb_service':master.lookupService('/move_base/TebLocalPlannerROS/set_parameters'),
        'final_cmd_peak':peak[0],'motion_enabled':rospy.get_param('/nvidia_cmd_vel_watchdog/motion_enabled')}
Path(os.environ['CONBINE_PROBE_OUTPUT']).write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
for topic in ('/move_base/global_costmap/costmap','/move_base/local_costmap/costmap'):
    assert counts[topic]>0 and latest[topic]['cells']==latest[topic]['width']*latest[topic]['height']>0
assert len(latest['/move_base/local_costmap/footprint']['points'])>=3
assert counts['/move_base/status']>0 and result['hybrid_protocol']==result['transit_protocol']==1
assert result['motion_enabled'] is True
print(json.dumps(result,ensure_ascii=False,indent=2))
rospy.signal_shutdown('Read-only display verification complete')
