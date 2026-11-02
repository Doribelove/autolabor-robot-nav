#!/usr/bin/env python3
"""Place a requested rectangle using one measured vehicle pose; never start motion."""
import argparse
import json
import math
from pathlib import Path
import time


def rectangle_in_map(x, y, yaw, forward=8., left=3., length=10., width=3.):
    values=(x,y,yaw,forward,left,length,width)
    if not all(math.isfinite(v) for v in values) or length<=0 or width<=0:
        raise ValueError('Rectangle dimensions and pose must be finite, with positive dimensions')
    c,s=math.cos(yaw),math.sin(yaw)
    local=[(forward-length/2,left-width/2),(forward+length/2,left-width/2),
           (forward+length/2,left+width/2),(forward-length/2,left+width/2)]
    return [(x+c*a-s*b,y+s*a+c*b) for a,b in local]


def main():
    import rospy
    import tf2_ros
    from geometry_msgs.msg import PolygonStamped,Point32
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True)
    parser.add_argument('--confirmed-flat',action='store_true')
    for name,default in [('forward',8.),('left',3.),('length',10.),('width',3.)]:
        parser.add_argument('--'+name,type=float,default=default)
    args=parser.parse_args(rospy.myargv()[1:]);rospy.init_node('coverage_region_preset',anonymous=True)
    buffer=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buffer)
    publisher=rospy.Publisher('/coverage/preset_region',PolygonStamped,queue_size=1,latch=True)
    deadline=time.monotonic()+35
    while time.monotonic()<deadline and not rospy.is_shutdown():
        try:
            transform=buffer.lookup_transform('map','base_link',rospy.Time(0),rospy.Duration(.2))
            age=(rospy.Time.now()-transform.header.stamp).to_sec()
            if not 0<=age<=.75 or not publisher.get_num_connections():
                time.sleep(.1);continue
            break
        except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
            time.sleep(.1)
    else:raise RuntimeError('No fresh vehicle pose or Qt preset subscriber')
    p=transform.transform.translation;q=transform.transform.rotation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    points=rectangle_in_map(p.x,p.y,yaw,args.forward,args.left,args.length,args.width)
    request=PolygonStamped();request.header.frame_id='map';request.header.stamp=transform.header.stamp
    request.polygon.points=[Point32(x=x,y=y,z=0) for x,y in points]
    evidence=dict(frame_id='map',base_pose_xyyaw=[p.x,p.y,yaw],pose_stamp=transform.header.stamp.to_sec(),
        center_relative_base_xy=[args.forward,args.left],length_along_heading_m=args.length,
        width_lateral_m=args.width,requested_area_m2=args.length*args.width,vertices_map=points,
        starts_motion=False,execution='operator clicks Qt Start')
    evidence['user_confirmed_flat_ground']=args.confirmed_flat
    Path(args.output).write_text(json.dumps(evidence,ensure_ascii=False,indent=2)+'\n')
    if args.confirmed_flat:
        flat_pub=rospy.Publisher('/online_mapping/confirmed_flat_region',PolygonStamped,queue_size=1,latch=True)
        deadline=time.monotonic()+5
        while not flat_pub.get_num_connections() and time.monotonic()<deadline:time.sleep(.1)
        if not flat_pub.get_num_connections():raise RuntimeError('Online map prior subscriber unavailable')
        flat_pub.publish(request);time.sleep(2)
    publisher.publish(request);time.sleep(1)
    print(json.dumps(evidence,ensure_ascii=False))


if __name__=='__main__':main()
