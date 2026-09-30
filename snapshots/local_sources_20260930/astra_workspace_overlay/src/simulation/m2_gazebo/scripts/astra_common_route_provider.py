#!/usr/bin/env python3
"""Simulation-only publisher of one immutable map and global route bundle.

No laser, actor, model-state, contact, or policy-result subscription. This node
does not plan online. Every GetPlan response has the same ordered XY coordinates.
"""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import sys
from urllib.parse import urlparse


def request_error(data,route,start_xy,start_frame,goal_xyyaw,goal_frame,max_start_distance):
    if start_frame!=data['frame_id'] or goal_frame!=data['frame_id']:return 'request_frame_mismatch'
    values=tuple(start_xy)+tuple(goal_xyyaw)
    if len(start_xy)!=2 or len(goal_xyyaw)!=3 or not all(math.isfinite(v) for v in values):return 'nonfinite_or_bad_request'
    if route.project(start_xy)['distance_m']>max_start_distance:return 'start_too_far_from_fixed_route'
    expected=data['goal_xy_yaw']
    if math.dist(goal_xyyaw[:2],expected[:2])>1e-6 or abs(math.atan2(math.sin(goal_xyyaw[2]-expected[2]),math.cos(goal_xyyaw[2]-expected[2])))>1e-6:
        return 'goal_changed_no_implicit_replan'
    return None


def main():
    import rospy
    from geometry_msgs.msg import PoseStamped,TransformStamped
    from nav_msgs.msg import OccupancyGrid,Path as RosPath
    from nav_msgs.srv import GetMap,GetMapResponse,GetPlan,GetPlanResponse
    from std_msgs.msg import String
    import tf2_ros
    rospy.init_node('astra_common_route_provider')
    uri=urlparse(os.environ.get('ROS_MASTER_URI',''))
    if uri.scheme!='http' or uri.hostname not in ('127.0.0.1','localhost','::1') or uri.username or uri.password:
        raise RuntimeError('loopback simulation master required')
    if rospy.get_param('/use_sim_time',False)is not True or rospy.get_param('/m2_gazebo/simulation_only',False)is not True:
        raise RuntimeError('simulation markers required')
    workspace=Path(os.environ['THESIS_WS']).resolve()
    sys.path.insert(0,str(workspace/'scripts/astra_gazebo'))
    from common_route import load_bundle
    bundle=Path(rospy.get_param('~bundle')).resolve();expected=rospy.get_param('~bundle_sha256')
    if hashlib.sha256((bundle/'common_route.json').read_bytes()).hexdigest()!=expected:raise RuntimeError('route bundle SHA mismatch')
    grid,route,data=load_bundle(bundle)
    max_distance=float(rospy.get_param('~maximum_start_distance_m',2.0))
    if not math.isfinite(max_distance) or max_distance<=0:raise ValueError('invalid start distance guard')
    map_pub=rospy.Publisher('/map',OccupancyGrid,queue_size=1,latch=True)
    metadata_pub=rospy.Publisher('/map_metadata',type(OccupancyGrid().info),queue_size=1,latch=True)
    path_pub=rospy.Publisher('/astra/common_route/path',RosPath,queue_size=1,latch=True)
    identity_pub=rospy.Publisher('/astra/common_route/identity',String,queue_size=1,latch=True)
    response_pub=rospy.Publisher('/astra/common_route/served_path',RosPath,queue_size=100)
    stamp=rospy.Time.now()
    occupancy=OccupancyGrid();occupancy.header.frame_id='map';occupancy.header.stamp=stamp
    occupancy.info.map_load_time=stamp;occupancy.info.resolution=grid.resolution
    occupancy.info.width=grid.width;occupancy.info.height=grid.height
    occupancy.info.origin.position.x=grid.origin[0];occupancy.info.origin.position.y=grid.origin[1];occupancy.info.origin.orientation.w=1.
    occupancy.data=grid.blocking_occupancy.ravel().tolist()
    path=RosPath();path.header.frame_id='map';path.header.stamp=stamp
    for index,point in enumerate(route.points):
        pose=PoseStamped();pose.header=copy.deepcopy(path.header)
        pose.pose.position.x,pose.pose.position.y=point
        yaw=route.goal_yaw if index==len(route.points)-1 else route.sample(route.arcs[index])[2]
        pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2);path.poses.append(pose)
    def get_plan(request):
        start=request.start;goal=request.goal;q=goal.pose.orientation
        if abs(q.x)+abs(q.y)>1e-9 or abs(q.z*q.z+q.w*q.w-1.)>1e-6:
            raise rospy.ServiceException('planar normalized goal quaternion required')
        error=request_error(data,route,(start.pose.position.x,start.pose.position.y),start.header.frame_id,
            (goal.pose.position.x,goal.pose.position.y,2*math.atan2(q.z,q.w)),goal.header.frame_id,max_distance)
        if error:raise rospy.ServiceException(error)
        result=copy.deepcopy(path);result.header.stamp=rospy.Time.now()
        for pose in result.poses:pose.header.stamp=result.header.stamp
        response_pub.publish(result)
        return GetPlanResponse(plan=result)
    rospy.Service('/static_map',GetMap,lambda request:GetMapResponse(map=occupancy))
    rospy.Service('/astra/common_route/get_plan',GetPlan,get_plan)
    # Exactly one publisher of map->odom must be selected by the future launch.
    broadcaster=None
    if rospy.get_param('~publish_map_to_odom',False):
        broadcaster=tf2_ros.StaticTransformBroadcaster();transform=TransformStamped()
        transform.header.stamp=stamp;transform.header.frame_id='map';transform.child_frame_id='odom'
        x,y,yaw=data['map_from_odom_xy_yaw'];transform.transform.translation.x=x;transform.transform.translation.y=y
        transform.transform.rotation.z=math.sin(yaw/2);transform.transform.rotation.w=math.cos(yaw/2)
        broadcaster.sendTransform(transform)
    map_pub.publish(occupancy);metadata_pub.publish(occupancy.info);path_pub.publish(path)
    identity_pub.publish(String(data=json.dumps({'bundle_sha256':expected,'path_points_sha256':data['path_points_sha256'],
        'map_image_sha256':data['map_image_sha256'],'map_yaml_sha256':data['map_yaml_sha256'],
        'occupancy_policy':'source free=0; occupied=100; unknown=100; no cell carving',
        'normalized_occupancy_sha256':hashlib.sha256(grid.blocking_occupancy.tobytes()).hexdigest(),
        'frame_id':'map','reference_point':'base_link route guide; rear conversion must be explicit',
        'online_replanning':False,'actor_oracle_input':False,'live_consumption_qualified':False})))
    rospy.spin()


if __name__=='__main__':main()
