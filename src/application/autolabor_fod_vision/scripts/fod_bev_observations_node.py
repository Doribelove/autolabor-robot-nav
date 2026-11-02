#!/usr/bin/env python3
"""Decode-result ROS bridge. Source-stamped TF, no vehicle or cleaning commands.
Input String JSON is the explicit adapter boundary for a validated HBM executor.
"""
import json,math
import numpy as np
import rospy,tf2_ros
from tf.transformations import quaternion_matrix
from std_msgs.msg import String
from autolabor_fod_msgs.msg import FodGroundObservation,FodGroundObservationArray
from visualization_msgs.msg import Marker,MarkerArray
from autolabor_fod_vision.tracking import GroundObservation,MultiTargetTracker

class Node:
 def __init__(self):
  self.buffer=tf2_ros.Buffer(rospy.Duration(60));self.listener=tf2_ros.TransformListener(self.buffer)
  self.base=rospy.get_param('~base_frame','base_link');self.map=rospy.get_param('~map_frame','map');self.last=0
  self.tracker=MultiTargetTracker(association_distance=.20,alpha=.45,min_hits=2,max_age=2,max_misses=5)
  self.base_pub=rospy.Publisher('/fod/bev/ground_observations',FodGroundObservationArray,queue_size=10)
  self.map_pub=rospy.Publisher('/fod/bev/map_observations',FodGroundObservationArray,queue_size=10)
  self.marker_pub=rospy.Publisher('/fod/bev/markers',MarkerArray,queue_size=10)
  self.metrics=rospy.Publisher('/fod/bev/latency',String,queue_size=10)
  rospy.Subscriber('/fod/bev/decoded',String,self.receive,queue_size=10)
 def matrix(self,target,source,stamp):
  if target==source:return np.eye(4)
  t=self.buffer.lookup_transform(target,source,stamp,rospy.Duration(.2)).transform
  m=quaternion_matrix([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]);m[:3,3]=[t.translation.x,t.translation.y,t.translation.z];return m
 def array(self,frame,source,stamp,centers,mat,cov):
  msg=FodGroundObservationArray();msg.header.stamp=stamp;msg.header.frame_id=frame;msg.source_frame=source;msg.status='OK'
  rot=mat[:2,:2];cov=rot@cov@rot.T
  for i,c in enumerate(centers):
   point=mat@np.array([c['x'],c['y'],0,1]);o=FodGroundObservation();o.source_detection_index=i;o.class_id=0;o.class_name='trash';o.confidence=c['confidence'];o.point.x=point[0];o.point.y=point[1];o.point.z=point[2];o.covariance_xy=cov.ravel().tolist();msg.observations.append(o)
  return msg
 def receive(self,message):
  try:
   data=json.loads(message.data);ns=data['stamp_ns'];stamps=data['source_stamps_ns']
   if type(ns) is not int or ns<=self.last or ns<=0:raise ValueError('duplicate/nonpositive/out-of-order capture stamp')
   if data.get('clock_domain')!='ros_epoch_ns' or not data.get('clock_verified') or set(stamps)!={'front','left','right','rear'}:raise ValueError('unverified four-camera timestamps')
   if any(type(t) is not int for t in stamps.values()):raise ValueError('capture timestamps must be integer nanoseconds')
   if min(stamps.values())<=0 or max(stamps.values())-min(stamps.values())>20000000 or ns!=max(stamps.values()):raise ValueError('camera skew exceeds 20ms')
   centers=data['centers'];source=data['frame_id']
   for c in centers:
    if not all(math.isfinite(c[k]) for k in ('x','y','confidence')) or not 0<=c['confidence']<=1:raise ValueError('invalid center')
   std=float(data['position_std_m'])
   if not math.isfinite(std) or std<=0:raise ValueError('invalid position uncertainty')
   stamp=rospy.Time(ns//1000000000,ns%1000000000);base_mat=self.matrix(self.base,source,stamp)
   base=self.array(self.base,source,stamp,centers,base_mat,np.eye(2)*std**2);self.base_pub.publish(base)
   map_mat=self.matrix(self.map,self.base,stamp)@base_mat
   mapped=self.array(self.map,source,stamp,centers,map_mat,np.eye(2)*std**2);self.map_pub.publish(mapped);self.last=ns
   self.tracker.update([GroundObservation(0,'trash',o.confidence,o.point.x,o.point.y,o.covariance_xy[0],o.covariance_xy[1],o.covariance_xy[3]) for o in mapped.observations],stamp.to_sec())
   markers=MarkerArray();clear=Marker();clear.action=Marker.DELETEALL;markers.markers.append(clear)
   for t in self.tracker.confirmed_tracks(stamp.to_sec()):
    m=Marker();m.header=mapped.header;m.ns='bev_trash';m.id=t.track_id;m.type=Marker.SPHERE;m.action=Marker.ADD;m.pose.position.x=t.x;m.pose.position.y=t.y;m.pose.orientation.w=1;m.scale.x=m.scale.y=m.scale.z=.10;m.color.r=1;m.color.g=.3;m.color.a=1;m.lifetime=rospy.Duration(2);markers.markers.append(m)
   self.marker_pub.publish(markers)
   elapsed=(rospy.Time.now()-stamp).to_sec()*1000
   self.metrics.publish(String(data=json.dumps(dict(capture_to_map_publish_ms=elapsed,model_bpu_ms=data.get('model_bpu_ms'),clock='ROS capture clock; replay uses simulated time'))))
  except (ValueError,KeyError,TypeError,tf2_ros.TransformException) as e:rospy.logwarn_throttle(2,'BEV observation rejected: %s',e)
if __name__=='__main__':
 rospy.init_node('fod_bev_observations');Node();rospy.spin()
