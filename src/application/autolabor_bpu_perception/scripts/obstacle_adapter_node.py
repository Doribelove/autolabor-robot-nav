#!/usr/bin/env python3
"""Transform fresh supported CenterPoint boxes into bounded TEB candidates.

Starts in shadow mode. Existing geometric /scan remains authoritative and is
never cleared or replaced by learned detections. No identity TF fallback.
"""
import math
import copy
import threading
import time
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Point32
from tf.transformations import quaternion_matrix
from costmap_converter.msg import ObstacleArrayMsg,ObstacleMsg
from autolabor_bpu_perception.msg import DetectedObjects
from autolabor_bpu_perception.core import fresh,obstacle_polygon


class ObstacleAdapter:
    def __init__(self):
        self.enabled=bool(rospy.get_param('~navigation_enabled',False))
        if self.enabled:
            raise ValueError('Navigation influence is unavailable until MID360 accuracy and the TEB custom-obstacle lease are validated')
        self.target=str(rospy.get_param('~target_frame','camera_init'))
        self.age=float(rospy.get_param('~maximum_age_sec',.35))
        self.confidence=float(rospy.get_param('~minimum_confidence',.60))
        if self.target not in ('map','camera_init') or not .1<=self.age<=.5 or not .4<=self.confidence<=.95:
            raise ValueError('Invalid obstacle adapter configuration')
        self.latest=None;self.receipt=0.;self.lock=threading.Lock()
        self.previous=[];self.next_id=1;self.previous_session='';self.previous_stamp=0.
        self.tf_buffer=tf2_ros.Buffer(cache_time=rospy.Duration(3))
        self.tf_listener=tf2_ros.TransformListener(self.tf_buffer)
        self.candidate=rospy.Publisher('/perception/centerpoint/obstacles_candidate',ObstacleArrayMsg,queue_size=1)
        self.navigation=rospy.Publisher('/move_base/TebLocalPlannerROS/obstacles',ObstacleArrayMsg,queue_size=1) if self.enabled else None
        self.sub=rospy.Subscriber('/perception/centerpoint/objects',DetectedObjects,self.callback,queue_size=1)
        self.timer=rospy.Timer(rospy.Duration(.1),self.tick)

    def callback(self,message):
        caller=(getattr(message,'_connection_header',None) or {}).get('callerid')
        if caller!='/centerpoint_mid360':
            rospy.logwarn_throttle(2,'Reject unexpected CenterPoint publisher');return
        with self.lock:
            if (self.latest is not None and message.session_id == self.latest.session_id and
                    message.header.stamp < self.latest.header.stamp):
                return
            self.latest=message;self.receipt=time.monotonic()

    def tick(self,_):
        with self.lock:message=self.latest;receipt=self.receipt
        result=ObstacleArrayMsg();result.header.frame_id=self.target
        result.header.stamp=rospy.Time.now()
        valid=(message is not None and message.valid and message.calibrated and
            message.model_sha256=='7a12188054b4f5a05c112dd6d8611436d84cda3e060e11f72d1f1e739040593c' and
            message.header.frame_id=='base_link' and len(message.objects)<=100 and
            fresh(message.header.stamp.to_sec(),rospy.Time.now().to_sec(),self.age) and
            time.monotonic()-receipt<=self.age)
        if valid:
            try:
                transform=self.tf_buffer.lookup_transform(self.target,message.header.frame_id,message.header.stamp,
                    rospy.Duration(.01)).transform
                q=transform.rotation;norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
                if not math.isfinite(norm) or abs(norm-1)>1e-3:raise ValueError('Invalid quaternion')
                rotation=quaternion_matrix([q.x,q.y,q.z,q.w])[:3,:3]
                offset=np.array([transform.translation.x,transform.translation.y,transform.translation.z])
                if not np.isfinite(offset).all():raise ValueError('Invalid translation')
                # A planar footprint requires a nearly upright robot.
                if rotation[2,2]<math.cos(.20):raise ValueError('Robot tilt exceeds planar adapter range')
                yaw_offset=math.atan2(rotation[1,0],rotation[0,0])
                stamp=message.header.stamp.to_sec()
                if message.session_id!=self.previous_session or stamp<self.previous_stamp:
                    self.previous=[];self.previous_stamp=0.;self.previous_session=message.session_id
                fresh_observation=stamp>self.previous_stamp
                current=[];matched=set()
                for item in message.objects:
                    if not item.position_valid or not item.dimensions_valid or not math.isfinite(item.confidence) or item.confidence<self.confidence:continue
                    point=rotation@np.array([item.position.x,item.position.y,item.position.z])+offset
                    dimensions=[item.dimensions.x,item.dimensions.y,item.dimensions.z]
                    polygon=obstacle_polygon(point,dimensions,item.yaw+yaw_offset)
                    candidates=[(np.linalg.norm(point[:2]-t['point'][:2]),i,t) for i,t in enumerate(self.previous)
                        if i not in matched and t['class_id']==item.class_id and 0<=stamp-t['stamp']<=self.age]
                    selected=min(candidates,key=lambda x:x[0]) if candidates else None
                    if selected and selected[0]<.65:
                        _,i,old=selected;matched.add(i)
                        track=dict(old,point=point,stamp=stamp,hits=old['hits']+int(fresh_observation))
                    else:
                        track=dict(id=self.next_id,point=point,stamp=stamp,hits=1,class_id=item.class_id);self.next_id+=1
                    current.append(track)
                    if track['hits']<2:continue
                    obstacle=ObstacleMsg();obstacle.id=track['id'];obstacle.header=copy.deepcopy(message.header)
                    obstacle.header.frame_id=self.target;obstacle.orientation.w=1.
                    obstacle.polygon.points=[Point32(x=float(x),y=float(y),z=0.) for x,y in polygon]
                    result.obstacles.append(obstacle)
                if fresh_observation:self.previous=current;self.previous_stamp=stamp
                result.header.stamp=message.header.stamp
                # Do not refresh the lease while repeatedly publishing one observation.
                if not fresh(stamp,rospy.Time.now().to_sec(),self.age):result.obstacles=[]
            except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException,ValueError) as error:
                self.previous=[];result.obstacles=[]
                rospy.logwarn_throttle(2,'CenterPoint obstacle adapter: %s',error)
        else:self.previous=[]
        self.candidate.publish(result)
        if self.navigation:self.navigation.publish(result)


if __name__=='__main__':
    rospy.init_node('centerpoint_obstacle_adapter')
    adapter=ObstacleAdapter();rospy.spin()
