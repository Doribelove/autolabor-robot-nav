#!/usr/bin/env python3
"""J6M ROS client consumes local, motion-compensated MID360 body cloud."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
import copy
import json
import math
import threading
import time
import uuid
import numpy as np
import rospy
from geometry_msgs.msg import Point,Vector3
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker,MarkerArray
from autolabor_bpu_perception.msg import DetectedObject,DetectedObjects
from autolabor_bpu_perception.core import fresh
from autolabor_bpu_perception.cloud import pointcloud_xyzi,model_points,supported_box
from autolabor_bpu_perception.protocol import Client
from autolabor_bpu_perception.centerpoint import CLASSES

MODEL_SHA='7a12188054b4f5a05c112dd6d8611436d84cda3e060e11f72d1f1e739040593c'


class CenterPointNode:
    def __init__(self):
        self.session=uuid.uuid4().hex
        self.maximum_age=float(rospy.get_param('~maximum_age_sec',.35))
        self.fps=float(rospy.get_param('~max_fps',5))
        if not .1<=self.maximum_age<=.5 or not .5<=self.fps<=5:raise ValueError('Invalid CenterPoint limits')
        self.sensor=np.array(rospy.get_param('~sensor_in_base_m',[.2,0,1]),float)
        self.lidar_in_body=rospy.get_param('~lidar_in_body_m',[-.011,-.02329,.04412])
        self.latest=None;self.condition=threading.Condition();self.stopping=False
        self.last_stamp=0.;self.last_result=0.;self.sequence=0;self.client=None
        self.received=0;self.dropped=0;self.processed=0;self.expired=0
        self.pub=rospy.Publisher('/perception/centerpoint/objects',DetectedObjects,queue_size=1)
        self.marker_pub=rospy.Publisher('/perception/centerpoint/markers',MarkerArray,queue_size=1)
        self.status_pub=rospy.Publisher('/perception/centerpoint/status',String,queue_size=1)
        self.sub=rospy.Subscriber(rospy.get_param('~cloud_topic','/cloud_registered_body'),PointCloud2,
            self.callback,queue_size=1,buff_size=16*1024*1024)
        self.thread=threading.Thread(target=self.loop,daemon=True);self.thread.start()
        self.timer=rospy.Timer(rospy.Duration(.1),self.heartbeat);rospy.on_shutdown(self.stop)

    def callback(self,message):
        caller=(getattr(message,'_connection_header',None) or {}).get('callerid')
        if message.header.frame_id!='body' or caller!='/laserMapping':
            rospy.logwarn_throttle(2,'CenterPoint requires FAST-LIO deskewed body cloud');return
        with self.condition:
            self.received+=1
            if self.latest is not None:self.dropped+=1
            self.latest=message;self.condition.notify()

    def empty(self,detail):
        message=DetectedObjects();message.header.frame_id='base_link'
        message.header.stamp=rospy.Time.from_sec(self.last_stamp);message.source_frame='body'
        message.session_id=self.session;message.model_name='centerpoint_pointpillar_nuscenes'
        message.model_sha256=MODEL_SHA;message.calibrated=True;message.motion_eligible=False;message.detail=detail
        return message

    def heartbeat(self,_):
        if time.monotonic()-self.last_result>self.maximum_age:
            self.pub.publish(self.empty('No fresh CenterPoint result'))

    def loop(self):
        next_frame=0.
        while not rospy.is_shutdown() and not self.stopping:
            delay=next_frame-time.monotonic()
            if delay>0:time.sleep(min(delay,.05));continue
            with self.condition:
                if self.latest is None:self.condition.wait(.1)
                cloud=self.latest;self.latest=None
            if cloud is None:continue
            stamp=cloud.header.stamp.to_sec()
            if stamp<=self.last_stamp or not fresh(stamp,rospy.Time.now().to_sec(),self.maximum_age):
                self.expired+=1;continue
            self.last_stamp=stamp;next_frame=time.monotonic()+1/self.fps
            try:
                points=model_points(pointcloud_xyzi(cloud),self.lidar_in_body,self.sensor)
                if len(points)<24:
                    self.pub.publish(self.empty('Insufficient valid deskewed points'));continue
                self.sequence+=1
                if self.client is None:self.client=Client(remote=False,timeout=2.)
                result=self.client.infer(dict(model='centerpoint',sequence=self.sequence,
                    stamp_ns=cloud.header.stamp.to_nsec(),points=len(points)),points.astype('<f4',copy=False).tobytes())
                if result.get('model_sha256')!=MODEL_SHA:raise ValueError('CenterPoint model SHA mismatch')
                self.publish(cloud,points,result)
            except Exception as error:
                if self.client:self.client.close();self.client=None
                self.pub.publish(self.empty(str(error)))
                self.status_pub.publish(String(data=json.dumps(dict(error=str(error),session_id=self.session))))
                rospy.logwarn_throttle(2,'CenterPoint: %s',error)

    def publish(self,cloud,points,result):
        detections=result.get('detections')
        if not isinstance(detections,list) or len(detections)>100:raise ValueError('Invalid detection count')
        message=self.empty('Reference single-sweep MID360 trial; velocity and collection disabled')
        message.header.stamp=cloud.header.stamp;message.valid=True
        for d in detections:
            label=d.get('class_id');score=d.get('confidence')
            if (type(label) is not int or not 0<=label<len(CLASSES) or d.get('class_name')!=CLASSES[label] or
                    not isinstance(score,(int,float)) or not math.isfinite(score) or not 0<=score<=1):
                raise ValueError('Malformed CenterPoint class/confidence')
            if not supported_box(d,points):continue
            p=np.asarray(d['position'])+self.sensor
            if p[2]+d['dimensions'][2]/2<.10 or p[2]-d['dimensions'][2]/2>2.0:continue
            item=DetectedObject();item.class_id=d['class_id'];item.class_name=d['class_name'];item.confidence=d['confidence']
            item.position_valid=True;item.position=Point(*map(float,p))
            item.dimensions_valid=True;item.dimensions=Vector3(*map(float,d['dimensions']));item.yaw=d['yaw']
            item.velocity_valid=False;message.objects.append(item)
        now=rospy.Time.now().to_sec()
        if not fresh(cloud.header.stamp.to_sec(),now,self.maximum_age):self.expired+=1;return
        message.latency_ms=(now-cloud.header.stamp.to_sec())*1000
        self.pub.publish(message);self.processed+=1;self.last_result=time.monotonic()
        markers=MarkerArray()
        for index,item in enumerate(message.objects):
            marker=Marker();marker.header=copy.deepcopy(message.header);marker.ns='centerpoint';marker.id=index
            marker.type=Marker.CUBE;marker.action=Marker.ADD;marker.pose.position=item.position
            marker.pose.orientation.z=np.sin(item.yaw/2);marker.pose.orientation.w=np.cos(item.yaw/2)
            marker.scale=item.dimensions;marker.color.r=1.;marker.color.g=.55;marker.color.a=.35
            marker.lifetime=rospy.Duration(max(.01,self.maximum_age-message.latency_ms/1000));markers.markers.append(marker)
        self.marker_pub.publish(markers)
        self.status_pub.publish(String(data=json.dumps(dict(session_id=self.session,received=self.received,
            processed=self.processed,dropped=self.dropped,expired=self.expired,objects=len(message.objects),
            source_age_ms=message.latency_ms,timing=result['timing'],motion_eligible=False))))

    def stop(self):
        with self.condition:self.stopping=True;self.condition.notify_all()
        self.thread.join(timeout=3)
        if self.client and not self.thread.is_alive():self.client.close()


if __name__=='__main__':
    rospy.init_node('centerpoint_mid360')
    node=CenterPointNode();rospy.spin()
