#!/usr/bin/env python3
"""FCOS on J6M + matching ZED depth on NVIDIA, independent of Qt visibility.

Produces reference-class metric observations, never collection candidates or
motion commands. World projection requires confirmed mount + source-time TF.
"""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
from collections import deque
import copy
import json
import math
from pathlib import Path
import threading
import time
import uuid

import cv2
import numpy as np
import rospy
import tf2_ros
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String
from tf.transformations import quaternion_matrix
from autolabor_bpu_perception.msg import DetectedObject,DetectedObjects
from autolabor_bpu_perception.core import fresh,ImageTracker,zed_mount
from autolabor_bpu_perception.protocol import Client
from autolabor_fod_vision.bpu_rgbd import Preprocessor,source_boxes,estimate_boxes,FCOS_SHA
from autolabor_fod_vision.depth_fusion import nearest_synchronized_message
from autolabor_fod_vision.two_stage import LatestFrameSlot


class FcosRgbd:
    def __init__(self):
        self.session=uuid.uuid4().hex
        self.maximum_age=float(rospy.get_param('~maximum_age_sec',.35))
        self.fps=float(rospy.get_param('~max_fps',15))
        if not .1<=self.maximum_age<=.5 or not .5<=self.fps<=30:raise ValueError('Invalid age/frequency limits')
        self.mount_file=Path(rospy.get_param('~mount_config'))
        self.mount=zed_mount(yaml.safe_load(self.mount_file.read_text()))
        self.classes=yaml.safe_load(Path(rospy.get_param('~classes_config')).read_text())['names']
        self.slot=LatestFrameSlot();self.result_slot=LatestFrameSlot();self.lock=threading.Lock()
        self.depth=deque(maxlen=20);self.info=deque(maxlen=20)
        self.bridge=CvBridge();self.preprocessor=Preprocessor();self.tracker=ImageTracker()
        self.client=None;self.sequence=0;self.last_stamp=0.;self.last_result=0.;self.last_receipt=0.
        self.processed=0;self.expired=0;self.depth_matches=0;self.depth_valid=0
        self.tf_buffer=tf2_ros.Buffer(cache_time=rospy.Duration(3))
        self.tf_listener=tf2_ros.TransformListener(self.tf_buffer)
        self.pub=rospy.Publisher('/perception/fcos/objects',DetectedObjects,queue_size=1)
        self.image_pub=rospy.Publisher('/fod/bpu_preview/image',Image,queue_size=1)
        self.status_pub=rospy.Publisher('/fod/bpu_preview/status',String,queue_size=1)
        self.diagnostic_pub=rospy.Publisher('/perception/fcos/status',String,queue_size=1)
        self.image_sub=rospy.Subscriber(rospy.get_param('~image_topic','/fod_camera/image_raw'),Image,
            self.image_callback,queue_size=1,buff_size=8*1024*1024)
        self.depth_sub=rospy.Subscriber(rospy.get_param('~depth_topic','/fod_camera/depth_registered'),Image,
            self.depth_callback,queue_size=2,buff_size=16*1024*1024)
        self.info_sub=rospy.Subscriber(rospy.get_param('~info_topic','/fod_camera/camera_info'),CameraInfo,
            self.info_callback,queue_size=2)
        self.worker=threading.Thread(target=self.loop,daemon=True)
        self.fusion_worker=threading.Thread(target=self.fusion_loop,daemon=True)
        self.worker.start();self.fusion_worker.start();self.timer=rospy.Timer(rospy.Duration(.1),self.heartbeat)
        rospy.on_shutdown(self.stop)

    def image_callback(self,message):
        if not message.header.frame_id.endswith('_left_camera_optical_frame'):
            rospy.logwarn_throttle(2,'FCOS RGB source must be the rectified left optical frame');return
        self.last_receipt=time.monotonic();self.slot.put(message)

    def depth_callback(self,message):
        with self.lock:self.depth.append(message)

    def info_callback(self,message):
        with self.lock:self.info.append(message)

    def heartbeat(self,_):
        if time.monotonic()-self.last_result>self.maximum_age:
            empty=DetectedObjects();empty.header.stamp=rospy.Time.from_sec(self.last_stamp)
            empty.header.frame_id='base_link';empty.session_id=self.session
            empty.model_name='fcos_efficientnetb3';empty.model_sha256=FCOS_SHA
            empty.valid=False;empty.calibrated=True;empty.motion_eligible=False
            empty.detail='No fresh RGB-D inference result';self.pub.publish(empty)

    def report(self,detail,**extra):
        status=dict(session_id=self.session,processed=self.processed,expired=self.expired,
                    dropped=self.slot.overwritten,fusion_dropped=self.result_slot.overwritten,depth_matched=self.depth_matches,
                    depth_valid_objects=self.depth_valid,motion_eligible=False,detail=detail,**extra)
        text=json.dumps(status,allow_nan=False)
        self.diagnostic_pub.publish(String(data=text))
        if 'source_age_ms' in extra:
            display='FCOS + ZED 深度 · 已处理 %d 帧 · 有效定位 %d 次 · 帧龄 %.0f ms' % (
                self.processed,self.depth_valid,extra['source_age_ms'])
        else:
            display='FCOS 深度链路等待恢复：'+str(detail)
        self.status_pub.publish(String(data=display))

    def loop(self):
        cv2.setNumThreads(1);next_frame=0.
        while not rospy.is_shutdown():
            delay=next_frame-time.monotonic()
            if delay>0:time.sleep(min(delay,.05));continue
            frame=self.slot.take(.1)
            if frame is None:continue
            stamp=frame.header.stamp.to_sec()
            if stamp<=self.last_stamp or not fresh(stamp,rospy.Time.now().to_sec(),self.maximum_age):
                self.expired+=1;continue
            self.last_stamp=stamp;next_frame=time.monotonic()+1/self.fps
            try:
                started=time.monotonic()
                input_age_ms=(rospy.Time.now().to_sec()-stamp)*1000
                image=self.bridge.imgmsg_to_cv2(frame,desired_encoding='bgr8')
                payload,rows,scales=self.preprocessor.prepare(image)
                prepared=time.monotonic()
                self.sequence+=1
                if self.client is None:self.client=Client(remote=True,timeout=2.)
                result=self.client.infer(dict(model='fcos',sequence=self.sequence,stamp_ns=frame.header.stamp.to_nsec(),
                    width=frame.width,height=frame.height,rows=rows),payload)
                inferred=time.monotonic()
                result['client_timing']=dict(input_age_ms=input_age_ms,preprocess_ms=(prepared-started)*1000,
                    transport_and_infer_ms=(inferred-prepared)*1000,service_ms=result['service_ms'])
                if not fresh(stamp,rospy.Time.now().to_sec(),self.maximum_age):self.expired+=1;continue
                boxes=source_boxes(result,frame.width,frame.height,scales,self.classes)
                self.result_slot.put((frame,image,boxes,result))
            except Exception as error:
                self.report(str(error))
                if self.client:self.client.close();self.client=None
                rospy.logwarn_throttle(2,'FCOS RGB-D: %s',error)

    def fusion_loop(self):
        while not rospy.is_shutdown():
            pending=self.result_slot.take(.1)
            if pending is None:continue
            frame,image,boxes,result=pending
            stamp=frame.header.stamp.to_sec()
            if not fresh(stamp,rospy.Time.now().to_sec(),self.maximum_age):
                self.expired+=1;continue
            try:
                boxes=self.tracker.update(boxes,stamp)
                self.publish(frame,image,boxes,result)
            except Exception as error:
                self.report(str(error));rospy.logwarn_throttle(2,'FCOS depth fusion: %s',error)

    def publish(self,frame,image,boxes,result):
        started=time.monotonic()
        stamp=frame.header.stamp.to_sec()
        with self.lock:
            depth,_=nearest_synchronized_message(self.depth,stamp,frame.header.frame_id,.02)
            info,_=nearest_synchronized_message(self.info,stamp,frame.header.frame_id,.02)
        estimates=None
        if depth is not None and info is not None and (info.width,info.height)==(frame.width,frame.height):
            array=self.bridge.imgmsg_to_cv2(depth,desired_encoding='passthrough')
            if depth.encoding=='32FC1':array=np.asarray(array,dtype=np.float32)
            elif depth.encoding=='16UC1':array=np.asarray(array,dtype=np.float32)*.001
            else:raise ValueError('Unsupported metric depth encoding')
            if array.shape!=image.shape[:2]:raise ValueError('RGB/depth dimensions do not match')
            estimates=estimate_boxes(boxes,array,info.K)
        message=DetectedObjects();message.header=copy.deepcopy(frame.header)
        message.header.frame_id='base_link';message.source_frame=frame.header.frame_id
        message.session_id=self.session;message.model_name='fcos_efficientnetb3';message.model_sha256=FCOS_SHA
        message.calibrated=False;message.motion_eligible=False;message.valid=True
        message.detail='Reference COCO classes; depth matched' if estimates is not None else 'No synchronized depth'
        try:
            measured=self.tf_buffer.lookup_transform('base_link',frame.header.frame_id,frame.header.stamp,rospy.Duration(0)).transform
            q=measured.rotation;norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
            if math.isfinite(norm) and abs(norm-1)<=1e-3:
                actual_rotation=quaternion_matrix([q.x,q.y,q.z,q.w])[:3,:3]
                actual_position=[measured.translation.x,measured.translation.y,measured.translation.z]
                message.calibrated=bool(np.allclose(actual_position,self.mount['left_position'],atol=.002,rtol=0) and
                    np.allclose(actual_rotation,self.mount['optical_to_base'],atol=.002,rtol=0))
        except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):pass
        if not message.calibrated:message.detail='Camera TF does not match confirmed left-lens calibration'
        rotation=None;translation=None
        try:
            transform=self.tf_buffer.lookup_transform('map','base_link',frame.header.stamp,rospy.Duration(0)).transform
            q=transform.rotation
            norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
            if not math.isfinite(norm) or abs(norm-1)>1e-3:raise ValueError('Invalid TF quaternion')
            rotation=quaternion_matrix([q.x,q.y,q.z,q.w])[:3,:3]
            translation=np.array([transform.translation.x,transform.translation.y,transform.translation.z])
            message.header.frame_id='map'
        except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):pass
        display=image.copy() if self.image_pub.get_num_connections() else None
        for index,d in enumerate(boxes):
            item=DetectedObject();item.object_id=d['object_id'];item.class_id=d['class_id']
            item.class_name=d['class_name'];item.confidence=d['confidence'];item.bbox=d['bbox']
            estimate=estimates[index] if estimates is not None else None
            label='%s #%d %.2f Z:N/A'%(item.class_name,item.object_id,item.confidence)
            if message.calibrated and estimate is not None and estimate.valid:
                point=self.mount['optical_to_base']@np.asarray(estimate.camera_point)+self.mount['left_position']
                if rotation is not None:point=rotation@point+translation
                if np.isfinite(point).all():
                    item.position_valid=True;item.position=Point(*map(float,point))
                    item.depth_m=estimate.depth_m;item.depth_mad_m=estimate.mad_m;item.depth_samples=estimate.sample_count
                    label='%s #%d Z:%.2fm'%(item.class_name,item.object_id,item.depth_m)
            message.objects.append(item)
            if display is not None:
                x1,y1,x2,y2=map(int,d['bbox'])
                color=(0,220,0) if item.position_valid else (0,180,255)
                cv2.rectangle(display,(x1,y1),(x2,y2),color,2)
                cv2.putText(display,label,(x1,max(18,y1-5)),cv2.FONT_HERSHEY_SIMPLEX,.45,color,1)
        # Depth clustering/TF also consume time: recheck at the publication boundary.
        age=rospy.Time.now().to_sec()-stamp
        if not fresh(stamp,rospy.Time.now().to_sec(),self.maximum_age):self.expired+=1;return
        message.latency_ms=age*1000;self.pub.publish(message)
        self.depth_matches+=int(estimates is not None)
        self.depth_valid+=sum(o.position_valid for o in message.objects)
        self.processed+=1;self.last_result=time.monotonic()
        if display is not None:
            preview=self.bridge.cv2_to_imgmsg(display,encoding='bgr8');preview.header=copy.deepcopy(frame.header)
            preview.header.frame_id='bpu_preview_demo_only';self.image_pub.publish(preview)
        self.report(message.detail,source_age_ms=age*1000,infer_ms=result['timing']['vendor_infer_ms'],
            fusion_ms=(time.monotonic()-started)*1000,**result['client_timing'])

    def stop(self):
        self.slot.stop();self.result_slot.stop();self.worker.join(timeout=3);self.fusion_worker.join(timeout=3)
        if self.client and not self.worker.is_alive():self.client.close()


if __name__=='__main__':
    rospy.init_node('fcos_rgbd')
    node=FcosRgbd();rospy.spin()
