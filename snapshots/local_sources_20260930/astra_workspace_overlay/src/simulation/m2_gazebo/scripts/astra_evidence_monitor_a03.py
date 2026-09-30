#!/usr/bin/env python3
"""Passive, simulator-only evidence capture; never publishes vehicle commands.

Images are native Gazebo sensor frames. Model/link source timestamps come from
the post-physics WorldState plugin. Camera association is latest-at-or-before,
not a claim of zero-latency simultaneous camera rendering.
Machine QC does not imply visual QC or formal safety.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import threading
from collections import deque

import numpy as np
from PIL import Image
import rospy
import tf2_ros
import yaml
from gazebo_msgs.msg import WorldState, ContactsState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image as RosImage, LaserScan, JointState


def pose_dict(p):
    return dict(x=p.position.x, y=p.position.y, z=p.position.z,
                qx=p.orientation.x, qy=p.orientation.y,
                qz=p.orientation.z, qw=p.orientation.w)


class Monitor:
    def __init__(self):
        if not rospy.get_param('/use_sim_time', False):
            raise RuntimeError('simulation time required')
        self.output = Path(rospy.get_param('~output'))
        self.output.mkdir(parents=True, exist_ok=False)
        (self.output/'raw').mkdir()
        (self.output/'robot_description.xml').write_text(rospy.get_param('/robot_description'))
        parameters = {key:rospy.get_param(key,None) for key in ('/use_sim_time','/m2_gazebo','/move_base')}
        (self.output/'effective_parameters.yaml').write_text(yaml.safe_dump(parameters,sort_keys=True))
        self.run_id = rospy.get_param('~run_id')
        self.scene = yaml.safe_load(Path(rospy.get_param('~scene_manifest')).read_text())
        self.lock = threading.RLock()
        self.models, self.links, self.joints = {}, {}, {}
        self.model_stamp = self.link_stamp = self.scan_stamp = self.odom_stamp = 0.
        self.odom_pose = None
        self.last_frame = self.last_state = -1.
        self.frames = self.states = self.contact_messages = 0
        self.machine_pass_frames = 0
        self.clock_regressions = 0
        self.last_clock = 0.
        self.buffers={key:deque(maxlen=1000) for key in ('models','links','odom','scan','joints')}
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.state_file = (self.output/'states.jsonl').open('w', buffering=1)
        self.frame_file = (self.output/'frames.jsonl').open('w', buffering=1)
        self.contact_file = (self.output/'contacts.jsonl').open('w', buffering=1)
        rospy.Subscriber('/astra/world_models', WorldState, self.on_models, queue_size=5)
        rospy.Subscriber('/astra/world_links', WorldState, self.on_links, queue_size=5)
        rospy.Subscriber('/odom', Odometry, self.on_odom, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.on_scan, queue_size=10)
        rospy.Subscriber('/joint_states', JointState, self.on_joints, queue_size=10)
        rospy.Subscriber('/astra/physics_contacts', ContactsState, self.on_contacts, queue_size=1000)
        rospy.Subscriber('/astra/camera/image_raw', RosImage, self.on_image, queue_size=1,
                         buff_size=2560*1440*3*2)
        rospy.on_shutdown(self.close)

    def on_models(self, msg):
        with self.lock:
            now = msg.header.stamp.to_sec()
            if now < self.last_clock: self.clock_regressions += 1
            self.last_clock = now
            self.models = dict(zip(msg.name, map(pose_dict, msg.pose)))
            self.model_stamp = now
            self.buffers['models'].append((now,self.models))
            if now-self.last_state >= .05:
                self.last_state = now
                self.state_file.write(json.dumps(self.context(now), allow_nan=False)+'\n')
                self.states += 1

    def on_links(self, msg):
        with self.lock:
            self.links = {n:pose_dict(p) for n,p in zip(msg.name,msg.pose) if n.startswith('autolabor_m2::')}
            self.link_stamp = msg.header.stamp.to_sec()
            self.buffers['links'].append((self.link_stamp,self.links))

    def on_odom(self, msg):
        with self.lock:
            self.odom_stamp = msg.header.stamp.to_sec()
            self.odom_pose = pose_dict(msg.pose.pose)
            self.buffers['odom'].append((self.odom_stamp,self.odom_pose))

    def on_scan(self, msg):
        with self.lock:
            self.scan_stamp = msg.header.stamp.to_sec()
            self.buffers['scan'].append((self.scan_stamp,True))

    def on_joints(self, msg):
        with self.lock:
            self.joints = dict(zip(msg.name, msg.position))
            self.buffers['joints'].append((msg.header.stamp.to_sec(),self.joints))

    def on_contacts(self, msg):
        if not msg.states: return
        with self.lock:
            self.contact_messages += 1
            self.contact_file.write(json.dumps({'stamp':msg.header.stamp.to_sec(),
                'pairs':[{'a':s.collision1_name,'b':s.collision2_name,'depths':list(s.depths),'source_time_info':s.info} for s in msg.states]})+'\n')

    def context(self, now):
        return {'run_id':self.run_id,'receipt_sim_time_s':now,
                'model_source_stamp_s':self.model_stamp,'models':self.models,
                'link_source_stamp_s':self.link_stamp,'links':self.links,
                'scan_stamp_s':self.scan_stamp,'odom_stamp_s':self.odom_stamp,
                'odom_pose':self.odom_pose,'joint_positions_rad':self.joints}

    def frame_context(self, stamp):
        def before(key,default):
            return next(((t,v) for t,v in reversed(self.buffers[key]) if t<=stamp), (0.,default))
        mt,models=before('models',{});lt,links=before('links',{})
        ot,odom=before('odom',None);st,_=before('scan',None);jt,joints=before('joints',{})
        return {'run_id':self.run_id,'receipt_sim_time_s':rospy.Time.now().to_sec(),
                'model_source_stamp_s':mt,'models':models,
                'link_source_stamp_s':lt,'links':links,
                'scan_stamp_s':st,'odom_stamp_s':ot,'odom_pose':odom,
                'joint_stamp_s':jt,'joint_positions_rad':joints,
                'association':'latest post-physics source at or before camera stamp; models and links independently chosen with reported source age'}

    def on_image(self, msg):
        try:
            stamp = msg.header.stamp.to_sec()
            if stamp-self.last_frame < .95: return
            self.last_frame = stamp
            with self.lock: row = self.frame_context(stamp)
            if msg.encoding not in ('rgb8','bgr8'): raise RuntimeError('unsupported image encoding '+msg.encoding)
            pixels = np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)
            pixels = pixels[:,:msg.width*3].reshape(msg.height,msg.width,3)
            if msg.encoding=='bgr8': pixels = pixels[:,:,::-1]
            path = self.output/'raw'/('frame_{:05d}_{:.3f}.png'.format(self.frames,stamp))
            Image.fromarray(pixels).save(str(path))
            tf_data = None
            try:
                tf = self.tf_buffer.lookup_transform('odom','base_link',msg.header.stamp,rospy.Duration(.03))
                tf_data = {'stamp_s':tf.header.stamp.to_sec(),
                           'x':tf.transform.translation.x,'y':tf.transform.translation.y,
                           'z':tf.transform.translation.z}
            except Exception as error: tf_data = {'error':type(error).__name__+':'+str(error)}
            expected = set(self.scene['expected_models'])
            observed = set(row['models'])
            robot = row['models'].get('autolabor_m2')
            # Exact vertical support of each rotated cylinder; link local Y
            # is its cylinder axis after the URDF collision-origin rotation.
            wheels={}
            for n,p in row['links'].items():
                if not n.endswith('_wheel_link'):continue
                axis_z=2*(p['qy']*p['qz']+p['qw']*p['qx'])
                wheels[n]=p['z']-(.15*math.sqrt(max(0.,1.-axis_z*axis_z))+.04*abs(axis_z))
            qc = {'native_resolution':msg.width>=2560 and msg.height>=1440,
                  'unique_expected_models':observed==expected,
                  'clock_settled':stamp>=1.,
                  'frame_model_time_aligned':abs(stamp-row['model_source_stamp_s'])<=.10,
                  'frame_link_time_aligned':0<=stamp-row['link_source_stamp_s']<=.05,
                  'fresh_scan':0<=stamp-row['scan_stamp_s']<=.20,
                  'fresh_odom':0<=stamp-row['odom_stamp_s']<=.10,
                  'tf_at_image_stamp':tf_data is not None and 'error' not in tf_data,
                  'four_wheels_observed':len(wheels)==4,
                  'wheel_ground_gap_within_1cm':len(wheels)==4 and all(abs(v)<=.01 for v in wheels.values()),
                  'no_clock_regression':self.clock_regressions==0}
            if robot and row['odom_pose']:
                spawn = self.scene.get('robot_collision_reference',{}).get('spawn_xyz_rpy',[-5.4,0,0,0,0,0])
                c,s = math.cos(spawn[5]),math.sin(spawn[5])
                ox,oy = row['odom_pose']['x'],row['odom_pose']['y']
                error = math.hypot(robot['x']-(spawn[0]+c*ox-s*oy),robot['y']-(spawn[1]+s*ox+c*oy))
            else: error = None
            qc['world_odom_consistent_with_bounded_stamp_age'] = error is not None and error<=.10
            row.update(frame_id=self.frames, image_path=str(path),
                       image_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                       image_stamp_s=stamp,width=msg.width,height=msg.height,encoding=msg.encoding,
                       camera=self.scene['evidence_camera'],tf_at_frame=tf_data,
                       world_odom_position_error_m=error,wheel_bottom_z_m=wheels,
                       machine_checks=qc,machine_qc='pass' if all(qc.values()) else 'fail',
                       visual_qc='pending',final_size_qc='pending',
                       note='Raw diagnostic frame; no visual approval. Wheel bottom uses rotated cylinder radius0.15/length0.08. General compound collision audit remains separate.')
            with self.lock:
                self.frame_file.write(json.dumps(row,allow_nan=False)+'\n')
                self.frames += 1
                self.machine_pass_frames += int(all(qc.values()))
                self.write_status()
        except Exception as error:
            rospy.logerr('Astra evidence capture error: %s',error)

    def write_status(self):
        data={'run_id':self.run_id,'frames':self.frames,'state_samples':self.states,
              'machine_pass_frames':self.machine_pass_frames,
              'contact_messages':self.contact_messages,'clock_regressions':self.clock_regressions,
              'visual_qc':'pending','contact_coverage':'all collision pairs via ContactManager; source-stamped raw bag required for full empty/nonempty coverage audit'}
        temp=self.output/'status.tmp'
        temp.write_text(json.dumps(data,indent=2)+'\n');os.replace(str(temp),str(self.output/'status.json'))

    def close(self):
        with self.lock:
            self.write_status()
            for f in (self.state_file,self.frame_file,self.contact_file): f.flush()


if __name__=='__main__':
    rospy.init_node('astra_evidence_monitor')
    monitor=Monitor()
    rospy.spin()
