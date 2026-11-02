"""Fresh FAST-LIO and local navigation for recorded MID360 inputs only."""
import copy
import hashlib
import json
from pathlib import Path
import shutil
import time

# Recorded estimator/planner outputs are deliberately absent from this mode.
MAPPING_INPUTS = {
    '/livox/lidar', '/livox/imu', '/scan', '/mid360/scan', '/odom', '/canbus_msg',
    '/m2_driver/chassis_info', '/m2_driver/left_wheel_vel', '/m2_driver/right_wheel_vel',
    '/m2_driver/wheel_angle', '/avoidance/mid360_active', '/avoidance/source_mode',
    '/experiment/http/views', '/experiment/http/vision', '/bev_cleaner/status',
}


def local_rviz(config):
    result=copy.deepcopy(config)
    manager=result['Visualization Manager']
    manager['Global Options']['Fixed Frame']='camera_init'
    manager['Views']['Current'].update({'Target Frame':'base_link','Scale':32,'X':0,'Y':0})
    for display in manager['Displays']:
        if display.get('Name') in ('Static global map','Known 3D global map (optional)',
                                   'Localization aligned scan (dynamic)','Mapping history cloud'):
            display.update(Enabled=False,Value=False)
        if display.get('Name')=='Recorded localization path (estimate)':
            display['Name']='Recomputed FAST-LIO trajectory'
        if display.get('Name') in ('TEB global plan','TEB local plan'):
            display.update(Enabled=True,Value=True)
        if display.get('Name') in ('Global costmap','Local costmap'):
            display['Alpha']=.25
    history=copy.deepcopy(next(d for d in manager['Displays'] if d.get('Name')=='Mapping history cloud'))
    history.update(Name='Incremental map from recorded MID360',Enabled=True,Value=True,Alpha=.9)
    manager['Displays'].append(history)
    return result


class MappingReplay:
    def __init__(self, ws, root, record, spawn, finish):
        import rosbag
        self.ws,self.root,self.spawn,self.finish=ws,root,spawn,finish
        self.workers=[]
        source=Path('/home/slam/robot_j6m_ws_navigation_20260907/devel/lib/fast_lio/fastlio_mapping')
        self.binary=root/'fastlio_mapping'
        shutil.copy2(source,self.binary)
        self.body_tf=None
        with rosbag.Bag(str(sorted(record.glob('localization_*.bag'))[0])) as bag:
            for _,message,_ in bag.read_messages(topics=['/tf','/tf_static']):
                for transform in message.transforms:
                    if (transform.header.frame_id,transform.child_frame_id)==('body','base_link'):
                        p=transform.transform.translation;q=transform.transform.rotation
                        self.body_tf=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
                        break
                if self.body_tf is not None:break
        if self.body_tf is None:raise RuntimeError('Recording has no body -> base_link extrinsic')
        (root/'mapping_inputs.json').write_text(json.dumps(dict(
            binary_source=str(source),binary_sha256=hashlib.sha256(self.binary.read_bytes()).hexdigest(),
            body_to_base=self.body_tf,recorded_input_topics=sorted(MAPPING_INPUTS),
            fixed_frame='camera_init',old_global_map_loaded=False,localization_recomputed=True,
            local_navigation_recomputed=True,vehicle_motion='recorded sensor motion, not simulated response'),indent=2)+'\n')

    def start(self, epoch):
        import rospy
        self.stop()
        for name,value in {'publish/path_en':True,'publish/odom_pub_en':True,'publish/tf_pub_en':True,
                           'publish/scan_publish_en':True,'publish/scan_bodyframe_pub_en':True,
                           'pcd_save/pcd_save_en':False,'runtime_pos_log_enable':False}.items():
            rospy.set_param('/'+name,value)
        output=self.root/('mapping_round_%d'%epoch);output.mkdir(exist_ok=True)
        self.workers.append(self.spawn('fastlio',[str(self.binary),'__name:=laserMapping']))
        self.workers.append(self.spawn('body_tf',['rosrun','tf2_ros','static_transform_publisher']+
            [str(v) for v in self.body_tf]+['body','base_link']))
        self.workers.append(self.spawn('voxel_mapper',[str(self.ws/'devel/lib/robot_bringup/voxel_cloud_mapper'),
            '__name:=replay_voxel_mapper','_output_file:='+str(output/'map.pcd'),
            '_slice_observations_file:='+str(output/'slice_observations.yaml'),
            '_voxel_size:=0.10','_min_frame_observations:=3','_max_point_range:=20',
            '_slice_self_crop_min_x:=-0.52','_slice_self_crop_max_x:=1.22',
            '_slice_self_crop_min_y:=-0.40','_slice_self_crop_max_y:=0.40',
            '_slice_sweep_front:=1.22','_slice_sweep_rear:=0.52','_slice_sweep_half_width:=0.40',
            '_body_to_base_x:='+str(self.body_tf[0]),'_body_to_base_y:='+str(self.body_tf[1]),
            '_body_to_base_z:='+str(self.body_tf[2])]))
        self.workers.append(self.spawn('navigation',['roslaunch',
            str(self.ws/'src/scripts/robot_bringup/launch/navigation_j6m.launch'),
            'localization_source:=fast_lio','use_static_map:=false','start_rviz:=false',
            'odom_topic:=/Odometry','teb_odom_topic:=/odom',
            'output_cmd_vel_topic:=/replay/computed_cmd_vel']))
        # Wait for the real subscriber registrations, before the first IMU/lidar messages.
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            self.check()
            graph=rospy.get_master().getSystemState()[2][1]
            subscribers={topic:set(nodes) for topic,nodes in graph}
            if all('/laserMapping' in subscribers.get(t,set()) for t in ('/livox/lidar','/livox/imu')):
                return
            time.sleep(.1)
        raise RuntimeError('FAST-LIO did not subscribe to recorded lidar and IMU')

    def check(self):
        if any(worker.poll() is not None for worker in self.workers):
            raise RuntimeError('A mapping replay backend exited; inspect fastlio/voxel_mapper/navigation logs')

    def stop(self):
        for worker in reversed(self.workers):self.finish(worker)
        self.workers=[]
