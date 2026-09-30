#!/usr/bin/env python3
"""Shared-execution wrapper, preserving frozen R2 algorithm/inference methods.

Only wait_ready/warmup are replaced. No Gazebo lifecycle call. R2's internal
report clock remains diagnostic; common coordinator/evaluator own formal time.
"""
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import threading
import time
from urllib.parse import urlparse


def load_frozen_class(workspace):
    path=workspace/'src/application/v5_e2e_scope_rl/scripts/pct_teb_dwa_benchmark_r2_episode.py'
    spec=importlib.util.spec_from_file_location('astra_shared_frozen_r2',path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.R2BenchmarkEpisode,path


def main():
    import rospy
    from std_msgs.msg import String
    rospy.init_node('astra_frozen_r2_method')
    uri=urlparse(os.environ.get('ROS_MASTER_URI',''))
    if uri.scheme!='http' or uri.hostname not in ('127.0.0.1','localhost','::1') or uri.username or uri.password:raise ValueError('loopback simulation required')
    if rospy.get_param('/use_sim_time',False)is not True or rospy.get_param('/m2_gazebo/simulation_only',False)is not True:raise ValueError('simulation markers required')
    workspace=Path(os.environ['THESIS_WS']).resolve();base_class,r2_path=load_frozen_class(workspace)
    run_id=str(rospy.get_param('~run_id'));seed=rospy.get_param('~seed');method='frozen_r2_v060_shared_activation'
    if rospy.get_param('~method')!='pct_scope_rl' or rospy.get_param('~candidate_variant')!='r2_candidate_v060':raise ValueError('exact frozen R2 PCT configuration required')
    status_pub=rospy.Publisher('/astra/method/status',String,queue_size=100,latch=True)
    identity_pub=rospy.Publisher('/astra/frozen_r2_execution_identity',String,queue_size=1,latch=True)
    shared={'gate':None,'fault':None};lock=threading.RLock()
    def publish_status(state,ready=False,**fields):
        row={'run_id':run_id,'seed':seed,'method':method,'state':state,'ready':ready,'reason':shared['fault']}
        row.update(fields);status_pub.publish(String(data=json.dumps(row,separators=(',',':'))))
    def on_gate(message):
        with lock:
            try:
                payload=json.loads(message.data)
                if payload.get('schema')!='astra_common_navigation_v1' or (payload.get('run_id'),payload.get('seed'),payload.get('method'))!=(run_id,seed,method):raise ValueError('shared R2 gate identity mismatch')
                if type(payload.get('source_ns'))is not int or payload['source_ns']<=0:raise ValueError('invalid gate source stamp')
                if shared['gate'] is not None and payload!=shared['gate']:raise ValueError('shared gate changed')
                shared['gate']=payload
            except Exception as exc:shared['fault']=str(exc);publish_status('fault')
    subscriber=rospy.Subscriber('/astra/navigation/gate',String,on_gate,queue_size=1)
    class SharedExecutionR2(base_class):
        def wait_ready(self):
            # Frozen callbacks populate observations unchanged. This wrapper
            # never calls the inherited Gazebo spawn/unpause readiness helper.
            deadline=time.monotonic()+90.
            while not rospy.is_shutdown() and time.monotonic()<deadline:
                now=rospy.Time.now().to_sec()
                good=(now>0 and self.odom is not None and self.scan is not None and self.model_states is not None
                      and self.ego_model in self.model_index and all(name in self.model_index for name in self.actor_models)
                      and self.odom_received_s is not None and self.scan_received_s is not None
                      and 0<=now-self.odom_received_s<=.2 and 0<=now-self.scan_received_s<=.2
                      and len(self.scan_stamps)>=6)
                if shared['fault']:raise RuntimeError(shared['fault'])
                if good:publish_status('ready',True);return
                publish_status('bootstrap');time.sleep(.02)
            raise RuntimeError('frozen R2 inputs not ready')
        def warmup(self):
            # Coordinator owns all moving warmup commands. No method command,
            # goal or inherited warmup loop is executed before the shared gate.
            deadline=time.monotonic()+180.
            while not rospy.is_shutdown() and time.monotonic()<deadline:
                with lock:
                    if shared['fault']:raise RuntimeError(shared['fault'])
                    gate_payload=shared['gate']
                    if gate_payload is not None and rospy.Time.now().to_nsec()>=gate_payload['source_ns']:
                        pose=gate_payload['pose_map'];twist=gate_payload['body_twist']
                        publish_status('active',True,gate_source_ns=gate_payload['source_ns'],ack_semantics='frozen execute_sensor begins after shared gate; internal report clock diagnostic')
                        return {'duration_s':(gate_payload['source_ns']-gate_payload['activation_source_ns'])/1e9,
                            'gate_pose':{'x_m':pose[0],'y_m':pose[1],'yaw_rad':pose[2]},'gate_speed_mps':twist[0],
                            'trace_tail':[],'shared_actual_odom_gate_source_ns':gate_payload['source_ns'],'shared_execution_protocol':'astra_common_navigation_v1'}
                publish_status('ready',True);time.sleep(.002)
            raise RuntimeError('shared gate did not arrive')
    instance=SharedExecutionR2()
    if (instance.spawn_world_x,instance.policy_start_world_x,instance.goal_world_x)!=(-5.4,-4.,5.):raise ValueError('frozen R2 legacy x-route constraint')
    r1_path=workspace/'src/application/v5_e2e_scope_rl/scripts/pct_teb_dwa_benchmark_episode.py'
    identity_pub.publish(String(data=json.dumps({'run_id':run_id,'seed':seed,'method':method,'algorithm_sources_unchanged':True,
        'r2_sha256':hashlib.sha256(r2_path.read_bytes()).hexdigest(),'r1_sha256':hashlib.sha256(r1_path.read_bytes()).hexdigest(),
        'actor_sha256':instance.actor_checkpoint_sha256,'shared_route_consumed':False,'old_full_episode_protocol_unchanged':False,
        'scene_manifest_sha256':hashlib.sha256(Path(instance.scene_manifest_path).read_bytes()).hexdigest(),
        'static_prior_box_count':len(instance.scene.get('static_boxes',[])),
        'static_input_adapter':instance.scene.get('r2_static_input_adapter'),
        'static_prior_consumer_semantics_equal':False,
        'static_prior_limitations':'unchanged R2 implicit y walls at +/-20m and outside-map semantics; no fixed global route consumption; raster union is independently audited input data only',
        'overrides':['wait_ready','warmup'],'formal_time_authority':'shared coordinator actual odom gate; old report clock diagnostic'})))
    try:
        result=instance.run();publish_status('complete',False,internal_report_goal_reached=result.get('result',{}).get('goal_reached'))
        rospy.spin()
    except Exception as exc:
        shared['fault']=type(exc).__name__+':'+str(exc);publish_status('fault');instance.publish_command(0.,0.);rospy.spin()

if __name__=='__main__':main()
