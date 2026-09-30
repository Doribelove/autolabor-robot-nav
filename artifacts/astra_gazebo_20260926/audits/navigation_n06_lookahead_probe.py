"""Pure old-trace steering-demand probe for a future lookahead-only factor."""
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

import rosbag

S = Path(__file__).resolve().parents[1]
W = S / 'workspace'
sys.path.insert(0, str(W / 'scripts/astra_gazebo'))
from axle_reference_adapter import base_pose_to_rear
from common_route import CommonRoute
from route_world_adapter import wrap_angle

RUN = S / 'runs/astra_navigation_n04_b00_astra_analytic_route_v1'
OUT = S / 'audits/navigation_n06_lookahead_hypothesis'
TARGET_X = (-3.0, -2.5, -2.0, -1.75, -1.5, -1.25, -1.0)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def desired(route, base, lookahead):
    rear = base_pose_to_rear(base, .65)
    projection = route.project(base[:2])
    sample = route.sample(projection['arc_m'] + lookahead)
    target = base_pose_to_rear(sample, .65)
    bearing = wrap_angle(math.atan2(target[1]-rear[1],target[0]-rear[0])-base[2])
    raw = math.atan2(1.3*math.sin(bearing), max(.05, math.dist(rear[:2],target[:2])))
    return max(-.4964, min(.4964, raw)), projection


def main():
    if OUT.exists():
        raise FileExistsError('probe identity exists')
    contract = json.loads((RUN/'contract.json').read_bytes())
    bundle_path = Path(contract['shared_bundle']['path'])
    if sha(bundle_path) != contract['shared_bundle']['sha256']:
        raise ValueError('shared route binding changed')
    bundle = json.loads(bundle_path.read_bytes())
    if bundle['map_from_odom_xy_yaw'] != [-5.4, 0., 0.]:
        raise ValueError('unexpected frame transform')
    route = CommonRoute(bundle['points_xy'])
    actual_joints = {}
    samples = []
    with rosbag.Bag(str(RUN/'fixture.bag')) as bag:
        for _, msg, _ in bag.read_messages(topics=['/joint_states']):
            actual_joints[msg.header.stamp.to_nsec()] = dict(zip(msg.name,msg.position))
        remaining = list(TARGET_X)
        for _, msg, _ in bag.read_messages(topics=['/odom']):
            if not remaining:
                break
            x_odom = msg.pose.pose.position.x
            x_map = x_odom - 5.4
            if x_map < remaining[0]:
                continue
            q = msg.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            base = (x_map,msg.pose.pose.position.y,yaw)
            old, projection = desired(route,base,.7)
            candidate,_ = desired(route,base,1.2)
            stamp = msg.header.stamp.to_nsec()
            joints = actual_joints[stamp]
            samples.append(dict(threshold_map_x_m=remaining.pop(0),source_ns=stamp,
                                elapsed_after_gate_s=(stamp-7_460_000_000)/1e9,
                                base_map_x_m=x_map,base_map_y_m=base[1],base_yaw_rad=yaw,
                                route_lateral_m=projection['lateral_m'],
                                actual_left_steer_rad=joints['front_left_steer_joint'],
                                actual_right_steer_rad=joints['front_right_steer_joint'],
                                original_lookahead_m=.7,original_desired_steer_rad=old,
                                proposed_lookahead_m=1.2,proposed_desired_steer_rad=candidate))
    if len(samples) != len(TARGET_X):
        raise ValueError('insufficient observed crossing states')
    OUT.mkdir()
    with (OUT/'old_trace_steering_demand.csv').open('x',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(samples[0]))
        writer.writeheader();writer.writerows(samples)
    report=dict(status='pure_old_trace_steering_demand_hypothesis_only',
                old_attempt='astra_navigation_n04_b00_astra_analytic_route_v1',
                old_attempt_task_phase='timeout',
                source_bag_sha256=sha(RUN/'fixture.bag'),
                source_assessment_sha256=sha(RUN/'assessment.json'),
                source_contract_sha256=sha(RUN/'contract.json'),
                shared_route_sha256=sha(bundle_path),
                samples=samples,
                factor='route-following lookahead 0.7m to 1.2m only',
                interpretation='At fixed observed states a longer lookahead requests less early left steering before the first route corner; it does not predict the closed-loop trajectory, safety, time or success.',
                N04_rerun_allowed=False,simulation_episode_allocated=False,formal_result=False)
    (OUT/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(status=report['status'],samples=len(samples),
                          x_minus2_original=samples[2]['original_desired_steer_rad'],
                          x_minus2_candidate=samples[2]['proposed_desired_steer_rad'])))


if __name__=='__main__':
    main()
