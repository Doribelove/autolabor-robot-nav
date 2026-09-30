#!/usr/bin/env python3
"""Shared offline episode evaluator. Never starts ROS or publishes commands.

Bag header nanoseconds join /astra/world_models and /astra/world_links exactly.
The native Gazebo contact source time is retained separately from its packet
stamp. Geometry is a compound of saved URDF and actual SDF collision primitives,
not the planner's radius. A pose-set backend has no qualified continuous-time
Lipschitz bound: computed swept bounds remain explicitly conditional diagnostics.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml

from geometry import (Pose3, RobotCollisionModel, compound_clearance,
                      project_box, project_cylinder)

MODEL_TOPIC = '/astra/world_models'
LINK_TOPIC = '/astra/world_links'
CONTACT_TOPIC = '/astra/physics_contacts'
METADATA_TOPIC = '/astra/physics_metadata'
WHEELS = ('front_left_wheel_link', 'front_right_wheel_link',
          'rear_left_wheel_link', 'rear_right_wheel_link')
MARGIN_M = .33
TCAP_S = 22.0


def finite(value):
    if isinstance(value, bool):
        raise ValueError('boolean is not numeric evidence')
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('nonfinite numeric evidence')
    return value


def seconds_ns(value):
    return int(round(finite(value)*1e9))


def yaw(pose):
    x, y, z, w = pose.xyzw
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def angle_difference(first, second):
    return math.atan2(math.sin(first-second), math.cos(first-second))


def pose_value(value):
    if isinstance(value, Pose3):
        return value
    if isinstance(value, dict):
        return Pose3(tuple(value[k] for k in ('x', 'y', 'z')),
                     tuple(value[k] for k in ('qx', 'qy', 'qz', 'qw')))
    if len(value) != 7:
        raise ValueError('pose must be xyz qxyzw')
    return Pose3(value[:3], value[3:])


def sdf_pose(element):
    if element is None:
        return Pose3()
    if element.get('relative_to') or element.get('frame'):
        raise ValueError('named SDF pose frames are unsupported, not silently ignored')
    values = [finite(x) for x in (element.text or '').split()]
    if len(values) != 6:
        raise ValueError('SDF pose must be xyz rpy')
    if element.get('degrees', 'false').lower() == 'true':
        values[3:] = [math.radians(v) for v in values[3:]]
    return Pose3.from_xyz_rpy(values[:3], values[3:])


def _minimum(values):
    return min((v for v in values if v is not None), default=None)


class WorldCollisions:
    """Actual SDF collision geometry, including origins and model/link poses.

    Box and cylinder collisions are supported. Horizontal planes are ground
    support surfaces only. Includes/meshes/named pose frames fail qualification.
    Collision-free camera models remain in the expected identity roster.
    """
    def __init__(self, xml, robot_model='autolabor_m2'):
        root = ET.fromstring(xml)
        world = root.find('world') if root.tag == 'sdf' else root
        if world is None or world.tag != 'world':
            raise ValueError('expected SDF world')
        self.models, self.primitives, self.grounds, self.errors = {}, [], [], []
        self.actor_speed_bounds = {}
        self.physics_dt_s = finite(world.findtext('physics/max_step_size', '.001'))
        if self.physics_dt_s <= 0:
            raise ValueError('positive physics step required')
        if world.findall('.//include'):
            self.errors.append('SDF includes require expanded collision geometry')
        for model in world.findall('model'):
            name = model.get('name')
            if not name or name in self.models:
                raise ValueError('duplicate or absent SDF model identity')
            self.models[name] = model
            if name == robot_model:
                continue
            try:
                if model.findall('model'):
                    raise ValueError('nested models require flattened collision geometry')
                model_pose = sdf_pose(model.find('pose'))
                plugins = model.findall('plugin')
                actor = next((p for p in plugins if p.get('filename') == 'libv3_reactive_actor_plugin.so'), None)
                if actor is not None:
                    # The bound includes all targets in the retained V3 actor
                    # source, not only the scene's nominal base speed.
                    self.actor_speed_bounds[name] = max(1.35, finite(actor.findtext('base_speed', '1.0')),
                                                       finite(actor.findtext('initial_speed', actor.findtext('base_speed', '1.0'))))
                elif plugins:
                    self.errors.append('unbounded model plugin: '+name)
                for link in model.findall('link'):
                    link_name = name+'::'+str(link.get('name'))
                    link_pose = sdf_pose(link.find('pose'))
                    for index, collision in enumerate(link.findall('collision')):
                        cid = link_name+'::'+collision.get('name', 'collision_'+str(index))
                        origin = sdf_pose(collision.find('pose'))
                        geom = collision.find('geometry')
                        if geom is None or len(geom) != 1:
                            raise ValueError('collision must contain one geometry')
                        kind = geom[0].tag
                        if kind == 'plane':
                            if model.findtext('static', 'false').lower() != 'true':
                                raise ValueError('moving ground plane unsupported')
                            normal = [finite(x) for x in geom[0].findtext('normal', '0 0 1').split()]
                            pose = model_pose.compose(link_pose).compose(origin)
                            n = pose.rotate(normal)
                            if len(n) != 3 or abs(n[0])+abs(n[1]) > 1e-9 or n[2] <= 0:
                                raise ValueError('only upward horizontal ground planes supported')
                            self.grounds.append({'name': cid, 'z_m': pose.xyz[2], 'model': name,
                                                 'expected_model_pose': model_pose})
                            continue
                        if kind == 'box':
                            dimensions = tuple(finite(x) for x in geom[0].findtext('size').split())
                        elif kind == 'cylinder':
                            dimensions = (finite(geom[0].findtext('radius')), finite(geom[0].findtext('length')))
                        else:
                            raise ValueError('unsupported SDF collision '+kind)
                        self.primitives.append(dict(name=cid, model=name, link=link_name, kind=kind,
                            dimensions=dimensions, model_pose=model_pose, link_pose=link_pose, origin=origin,
                            static=model.findtext('static', 'false').lower() == 'true'))
            except (ValueError, TypeError, AttributeError) as error:
                self.errors.append(name+': '+str(error))
        identities = [p['name'] for p in self.primitives]+[p['name'] for p in self.grounds]
        if len(identities) != len(set(identities)):
            raise ValueError('duplicate world collision identity')

    def project(self, models, links):
        result = []
        for item in self.primitives:
            if item['link'] in links:
                pose = links[item['link']]
                source = 'source_stamped_link_world_pose'
            elif item['static'] and item['model'] in models:
                pose = models[item['model']].compose(item['link_pose'])
                source = 'static_link_from_source_stamped_model_and_SDF'
            else:
                raise ValueError('missing source-stamped collision link '+item['link'])
            pose = pose.compose(item['origin'])
            if item['kind'] == 'box':
                value = project_box(item['name'], item['dimensions'], pose, item['link'], source)
            else:
                value = project_cylinder(item['name'], *item['dimensions'], pose=pose,
                                         link_name=item['link'], pose_source=source)
            result.append(value)
        return result


def robot_motion_bounds(robot, urdf_xml):
    """Finite model-based envelopes, explicitly not a continuous backend proof."""
    root = ET.fromstring(urdf_xml)
    plugin = root.find(".//plugin[@filename='libm2_ackermann_plugin.so']")
    if plugin is None:
        return {'available': False, 'qualification': 'pending', 'reason': 'effective driver plugin not in saved expanded URDF'}
    names = ('maxSpeed', 'maxSteer', 'wheelbase', 'track', 'wheelRadius', 'maxSteeringRate')
    if any(plugin.findtext(name) is None for name in names):
        return {'available': False, 'qualification': 'pending', 'reason': 'missing effective driver bounds; no silent defaults'}
    values = {name: finite(plugin.findtext(name)) for name in names}
    if any(x <= 0 for x in values.values()) or values['maxSteer'] >= math.pi/2:
        raise ValueError('invalid saved driver bounds')
    radius = 0.0
    for collision in robot.collisions:
        extent = (math.sqrt(sum((x/2)**2 for x in collision.dimensions)) if collision.kind == 'box'
                  else math.hypot(collision.dimensions[0], collision.dimensions[1]/2))
        extent += math.sqrt(sum(x*x for x in collision.origin.xyz))
        link = collision.link
        while link in robot.by_child:
            joint = robot.by_child[link]
            if joint.kind == 'prismatic':
                return {'available': False, 'qualification': 'pending', 'reason': 'prismatic joint has no speed/extent envelope'}
            extent += math.sqrt(sum(x*x for x in joint.origin.xyz))
            link = joint.parent
        radius = max(radius, extent)
    speed, steer, base = values['maxSpeed'], values['maxSteer'], values['wheelbase']
    angular = speed*math.tan(steer)/base
    rigid = speed+angular*radius
    denominator = 1-values['track']/(2*base)*math.tan(steer)
    if denominator <= 0:
        return {'available': False, 'qualification': 'pending', 'reason': 'steering geometry singular envelope'}
    steer_rate = values['maxSteeringRate']/(math.cos(steer)**2*denominator**2)
    spin_rate = (speed+angular*values['track']/2)/values['wheelRadius']
    # Sum rates times the full enclosing radius overbounds articulated points.
    # Cylinder spin is geometrically symmetric, but retained polygonal outer
    # prisms rotate; this deliberately loose term also covers their motion.
    point = rigid+(steer_rate+spin_rate)*radius
    return {'available': True, 'qualification': 'pending', 'effective_parameters': values,
            'robot_collision_enclosing_radius_m': radius, 'yaw_rate_bound_radps': angular,
            'rigid_point_speed_bound_mps': rigid, 'steer_joint_rate_model_bound_radps': steer_rate,
            'wheel_spin_model_bound_radps': spin_rate, 'compound_point_speed_model_bound_mps': point,
            'actuator_dynamics_enabled': plugin.findtext('enableActuatorDynamics', 'false').lower() == 'true',
            'reason': 'SetWorldPose/SetPosition are discrete pose changes; no continuous-time Lipschitz qualification',
            'formula': 'v_max + (v_max*tan(steer_max)/wheelbase)*R + (steer_joint_rate + wheel_spin_rate)*R'}


def swept_lower_bound(first_distance, second_distance, lipschitz_mps, dt_s):
    if any(x is None for x in (first_distance, second_distance, lipschitz_mps)):
        return None
    values = [finite(x) for x in (first_distance, second_distance, lipschitz_mps, dt_s)]
    if values[2] < 0 or values[3] <= 0:
        raise ValueError('invalid swept interval or Lipschitz bound')
    return min(values[:2])-values[2]*values[3]/2


def _index_stream(rows, label, issues):
    index = {}
    previous = None
    for number, row in enumerate(rows):
        try:
            stamp = row['stamp_ns']
            if isinstance(stamp, bool) or not isinstance(stamp, int) or stamp <= 0:
                raise ValueError('positive integer source nanoseconds required')
            if previous is not None and stamp <= previous:
                issues.append(label+': duplicate/out-of-order/clock-reset source stamp at row '+str(number))
            previous = stamp
            if stamp in index:
                continue
            if row.get('frame_id') != 'world':
                raise ValueError('source frame is not world')
            if label in ('models', 'links'):
                names, poses = row['names'], row['poses']
                if len(names) != len(poses) or len(names) != len(set(names)) or not names:
                    raise ValueError('invalid name/pose arrays')
                row = dict(row, pose_map={name: pose_value(pose) for name, pose in zip(names, poses)})
            index[stamp] = row
        except (ValueError, KeyError, TypeError) as error:
            issues.append(label+': invalid row '+str(number)+': '+str(error))
    if not index:
        issues.append(label+': source messages absent')
    return index


def policy_window(report, issues):
    try:
        if 'policy_window' in report:
            data = report['policy_window']
            start, end = finite(data['start_s']), finite(data['end_s'])
            source = 'explicit episode policy_window'
        else:
            trace = report.get('control_trace') or report.get('decision_trace') or []
            start = finite(trace[0]['time_s'])
            end = start+finite(report['result']['elapsed_s'])
            source = 'first policy control/decision trace time plus reported elapsed_s'
        if start <= 0 or end < start:
            raise ValueError('invalid policy window')
        return (seconds_ns(start), seconds_ns(min(end, start+TCAP_S)), source)
    except (KeyError, ValueError, TypeError, IndexError) as error:
        issues.append('policy window absent/invalid: '+str(error))
        return None


def _coverage(index, start, end, period_ns, label, issues):
    stamps = sorted(index)
    if not stamps or stamps[0] > start or stamps[-1] < end:
        issues.append(label+': policy window not bracketed by source packets')
    relevant = [s for s in stamps if start-period_ns <= s <= end+period_ns]
    gaps = [b-a for a, b in zip(relevant, relevant[1:])]
    if not gaps or any(gap != period_ns for gap in gaps):
        issues.append(label+': source cadence gap or unexpected interval')
    return {'packet_count': len(index), 'window_adjacent_packet_count': len(relevant),
            'expected_period_ns': period_ns, 'maximum_interval_s': max(gaps)/1e9 if gaps else None,
            'first_stamp_s': stamps[0]/1e9 if stamps else None,
            'last_stamp_s': stamps[-1]/1e9 if stamps else None}


def mask_audit(report, issues):
    method = report.get('method')
    applicable = method in ('pct_scope_rl', 'pct_shadow', 'analytic_prior')
    failures = []
    trace = report.get('decision_trace', [])
    if applicable and not trace:
        failures.append('candidate decision trace absent')
    for number, row in enumerate(trace):
        if not applicable:
            break
        try:
            action, mask = row['executed_action'], row['action_mask']
            if isinstance(action, bool) or not isinstance(action, int) or action < 0 or action >= len(mask):
                raise ValueError('executed action index invalid')
            if any(v not in (0, 1, False, True) for v in mask) or not bool(mask[action]):
                raise ValueError('executed action outside logged mask')
        except (KeyError, TypeError, ValueError) as error:
            failures.append('decision '+str(number)+': '+str(error))
    issues.extend('mask: '+error for error in failures)
    return {'applicable': applicable, 'decision_count': len(trace), 'violations': failures,
            'status': 'pass' if applicable and not failures else ('fail' if failures else 'not_applicable'),
            'scope': 'executed action membership in recorded mask, not proof that mask represents true free space'}


def input_execution_audit(report, issues):
    """Separate actual input/execution evidence from legacy circular geometry."""
    result = report.get('result', {})
    checks = {'gate_valid': report.get('gate_valid') is True,
              'message_contract_flag': report.get('message_contract_pass') is True}
    for name in ('maximum_scan_age_s', 'maximum_odometry_age_s', 'maximum_tf_age_s'):
        try:
            checks[name] = 0 <= finite(result[name]) <= .20
        except (KeyError, TypeError, ValueError):
            checks[name] = False
    for name in ('tf_success_rate', 'scan_timestamp_transform_success_rate'):
        try:
            checks[name] = .999 <= finite(result[name]) <= 1.0
        except (KeyError, TypeError, ValueError):
            checks[name] = False
    for name, lower in (('command_count', 1), ('mask_violation_count', 0)):
        value = result.get(name)
        checks[name] = (isinstance(value, int) and not isinstance(value, bool)
                        and (value >= lower if name == 'command_count' else value == 0))
    if report.get('method') in ('actual_teb', 'dwa'):
        count = result.get('local_plan_count')
        checks['local_plan_count_positive'] = isinstance(count, int) and not isinstance(count, bool) and count > 0
    else:
        count = result.get('decision_count')
        checks['decision_count_matches_trace'] = (isinstance(count, int) and not isinstance(count, bool)
                                                  and count > 0 and count == len(report.get('decision_trace', [])))
    issues.extend('input/execution: '+name for name, passed in checks.items() if not passed)
    return {'checks': checks, 'pass': all(checks.values()),
            'legacy_combined_execution_integrity_pass': report.get('execution_integrity_pass'),
            'legacy_circle_conservative_clearance_m': (result.get('minimum_conservative_clearance_m')
                if result.get('minimum_conservative_clearance_m') is None or math.isfinite(float(result['minimum_conservative_clearance_m'])) else None),
            'legacy_chassis_contact_state_count': result.get('contact_state_count'),
            'legacy_geometry_and_contact_are_used_as_new_gates': False}


def contact_audit(rows, world, start, end, robot_name, actor_names, issues):
    illegal, support, environment, offsets = [], [], [], []
    epochs = set()
    empty = 0
    ground_names = {g['name'] for g in world.grounds}
    for stamp in sorted(rows):
        if not start <= stamp <= end:
            continue
        row = rows[stamp]
        states = row.get('states')
        if not isinstance(states, list):
            issues.append('contacts: missing states array')
            continue
        if not states:
            empty += 1
        for state in states:
            try:
                first, second = state['collision1_name'], state['collision2_name']
                depths, positions, normals = state['depths'], state['positions'], state['normals']
                if not (len(depths) == len(positions) == len(normals) > 0):
                    raise ValueError('nonempty equal contact arrays required')
                for x in depths:
                    finite(x)
                for vector in positions+normals:
                    if len(vector) != 3:
                        raise ValueError('contact vector length')
                    for x in vector:
                        finite(x)
                match = re.search(r'(?:^|;)source_contact_time_s=([^;]+)', state.get('info', ''))
                if not match:
                    raise ValueError('native contact source time absent')
                contact_time = finite(match.group(1))
                epoch = re.search(r'(?:^|;)epoch=(\d+)(?:;|$)', state.get('info', ''))
                if not epoch:
                    raise ValueError('contact clock epoch absent')
                epochs.add(int(epoch.group(1)))
                offset = stamp/1e9-contact_time
                offsets.append(offset)
                if offset < -1e-9 or offset > world.physics_dt_s+1e-9:
                    raise ValueError('contact source time outside declared one-step packet offset')
                record = dict(packet_stamp_s=stamp/1e9, contact_source_time_s=contact_time,
                              collision1=first, collision2=second, maximum_depth_m=max(depths))
                robot_sides = [c for c in (first, second) if c.startswith(robot_name+'::')]
                if robot_sides:
                    other = second if first in robot_sides else first
                    robot_collision = robot_sides[0]
                    link_name = robot_collision.split('::')[1]
                    if len(robot_sides) == 1 and other in ground_names and link_name in WHEELS:
                        record['wheel_link'] = link_name
                        support.append(record)
                    else:
                        illegal.append(record)
                elif any(c.split('::')[0] in actor_names for c in (first, second)) and not any(c in ground_names for c in (first, second)):
                    environment.append(record)
            except (KeyError, TypeError, ValueError, IndexError) as error:
                issues.append('contacts: invalid source evidence: '+str(error))
    if len(epochs) > 1 or any(epoch != 0 for epoch in epochs):
        issues.append('contacts: reset/noninitial physics clock epoch observed')
    return {'illegal_robot_contacts': illegal, 'wheel_ground_support_contacts': support,
            'actor_environment_contacts': environment, 'empty_packet_count_in_window': empty,
            'observed_contact_clock_epochs': sorted(epochs),
            'packet_count': len(rows), 'source_minus_packet_alignment': 'contact source may precede packet by 0..one physics step',
            'packet_minus_source_time_min_s': min(offsets) if offsets else None,
            'packet_minus_source_time_max_s': max(offsets) if offsets else None,
            'contact_absence_interpretation': 'No recorded illegal contact is not collision-free proof; require packet coverage, geometry and backend qualification.'}


def kinematics(rows):
    segments, accelerations, jerks = [], [], []
    path = 0.0
    for a, b in zip(rows, rows[1:]):
        dt = b['stamp_s']-a['stamp_s']
        if dt <= 0:
            continue
        distance = math.hypot(b['robot_x_m']-a['robot_x_m'], b['robot_y_m']-a['robot_y_m'])
        path += distance
        segments.append({'time_s': (a['stamp_s']+b['stamp_s'])/2,
                         'speed_mps': distance/dt,
                         'yaw_rate_radps': angle_difference(b['robot_yaw_rad'], a['robot_yaw_rad'])/dt})
    for a, b in zip(segments, segments[1:]):
        accelerations.append({'time_s': (a['time_s']+b['time_s'])/2,
                             'value': (b['speed_mps']-a['speed_mps'])/(b['time_s']-a['time_s'])})
    for a, b in zip(accelerations, accelerations[1:]):
        jerks.append((b['value']-a['value'])/(b['time_s']-a['time_s']))
    return {'sampled_path_length_m': path,
            'maximum_pose_derived_speed_mps': max((s['speed_mps'] for s in segments), default=None),
            'maximum_absolute_pose_derived_yaw_rate_radps': max((abs(s['yaw_rate_radps']) for s in segments), default=None),
            'maximum_absolute_sampled_acceleration_mps2': max((abs(a['value']) for a in accelerations), default=None),
            'maximum_absolute_sampled_jerk_mps3': max((abs(v) for v in jerks), default=None),
            'definition': 'XY secant speeds and yaw differences; acceleration/jerk use consecutive midpoint time differences, not physics accelerometer measurements'}


def evaluate_records(model_rows, link_rows, contact_rows, metadata, scene, episode_report,
                     world_xml, urdf_xml, robot_name='autolabor_m2'):
    """Evaluate normalized bag records; returns finite report and CSV rows.

    WorldState records: integer stamp_ns, frame_id, names, poses (xyz qxyzw).
    Contact records: same header plus states with names, depths, positions,
    normals and original info. Input list order is retained for reset detection.
    """
    issues, sampled_failures, pending = [], [], []
    world = WorldCollisions(world_xml, robot_name)
    robot = RobotCollisionModel(urdf_xml)
    bounds = robot_motion_bounds(robot, urdf_xml)
    issues.extend('world geometry: '+x for x in world.errors)
    if not world.grounds:
        pending.append('ground support geometry absent')
    elif len({g['z_m'] for g in world.grounds}) != 1:
        pending.append('multiple ground elevations require spatial support association')
    for key in ('scene_id', 'seed'):
        if key not in scene or scene.get(key) != episode_report.get(key):
            issues.append('scene/report identity mismatch or absent: '+key)
    if episode_report.get('method') not in ('pct_scope_rl', 'pct_shadow', 'analytic_prior', 'actual_teb', 'dwa'):
        issues.append('unknown method identity')
    models = _index_stream(model_rows, 'models', issues)
    links = _index_stream(link_rows, 'links', issues)
    contacts = _index_stream(contact_rows, 'contacts', issues)
    window = policy_window(episode_report, issues)
    start, end, window_source = window if window else (0, 0, 'unavailable')
    expected_models = set(scene.get('expected_models', []))
    if not expected_models:
        issues.append('expected world model roster absent')
        expected_models = set(world.models)|{robot_name}
    relevant_model_stamps = {s for s in models if start <= s <= end}
    relevant_link_stamps = {s for s in links if start <= s <= end}
    if relevant_model_stamps != relevant_link_stamps:
        issues.append('model/link source timestamps are not exact one-to-one sets')
    common = sorted(set(models)&set(links))
    if not metadata or metadata.get('schema') != 'astra_physics_evidence_v1':
        issues.append('physics evidence metadata absent or unknown')
        metadata = metadata or {}
    for flag in ('contacts_every_physics_step', 'all_collision_pairs_retained', 'empty_contact_packets_published'):
        if metadata.get(flag) is not True:
            issues.append('physics contact coverage flag missing: '+flag)
    if metadata.get('phase') != 'WorldUpdateEnd' or metadata.get('pose_frame') != 'world':
        issues.append('physics state phase/frame contract absent')
    if metadata.get('max_step_s') != world.physics_dt_s:
        issues.append('physics metadata/SDF step mismatch')
    state_hz = finite(metadata.get('state_hz', 50.0))
    if state_hz <= 0:
        raise ValueError('positive evidence state rate required')
    state_period, contact_period = seconds_ns(1/state_hz), seconds_ns(world.physics_dt_s)
    coverage = {}
    if window:
        coverage = {name: _coverage(index, start, end, period, name, issues)
                    for name, index, period in (('models', models, state_period), ('links', links, state_period),
                                                ('contacts', contacts, contact_period))}
    mask = mask_audit(episode_report, issues)
    execution = input_execution_audit(episode_report, issues)
    actors = {a['model_name'] for a in scene.get('actors', [])}
    if actors != set(world.actor_speed_bounds):
        issues.append('scene actor roster does not match supported actor plugins in actual world SDF')
    contact_result = contact_audit(contacts, world, start, end, robot_name, actors, issues)
    if contact_result['illegal_robot_contacts']:
        sampled_failures.append('measured illegal robot contact')
    if contact_result['actor_environment_contacts']:
        sampled_failures.append('actor/environment collision anomaly')
    goal = scene.get('route', {}).get('goal')
    if not isinstance(goal, (tuple, list)) or len(goal) != 3:
        issues.append('world route goal x/y/yaw absent')
        goal = None
    else:
        goal = tuple(finite(v) for v in goal)
    if scene.get('route', {}).get('frame') != 'world':
        issues.append('route frame must be explicitly world')
    rows, per_model_poses, per_link_poses, first_arrival = [], [], [], None
    ground_z = world.grounds[0]['z_m'] if len({g['z_m'] for g in world.grounds}) == 1 else None
    for stamp in common:
        if not start <= stamp <= end:
            continue
        model_map, link_map = models[stamp]['pose_map'], links[stamp]['pose_map']
        try:
            if set(model_map) != expected_models:
                raise ValueError('unexpected/missing/duplicated model roster: '+repr(sorted(set(model_map)^expected_models)))
            root_pose = model_map[robot_name]
            for ground in world.grounds:
                actual, expected = model_map[ground['model']], ground['expected_model_pose']
                if max(abs(a-b) for a, b in zip(actual.xyz, expected.xyz)) > 1e-6 or abs(sum(a*b for a, b in zip(actual.xyzw, expected.xyzw))) < 1-1e-12:
                    raise ValueError('actual ground model pose differs from support geometry')
            robot_links = {name[len(robot_name)+2:]: pose for name, pose in link_map.items()
                           if name.startswith(robot_name+'::') and name[len(robot_name)+2:] in robot.links}
            # Fixed links merged by Gazebo are reconstructed from their actual
            # parent. Movable wheel/steer links must be present; no zero joints.
            compound = robot.project(root_world_pose=root_pose, link_world_poses=robot_links)
            obstacles = world.project(model_map, link_map)
            clearance = compound_clearance(compound, obstacles, MARGIN_M)
            wheel_z = {wheel: _minimum([p.z_min_m-ground_z for p in compound if p.link_name == wheel])
                       if ground_z is not None else None for wheel in WHEELS}
            pose = robot_links.get(robot.root_link, root_pose)
            position_error = math.hypot(pose.xyz[0]-goal[0], pose.xyz[1]-goal[1]) if goal else None
            yaw_error = abs(angle_difference(yaw(pose), goal[2])) if goal else None
            arrival = goal is not None and position_error <= .25 and yaw_error <= .25
            if arrival and first_arrival is None:
                first_arrival = stamp
            conservative = clearance['minimum_conservative_projected_clearance_m']
            row = {'stamp_ns': stamp, 'stamp_s': stamp/1e9, 'policy_elapsed_s': (stamp-start)/1e9,
                   'robot_x_m': pose.xyz[0], 'robot_y_m': pose.xyz[1], 'robot_z_m': pose.xyz[2],
                   'robot_yaw_rad': yaw(pose), 'robot_component_count': len(compound),
                   'obstacle_component_count': len(obstacles),
                   'physical_xy_clearance_m': clearance['minimum_physical_projected_clearance_m'],
                   'conservative_xy_clearance_m': conservative,
                   'z_screened_physical_xy_clearance_m': clearance['minimum_z_conditioned_projected_clearance_m'],
                   'z_screened_conservative_xy_clearance_m': (clearance['minimum_z_conditioned_projected_clearance_m']-MARGIN_M
                        if clearance['minimum_z_conditioned_projected_clearance_m'] is not None else None),
                   'potential_3d_intersection_count': len(clearance['potential_3d_intersection_pairs']),
                   'worst_pair': '|'.join(clearance['worst_projected_pair'] or []),
                   'position_error_m': position_error, 'yaw_error_rad': yaw_error, 'geometric_goal_arrival': arrival,
                   'maximum_primitive_approximation_error_m': max((p.approximation_error_m for p in compound+tuple(obstacles)), default=0.0),
                   'nonwheel_minimum_bottom_relative_ground_m': _minimum(p.z_min_m-ground_z for p in compound if p.link_name not in WHEELS) if ground_z is not None else None,
                   'world_state_alignment_bound_s': 0.0,
                   'model_swept_lower_bound_m': None}
            row.update({wheel+'_bottom_relative_ground_m': value for wheel, value in wheel_z.items()})
            rows.append(row)
            per_model_poses.append((stamp, model_map))
            per_link_poses.append((stamp, link_map))
            if clearance['potential_3d_intersection_pairs']:
                sampled_failures.append('sampled XY intersection with overlapping Z interval')
            if conservative is not None and conservative < 0:
                sampled_failures.append('sampled conservative XY clearance negative')
            # Raw wheel Z ranges and diagnostic counts are retained below.
            # The 1 mm counter is not a preregistered task failure threshold;
            # ordinary wheel-ground support contact is explicitly permitted.
            if row['nonwheel_minimum_bottom_relative_ground_m'] is not None and row['nonwheel_minimum_bottom_relative_ground_m'] < -.001:
                sampled_failures.append('nonwheel robot collision below ground')
        except (KeyError, ValueError, TypeError) as error:
            issues.append('geometry at '+str(stamp)+': '+str(error))
    if not rows:
        issues.append('no valid source-joined policy geometry rows')
    if set(actors)-set(world.actor_speed_bounds):
        pending.append('one or more actor speed/heading envelopes unknown')
    robot_point = bounds.get('compound_point_speed_model_bound_mps')
    actor_speed = max(world.actor_speed_bounds.values(), default=0.0)
    lipschitz = robot_point+actor_speed if robot_point is not None else None
    for a, b in zip(rows, rows[1:]):
        dt = b['stamp_s']-a['stamp_s']
        b['model_swept_lower_bound_m'] = swept_lower_bound(a['conservative_xy_clearance_m'],
                                                         b['conservative_xy_clearance_m'], lipschitz, dt)
        if bounds.get('available'):
            translation = math.hypot(b['robot_x_m']-a['robot_x_m'], b['robot_y_m']-a['robot_y_m'])
            rotation = abs(angle_difference(b['robot_yaw_rad'], a['robot_yaw_rad']))
            if translation > bounds['effective_parameters']['maxSpeed']*dt+1e-6:
                issues.append('observed robot source-pose jump exceeds driver speed envelope')
            if rotation > bounds['yaw_rate_bound_radps']*dt+1e-6:
                issues.append('observed robot yaw jump exceeds driver steering envelope')
    for (ta, a), (tb, b) in zip(per_model_poses, per_model_poses[1:]):
        for name in actors:
            if name not in a or name not in b or name not in world.actor_speed_bounds:
                continue
            delta = math.hypot(b[name].xyz[0]-a[name].xyz[0], b[name].xyz[1]-a[name].xyz[1])
            if delta > world.actor_speed_bounds[name]*(tb-ta)/1e9+1e-6:
                issues.append('observed actor source-pose jump exceeds configured actor envelope: '+name)
            if abs(angle_difference(yaw(b[name]), yaw(a[name]))) > 1e-6:
                pending.append('actor heading changes without qualified angular speed bound: '+name)
        for name, model in world.models.items():
            if name in actors or name == robot_name or name not in a or name not in b:
                continue
            if model.findtext('static', 'false').lower() == 'true':
                if max(abs(x-y) for x, y in zip(a[name].xyz, b[name].xyz)) > 1e-6 or abs(sum(x*y for x, y in zip(a[name].xyzw, b[name].xyzw))) < 1-1e-12:
                    issues.append('static world model moved without a motion bound: '+name)
    if robot_point is not None:
        for (ta, a), (tb, b) in zip(per_link_poses, per_link_poses[1:]):
            for name in set(a)&set(b):
                if name.startswith(robot_name+'::') and math.sqrt(sum((x-y)**2 for x, y in zip(a[name].xyz, b[name].xyz))) > robot_point*(tb-ta)/1e9+1e-6:
                    issues.append('observed robot link position jump exceeds compound model envelope: '+name)
    pending.append('continuous swept clearance unqualified for pose-set backend')
    supported = {p['wheel_link'] for p in contact_result['wheel_ground_support_contacts']}
    contact_result['wheel_links_without_measured_support_contact'] = sorted(set(WHEELS)-supported)
    if set(WHEELS)-supported:
        pending.append('not all four wheels have measured support contacts; geometric wheel height alone is not support-force evidence')
    illegal_stamps = [c['packet_stamp_s'] for c in contact_result['illegal_robot_contacts']]
    overlap_rows = [r for r in rows if r['potential_3d_intersection_count']]
    unmatched_overlap = [r['stamp_s'] for r in overlap_rows
                         if not any(abs(t-r['stamp_s']) <= 1/state_hz+world.physics_dt_s for t in illegal_stamps)]
    # A mismatch is diagnostic, not proof of detector failure: XY/Z can false
    # positive in 3D, and a contact may occur between 50 Hz geometry samples.
    issues = sorted(set(issues))
    failures = sorted(set(sampled_failures))
    integrity = not issues
    observed_success = first_arrival is not None and integrity and not failures
    raw_elapsed = (first_arrival-start)/1e9 if first_arrival is not None else None
    result = {'schema': 'astra_shared_posthoc_evaluator_v1', 'mode': 'exact_source_stamped_bag',
              'identity': {k: episode_report.get(k) for k in ('stage', 'split', 'method', 'scene_id', 'seed')},
              'policy_window': {'start_s': start/1e9 if window else None, 'end_s': end/1e9 if window else None,
                                'source': window_source, 'cap_s': TCAP_S},
              'integrity': {'pass': integrity, 'issues': issues,
                            'original_message_contract_pass': episode_report.get('message_contract_pass'),
                            'original_execution_integrity_pass': episode_report.get('execution_integrity_pass'),
                            'input_execution_separated_from_legacy_geometry': execution,
                            'source_model_link_alignment_bound_s': 0.0, 'coverage': coverage},
              'task': {'raw_geometric_arrival': first_arrival is not None,
                       'first_arrival_source_stamp_s': first_arrival/1e9 if first_arrival is not None else None,
                       'raw_geometric_arrival_elapsed_s': raw_elapsed,
                       'goal_position_tolerance_m': .25, 'goal_yaw_tolerance_rad': .25,
                       'observed_task_integrity_success': observed_success,
                       'failure_inclusive_observed_time_s': raw_elapsed if observed_success else TCAP_S,
                       'observed_time_scope': 'development diagnostic; integrity/contact/sampled-clearance failures receive Tcap; pending continuous safety is reported separately',
                       'fully_safety_qualified_success': False,
                       'failure_inclusive_qualified_time_s': TCAP_S},
              'safety': {'sampled_status': 'fail' if failures else ('pass_within_sampled_scope' if integrity else 'unavailable'),
                         'sampled_failures': failures, 'qualification': 'pending', 'pending': sorted(set(pending)),
                         'minimum_physical_xy_clearance_m': _minimum(r['physical_xy_clearance_m'] for r in rows),
                         'minimum_conservative_xy_clearance_m': _minimum(r['conservative_xy_clearance_m'] for r in rows),
                         'minimum_z_screened_physical_xy_clearance_m': _minimum(r['z_screened_physical_xy_clearance_m'] for r in rows),
                         'minimum_z_screened_conservative_xy_clearance_m': _minimum(r['z_screened_conservative_xy_clearance_m'] for r in rows),
                         'physical_xy_definition': 'unmargined compound projection gap; exact boxes and outward-approximated cylinders, not exact 3D distance',
                         'margin_m': MARGIN_M, 'sample_count': len(rows),
                         'swept': {'qualification': 'pending', 'robot_bounds': bounds,
                                   'actor_speed_model_bounds_mps': world.actor_speed_bounds,
                                   'combined_point_speed_model_bound_mps': lipschitz,
                                   'minimum_conditional_conservative_bound_m': _minimum(r['model_swept_lower_bound_m'] for r in rows),
                                   'formula': 'min(d0,d1)-L*dt/2, conditional on an L-Lipschitz continuous geometric clearance, unproved for pose-set backend'}},
              'contacts': contact_result, 'mask_trace': mask, 'kinematics': kinematics(rows),
              'wheel_height': {wheel: {'minimum_m': _minimum(r[wheel+'_bottom_relative_ground_m'] for r in rows),
                                    'maximum_m': max((r[wheel+'_bottom_relative_ground_m'] for r in rows if r[wheel+'_bottom_relative_ground_m'] is not None), default=None),
                                    'above_0_005m_count': sum(r[wheel+'_bottom_relative_ground_m'] is not None and r[wheel+'_bottom_relative_ground_m'] > .005 for r in rows),
                                    'below_minus_0_001m_count': sum(r[wheel+'_bottom_relative_ground_m'] is not None and r[wheel+'_bottom_relative_ground_m'] < -.001 for r in rows)} for wheel in WHEELS},
              'contact_geometry_crosscheck': {'projected_overlap_samples_without_nearby_illegal_contact': unmatched_overlap,
                    'interpretation': 'potential contact omissions for review, not proven misses; XY+Z screening is not exact 3D collision and contacts have 0..one-step phase offset'},
              'limits': ['Finite source samples and simulator contacts do not establish formal safety.',
                         'Cylinder XY is an outer 48-gon approximation with reported per-primitive Hausdorff bound; cylinder Z extrema and box eight-corner projection are exact.',
                         'Negative XY clearance is convex-pair minimum separating translation, not 3D penetration or compound translation depth.',
                         'No privileged world states or this evaluator are planner inputs.',
                         'Contact wrench/forces are not measured by the evidence plugin.'],
              'formal_result': False, 'runtime_qualified': False}
    json.dumps(result, allow_nan=False)
    return result, rows


def read_bag(path):
    """Import rosbag only here; reading a file does not initialize a ROS node."""
    import rosbag
    streams = {MODEL_TOPIC: [], LINK_TOPIC: [], CONTACT_TOPIC: []}
    metadata = None
    with rosbag.Bag(str(path), 'r') as bag:
        for topic, message, _receipt in bag.read_messages(topics=list(streams)+[METADATA_TOPIC]):
            if topic == METADATA_TOPIC:
                incoming = json.loads(message.data)
                if metadata is not None and metadata != incoming:
                    raise ValueError('physics metadata changed during recording')
                metadata = incoming
                continue
            row = {'stamp_ns': int(message.header.stamp.to_nsec()), 'frame_id': message.header.frame_id}
            if topic in (MODEL_TOPIC, LINK_TOPIC):
                row.update(names=list(message.name), poses=[(p.position.x, p.position.y, p.position.z,
                           p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w) for p in message.pose])
            else:
                row['states'] = [dict(collision1_name=s.collision1_name, collision2_name=s.collision2_name,
                    depths=list(s.depths), positions=[(p.x, p.y, p.z) for p in s.contact_positions],
                    normals=[(p.x, p.y, p.z) for p in s.contact_normals], info=s.info) for s in message.states]
            streams[topic].append(row)
    return streams[MODEL_TOPIC], streams[LINK_TOPIC], streams[CONTACT_TOPIC], metadata


def file_binding(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    return {'path': str(Path(path).resolve()), 'sha256': digest.hexdigest()}


def evaluate_run(run, legacy=False):
    """Read immutable run evidence and bind every input; failures stay finite."""
    run = Path(run)
    sources = [run/'scene_manifest.yaml', run/'world.sdf', run/'episode_report.yaml']
    rows = []
    if legacy:
        sources += [run/'evidence/states.jsonl', run/'evidence/contacts.jsonl']
        result = {'schema': 'astra_shared_posthoc_evaluator_v1', 'mode': 'legacy_receipt_time_diagnostic_only',
                  'integrity': {'pass': False, 'issues': ['source-stamped WorldState/contact packet coverage unavailable; do not fabricate exact joins']},
                  'safety': {'qualification': 'pending'}, 'task': {'failure_inclusive_qualified_time_s': TCAP_S},
                  'formal_result': False, 'runtime_qualified': False}
    else:
        sources += [run/'episode_report.yaml.inputs.bag', run/'evidence/robot_description.xml',
                    run/'evidence/effective_parameters.yaml']
        sources += [run/name for name in ('allocated.json', 'receipt.json', 'bindings.json') if (run/name).is_file()]
        try:
            bindings_before = [file_binding(p) for p in sources]
            model, link, contact, metadata = read_bag(sources[3])
            episode = yaml.safe_load(sources[2].read_text())
            result, rows = evaluate_records(model, link, contact, metadata,
                yaml.safe_load(sources[0].read_text()), episode, sources[1].read_text(), sources[4].read_text())
            extra_issues = []
            if episode.get('scene_manifest_sha256') != bindings_before[0]['sha256']:
                extra_issues.append('episode scene manifest SHA does not match actual evaluated manifest')
            parameters = yaml.safe_load(sources[5].read_text())
            if parameters.get('/use_sim_time') is not True or parameters.get('/m2_gazebo', {}).get('simulation_only') is not True:
                extra_issues.append('saved effective simulation-only parameters absent/false')
            if (run/'allocated.json').is_file():
                allocation = json.loads((run/'allocated.json').read_text())
                if allocation.get('run_id') != run.name or allocation.get('seed') != episode.get('seed'):
                    extra_issues.append('allocation/run/report identity mismatch')
            if (run/'receipt.json').is_file():
                receipt = json.loads((run/'receipt.json').read_text())
                if receipt.get('run_id') != run.name or any(receipt.get(k) != episode.get(k) for k in ('seed', 'method')):
                    extra_issues.append('receipt/run/report identity mismatch')
                if receipt.get('report_sha256') and receipt['report_sha256'] != bindings_before[2]['sha256']:
                    extra_issues.append('receipt episode report hash mismatch')
            if bindings_before != [file_binding(p) for p in sources]:
                extra_issues.append('source evidence changed during posthoc evaluation')
            if extra_issues:
                result['integrity']['issues'].extend(extra_issues)
                result['integrity']['pass'] = False
                result['task']['observed_task_integrity_success'] = False
                result['task']['failure_inclusive_observed_time_s'] = TCAP_S
        except Exception as error:
            result = {'schema': 'astra_shared_posthoc_evaluator_v1', 'mode': 'exact_source_input_failure',
                      'integrity': {'pass': False, 'issues': [type(error).__name__+': '+str(error)]},
                      'safety': {'qualification': 'pending'},
                      'task': {'failure_inclusive_observed_time_s': TCAP_S, 'failure_inclusive_qualified_time_s': TCAP_S},
                      'formal_result': False, 'runtime_qualified': False}
    result['source_bindings'] = [file_binding(p) if p.is_file() else {'path': str(p.resolve()), 'sha256': None, 'missing': True}
                                 for p in sources]
    result['evaluator_bindings'] = [file_binding(Path(__file__)), file_binding(Path(__file__).with_name('geometry.py'))]
    return result, rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='new output directory; never overwrite evidence')
    parser.add_argument('--legacy', action='store_true', help='explicit unavailable-source audit for old receipt-time evidence')
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('evaluation output already exists')
    result, rows = evaluate_run(args.run, args.legacy)
    args.output.mkdir(parents=True)
    csv_path = args.output/'per_step.csv'
    with csv_path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else ['stamp_ns'])
        writer.writeheader()
        writer.writerows(rows)
    result['per_step_csv'] = file_binding(csv_path)
    (args.output/'evaluation.json').write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False)+'\n')
    print(json.dumps({'mode': result['mode'], 'output': str(args.output), 'integrity_pass': result['integrity']['pass'],
                      'safety_qualification': result['safety']['qualification']}, ensure_ascii=False))
    return 0 if result['integrity']['pass'] else 3


if __name__ == '__main__':
    raise SystemExit(main())
