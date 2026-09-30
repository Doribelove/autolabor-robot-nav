"""Independent evaluator fixtures; no ROS master or simulator is started."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import sys

import pytest

DIRECTORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DIRECTORY))
import evaluate_episode as evaluator


URDF = '''<robot name="test_m2"><link name="base_link"><collision name="body">
<origin xyz="0 0 .3"/><geometry><box size=".8 .4 .2"/></geometry></collision></link>
%s
<gazebo><plugin name="driver" filename="libm2_ackermann_plugin.so">
<maxSpeed>2.778</maxSpeed><maxSteer>.4964</maxSteer><wheelbase>.65</wheelbase>
<track>.60</track><wheelRadius>.10</wheelRadius><maxSteeringRate>.80</maxSteeringRate>
<enableActuatorDynamics>true</enableActuatorDynamics></plugin></gazebo></robot>''' % ''.join(
    '''<link name="%s"><collision name="wheel"><origin rpy="1.5707963267948966 0 0"/>
    <geometry><cylinder radius=".1" length=".06"/></geometry></collision></link>
    <joint name="%s_joint" type="fixed"><parent link="base_link"/><child link="%s"/>
    <origin xyz="%s %s .1"/></joint>''' % (name, name, name, x, y)
    for name, x, y in zip(evaluator.WHEELS, (.3, .3, -.3, -.3), (.23, -.23, .23, -.23)))


def sdf_box(name, x, y, sx=1., sy=1., yaw=0., actor=False):
    plugin = ('<plugin name="actor" filename="libv3_reactive_actor_plugin.so">'
              '<base_speed>.8</base_speed><initial_speed>.8</initial_speed></plugin>') if actor else ''
    return '''<model name="%s"><static>true</static><pose>%s %s .5 0 0 %s</pose>
      <link name="body"><collision name="collision"><geometry><box><size>%s %s 1</size></box>
      </geometry></collision></link>%s</model>''' % (name, x, y, yaw, sx, sy, plugin)


def fixture(obstacle=None):
    obstacle = obstacle or sdf_box('wall', 4, 3)
    world = '''<sdf version="1.6"><world name="fixture"><physics type="ode"><max_step_size>.002</max_step_size></physics>
      <model name="ground_plane"><static>true</static><link name="body"><collision name="floor"><geometry>
      <plane><normal>0 0 1</normal><size>30 30</size></plane></geometry></collision></link></model>%s</world></sdf>''' % obstacle
    import xml.etree.ElementTree as ET
    parsed = ET.fromstring(world).find('world')
    entities = {model.get('name'): model for model in parsed.findall('model')}
    static = {}
    actor_names = []
    for name, model in entities.items():
        p = evaluator.sdf_pose(model.find('pose'))
        static[name] = tuple(p.xyz)+tuple(p.xyzw)
        if model.find('plugin') is not None:
            actor_names.append(name)
    scene = {'scene_id': 'fixture', 'seed': 123, 'route': {'frame': 'world', 'goal': [.5, 0., 0.]},
             'actors': [{'model_name': name} for name in actor_names],
             'expected_models': list(entities)+['autolabor_m2']}
    result = {'elapsed_s': .38, 'command_count': 8, 'mask_violation_count': 0, 'decision_count': 2,
              'maximum_scan_age_s': .1, 'maximum_odometry_age_s': .02, 'maximum_tf_age_s': .02,
              'tf_success_rate': 1., 'scan_timestamp_transform_success_rate': 1.,
              'minimum_conservative_clearance_m': -10., 'contact_state_count': 999}
    report = {'scene_id': 'fixture', 'seed': 123, 'method': 'analytic_prior',
              'policy_window': {'start_s': 1., 'end_s': 1.38}, 'gate_valid': True,
              'message_contract_pass': True, 'execution_integrity_pass': False,
              'result': result, 'decision_trace': [
                  {'time_s': 1., 'executed_action': 1, 'action_mask': [1, 1]},
                  {'time_s': 1.25, 'executed_action': 1, 'action_mask': [1, 1]}]}
    models, links = [], []
    for index in range(20):
        stamp = 1_000_000_000+index*20_000_000
        root = (index*.02, 0., 0., 0., 0., 0., 1.)
        models.append({'stamp_ns': stamp, 'frame_id': 'world', 'names': list(static)+['autolabor_m2'],
                       'poses': list(static.values())+[root]})
        links.append({'stamp_ns': stamp, 'frame_id': 'world', 'names': [name+'::body' for name in static]+['autolabor_m2::base_link'],
                      'poses': list(static.values())+[root]})
    contacts = [{'stamp_ns': 1_000_000_000+index*2_000_000, 'frame_id': 'world', 'states': []} for index in range(191)]
    metadata = {'schema': 'astra_physics_evidence_v1', 'max_step_s': .002, 'state_hz': 50.,
                'phase': 'WorldUpdateEnd', 'pose_frame': 'world', 'contacts_every_physics_step': True,
                'all_collision_pairs_retained': True, 'empty_contact_packets_published': True}
    return dict(model_rows=models, link_rows=links, contact_rows=contacts, metadata=metadata,
                scene=scene, episode_report=report, world_xml=world, urdf_xml=URDF)


def add_contact(data, first, second, packet=20, offset=0.):
    row = data['contact_rows'][packet]
    source = row['stamp_ns']/1e9-offset
    row['states'].append({'collision1_name': first, 'collision2_name': second,
                          'depths': [.0001], 'positions': [[0., 0., 0.]], 'normals': [[0., 0., 1.]],
                          'info': 'source_contact_time_s='+str(source)+';epoch=0;forces_omitted'})


def test_exact_source_join_goal_and_legacy_geometry_separated():
    data = fixture()
    report, rows = evaluator.evaluate_records(**data)
    assert report['integrity']['pass'], report['integrity']['issues']
    assert report['task']['raw_geometric_arrival']
    assert report['task']['first_arrival_source_stamp_s'] == pytest.approx(1.26)
    assert report['task']['failure_inclusive_observed_time_s'] == pytest.approx(.26)
    assert report['task']['fully_safety_qualified_success'] is False
    assert report['safety']['qualification'] == 'pending'
    assert report['safety']['swept']['robot_bounds']['qualification'] == 'pending'
    assert report['integrity']['input_execution_separated_from_legacy_geometry']['legacy_geometry_and_contact_are_used_as_new_gates'] is False
    assert all(r['robot_component_count'] == 5 for r in rows)
    assert rows[0]['physical_xy_clearance_m']-rows[0]['conservative_xy_clearance_m'] == pytest.approx(.33)
    assert report['contacts']['empty_packet_count_in_window'] == 191
    json.dumps(report, allow_nan=False)
    json.dumps(rows, allow_nan=False)


def test_side_wheel_collision_outside_chassis_is_detected():
    data = fixture(sdf_box('side_box', .3, .34, .1, .18))
    report, rows = evaluator.evaluate_records(**data)
    assert rows[0]['physical_xy_clearance_m'] < 0
    assert 'wheel_link' in rows[0]['worst_pair']
    assert report['safety']['sampled_status'] == 'fail'
    assert report['task']['failure_inclusive_observed_time_s'] == 22.
    assert report['contact_geometry_crosscheck']['projected_overlap_samples_without_nearby_illegal_contact']


def test_rotated_robot_and_goal_yaw_use_world_geometry():
    data = fixture(sdf_box('turn_box', 0., .45, .2, .2))
    for stream in ('model_rows', 'link_rows'):
        for row in data[stream]:
            row['poses'][-1] = (0., 0., 0., 0., 0., math.sqrt(.5), math.sqrt(.5))
    data['scene']['route']['goal'] = [0., 0., math.pi/2]
    report, rows = evaluator.evaluate_records(**data)
    assert rows[0]['physical_xy_clearance_m'] < 0
    assert report['task']['raw_geometric_arrival']
    assert rows[0]['yaw_error_rad'] == pytest.approx(0.)
    data['scene']['route']['goal'][2] = 0.
    report, _ = evaluator.evaluate_records(**data)
    assert not report['task']['raw_geometric_arrival']


def test_actor_box_uses_actual_yaw_and_dimensions_and_fixed_response_speed_bound():
    data = fixture(sdf_box('actor_1', 0., .72, .72, .72, math.pi/4, True))
    report, rows = evaluator.evaluate_records(**data)
    # A rotated .72 box reaches .72/sqrt(2) toward the robot; a .36 radius
    # substitute would report a materially larger side gap.
    assert rows[0]['physical_xy_clearance_m'] < .72-.36-.2
    assert report['safety']['swept']['actor_speed_model_bounds_mps']['actor_1'] == 1.35


def astra_actor_box(initial=.8):
    return sdf_box('actor_1', 4., 3., .72, .72, actor=True).replace(
        'libv3_reactive_actor_plugin.so', 'libastra_reactive_actor_plugin.so').replace(
        '<initial_speed>.8</initial_speed>', '<initial_speed>'+str(initial)+'</initial_speed>').replace(
        '</plugin>', '''<ego_model>autolabor_m2</ego_model><policy>hesitant</policy>
        <start>4 3</start><end>4 -3</end><conflict>4 0</conflict>
        <route_origin>0 0</route_origin><route_yaw>.5</route_yaw>
        <gate_route_x>-4.02</gate_route_x><gate_delay>.3</gate_delay></plugin>''')


@pytest.mark.parametrize('initial,expected', [(0., 1.35), (.8, 1.35), (2.1, 2.1)])
def test_astra_actor_initial_and_all_response_targets_bound_without_swept_promotion(initial, expected):
    data = fixture(astra_actor_box(initial))
    report, _ = evaluator.evaluate_records(**data)
    assert report['integrity']['pass'], report['integrity']['issues']
    swept = report['safety']['swept']
    assert swept['actor_speed_model_bounds_mps']['actor_1'] == expected
    envelope = swept['actor_model_envelopes']['actor_1']
    assert envelope['target_speeds_mps'] == {'base': .8, 'cautious': .12,
        'assertive': 1.35, 'hesitant_slow': .32, 'hesitant_fast': .6240000000000001}
    assert envelope['qualification'] == swept['qualification'] == 'pending'
    assert 'zero twist is not a measured speed bound' in envelope['limit']
    assert not report['task']['fully_safety_qualified_success']


@pytest.mark.parametrize('old,new', [
    ('<gate_delay>.3</gate_delay>', ''),
    ('<gate_delay>.3</gate_delay>', '<gate_delay>-.1</gate_delay>'),
    ('<end>4 -3</end>', '<end>4 3</end>'),
    ('<base_speed>.8</base_speed>', '<base_speed>1.4</base_speed>'),
    ('<initial_speed>0.8</initial_speed>', '<initial_speed>-1</initial_speed>'),
    ('<policy>hesitant</policy>', '<policy>unknown</policy>'),
    ('</plugin>', '<start_delay>0</start_delay></plugin>'),
    ('</plugin>', '<ego_observation_period>.1</ego_observation_period></plugin>'),
    ('</plugin>', '</plugin><plugin name="other" filename="libunknown.so"/>')])
def test_astra_actor_invalid_or_composed_plugins_never_receive_a_partial_bound(old, new):
    data = fixture(astra_actor_box().replace(old, new))
    report, _ = evaluator.evaluate_records(**data)
    assert not report['integrity']['pass']
    assert report['safety']['swept']['combined_point_speed_model_bound_mps'] is None
    assert report['task']['failure_inclusive_observed_time_s'] == 22.


def test_astra_actor_observed_jump_is_not_excused_by_known_filename():
    data = fixture(astra_actor_box())
    for stream in ('model_rows', 'link_rows'):
        row = data[stream][10]
        index = next(i for i, name in enumerate(row['names']) if name.startswith('actor_1'))
        pose = list(row['poses'][index]); pose[0] += .5; row['poses'][index] = pose
    report, _ = evaluator.evaluate_records(**data)
    assert any('actor source-pose jump' in issue for issue in report['integrity']['issues'])
    assert not report['integrity']['pass']
    assert report['safety']['swept']['qualification'] == 'pending'


def test_actual_astra_live_world_actor_rosters_and_envelopes():
    stage = DIRECTORY.parents[2]
    worlds = sorted((stage/'scenes/development_a02_live_v1').glob('*/replicate_*/world.sdf'))
    assert len(worlds) == 12
    import yaml
    for path in worlds:
        scene = yaml.safe_load(path.with_name('scene_manifest.yaml').read_text())
        world = evaluator.WorldCollisions(path.read_text())
        assert not world.errors, world.errors
        assert set(world.actor_speed_bounds) == {a['model_name'] for a in scene['actors']}
        assert all(e['qualification'] == 'pending' for e in world.actor_envelopes.values())


def test_native_gated_actor_rule_respects_envelope_and_stopped_prefix(tmp_path):
    # The reference is the actual C++ production rule, not a Python rewrite.
    import subprocess
    workspace = DIRECTORY.parents[1]
    source = tmp_path/'native_actor_bound.cpp'
    source.write_text(r'''
#include <m2_gazebo/astra_actor_gate.h>
#include <algorithm>
#include <cmath>
#include <iostream>
int main() {
  unsigned long samples = 0;
  for (double base : {.4, .8, 1.2}) for (double initial : {0., .5, 1.35, 2.1})
  for (auto policy : {m2_gazebo::V3ActorPolicy::kCautious,
                     m2_gazebo::V3ActorPolicy::kAssertive,
                     m2_gazebo::V3ActorPolicy::kHesitant}) {
    m2_gazebo::V3ReactiveActorConfig cfg; cfg.base_speed_mps=base; cfg.policy=policy;
    m2_gazebo::V3ReactiveActorState state; state.speed_mps=initial;
    m2_gazebo::AstraRouteFrame route;
    m2_gazebo::AstraGateRelativeActivation gate(route, .2, .3);
    gate.observeWithoutEgo(0.);
    double previous=0., bound=std::max(initial, 1.35);
    for (int i=1;i<=5000;++i) {
      const double time=.002*i, ego_x=i<100 ? 0. : .3;
      const auto g=gate.update(time, ego_x, 0.);
      const double old_progress=state.progress_m;
      state=m2_gazebo::advanceAstraGatedActor(state,cfg,.5,.5,
          i%600<300 ? base : 0.,0.,g);
      if (!g.valid || state.speed_mps>bound+1e-12 || state.speed_mps<0. ||
          state.progress_m-old_progress>bound*(time-previous)+1e-12 ||
          (time<=.5 && state.progress_m!=0.)) return 1;
      previous=time; ++samples;
    }
  }
  std::cout << samples << " native gated samples passed\n";
}
''')
    binary = tmp_path/'native_actor_bound'
    subprocess.run(['g++', '-std=c++11', '-O2', '-I'+str(workspace/'src/simulation/m2_gazebo/include'),
                    str(source), '-o', str(binary)], check=True, capture_output=True, text=True)
    completed = subprocess.run([str(binary)], check=True, capture_output=True, text=True)
    assert completed.stdout.strip() == '180000 native gated samples passed'


def test_actual_sdf_cylinder_collision_and_z_disjoint_screen():
    data = fixture('''<model name="overhead"><static>true</static><pose>0 0 3 0 0 0</pose>
    <link name="body"><collision name="c"><geometry><cylinder><radius>.3</radius><length>.2</length>
    </cylinder></geometry></collision></link></model>''')
    report, rows = evaluator.evaluate_records(**data)
    assert rows[0]['physical_xy_clearance_m'] < 0
    assert rows[0]['z_screened_physical_xy_clearance_m'] is None
    assert rows[0]['potential_3d_intersection_count'] == 0
    assert rows[0]['maximum_primitive_approximation_error_m'] > 0


@pytest.mark.parametrize('stream', ['model_rows', 'link_rows', 'contact_rows'])
def test_missing_topic_fails_closed_and_remains_finite(stream):
    data = fixture();data[stream] = []
    report, rows = evaluator.evaluate_records(**data)
    assert not report['integrity']['pass']
    assert report['task']['failure_inclusive_observed_time_s'] == 22.
    json.dumps(report, allow_nan=False)


def test_empty_contact_packets_are_coverage_but_never_collision_free_proof():
    data = fixture()
    report, _ = evaluator.evaluate_records(**data)
    assert report['integrity']['pass']
    assert report['contacts']['packet_count'] == 191
    assert not report['contacts']['illegal_robot_contacts']
    assert report['safety']['qualification'] == 'pending'
    assert 'not collision-free proof' in report['contacts']['contact_absence_interpretation']
    del data['contact_rows'][40]
    report, _ = evaluator.evaluate_records(**data)
    assert not report['integrity']['pass']
    assert any('cadence gap' in x for x in report['integrity']['issues'])


@pytest.mark.parametrize('mutation', ['mismatch', 'reset', 'duplicate', 'wrong_frame', 'identity'])
def test_source_integrity_rejects_bad_timestamps_frames_and_identity(mutation):
    data = fixture()
    if mutation == 'mismatch':
        data['link_rows'][5]['stamp_ns'] += 1
    elif mutation == 'reset':
        data['model_rows'][5]['stamp_ns'] = 500_000_000
    elif mutation == 'duplicate':
        data['link_rows'].insert(4, copy.deepcopy(data['link_rows'][3]))
    elif mutation == 'wrong_frame':
        data['model_rows'][5]['frame_id'] = 'odom'
    else:
        data['episode_report']['seed'] = 124
    report, _ = evaluator.evaluate_records(**data)
    assert not report['integrity']['pass']
    assert report['task']['failure_inclusive_observed_time_s'] == 22.


def test_four_wheels_only_ground_support_exception_and_actor_screen_anomaly():
    data = fixture()
    for index, wheel in enumerate(evaluator.WHEELS):
        add_contact(data, 'autolabor_m2::'+wheel+'::wheel', 'ground_plane::body::floor', packet=20+index)
    report, _ = evaluator.evaluate_records(**data)
    assert len(report['contacts']['wheel_ground_support_contacts']) == 4
    assert not report['contacts']['illegal_robot_contacts']
    assert not report['contacts']['wheel_links_without_measured_support_contact']
    add_contact(data, 'autolabor_m2::base_link::body', 'ground_plane::body::floor')
    report, _ = evaluator.evaluate_records(**data)
    assert len(report['contacts']['illegal_robot_contacts']) == 1
    assert report['task']['failure_inclusive_observed_time_s'] == 22.
    data = fixture(sdf_box('actor_1', 4., 3., .72, .72, actor=True)+sdf_box('screen', 5., 3.))
    add_contact(data, 'actor_1::body::collision', 'screen::body::collision')
    report, _ = evaluator.evaluate_records(**data)
    assert len(report['contacts']['actor_environment_contacts']) == 1
    assert 'actor/environment collision anomaly' in report['safety']['sampled_failures']


@pytest.mark.parametrize('offset,passes', [(0., True), (.002, True), (.004, False), (-.002, False)])
def test_contact_source_phase_is_retained_and_checked(offset, passes):
    data = fixture()
    add_contact(data, 'autolabor_m2::front_left_wheel_link::wheel', 'ground_plane::body::floor', offset=offset)
    report, _ = evaluator.evaluate_records(**data)
    assert report['integrity']['pass'] is passes
    assert report['contacts']['packet_minus_source_time_min_s'] == pytest.approx(offset)


def test_observed_pose_jump_and_mask_violation_fail_inclusively():
    data = fixture()
    data['episode_report']['decision_trace'][0]['action_mask'] = [1, 0]
    report, _ = evaluator.evaluate_records(**data)
    assert report['task']['raw_geometric_arrival']
    assert not report['integrity']['pass']
    assert report['task']['failure_inclusive_observed_time_s'] == 22.
    data = fixture()
    for stream in ('model_rows', 'link_rows'):
        row = data[stream][10]
        pose = row['poses'][-1]
        row['poses'][-1] = (4.,)+pose[1:]
    report, _ = evaluator.evaluate_records(**data)
    assert any('pose jump' in x for x in report['integrity']['issues'])


def test_constant_speed_midpoint_acceleration_and_yaw_are_zero():
    rows = [{'stamp_s': 1.+i*.02, 'robot_x_m': 3.*(1.+i*.02), 'robot_y_m': .4,
             'robot_yaw_rad': .3} for i in range(20)]
    result = evaluator.kinematics(rows)
    assert result['maximum_pose_derived_speed_mps'] == pytest.approx(3.)
    assert result['maximum_absolute_sampled_acceleration_mps2'] < 1e-10
    assert result['maximum_absolute_sampled_jerk_mps3'] < 1e-8
    assert result['maximum_absolute_pose_derived_yaw_rate_radps'] == 0.
    assert evaluator.swept_lower_bound(.5, .7, 2., .1) == pytest.approx(.4)
    assert evaluator.swept_lower_bound(None, .7, 2., .1) is None


def test_nonuniform_sample_acceleration_uses_midpoint_time_not_raw_step():
    times = [0., .07, .2, .28, .5]
    rows = [{'stamp_s': t, 'robot_x_m': .5*2*t*t, 'robot_y_m': 0., 'robot_yaw_rad': 0.} for t in times]
    result = evaluator.kinematics(rows)
    assert result['maximum_absolute_sampled_acceleration_mps2'] == pytest.approx(2.)
    assert result['maximum_absolute_sampled_jerk_mps3'] < 1e-10


def test_timeout_missing_motion_bound_and_unsupported_sdf_are_not_passes():
    data = fixture();data['scene']['route']['goal'] = [10., 0., 0.]
    data['urdf_xml'] = data['urdf_xml'].replace('<maxSpeed>2.778</maxSpeed>', '')
    report, _ = evaluator.evaluate_records(**data)
    assert not report['task']['raw_geometric_arrival']
    assert report['task']['failure_inclusive_observed_time_s'] == 22.
    assert report['safety']['swept']['robot_bounds']['available'] is False
    assert report['safety']['swept']['minimum_conditional_conservative_bound_m'] is None
    data = fixture();data['world_xml'] = data['world_xml'].replace('<box><size>1.0 1.0 1</size></box>', '<mesh><uri>opaque.stl</uri></mesh>')
    report, _ = evaluator.evaluate_records(**data)
    assert any('unsupported SDF collision' in x for x in report['integrity']['issues'])


def test_external_planner_uses_same_geometry_and_requires_local_plan_evidence():
    data = fixture();r = data['episode_report'];r['method'] = 'actual_teb';r['decision_trace'] = []
    r['result']['local_plan_count'] = 4
    report, _ = evaluator.evaluate_records(**data)
    assert report['integrity']['pass']
    assert report['mask_trace']['status'] == 'not_applicable'
    r['result']['local_plan_count'] = 0
    report, _ = evaluator.evaluate_records(**data)
    assert not report['integrity']['pass']


def test_legacy_mode_is_explicit_and_cannot_fake_exact_source_stamps(tmp_path, monkeypatch):
    run, output = tmp_path/'legacy', tmp_path/'evaluation'
    (run/'evidence').mkdir(parents=True)
    for relative in ('scene_manifest.yaml', 'world.sdf', 'episode_report.yaml',
                     'evidence/states.jsonl', 'evidence/contacts.jsonl'):
        (run/relative).write_text('{}\n')
    monkeypatch.setattr(sys, 'argv', ['evaluate_episode.py', '--run', str(run), '--output', str(output), '--legacy'])
    assert evaluator.main() == 3
    report = json.loads((output/'evaluation.json').read_text())
    assert report['mode'] == 'legacy_receipt_time_diagnostic_only'
    assert not report['integrity']['pass']
    assert report['safety']['qualification'] == 'pending'
    with pytest.raises(FileExistsError):
        evaluator.main()


def test_missing_run_sources_produce_durable_finite_failure_not_empty_success(tmp_path, monkeypatch):
    run, output = tmp_path/'missing_run', tmp_path/'evaluation'
    run.mkdir()
    monkeypatch.setattr(sys, 'argv', ['evaluate_episode.py', '--run', str(run), '--output', str(output)])
    assert evaluator.main() == 3
    result = json.loads((output/'evaluation.json').read_text())
    assert result['mode'] == 'exact_source_input_failure'
    assert not result['integrity']['pass']
    assert result['task']['failure_inclusive_observed_time_s'] == 22.
    assert any(item.get('missing') for item in result['source_bindings'])


def test_source_binding_checks_use_actual_manifest_not_report_boolean(tmp_path, monkeypatch):
    import yaml
    data = fixture()
    run = tmp_path/'source_binding_fixture'
    (run/'evidence').mkdir(parents=True)
    (run/'scene_manifest.yaml').write_text(yaml.safe_dump(data['scene']))
    (run/'world.sdf').write_text(data['world_xml'])
    (run/'evidence/robot_description.xml').write_text(data['urdf_xml'])
    (run/'evidence/effective_parameters.yaml').write_text(yaml.safe_dump({'/use_sim_time': True, '/m2_gazebo': {'simulation_only': True}}))
    (run/'episode_report.yaml.inputs.bag').write_bytes(b'explicit offline mocked bag fixture')
    data['episode_report']['scene_manifest_sha256'] = evaluator.file_binding(run/'scene_manifest.yaml')['sha256']
    (run/'episode_report.yaml').write_text(yaml.safe_dump(data['episode_report']))
    monkeypatch.setattr(evaluator, 'read_bag', lambda path: tuple(data[k] for k in ('model_rows', 'link_rows', 'contact_rows', 'metadata')))
    result, rows = evaluator.evaluate_run(run)
    assert result['integrity']['pass']
    assert result['source_bindings'][0]['sha256'] == data['episode_report']['scene_manifest_sha256']
    (run/'scene_manifest.yaml').write_text((run/'scene_manifest.yaml').read_text()+'# post-report mutation\n')
    result, _ = evaluator.evaluate_run(run)
    assert not result['integrity']['pass']
    assert any('SHA' in s for s in result['integrity']['issues'])
    assert result['task']['failure_inclusive_observed_time_s'] == 22.


def test_saved_real_m2_urdf_reconstructs_nine_components_from_actual_legacy_link_sample():
    # This is a component compatibility check only. Receipt-time legacy rows
    # are never promoted to source-stamped episode evaluation evidence.
    stage = DIRECTORY.parents[2]
    xml = (stage/'audits/geometry_urdf.xml').read_text()
    path = stage/'runs/smoke_a02_02_analytic_prior/evidence/states.jsonl'
    with path.open() as stream:
        sample = next(json.loads(line) for line in stream if 'autolabor_m2::rear_right_wheel_link' in line)
    model = evaluator.RobotCollisionModel(xml)
    poses = {name.split('::', 1)[1]: evaluator.pose_value(pose) for name, pose in sample['links'].items()
             if name.startswith('autolabor_m2::') and name.split('::', 1)[1] in model.links}
    compound = model.project(link_world_poses=poses)
    assert len(compound) == 9
    assert set(evaluator.WHEELS).issubset({p.link_name for p in compound})
    bounds = evaluator.robot_motion_bounds(model, xml)
    assert bounds['effective_parameters']['maxSpeed'] == 2.778
    assert bounds['effective_parameters']['maxSteer'] == .4964
    assert bounds['effective_parameters']['wheelbase'] == .65
    assert bounds['qualification'] == 'pending'


def test_wheel_ground_small_penetration_counter_is_not_an_undeclared_failure_gate():
    data = fixture()
    for stream in ('model_rows', 'link_rows'):
        for row in data[stream]:
            pose = row['poses'][-1]
            row['poses'][-1] = pose[:2]+(-.002,)+pose[3:]
    report, _ = evaluator.evaluate_records(**data)
    assert report['integrity']['pass']
    assert report['safety']['sampled_failures'] == []
    for wheel in evaluator.WHEELS:
        assert report['wheel_height'][wheel]['below_minus_0_001m_count'] == 20
        assert report['wheel_height'][wheel]['minimum_m'] == pytest.approx(-.002)
    assert report['safety']['qualification'] == 'pending'
