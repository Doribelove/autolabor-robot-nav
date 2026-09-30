#!/usr/bin/env python3
"""Derive unexecuted A02 inputs into a separately bound common environment.

The parent geometry/parameter/episode seeds are retained. This is not a new
independent geometry sample and does not authorize execution. Original inputs
are only read, never rewritten. Nonidentity planner routes fail closed.
"""
import argparse
import copy
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import yaml

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from route_world_adapter import RouteFrame
STAGE=HERE.parents[2]
VERSION='astra_live_actor_input_derivation_v2'
ENVIRONMENT='astra_common_environment_v4_routegate_ego_clock_fault_guard_z0_evidence'
MAPPING=STAGE/'audits/actor_live_parameter_mapping_r1.json'
PLUGIN=STAGE/'workspace/src/simulation/m2_gazebo/src/astra_reactive_actor_plugin.cpp'
GATE=STAGE/'workspace/src/simulation/m2_gazebo/include/m2_gazebo/astra_actor_gate.h'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()


def transformed_actor(actor,frame):
    """World path + box initial pose helper; planner SE2 bridge is separate."""
    out=frame.actor_path_to_world(copy.deepcopy(actor))
    initial=list(actor['initial_pose_xyz_rpy'])
    initial[:2]=frame.point_to_world(initial[:2])
    initial[5]=math.atan2(math.sin(initial[5]+frame.yaw_rad),math.cos(initial[5]+frame.yaw_rad))
    out['initial_pose_xyz_rpy']=initial
    return out


def derive(scene,world_text,frame=RouteFrame()):
    if frame!=RouteFrame():
        raise ValueError('Nonidentity full scene forbidden until planner/map/evaluator route bridge is live')
    if scene['route']['start']!=[-4.,0.,0.] or scene['route']['goal']!=[5.,0.,0.] or scene['route']['spawn']!=[-5.4,0.,0.]:
        raise ValueError('Unexpected route; current common environment uses declared yaw-zero legacy route')
    out=copy.deepcopy(scene);root=ET.fromstring(world_text);world=root.find('world')
    if world is None:raise ValueError('No SDF world')
    transformed=[]
    for actor in out['actors']:
        actor=transformed_actor(actor,frame)
        model=world.find("model[@name='%s']"%actor['model_name'])
        if model is None:raise ValueError('Actor model missing')
        plugin=model.find('plugin')
        if plugin is None or plugin.get('filename')!='libv3_reactive_actor_plugin.so':
            raise ValueError('Expected retained original actor plugin')
        values={'base_speed':actor['base_speed_mps'],'initial_speed':actor['base_speed_mps'],
                'response_distance':actor['response_distance_m'],'response_time_margin':actor['response_time_margin_s'],
                'seed':actor['response_seed']}
        if plugin.findtext('policy')!=actor['policy']:raise ValueError('Input policy disagreement')
        for key,value in values.items():
            if float(plugin.findtext(key))!=value:raise ValueError('Input SDF/manifest disagreement: '+key)
        for key in ('start','end','conflict'):
            if list(map(float,plugin.findtext(key).split()))!=actor[key]:raise ValueError('Input path disagreement')
        if not math.isfinite(actor['gate_delay_s']) or not 0.<=actor['gate_delay_s']<=1.:
            raise ValueError('Actor gate delay outside frozen range')
        plugin.set('filename','libastra_reactive_actor_plugin.so')
        for old in ('start_delay','activation_ego_x'):
            element=plugin.find(old)
            if element is None:raise ValueError('Missing expected legacy gate field')
            plugin.remove(element)
        for key,value in [('route_origin','0 0'),('route_yaw','0'),('gate_route_x','-4.02'),('gate_delay',repr(actor['gate_delay_s']))]:
            if plugin.find(key) is not None:raise ValueError('Duplicate new gate input')
            ET.SubElement(plugin,key).text=value
        actor['live_plugin']='libastra_reactive_actor_plugin.so'
        actor['gate_relative_delay_wired']=True
        actor['response_semantics']='parameterized_deterministic_frozen_v3_rule'
        actor['response_seed_behavioral_effect']=False
        actor['nominal_arrival_difference_with_gate_delay_s']=actor['nominal_arrival_difference_s']+actor['gate_delay_s']
        transformed.append(actor)
    out['actors']=transformed
    out['environment_version']=ENVIRONMENT
    out['input_variant_id']=out['layout_id']+'_live_gate_v2'
    out['input_derivation']={'version':VERSION,'script_sha256':sha(__file__),
        'parameter_mapping_path':str(MAPPING),'parameter_mapping_sha256':sha(MAPPING),
        'actor_plugin_source_sha256':sha(PLUGIN),'gate_helper_sha256':sha(GATE),
        'parent_geometry_identity_retained':True,'parent_episode_seed_retained_unexecuted_input':True,
        'new_independent_geometry':False,'live_execution_started':False}
    out['actor_activation']={'trigger':'first_observed_ego_route_projection_crossing',
        'route_origin_xy':[0.,0.],'route_yaw_rad':0.,'gate_route_x_m':-4.02,
        'delay_source':'actor.gate_delay_s','clock':'simulation_source_time',
        'clock_regression':'permanent_actor_fault_requires_new_episode_including_Load_before_first_update',
        'ego_disappearance':'permanent_actor_fault_after_first_observation_no_catchup_integration',
        'initial_ego_spawn':'allowed_with_clock_anchored_and_actor_fixed_at_start',
        'preactivation_progress_m':0.,'boundary_integration':'clip_each_physics_step_to_active_interval',
        'live_timing_qualification':'pending'}
    out['randomization_scope']={'parameter_stream':'rng_substreams.actor_parameters is bound to sampled policy,base_speed,response_distance,response_time_margin,gate_delay',
        'gazebo_seed':'episode seed must be passed as --seed; scan noise uses Gazebo global RNG',
        'unused_reserved_streams':['rng_substreams.actor_response','rng_substreams.sensor_noise','rng_substreams.actuator'],
        'actor_response_seed':'passed but inherited zero-jitter rule means no behavioral effect',
        'actuator_jitter':'command_jitter=0; no random actuator delay',
        'unused_reserved_streams_block_execution':False}
    out['readiness']['gate_relative_actor_delay']=True
    out['readiness']['gate_relative_actor_delay_live_qualified']=False
    out['readiness']['named_rng_wired_to_all_live_plugins']=False
    out['readiness']['declared_active_rng_scope_mapping_complete']=True
    out['readiness']['final_execution']=False
    return out,'<?xml version="1.0"?>\n'+ET.tostring(root,encoding='unicode')+'\n'


def build(source,output):
    source=Path(source).resolve();output=Path(output).resolve()
    if output.exists():raise FileExistsError('No overwrite of input variants')
    if not MAPPING.is_file():raise ValueError('Parameter mapping must be materialized before executable derivation')
    rows=list(csv.DictReader((source/'diagnostic_blocks.csv').open()));pending=[]
    before={str(source/'diagnostic_blocks.csv'):sha(source/'diagnostic_blocks.csv')}
    for row in rows:
        mp,wp=Path(row['manifest_path']),Path(row['world_path'])
        if sha(mp)!=row['manifest_sha256'] or sha(wp)!=row['world_sha256']:raise ValueError('Retained A02 input hash disagreement')
        before.update({str(mp):sha(mp),str(wp):sha(wp)})
        scene,world=derive(yaml.safe_load(mp.read_text()),wp.read_text())
        scene['input_derivation'].update(parent_manifest_path=str(mp),parent_manifest_sha256=sha(mp),parent_world_path=str(wp),parent_world_sha256=sha(wp))
        pending.append((row,scene,world))
    output.mkdir(parents=True);derived=[]
    for old,scene,text in pending:
        folder=output/scene['layout_id']/('replicate_%02d'%scene['replicate']);folder.mkdir(parents=True)
        mp,wp=folder/'scene_manifest.yaml',folder/'world.sdf';mp.write_text(yaml.safe_dump(scene,sort_keys=False));wp.write_text(text)
        row=dict(old);row.update(world_path=str(wp),world_sha256=sha(wp),manifest_path=str(mp),manifest_sha256=sha(mp),
            input_variant_id=scene['input_variant_id'],environment_version=ENVIRONMENT,eligibility='component_wired_pending_common_environment_live_qualification')
        derived.append(row)
    with (output/'diagnostic_blocks.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(derived[0]));w.writeheader();w.writerows(derived)
    retained=all(sha(p)==digest for p,digest in before.items())
    if not retained:raise RuntimeError('Source inputs changed concurrently; derived inputs invalid')
    report={'status':'live_actor_variant_materialized_component_wired_live_qualification_pending',
        'source_input_set':str(source),'output_input_set':str(output),'environment_version':ENVIRONMENT,
        'episode_inputs':len(derived),'independent_geometry_count':len({r['geometry_sha256'] for r in derived}),
        'new_independent_geometries':0,'episode_seed_policy':'retain existing unexecuted A02 proposed episode seeds; common variant used by all methods',
        'parent_input_hashes_preserved':retained,'parent_bindings':before,
        'bindings':[{'path':str(p),'sha256':sha(p)} for p in [Path(__file__),MAPPING,PLUGIN,GATE,HERE/'route_world_adapter.py']],
        'materialized_gate_delays_s':sorted({a['gate_delay_s'] for _,s,_ in pending for a in s['actors']}),
        'actual_actor_randomization':'offline named actor parameter stream; conditional live reaction remains deterministic',
        'nonidentity_live_route_supported':False,'actor_plugin_se2_projection_supported':True,
        'required_runtime_checks':['binary/source binding and plugin loaded for every actor','gate/source-time delay and zero preactivation pose progress; no actor gate_fault logs','source-stamped state/contact/image/bag qualification','spawn and steering physical envelope qualification','all methods use same derived environment and inputs'],
        'reserved_unused_streams_are_not_execution_blockers':True,'execution_started':False,'final_outcomes_accessed':False}
    (output/'derivation_report.json').write_text(json.dumps(report,indent=2)+'\n')
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',required=True,type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
    r=build(a.source,a.output);print(json.dumps({k:r[k] for k in ['status','episode_inputs','new_independent_geometries','parent_input_hashes_preserved','materialized_gate_delays_s']},indent=2))
