#!/usr/bin/env python3
"""Offline qualification of retained V2 input geometry; never starts ROS."""
import csv,hashlib,importlib.util,json,math,sys,time
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
import shapely
S=Path(__file__).resolve().parents[1];W=S/'workspace';D=S/'scenes/development_a02'
def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
old=load('retained_a01_auditor',S/'audits/scene_execution_readiness_source.py')
g=load('new_generator',W/'scripts/astra_gazebo/scene_generator_v2.py')
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    rows=list(csv.DictReader((D/'diagnostic_blocks.csv').open()));lr=[];er=[];wd=S/'audits/scene_witnesses_v2';wd.mkdir(exist_ok=True)
    for row in rows:
        scene=yaml.safe_load(Path(row['manifest_path']).read_text());world=ET.parse(row['world_path']).getroot().find('world')
        problems=old.sdf_consistency(scene,row['world_path'])
        cam=world.find("model[@name='astra_evidence_camera']/pose");cp=list(map(float,cam.text.split()))
        if cp!=scene['evidence_camera']['pose_xyz_rpy']:problems.append('camera_pose')
        if cp!=[0.,0.,19.,0.,math.pi/2,math.pi/2]:problems.append('camera_target_pose')
        plugin=world.find("plugin[@name='astra_evidence']")
        if plugin is None or plugin.get('filename')!='libastra_evidence_plugin.so' or float(plugin.findtext('state_hz'))!=50.:problems.append('source_stamped_plugin')
        if sha(row['world_path'])!=row['world_sha256'] or sha(row['manifest_path'])!=row['manifest_sha256']:problems.append('retained_input_hash')
        er.append({'layout_id':scene['layout_id'],'seed':scene['seed'],'replicate':scene['replicate'],'input_consistency_issues':problems,'actors':old.actor_semantics(scene)})
        if scene['replicate']!=0:continue
        start=time.monotonic();path=old.static_path(scene);points=path.pop('path');wp=wd/(scene['layout_id']+'.json')
        wp.write_text(json.dumps({'layout_id':scene['layout_id'],'state_fields':['x_m','y_m','yaw_rad','steering_rad'],'path':points},indent=2)+'\n')
        lr.append({'layout_id':scene['layout_id'],'family':scene['family'],'static_ackermann':path,'witness_path':str(wp),'witness_sha256':sha(wp),'offline_semantics':g.geometry_semantics(scene),'road_width':old.road_width(scene),'wall_s':time.monotonic()-start})
        print(scene['layout_id'],path['status'],path.get('expanded_nodes'),flush=True)
    actors=[a for e in er for a in e['actors']];prior=json.loads((S/'scenes/development_a01/generation_report.json').read_text());gen=json.loads((D/'generation_report.json').read_text())
    new_seeds=set(gen['rng_records'])-set(prior['rng_records'])
    checks={'six_independent_layouts':len(lr)==6,'twelve_episode_inputs':len(er)==12,'all_static_ackermann_witness':all(r['static_ackermann']['status']=='static_forward_ackermann_witness' for r in lr),'all_actor_swept_clearance_pass':all(r['offline_semantics']['actor_swept_static_pass'] for r in lr),'all_initial_and_reveal_visibility_pass':all(r['offline_semantics'].get('initial_low_visibility_pass',True) and r['offline_semantics'].get('geometric_reveal_pass',True) for r in lr),'all_sdf_and_manifest_consistent':not any(e['input_consistency_issues'] for e in er),'all_prior_seed_records_preserved':all(gen['rng_records'][k]==v for k,v in prior['rng_records'].items()),'all_episodes_fresh_against_a01':all(str(e['seed']) in new_seeds for e in er),'geometry_registry_clear':not gen['geometry_registry_issues']}
    deps=[Path(__file__),S/'audits/scene_execution_readiness_source.py',W/'scripts/astra_gazebo/scene_generator_v2.py',W/'scripts/astra_gazebo/scene_generator.py',W/'scripts/astra_gazebo/route_world_adapter.py',D/'diagnostic_blocks.csv',D/'generation_report.json',D/'layouts.json',S/'contracts/splits.yaml',W/'src/simulation/m2_gazebo/src/astra_evidence_plugin.cpp',W/'src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp',W/'src/simulation/m2_gazebo/include/m2_gazebo/v3_reactive_actor.h',W/'src/simulation/m2_gazebo/urdf/m2_sensors.xacro']
    result={'status':'offline_v2_geometry_qualification_pass_live_integration_pending' if all(checks.values()) else 'offline_v2_geometry_qualification_failed','checks':checks,'ros_started':False,'execution_ready':False,'final_seeds_accessed':False,'layout_count':len(lr),'episode_count':len(er),'actor_count':len(actors),'unreactive_conservative_conflict_actor_count':sum(a['conservative_conflict_proxy_present'] for a in actors),'new_allocated_seed_count':len(new_seeds),'seed_allocation_collision_count':len(gen['seed_allocation_collision_log']),'layouts':lr,'episodes':er,'environment_version':gen['environment_version'],'scope':['Full compound chassis/wheel footprint static path existence with Ackermann dynamics and .33m physical margin plus .03m sampling envelope.','720 planar scan rays at two static sensor poses and actor initial pose; geometric visibility only.','Actor full straight swept .72m square separation from static objects; all actual reaction trajectories still require live verification.','Input generation/rejection depended only on geometry, split allocation and nominal timing; no method outcomes.'],'pending_live_gates':['New physics source stamp/contact fixture and source-stamped bag/image association.','Wheel/body support at spawn z0; only a horizontal witness exists.','Gate-relative actor delay is not wired: inherited live plugin still uses absolute start_delay5.0 and world_x activation -4.02.','Named sensor/actuator RNG are not wired; response_seed multiplier is zero in frozen actor header. Parameter diversity is real, response stochasticity is not.','World-route SE2 adapter exists as pure tested functions; live planner/map/actor/evaluator bridge is pending. All these layouts use yaw0.','720beam visibility under measured ego/actor trajectories and reactive timing; static reveal is not temporal emergence qualification.','Static witness is not planner success or22s completion proof.','Complete contracted24layouts/family×3seeds development; all other required topology variants and stress/OOD pending.'],'dependencies':[{'path':str(p),'sha256':sha(p)} for p in deps],'environment':{'python':sys.version,'shapely':shapely.__version__}}
    out=S/'audits/scene_v2_execution_readiness.json';out.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print(json.dumps({k:result[k] for k in ['status','checks','actor_count','unreactive_conservative_conflict_actor_count','new_allocated_seed_count']},indent=2))

if __name__=='__main__':main()
