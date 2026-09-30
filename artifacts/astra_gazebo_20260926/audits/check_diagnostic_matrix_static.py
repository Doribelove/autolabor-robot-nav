#!/usr/bin/env python3
"""ROS-free runner/monitor component audit. Never calls execute or main."""
import ast,copy,csv,hashlib,json,math,tempfile,types
from collections import deque
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
S=Path(__file__).resolve().parents[1];W=S/'workspace'
R=W/'scripts/astra_gazebo/run_diagnostic_matrix.py'
M=W/'src/simulation/m2_gazebo/scripts/astra_evidence_monitor_a03.py'
L=W/'src/simulation/m2_gazebo/launch/astra_diagnostic_episode.launch'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    checks={};rt=ast.parse(R.read_text());mt=ast.parse(M.read_text());launch=ET.parse(L).getroot();checks['three_sources_parse']=True
    fn=next(n for n in rt.body if isinstance(n,ast.FunctionDef) and n.name=='load_scene');ns={'Path':Path,'sha':sha,'yaml':yaml,'ET':ET}
    exec(compile(ast.Module(body=[fn],type_ignores=[]),str(R),'exec'),ns)
    data=S/'scenes'/('development_a02_live_v2' if 'astra_common_environment_v4_' in R.read_text() else 'development_a02_live_v1')
    rows=list(csv.DictReader((data/'diagnostic_blocks.csv').read_text().splitlines()))
    for row in rows:row['seed']=int(row['seed']);ns['load_scene'](row)
    checks['all_twelve_bound_inputs_load']=len(rows)==12
    rejected=[]
    for key in ('seed','layout_id','scene_id','environment_version','input_variant_id','parent_geometry_id','geometry_sha256'):
        row=copy.deepcopy(rows[0]);row[key]=row[key]+1 if key=='seed' else row[key]+'_wrong'
        try:ns['load_scene'](row)
        except Exception:rejected.append(key)
    checks['seven_schedule_identity_mutations_rejected']=len(rejected)==7
    actor_row=next(r for r in rows if yaml.safe_load(Path(r['manifest_path']).read_text())['actors'])
    with tempfile.TemporaryDirectory(prefix='astra_readonly_d01_fixture_') as td:
        for kind in ('missing_physics_plugin','legacy_actor_plugin','legacy_gate_field','mismatched_actor_delay'):
            row=dict(actor_row);root=ET.parse(row['world_path']).getroot();world=root.find('world');actor=next(m for m in world.findall('model') if m.get('name','').startswith('benchmark_actor_'));plugin=actor.find('plugin')
            if kind=='missing_physics_plugin':world.remove(world.find("plugin[@filename='libastra_evidence_plugin.so']"))
            elif kind=='legacy_actor_plugin':plugin.set('filename','libv3_reactive_actor_plugin.so')
            elif kind=='legacy_gate_field':ET.SubElement(plugin,'start_delay').text='5'
            else:plugin.find('gate_delay').text='2.0'
            p=Path(td)/(kind+'.sdf');p.write_text(ET.tostring(root,encoding='unicode'));row['world_path']=str(p);row['world_sha256']=sha(p)
            try:ns['load_scene'](row);checks[kind+'_rejected']=False
            except Exception:checks[kind+'_rejected']=True
    monitor=next(n for n in mt.body if isinstance(n,ast.ClassDef) and n.name=='Monitor')
    frame=next(n for n in monitor.body if isinstance(n,ast.FunctionDef) and n.name=='frame_context')
    fake_ros=types.SimpleNamespace(Time=types.SimpleNamespace(now=lambda:types.SimpleNamespace(to_sec=lambda:8.)))
    ctx={'rospy':fake_ros};exec(compile(ast.Module(body=[frame],type_ignores=[]),str(M),'exec'),ctx)
    self=types.SimpleNamespace(run_id='fixture',buffers={k:deque([(1.,{'sample':1}),(2.,{'sample':2}),(3.,{'sample':3})]) for k in ['models','links','scan','odom','joints']})
    selected=ctx['frame_context'](self,2.5)
    checks['monitor_uses_only_source_stamps_at_or_before_image']=all(selected[k]==2. for k in ['model_source_stamp_s','link_source_stamp_s','scan_stamp_s','odom_stamp_s','joint_stamp_s']) and selected['models']=={'sample':2}
    empty=ctx['frame_context'](self,.5);checks['monitor_no_prior_sample_explicit_missing']=empty['models']=={} and empty['odom_pose'] is None and empty['model_source_stamp_s']==0.
    image=next(n for n in monitor.body if isinstance(n,ast.FunctionDef) and n.name=='on_image')
    nodes=[]
    for node in ast.walk(image):
        if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='wheels' for t in node.targets):nodes.append(node)
        if isinstance(node,ast.For) and isinstance(node.iter,ast.Call) and isinstance(node.iter.func,ast.Attribute) and isinstance(node.iter.func.value,ast.Subscript) and ast.get_source_segment(M.read_text(),node.iter.func.value)=="row['links']":nodes.append(node)
    checks['rotated_wheel_support_formula']=len(nodes)==2
    for angle in (0.,.2,-.5,math.pi/2):
        env={'math':math,'row':{'links':{'autolabor_m2::left_front_wheel_link':{'z':.4,'qx':math.sin(angle/2),'qy':0.,'qz':0.,'qw':math.cos(angle/2)}}}}
        exec(compile(ast.Module(body=nodes,type_ignores=[]),str(M),'exec'),env)
        expected=.4-(.15*abs(math.cos(angle))+.04*abs(math.sin(angle)))
        checks['rotated_wheel_support_formula'] &= abs(next(iter(env['wheels'].values()))-expected)<1e-10
    recorder=launch.find("node[@name='astra_raw_recorder']").get('args').split()
    source_topics=['/astra/world_models','/astra/world_links','/astra/physics_contacts','/astra/physics_metadata']
    checks['all_source_evidence_topics_recorded']=all(t in recorder for t in source_topics)
    checks['driver_and_command_topics_recorded']=all(t in recorder for t in ['/cmd_vel_navigation','/cmd_vel','/m2_driver/wheel_angle','/m2_driver/applied_speed'])
    spawn=launch.find('include');checks['spawn_z0_and_yaw0']=spawn.find("arg[@name='z']").get('value')=='0.0' and spawn.find("arg[@name='yaw']").get('value')=='0.0'
    checks['passive_monitor_no_publishers']=not any(isinstance(n,ast.Attribute) and n.attr in ('Publisher','publish') for n in ast.walk(mt))
    checks['runner_main_guards_execute_flag']=bool(next(n for n in rt.body if isinstance(n,ast.FunctionDef) and n.name=='main')) and 'if not args.execute:' in R.read_text()
    r={'status':'diagnostic_runner_monitor_static_components_pass' if all(checks.values()) else 'diagnostic_static_check_failed','checks':checks,'ros_started':False,'execution_called':False,'input_set_checked':str(data),'input_rows_checked':len(rows),'source_bindings':[{'path':str(p),'sha256':sha(p)} for p in [R,L,M,Path(__file__)]],'scope':'Source/static components only; final selected contract, runtime actuation/evaluator, bag/source consistency and independent live outcome assessment remain root gates.','limits':['Frame QC is per selected frame and runner requires at least one machine-pass frame; not proof all navigation frames pass.','Model/link camera associations are independent at-or-before source samples, not exact simultaneous rendering.','states.jsonl is a convenience summary; full empty/nonempty contacts and precise temporal analysis require source-stamped raw bag.','Nonidentity route intentionally rejected.','Reserved unused future RNG streams are not blocking.']}
    (S/'audits/diagnostic_matrix_static_review.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
    if not all(checks.values()):raise SystemExit(1)

if __name__=='__main__':main()
