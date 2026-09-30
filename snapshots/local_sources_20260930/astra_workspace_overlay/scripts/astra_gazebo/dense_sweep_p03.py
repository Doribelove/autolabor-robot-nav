#!/usr/bin/env python3
"""P03 engineering sweep for a closed, authorized Gazebo run; never a ROS node.

Runtime pre/End packets, actual ODE flags/COM/collision geometry and authenticated
run-scope audit files replace library-name assumptions. The output is expressly
about the declared Gazebo normal-Euler extension and bounded actor Begin jumps.
It is not a continuous physical trajectory or a formal safety certificate.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import re

import dense_sweep as core
from geometry import Pose3, compound_clearance, project_box, project_cylinder
from evaluate_episode import WorldCollisions

PROFILE = {
    'name': 'astra_p03_engineering_numeric_v1',
    'world_coordinate_abs_max_m': 1000., 'nonrobot_shape_origin_radius_max_m': 1000., 'com_radius_max_m': 10., 'shape_radius_max_m': 10.,
    'quaternion_norm_residual_max': 1e-8, 'rotation_per_step_max_rad': .25,
    'com_translation_per_step_max_m': .25, 'point_budget_pad_per_step_m': 1e-5,
    'endpoint_clearance_pad_m': 1e-5, 'phase_join_pose_pad_m': 1e-7,
    'phase_join_rotation_pad_rad': 1e-7, 'physics_step_ns': 2000000,
    'coarse_stride_steps': 10, 'margin_m': .33,
    'interpretation': 'conservative engineering envelope tested against high-precision and independent references in this bounded domain; not a formal floating-point proof'
}
SCOPE = 'gazebo_normal_euler_extension_with_actor_begin_jumps'
ROBOT = 'autolabor_m2'


def sha(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(4*1024*1024),b''):digest.update(block)
    return digest.hexdigest()


def authenticate_closure(path, start_ns, end_ns):
    """Authenticate run artifacts and the bounded engineering audit scope.

    This trusts the bound independent audit's facts, rather than pretending to
    detect malicious callers. Audit scope is the authorized launch/nodes/service
    paths under the isolated loopback master, as explicitly agreed for P03.
    """
    value = json.loads(Path(path).read_text()); issues = []; bound = []
    if value.get('schema') != 'astra_p03_run_closure_v1' or value.get('scope') != SCOPE:
        issues.append('unknown authorized-run closure schema or scope')
    if value.get('numeric_profile') != PROFILE:
        issues.append('numeric profile was changed or omitted')
    roles = ('launch', 'world', 'robot_description', 'driver_source', 'driver_binary',
             'evidence_source', 'evidence_binary', 'engine_binary_maps', 'node_roster_audit', 'service_call_audit', 'pose_writer_review')
    audits = {}
    for role in roles+('actor_sources', 'actor_binaries'):
        items = value.get('bindings', {}).get(role)
        if not isinstance(items, list) or (role in roles and not items):
            issues.append('missing closure binding role: '+role);continue
        for item in items:
            try:
                file = Path(item['path'])
                if not isinstance(item['sha256'], str) or not re.fullmatch('[0-9a-f]{64}', item['sha256']) or sha(file) != item['sha256']:
                    raise ValueError('hash mismatch')
                bound.append({'role': role, 'path': str(file.resolve()), 'sha256': item['sha256']})
                if role.endswith('_audit') or role in ('pose_writer_review', 'engine_binary_maps'):
                    audits.setdefault(role, []).append(json.loads(file.read_text()))
            except (OSError, KeyError, ValueError, TypeError) as error:
                issues.append(role+': '+str(error))
    for role in ('node_roster_audit', 'service_call_audit', 'pose_writer_review', 'engine_binary_maps'):
        records = audits.get(role, [])
        if len(records) != 1:
            issues.append('exactly one review document required: '+role);continue
        item = records[0]
        if item.get('pass') is not True or item.get('run_id') != value.get('run_id'):
            issues.append('audit failed or run identity differs: '+role)
        if role != 'pose_writer_review' and (item.get('start_ns', end_ns+1) > start_ns or item.get('end_ns', start_ns-1) < end_ns):
            issues.append('audit does not cover evaluation interval: '+role)
    def audit(role):
        records = audits.get(role, [])
        return records[0] if len(records) == 1 else {}
    roster = audit('node_roster_audit')
    if roster.get('loopback_master_isolated') is not True or roster.get('unexpected_nodes') != []:
        issues.append('loopback master/node roster audit incomplete')
    services = audit('service_call_audit')
    if services.get('forbidden_calls') != [] or services.get('scope') != 'authorized_launch_and_nodes':
        issues.append('authorized service-path audit incomplete or forbidden pose service used')
    writers = audit('pose_writer_review')
    if writers.get('robot_pose_velocity_writes') != [] or writers.get('actor_updates') not in ('none', 'reviewed_bounded_Begin_updates'):
        issues.append('pose writers not closed for declared scope')
    engines = audit('engine_binary_maps')
    if not isinstance(engines.get('loaded_modules'), list) or not engines.get('loaded_modules'):
        issues.append('loaded engine binary maps absent')
    else:
        for module in engines['loaded_modules']:
            try:
                if sha(module['path']) != module['sha256']:
                    raise ValueError('loaded binary no longer matches archived binding')
            except (OSError, ValueError, TypeError, KeyError) as error:
                issues.append('loaded module: '+str(error))
    loaded = {(str(Path(m['path']).resolve()), m['sha256']) for m in engines.get('loaded_modules', [])
              if isinstance(m, dict) and 'path' in m and 'sha256' in m}
    for entry in bound:
        if entry['role'] in ('driver_binary', 'actor_binaries', 'evidence_binary') and (entry['path'], entry['sha256']) not in loaded:
            issues.append('bound control/actor binary missing from process maps: '+entry['role'])
    # Roles are supplied by an independent source/symbol/process-map review, not
    # inferred from a library filename. Require the actual implementation review.
    engine_roles=set()
    for module in engines.get('loaded_modules', []):
        engine_roles.update(module.get('roles', []))
    if not {'ode_integrator','gazebo_physics'}.issubset(engine_roles):
        issues.append('actual loaded ODE/Gazebo implementation roles not bound')
    implementation = engines.get('implementation_review', {})
    if (implementation.get('ode_normal_euler_update_verified') is not True or
        implementation.get('gazebo_pre_end_callback_order_verified') is not True or
        implementation.get('loaded_module_source_correspondence_reviewed') is not True):
        issues.append('actual engine implementation correspondence review incomplete')
    source_records=implementation.get('source_bindings', [])
    if not source_records:issues.append('reviewed engine primary-source bindings absent')
    for item in source_records:
        try:
            if sha(item['path'])!=item['sha256']:raise ValueError('engine primary source hash mismatch')
        except (OSError,ValueError,KeyError,TypeError) as error:issues.append(str(error))
    return {'pass': not issues, 'issues': issues, 'run_id': value.get('run_id'), 'scope': SCOPE,
            'bindings': bound, 'pose_writer_review': writers, 'closure_file': {'path': str(Path(path).resolve()), 'sha256': sha(path)},
            'audit_limit': 'authorized run and isolated master; not adversarial process isolation proof'}


def engineering_pose(value):
    if isinstance(value, Pose3):
        value = tuple(value.xyz)+tuple(value.xyzw)
    value = core.vector(value, 7)
    if max(abs(x) for x in value[:3]) > PROFILE['world_coordinate_abs_max_m']:
        raise ValueError('pose exceeds engineering coordinate domain')
    if abs(core.norm(value[3:])-1.) > PROFILE['quaternion_norm_residual_max']:
        raise ValueError('source quaternion exceeds engineering norm residual')
    return Pose3(value[:3], value[3:])


def rotation_delta(first, second):
    # Reuse robust small-angle handling, without a radius-dependent result.
    return core.extension_point_budget(first, second, (0,0,0), 0.)['rotation_arc_rad']


def point_budget(first, second, com, radius):
    first, second = engineering_pose(first), engineering_pose(second)
    if core.norm(core.vector(com)) > PROFILE['com_radius_max_m'] or not 0 <= core.finite(radius) <= PROFILE['shape_radius_max_m']:
        raise ValueError('COM/radius exceeds engineering domain')
    result = core.extension_point_budget(first, second, com, radius)
    if result['rotation_arc_rad'] > PROFILE['rotation_per_step_max_rad'] or result['translation_m'] > PROFILE['com_translation_per_step_max_m']:
        raise ValueError('per-step motion exceeds tested engineering domain')
    result['engineering_numeric_pad_m'] = PROFILE['point_budget_pad_per_step_m']
    result['engineering_point_budget_m'] = result['point_phase_budget_m']+PROFILE['point_budget_pad_per_step_m']
    return result


def runtime_geometry(metadata):
    if metadata.get('schema') != 'astra_physics_runtime_p03_v1' or metadata.get('valid_numeric') is not True:
        raise ValueError('runtime metadata absent or nonfinite')
    if metadata.get('epoch') != 0 or metadata.get('passive_observer') is not True or metadata.get('planner_input_authorized') is not False:
        raise ValueError('runtime metadata source scope/epoch invalid')
    config = metadata['config']
    if config.get('physics_type') != 'ode' or config.get('solver_type') != 'quick':
        raise ValueError('unsupported runtime solver')
    if round(core.finite(config['max_step_s'])*1e9) != PROFILE['physics_step_ns']:
        raise ValueError('runtime step differs from engineering profile')
    result = {}
    for model in config['models']:
        for link in model['links']:
            name = link['name']
            if not name.startswith(model['name']+'::'):raise ValueError('runtime link scope mismatch')
            if name in result:
                raise ValueError('duplicate runtime body identity')
            shapes = []
            com = core.vector(link['com_local_m']) if link.get('com_local_m') is not None else (0.,0.,0.)
            radius = 0.
            for collision in link.get('collisions', []):
                if not collision['name'].startswith(name+'::'):raise ValueError('runtime collision scope mismatch')
                rp = collision['relative_pose']; origin = engineering_pose(tuple(rp['xyz'])+tuple(rp['xyzw']))
                kind = collision['kind'];shape = {'name':collision['name'],'kind':kind,'origin':origin,'link':name}
                if kind == 'box':
                    dimensions = core.vector(collision['size_xyz_m'])
                    if min(dimensions) <= 0:raise ValueError('nonpositive runtime box')
                    extent = core.norm(tuple(x/2 for x in dimensions))
                elif kind == 'cylinder':
                    dimensions = (core.finite(collision['radius_m']),core.finite(collision['length_m']))
                    if min(dimensions) <= 0:raise ValueError('nonpositive runtime cylinder')
                    extent = math.hypot(dimensions[0]/math.cos(math.pi/48),dimensions[1]/2)
                else:
                    dimensions = ();extent = 0.
                shape['dimensions']=dimensions;shapes.append(shape)
                radius = max(radius,core.norm(tuple(a-b for a,b in zip(origin.xyz,com)))+extent)
            if name.startswith(ROBOT+'::'):
                if link.get('has_ode_body') is not True or link.get('finite_rotation_mode') != 0 or link.get('kinematic') is not False:
                    raise ValueError('robot runtime body is not normal dynamic ODE: '+name)
                if abs(core.finite(link['native_ode_mass_kg'])-core.finite(link['mass_kg']))>1e-7 or core.norm(core.vector(link['native_ode_mass_center']))>1e-7:
                    raise ValueError('Gazebo/ODE mass correspondence failed')
                if core.finite(link['mass_kg']) <= 0 or not shapes or any(s['kind'] not in ('box','cylinder') for s in shapes):
                    raise ValueError('robot runtime mass/collisions unsupported: '+name)
                if core.norm(com)>PROFILE['com_radius_max_m'] or radius>PROFILE['shape_radius_max_m']:
                    raise ValueError('runtime geometry exceeds engineering domain')
            if radius>PROFILE['nonrobot_shape_origin_radius_max_m']:raise ValueError('shape exceeds numerical engineering domain')
            result[name]={'model':model['name'],'static':model['static'],'raw':link,'com':com,'radius':radius,'origin_radius':max((core.norm(s['origin'].xyz)+(core.norm(tuple(d/2 for d in s['dimensions'])) if s['kind']=='box' else math.hypot(s['dimensions'][0]/math.cos(math.pi/48),s['dimensions'][1]/2) if s['kind']=='cylinder' else 0.) for s in shapes),default=0.),'shapes':shapes}
    return result


def projections(geometry, states, robot, ground_names):
    result=[]
    for name,body in geometry.items():
        if name.startswith(ROBOT+'::') != robot:
            continue
        if name not in states:raise ValueError('missing runtime collision link '+name)
        for shape in body['shapes']:
            if shape['name'] in ground_names:continue
            transform=states[name]['pose'].compose(shape['origin'])
            if shape['kind']=='box':
                value=project_box(shape['name'],shape['dimensions'],transform,name,'P03_actual_runtime_collision_and_source_pose')
            elif shape['kind']=='cylinder':
                value=project_cylinder(shape['name'],*shape['dimensions'],pose=transform,link_name=name,pose_source='P03_actual_runtime_collision_and_source_pose')
            else:raise ValueError('unsupported non-ground runtime shape '+shape['name'])
            result.append(value)
    return tuple(result)


def assess_records(pre_model_rows,pre_link_rows,end_model_rows,end_link_rows,statuses,metadata_records,
                   world_xml,start_ns,end_ns,closure_audit):
    issues=[];world=WorldCollisions(world_xml);issues.extend(world.errors)
    if type(start_ns) is not int or type(end_ns) is not int or start_ns <= 0 or end_ns <= start_ns:
        raise ValueError('positive increasing integer source window required')
    world_hash=hashlib.sha256(world_xml.encode()).hexdigest()
    if not any(b.get('role')=='world' and b.get('sha256')==world_hash for b in closure_audit.get('bindings', [])):
        issues.append('saved world bytes not authenticated in run closure')
    for model, xml in world.models.items():
        if model not in world.actor_speed_bounds and xml.findtext('static','false').lower()!='true':
            issues.append('unbounded dynamic nonrobot model: '+model)
    # Validate source quaternions BEFORE core.Pose3 normalizes them. Restrict the
    # roster/window checks to the requested post-bootstrap source interval.
    def selected(rows, label):
        result=[]
        for row in rows:
            try:
                stamp=row['stamp_ns']
                if type(stamp) is not int:raise ValueError('integer source stamp required')
                if not start_ns<=stamp<=end_ns:continue
                for p in row['poses']:engineering_pose(p)
                result.append(row)
            except (ValueError,KeyError,TypeError) as error:
                issues.append(label+': '+str(error))
        return result
    pre_model_rows=selected(pre_model_rows,'pre_models');pre_link_rows=selected(pre_link_rows,'pre_links')
    end_model_rows=selected(end_model_rows,'end_models');end_link_rows=selected(end_link_rows,'end_links')
    streams=[core.index_stream(rows,label,issues) for rows,label in (
        (pre_model_rows,'pre_models'),(pre_link_rows,'pre_links'),(end_model_rows,'end_models'),(end_link_rows,'end_links'))]
    pm,pl,em,el=streams;stamps=set(em)
    if any(set(stream)!=stamps for stream in streams):issues.append('P03 phase source sets differ')
    stamp_list=sorted(s for s in stamps if start_ns<=s<=end_ns)
    if not stamp_list or stamp_list[0]>start_ns or stamp_list[-1]<end_ns:issues.append('P03 source states do not cover exact requested interval endpoints')
    statuses_by_ns={};metadata_by_rev={}
    last_status=None
    for item in statuses:
        stamp=item.get('stamp_ns')
        if type(stamp) is not int or stamp<=0:
            issues.append('invalid status source stamp');continue
        if not start_ns<=stamp<=end_ns:continue
        if last_status is not None and stamp<=last_status:issues.append('status source reset/out-of-order')
        last_status=stamp
        if stamp in statuses_by_ns:issues.append('duplicate P03 source status')
        statuses_by_ns[stamp]=item
    for item in metadata_records:
        revision=item.get('revision')
        if type(revision) is not int or revision<=0:
            issues.append('invalid metadata revision');continue
        if revision in metadata_by_rev and metadata_by_rev[revision]!=item:issues.append('runtime metadata revision changed')
        metadata_by_rev[revision]=item
    if closure_audit.get('pass') is not True:issues.append('authorized run closure did not pass authentication')
    if set(statuses_by_ns)!=stamps:issues.append('status/source state coverage differs')
    expected_models=set(world.models)|{ROBOT};actors=world.actor_speed_bounds
    grounds={g['name'] for g in world.grounds}
    if actors:
        roles={b.get('role') for b in closure_audit.get('bindings',[])}
        if not {'actor_sources','actor_binaries'}.issubset(roles) or closure_audit.get('pose_writer_review',{}).get('actor_updates')!='reviewed_bounded_Begin_updates':
            issues.append('actor source/binary/Begin writer closure missing')
    interval_rows=[];endpoint_rows=[];metadata_cache={};prior_stamp=None;active_revision=None;prior_step=None
    for sample_index,stamp in enumerate(stamp_list):
        try:
            if any(stamp not in stream for stream in streams):raise ValueError('missing exact phase packet')
            if set(pm[stamp])!=expected_models or set(em[stamp])!=expected_models:raise ValueError('runtime model roster differs from saved world')
            status=statuses_by_ns.get(stamp,{})
            if status.get('schema')!='astra_physics_step_p03_v1' or status.get('valid') is not True or status.get('epoch')!=0:
                raise ValueError('invalid/missing P03 per-step source status')
            step=status.get('step_index')
            if type(step) is not int or step<0 or (prior_step is not None and step!=prior_step+1):
                raise ValueError('step index absent/reset/missing')
            prior_step=step
            if status.get('pre_end_same_source_stamp') is not True or status.get('same_roster') is not True:
                raise ValueError('pre/End source pairing or roster failed')
            revision=status['end_revision']
            if status.get('pre_revision')!=revision or status.get('config_stable_within_step') is not True:
                raise ValueError('runtime configuration changed inside solver step')
            if active_revision is not None and active_revision!=revision:raise ValueError('runtime configuration changed during evaluated interval')
            active_revision=revision
            metadata=metadata_by_rev[revision]
            if type(metadata.get('stamp_ns')) is not int or metadata['stamp_ns']>stamp or metadata.get('phase') not in ('BeforePhysicsUpdate','WorldUpdateEnd'):
                raise ValueError('metadata source timestamp or phase invalid')
            if revision not in metadata_cache:metadata_cache[revision]=runtime_geometry(metadata)
            geometry=metadata_cache[revision]
            actual={shape['name']:shape for body in geometry.values() if body['model']!=ROBOT for shape in body['shapes']}
            expected={p['name'] for p in world.primitives}|grounds
            if set(actual)!=expected:raise ValueError('runtime nonrobot collision identities differ from saved world')
            for primitive in world.primitives:
                shape=actual[primitive['name']]
                if shape['kind']!=primitive['kind'] or len(shape['dimensions'])!=len(primitive['dimensions']) or any(abs(a-b)>1e-9 for a,b in zip(shape['dimensions'],primitive['dimensions'])):
                    raise ValueError('runtime nonrobot dimensions differ from saved world')
                origin=primitive['origin']
                if core.norm(tuple(a-b for a,b in zip(shape['origin'].xyz,origin.xyz)))>1e-8 or rotation_delta(shape['origin'],origin)>1e-8:
                    raise ValueError('runtime nonrobot collision origin differs from saved world')
            if set(pl[stamp])!=set(geometry) or set(el[stamp])!=set(geometry):raise ValueError('runtime body roster differs from metadata')
            for stream in (pl[stamp],el[stamp],pm[stamp],em[stamp]):
                for state in stream.values():state['pose']=engineering_pose(state['pose'])
            native={label:{x['name']:x for x in status[label]} for label in ('pre_enabled','end_enabled')}
            enabled={label:{n:x['enabled'] for n,x in group.items()} for label,group in native.items()}
            robot_budget=0.;robot_solver_budget=0.;robot_join_budget=0.;actor_budget=0.;static_budget=0.;phase_jump_max=0.;bodies={}
            for name,body in geometry.items():
                if not name.startswith(ROBOT+'::'):continue
                if enabled['pre_enabled'].get(name) is not True or enabled['end_enabled'].get(name) is not True:
                    raise ValueError('robot body disabled within evaluated interval')
                for label,poses in (('pre_enabled',pl[stamp]),('end_enabled',el[stamp])):
                    native_pose=engineering_pose(tuple(native[label][name]['native_com_world_m'])+tuple(native[label][name]['native_quaternion_xyzw']))
                    expected_pose=poses[name]['pose'].compose(Pose3(body['com']))
                    if core.norm(tuple(a-b for a,b in zip(native_pose.xyz,expected_pose.xyz)))>PROFILE['phase_join_pose_pad_m'] or rotation_delta(native_pose,expected_pose)>PROFILE['phase_join_rotation_pad_rad']:
                        raise ValueError('native ODE COM/quaternion differs from published link pose: '+name)
                budget=point_budget(pl[stamp][name]['pose'],el[stamp][name]['pose'],body['com'],body['radius'])
                join_budget=0.
                if prior_stamp is not None:
                    old,new=el[prior_stamp][name]['pose'],pl[stamp][name]['pose']
                    jump=core.norm(tuple(b-a for a,b in zip(old.xyz,new.xyz)));phase_jump_max=max(phase_jump_max,jump)
                    if jump>PROFILE['phase_join_pose_pad_m'] or rotation_delta(old,new)>PROFILE['phase_join_rotation_pad_rad']:
                        raise ValueError('unclassified robot pose write between End and Pre: '+name)
                    join_budget=jump+body['origin_radius']*rotation_delta(old,new)+PROFILE['point_budget_pad_per_step_m']
                budget['End_to_Pre_engineering_variation_m']=join_budget
                robot_solver_budget=max(robot_solver_budget,budget['engineering_point_budget_m'])
                robot_join_budget=max(robot_join_budget,join_budget);bodies[name]=budget
            robot_budget=robot_solver_budget+robot_join_budget
            for name in expected_models-{ROBOT}:
                before,after=pm[stamp][name]['pose'],em[stamp][name]['pose']
                if core.norm(tuple(b-a for a,b in zip(before.xyz,after.xyz)))>PROFILE['phase_join_pose_pad_m'] or rotation_delta(before,after)>PROFILE['phase_join_rotation_pad_rad']:
                    raise ValueError('nonrobot model moved during solver interval: '+name)
                if prior_stamp is not None:
                    old=em[prior_stamp][name]['pose'];jump=core.norm(tuple(b-a for a,b in zip(old.xyz,before.xyz)))
                    if name in actors:
                        cap=actors[name]*PROFILE['physics_step_ns']/1e9
                        if jump>cap+PROFILE['phase_join_pose_pad_m'] or rotation_delta(old,before)>PROFILE['phase_join_rotation_pad_rad']:
                            raise ValueError('actor Begin update exceeds reviewed source rule')
                        actor_budget+=cap+PROFILE['point_budget_pad_per_step_m']
                    elif jump>PROFILE['phase_join_pose_pad_m'] or rotation_delta(old,before)>PROFILE['phase_join_rotation_pad_rad']:
                        raise ValueError('unclassified static model Begin pose write: '+name)
            # A model pose alone cannot constrain an independently moving child
            # link. Check all nonrobot links as well, including hidden articulations.
            nonrobot_phase={}
            for name,body in geometry.items():
                model=body['model']
                if model==ROBOT:continue
                before,after=pl[stamp][name]['pose'],el[stamp][name]['pose']
                delta=core.norm(tuple(b-a for a,b in zip(before.xyz,after.xyz)));angle=rotation_delta(before,after)
                if delta>PROFILE['phase_join_pose_pad_m'] or angle>PROFILE['phase_join_rotation_pad_rad']:
                    raise ValueError('nonrobot link moved during solver interval: '+name)
                phase=delta+body['origin_radius']*angle;join_phase=0.
                if prior_stamp is not None:
                    old=el[prior_stamp][name]['pose'];delta=core.norm(tuple(b-a for a,b in zip(old.xyz,before.xyz)));angle=rotation_delta(old,before)
                    cap=actors.get(model,0.)*PROFILE['physics_step_ns']/1e9
                    if delta>cap+PROFILE['phase_join_pose_pad_m'] or angle>PROFILE['phase_join_rotation_pad_rad']:
                        raise ValueError('nonrobot link Begin update exceeds model rule: '+name)
                    # The actor cap is already paid above. Pay tolerance/rotation
                    # and during-step drift explicitly, including static shapes.
                    join_phase=max(0.,delta-cap)+body['origin_radius']*angle
                old_phase,old_join=nonrobot_phase.get(model,(0.,0.))
                nonrobot_phase[model]=(max(old_phase,phase),max(old_join,join_phase))
            static_budget=sum(a+b+PROFILE['point_budget_pad_per_step_m'] for a,b in nonrobot_phase.values())
            if sample_index%PROFILE['coarse_stride_steps']==0 or sample_index==len(stamp_list)-1:
                robot_shapes=projections(geometry,el[stamp],True,grounds);other_shapes=projections(geometry,el[stamp],False,grounds)
                clearance=compound_clearance(robot_shapes,other_shapes,PROFILE['margin_m'])
                endpoint_rows.append({'stamp_ns':stamp,'physical_xy_m':clearance['minimum_physical_projected_clearance_m'],
                    'conservative_xy_m':clearance['minimum_conservative_projected_clearance_m'],
                    'z_screened_physical_xy_m':clearance['minimum_z_conditioned_projected_clearance_m'],
                    'potential_3d_intersection_count':len(clearance['potential_3d_intersection_pairs']),
                    'robot_components':len(robot_shapes),'obstacle_components':len(other_shapes)})
            if prior_stamp is not None:
                if stamp-prior_stamp!=PROFILE['physics_step_ns']:raise ValueError('missing solver step')
                interval_rows.append({'start_ns':prior_stamp,'end_ns':stamp,'robot_point_budget_m':robot_budget,
                    'actor_Begin_variation_budget_m':actor_budget,'other_phase_variation_budget_m':static_budget,
                    'relative_clearance_variation_budget_m':robot_budget+actor_budget+static_budget,
                    'maximum_End_to_Pre_robot_position_jump_m':phase_jump_max,'bodies':bodies})
        except (ValueError,KeyError,TypeError) as error:
            issues.append('source step '+str(stamp)+': '+str(error))
        prior_stamp=stamp
    coarse=[]
    if endpoint_rows and all(row['conservative_xy_m'] is not None for row in endpoint_rows):
        chosen=endpoint_rows
        endpoints=[{'stamp_ns':r['stamp_ns'],'clearance_m':r['conservative_xy_m']-PROFILE['endpoint_clearance_pad_m']} for r in chosen]
        coarse=core.aggregate_clearance_bounds(endpoints,interval_rows)
        if any(not r['covered_exactly'] for r in coarse):issues.append('coarse geometry interval has missing solver coverage')
    valid_bounds=[r['conditional_clearance_lower_bound_m'] for r in coarse if r['conditional_clearance_lower_bound_m'] is not None]
    minimum=min(valid_bounds) if valid_bounds else None
    integrity=not issues and bool(interval_rows)
    return {'schema':'astra_p03_engineering_sweep_v1','scope':SCOPE,'numeric_profile':PROFILE,'planner_input_authorized':False,
            'source_window_ns':[start_ns,end_ns],'integrity_pass':integrity,'issues':sorted(set(issues)),
            'qualification':'qualified_within_declared_Gazebo_engineering_scope' if integrity else 'pending_or_invalid_evidence',
            'minimum_engineering_conservative_swept_xy_m':minimum,
            'clearance_status':('not_applicable_no_nonground_obstacle' if minimum is None else
                ('pass_within_model_scope' if integrity and minimum>=0 else 'fail_or_unavailable')),
            'dense_solver_intervals':len(interval_rows),'coarse_intervals':coarse,'closure_audit':closure_audit,
            'limits':['Numerical pads are a tested conservative engineering envelope, not a formal floating-point proof.',
                      'This is the specified Gazebo algorithmic extension plus bounded actor Begin jumps; not real continuous physical safety.',
                      'XY and Z screening do not prove exact3D contact; combine with whole-system contact and task/input evidence.',
                      'Qualified scope covers authorized run nodes/service paths, not malicious external process isolation.'],
            'formal_safety':False,'real_continuous_physical_safety':False},interval_rows,endpoint_rows


def read_p03_bag(path):
    """Read only; no ROS node/master. Preserve original per-topic source order."""
    import rosbag
    keys=('pre_models','pre_links','end_models','end_links')
    rows={key:[] for key in keys};metadata=[];statuses=[]
    topics=['/astra/p03/'+k for k in keys+('runtime_metadata','step_status')]
    with rosbag.Bag(str(path)) as bag:
        for topic,msg,_receipt in bag.read_messages(topics=topics):
            key=topic.rsplit('/',1)[1]
            if key in ('runtime_metadata','step_status'):
                value=json.loads(msg.data)
                if not isinstance(value,dict):raise ValueError('P03 JSON object required')
                (metadata if key=='runtime_metadata' else statuses).append(value);continue
            rows[key].append({'stamp_ns':int(msg.header.stamp.to_nsec()),'frame_id':msg.header.frame_id,
                'names':list(msg.name),'poses':[(p.position.x,p.position.y,p.position.z,
                    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w) for p in msg.pose],
                'twists':[(t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z) for t in msg.twist]})
    return dict(pre_model_rows=rows['pre_models'],pre_link_rows=rows['pre_links'],
                end_model_rows=rows['end_models'],end_link_rows=rows['end_links'],
                statuses=statuses,metadata_records=metadata)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('bag','world','urdf','closure','output'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--start-ns',type=int,required=True);parser.add_argument('--end-ns',type=int,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError('output already exists; preserve previous evidence')
    sources=[args.bag,args.world,args.urdf,args.closure,Path(__file__),Path(core.__file__),
             Path(__file__).with_name('geometry.py'),Path(__file__).with_name('evaluate_episode.py')]
    binding=lambda p:{'path':str(p.resolve()),'sha256':sha(p)}
    before=[binding(p) for p in sources]
    closure=authenticate_closure(args.closure,args.start_ns,args.end_ns)
    for role,path in (('world',args.world),('robot_description',args.urdf)):
        if not any(b['role']==role and b['sha256']==sha(path) for b in closure['bindings']):
            closure['issues'].append('CLI '+role+' input not bound to run')
    closure['pass']=not closure['issues']
    records=read_p03_bag(args.bag)
    nominal=core.robot_body_envelopes(args.urdf.read_text())
    used={s.get('end_revision') for s in records['statuses'] if args.start_ns<=s.get('stamp_ns',-1)<=args.end_ns}
    for metadata in records['metadata_records']:
        if metadata.get('revision') not in used:continue
        try:
            actual=runtime_geometry(metadata)
            robot={n.split('::',1)[1]:body for n,body in actual.items() if body['model']==ROBOT}
            if set(robot)!=set(nominal):raise ValueError('runtime/URDF merged body identities differ')
            for name,body in robot.items():
                expect=nominal[name]
                if abs(body['raw']['mass_kg']-expect['mass_kg'])>1e-7 or core.norm(tuple(a-b for a,b in zip(body['com'],expect['com_local_m'])))>1e-7:
                    raise ValueError('runtime/URDF merged mass or COM differs: '+name)
                if abs(body['radius']-expect['radius_m'])>1e-7 or len(body['shapes'])!=len(expect['collisions']):
                    raise ValueError('runtime/URDF compound radius or count differs: '+name)
        except (ValueError,KeyError,TypeError) as error:closure['issues'].append(str(error))
    closure['pass']=not closure['issues']
    report,intervals,endpoints=assess_records(**records,world_xml=args.world.read_text(),
        start_ns=args.start_ns,end_ns=args.end_ns,closure_audit=closure)
    # Authenticate again after parsing, including every externally bound artifact.
    after_closure=authenticate_closure(args.closure,args.start_ns,args.end_ns)
    if before!=[binding(p) for p in sources] or after_closure['bindings']!=closure['bindings'] or not after_closure['pass']:
        report['issues'].append('input changed during evaluation or closure authentication failed')
        report['integrity_pass']=False;report['qualification']='pending_or_invalid_evidence'
        if report['clearance_status']=='pass_within_model_scope':report['clearance_status']='fail_or_unavailable'
    report['source_bindings']=before
    report['geometry_source']='actual runtime merged collision geometry; saved expanded URDF hash is provenance/QC, not substituted for native geometry'
    args.output.mkdir(parents=True)
    with (args.output/'intervals.jsonl').open('w') as out:
        for row in intervals:out.write(json.dumps(row,allow_nan=False)+'\n')
    with (args.output/'endpoints.csv').open('w',newline='') as out:
        if endpoints:
            writer=csv.DictWriter(out,fieldnames=list(endpoints[0]));writer.writeheader();writer.writerows(endpoints)
    report['outputs']=[binding(args.output/p) for p in ('intervals.jsonl','endpoints.csv')]
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:report[k] for k in ('integrity_pass','qualification','clearance_status','dense_solver_intervals')}))
    return 0 if report['integrity_pass'] else 3


if __name__=='__main__':
    raise SystemExit(main())
