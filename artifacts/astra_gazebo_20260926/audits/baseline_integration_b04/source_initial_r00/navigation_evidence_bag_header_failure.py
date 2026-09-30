"""New navigation evidence contract and generic passive collection helpers.

No ROS or execution on import. Q02 remains a separate frozen two-fixture stage.
"""
import json
import hashlib
import math
from pathlib import Path
import re
import time
import xml.etree.ElementTree as ET
import yaml
import baseline_rear_evidence_b04 as rear

import passive_sweep_q05 as passive
from passive_sweep_q05 import (EvidenceStore,WallDeadline,sha,binding,read_bound,
    source_coverage,physical_status_audit,recorder_connections,proc_sample,elf_inventory,source_rtf)

WORKSPACE=passive.WORKSPACE
STAGE=passive.STAGE
SCHEMA='astra_navigation_evidence_b04_v1'
IMAGE_POLICY='supplementary_native_frames_do_not_suppress_physical_failure_evidence'
STEP_NS=2_000_000
METHODS=rear.METHODS
DENSE_TOPICS=passive.DENSE_TOPICS
PHYSICAL_TOPICS=DENSE_TOPICS+('/astra/p03/runtime_metadata','/astra/physics_metadata','/astra/world_models','/astra/world_links',
 '/clock','/odom','/joint_states','/m2_driver/physical_status','/m2_driver/physical_metadata','/m2_driver/command_ingress',
 '/m2_driver/actuator_target','/m2_driver/actuator_commanded_effort','/cmd_vel_navigation','/cmd_vel')
COORDINATOR_TOPICS=('/astra/navigation/identity','/astra/navigation/status','/astra/navigation/gate','/astra/navigation/command_trace')
ESSENTIAL_BEFORE_UNPAUSE=PHYSICAL_TOPICS+COORDINATOR_TOPICS
ESSENTIAL_BEFORE_ARM=ESSENTIAL_BEFORE_UNPAUSE+('/scan','/tf','/tf_static','/map','/astra/common_route/path','/astra/method_cmd','/astra/method/status')
ESSENTIAL_BEFORE_ARM=tuple(dict.fromkeys(ESSENTIAL_BEFORE_ARM+rear.REQUIRED_RECORD_TOPICS))
SERVICE_PHASES={'/gazebo/spawn_urdf_model':{'bootstrap'},'/gazebo/unpause_physics':{'bootstrap'},
 '/gazebo/pause_physics':{'terminal','failure'},'/gazebo/get_world_properties':{'bootstrap','warmup','terminal'},
 '/gazebo/get_physics_properties':{'bootstrap','terminal'},'/astra/navigation/arm':{'warmup'},
 '/astra/navigation/stop':{'terminal','failure'}}
TERMINAL_PHASES={'complete','timeout','fault','stopped'}
TERMINAL_AUTHORITIES={'complete':'first_actual_odom_header_meeting_base_goal',
 'timeout':'declared_gate_actual_odom_source_plus22s',
 'fault':'latest_actual_odom_header_at_fault_detection_not_fault_generation_stamp',
 'stopped':'latest_actual_odom_header_at_stop_request'}
CONTRACT_FIELDS={'reference_contract','schema','stage','run_id','seed','attempt','method','scene_id','world','robot_description','scene_manifest','image_policy',
 'shared_bundle','spawn_world_xyz_yaw','steering_kd','expected_models','allowed_nodes','required_nodes',
 'required_before_unpause','required_before_arm','record_topics','coordinator_node','relay_node','camera_topic',
 'activation_source_ns','policy_cap_ns','physics_step_ns','readiness_wall_s','wall_cap_s','clock_stall_wall_s','drain_wall_s',
 'maximum_robot_xy_radius_m','training_authorized','execution_authorized'}


def _identity(value,label):
    if type(value)is not str or not re.fullmatch(r'[a-z][a-z0-9_]{1,95}',value):raise ValueError('invalid '+label)


def _names(values,label):
    if type(values)is not list or not values or any(type(x)is not str or not re.fullmatch(r'/[A-Za-z][A-Za-z0-9_/]*',x) for x in values) or len(set(values))!=len(values):
        raise ValueError('unique absolute ROS names required: '+label)


def _vector(values,length,label):
    if type(values) not in (list,tuple) or len(values)!=length or any(type(x) not in (int,float) or not math.isfinite(x) for x in values):
        raise ValueError('finite vector required: '+label)
    return tuple(values)


class LifecycleFault(RuntimeError):
    """A permanent evidence fault that a readiness retry must not swallow."""


def sensor_method_graph(system):
    """Actual ROS master registration audit; not an adversarial process proof."""
    node='/astra_frozen_r2_method'
    inputs=sorted(name for name,nodes in system[1] if node in nodes)
    expected={'/clock','/scan','/odom','/tf','/tf_static','/map','/astra/common_route/path',
              '/astra/navigation/gate','/astra/navigation/status'}
    command_publishers=sorted(dict(system[0]).get('/astra/method_cmd',[]))
    outputs=sorted(name for name,nodes in system[0] if node in nodes)
    permitted_outputs={'/rosout','/astra/method_cmd','/astra/method/status'}|{
        '/astra/frozen_r2_sensor_lifecycle/'+name for name in
        ('identity','input_receipt','trace','command_event','report')}
    return dict(inputs=inputs,outputs=outputs,missing_inputs=sorted(expected-set(inputs)),
        unexpected_inputs=sorted(set(inputs)-expected),unexpected_outputs=sorted(set(outputs)-permitted_outputs),
        private_command_publishers=command_publishers,
        pass_=set(inputs)==expected and set(outputs)<=permitted_outputs and command_publishers==[node],
        scope='Observed trusted-launch ROS registrations; source review covers service clients and file inputs.')


def image_coverage(images,errors,camera_topic):
    stamps=[item['source_ns'] for item in images]
    return dict(policy=IMAGE_POLICY,requested=camera_topic is not None,
        saved_count=len(images),unique_source_frame_count=len(set(stamps)),errors=list(errors),
        two_distinct_frames_available=len(set(stamps))>=2 and not errors,
        capture_integrity_dependency=False,
        interpretation='Sparse or invalid images remain a visual coverage limitation; original task failures and complete physical records remain assessable.')


def readiness_guard(source_ns,status,value):
    if source_ns>=value['activation_source_ns']:raise LifecycleFault('common activation reached before arm readiness')
    if status and status.get('phase') in TERMINAL_PHASES:raise LifecycleFault('coordinator terminated before arm')
    return bool(status and status.get('ready_to_arm')is True)


def wait_condition(predicate,seconds,label,*,clock=time.monotonic,sleep=time.sleep,shutdown=lambda:False,guard=lambda:None):
    """Retry transient RPC readiness errors; exact wall deadline includes calls.

    A predicate that returns after the deadline cannot turn a timeout into pass.
    LifecycleFault is permanent; other errors remain visible in the timeout.
    """
    deadline=clock()+seconds;last_error=None
    while not shutdown():
        guard()
        remaining=deadline-clock()
        if remaining<=0:break
        try:
            result=predicate()
            if clock()>=deadline:break
            if result:return result
        except LifecycleFault:raise
        except Exception as error:last_error=repr(error)
        remaining=deadline-clock()
        if remaining<=0:break
        sleep(min(.02,remaining))
    raise TimeoutError(label+': '+str(last_error))


def make_contract(*,stage,run_id,seed,method,scene_id,world,robot_description,scene_manifest,shared_bundle,
                  spawn_world_xyz_yaw,steering_kd,allowed_nodes,required_nodes,record_topics=None,
                  required_before_unpause=None,required_before_arm=None,camera_topic='/astra_n01/camera/image_raw',
                  coordinator_node='/astra_navigation_coordinator',relay_node='/navigation_command_relay',wall_cap_s=300.):
    world=Path(world);models=[m.get('name') for m in ET.fromstring(world.read_text()).findall('world/model')]+['autolabor_m2']
    before=list(ESSENTIAL_BEFORE_UNPAUSE if required_before_unpause is None else required_before_unpause)
    arm=list(ESSENTIAL_BEFORE_ARM if required_before_arm is None else required_before_arm)
    topics=list(dict.fromkeys(arm) if record_topics is None else record_topics)
    value={'schema':SCHEMA,'reference_contract':json.loads(json.dumps(rear.REFERENCE_CONTRACT)),'stage':stage,'run_id':run_id,'seed':seed,'attempt':1,'method':method,'scene_id':scene_id,
      'world':binding(world),'robot_description':binding(robot_description),'scene_manifest':binding(scene_manifest),
      'shared_bundle':binding(Path(shared_bundle)/'common_route.json'),'spawn_world_xyz_yaw':list(spawn_world_xyz_yaw),
      'steering_kd':steering_kd,'expected_models':models,'allowed_nodes':list(allowed_nodes),'required_nodes':list(required_nodes),
      'required_before_unpause':before,'required_before_arm':arm,'record_topics':topics,
      'coordinator_node':coordinator_node,'relay_node':relay_node,'camera_topic':camera_topic,'image_policy':IMAGE_POLICY,
      'activation_source_ns':5_000_000_000,'policy_cap_ns':22_000_000_000,'physics_step_ns':2_000_000,
      'readiness_wall_s':90.,'wall_cap_s':float(wall_cap_s),'clock_stall_wall_s':5.,'drain_wall_s':5.,
      'maximum_robot_xy_radius_m':100.,'training_authorized':False,'execution_authorized':False}
    return validate_contract(value)


def validate_contract(value):
    if type(value)is not dict or set(value)!=CONTRACT_FIELDS:raise ValueError('closed navigation evidence contract required')
    if value['schema']!=SCHEMA:raise ValueError('navigation evidence schema differs')
    if json.dumps(value['reference_contract'],sort_keys=True,allow_nan=False)!=json.dumps(rear.REFERENCE_CONTRACT,sort_keys=True):raise ValueError('fixed B04 rear reference contract differs')
    rear.static_world_only(read_bound(value['world']).decode())
    if not set(rear.REQUIRED_RECORD_TOPICS)<=set(value['record_topics']) or not set(rear.REQUIRED_RECORD_TOPICS)<=set(value['required_before_arm']):raise ValueError('complete B04 rear recorder roster required')
    if not set(rear.REQUIRED_NODES)<=set(value['required_nodes']):raise ValueError('required B04 rear nodes absent')
    if value['image_policy']!=IMAGE_POLICY:raise ValueError('separate supplementary image policy required')
    for key in ('stage','run_id','scene_id'):_identity(value[key],key)
    if type(value['seed'])is not int or not 0<value['seed']<2**32 or type(value['attempt'])is not int or value['attempt']!=1:raise ValueError('positive seed/attempt1 required')
    if value['method'] not in METHODS:raise ValueError('unknown navigation method identity')
    if value['training_authorized']is not False or value['execution_authorized']is not False:raise ValueError('collector contract conveys no execution/training authority')
    for name,expected in (('activation_source_ns',5_000_000_000),('policy_cap_ns',22_000_000_000),('physics_step_ns',2_000_000)):
        if type(value[name])is not int or value[name]!=expected:raise ValueError('fixed shared timing differs: '+name)
    for key in ('readiness_wall_s','wall_cap_s','clock_stall_wall_s','drain_wall_s','maximum_robot_xy_radius_m'):
        if type(value[key]) not in (int,float) or not math.isfinite(value[key]) or value[key]<=0:raise ValueError('positive finite '+key+' required')
    if value['drain_wall_s']!=5. or not 90.<=value['wall_cap_s']<=900.:raise ValueError('bounded declared lifecycle budgets required')
    if type(value['steering_kd']) not in (int,float) or value['steering_kd'] not in (4.,6.):raise ValueError('root-resolved physical Kd required')
    pose=value['spawn_world_xyz_yaw']
    if type(pose)is not list or len(pose)!=4 or any(type(x) not in (int,float) or not math.isfinite(x) for x in pose) or pose[2]!=0.:raise ValueError('finite source-world spawn xyz/yaw at ground z0 required')
    for key in ('allowed_nodes','required_nodes','required_before_unpause','required_before_arm','record_topics'):_names(value[key],key)
    if not set(value['required_nodes'])<=set(value['allowed_nodes']):raise ValueError('required node absent from allowlist')
    for name in ('coordinator_node','relay_node'):
        _names([value[name]],name)
        if value[name] not in value['required_nodes']:raise ValueError('command owner absent from required nodes')
    if not {'/rosout','/gazebo','/fixture_bag','/astra_navigation_collector'}<=set(value['required_nodes']):raise ValueError('lifecycle evidence nodes omitted')
    if not set(ESSENTIAL_BEFORE_UNPAUSE)<=set(value['required_before_unpause']):raise ValueError('essential paused recorder connections omitted')
    if not set(ESSENTIAL_BEFORE_ARM)<=set(value['required_before_arm']):raise ValueError('essential positive-time recorder connections omitted')
    if not set(value['required_before_unpause'])<=set(value['required_before_arm'])<=set(value['record_topics']):raise ValueError('recorder barrier sets not nested')
    if value['camera_topic'] is not None:_names([value['camera_topic']],'camera_topic')
    if value['camera_topic'] in value['record_topics']:raise ValueError('native camera frames must remain outside dense bag')
    for key in ('world','robot_description','scene_manifest','shared_bundle'):
        item=value[key]
        if type(item)is not dict or set(item)!={'path','sha256'} or not Path(item['path']).is_absolute() or not re.fullmatch('[0-9a-f]{64}',item['sha256']):raise ValueError('exact path+SHA resource required: '+key)
        read_bound(item)
    world=ET.fromstring(read_bound(value['world']));models=[m.get('name') for m in world.findall('world/model')]+['autolabor_m2']
    if value['expected_models']!=models or len(set(models))!=len(models):raise ValueError('expected models differ from saved world')
    if abs(float(world.findtext('world/physics/max_step_size'))-.002)>1e-12:raise ValueError('world physics step differs')
    bundle=json.loads(read_bound(value['shared_bundle']))
    if tuple(bundle['map_from_odom_xy_yaw'])!=(pose[0],pose[1],pose[3]):raise ValueError('spawn reference differs from shared bundle')
    directory=Path(value['shared_bundle']['path']).parent
    for filename,key in (('map.pgm','map_image_sha256'),('map.yaml','map_yaml_sha256')):
        if sha(directory/filename)!=bundle[key]:raise ValueError('shared map asset differs: '+filename)
    if bundle.get('schema')!='astra_common_route_v1' or bundle.get('frame_id')!='map':raise ValueError('shared route schema/frame differs')
    scene=yaml.safe_load(read_bound(value['scene_manifest']))
    if scene.get('scene_id')!=value['scene_id'] or type(scene.get('seed'))is not int or scene['seed']!=value['seed']:raise ValueError('scene identity differs')
    route=scene['route']
    if route.get('frame')!='world':raise ValueError('scene route must use world coordinates')
    for name,key in (('spawn','map_from_odom_xy_yaw'),('start','start_xy_yaw'),('goal','goal_xy_yaw')):
        if _vector(route[name],3,'scene '+name)!=_vector(bundle[key],3,'bundle '+key):raise ValueError('scene/shared route differs: '+name)
    if value['camera_topic'] is not None:
        camera=scene.get('camera',{})
        if camera.get('topic')!=value['camera_topic'] or camera.get('width')!=2560 or camera.get('height')!=1440:raise ValueError('scene camera differs')
        matching=[]
        for sensor in world.findall('.//sensor'):
            for plugin in sensor.findall('plugin'):
                if plugin.get('filename')=='libgazebo_ros_camera.so':
                    topic='/'+ '/'.join(x.strip('/') for x in (plugin.findtext('robotNamespace',''),plugin.findtext('cameraName',''),plugin.findtext('imageTopicName','')) if x.strip('/'))
                    if topic==value['camera_topic']:matching.append(sensor)
        if len(matching)!=1 or matching[0].findtext('camera/image/width')!='2560' or matching[0].findtext('camera/image/height')!='1440':raise ValueError('actual world native camera differs')
    validate_robot_description(read_bound(value['robot_description']).decode(),value)
    return value


def validate_robot_description(xml,value):
    # New stage-independent validator; actual URDF bytes are separately bound.
    # Preserve the explicit P05 controller parameters used by the Q02 fixture.
    root=ET.fromstring(xml);plugins=root.findall('.//plugin')
    required={'libastra_physical_ackermann_p05_plugin.so','libgazebo_ros_laser.so','libgazebo_ros_bumper.so','libgazebo_ros_imu_sensor.so'}
    if root.tag!='robot' or root.get('name')!='autolabor_m2' or len(plugins)!=4 or {p.get('filename') for p in plugins}!=required:raise ValueError('exact P05+three retained sensor robot required')
    driver=next(p for p in plugins if p.get('filename')=='libastra_physical_ackermann_p05_plugin.so')
    tags=[x.tag for x in driver]
    if len(tags)!=len(set(tags)):raise ValueError('duplicate physical parameter')
    expected={'dynamicsSeed':value['seed'],'steeringKd':value['steering_kd'],'wheelbase':.65,'track':.6,'wheelRadius':.15,
      'expectedMass':79.6,'maxSpeed':2.778,'maxSteer':.4964,'commandTimeout':.5,'publishRate':50.,'commandDelay':0.,'commandJitter':0.,
      'wheelKp':4.,'wheelKi':2.,'wheelIntegralTorqueLimit':1.5,'wheelDriveTorqueLimit':8.,'wheelBrakeTorqueLimit':12.,
      'steeringKp':60.,'steeringTorqueLimit':20.,'speedTimeConstant':.22,'steeringTimeConstant':.18,
      'maxAcceleration':.6,'maxDeceleration':.9,'maxBrakeDeceleration':.9,'maxEmergencyDeceleration':.9,'maxSteeringRate':.75}
    for name,wanted in expected.items():
        actual=float(driver.findtext(name,'nan'))
        if not math.isfinite(actual) or abs(actual-wanted)>1e-12:raise ValueError('physical parameter mismatch: '+name)
    if (len(root.findall('link')),len(root.findall('joint')),len(root.findall('link/collision')))!=(10,9,9):raise ValueError('full robot collision/joint roster differs')
    return {'driver':'physical_p05','seed':value['seed'],'steering_kd':value['steering_kd'],'collision_components':9}


def coordinator_identity(row,value):
    if not isinstance(row,dict) or row.get('schema')!='astra_common_navigation_v1' or any(type(row.get(k))is not type(value[k]) or row.get(k)!=value[k] for k in ('run_id','seed','method')):raise ValueError('navigation coordinator identity differs')
    return row


def planner_parameters(values,method):
    expected=({'base_local_planner':'teb_local_planner/TebLocalPlannerROS' if method=='actual_teb_rear_reference_b03' else 'dwa_local_planner/DWAPlannerROS',
      'base_global_planner':'astra_common_route/FixedRouteGlobalPlanner'} if method in ('actual_teb_rear_reference_b03','dwa_rear_reference_b03') else {})
    if values!=expected:raise ValueError('actual configured planner classes differ from method identity')
    return dict(values)


def gate_record(row,value):
    coordinator_identity(row,value);stamp=row.get('source_ns');deadline=row.get('deadline_source_ns')
    if type(stamp)is not int or stamp<value['activation_source_ns'] or stamp%value['physics_step_ns'] or deadline!=stamp+value['policy_cap_ns'] or row.get('deadline_ns')!=deadline:
        raise ValueError('gate requires actual source lattice and fixed22s deadline')
    if row.get('source')!='actual_odom_header_first_crossing':raise ValueError('gate source authority differs')
    for name in ('pose_map','body_twist','goal_pose'):_vector(row.get(name),3,name)
    activation=row.get('activation_source_ns')
    # Activation is the latest actual odom source at the fixed /clock boundary;
    # it can precede 5 s by the published odom interval. Preserve that distinction.
    if type(activation)is not int or not 0<activation<=stamp or activation%value['physics_step_ns']:raise ValueError('actual activation odom source missing')
    return row


def terminal_record(row,value):
    coordinator_identity(row,value)
    if row.get('phase') not in TERMINAL_PHASES:return None
    stamp=row.get('terminal_source_ns')
    if type(stamp)is not int or stamp<0 or stamp%value['physics_step_ns'] or type(row.get('terminal_reason'))is not str or not row['terminal_reason'] or row.get('terminal_time_authority')!=TERMINAL_AUTHORITIES[row['phase']]:
        raise ValueError('terminal source/reason/authority missing')
    if row['phase'] in ('complete','timeout'):
        gate=row.get('gate_source_ns')
        if type(gate)is not int or gate<value['activation_source_ns'] or gate>=stamp:raise ValueError('terminal lacks positive gate interval')
        if row['phase']=='timeout' and stamp!=gate+value['policy_cap_ns']:raise ValueError('timeout differs from declared gate deadline')
        if row['phase']=='complete' and stamp>gate+value['policy_cap_ns']:raise ValueError('completion after policy deadline')
    return row


def validate_services(rows,start,end):
    failures=[];names=[]
    for row in rows:
        name=row.get('name');phase=row.get('phase');stamp=row.get('source_ns');names.append(name)
        if name not in SERVICE_PHASES or phase not in SERVICE_PHASES.get(name,set()) or row.get('completed')is not True or row.get('error') or type(stamp)is not int or stamp<0:
            failures.append(row);continue
        if name in ('/gazebo/spawn_urdf_model','/gazebo/unpause_physics','/astra/navigation/arm') and stamp>=start:failures.append(row)
        if name in ('/gazebo/pause_physics','/astra/navigation/stop') and stamp<end:failures.append(row)
    required=('/gazebo/spawn_urdf_model','/gazebo/unpause_physics','/astra/navigation/arm','/astra/navigation/stop','/gazebo/pause_physics')
    counts={name:names.count(name) for name in required}
    return {'pass':not failures and all(n==1 for n in counts.values()),'forbidden_calls':failures,'required_call_counts':counts}


def navigation_outcome(samples,goal,start,end,terminal,value):
    """Failure-inclusive raw actual-base geometry; not a safety verdict.

    samples are (source_ns, xyz+xyzw) in original bag order, world frame.
    Goal is the scene's world x/y/yaw; first qualifying physics End is retained.
    """
    goal=_vector(goal,3,'goal');terminal_record(terminal,value)
    previous=None;first=None;final_inside=False;count=0;issues=[]
    for stamp,pose in samples:
        if type(stamp)is not int or stamp%STEP_NS:raise ValueError('actual base source must be physics lattice')
        if previous is not None and stamp<=previous:raise ValueError('actual base source replay/regression')
        previous=stamp
        if not start<=stamp<=end:continue
        p=_vector(pose,7,'actual base pose');qx,qy,qz,qw=p[3:]
        norm=sum(x*x for x in p[3:])
        if abs(norm-1)>1e-5:raise ValueError('actual base quaternion not normalized')
        yaw=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
        dyaw=math.atan2(math.sin(yaw-goal[2]),math.cos(yaw-goal[2]))
        inside=math.hypot(p[0]-goal[0],p[1]-goal[1])<=.25 and abs(dyaw)<=.25
        count+=1
        if inside and first is None:first=stamp
        if stamp==end:final_inside=inside
    if count!=(end-start)//STEP_NS+1:issues.append('actual base goal stream missing source samples')
    if terminal['terminal_source_ns']!=end:issues.append('terminal source differs from evaluated end')
    if terminal['phase']=='complete' and (first is None or not final_inside):issues.append('coordinator completion lacks actual base goal geometry')
    return {'task_phase':terminal['phase'],'terminal_reason':terminal['terminal_reason'],
      'terminal_source_ns':end,'terminal_time_authority':terminal['terminal_time_authority'],
      'raw_geometric_arrival_source_ns':first,'raw_geometric_arrival_time_s':None if first is None else (first-start)/1e9,
      'coordinator_goal_time_s':(end-start)/1e9 if terminal['phase']=='complete' else None,
      'goal_position_tolerance_m':.25,'goal_yaw_tolerance_rad':.25,'actual_geometry_period_ns':STEP_NS,
      'goal_geometry_contract_pass':not issues,'issues':issues,'failure_restricted_time_s':value['policy_cap_ns']/1e9,
      'raw_geometry_is_safety_qualification':False}


def closure(root,release,review,value):
    """Create a new run closure from actual inventories and a bound new review.

    release must bind all active sources. The review reuses native implementation
    facts but must separately cover navigation writer/node/service authority.
    This function grants no execution authority and never starts ROS.
    """
    import run_passive_sweep_q05 as inherited
    import run_smoke_a02 as common
    import dense_sweep_p04 as dense
    root=Path(root);validate_contract(value)
    if json.loads(read_bound(release['source_review']))!=review:raise ValueError('source review is not release-bound')
    if review.get('schema')!='astra_navigation_source_review_v1' or review.get('status')!='independently_reviewed':raise ValueError('new navigation source review required')
    resources={str(Path(x['path']).resolve()):x for x in release['resources']}
    reviewed={str(Path(x['path']).resolve()):x for x in review['resources']}
    required_sources=[WORKSPACE/'scripts/astra_gazebo/navigation_evidence_b04.py',
      WORKSPACE/'src/simulation/m2_gazebo/launch/astra_baseline_rear_b04.launch',
      WORKSPACE/'scripts/astra_gazebo/baseline_rear_evidence_b04.py',
      WORKSPACE/'scripts/astra_gazebo/baseline_rear_runtime_b03.py',
      WORKSPACE/'scripts/astra_gazebo/baseline_rear_axle_adapter_b02.py',
      WORKSPACE/'src/simulation/m2_gazebo/scripts/astra_baseline_rear_adapter_b03.py',
      WORKSPACE/'src/simulation/m2_gazebo/config/astra_baseline_rear_b03.yaml',
      WORKSPACE/'src/simulation/m2_gazebo/scripts/astra_navigation_collector_b04.py',
      WORKSPACE/'scripts/astra_gazebo/navigation_coordinator_b03.py',
      WORKSPACE/'src/simulation/m2_gazebo/scripts/astra_navigation_coordinator_b03.py',
      root/'fixture.launch']
    # The launch copy is bound by content, since its prepared canonical path differs.
    resource_digests={x['sha256'] for x in resources.values()};review_digests={x['sha256'] for x in reviewed.values()}
    for path in required_sources:
        digest=sha(path)
        if digest not in resource_digests or digest not in review_digests:raise ValueError('navigation source not reviewed/released: '+str(path))
    for item in release['resources']:read_bound(item)
    if review.get('robot_pose_velocity_writes')!=[]:raise ValueError('unreviewed robot pose/velocity writer')
    if review.get('navigation_lifecycle_authority')!='collector_only_spawn_unpause_pause_fixed_gateway':raise ValueError('navigation service authority review missing')
    raw=json.loads((root/'collector_raw.json').read_text());start,end=raw['start_ns'],raw['end_ns']
    before=json.loads((root/'elf_pre.json').read_text());after=json.loads((root/'elf_final.json').read_text())
    from mapped_elf_inventory import same_process
    process_identity_pass=same_process(before,after)
    facts=inherited.native_review_facts(review)
    expected_modules={x['role']:x['sha256'] for x in review['engine_module_bindings']}
    pre={role:x for x in before['loaded_modules'] for role in x['roles']}
    post={role:x for x in after['loaded_modules'] for role in x['roles']}
    module_checks={role:role in pre and role in post and pre[role]['sha256']==post[role]['sha256']==digest for role,digest in expected_modules.items()}
    from evaluate_episode import WorldCollisions
    world=WorldCollisions(read_bound(value['world']).decode())
    if world.errors:raise ValueError('world geometry/plugin unsupported: '+str(world.errors))
    actor=bool(world.actor_speed_bounds)
    if actor and review.get('actor_updates')!='reviewed_bounded_Begin_updates':raise ValueError('actor pose writers not reviewed')
    required_roles=set(passive.ELF_ROLES.values())-{'actor_binaries','camera'}
    if actor:required_roles.add('actor_binaries')
    if value['camera_topic'] is not None:required_roles.add('camera')
    module_complete=process_identity_pass and required_roles<=set(module_checks) and all(module_checks[r] for r in required_roles)
    native_ok=facts['native_reference_pass'] and pre.get('ode_integrator',{}).get('sha256')==facts['expected_ode_sha256']
    source_ok=facts['upstream_source_review_pass']
    roster_rows=[r for r in raw['node_roster'] if start-200_000_000<=r['source_ns']<=raw['final_drain']['source_watermark_ns']]
    roster_ok=bool(roster_rows) and all(r['pass'] for r in roster_rows) and roster_rows[0]['source_ns']<=start and roster_rows[-1]['source_ns']>=end
    service=validate_services(raw['services'],start,end)
    fields={'run_id':value['run_id'],'start_ns':start,'end_ns':end,'scope_limit':'authorized launch/nodes; not adversarial process proof'}
    audits={
      'engine_binary_maps':dict(fields,**{'pass':module_complete and native_ok and source_ok,'loaded_modules':after['loaded_modules'],
        'same_process_identity_pass':process_identity_pass,
        'external_graphics_dynamic_code':after.get('external_graphics_dynamic_code',[]),
        'graphics_driver_provenance':'binary/pattern evidence only; no NVIDIA source correspondence or anonymous executable memory equality claim',
        'module_checks':module_checks,'runtime_inventory_bindings':[binding(root/'elf_pre.json'),binding(root/'elf_final.json')],
        'native_report_binding':review['native_engine_report'],'native_records_binding':review['native_engine_records'],
        'implementation_review':{'ode_normal_euler_update_verified':native_ok,'gazebo_pre_end_callback_order_verified':source_ok,
         'loaded_module_source_correspondence_reviewed':module_complete and source_ok,'source_bindings':review['engine_source_bindings']},
        'interpretation':'mapped ELF SHA + native reference + reviewed versioned source; not a bit-for-bit source-build theorem'}),
      'node_roster_audit':dict(fields,**{'pass':roster_ok,'loopback_master_isolated':roster_ok,
        'unexpected_nodes':sorted({n for r in roster_rows for n in r['unexpected_nodes']}),'observations':binding(root/'node_roster.jsonl'),
        'sampling':'0.20wall s; trusted authorized node scope'}),
      'service_call_audit':dict(fields,**service,scope='authorized_launch_and_nodes',journal=binding(root/'service_calls.jsonl')),
      'pose_writer_review':dict(fields,**{'pass':module_complete and service['pass'],'robot_pose_velocity_writes':[],
        'actor_updates':'reviewed_bounded_Begin_updates' if actor else 'none','independent_review':release['source_review']})}
    for name,data in audits.items():common.atomic(root/(name+'.json'),data)
    def archived_source(relative):
        path=str((WORKSPACE/relative).resolve());item=resources.get(path)
        if item is None or reviewed.get(path,{}).get('sha256')!=item['sha256']:raise ValueError('active engine source not bound/reviewed: '+path)
        return inherited.archive(root,item)
    bindings={key:[binding(root/(key+'.json'))] for key in audits}
    bindings.update(launch=[binding(root/'fixture.launch')],world=[binding(root/'world.sdf')],
       robot_description=[binding(root/'robot_description_runtime.xml')],
       driver_source=[archived_source('src/simulation/m2_gazebo/src/astra_physical_ackermann_p05_plugin.cpp')],
       evidence_source=[archived_source('src/simulation/m2_gazebo/src/astra_physics_metadata_p03_plugin.cpp')],
       driver_binary=[post['driver_binary']],evidence_binary=[post['evidence_binary']],
       actor_sources=[archived_source('src/simulation/m2_gazebo/src/astra_reactive_actor_plugin.cpp')] if actor else [],
       actor_binaries=[post['actor_binaries']] if actor else [])
    conversion=json.loads((root/'conversion_manifest.json').read_bytes())
    bindings.update(converted_robot_sdf=[binding(root/'converted_robot.sdf')],
        urdf_sdf_conversion_manifest=[binding(root/'conversion_manifest.json')],
        conversion_tool=[conversion['converter_executable']],
        conversion_libraries=conversion['runtime_correspondence_libraries'])
    result={'schema':'astra_p04_run_closure_v1','run_id':value['run_id'],'scope':dense.SCOPE,'numeric_profile':dense.PROFILE,
      'bindings':bindings,'stage':value['stage']}
    common.atomic(root/'run_closure.json',result);return result


def assess(root,value,timeout_s=120.):
    """Offline common physical assessment; capture, safety and task stay separate.

    P04 CLI binds the converted SDF and checks native merged-COM/shape identity. Only
    compact summaries and its derived scalar bounds are written; no raw JSON
    duplicates of dense WorldState/contact bags are created.
    """
    import subprocess
    import rosbag
    import dense_sweep_p04 as dense
    import evaluate_episode as evaluator
    import run_smoke_a02 as common
    root=Path(root);validate_contract(value)
    raw=json.loads((root/'collector_raw.json').read_text());start,end=raw['start_ns'],raw['end_ns']
    source_files={name:binding(root/name) for name in ('fixture.bag','world.sdf','robot_description_runtime.xml','collector_raw.json','collector_report.json','run_closure.json')}
    if source_files['world.sdf']['sha256']!=value['world']['sha256'] or source_files['robot_description_runtime.xml']['sha256']!=value['robot_description']['sha256']:raise ValueError('attempt world/URDF differs from contract')
    out=root/'sweep_evaluation'
    command=[str(WORKSPACE/'.venv/bin/python'),str(Path(dense.__file__)),'--bag',str(root/'fixture.bag'),
      '--world',str(root/'world.sdf'),'--urdf',str(root/'robot_description_runtime.xml'),'--closure',str(root/'run_closure.json'),
      '--converted-sdf',str(root/'converted_robot.sdf'),'--conversion-manifest',str(root/'conversion_manifest.json'),
      '--start-ns',str(start),'--end-ns',str(end),'--output',str(out)]
    with (root/'sweep_evaluation.log').open('x') as stream:
        process=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT,timeout=timeout_s)
    sweep=json.loads((out/'report.json').read_text())
    _models,_links,contacts,metadata=evaluator.read_bag(root/'fixture.bag')
    coverage=source_coverage([r['stamp_ns'] for r in contacts],start,end)
    errors=[];world=evaluator.WorldCollisions((root/'world.sdf').read_text())
    contact=evaluator.contact_audit({r['stamp_ns']:r for r in contacts},world,start,end,'autolabor_m2',set(world.actor_speed_bounds),errors)
    contact_summary=dict(contact)
    for key in ('illegal_robot_contacts','wheel_ground_support_contacts','actor_environment_contacts'):
        contact_summary[key+'_count']=len(contact[key]);contact_summary[key+'_examples']=contact[key][:10];del contact_summary[key]
    samples=[];bag_rear=rear.new_evidence()
    with rosbag.Bag(str(root/'fixture.bag'),'r') as bag:
        for _topic,msg,_receipt in bag.read_messages(topics=['/astra/p03/end_links']+list(rear.TOPICS)):
            if _topic in rear.TOPICS:
                rear.capture(bag_rear,_topic,msg,_receipt.to_nsec());continue
            if msg.header.frame_id!='world':raise ValueError('actual link goal stream not world frame')
            names=list(msg.name);key='autolabor_m2::base_link'
            if names.count(key)!=1 or len(msg.pose)!=len(names):raise ValueError('actual base link identity absent/duplicated')
            p=msg.pose[names.index(key)]
            samples.append((msg.header.stamp.to_nsec(),(p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)))
    scene=yaml.safe_load(read_bound(value['scene_manifest']))
    task=navigation_outcome(samples,scene['route']['goal'],start,end,raw['terminal'],value)
    physical=physical_status_audit(raw['status_messages'],start,end)
    checks={'collector_complete':json.loads((root/'collector_report.json').read_text())['pass'],
      'passive_source_geometry_complete':process.returncode==0 and sweep['integrity_pass'],
      'contact_source_coverage':coverage['pass'],'contact_message_contract':not errors,
      'contact_metadata_complete':all(metadata.get(k)is True for k in ('contacts_every_physics_step','all_collision_pairs_retained','empty_contact_packets_published')),
      'physical_status_message_contract':physical['message_contract_pass'],'physical_status_source_coverage':physical['coverage']['pass'],
      'goal_source_contract':task['goal_geometry_contract_pass']}
    if value['method']=='frozen_r2_v060_sensor_lifecycle_v1':
        graph_rows=[r for r in raw['node_roster'] if start<=r['source_ns']<=raw['final_drain']['source_watermark_ns']]
        checks['sensor_only_method_registrations']=bool(graph_rows) and all(
            r.get('sensor_method_graph',{}).get('pass_') is True for r in graph_rows)
    rear_reference=rear.audit(raw['rear_reference_evidence'],value,start,end,raw['final_drain']['source_watermark_ns'],raw['planner_full_parameters'])
    rear_bag_reference=rear.audit(bag_rear,value,start,end,raw['final_drain']['source_watermark_ns'],raw['planner_full_parameters'])
    checks['rear_reference_source_and_parameter_integrity']=rear_reference['pass_']
    checks['rear_reference_bag_integrity']=rear_bag_reference['pass_']
    checks['rear_reference_bag_collector_source_equality']=rear.bag_source_equality(raw['rear_reference_evidence'],bag_rear,start,end,raw['final_drain']['source_watermark_ns'])
    checks['rear_reference_observed_graph']=bool(raw['node_roster']) and all(r.get('rear_reference_graph',{}).get('pass_')is True for r in raw['node_roster'])
    integrity=all(checks.values())
    safety={'positive_engineering_clearance':sweep['clearance_status']=='pass_within_model_scope',
      'no_illegal_robot_contact':not contact['illegal_robot_contacts'],'no_actor_environment_contact':not contact['actor_environment_contacts'],
      'no_physical_backend_fault_timeout_brake_emergency':physical['no_fault_timeout_brake_emergency']}
    safety_pass=integrity and all(safety.values())
    completed=safety_pass and task['task_phase']=='complete' and task['raw_geometric_arrival_source_ns'] is not None
    for item in source_files.values():
        if sha(item['path'])!=item['sha256']:raise ValueError('assessment source changed during reading: '+item['path'])
    report={'schema':'astra_navigation_assessment_v1','stage':value['stage'],'run_id':value['run_id'],'seed':value['seed'],'method':value['method'],
      'capture_integrity_pass':integrity,'safety_within_declared_engineering_scope_pass':safety_pass,
      'valid_completed':completed,'restricted_goal_time_s':task['raw_geometric_arrival_time_s'] if completed else value['policy_cap_ns']/1e9,
      'failure_inclusive':True,'rear_reference_audit':rear_reference,'rear_reference_bag_audit':rear_bag_reference,'task':task,'integrity_checks':checks,'safety_checks':safety,'physical_status_audit':physical,
      'contact_audit':contact_summary,'contact_issues':errors,'sweep_report':binding(out/'report.json'),'source_bindings':source_files,
      'visual_review':'pending independent inspection of available native PNGs' if value['camera_topic'] else 'not requested',
      'image_coverage':raw['image_coverage'],
      'planner_input_mask_or_decision_integrity_assessed':False,
      'formal_safety':False,'real_continuous_physical_safety':False,'navigation_performance_claim':False}
    common.atomic(root/'assessment.json',report);return report
