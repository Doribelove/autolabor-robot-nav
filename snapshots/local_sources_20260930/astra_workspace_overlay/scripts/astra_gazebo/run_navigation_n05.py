#!/usr/bin/env python3
"""Six-family two-method development diagnosis with shared event actors.

Preparation is offline. Execution requires an exact root release, qualified
passive geometry evidence, fresh identities, and a one-shot 24-unit ledger.
Six parent geometries with two repetitions are not a final performance study.
"""
import argparse
import csv
import fcntl
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import navigation_evidence_n05 as evidence
import run_smoke_a02 as common
import navigation_process_cleanup_n05 as cleanup

WORKSPACE = evidence.WORKSPACE
STAGE = evidence.STAGE
NAME = 'astra_navigation_n05_preview_selection'
DESIGN = STAGE/'contracts/navigation_n05_preview_selection_design.json'
DESIGN_SHA = '959b214b3f4a10da276ee46ce9c319c9f5ea8001551f9dc4626b97f9714baefc'
SCENE = STAGE/'scenes/navigation_n05_development_r01'
LAUNCH = WORKSPACE/'src/simulation/m2_gazebo/launch/astra_common_navigation_n05.launch'
CHECKPOINT = common.CHECKPOINT


def design():
    value=json.loads(evidence.read_bound(dict(path=str(DESIGN),sha256=DESIGN_SHA)))
    if value['stage']!=NAME or value['proposed_maximum_episodes']!=24 or len(value['schedule'])!=24:
        raise ValueError('fixed N05 development design required')
    return value


def input_blocks():
    rows=list(csv.DictReader((SCENE/'diagnostic_blocks.csv').open()))
    if len(rows)!=12:
        raise ValueError('exact twelve development input blocks required')
    schedule=design()['schedule']
    for index,row in enumerate(rows):
        expected=schedule[index*2]
        if int(row['seed'])!=263002001+index or int(row['replicate'])!=expected['replicate']:
            raise ValueError('fresh block seed or repetition changed')
        for key in ('family','layout_id','parent_geometry_id','scene_id'):
            if row[key]!=expected[key]:raise ValueError('parent input mapping changed: '+key)
        for pathkey,hashkey in (('world_path','world_sha256'),('manifest_path','manifest_sha256')):
            path=Path(row[pathkey]).resolve()
            if SCENE.resolve() not in path.parents:
                raise ValueError('input scene path outside versioned scene tree')
            evidence.read_bound(dict(path=str(path),sha256=row[hashkey]))
    return rows


def block_for(run_id):
    matches=[row for row in design()['schedule'] if row['run_id']==run_id]
    if len(matches)!=1:raise ValueError('unknown or duplicate scheduled run')
    return input_blocks()[matches[0]['block_index']]


def contract_for(run_id,robot_description,world):
    import yaml
    row=next(row for row in design()['schedule'] if row['run_id']==run_id)
    block=block_for(run_id);manifest=Path(block['manifest_path'])
    scene=yaml.safe_load(manifest.read_bytes())
    pose=scene['route']['spawn']
    if scene['seed']!=row['seed'] or scene['split']!='development':
        raise ValueError('new development scene identity required')
    return evidence.make_contract(stage=NAME,run_id=run_id,seed=row['seed'],method=row['method'],
        scene_id=row['scene_id'],world=world,robot_description=robot_description,scene_manifest=manifest,
        shared_bundle=manifest.parent/'common_bundle',spawn_world_xyz_yaw=[pose[0],pose[1],0.,pose[2]],
        steering_kd=6.,allowed_nodes=node_roster(row['method']),required_nodes=node_roster(row['method']),
        record_topics=recorder_topics(row['method']),required_before_arm=recorder_topics(row['method']),
        camera_topic=scene['camera']['topic'])


def identities():
    return [(row['run_id'], row['seed'], row['method']) for row in design()['schedule']]


def required_resources():
    """Explicit active import/launch/model graph; excludes unrelated new work."""
    import passive_sweep_q05 as passive
    paths = set(passive.required_resources())
    scripts = ('run_navigation_n05.py', 'navigation_evidence_n05.py', 'navigation_coordinator_n05.py',
        'run_passive_sweep_q05.py', 'passive_sweep_q05.py', 'dense_sweep_n05.py', 'dense_sweep_p04.py', 'event_actor_contract_n05.py', 'bound_archive.py',
        'run_passive_sweep_q04.py', 'passive_sweep_q04.py', 'mapped_elf_inventory_v2.py',
        'run_passive_sweep_q03.py', 'passive_sweep_q03.py', 'mapped_elf_inventory.py',
        'run_passive_sweep_q02.py', 'passive_sweep_q02.py', 'run_smoke_a02.py',
        'dense_sweep_p03.py', 'dense_sweep.py', 'geometry.py', 'evaluate_episode.py', 'evaluate_episode_n05.py', 'prepare_navigation_n05_scenes.py',
        'prepare_navigation_n01_scene.py', 'scene_generator_v2.py', 'scene_generator.py',
        'common_route.py', 'static_map_export.py', 'route_world_adapter.py',
        'axle_reference_adapter.py', 'route_candidate_core.py', 'route_sensor_buffer.py', 'route_actor_adapter.py', 'frozen_r2_sensor_lifecycle.py',
        'route_preview_selection_n05.py', 'release_validation_session_n05.py', 'navigation_process_cleanup_n05.py')
    paths.update(WORKSPACE/'scripts/astra_gazebo'/name for name in scripts)
    nodes = ('astra_navigation_collector_n05.py', 'astra_navigation_coordinator_n05.py',
        'astra_common_route_provider.py', 'astra_route_coordinated_runtime.py', 'astra_route_preview_runtime_n05.py', 'astra_frozen_r2_sensor_lifecycle.py')
    paths.update(WORKSPACE/'src/simulation/m2_gazebo/scripts'/name for name in nodes)
    configs = ('benchmark_move_base.yaml', 'costmap_common.yaml', 'local_costmap.yaml',
        'global_costmap.yaml', 'benchmark_teb_v0.yaml', 'benchmark_dwa_v0.yaml',
        'astra_common_map_route.yaml', 'astra_navigation_common_footprint.yaml',
        'benchmark_dwa_nonholonomic_r1.yaml')
    paths.update(WORKSPACE/'src/simulation/m2_gazebo/config'/name for name in configs)
    paths.update((LAUNCH, CHECKPOINT,
        WORKSPACE/'src/simulation/m2_gazebo/astra_common_route_plugins.xml',
        WORKSPACE/'src/simulation/m2_gazebo/package.xml',
        WORKSPACE/'src/simulation/m2_gazebo/src/astra_common_route_global_planner.cpp',
        WORKSPACE/'devel/lib/libastra_common_route_global_planner.so',
        WORKSPACE/'devel/lib/libteb_local_planner.so',
        Path('/opt/ros/noetic/lib/libdwa_local_planner.so'),
        Path('/opt/ros/noetic/lib/move_base/move_base')))
    for package in ('v5_e2e_scope_rl', 'v3_scope_planner'):
        paths.update((WORKSPACE/'src/application'/package/'src'/package).glob('*.py'))
    paths.update(WORKSPACE/'src/application/v5_e2e_scope_rl/scripts'/name for name in
        ('pct_teb_dwa_benchmark_episode.py', 'pct_teb_dwa_benchmark_r2_episode.py'))
    paths.update(p for p in SCENE.rglob('*') if p.is_file())
    paths.update((DESIGN, WORKSPACE/'devel/lib/libastra_event_actor_n05_plugin.so',
        WORKSPACE/'src/simulation/m2_gazebo/src/astra_event_actor_n05_plugin.cpp',
        WORKSPACE/'src/simulation/m2_gazebo/include/m2_gazebo/astra_event_actor_gate_n05.h'))
    return sorted({path.resolve() for path in paths})


def write_new(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())


def node_roster(method):
    names = ['/rosout', '/gazebo', '/fixture_bag', '/astra_navigation_collector',
             '/m2_robot_state_publisher', '/navigation_command_relay',
             '/astra_common_route_provider', '/astra_navigation_coordinator']
    names.append('/move_base' if method in ('actual_teb_common_route', 'dwa_common_route')
                 else '/astra_frozen_r2_method' if method == 'frozen_r2_v060_sensor_lifecycle_v1'
                 else '/astra_route_runtime')
    return names


def recorder_topics(method):
    topics = list(evidence.ESSENTIAL_BEFORE_ARM)
    topics += ['/astra/common_route/identity', '/astra/common_route/served_path']
    if method in ('astra_analytic_route_v1', 'astra_analytic_preview_eligible_n05'):
        topics += ['/astra/route_runtime/identity', '/astra/route_runtime/status',
                   '/astra/route_runtime/trace', '/astra/route_runtime/candidates']
    elif method == 'frozen_r2_v060_sensor_lifecycle_v1':
        topics += ['/astra/frozen_r2_sensor_lifecycle/'+name for name in
                   ('identity', 'input_receipt', 'trace', 'command_event', 'report')]
    else:
        planner = 'TebLocalPlannerROS' if method == 'actual_teb_common_route' else 'DWAPlannerROS'
        topics += ['/move_base/'+planner+'/local_plan', '/move_base/'+planner+'/global_plan']
    return list(dict.fromkeys(topics))


def render_launch(value, contract_sha, output):
    output = Path(output)
    root = ET.Element('launch')
    include = ET.SubElement(root, 'include', file=str(LAUNCH))
    args = dict(world=value['world']['path'], run_id=value['run_id'], seed=str(value['seed']),
                method=value['method'], bundle=str(Path(value['shared_bundle']['path']).parent),
                bundle_sha256=value['shared_bundle']['sha256'], scene_manifest=value['scene_manifest']['path'],
                scene_id=value['scene_id'], legacy_report_path=str(output/'legacy_report.json'),
                steering_kd=str(value['steering_kd']), legacy_checkpoint_path=str(CHECKPOINT),
                legacy_checkpoint_sha256=evidence.sha(CHECKPOINT),
                collector_script='astra_navigation_collector_n05.py',
                dwa_config=str(WORKSPACE/'src/simulation/m2_gazebo/config/benchmark_dwa_nonholonomic_r1.yaml'),
                contract_path=str(output/'contract.json'), contract_sha256=contract_sha,
                output=str(output), bag_path=str(output/'fixture.bag'),
                bag_topics=' '.join(topic for topic in value['record_topics']
                                    if not topic.startswith('/astra/frozen_r2_sensor_lifecycle/')))
    for name, text in args.items():
        ET.SubElement(include, 'arg', name=name, value=text)
    return ET.tostring(root, encoding='unicode')+'\n'


def prepare(output):
    import dense_sweep_n05 as dense
    output = Path(output).resolve(); output.mkdir(exist_ok=False)
    units = []
    for run_id, seed, method in identities():
        directory = output/run_id; directory.mkdir()
        urdf = subprocess.check_output(['xacro', '--inorder',
            str(WORKSPACE/'src/simulation/m2_gazebo/urdf/m2_physical_p05.urdf.xacro'),
            'dynamics_seed:='+str(seed), 'steering_kd:=6.0'], text=True, timeout=30)
        (directory/'robot_description.xml').write_text(urdf)
        from prepare_navigation_n05_scenes import bind_gate_identity
        block=block_for(run_id)
        template=Path(block['world_path']).read_text()
        (directory/'world.sdf').write_text(bind_gate_identity(template,run_id,seed,method))
        value=contract_for(run_id,directory/'robot_description.xml',directory/'world.sdf')
        write_new(directory/'contract.json', value)
        (directory/'fixture.launch').write_text(render_launch(value, evidence.sha(directory/'contract.json'),
                                                              STAGE/'runs'/run_id))
        conversion_path = dense.create_conversion(directory/'robot_description.xml', directory/'conversion')
        conversion = json.loads(conversion_path.read_bytes())
        units.append(dict(run_id=run_id, seed=seed, method=method, attempt=1,
            conversion_manifest=evidence.binding(conversion_path), converted_sdf=conversion['converted_sdf'],
            conversion_resources=dense.conversion_resources(conversion_path),
            files=[evidence.binding(directory/name) for name in
                   ('contract.json', 'robot_description.xml', 'fixture.launch', 'world.sdf')]))
    result = dict(stage=NAME, status='offline_prepared_no_authority', execution_authorized=False,
        maximum_episodes=len(identities()), units=units, scope='six-family two-method development diagnosis with six parent geometries and two repetitions',
        algorithm_failures_retained=True, infrastructure_failure_retires_remaining=True,
        retry_allowed=False, resume_allowed=False, training_authorized=False, formal_result=False)
    write_new(output/'prepared.json', result)
    return result


def verify_release(path, digest):
    release = json.loads(evidence.read_bound(dict(path=str(path), sha256=digest)))
    if (release.get('stage') != NAME or release.get('status') != 'root_reviewed_release'
            or release.get('execution_authorized') is not True
            or type(release.get('maximum_episodes')) is not int or release['maximum_episodes'] != len(identities())
            or release.get('retry_allowed') is not False or release.get('resume_allowed') is not False):
        raise ValueError('exact one-shot24-unit root release required')
    prepared = json.loads(evidence.read_bound(release['prepared']))
    expected = identities()
    if [(u['run_id'], u['seed'], u['method']) for u in prepared['units']] != expected:
        raise ValueError('schedule identity/order changed')
    if any(type(u.get('attempt')) is not int or u['attempt'] != 1 for u in prepared['units']):
        raise ValueError('only attempt1 is authorized')
    review = json.loads(evidence.read_bound(release['source_review']))
    if (review.get('status') != 'independently_reviewed' or review.get('stage') != NAME
            or review.get('schema') != 'astra_navigation_source_review_v1'):
        raise ValueError('new navigation source review absent')
    if (review.get('navigation_lifecycle_authority') != 'collector_only_spawn_unpause_pause_fixed_gateway'
            or review.get('robot_pose_velocity_writes') != []
            or review.get('actor_updates') != 'reviewed_bounded_Begin_updates'):
        raise ValueError('navigation writer and lifecycle review incomplete')
    prerequisite = json.loads(evidence.read_bound(release['passive_geometry_qualification']))
    if (prerequisite.get('qualified_for_navigation_evidence') is not True
            or prerequisite.get('resolved_steering_kd') != 6.
            or not isinstance(prerequisite.get('evidence_bindings'), list)
            or not prerequisite['evidence_bindings']):
        raise ValueError('passive geometry prerequisite not qualified')
    for item in prerequisite['evidence_bindings']:
        evidence.read_bound(item)
    resources = release['resources']
    by_path = {str(Path(item['path']).resolve()): item['sha256'] for item in resources}
    if len(resources) != len(by_path):
        raise ValueError('duplicate release resource paths')
    for item in resources:
        evidence.read_bound(item)
    # Static evidence required by the terminal closure must fail before any
    # ledger/allocation, rather than after an otherwise valid navigation run.
    import run_passive_sweep_q05 as passive_manager
    native = passive_manager.native_review_facts(review)
    if not native['native_reference_pass'] or not native['upstream_source_review_pass']:
        raise ValueError('native implementation review facts failed')
    engine_rows = review['engine_module_bindings']
    roles = [x['role'] for x in engine_rows]
    required_roles = set(evidence.ELF_ROLES.values())
    if len(roles) != len(set(roles)) or not required_roles <= set(roles):
        raise ValueError('N05 reviewed engine role roster incomplete or duplicated')
    static_bindings = engine_rows + review['engine_source_bindings'] + [review[x] for x in
        ('native_engine_report', 'native_engine_records', 'upstream_source_manifest')]
    for item in static_bindings:
        evidence.read_bound(item)
        if by_path.get(str(Path(item['path']).resolve())) != item['sha256']:
            raise ValueError('native review resource absent from exact release')
    for item in review['resources']:
        evidence.read_bound(item)
        if by_path.get(str(Path(item['path']).resolve())) != item['sha256']:
            raise ValueError('reviewed resource not bound by release')
    reviewed = {str(Path(item['path']).resolve()): item['sha256'] for item in review['resources']}
    for path in required_resources():
        if by_path.get(str(path.resolve())) != evidence.sha(path):
            raise ValueError('required runner/runtime resource missing: '+str(path))
        if path.suffix in ('.py', '.yaml', '.xacro', '.xml', '.cpp', '.h', '.launch') and reviewed.get(str(path.resolve())) != evidence.sha(path):
            raise ValueError('active source/config absent from independent review: '+str(path))
    for unit in prepared['units']:
        if [Path(x['path']).name for x in unit['files']] != ['contract.json', 'robot_description.xml', 'fixture.launch', 'world.sdf']:
            raise ValueError('prepared unit files changed')
        value = json.loads(evidence.read_bound(unit['files'][0])); evidence.validate_contract(value)
        if (value['stage'], value['run_id'], value['seed'], value['method']) != (NAME, unit['run_id'], unit['seed'], unit['method']):
            raise ValueError('contract identity differs from schedule')
        expected_contract = contract_for(unit['run_id'],unit['files'][1]['path'],unit['files'][3]['path'])
        from prepare_navigation_n05_scenes import bind_gate_identity
        expected_world=bind_gate_identity(Path(block_for(unit['run_id'])['world_path']).read_text(),
            unit['run_id'],unit['seed'],unit['method'])
        if evidence.read_bound(unit['files'][3]).decode()!=expected_world:
            raise ValueError('runtime world differs from exact template identity binding')
        if not evidence.passive.same_typed(value, expected_contract):
            raise ValueError('contract differs from exact shared N05 scene/protocol/physical configuration')
        for item in unit['files']:
            evidence.read_bound(item)
            if by_path.get(str(Path(item['path']).resolve())) != item['sha256']:
                raise ValueError('unit file not in release')
        if evidence.read_bound(unit['files'][2]).decode() != render_launch(value, unit['files'][0]['sha256'], STAGE/'runs'/unit['run_id']):
            raise ValueError('launch differs from reviewed generator')
        import dense_sweep_n05 as dense
        conversion = json.loads(evidence.read_bound(unit['conversion_manifest']))
        if conversion['input_urdf'] != unit['files'][1] or conversion['converted_sdf'] != unit['converted_sdf']:
            raise ValueError('N05 model conversion differs from prepared unit')
        conversion_resources = dense.conversion_resources(unit['conversion_manifest']['path'])
        if conversion_resources != unit['conversion_resources']:
            raise ValueError('N05 conversion dependency roster differs')
        for item in conversion_resources:
            evidence.read_bound(item)
            if by_path.get(str(Path(item['path']).resolve())) != item['sha256']:
                raise ValueError('N05 converter resource absent from release')
    return release, prepared, review


def capture_baseline_runtime(root, marker, proc, value, release, passive_manager):
    """Bind actual move_base plugin mappings after TF initialization.

    The bootstrap marker records its process identity without waiting for
    plugins that require a positive clock. Final mappings are captured while
    paused, before cleanup. They prove loaded implementation, not every call.
    """
    if value['method'] not in ('actual_teb_common_route', 'dwa_common_route'):
        return
    from mapped_elf_inventory import inventory
    request = json.loads(marker.read_bytes())
    expected_parameters = dict(
        base_local_planner='teb_local_planner/TebLocalPlannerROS' if value['method'] == 'actual_teb_common_route' else 'dwa_local_planner/DWAPlannerROS',
        base_global_planner='astra_common_route/FixedRouteGlobalPlanner')
    if request.get('planner_parameters') != expected_parameters:
        raise ValueError('actual move_base planner selection parameters differ')
    processes = [r for r in request['node_processes'] if r['node'] == '/move_base']
    if len(processes) != 1:
        raise ValueError('exact actual move_base process marker required')
    process = processes[0]; pid = process['pid']
    if not passive_manager.descendant(pid, proc.pid):
        raise ValueError('move_base process not owned by launched tree')
    if request['phase'] == 'bootstrap':
        write_new(root/'baseline_process_bootstrap.json', process)
        return
    before = json.loads((root/'baseline_process_bootstrap.json').read_bytes())
    if (before['pid'], str(before['proc_start_ticks'])) != (pid, str(process['proc_start_ticks'])):
        raise ValueError('move_base process changed between bootstrap and terminal')
    roles = {'move_base': 'move_base_executable',
             'libastra_common_route_global_planner.so': 'fixed_global_planner',
             'libteb_local_planner.so': 'actual_teb', 'libdwa_local_planner.so': 'actual_dwa'}
    active = 'actual_teb' if value['method'] == 'actual_teb_common_route' else 'actual_dwa'
    expected = {'move_base_executable', 'fixed_global_planner', active}
    actual = inventory(pid, roles, expected, root/'baseline_final.maps.txt')
    if actual['proc_start_ticks'] != str(process['proc_start_ticks']):
        raise ValueError('move_base marker/current process identity differs')
    bound = {str(Path(r['path']).resolve()): r['sha256'] for r in release['resources']}
    for item in actual['loaded_modules']:
        if bound.get(str(Path(item['path']).resolve())) != item['sha256']:
            raise ValueError('actual baseline mapped module absent from release')
        original = item['path']; archived = passive_manager.archive(root, item)
        item.update(path=archived['path'], original_path=original)
    actual.update(method=value['method'], pass_=True, source_ns=request['source_ns'],
                  planner_parameters=request['planner_parameters'],
                  bootstrap_process=evidence.binding(root/'baseline_process_bootstrap.json'),
                  claim_scope='actual plugins loaded in same owned move_base process; not proof of every planner call')
    write_new(root/'baseline_runtime_inventory.json', actual)


def execute(release_path, release_sha):
    if Path(os.environ.get('THESIS_WS', '')).resolve() != WORKSPACE:
        raise ValueError('source isolated astra_env first')
    # Q05 also binds the actual model converter and shares immutable code blobs.
    import run_passive_sweep_q05 as passive_manager
    guard = (STAGE/'queue/simulation.lock').open('a')
    fcntl.flock(guard, fcntl.LOCK_EX | fcntl.LOCK_NB)
    # Live predecessor and identity checks stay outside the immutable-file cache.
    predecessor = json.loads((STAGE/'queue/navigation_n04_ledger.json').read_text())
    if predecessor.get('stage') != 'astra_navigation_n04' or predecessor.get('status') != 'terminal':
        raise ValueError('N04 must be terminal before N05 execution')
    for previous in predecessor['units']:
        if common.owned_processes(STAGE/'runs'/previous['run_id']):
            raise ValueError('N04 still has live owned processes')
    from release_validation_session_n05 import UnchangedReleaseSession
    release_session = UnchangedReleaseSession(sys.modules[__name__], release_path, release_sha)
    release, prepared, review = release_session.verify_unchanged()
    ledger = STAGE/'queue/navigation_n05_ledger.json'
    if ledger.exists() or any((STAGE/'runs'/row[0]).exists() for row in identities()):
        raise ValueError('immutable identity already exists; retry prohibited')
    registry = list(csv.DictReader((STAGE/'seed_registry.csv').open()))
    for seed in sorted({x[1] for x in identities()}):
        same_seed=[r for r in registry if r['seed']==str(seed)]
        expected_names={x[0] for x in identities() if x[1]==seed}
        if len(same_seed)!=2 or {r['run_id'] for r in same_seed}!=expected_names:
            raise ValueError('N05 fresh block seed has foreign or duplicate identities')
    for name, seed, _ in identities():
        own = [r for r in registry if r['run_id'] == name]
        if len(own) != 1 or own[0]['seed'] != str(seed) or own[0]['status'] != 'reserved_exact_n05_release' or own[0]['source_binding'] != release_sha:
            raise ValueError('exact root reservation missing or already consumed')
    state = dict(stage=NAME, status='active', release_sha256=release_sha, maximum_units=len(identities()),
        started_utc=common.utc(), units=[dict(run_id=n, seed=s, method=m, status='reserved') for n, s, m in identities()])
    write_new(ledger, state); receipts = []
    try:
        for index, unit in enumerate(prepared['units']):
            state['units'][index]['status'] = 'consumed'; common.atomic(ledger, state)
            root = STAGE/'runs'/unit['run_id']; started = time.monotonic()
            proc = None; error = None; code = None; report = None; value = None
            try:
                root.mkdir()
                for item in unit['files']:
                    (root/Path(item['path']).name).write_bytes(evidence.read_bound(item))
                (root/'conversion_manifest.json').write_bytes(evidence.read_bound(unit['conversion_manifest']))
                (root/'converted_robot.sdf').write_bytes(evidence.read_bound(unit['converted_sdf']))
                value = json.loads((root/'contract.json').read_bytes())
                (root/'world.sdf').write_bytes(evidence.read_bound(value['world']))
                (root/'scene_manifest.yaml').write_bytes(evidence.read_bound(value['scene_manifest']))
                common.atomic(root/'shared_bundle_binding.json', value['shared_bundle'])
                common.atomic(root/'allocated.json', dict(state['units'][index], attempt=1, started_utc=common.utc()))
                for item in release['resources']:
                    passive_manager.archive(root, item)
                for name in ('prepared', 'source_review', 'passive_geometry_qualification'):
                    passive_manager.archive(root, release[name])
                release_session.verify_unchanged()
                for port in (14830, 14831):
                    with socket.socket() as sock:
                        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                        sock.bind(('127.0.0.1', port))
                env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:14830', GAZEBO_MASTER_URI='http://127.0.0.1:14831',
                    ROS_IP='127.0.0.1', ROS_HOSTNAME='127.0.0.1', GAZEBO_IP='127.0.0.1',
                    ROS_LOG_DIR=str(root/'roslog'), ROS_HOME=str(root/'ros_home'), GAZEBO_MODEL_DATABASE_URI='')
                with (root/'roslaunch.log').open('wb') as log:
                    proc = subprocess.Popen(['roslaunch', str(root/'fixture.launch')], cwd=str(WORKSPACE), env=env,
                                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                    common.atomic(root/'process.json', dict(pid=proc.pid, pgid=proc.pid,
                        proc_start_ticks=Path('/proc/{}/stat'.format(proc.pid)).read_text().rsplit(')', 1)[1].split()[19]))
                    while proc.poll() is None:
                        for marker, ack in [('ready_for_maps.json', 'maps_pre_ready.json'), ('ready_for_maps_final.json', 'maps_final_ready.json')]:
                            if (root/marker).exists() and not (root/ack).exists():
                                capture_baseline_runtime(root, root/marker, proc, value, release, passive_manager)
                                evidence.capture_maps(root, root/marker, proc)
                        if time.monotonic()-started > value['wall_cap_s']:
                            raise TimeoutError('navigation fixture wall cap')
                        common.atomic(root/'heartbeat.json', dict(utc=common.utc(), elapsed_wall_s=time.monotonic()-started, pid=proc.pid))
                        time.sleep(.1)
                    code = proc.returncode
            except BaseException as exc:
                error = type(exc).__name__+': '+str(exc)
            finally:
                if proc is not None:
                    try:
                        cleanup.stop_group(proc, root)
                    except BaseException as exc:
                        error = (error or '')+'; cleanup '+repr(exc)
            try:
                if error or code != 0:
                    raise RuntimeError('process/preparation failure: '+str(error)+' exit='+str(code))
                evidence.closure(root, release, review, value)
                report = evidence.assess(root, value)
                release_session.verify_unchanged()
                if time.monotonic()-started > value['wall_cap_s']:
                    raise TimeoutError('complete attempt including assessment exceeded wall cap')
            except BaseException as exc:
                error = (error or '')+'; assessment '+repr(exc)
            integrity_pass = bool(not error and report and report.get('capture_integrity_pass') is True)
            receipt = dict(stage=NAME, run_id=unit['run_id'], seed=unit['seed'], method=unit['method'], attempt=1,
                retry_allowed=False, integrity_pass=integrity_pass,
                task_outcome=None if report is None else report.get('task', {}).get('task_phase'),
                error=error, exit_code=code, wall_elapsed_s=time.monotonic()-started, release_sha256=release_sha,
                report=evidence.binding(root/'assessment.json') if (root/'assessment.json').is_file() else None,
                artifacts=[evidence.binding(p) for p in root.iterdir() if p.is_file() and p.suffix in ('.json', '.sdf', '.xml', '.bag', '.launch')] if root.is_dir() else [])
            receipt_path = root/'receipt.json' if root.is_dir() else STAGE/'queue'/(unit['run_id']+'_preparation_failure.json')
            common.atomic(receipt_path, receipt); receipts.append(receipt)
            state['units'][index].update(receipt=evidence.binding(receipt_path), integrity_pass=integrity_pass,
                                         task_outcome=receipt['task_outcome'])
            common.atomic(ledger, state)
            if not integrity_pass:
                break
    finally:
        for row in state['units']:
            if row['status'] == 'reserved':
                row['status'] = 'retired_unrun'
        state.update(status='terminal', ended_utc=common.utc(), consumed_units=sum(r['status'] == 'consumed' for r in state['units']))
        common.atomic(ledger, state)
    return 0 if len(receipts) == len(identities()) and all(r['integrity_pass'] for r in receipts) else 3


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepare', type=Path); parser.add_argument('--execute', action='store_true')
    parser.add_argument('--release', type=Path); parser.add_argument('--release-sha256')
    args = parser.parse_args()
    if args.execute:
        if args.prepare or not args.release or not args.release_sha256:
            raise ValueError('execute requires only exact release path and SHA')
        result = execute(args.release, args.release_sha256)
        # Persist the manager's returned value separately from roslaunch status
        # and the outer command-session exit reported by the host.
        write_new(STAGE/'queue/navigation_n05_manager_return.json', dict(
            stage=NAME, returned_code=result, manager_pid=os.getpid(),
            release_sha256=args.release_sha256, utc=common.utc(),
            authority='execute() returned after terminal ledger; not proof of later host shell exit'))
        print(json.dumps(dict(stage=NAME, manager_returned_code=result)), flush=True)
        return result
    if args.prepare:
        print(json.dumps(prepare(args.prepare), indent=2)); return 0
    print(json.dumps(dict(stage=NAME, execution_authorized=False, identities=identities())))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
