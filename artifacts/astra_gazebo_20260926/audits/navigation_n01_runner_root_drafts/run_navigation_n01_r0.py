#!/usr/bin/env python3
"""Bounded four-method shared-navigation integration, never a performance study.

Preparation is offline. Execution requires an exact root release, qualified
passive geometry evidence, fresh identities, and a one-shot four-unit ledger.
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

import navigation_evidence as evidence
import run_smoke_a02 as common

WORKSPACE = evidence.WORKSPACE
STAGE = evidence.STAGE
NAME = 'astra_navigation_n01'
SEED = 260926026
METHODS = ('astra_analytic_route_v1', 'actual_teb_common_route',
           'frozen_r2_v060_shared_activation', 'dwa_common_route')
SCENE = STAGE/'scenes/navigation_n01_clear_r03'
LAUNCH = WORKSPACE/'src/simulation/m2_gazebo/launch/astra_common_navigation_smoke.launch'
CHECKPOINT = common.CHECKPOINT


def identities():
    return [(NAME+'_'+method, SEED, method) for method in METHODS]


def write_new(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())


def node_roster(method):
    names = ['/rosout', '/gazebo', '/fixture_bag', '/astra_navigation_collector',
             '/m2_robot_state_publisher', '/navigation_command_relay',
             '/astra_common_route_provider', '/astra_navigation_coordinator']
    names.append('/move_base' if method in ('actual_teb_common_route', 'dwa_common_route')
                 else '/astra_frozen_r2_method' if method == 'frozen_r2_v060_shared_activation'
                 else '/astra_route_runtime')
    return names


def render_launch(value, contract_sha, output):
    output = Path(output)
    root = ET.Element('launch')
    include = ET.SubElement(root, 'include', file=str(LAUNCH))
    args = dict(world=value['world']['path'], run_id=value['run_id'], seed=str(value['seed']),
                method=value['method'], bundle=str(Path(value['shared_bundle']['path']).parent),
                bundle_sha256=value['shared_bundle']['sha256'], scene_manifest=value['scene_manifest']['path'],
                scene_id=value['scene_id'], legacy_report_path=str(output/'legacy_report.json'),
                steering_kd=str(value['steering_kd']), legacy_checkpoint_path=str(CHECKPOINT),
                contract_path=str(output/'contract.json'), contract_sha256=contract_sha,
                output=str(output), bag_path=str(output/'fixture.bag'),
                bag_topics=' '.join(value['record_topics']))
    for name, text in args.items():
        ET.SubElement(include, 'arg', name=name, value=text)
    return ET.tostring(root, encoding='unicode')+'\n'


def prepare(output):
    output = Path(output).resolve(); output.mkdir(exist_ok=False)
    units = []
    for run_id, seed, method in identities():
        directory = output/run_id; directory.mkdir()
        urdf = subprocess.check_output(['xacro', '--inorder',
            str(WORKSPACE/'src/simulation/m2_gazebo/urdf/m2_physical_p05.urdf.xacro'),
            'dynamics_seed:='+str(seed), 'steering_kd:=6.0'], text=True, timeout=30)
        (directory/'robot_description.xml').write_text(urdf)
        value = evidence.make_contract(stage=NAME, run_id=run_id, seed=seed, method=method,
            scene_id='navigation_n01_clear', world=SCENE/'world.sdf',
            robot_description=directory/'robot_description.xml', scene_manifest=SCENE/'scene_manifest.yaml',
            shared_bundle=SCENE/'common_bundle', spawn_world_xyz_yaw=[-5.4, 0., 0., 0.],
            steering_kd=6., allowed_nodes=node_roster(method), required_nodes=node_roster(method))
        write_new(directory/'contract.json', value)
        (directory/'fixture.launch').write_text(render_launch(value, evidence.sha(directory/'contract.json'),
                                                              STAGE/'runs'/run_id))
        units.append(dict(run_id=run_id, seed=seed, method=method, attempt=1,
            files=[evidence.binding(directory/name) for name in
                   ('contract.json', 'robot_description.xml', 'fixture.launch')]))
    result = dict(stage=NAME, status='offline_prepared_no_authority', execution_authorized=False,
        maximum_episodes=4, units=units, scope='clear-fixture software and evidence integration only',
        algorithm_failures_retained=True, infrastructure_failure_retires_remaining=True,
        retry_allowed=False, resume_allowed=False, training_authorized=False, formal_result=False)
    write_new(output/'prepared.json', result)
    return result


def verify_release(path, digest):
    release = json.loads(evidence.read_bound(dict(path=str(path), sha256=digest)))
    if (release.get('stage') != NAME or release.get('status') != 'root_reviewed_release'
            or release.get('execution_authorized') is not True
            or type(release.get('maximum_episodes')) is not int or release['maximum_episodes'] != 4
            or release.get('retry_allowed') is not False or release.get('resume_allowed') is not False):
        raise ValueError('exact one-shot four-unit root release required')
    prepared = json.loads(evidence.read_bound(release['prepared']))
    expected = identities()
    if [(u['run_id'], u['seed'], u['method']) for u in prepared['units']] != expected:
        raise ValueError('schedule identity/order changed')
    if any(type(u.get('attempt')) is not int or u['attempt'] != 1 for u in prepared['units']):
        raise ValueError('only attempt1 is authorized')
    review = json.loads(evidence.read_bound(release['source_review']))
    if review.get('status') != 'independently_reviewed' or review.get('stage') != NAME:
        raise ValueError('new navigation source review absent')
    prerequisite = json.loads(evidence.read_bound(release['passive_geometry_qualification']))
    if prerequisite.get('qualified_for_navigation_evidence') is not True:
        raise ValueError('passive geometry prerequisite not qualified')
    for item in prerequisite['evidence_bindings']:
        evidence.read_bound(item)
    resources = release['resources']
    by_path = {str(Path(item['path']).resolve()): item['sha256'] for item in resources}
    if len(resources) != len(by_path):
        raise ValueError('duplicate release resource paths')
    for item in resources:
        evidence.read_bound(item)
    for item in review['resources']:
        evidence.read_bound(item)
        if by_path.get(str(Path(item['path']).resolve())) != item['sha256']:
            raise ValueError('reviewed resource not bound by release')
    for path in (Path(__file__), LAUNCH, CHECKPOINT, Path(evidence.__file__)):
        if by_path.get(str(path.resolve())) != evidence.sha(path):
            raise ValueError('required runner/runtime resource missing')
    for unit in prepared['units']:
        if [Path(x['path']).name for x in unit['files']] != ['contract.json', 'robot_description.xml', 'fixture.launch']:
            raise ValueError('prepared unit files changed')
        value = json.loads(evidence.read_bound(unit['files'][0])); evidence.validate_contract(value)
        if (value['stage'], value['run_id'], value['seed'], value['method']) != (NAME, unit['run_id'], unit['seed'], unit['method']):
            raise ValueError('contract identity differs from schedule')
        for item in unit['files']:
            evidence.read_bound(item)
            if by_path.get(str(Path(item['path']).resolve())) != item['sha256']:
                raise ValueError('unit file not in release')
        if evidence.read_bound(unit['files'][2]).decode() != render_launch(value, unit['files'][0]['sha256'], STAGE/'runs'/unit['run_id']):
            raise ValueError('launch differs from reviewed generator')
    return release, prepared, review


def execute(release_path, release_sha):
    if Path(os.environ.get('THESIS_WS', '')).resolve() != WORKSPACE:
        raise ValueError('source isolated astra_env first')
    # Q03 replaces the overbroad Q02 mapped-file classifier. Never mutate Q02.
    import run_passive_sweep_q03 as passive_manager
    guard = (STAGE/'queue/simulation.lock').open('a')
    fcntl.flock(guard, fcntl.LOCK_EX | fcntl.LOCK_NB)
    release, prepared, review = verify_release(release_path, release_sha)
    ledger = STAGE/'queue/navigation_n01_ledger.json'
    if ledger.exists() or any((STAGE/'runs'/row[0]).exists() for row in identities()):
        raise ValueError('immutable identity already exists; retry prohibited')
    registry = list(csv.DictReader((STAGE/'seed_registry.csv').open()))
    for name, seed, _ in identities():
        own = [r for r in registry if r['run_id'] == name]
        if len(own) != 1 or own[0]['seed'] != str(seed) or own[0]['status'] != 'reserved_exact_n01_release' or own[0]['source_binding'] != release_sha:
            raise ValueError('exact root reservation missing or already consumed')
    state = dict(stage=NAME, status='active', release_sha256=release_sha, maximum_units=4,
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
                value = json.loads((root/'contract.json').read_bytes())
                common.atomic(root/'allocated.json', dict(state['units'][index], attempt=1, started_utc=common.utc()))
                for item in release['resources']:
                    passive_manager.archive(root, item)
                for name in ('prepared', 'source_review', 'passive_geometry_qualification'):
                    passive_manager.archive(root, release[name])
                verify_release(release_path, release_sha)
                for port in (14820, 14821):
                    with socket.socket() as sock:
                        sock.bind(('127.0.0.1', port))
                env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:14820', GAZEBO_MASTER_URI='http://127.0.0.1:14821',
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
                                passive_manager.capture_maps(root, root/marker, proc)
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
                        common.stop_group(proc, root)
                    except BaseException as exc:
                        error = (error or '')+'; cleanup '+repr(exc)
            try:
                if error or code != 0:
                    raise RuntimeError('process/preparation failure: '+str(error)+' exit='+str(code))
                evidence.closure(root, release, review, value)
                report = evidence.assess(root, value)
                verify_release(release_path, release_sha)
                if time.monotonic()-started > value['wall_cap_s']:
                    raise TimeoutError('complete attempt including assessment exceeded wall cap')
            except BaseException as exc:
                error = (error or '')+'; assessment '+repr(exc)
            integrity_pass = bool(not error and report and report.get('integrity_pass') is True)
            receipt = dict(stage=NAME, run_id=unit['run_id'], seed=unit['seed'], method=unit['method'], attempt=1,
                retry_allowed=False, integrity_pass=integrity_pass, task_outcome=None if report is None else report.get('task_outcome'),
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
    return 0 if len(receipts) == 4 and all(r['integrity_pass'] for r in receipts) else 3


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepare', type=Path); parser.add_argument('--execute', action='store_true')
    parser.add_argument('--release', type=Path); parser.add_argument('--release-sha256')
    args = parser.parse_args()
    if args.execute:
        if args.prepare or not args.release or not args.release_sha256:
            raise ValueError('execute requires only exact release path and SHA')
        return execute(args.release, args.release_sha256)
    if args.prepare:
        print(json.dumps(prepare(args.prepare), indent=2)); return 0
    print(json.dumps(dict(stage=NAME, execution_authorized=False, identities=identities())))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
