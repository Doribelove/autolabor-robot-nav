#!/usr/bin/env python3
"""Passive ROS capture, scoped process lifecycle and bag validation."""
import collections
import datetime as dt
import json
import math
import os
from pathlib import Path
import platform
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import uuid

from three_party_test import (WS, read_json, write_json, locked, clock_pair, stamp,
                              event, update_session, process_identity, alive,
                              append_line, sha256, iso_ns)


def scrub(value):
    if isinstance(value, dict):
        return {k: '[redacted]' if any(word in k.lower() for word in ('password', 'secret', 'access_token', 'api_key')) else scrub(v) for k, v in value.items()}
    if isinstance(value, list):
        return [scrub(v) for v in value]
    return value


def discover(config_path, output):
    import rospy
    import rosgraph
    import roslib.message
    config = read_json(config_path)
    if config.get('extends'):
        base = read_json(Path(config_path).parent / config['extends'])
        overrides = {entry['name']: entry for entry in config.get('topics', [])}
        topics = [dict(entry, **overrides.pop(entry['name'], {})) for entry in base['topics']]
        topics.extend(overrides.values())
        config = dict(base, **config)
        config['topics'] = topics
    master = rosgraph.Master('/three_party_discovery')
    pubs, subs, services = master.getSystemState()
    types = dict(master.getTopicTypes())
    publishers = dict(pubs)
    if not rospy.core.is_initialized():
        rospy.init_node('three_party_observer_' + uuid.uuid4().hex[:8], disable_signals=True)
    requested = [entry['name'] for entry in config['topics']]
    definitions = {}
    for name in requested:
        cls = roslib.message.get_message_class(types.get(name, '')) if types.get(name) else None
        if cls:
            definitions[name] = {'type': cls._type, 'md5': cls._md5sum, 'definition': cls._full_text}
    parameters = {p: scrub(rospy.get_param(p)) for p in config.get('parameter_roots', []) if rospy.has_param(p)}
    result = dict(config, actual_topic_types=types, actual_publishers=publishers, services=services,
                  requested_topics=requested, missing_topics=[t for t in requested if not publishers.get(t)],
                  observed=clock_pair(), use_sim_time=rospy.get_param('/use_sim_time', False),
                  actual_ros_master_uri=os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'),
                  actual_ros_distro=os.environ.get('ROS_DISTRO'), python=platform.python_version(),
                  kernel=platform.release(), machine=platform.machine(), disk_free_bytes=shutil.disk_usage(output).free,
                  type_changes=[{'name': e['name'], 'expected': e['observed_type'], 'actual': types.get(e['name'])}
                                for e in config['topics'] if e['observed_type'] and types.get(e['name']) and types[e['name']] != e['observed_type']])
    for command, key in [(['git', 'rev-parse', 'HEAD'], 'git_commit'), (['git', 'status', '--porcelain=v1'], 'git_status'),
                         (['git', 'submodule', 'status', '--recursive'], 'submodules'), (['timedatectl', 'show'], 'time_service')]:
        p = subprocess.run(command, cwd=str(WS), capture_output=True, text=True, timeout=10)
        result[key] = p.stdout.strip() or p.stderr.strip()
    files = ['scripts/record_rosbag.sh', 'scripts/three_party_test.py', 'scripts/three_party_capture.py',
             'scripts/three_party_report.py', 'src/application/autolabor_fod_control/scripts/fod_navigation_mode_manager.py',
             'src/application/autolabor_fod_control/scripts/fod_visual_servo_node.py',
             'src/application/autolabor_fod_control/src/autolabor_fod_control/recovery_verification.py',
             'src/application/gps_module/scripts/gps_localization_node.py',
             'src/application/autolabor_fod_control/config/visual_camera_zed2.yaml']
    result['software_sha256'] = {p: sha256(WS / p) for p in files if (WS / p).is_file()}
    output = Path(output)
    write_json(output / 'capture_config.json', result)
    write_json(output / 'parameters.json', parameters)
    write_json(output / 'message_definitions.json', definitions)
    (output / 'topics.txt').write_text('\n'.join(requested) + '\n')
    return result


class EventTracker:
    """Ignore initial latched states; only a new transition starts a task."""
    def __init__(self, directory):
        self.directory = Path(directory)
        self.previous = {}
        self.previous_pair = {}

    def observe(self, topic, data, pair):
        previous = self.previous.get(topic)
        old_pair = self.previous_pair.get(topic)
        self.previous[topic], self.previous_pair[topic] = data, pair
        fields = ('state', 'phase', 'result', 'recovery_armed', 'sweep_status', 'reason', 'route_id', 'target_track_id')
        changed = previous is None or any(data.get(k) != previous.get(k) for k in fields)
        if changed:
            event(self.directory, 'initial_state' if previous is None else 'state_transition', topic, data, pair)
        if previous is None:
            return
        def apply(s):
            if topic == '/fod_navigation_mode/status':
                fresh_arm = not previous.get('recovery_armed') and data.get('recovery_armed')
                direct_start = previous.get('state') == 'GPS_ACTIVE' and data.get('state') == 'ENTERING_FOD' and not data.get('retain_gps_route', True)
                if s['kind'] in ('recovery', 'verification') and (fresh_arm or direct_start):
                    at = stamp(pair, topic, method='observed successful module activation transition; receipt time, no source header',
                               bracket_start_ns=old_pair['system_ns'], bracket_end_ns=pair['system_ns'])
                    if s['task_started_at'] is None:
                        s['task_started_at'] = at
                        event(self.directory, 'task_started', topic, data, pair, at)
                    else:
                        s['anomalies'].append({'kind': 'multiple_module_starts', 'at': pair})
                        event(self.directory, 'additional_module_start', topic, data, pair)
                if data.get('state') == 'FOD_COMPLETE_STOP' and previous.get('state') != 'FOD_COMPLETE_STOP':
                    s['software_completed_at'] = stamp(pair, topic, meaning='crossing/navigation finished; physical pickup unconfirmed')
                    event(self.directory, 'software_completed', topic, data, pair)
                if changed and (data.get('state') in ('FOD_ABORTED', 'FAULT_STOP') or '接管' in data.get('reason', '')):
                    s['anomalies'].append({'kind': 'controller_abort_or_handover', 'at': pair, 'status': data})
            if topic == '/gps/long_range/status' and data.get('route_id'):
                if data['route_id'] not in s['software_task_ids']:
                    s['software_task_ids'].append(data['route_id'])
                if (s['kind'] == 'obstacle' and data.get('active') and not data.get('paused')
                        and (not previous.get('active') or data['route_id'] != previous.get('route_id'))):
                    at = stamp(pair, topic, method='observed new navigation route activation; not first physical motion',
                               bracket_start_ns=old_pair['system_ns'], bracket_end_ns=pair['system_ns'])
                    if s['task_started_at'] is None:
                        s['task_started_at'] = at
                        event(self.directory, 'obstacle_navigation_started', topic, data, pair, at)
                    else:
                        event(self.directory, 'additional_navigation_route', topic, data, pair)
            if topic == '/fod_navigation_mode/verification/status' and changed:
                event(self.directory, 'verification_transition', topic, data, pair)
                s['verification'] = data
                if data.get('phase') == 'PREPARING':
                    s['verification_started_at'] = stamp(pair, topic, method='observed reinspection transition')
                    s['software_completed_at'] = stamp(pair, topic, meaning='original recovery travel ended; reinspection pending')
                    s['original_recovery_finished_at'] = s['software_completed_at']
                if data.get('result'):
                    s['verification_completed_at'] = stamp(pair, topic, method='observed reinspection result')
                    # No optical result is promoted to physical recovery_success or its timestamp.
                    event(self.directory, 'verification_result', topic, data, pair)
        update_session(self.directory, apply)


def scan_bags(paths, live=False):
    import rosbag
    result = {'readable': True, 'files': [], 'counts': {}, 'types': {}, 'topic_ranges_ns': {}, 'first_ros_ns': None, 'last_ros_ns': None, 'errors': []}
    counts = collections.Counter()
    for original in paths:
        temporary = None
        path = original
        try:
            if live:
                temporary = tempfile.TemporaryDirectory(prefix='tri_bag_probe_')
                path = Path(temporary.name) / 'probe.bag'
                # A bounded copy cannot chase a continuously growing file.
                remaining = min(original.stat().st_size, 32 * 1024 * 1024)
                result['live_prefix_limit_bytes'] = 32 * 1024 * 1024
                with original.open('rb') as source, path.open('wb') as target:
                    while remaining:
                        block = source.read(min(1024 * 1024, remaining))
                        if not block: break
                        target.write(block); remaining -= len(block)
            if live:
                # Reindex only the disposable bounded copy. rosbag's reindex
                # truncates an unfinished chunk and requires a writable file.
                with rosbag.Bag(str(path), 'a', allow_unindexed=True) as probe:
                    try:
                        for _ in probe.reindex(): pass
                    except Exception as exc:
                        result.setdefault('live_tail_notes', []).append(str(exc))
            with rosbag.Bag(str(path)) as bag:
                file_count = 0
                for topic, raw, t in bag.read_messages(raw=True):
                    counts[topic] += 1
                    file_count += 1
                    result['types'][topic] = raw[0]
                    ns = t.to_nsec()
                    bounds = result['topic_ranges_ns'].setdefault(topic, [ns, ns])
                    bounds[0], bounds[1] = min(bounds[0], ns), max(bounds[1], ns)
                    result['first_ros_ns'] = ns if result['first_ros_ns'] is None else min(ns, result['first_ros_ns'])
                    result['last_ros_ns'] = ns if result['last_ros_ns'] is None else max(ns, result['last_ros_ns'])
                result['files'].append({'path': str(original), 'messages': file_count, 'bytes': original.stat().st_size})
        except Exception as exc:
            result['readable'] = False
            result['errors'].append({'path': str(original), 'error': str(exc)})
        finally:
            if temporary: temporary.cleanup()
    result['counts'] = dict(counts)
    if not paths:
        result['readable'] = False
        result['errors'].append({'error': 'no bag files'})
    return result


def critical_topics(config, kind):
    # Camera evidence is also needed for obstacle tests; '/fod_camera' is not a
    # FOD controller channel. Event-only goals/plans stay optional at startup.
    return [e['name'] for e in config['topics'] if e['critical'] and
            (kind in ('recovery', 'verification') or not e['name'].startswith(('/fod/', '/fod_navigation_mode/', '/fod_visual_servo/')))]


def startup_check(directory, config):
    directory = Path(directory)
    first = sum(p.stat().st_size for p in (directory / 'bags').glob('*.bag*'))
    time.sleep(1)
    second = sum(p.stat().st_size for p in (directory / 'bags').glob('*.bag*'))
    stats = scan_bags(sorted((directory / 'bags').glob('*.bag*')), live=True)
    s = read_json(directory / 'session.json')
    missing = [t for t in critical_topics(config, s['kind']) if not stats['counts'].get(t)]
    recorder_alive = alive(read_json(directory / 'recorder_process.json'))
    confirmed = recorder_alive and second > first and stats['readable'] and not missing
    result = {'confirmed': confirmed, 'recorder_alive': recorder_alive, 'file_grew': second > first,
              'bytes_before': first, 'bytes_after': second, 'missing_critical_topics': missing, 'bag_probe': stats, 'checked_at': clock_pair()}
    write_json(directory / 'startup_check.json', result)
    if stats['first_ros_ns'] is not None:
        pair = result['checked_at']
        def apply(s):
            if s['recording_started_at'] is None:
                ns = pair['system_ns'] + stats['first_ros_ns'] - pair['ros_ns']
                s['recording_started_at'] = {'epoch_ns': ns, 'iso': iso_ns(ns), 'ros_ns': stats['first_ros_ns'],
                                              'source': 'first message actually stored in rosbag', 'clock_mapping': pair}
                event(directory, 'recording_started', 'rosbag contents verified', result, actual=s['recording_started_at'])
            if s['phase'] == 'preparing': s['phase'] = 'recording'
        update_session(directory, apply)
    return result


def log_sources(directory):
    pointer = Path('/tmp/agx_airport_current_log')
    sources = []
    if pointer.exists():
        folder = Path(pointer.read_text().strip())
        for name in ('bringup.log', 'gui.log', 'vision.log', 'zed.log', 'launcher.log'):
            p = folder / name
            if p.is_file():
                sources.append({'path': str(p), 'start_offset': p.stat().st_size, 'offset': p.stat().st_size, 'output': name})
    write_json(Path(directory) / 'source_logs.json', sources)
    return sources


def tail_logs(directory, sources):
    for source in sources:
        p = Path(source['path'])
        try:
            if p.stat().st_size < source['offset']:
                event(directory, 'source_log_truncated', 'file observation', source)
                source['offset'] = 0
            with p.open('rb') as f:
                f.seek(source['offset'])
                data = f.read(1024 * 1024)
                source['offset'] = f.tell()
            if data:
                with (Path(directory) / 'logs' / source['output']).open('ab') as f: f.write(data)
        except OSError as exc:
            source['error'] = str(exc)


def collect(directory, config_path):
    import rospy
    import roslib.message
    import genpy
    directory = Path(directory)
    for child in ('bags', 'logs'): (directory / child).mkdir(exist_ok=True)
    config = discover(config_path, directory)
    sources = log_sources(directory)
    # Missing channels never become navigation gates. They are data-quality findings.
    if config['disk_free_bytes'] < 256 * 1024**2:
        raise RuntimeError('Insufficient space to start a recorder (less than 256 MiB)')
    env = os.environ.copy()
    env.update(BAG_DIR=str(directory / 'bags'), BAG_PREFIX='capture', BAG_TOPICS_FILE=str(directory / 'topics.txt'),
               COMPRESSION=config['compression'], SPLIT_SIZE_MB=str(config['split_size_mb']),
               BAG_BUFFER_MB=str(config['buffer_mb']), BAG_NODE_NAME='tri_bag_' + uuid.uuid4().hex[:10])
    with (directory / 'recorder.log').open('ab') as log:
        proc = subprocess.Popen(['bash', str(WS / 'scripts/record_rosbag.sh'), 'custom'], cwd=str(WS), env=env,
                                stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    write_json(directory / 'recorder_process.json', process_identity(proc.pid))
    event(directory, 'recorder_spawned', 'collector', {'pid': proc.pid, 'command': 'scripts/record_rosbag.sh custom'})
    tracker = EventTracker(directory)
    listeners = []
    event_topics = ['/fod_navigation_mode/status', '/fod_visual_servo/status', '/gps/long_range/status',
                    '/fod_navigation_mode/verification/status',
                    '/gps/goal_fix', '/move_base_simple/goal', '/move_base/goal', '/rabbitmq_bridge/latest_target', '/rosout']
    def callback(msg, topic):
        pair = clock_pair()
        try:
            if topic.endswith('/status') and hasattr(msg, 'data'):
                tracker.observe(topic, json.loads(msg.data), pair)
            elif topic == '/rosout':
                if msg.level >= 4 or ('heading' in msg.msg.lower() and 'baseline' in msg.msg.lower()):
                    event(directory, 'log_message', topic, {'node': msg.name, 'level': msg.level, 'message': msg.msg,
                                                           'source_ros_ns': msg.header.stamp.to_nsec()}, pair)
            else:
                event(directory, 'goal_or_remote_request_observed', topic,
                      {'ros_message_type': msg._type, 'body_yaml': str(msg),
                       'first_observation_may_be_latched': topic not in tracker.previous,
                       'source_ros_ns': msg.header.stamp.to_nsec() if hasattr(msg, 'header') else None}, pair)
                tracker.previous[topic] = {}
        except Exception as exc:
            event(directory, 'observer_parse_error', topic, {'error': str(exc)}, pair)
    for topic in event_topics:
        typ = config['actual_topic_types'].get(topic)
        cls = roslib.message.get_message_class(typ) if typ else None
        if cls:
            listeners.append(rospy.Subscriber(topic, cls, callback, callback_args=topic, queue_size=100))
    interrupted = []
    signal.signal(signal.SIGTERM, lambda sig, frame: interrupted.append(sig))
    signal.signal(signal.SIGINT, lambda sig, frame: interrupted.append(sig))
    last_pair = None
    last_check = 0
    last_heartbeat = 0
    while True:
        now = time.monotonic()
        if now - last_heartbeat >= 1:
            pair = clock_pair()
            append_line(directory / 'clock_pairs.jsonl', pair)
            if last_pair:
                wall_delta = pair['system_ns'] - last_pair['system_ns']
                ros_delta = pair['ros_ns'] - last_pair['ros_ns']
                mono_delta = pair['monotonic_ns'] - last_pair['monotonic_ns']
                if abs(wall_delta - mono_delta) > 100000000 or abs(ros_delta - wall_delta) > 100000000:
                    event(directory, 'clock_discontinuity', 'system/ROS/monotonic comparison', {'before': last_pair, 'after': pair})
                    update_session(directory, lambda s: s['anomalies'].append({'kind': 'clock_discontinuity', 'observed': pair}))
            last_pair, last_heartbeat = pair, now
            tail_logs(directory, sources)
        request = read_json(directory / 'stop_request.json')
        if interrupted and request is None:
            request = {'disposition': 'abort', 'reason': 'collector received signal; no vehicle command sent'}
            write_json(directory / 'stop_request.json', request)
        if request:
            for listener in listeners: listener.unregister()
            tail_logs(directory, sources)
            write_json(directory / 'source_logs.json', sources)
            if finish(directory, request):
                break
            time.sleep(1)
            continue
        if proc.poll() is not None:
            update_session(directory, lambda s: s.update(phase='interrupted'))
            event(directory, 'recorder_unexpected_exit', 'process observation', {'exit_code': proc.returncode})
            from three_party_report import report
            report(directory.parent.parent)
            break
        if (not read_json(directory / 'startup_check.json', {}).get('confirmed')) and now - last_check >= 3:
            startup_check(directory, config)
            last_check = time.monotonic()
        time.sleep(.1)
    rospy.signal_shutdown('test observer finished')


def stop_owned_recorder(directory):
    directory = Path(directory)
    identity = read_json(directory / 'recorder_process.json')
    if alive(identity) and not (directory / 'stop_signal_sent.json').exists():
        if identity['pgid'] != identity['pid']:
            raise RuntimeError('Recorder is not an isolated process group; refusing to signal other processes')
        os.killpg(identity['pid'], signal.SIGINT)
        write_json(directory / 'stop_signal_sent.json', {'identity': identity, 'sent_at': clock_pair()})
    deadline = time.monotonic() + 20
    while alive(identity) and time.monotonic() < deadline:
        time.sleep(.1)
    return not alive(identity)


def finish(directory, request):
    directory = Path(directory)
    update_session(directory, lambda s: s.update(phase='stopping'))
    if not stop_owned_recorder(directory):
        return False
    pair = clock_pair()
    update_session(directory, lambda s: s.update(recording_stopped_at=stamp(pair, 'owned recorder exit observed'),
                                                phase='completed' if request['disposition'] == 'complete' else 'aborted',
                                                termination=request))
    event(directory, 'recording_stopped', 'owned recorder exit observed', request, pair)
    check_session(directory)
    root = directory.parent.parent
    with locked(root / 'campaign.lock'):
        active = read_json(root / 'active.json', {})
        if active.get('session_id') == directory.name:
            write_json(root / 'active.json', {'session_id': None})
    from three_party_report import report
    summary = report(root)
    write_json(directory / 'finalized.json', {'finished_at': clock_pair(), 'report_directory': summary['report_directory']})
    return True


def check_session(directory):
    import rosbag
    directory = Path(directory)
    s = read_json(directory / 'session.json')
    config = read_json(directory / 'capture_config.json', {'topics': [], 'requested_topics': []})
    if alive(read_json(directory / 'recorder_process.json')):
        result = startup_check(directory, config)
        return dict(result, still_recording=True)
    paths = sorted((directory / 'bags').glob('*.bag'))
    stats = scan_bags(paths)
    stats['active_files'] = [p.name for p in (directory / 'bags').glob('*.active')]
    stats['missing_topics'] = [t for t in config['requested_topics'] if not stats['counts'].get(t)]
    stats['missing_critical_topics'] = [t for t in critical_topics(config, s['kind']) if not stats['counts'].get(t)]
    stats['task_time_covered'] = None
    start_at, end_at = s.get('task_started_at'), s.get('recovery_completed_at')
    if s['kind'] == 'obstacle':
        end_at = s.get('obstacle', {}).get('completed_at') or end_at
    first, last = stats['first_ros_ns'], stats['last_ros_ns']
    pair = (s.get('recording_started_at') or {}).get('clock_mapping')
    if s.get('kind') == 'verification':
        a, b = s.get('verification_started_at'), s.get('verification_completed_at')
        stats['verification_time_covered'] = None
        if a and b and pair and first is not None:
            a_ros = a['epoch_ns'] - pair['system_ns'] + pair['ros_ns']
            b_ros = b['epoch_ns'] - pair['system_ns'] + pair['ros_ns']
            stats['verification_time_covered'] = first <= a_ros <= b_ros <= last
            stats['verification_critical_time_coverage'] = {topic: bool(stats['topic_ranges_ns'].get(topic) and
                stats['topic_ranges_ns'][topic][0] <= a_ros <= b_ros <= stats['topic_ranges_ns'][topic][1])
                for topic in critical_topics(config, s['kind'])}
    if start_at and end_at and pair and first is not None:
        start_ros = start_at['epoch_ns'] - pair['system_ns'] + pair['ros_ns']
        end_ros = end_at['epoch_ns'] - pair['system_ns'] + pair['ros_ns']
        stats['task_time_covered'] = first <= start_ros <= end_ros <= last
        stats['task_ros_range_ns'] = [start_ros, end_ros]
        stats['critical_time_coverage'] = {topic: bool(stats['topic_ranges_ns'].get(topic) and
            stats['topic_ranges_ns'][topic][0] <= start_ros <= end_ros <= stats['topic_ranges_ns'][topic][1])
            for topic in critical_topics(config, s['kind'])}
    if s['kind'] == 'obstacle' and s.get('obstacle', {}).get('encounter_at') and pair:
        encounter = s['obstacle']['encounter_at']['epoch_ns'] - pair['system_ns'] + pair['ros_ns']
        values = []
        for path in paths:
            try:
                with rosbag.Bag(str(path)) as bag:
                    for _, msg, t in bag.read_messages(topics=['/odom']):
                        if encounter - 2 * 10**9 <= t.to_nsec() < encounter:
                            speed = abs(msg.twist.twist.linear.x)
                            if math.isfinite(speed):
                                values.append((t.to_nsec(), speed))
            except Exception as exc:
                stats['readable'] = False
                stats['errors'].append({'path': str(path), 'error': 'obstacle speed read: ' + str(exc)})
        values.sort()
        stats['pre_obstacle_actual_speed'] = {'source': '/odom.twist.twist.linear.x', 'window_sec': 2,
                                              'encounter_ros_ns': encounter, 'time_basis': 'bag receive ROS timestamp',
                                              'samples': len(values), 'mean_mps': sum(v for _, v in values) / len(values) if values else None,
                                              'max_mps': max(v for _, v in values) if values else None,
                                              'last_before_mps': values[-1][1] if values else None,
                                              'last_sample_ros_ns': values[-1][0] if values else None}
    stats['recording_integrity_ok'] = stats['readable'] and not stats['active_files'] and not stats['missing_critical_topics']
    stats['known_data_limits'] = config.get('known_data_limits', [])
    stats['checked_at'] = clock_pair()
    write_json(directory / 'check.json', stats)
    inventory = []
    for path in sorted(directory.rglob('*')):
        if path.is_file() and path.name not in ('files.json', 'session.lock', 'events.lock'):
            inventory.append({'path': str(path.relative_to(directory)), 'bytes': path.stat().st_size, 'sha256': sha256(path)})
    write_json(directory / 'files.json', {'created_at': clock_pair(), 'files': inventory})
    return stats


if __name__ == '__main__':
    try:
        if len(sys.argv) != 4 or sys.argv[1] != 'collect': raise ValueError('internal collector invocation only')
        collect(Path(sys.argv[2]), Path(sys.argv[3]))
    except Exception as exc:
        if len(sys.argv) >= 3 and (Path(sys.argv[2]) / 'session.json').exists():
            directory = Path(sys.argv[2])
            event(directory, 'collector_error', 'collector', {'error': str(exc)})
            stop_owned_recorder(directory)
            update_session(directory, lambda s: s.update(phase='start_failed' if s['recording_started_at'] is None else 'interrupted'))
        raise
