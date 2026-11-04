#!/usr/bin/env python3
"""Three-party field-test records. This program never commands the vehicle."""
import argparse
import contextlib
import datetime as dt
from decimal import Decimal, InvalidOperation
import fcntl
from fractions import Fraction
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import time
import uuid

WS = Path(__file__).resolve().parents[1]
DEFAULT_ROOT = WS / 'log/three_party_tests'
CONFIG = WS / 'config/three_party_capture.json'
LIGHTS = ('黎明', '白天', '黄昏', '夜晚')
# Keep known point labels attached to their original IDs; never shift pt0
# metadata onto pt1 when the operator changes the accepted numbering.
SCENES = {'机坪': ('18743.63', ('左上角', '右下角', '待补', '待补', '待补', '待补')),
          '滑行道': ('22881.75', ('中端', '远端', '待补', '待补', '待补', '待补'))}
POINTS = ('pt1', 'pt2', 'pt3', 'pt4', 'pt5', 'pt6')
TEST_PLAN = {'version': '2026-10-14_pt1-pt6', 'points': list(POINTS),
             'recovery_slots_total': len(LIGHTS) * len(SCENES) * len(POINTS),
             'group_mean_divisor': len(POINTS), 'source': '用户更正：1-6 共 6 个 pt 点'}
OPEN_PHASES = ('preparing', 'recording', 'stopping', 'interrupted')


def read_json(path, default=None):
    return json.loads(Path(path).read_text()) if Path(path).exists() else default


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile('w', dir=str(path.parent), delete=False) as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n')
        f.flush()
        os.fsync(f.fileno())
        temporary = f.name
    os.replace(temporary, str(path))


@contextlib.contextmanager
def locked(path):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'a') as f:
        fcntl.flock(f, fcntl.LOCK_EX)
        yield


def iso_ns(ns):
    seconds, nano = divmod(int(ns), 10**9)
    date = dt.datetime.fromtimestamp(seconds, dt.timezone.utc).astimezone()
    return date.strftime('%Y-%m-%dT%H:%M:%S') + '.%09d' % nano + date.strftime('%z')[:3] + ':' + date.strftime('%z')[3:]


def parse_time(text):
    match = re.fullmatch(r'(\d{4}-\d\d-\d\d[T ]\d\d:\d\d:\d\d)(?:\.(\d{1,9}))?(Z|[+-]\d\d:\d\d)', text)
    if not match:
        raise ValueError('实际时刻须含日期、时分秒和时区，可保留 9 位小数；不能只写聊天收到时间')
    date = dt.datetime.fromisoformat(match[1] + ('+00:00' if match[3] == 'Z' else match[3]))
    return int(date.timestamp()) * 10**9 + int((match[2] or '').ljust(9, '0') or '0')


def clock_pair():
    before = time.time_ns()
    ros_ns = None
    try:
        import rospy
        if rospy.core.is_initialized():
            ros_ns = rospy.Time.now().to_nsec()
    except ImportError:
        pass
    after = time.time_ns()
    return {'system_ns': (before + after) // 2, 'system_iso': iso_ns((before + after) // 2),
            'ros_ns': ros_ns, 'monotonic_ns': time.monotonic_ns(), 'pair_read_span_ns': after - before}


def stamp(pair, source, **extra):
    return dict(epoch_ns=pair['system_ns'], iso=iso_ns(pair['system_ns']), ros_ns=pair.get('ros_ns'), source=source, **extra)


def append_line(path, value):
    data = (json.dumps(value, ensure_ascii=False, allow_nan=False) + '\n').encode()
    with open(path, 'ab', buffering=0) as f:
        f.write(data)


def update_session(directory, callback):
    directory = Path(directory)
    with locked(directory / 'session.lock'):
        value = read_json(directory / 'session.json')
        result = callback(value)
        write_json(directory / 'session.json', value)
        return result


def event(directory, kind, source, data=None, pair=None, actual=None):
    directory = Path(directory)
    value = {'id': uuid.uuid4().hex, 'kind': kind, 'source': source,
             'observed': pair or clock_pair(), 'actual': actual, 'data': data or {}}
    with locked(directory / 'events.lock'):
        append_line(directory / 'events.jsonl', value)
    return value


def parse_command(text):
    compact = re.sub(r'\s+', '', text.strip())
    if compact in ('垃圾回收判定开始', '垃圾回收判定测试开始', '测试垃圾回收判定', '重测垃圾回收判定'):
        return dict(action='start', kind='verification', light=None, scene=None, point=None,
                    retest=compact.startswith('重测'), missing=[])
    obstacle = re.fullmatch(r'(重测)?避障测试(黎明|白天|黄昏|夜晚)?(机坪|滑行道)?(?:开始)?', compact)
    if obstacle:
        return dict(action='start', kind='obstacle', light=obstacle[2], scene=obstacle[3],
                    point='obstacle', retest=bool(obstacle[1]), missing=[])
    if compact in ('避障结束', '避障测试结束', '避障中止', '避障测试中止'):
        return dict(action='abort' if compact.endswith('中止') else 'complete', expected_kind='obstacle')
    simple = {'任务开始': 'task_started', '回收完成': 'complete', '测试中止': 'abort',
              '测试进度': 'progress', '汇总报告': 'report'}
    if compact in simple:
        return {'action': simple[compact]}
    match = re.fullmatch(r'(测试|重测)(黎明|白天|黄昏|夜晚)?(机坪|滑行道)?(pt[1-6])?(?:点)?', compact)
    if not match:
        return {'action': 'not_a_command', 'reason': '仅接受当前用户的完整明确口令；引用、示例、讨论和非法值不触发'}
    values = dict(zip(('light', 'scene', 'point'), match.groups()[1:]))
    missing = [k for k, v in values.items() if v is None]
    return dict(action='missing' if missing else 'start', retest=match[1] == '重测', missing=missing, **values)


def coordinate(text, axis):
    original = str(text).strip()
    match = re.fullmatch(r'\s*([+-]?\d+(?:\.\d+)?)\s*(?:[°度: ]\s*(\d+(?:\.\d+)?)\s*(?:[′分\x27: ]\s*(\d+(?:\.\d+)?)\s*[″秒\x22]?)?)?\s*([NSEW北南东西])?\s*', original, re.I)
    if not match:
        raise ValueError('无法解析坐标；使用十进制度或例如 104°23′21.5″E')
    degree, minute, second = (Decimal(v or '0') for v in match.groups()[:3])
    if not all(v.is_finite() for v in (degree, minute, second)) or not (0 <= minute < 60 and 0 <= second < 60):
        raise ValueError('分、秒必须在 [0,60) 内')
    hemisphere = (match[4] or '').upper()
    if hemisphere and hemisphere not in (('E', 'W', '东', '西') if axis == 'longitude' else ('N', 'S', '北', '南')):
        raise ValueError('经纬度方向不匹配')
    sign = -1 if original.startswith('-') or hemisphere in ('W', 'S', '西', '南') else 1
    if original.startswith('-') and hemisphere in ('E', 'N', '东', '北'):
        raise ValueError('坐标符号与方向冲突')
    value = sign * (abs(degree) + minute / 60 + second / 3600)
    if abs(value) > (180 if axis == 'longitude' else 90):
        raise ValueError('坐标越界')
    return {'raw': original, 'decimal_degrees': str(value), 'unit': 'degree'}


def session_dirs(root):
    return sorted(p.parent for p in (Path(root) / 'sessions').glob('*/session.json'))


def active_dir(root):
    active = read_json(Path(root) / 'active.json', {})
    if not active.get('session_id'):
        return None
    return resolve_session(root, active['session_id'])


def resolve_session(root, name=None):
    if not name:
        directory = active_dir(root)
        if directory is None:
            raise ValueError('当前没有正在采集的测试；补录旧记录请明确 --session 编号')
        return directory
    if Path(name).name != name:
        raise ValueError('必须使用测试编号，不接受目录跳转')
    directory = Path(root) / 'sessions' / name
    if not (directory / 'session.json').is_file():
        raise ValueError('测试编号不存在：' + name)
    return directory


def new_session(root, light, scene, point, kind='recovery', retest=False, raw_command=''):
    demo = kind == 'verification'
    obstacle = kind == 'obstacle'
    if obstacle and (light not in (None, *LIGHTS) or scene not in (None, *SCENES)):
        raise ValueError('非法避障光照或场景；未知信息须留空待补')
    if not demo and not obstacle and (light not in LIGHTS or scene not in SCENES or point not in POINTS):
        raise ValueError('非法光照、场景或点位')
    prior = [read_json(p / 'session.json') for p in session_dirs(root)]
    prior = sorted([s for s in prior if (s['light'], s['scene'], s['point'], s['kind']) == (light, scene, point, kind)], key=lambda s: s['attempt'])
    attempt = 1 + max([s['attempt'] for s in prior] or [0])
    date = dt.datetime.now().astimezone()
    sid = '{}_{}_{}_{}_a{:02d}_{}_{}'.format(date.strftime('%Y%m%d'), light, scene, point, attempt, date.strftime('%H%M%S'), uuid.uuid4().hex[:6])
    if demo:
        sid = '{}_垃圾回收判定_a{:02d}_{}_{}'.format(date.strftime('%Y%m%d'), attempt, date.strftime('%H%M%S'), uuid.uuid4().hex[:6])
    elif obstacle:
        sid = '{}_{}_{}_避障_a{:02d}_{}_{}'.format(date.strftime('%Y%m%d'), light or '光照待补', scene or '场景待补', attempt, date.strftime('%H%M%S'), uuid.uuid4().hex[:6])
    directory = Path(root) / 'sessions' / sid
    directory.mkdir(parents=True, exist_ok=False)
    value = dict(schema_version=1, test_plan=TEST_PLAN, session_id=sid, kind=kind, light=light, scene=scene, point=point,
                 point_description=('垃圾回收判定功能演示；光照、场景、点位待补' if demo else
                                    SCENES[scene][1][POINTS.index(point)] if kind == 'recovery' else '障碍物位置待补'),
                 area_m2=None if demo or obstacle else SCENES[scene][0], attempt=attempt, retest_of=prior[-1]['session_id'] if prior else None,
                 retest_requested=retest, raw_command=raw_command, created_at=stamp(clock_pair(), 'record creation'),
                 phase='preparing', coordinates=None, illuminance=None, personnel=None, fod=None,
                 recovery_success=None, steps_conform=None, recording_started_at=None, task_started_at=None,
                 recovery_completed_at=None, recording_stopped_at=None, software_completed_at=None,
                 evidence=[], obstacle=({'evidence_schema': 'report_20261017'} if obstacle else {}),
                 anomalies=[], software_task_ids=[], missing_information=[])
    write_json(directory / 'session.json', value)
    event(directory, 'session_created', 'explicit operator test request', {'raw_command': raw_command, 'retest_of': value['retest_of']})
    return directory


def process_identity(pid):
    try:
        stat = Path('/proc/{}/stat'.format(pid)).read_text().rsplit(')', 1)[1].split()
        if stat[0] == 'Z':
            return None
        return {'pid': pid, 'start_ticks': stat[19], 'boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                'pgid': os.getpgid(pid)}
    except (OSError, ValueError, IndexError):
        return None


def alive(identity):
    return bool(identity and process_identity(identity['pid']) == identity)


def start(root, parsed, config=CONFIG, kind='recovery'):
    root = Path(root)
    kind = parsed.get('kind', kind)
    if kind == 'verification' and Path(config) == CONFIG:
        config = WS / 'config/recovery_verification_capture.json'
    elif kind == 'obstacle' and Path(config) == CONFIG:
        config = WS / 'config/obstacle_capture.json'
    with locked(root / 'campaign.lock'):
        current = active_dir(root)
        if current:
            state = read_json(current / 'session.json')
            if state['phase'] in OPEN_PHASES:
                same = (state['light'], state['scene'], state['point'], state['kind']) == (parsed['light'], parsed['scene'], parsed['point'], kind)
                if kind == state['kind'] == 'obstacle':
                    same = all(parsed.get(k) is None or parsed[k] == state[k] for k in ('light', 'scene'))
                return {'result': 'already_recording' if same and not parsed.get('retest') else 'conflict',
                        'session_id': state['session_id'], 'phase': state['phase'],
                        'message': '返回现有采集状态，不重复启动' if same and not parsed.get('retest') else '当前测试尚未结束，请明确继续当前测试，或先结束/中止后再开始新测试',
                        'startup_check': read_json(current / 'startup_check.json')}
        directory = new_session(root, parsed['light'], parsed['scene'], parsed['point'], kind, parsed.get('retest', False), parsed.get('raw_command', ''))
        write_json(root / 'active.json', {'session_id': directory.name})
        with (directory / 'collector.log').open('ab') as log:
            proc = subprocess.Popen([sys.executable, str(WS / 'scripts/three_party_capture.py'), 'collect', str(directory), str(Path(config).resolve())],
                                    cwd=str(WS), stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        write_json(directory / 'collector_process.json', process_identity(proc.pid))
    deadline = time.monotonic() + 18
    while time.monotonic() < deadline and proc.poll() is None:
        verified = read_json(directory / 'startup_check.json')
        if verified and verified['confirmed']:
            return {'result': 'recording', 'session_id': directory.name, 'message': '已开始录制', 'check': verified}
        time.sleep(.2)
    return {'result': 'awaiting_data' if proc.poll() is None else 'start_failed', 'session_id': directory.name,
            'message': '尚未确认录制成功，请检查本次 collector.log 和 recorder.log', 'collector_alive': proc.poll() is None,
            'check': read_json(directory / 'startup_check.json')}


def manual_event(directory, kind, source, raw='', actual_time=None, details=None):
    if not source.strip():
        raise ValueError('事件必须记录来源')
    actual = None
    if actual_time:
        ns = parse_time(actual_time)
        actual = {'epoch_ns': ns, 'iso': iso_ns(ns), 'ros_ns': None, 'source': source, 'raw': actual_time,
                  'method': 'operator supplied actual time or timestamped evidence'}
    item = event(directory, kind + ('_actual' if actual else '_reported'), source, dict(raw_input=raw, **(details or {})), actual=actual)
    def apply(s):
        if kind == 'recovery_completed' and s['kind'] in ('recovery', 'verification'):
            s['recovery_success'] = True
        if kind in ('task_started', 'recovery_completed') and actual:
            previous = s[kind + '_at']
            if previous:
                event(directory, 'timestamp_revision', source, {'field': kind + '_at', 'previous': previous, 'replacement': actual})
            s[kind + '_at'] = actual
        if kind == 'obstacle_encounter' and actual:
            s['obstacle']['encounter_at'] = actual
        if kind == 'obstacle_completed' and actual:
            s['obstacle']['completed_at'] = actual
        if kind in ('abort', 'takeover'):
            s['anomalies'].append({'kind': kind, 'source': source, 'raw': raw, 'event_id': item['id']})
    update_session(directory, apply)
    return item


def stop(root, directory, disposition, reason='', source='operator command', actual_time=None):
    state = read_json(directory / 'session.json')
    if state['phase'] not in OPEN_PHASES:
        return {'result': 'already_stopped', 'session_id': directory.name, 'phase': state['phase']}
    completion = 'obstacle_completed' if state['kind'] == 'obstacle' else 'recovery_completed'
    manual_event(directory, completion if disposition == 'complete' else 'abort', source, reason, actual_time)
    request = {'disposition': disposition, 'reason': reason or ('待补' if disposition == 'abort' else ''), 'reported': clock_pair()}
    write_json(directory / 'stop_request.json', request)
    if not alive(read_json(directory / 'collector_process.json')):
        from three_party_capture import finish
        finish(directory, request)
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        state = read_json(directory / 'session.json')
        if state['phase'] not in OPEN_PHASES and (directory / 'finalized.json').exists():
            return {'result': 'stopped', 'session_id': directory.name, 'phase': state['phase'], 'check': read_json(directory / 'check.json')}
        time.sleep(.25)
    return {'result': 'finalizing', 'session_id': directory.name, 'message': '仍在正常封口或检查数据，未强制终止其他进程；请用 check 查看'}


def set_info(directory, fields, source):
    allowed = {'coordinates', 'illuminance', 'personnel', 'fod', 'recovery_success', 'steps_conform', 'point_description', 'obstacle', 'notes', 'gnss_quality', 'light', 'scene'}
    if set(fields) - allowed or not source.strip():
        raise ValueError('信息字段非法或缺少来源')
    fields = dict(fields)
    if {'light', 'scene'} & fields.keys():
        if read_json(Path(directory) / 'session.json')['kind'] not in ('obstacle', 'verification'):
            raise ValueError('普通回收的光照和场景由启动口令固定，不能在此更改')
        for key, choices in (('light', LIGHTS), ('scene', SCENES)):
            if key in fields and fields[key] not in choices:
                raise ValueError('非法' + key)
    if 'coordinates' in fields:
        c = fields['coordinates']
        fields['coordinates'] = {'longitude': coordinate(c['longitude'], 'longitude'), 'latitude': coordinate(c['latitude'], 'latitude'),
                                 'crs': c.get('crs'), 'unit': 'degree', 'source': source, 'raw_input': c.get('raw_input', c)}
    if 'illuminance' in fields:
        value = Decimal(str(fields['illuminance']['lux']))
        if not value.is_finite() or value < 0:
            raise ValueError('照度必须是非负仪器读数')
        fields['illuminance'] = dict(fields['illuminance'], lux=str(value), unit='lx', source=source)
    for key in ('recovery_success', 'steps_conform'):
        if key in fields and fields[key] not in (True, False, None):
            raise ValueError(key + ' 必须为 true、false 或 null')
    if 'obstacle' in fields:
        o = fields['obstacle']
        for key in ('length_m', 'width_m', 'height_m'):
            if key in o and (not Decimal(str(o[key])).is_finite() or Decimal(str(o[key])) <= 0):
                raise ValueError('障碍物尺寸必须为正数')
        for key in ('bypassed', 'continued'):
            if key in o and o[key] is not None and type(o[key]) is not bool:
                raise ValueError('避障结果必须为 true、false 或 null')
    def apply(s):
        event(directory, 'metadata_updated', source, {'old': {k: s.get(k) for k in fields}, 'new': fields})
        if 'obstacle' in fields:
            fields['obstacle'] = dict(s.get('obstacle', {}), **fields['obstacle'])
        s.update(fields)
        if s.get('coordinates'):
            s['coordinates']['binding'] = {k: s[k] for k in ('session_id', 'light', 'scene', 'point')}
    update_session(directory, apply)


def attach_evidence(directory, path, source, position='', actual_time=None, category=None):
    if not source.strip():
        raise ValueError('影像证据必须记录来源')
    path = Path(path).expanduser().resolve()
    if not path.is_file():
        raise ValueError('只能关联用户提供的已有文件，路径不存在')
    state = read_json(directory / 'session.json')
    record = {'evidence_id': '{}-E{:03d}'.format(state['session_id'], len(state['evidence']) + 1),
              'path': str(path), 'bytes': path.stat().st_size, 'mtime_ns': path.stat().st_mtime_ns, 'sha256': sha256(path), 'source': source,
              'position_in_file': position or '待补', 'actual_time': actual_time, 'linked_at': clock_pair()}
    if category:
        if category not in ('illuminance', 'obstacle', 'client_speed', 'bypass_frame', 'other'):
            raise ValueError('未知影像类别')
        record['category'] = category
    if actual_time:
        parse_time(actual_time)
    def apply(s):
        s['evidence'].append(record)
        event(directory, 'evidence_linked', source, record)
    update_session(directory, apply)
    return record


def sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def metric(session):
    if session.get('kind') in ('verification', 'obstacle'):
        return None  # Neither reinspection nor obstacle runs are efficiency tests.
    start_at, end_at = session.get('task_started_at'), session.get('recovery_completed_at')
    if not start_at or not end_at:
        return None
    if any(a.get('kind') == 'clock_discontinuity' for a in session.get('anomalies', [])):
        if any(t.get('method', '').startswith('observed') for t in (start_at, end_at)):
            return None
    duration_ns = end_at['epoch_ns'] - start_at['epoch_ns']
    if duration_ns <= 0:
        return None
    return Fraction(str(session['area_m2'])) * 10**9 / duration_ns


def adopt(root, directory, reason, source):
    if not reason.strip() or not source.strip():
        raise ValueError('采用重测结果必须说明原因，不能自动挑最快的一次')
    s = read_json(directory / 'session.json')
    if s['phase'] in OPEN_PHASES:
        raise ValueError('本次测试仍在采集中')
    item = {'session_id': s['session_id'], 'key': [s['kind'], s['light'], s['scene'], s['point']],
            'reason': reason, 'source': source, 'recorded': clock_pair()}
    with locked(Path(root) / 'campaign.lock'):
        append_line(Path(root) / 'adoptions.jsonl', item)
    event(directory, 'result_adopted', source, item)
    return item


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=DEFAULT_ROOT)
    sub = parser.add_subparsers(dest='action', required=True)
    for action in ('parse', 'command'):
        p = sub.add_parser(action); p.add_argument('text')
    p = sub.add_parser('probe'); p.add_argument('--config', type=Path, default=CONFIG)
    sub.add_parser('status'); sub.add_parser('report')
    p = sub.add_parser('obstacle-start'); p.add_argument('--light', required=True, choices=LIGHTS); p.add_argument('--scene', required=True, choices=SCENES)
    for action in ('check', 'stop', 'info', 'event', 'evidence', 'adopt', 'request'):
        p = sub.add_parser(action); p.add_argument('--session')
        if action == 'stop':
            p.add_argument('--disposition', choices=('complete', 'abort'), required=True); p.add_argument('--reason', default=''); p.add_argument('--actual-time')
        if action in ('info', 'event', 'evidence', 'adopt', 'request'):
            p.add_argument('--source', required=True)
        if action in ('info', 'request'):
            p.add_argument('--file', type=Path, required=True)
        if action == 'event':
            p.add_argument('--kind', required=True, choices=('task_started', 'recovery_completed', 'obstacle_encounter', 'obstacle_completed', 'takeover', 'abort', 'note'))
            p.add_argument('--actual-time'); p.add_argument('--text', default='')
        if action == 'evidence':
            p.add_argument('--file', type=Path, required=True); p.add_argument('--position', default=''); p.add_argument('--actual-time')
            p.add_argument('--category', choices=('illuminance', 'obstacle', 'client_speed', 'bypass_frame', 'other'))
        if action == 'adopt': p.add_argument('--reason', required=True)
    args = parser.parse_args()
    from three_party_report import report
    result = None
    if args.action == 'parse':
        result = parse_command(args.text)
    elif args.action == 'probe':
        from three_party_capture import discover
        output = args.root / 'setup' / (dt.datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:6])
        output.mkdir(parents=True)
        result = discover(args.config, output)
        result = {'saved_at': str(output), 'topics': len(result['requested_topics']), 'missing': result['missing_topics'], 'known_data_limits': result['known_data_limits']}
    elif args.action in ('status', 'report'):
        result = report(args.root, force=args.action == 'report')
        if args.action == 'status':
            result['progress'] = read_json(args.root / 'progress.json')
            current = active_dir(args.root)
            result['active_session'] = read_json(current / 'session.json') if current else None
    elif args.action == 'obstacle-start':
        result = start(args.root, {'light': args.light, 'scene': args.scene, 'point': 'obstacle', 'raw_command': 'explicit obstacle-start'}, kind='obstacle')
    elif args.action == 'command':
        parsed = parse_command(args.text)
        action = parsed['action']
        if action == 'start':
            parsed['raw_command'] = args.text
            result = start(args.root, parsed)
        elif action in ('progress', 'report'):
            result = report(args.root, force=action == 'report')
            if action == 'progress':
                result['progress'] = read_json(args.root / 'progress.json')
                current = active_dir(args.root)
                result['active_session'] = read_json(current / 'session.json') if current else None
        elif action in ('complete', 'abort'):
            directory = resolve_session(args.root)
            if parsed.get('expected_kind') and read_json(directory / 'session.json')['kind'] != parsed['expected_kind']:
                raise ValueError('当前不是避障测试，保留当前采集；请明确要结束的测试')
            result = stop(args.root, directory, action, args.text)
        elif action == 'task_started':
            result = manual_event(resolve_session(args.root), 'task_started', 'operator chat report', args.text)
        else:
            result = parsed
    else:
        directory = resolve_session(args.root, args.session)
        if args.action == 'check':
            from three_party_capture import check_session
            result = check_session(directory)
        elif args.action == 'stop':
            result = stop(args.root, directory, args.disposition, args.reason, actual_time=args.actual_time)
        elif args.action == 'info':
            set_info(directory, read_json(args.file), args.source)
        elif args.action == 'event':
            result = manual_event(directory, args.kind, args.source, args.text, args.actual_time)
        elif args.action == 'evidence':
            result = attach_evidence(directory, args.file, args.source, args.position, args.actual_time, args.category)
        elif args.action == 'adopt':
            result = adopt(args.root, directory, args.reason, args.source)
        elif args.action == 'request':
            body = read_json(args.file)
            if not isinstance(body, dict) or not args.source.strip():
                raise ValueError('请求文件必须存在、是 JSON 对象并记录来源')
            result = event(directory, 'non_topic_request', args.source, {'original_file': str(args.file.resolve()), 'body': body})
        if args.action != 'check':
            if read_json(directory / 'session.json')['phase'] not in OPEN_PHASES:
                from three_party_capture import check_session
                check_session(directory)
            report(args.root)
    print(json.dumps(result or {'result': 'saved'}, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError, KeyError, InvalidOperation) as exc:
        print(json.dumps({'error': str(exc)}, ensure_ascii=False), file=sys.stderr)
        sys.exit(2)
