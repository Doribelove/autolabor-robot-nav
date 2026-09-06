#!/usr/bin/env python3
"""Analyze the interrupted 2026-09-05 run without touching a live ROS graph."""
from collections import Counter, defaultdict
import datetime as dt
import json
from pathlib import Path
import re
import statistics

import rosbag


ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / 'validation' / 'stability_20260905_1819'
END = '18:22:00'  # Resource baseline excludes the detected wheel event and shutdown.


def stats(values):
    values = sorted(values)
    if not values:
        return {'n': 0}
    return {'n': len(values), 'mean': statistics.mean(values),
            'median': statistics.median(values), 'min': values[0], 'max': values[-1]}


def memory_mib(value):
    if value[-1:].lower() in ('g', 'm', 'k'):
        return float(value[:-1]) * {'g': 1024, 'm': 1, 'k': 1 / 1024}[value[-1].lower()]
    return float(value) / 1024


def top_summary(filename, pids, end=END):
    rows = defaultdict(list)
    memory = defaultdict(list)
    cpu = []
    available = []
    blocks = (RUN / filename).read_text().split('top - ')[1:]
    for block in blocks[1:]:  # First top frame is not an interval measurement.
        stamp = block[:8]
        if not '18:17:00' <= stamp <= end:
            continue
        idle = re.search(r'([\d.]+) id', block)
        avail = re.search(r'([\d.]+) avail Mem', block)
        if idle:
            cpu.append(100 - float(idle.group(1)))
        if avail:
            available.append(float(avail.group(1)))
        for line in block.splitlines():
            fields = line.split()
            if len(fields) < 12 or fields[0] not in pids:
                continue
            rows[pids[fields[0]]].append(float(fields[8]))
            memory[pids[fields[0]]].append(memory_mib(fields[5]))
    return {'whole_cpu_percent': stats(cpu), 'available_memory_mib': stats(available),
            'process_single_core_cpu_percent': {k: stats(v) for k, v in rows.items()},
            'rss_mib_top_display_precision': {k: stats(v) for k, v in memory.items()}}


def main():
    result = {'scope': 'Interrupted stationary diagnostic load, not original/new whole-system A/B',
              'resource_window': '2026-09-05 18:17:00..18:22:00 +0800'}
    states = Counter()
    rmse = []
    counts = Counter()
    nonzero = []
    max_command = 0.0
    diagnostics = {}
    tf_owners = defaultdict(set)
    with rosbag.Bag(str(RUN / 'diagnostic_topics.bag')) as bag:
        result['bag_duration_sec'] = bag.get_end_time() - bag.get_start_time()
        for topic, msg, timestamp, connection in bag.read_messages(return_connection_header=True):
            counts[topic] += 1
            if topic == '/fast_lio/localization_status' and timestamp.to_sec() <= 1788603725.5:
                fields = dict(v.split('=', 1) for v in msg.data.split(';') if '=' in v)
                states[fields.get('state', 'UNKNOWN')] += 1
                value = float(fields.get('rmse', 'inf'))
                if value != float('inf'):
                    rmse.append(value)
            elif topic.endswith('_wheel_vel') and abs(msg.data) > 1e-6:
                nonzero.append({'topic': topic, 'stamp_epoch': timestamp.to_sec(),
                                'wall_local': dt.datetime.fromtimestamp(timestamp.to_sec()).isoformat(),
                                'mps': msg.data})
            elif topic == '/cmd_vel':
                max_command = max([max_command] + [abs(getattr(v, axis))
                                  for v in (msg.linear, msg.angular) for axis in ('x', 'y', 'z')])
            elif topic == '/diagnostics' and timestamp.to_sec() <= 1788603725.5:
                for status in msg.status:
                    diagnostics[status.name] = {'level': status.level, 'message': status.message,
                                                'values': {v.key: v.value for v in status.values}}
            elif topic in ('/tf', '/tf_static'):
                owner = connection.get('callerid', 'unknown')
                if isinstance(owner, bytes):
                    owner = owner.decode('utf-8', errors='replace')
                for transform in msg.transforms:
                    tf_owners[transform.header.frame_id + ' -> ' + transform.child_frame_id].add(
                        owner)
    result.update({'bag_topic_counts': dict(counts), 'localization_state_counts_before_event': dict(states),
                   'status_rmse_m_repeated_at_20hz': stats(rmse), 'nonzero_wheel_feedback': nonzero,
                   'maximum_abs_final_command': max_command, 'last_diagnostics_before_event': diagnostics,
                   'tf_publishers': {k: sorted(v) for k, v in tf_owners.items()}})
    result['nvidia'] = top_summary('nvidia_top.log', {
        '2715050': 'ZED', '2715058': 'detector', '2716998': 'Qt_RViz',
        '2714403': 'watchdog', '2714352': 'Livox', '3176113': 'ToDesk_session',
        '4253': 'GNOME', '3311': 'Xorg'})
    result['j6m'] = top_summary('j6m_top.log', {
        '1426029': 'FAST_LIO', '1426040': 'ICP', '1426079': 'move_base',
        '1425768': 'rosmaster'})
    result['resource_baseline_before_image_probe'] = {
        'window': '18:17:00..18:20:40 +0800; excludes CPU-heavy diagnostic capture',
        'nvidia': top_summary('nvidia_top.log', {}, '18:20:40'),
        'j6m': top_summary('j6m_top.log', {}, '18:20:40')}
    result['measurement_interference'] = {
        'image_probe_pid': 3221346, 'window': '18:20:49..18:21:07 +0800',
        'peak_probe_single_core_cpu_percent': 770.1,
        'note': 'Initial image probe used library thread defaults. Later capped to one thread; not a navigation regression.'}
    gpu, temperatures = [], []
    for line in (RUN / 'nvidia_tegrastats.log').read_text().splitlines():
        fields = line.split()
        if len(fields) < 2 or not '18:17:00' <= fields[1] <= END:
            continue
        value = re.search(r'GR3D_FREQ (\d+)%', line)
        temp = re.search(r'CPU@([\d.]+)C', line)
        if value:
            gpu.append(int(value.group(1)))
        if temp:
            temperatures.append(float(temp.group(1)))
    result['gpu_percent'] = stats(gpu)
    result['cpu_temperature_c'] = stats(temperatures)
    scene = json.loads((ROOT / 'validation/input_inspection_before_20260905/summary.json').read_text())
    result['depth_valid_pixel_fraction'] = stats([v['valid_fraction'] for v in scene['depth']])
    result['camera_info_counts'] = scene['counts']
    result['camera_info_variants'] = len(scene['camera_info_variants'])
    result['completed_full_minute_windows'] = len(list(RUN.glob('minute_*.log'))) - 1
    serialized = json.dumps(result, indent=2, allow_nan=False) + '\n'
    (RUN / 'analysis.json').write_text(serialized)
    compact = {k: v for k, v in result.items() if k not in ('last_diagnostics_before_event', 'tf_publishers')}
    print(json.dumps(compact, indent=2))


if __name__ == '__main__':
    main()
