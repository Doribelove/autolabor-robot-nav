#!/usr/bin/env python3
"""Summarize recorded rates and sampled serialized sizes, not measured NIC traffic."""
import collections
import io
import json
from pathlib import Path
import rosbag

root = Path(__file__).resolve().parents[1]
path = '/home/slam/robot_j6m_ws/rosbags/mode1_nav_mode1_20260905_140157_0.bag'
with rosbag.Bag(path, 'r') as bag:
    duration = bag.get_end_time() - bag.get_start_time()
    topics = bag.get_type_and_topic_info().topics
    sizes = collections.defaultdict(list)
    selected = ['/livox/lidar', '/livox/imu', '/cloud_registered', '/cloud_registered_body',
                '/dual_lidar/scan', '/mid360/scan', '/scan', '/Odometry', '/odom', '/cmd_vel']
    for topic, message, _ in bag.read_messages(topics=selected):
        if len(sizes[topic]) < 5:
            buffer = io.BytesIO()
            message.serialize(buffer)
            sizes[topic].append(buffer.tell())
        if all(len(sizes[t]) >= 5 for t in selected):
            break
    summary = {'bag': path, 'duration_sec': duration, 'topics': {}, 'warnings': []}
    for topic, info in sorted(topics.items()):
        entry = {'count': info.message_count, 'mean_recorded_hz': info.message_count / duration,
                 'type': info.msg_type}
        if sizes.get(topic):
            entry['sample_mean_serialized_bytes'] = sum(sizes[topic]) / len(sizes[topic])
            entry['estimated_payload_bytes_per_sec'] = entry['sample_mean_serialized_bytes'] * entry['mean_recorded_hz']
        summary['topics'][topic] = entry
    for _, message, stamp in bag.read_messages(topics=['/rosout']):
        if message.level >= 4:
            summary['warnings'].append({'stamp': stamp.to_sec(), 'node': message.name,
                                        'level': message.level, 'message': message.msg})
(root / 'validation/recorded_baseline.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
print('Recorded {:.1f}s; {} topics; {} WARN/ERROR/FATAL entries'.format(duration, len(topics), len(summary['warnings'])))
