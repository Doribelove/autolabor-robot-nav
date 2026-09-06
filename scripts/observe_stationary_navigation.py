#!/usr/bin/env python3
"""Bounded, read-only observation of the live candidate graph. Never publishes motion.

Run via optimized.sh after sourcing load_config.sh and setup_env.sh. This is a
diagnostic subscriber, not a replacement safety controller or a motion test.
"""
import argparse
import json
import math
from pathlib import Path
import threading
import time

import rosgraph
import roslib.message
import rospy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30.0)
    parser.add_argument('--output', default='stationary_navigation_observation.json')
    args = parser.parse_args()
    if not 5 <= args.seconds <= 60 or Path(args.output).name != args.output:
        parser.error('Use 5..60 seconds and a filename within candidate validation/.')
    root = Path(__file__).resolve().parents[1]
    rospy.init_node('candidate_stationary_observer', anonymous=True, disable_signals=True)
    master = rosgraph.Master(rospy.get_name())
    if rospy.get_param('/nvidia_cmd_vel_watchdog/motion_enabled', None) is not False:
        raise RuntimeError('Refusing observation: the final motion gate must be false.')
    published = dict(master.getPublishedTopics(''))
    raw_topics = {
        '/gateway/livox/lidar', '/gateway/livox/imu',
        '/fod_camera/image_raw', '/fod_camera/depth_registered', '/fod/detections',
    }
    typed_topics = {
        '/Odometry', '/odom', '/fast_lio/body_point_count',
        '/dual_lidar/front/scan_raw', '/dual_lidar/rear/scan_raw',
        '/dual_lidar/scan', '/mid360/scan', '/scan',
        '/cmd_vel', '/cmd_vel_safe', '/m2_driver/left_wheel_vel',
        '/m2_driver/right_wheel_vel', '/m2_driver/chassis_info',
        '/m2_driver/chassis_monitor', '/m2_driver/control_timeout',
        '/avoidance/source_mode', '/avoidance/dual_lidar_active',
        '/fast_lio/localization_status', '/autolabor_operator_gui/map_display_status',
        '/nvidia_cmd_vel_watchdog/status', '/move_base/status',
        '/navigation_pause/paused', '/coverage/active',
    }
    required = {
        '/gateway/livox/lidar', '/gateway/livox/imu', '/Odometry', '/odom',
        '/fast_lio/body_point_count', '/dual_lidar/front/scan_raw',
        '/dual_lidar/rear/scan_raw', '/scan', '/cmd_vel', '/cmd_vel_safe',
        '/m2_driver/left_wheel_vel', '/m2_driver/right_wheel_vel',
        '/fod_camera/image_raw', '/fod_camera/depth_registered',
    }
    records = {topic: {'type': published.get(topic), 'count': 0}
               for topic in sorted(raw_topics | typed_topics)}
    lock = threading.Lock()
    faults = set()
    start = time.monotonic()
    started_wall = time.strftime('%Y-%m-%d %H:%M:%S %z')
    subscriptions = []

    def callback(msg, topic):
        now = time.monotonic()
        with lock:
            record = records[topic]
            if not record['count']:
                record['first_monotonic'] = now
                record['max_gap_sec'] = 0.0
            else:
                record['max_gap_sec'] = max(record['max_gap_sec'], now - record['last_monotonic'])
            record['last_monotonic'] = now
            record['count'] += 1
            if hasattr(msg, 'header') and msg.header.stamp.to_sec() > 0:
                age = rospy.Time.now().to_sec() - msg.header.stamp.to_sec()
                record['last_stamp_age_sec'] = age
                record['max_stamp_age_sec'] = max(record.get('max_stamp_age_sec', age), age)
            if topic in ('/cmd_vel', '/cmd_vel_safe'):
                values = [getattr(vector, axis) for vector in (msg.linear, msg.angular)
                          for axis in ('x', 'y', 'z')]
                if not all(math.isfinite(v) for v in values) or any(abs(v) > 1e-6 for v in values):
                    faults.add('Nonzero or invalid command observed: ' + topic)
                record['max_abs_component'] = max(record.get('max_abs_component', 0), *map(abs, values))
            elif topic.endswith('_wheel_vel'):
                record['max_abs_mps'] = max(record.get('max_abs_mps', 0), abs(msg.data))
                if not math.isfinite(msg.data) or abs(msg.data) > 0.01:
                    faults.add('Wheel feedback is not stationary: ' + topic)
            elif published.get(topic) == 'sensor_msgs/LaserScan':
                finite = [v for v in msg.ranges if math.isfinite(v) and msg.range_min <= v <= msg.range_max]
                record['scan_bins'] = len(msg.ranges)
                record['last_valid_bins'] = len(finite)
                record['min_valid_bins'] = min(record.get('min_valid_bins', len(finite)), len(finite))
                record['last_nearest_m'] = min(finite) if finite else None
            elif published.get(topic) == 'nav_msgs/Odometry':
                pose = msg.pose.pose
                values = [pose.position.x, pose.position.y, pose.position.z,
                          pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                record['last_pose_xyz_xyzw'] = values
                record.setdefault('first_pose_xyz_xyzw', values)
                if not all(math.isfinite(v) for v in values):
                    faults.add('Invalid odometry: ' + topic)
            elif hasattr(msg, 'data'):
                record['last_value'] = msg.data
                if topic == '/fast_lio/localization_status':
                    state = msg.data.split(';', 1)[0]
                    counts = record.setdefault('state_counts', {})
                    counts[state] = counts.get(state, 0) + 1
                if isinstance(msg.data, bool):
                    record['true_count'] = record.get('true_count', 0) + int(msg.data)
                if topic == '/m2_driver/control_timeout' and msg.data:
                    faults.add('New VCU control timeout observed')
            elif topic in ('/m2_driver/chassis_info', '/m2_driver/chassis_monitor'):
                record['last_value'] = {slot: getattr(msg, slot) for slot in msg.__slots__}
            elif topic == '/move_base/status':
                record['last_goals'] = [{'id': status.goal_id.id, 'status': status.status,
                                        'text': status.text} for status in msg.status_list]

    for topic in sorted(records):
        message_type = published.get(topic)
        kind = (rospy.AnyMsg if topic in raw_topics else
                roslib.message.get_message_class(message_type) if message_type else None)
        if kind is None:
            records[topic]['subscription_error'] = 'No published message type'
            continue
        subscriptions.append(rospy.Subscriber(topic, kind, callback, callback_args=topic,
                                              queue_size=1, buff_size=4 * 1024 * 1024))
    while not rospy.is_shutdown() and time.monotonic() - start < args.seconds:
        time.sleep(0.1)
        with lock:
            now = time.monotonic()
            if now - start > 5:
                for topic in required:
                    if now - records[topic].get('last_monotonic', start) > 2:
                        faults.add('Required data absent/stale: ' + topic)
            if faults:
                break
    for subscription in subscriptions:
        subscription.unregister()
    finish = time.monotonic()
    state = master.getSystemState()
    publishers = dict(state[0])
    for topic, expected in (('/cmd_vel', '/nvidia_cmd_vel_watchdog'),
                            ('/scan', '/avoidance_scan_fusion')):
        if publishers.get(topic) != [expected]:
            faults.add('Unexpected publisher ownership: ' + topic)
    if rospy.get_param('/nvidia_cmd_vel_watchdog/motion_enabled', None) is not False:
        faults.add('Motion gate changed during observation')
    with lock:
        for record in records.values():
            if record['count']:
                first = record.pop('first_monotonic')
                last = record.pop('last_monotonic')
                record['observed_hz'] = ((record['count'] - 1) / (last - first)
                                         if last > first else None)
                record['age_at_end_sec'] = finish - last
        result = {'scope': 'stationary streams and zero output; localization reported separately',
                  'duration_sec': finish - start, 'started_wall_local': started_wall,
                  'motion_enabled': False, 'goals_sent_by_observer': 0,
                  'faults': sorted(faults), 'topics': records,
                  'publishers': {t: publishers.get(t, []) for t in (
                      '/cmd_vel', '/cmd_vel_safe', '/scan', '/ackerman_vel',
                      '/m2_driver/emergency_stop', '/m2_driver/brake_set')},
                  'gui_subscriptions': [t for t, nodes in state[1] if '/autolabor_operator_gui' in nodes]}
    (root / 'validation' / args.output).write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps(result, indent=2, allow_nan=False))
    rospy.signal_shutdown('Passive observation complete')
    return int(bool(faults)) * 2


if __name__ == '__main__':
    raise SystemExit(main())
