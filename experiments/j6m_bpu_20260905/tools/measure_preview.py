#!/usr/bin/env python3
"""Bounded read-only timing probe for the camera/Qt fixture or candidate graph."""
import argparse
from collections import deque
from datetime import datetime
import json
import os
from pathlib import Path
import threading
import time
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String


def metrics(values):
    return {key: float(value) for key, value in (
        ('mean', np.mean(values)), ('p50', np.percentile(values, 50)),
        ('p95', np.percentile(values, 95)), ('maximum', np.max(values)))}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=int, default=60)
    args = parser.parse_args()
    master = os.environ.get('ROS_MASTER_URI')
    if os.environ.get('ROBOT_OPTIMIZED_SANDBOX') != '1' or master not in (
            'http://127.0.0.1:11571', 'http://192.168.10.100:11311') or not 5 <= args.seconds <= 300:
        raise ValueError('Use the isolated candidate environment and a bounded duration')
    rospy.init_node('bpu_performance_readonly', disable_signals=True)
    raw_topic = ('/bpu_preview_zed/zed_node/rgb/image_rect_color'
                 if '11571' in master else '/fod_camera/image_raw')
    lock = threading.Lock()
    raw, results, statuses = [], [], deque(maxlen=10)
    def receive_raw(msg):
        with lock:
            raw.append((time.monotonic(), msg.header.stamp.to_nsec()))
    def receive_result(msg):
        with lock:
            results.append(dict(at=time.monotonic(), stamp=msg.header.stamp.to_nsec(),
                                age_ms=(rospy.Time.now() - msg.header.stamp).to_sec() * 1000,
                                width=msg.width, height=msg.height, frame_id=msg.header.frame_id))
    subscriptions = [rospy.Subscriber(raw_topic, Image, receive_raw, queue_size=1, buff_size=10 * 1024 * 1024),
                     rospy.Subscriber('/fod/bpu_preview/image', Image, receive_result, queue_size=1,
                                      buff_size=10 * 1024 * 1024),
                     rospy.Subscriber('/fod/bpu_preview/status', String,
                                      lambda msg: statuses.append(msg.data), queue_size=1)]
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.seconds and not rospy.is_shutdown():
            time.sleep(0.1)
    finally:
        for subscription in subscriptions:
            subscription.unregister()
    with lock:
        if len(raw) < 2 or len(results) < 2:
            raise RuntimeError('Insufficient real camera/result frames')
        stamps = [r['stamp'] for r in results]
        report = dict(master=master, duration_seconds=time.monotonic() - started,
                      raw_frames=len(raw), result_frames=len(results),
                      raw_hz=(len(raw) - 1) / (raw[-1][0] - raw[0][0]),
                      result_hz=(len(results) - 1) / (results[-1]['at'] - results[0]['at']),
                      source_age_ms=metrics([r['age_ms'] for r in results]),
                      result_interval_ms=metrics([1000 * (b['at'] - a['at']) for a, b in zip(results, results[1:])]),
                      repeated_or_out_of_order=sum(b <= a for a, b in zip(stamps, stamps[1:])),
                      display_only_frame_ids=all(r['frame_id'] == 'bpu_preview_demo_only' for r in results),
                      recent_status=list(statuses), results=results)
    lab = Path(__file__).resolve().parents[1]
    output = lab / 'logs' / ('performance_live_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '.json')
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps(dict(output=str(output), **{k: v for k, v in report.items() if k != 'results'}),
                     indent=2, ensure_ascii=False))
    if report['repeated_or_out_of_order'] or not report['display_only_frame_ids']:
        raise RuntimeError('Result identity contract failed')


if __name__ == '__main__':
    main()
