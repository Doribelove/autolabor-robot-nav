#!/usr/bin/env python3
"""Bounded subscriber-only camera/ICP inspection; no goals or parameter writes.

Use optimized.sh, load_config.sh and setup_env.sh. Saves original sensor samples
and descriptive statistics, NOT a depth-accuracy or localization acceptance test.
"""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import threading
import time

# Limit only this diagnostic process. OpenCV/BLAS defaults created a measured
# CPU spike during the first capture; do not let a probe contend with navigation.
for library_limit in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS',
                      'NUMEXPR_NUM_THREADS'):
    os.environ[library_limit] = '1'
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs import point_cloud2
from scipy.spatial import cKDTree

cv2.setNumThreads(1)


def summary(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return ({'count': int(values.size), 'min': float(values.min()),
             'p50': float(np.median(values)), 'p95': float(np.percentile(values, 95)),
             'max': float(values.max())} if values.size else {'count': 0})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--label', required=True)
    parser.add_argument('--seconds', type=float, default=15)
    parser.add_argument('--clouds', action='store_true')
    args = parser.parse_args()
    if (not 5 <= args.seconds <= 30 or not args.label or
            any(c not in 'abcdefghijklmnopqrstuvwxyz0123456789_-' for c in args.label)):
        parser.error('Use 5..30 seconds and a lowercase filename label.')
    root = Path(__file__).resolve().parents[1]
    if (str(root) != '/home/slam/robot_j6m_ws_navigation_20260907' or
            os.environ.get('ROBOT_OPTIMIZED_SANDBOX') != '1'):
        raise RuntimeError('Use the candidate isolation entry.')
    output = root / 'validation' / ('input_inspection_' + args.label)
    output.mkdir(exist_ok=False)
    rospy.init_node('candidate_input_inspection', anonymous=True)
    for param in ('/nvidia_cmd_vel_watchdog/motion_enabled', '/fod_visual_servo/allow_motion'):
        if rospy.get_param(param, None) is not False:
            raise RuntimeError('Static inspection requires both motion gates false.')
    bridge = CvBridge()
    lock = threading.Lock()
    stamps = {'rgb': [], 'depth': [], 'info': []}
    arrivals = {'rgb': [], 'depth': [], 'info': []}
    info_types = Counter()
    info_examples = {}
    image_stats = []
    depth_stats = []
    errors = []
    first = {}

    def callback(msg, kind):
        now = time.monotonic()
        with lock:
            stamps[kind].append(msg.header.stamp.to_nsec())
            arrivals[kind].append(now)
            index = len(stamps[kind])
            if kind == 'info':
                calibration = {'frame': msg.header.frame_id, 'width': msg.width,
                               'height': msg.height, 'D': list(msg.D), 'K': list(msg.K),
                               'R': list(msg.R), 'P': list(msg.P),
                               'distortion_model': msg.distortion_model}
                key = json.dumps(calibration, sort_keys=True)
                info_types[key] += 1
                info_examples[key] = calibration
                return
            # Retain only one original image and one original depth array.
            if kind not in first:
                first[kind] = msg
        # Statistics at about 1.5 Hz; do not add per-pixel work on every frame.
        if index % 10 != 1:
            return
        try:
            if kind == 'rgb':
                array = bridge.imgmsg_to_cv2(msg, 'bgr8')
                gray = cv2.cvtColor(array, cv2.COLOR_BGR2GRAY)
                entry = {'stamp_ns': msg.header.stamp.to_nsec(),
                         'brightness_p50': float(np.median(gray)),
                         'dark_fraction_below_10': float(np.mean(gray < 10)),
                         'bright_fraction_above_245': float(np.mean(gray > 245)),
                         'laplacian_variance': float(cv2.Laplacian(gray, cv2.CV_64F).var())}
                with lock:
                    image_stats.append(entry)
            else:
                array = bridge.imgmsg_to_cv2(msg, 'passthrough')
                if msg.encoding != '32FC1':
                    raise ValueError('Expected current 32FC1 meter depth, got ' + msg.encoding)
                valid = np.isfinite(array) & (array >= 0.3) & (array <= 15.0)
                h, w = array.shape
                center = array[h // 4:3 * h // 4, w // 4:3 * w // 4]
                center_valid = np.isfinite(center) & (center >= 0.3) & (center <= 15.0)
                entry = {'stamp_ns': msg.header.stamp.to_nsec(),
                         'valid_fraction': float(np.mean(valid)),
                         'center_valid_fraction': float(np.mean(center_valid)),
                         'valid_depth_m': summary(array[valid]),
                         'nan_fraction': float(np.mean(np.isnan(array))),
                         'positive_inf_fraction': float(np.mean(np.isposinf(array))),
                         'negative_inf_fraction': float(np.mean(np.isneginf(array)))}
                with lock:
                    depth_stats.append(entry)
        except Exception as exc:
            with lock:
                errors.append(str(exc))

    subscriptions = [rospy.Subscriber(topic, kind, callback, callback_args=name,
                                      queue_size=2, buff_size=4 * 1024 * 1024)
                     for topic, kind, name in (
                         ('/fod_camera/image_raw', Image, 'rgb'),
                         ('/fod_camera/depth_registered', Image, 'depth'),
                         ('/fod_camera/camera_info', CameraInfo, 'info'))]
    started = time.monotonic()
    while time.monotonic() - started < args.seconds and not rospy.is_shutdown():
        time.sleep(0.1)
    for sub in subscriptions:
        sub.unregister()
    time.sleep(0.1)
    with lock:
        result = {'started_wall_epoch': time.time() - (time.monotonic() - started),
                  'scope': 'descriptive sensor data, no external ground truth',
                  'counts': {k: len(v) for k, v in stamps.items()},
                  'hz': {k: (len(v) - 1) / (v[-1] - v[0]) if len(v) > 1 else None
                         for k, v in arrivals.items()},
                  'exact_rgb_depth_stamp_matches': len(set(stamps['rgb']) & set(stamps['depth'])),
                  'camera_info_copies_per_stamp': dict(Counter(Counter(stamps['info']).values())),
                  'camera_info_variants': [{'count': count, 'calibration': info_examples[key]}
                                           for key, count in info_types.items()],
                  'images': image_stats, 'depth': depth_stats, 'errors': errors}
        if 'rgb' in first:
            if not cv2.imwrite(str(output / 'camera_rgb.png'), bridge.imgmsg_to_cv2(first['rgb'], 'bgr8')):
                raise RuntimeError('Could not save original camera image.')
        if 'depth' in first:
            np.save(str(output / 'camera_depth_m.npy'), bridge.imgmsg_to_cv2(first['depth'], 'passthrough'))
    if args.clouds:
        submap = rospy.wait_for_message('/fast_lio_localization/submap', PointCloud2, timeout=8)
        aligned = rospy.wait_for_message('/fast_lio_localization/aligned_scan', PointCloud2, timeout=8)
        target = np.asarray(list(point_cloud2.read_points(submap, field_names=('x', 'y', 'z'), skip_nans=True)))
        source = np.asarray(list(point_cloud2.read_points(aligned, field_names=('x', 'y', 'z'), skip_nans=True)))
        distances, _ = cKDTree(target).query(source, workers=1)
        inliers = distances <= 1.0
        result['clouds'] = {'note': 'Adjacent accepted matches; submap and scan are not a synchronized ICP trial.',
                            'source_points': len(source), 'submap_points': len(target),
                            'source_frame': aligned.header.frame_id, 'target_frame': submap.header.frame_id,
                            'nn_distance_m': summary(distances),
                            'fraction_over_0_35m': float(np.mean(distances > 0.35)),
                            'inlier_fraction_1m': float(np.mean(inliers)),
                            'inlier_rmse_1m': float(np.sqrt(np.mean(distances[inliers] ** 2))),
                            'source_xyz_ranges': [source.min(axis=0).tolist(), source.max(axis=0).tolist()]}
        np.savez_compressed(str(output / 'aligned_and_submap.npz'), source=source, target=target)
    result['final_motion_gates'] = {p: rospy.get_param(p, None) for p in (
        '/nvidia_cmd_vel_watchdog/motion_enabled', '/fod_visual_servo/allow_motion')}
    with (output / 'summary.json').open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(result, indent=2, allow_nan=False))
    return int(bool(errors))


if __name__ == '__main__':
    raise SystemExit(main())
