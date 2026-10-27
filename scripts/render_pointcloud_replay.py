#!/usr/bin/python3 -s
"""Render a recorded 3-D point cloud to MP4, entirely offline."""
import argparse
import collections
import csv
import datetime
import json
import math
from pathlib import Path
import subprocess

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import rosbag
from scipy.spatial.transform import Rotation, Slerp

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('dataset', type=Path)
parser.add_argument('output', type=Path)
parser.add_argument('--fps', type=int, default=10)
parser.add_argument('--style', choices=['overview','rear','bird'], default='overview')
parser.add_argument('--speed', type=float, default=1)
parser.add_argument('--preview-only', action='store_true')
parser.add_argument('--start-second', type=float, default=0)
parser.add_argument('--end-second', type=float, default=300)
args = parser.parse_args()
root = args.dataset.resolve()
args.output.parent.mkdir(parents=True, exist_ok=True)
cache = root / 'video_cache'
cache.mkdir(exist_ok=True)
manifest = json.loads((root / 'crop_manifest.json').read_text())
start_ns = manifest['start_ns']
cache_file = cache / 'recorded_clouds.npz'

if not cache_file.exists():
    stamps, offsets, pieces, transforms = [], [0], [], []
    with rosbag.Bag(str(root / 'localization_first_300s.bag')) as bag:
        for topic, msg, stamp in bag.read_messages(topics=['/cloud_registered', '/tf']):
            if topic == '/tf':
                for tf in msg.transforms:
                    if (tf.header.frame_id.lstrip('/'), tf.child_frame_id.lstrip('/')) != ('map', 'camera_init'):
                        continue
                    p, q = tf.transform.translation, tf.transform.rotation
                    transforms.append([(tf.header.stamp.to_nsec() - start_ns) / 1e9,
                                       p.x, p.y, p.z, q.x, q.y, q.z, q.w])
                continue
            time_s = (msg.header.stamp.to_nsec() - start_ns) / 1e9
            if not 0 <= time_s < 300:
                continue
            assert msg.header.frame_id.lstrip('/') == 'camera_init'
            fields = {f.name: f for f in msg.fields}
            assert all(fields[k].datatype == 7 for k in ('x', 'y', 'z'))
            endian = '>' if msg.is_bigendian else '<'
            dtype = np.dtype({'names': ['x', 'y', 'z'],
                              'formats': [endian + 'f4'] * 3,
                              'offsets': [fields[k].offset for k in ('x', 'y', 'z')],
                              'itemsize': msg.point_step})
            view = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                              strides=(msg.row_step, msg.point_step))
            points = np.column_stack([view[k].ravel() for k in ('x', 'y', 'z')])
            points = points[np.isfinite(points).all(axis=1)]
            if len(points) > 16000:
                points = points[::int(math.ceil(len(points) / 16000))]
            pieces.append(points.astype(np.float32))
            offsets.append(offsets[-1] + len(points))
            stamps.append(time_s)
            if len(stamps) % 300 == 0:
                print('Extracted', len(stamps), 'measured point-cloud frames', flush=True)
    assert len(stamps) > 2900 and transforms
    np.savez(cache_file, times=np.asarray(stamps), offsets=np.asarray(offsets),
             points=np.concatenate(pieces), transforms=np.asarray(transforms))

data = np.load(cache_file)
cloud_times, offsets, cloud_points = data['times'], data['offsets'], data['points']
tf = data['transforms']
tf = tf[np.argsort(tf[:, 0], kind='mergesort')]
_, unique = np.unique(tf[:, 0][::-1], return_index=True)
tf = tf[len(tf) - 1 - unique]
tf = tf[np.argsort(tf[:, 0])]
assert np.all(np.diff(tf[:, 0]) > 0)
tf_time = np.clip(cloud_times, tf[0, 0], tf[-1, 0])
map_rotations = Slerp(tf[:, 0], Rotation.from_quat(tf[:, 4:8]))(tf_time)
map_translations = np.column_stack([np.interp(tf_time, tf[:, 0], tf[:, i]) for i in (1, 2, 3)])

pose = np.genfromtxt(root / 'analysis/global_pose.csv', delimiter=',', names=True)
pose_time = pose['t_header']
with (root / 'analysis/localization_quality.csv').open() as stream:
    quality_rows = list(csv.DictReader(stream))
quality_times = np.array([float(row['t']) for row in quality_rows])
quality_states = [row['state'] for row in quality_rows]
base_positions = np.column_stack([pose[k] for k in ('x_base', 'y_base', 'z_base')])
extrinsic = json.loads((root / 'analysis/metrics.json').read_text())['body_to_base_link']
base_rotations = Rotation.from_quat(np.column_stack([pose[k] for k in ('qx', 'qy', 'qz', 'qw')]))
base_rotations = base_rotations * Rotation.from_quat(extrinsic['quaternion_xyzw'])
duration, width, height = 300, 1280, 720
frame_times = np.arange(duration * args.fps) / args.fps
sample_times = np.clip(frame_times, pose_time[0], pose_time[-1])
robot_positions = np.column_stack([np.interp(sample_times, pose_time, base_positions[:, i]) for i in range(3)])
robot_rotations = Slerp(pose_time, base_rotations)(sample_times)
robot_yaw = robot_rotations.as_euler('xyz')[:, 2]

with (root / 'map_reference/map_3d/map.pcd').open('rb') as stream:
    header = []
    while True:
        line = stream.readline()
        if not line:
            raise ValueError('Invalid PCD header')
        header.append(line)
        if line.startswith(b'DATA '):
            assert line.strip() == b'DATA binary'
            break
    # The pinned original map has five packed 32-bit fields.
    assert b'FIELDS x y z intensity observations\n' in header
    packed = np.frombuffer(stream.read(), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                                                ('intensity', '<f4'), ('observations', '<u4')])
    prior = np.column_stack([packed[k] for k in ('x', 'y', 'z')])
lo = base_positions.min(axis=0) - [9, 9, 1]
hi = base_positions.max(axis=0) + [9, 9, 8]
mask = np.isfinite(prior).all(axis=1) & (prior >= lo).all(axis=1) & (prior <= hi).all(axis=1)
prior = prior[mask]
_, keep = np.unique(np.floor(prior / .16).astype(np.int32), axis=0, return_index=True)
prior = prior[np.sort(keep)]
print('Prior map display points:', len(prior), 'Live frames:', len(cloud_times), flush=True)

def camera(eye, target):
    forward = target - eye
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.array([0., 0., 1.]))
    right /= np.linalg.norm(right)
    vertical = np.cross(right, forward)
    return np.column_stack([right, vertical, forward])

def projection(points, eye, basis, focal, panel_width, panel_height):
    local = (points - eye).dot(basis)
    z = local[:, 2]
    valid = z > .4
    uv = np.zeros((len(points), 2), dtype=np.float64)
    uv[valid, 0] = panel_width / 2 + focal * local[valid, 0] / z[valid]
    uv[valid, 1] = panel_height / 2 - focal * local[valid, 1] / z[valid]
    valid &= (uv[:, 0] >= 0) & (uv[:, 0] < panel_width - 1) & (uv[:, 1] >= 0) & (uv[:, 1] < panel_height - 1)
    return uv, z, valid

def raster(image, depth, points, colors, eye, basis, focal, point_size=1):
    h, w = image.shape[:2]
    uv, z, valid = projection(points, eye, basis, focal, w, h)
    if not np.any(valid):
        return
    xy = uv[valid].astype(np.int32)
    z = z[valid]
    c = np.broadcast_to(colors, (len(points), 3))[valid]
    flat = xy[:, 1] * w + xy[:, 0]
    order = np.lexsort((z, flat))
    ordered = flat[order]
    order = order[np.r_[True, ordered[1:] != ordered[:-1]]]
    xy, z, c = xy[order], z[order], c[order]
    for dx, dy in ((0, 0), (1, 0), (0, 1), (1, 1))[:point_size ** 2]:
        x, y = xy[:, 0] + dx, xy[:, 1] + dy
        visible = z <= depth[y, x] + .10
        x, y = x[visible], y[visible]
        image[y, x] = c[visible]
        depth[y, x] = np.minimum(depth[y, x], z[visible])

def polyline(image, points, color, eye, basis, focal, thickness=2):
    uv, _, valid = projection(points, eye, basis, focal, image.shape[1], image.shape[0])
    if valid.all():
        cv2.polylines(image, [uv.astype(np.int32).reshape(-1, 1, 2)], False, color, thickness, cv2.LINE_AA)

if args.style in ('rear','bird'):
    from render_pointcloud_rear import render_rear
    render_rear(args, root, cache, cloud_times, offsets, cloud_points,
                map_rotations, map_translations, pose_time, base_positions,
                base_rotations, prior, quality_times, quality_states)
    raise SystemExit(0)

panel_height, main_width, local_width = 574, 824, 430
target = (lo + hi) / 2
target[2] = float(np.median(base_positions[:, 2])) + 1.3
eye = target + [38., -46., 35.]
basis = camera(eye, target)
# Fit the displayed prior geometry with a fixed camera, preserving a stable global view.
local = (prior - eye).dot(basis)
span = np.max(np.abs(local[:, :2] / local[:, 2, None]), axis=0)
focal = float(min(main_width * .44 / span[0], panel_height * .44 / span[1]))
background = np.empty((panel_height, main_width, 3), dtype=np.uint8)
background[:] = (22, 18, 15)
background_depth = np.full((panel_height, main_width), np.inf, dtype=np.float32)
map_color = np.array([115, 120, 125], dtype=np.uint8)
raster(background, background_depth, prior, map_color, eye, basis, focal)

font_path = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
canvas = Image.new('RGB', (width, height), (14, 18, 25))
draw = ImageDraw.Draw(canvas)
draw.text((18, 10), 'MID360 三维点云离线回放', font=ImageFont.truetype(font_path, 27), fill=(235, 240, 248))
draw.text((18, 48), '原三维地图 + 实测点云 + 车体轨迹  |  前 5 分钟 · 1 倍速', font=ImageFont.truetype(font_path, 19), fill=(170, 186, 202))
draw.text((18, 82), '固定三维总览', font=ImageFont.truetype(font_path, 19), fill=(215, 225, 238))
draw.text((852, 82), '车体跟随视角', font=ImageFont.truetype(font_path, 19), fill=(215, 225, 238))
draw.text((18, 686), '灰色：旧地图  彩色：实测点云（按高度着色）  青色：轨迹  绿色：车体', font=ImageFont.truetype(font_path, 17), fill=(183, 196, 210))
template = np.asarray(canvas)[:, :, ::-1].copy()
params = json.loads((root / 'parameters_start.json').read_text())
footprint_xy = params['move_base']['local_costmap']['footprint']
if isinstance(footprint_xy, str):
    footprint_xy = json.loads(footprint_xy)
footprint = np.column_stack([np.asarray(footprint_xy, dtype=float), np.full(len(footprint_xy), .05)])
footprint = np.vstack([footprint, footprint[0]])
arrow = np.array([[0., 0., .15], [1.2, 0., .15], [.85, .2, .15], [1.2, 0., .15], [.85, -.2, .15]])
cloud_history = collections.deque(maxlen=6)
last_cloud = -1
log = (cache / 'ffmpeg_encode.log').open('w')
command = ['ffmpeg', '-hide_banner', '-loglevel', 'warning', '-y', '-f', 'rawvideo', '-pixel_format', 'bgr24',
           '-video_size', '1280x720', '-framerate', str(args.fps), '-i', '-', '-an', '-c:v', 'libx264',
           '-preset', 'veryfast', '-crf', '21', '-threads', '2', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(args.output)]
encoder = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=log)
try:
    for index, t in enumerate(frame_times):
        cloud_index = int(np.argmin(np.abs(cloud_times - t)))
        if cloud_index != last_cloud:
            points = cloud_points[offsets[cloud_index]:offsets[cloud_index + 1]]
            world = map_rotations[cloud_index].apply(points) + map_translations[cloud_index]
            cloud_history.append(world.astype(np.float32))
            last_cloud = cloud_index
        current = np.concatenate(cloud_history)
        color_value = np.clip((current[:, 2] + 1.3) / 5.0 * 255, 0, 255).astype(np.uint8)
        colors = cv2.applyColorMap(color_value[:, None], cv2.COLORMAP_JET)[:, 0, :]
        global_view = background.copy()
        global_depth = background_depth.copy()
        raster(global_view, global_depth, current, colors, eye, basis, focal, point_size=2)
        position = robot_positions[index]
        rotation = robot_rotations[index]
        history = robot_positions[:index + 1] + [0., 0., .08]
        polyline(global_view, history, (240, 225, 65), eye, basis, focal, 2)
        polyline(global_view, rotation.apply(footprint) + position, (75, 245, 115), eye, basis, focal, 2)
        polyline(global_view, rotation.apply(arrow) + position, (75, 245, 115), eye, basis, focal, 2)

        local_view = np.empty((panel_height, local_width, 3), dtype=np.uint8)
        local_view[:] = (22, 18, 15)
        local_depth = np.full((panel_height, local_width), np.inf, dtype=np.float32)
        local_target = position + [0., 0., .8]
        local_eye = local_target + Rotation.from_euler('z', robot_yaw[index]).apply([-8., -10., 5.5])
        local_basis = camera(local_eye, local_target)
        nearby = prior[np.sum((prior[:, :2] - position[:2]) ** 2, axis=1) < 15 ** 2]
        raster(local_view, local_depth, nearby, map_color, local_eye, local_basis, 385.)
        raster(local_view, local_depth, current, colors, local_eye, local_basis, 385., point_size=2)
        polyline(local_view, rotation.apply(footprint) + position, (75, 245, 115), local_eye, local_basis, 385., 2)
        polyline(local_view, rotation.apply(arrow) + position, (75, 245, 115), local_eye, local_basis, 385., 2)

        frame = template.copy()
        frame[108:108 + panel_height, 12:12 + main_width] = global_view
        frame[108:108 + panel_height, 846:846 + local_width] = local_view
        state_index = max(0, int(np.searchsorted(quality_times, t, side='right') - 1))
        state = quality_states[state_index]
        label = '%02d:%04.1f / 05:00.0   |   1x  %d fps   |   %s' % (int(t // 60), t % 60, args.fps, state)
        cv2.putText(frame, label, (520, 35), cv2.FONT_HERSHEY_SIMPLEX, .50, (215, 230, 240), 1, cv2.LINE_AA)
        cv2.putText(frame, 'map xyz=(%.2f, %.2f, %.2f)m  yaw=%.1f deg' % (*position, math.degrees(robot_yaw[index])),
                    (520, 67), cv2.FONT_HERSHEY_SIMPLEX, .46, (180, 200, 215), 1, cv2.LINE_AA)
        if index in (0, 60 * args.fps, 150 * args.fps, 299 * args.fps):
            cv2.imwrite(str(cache / ('frame_%03ds.png' % int(t))), frame)
        encoder.stdin.write(frame.tobytes())
        if index % (30 * args.fps) == 0:
            print('Rendered %.1f / 300 seconds' % t, flush=True)
finally:
    encoder.stdin.close()
    code = encoder.wait()
    log.close()
assert code == 0, 'ffmpeg failed; see video_cache/ffmpeg_encode.log'
metadata = {'output': str(args.output), 'source_bag': str(root / 'localization_first_300s.bag'),
            'duration_seconds': duration, 'fps': args.fps, 'frames': len(frame_times), 'size': [width, height],
            'pointcloud_topic': '/cloud_registered', 'source_frame': 'camera_init',
            'display_frame': 'map', 'transform': 'recorded map -> camera_init TF, interpolated at point-cloud header timestamps',
            'map_id': 'map_20260829_221335_gimp_cleaned', 'prior_display_voxel_m': .16,
            'cloud_trail_seconds': .6, 'max_display_points_per_cloud': 16000,
            'resampling': 'nearest recorded cloud per video frame; no synthetic point clouds',
            'robot_pose': 'recorded localization, converted from body to base_link',
            'rendering': 'perspective 3D point projection with per-pixel depth test; fixed overview and following camera',
            'localization_status': 'recorded /fast_lio/localization_status',
            'display_footprint': footprint_xy,
            'offline': True, 'hardware_connected': False}
(cache / 'render_manifest.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + '\n')
print('Video complete:', args.output, flush=True)
