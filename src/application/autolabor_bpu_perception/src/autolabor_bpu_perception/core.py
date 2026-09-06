"""ROS-independent time, geometry, tracking and obstacle validation."""
import math
import numpy as np


def fresh(stamp, now, maximum_age=0.35, future_tolerance=0.02):
    return (all(math.isfinite(x) for x in (stamp, now, maximum_age)) and
            stamp > 0 and maximum_age > 0 and -future_tolerance <= now-stamp <= maximum_age)


def rotation_rpy(roll, pitch, yaw):
    if not all(math.isfinite(x) for x in (roll, pitch, yaw)):
        raise ValueError('Nonfinite rotation')
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def zed_mount(config):
    """Convert a measured left-lens/center pose to the wrapper's screw-base pose.

    REP-103: x forward, y left, z up; positive pitch looks down. The bundled
    ZED2 URDF adds +0.05 rad pitch and +0.015 m height internally.
    Missing measurements deliberately prevent a calibrated vehicle position.
    """
    if config.get('confirmed') is not True:
        raise ValueError('ZED mount measurements have not been confirmed')
    required = ('forward_m', 'right_m', 'up_m', 'pitch_down_deg', 'yaw_left_deg', 'roll_deg')
    if any(not isinstance(config.get(k), (int, float)) or not math.isfinite(config[k]) for k in required):
        raise ValueError('Incomplete or nonfinite ZED mount measurements')
    reference = config.get('reference')
    if reference not in ('left_camera', 'camera_center', 'mounting_base'):
        raise ValueError('Specify the measured ZED reference point')
    mid = np.asarray(config.get('mid360_in_base_m'), dtype=float)
    if mid.shape != (3,) or not np.isfinite(mid).all():
        raise ValueError('Invalid MID360 position')
    measured = mid + [config['forward_m'], -config['right_m'], config['up_m']]
    angles = [math.radians(config[k]) for k in ('roll_deg','pitch_down_deg','yaw_left_deg')]
    optical_body_rotation = rotation_rpy(*angles)
    if reference == 'mounting_base':
        raise ValueError('Mounting-base measurements require an optical-axis reference; measure camera center or left lens')
    base_rotation = optical_body_rotation @ rotation_rpy(0, -0.05, 0)
    left_offset = np.array([-0.01, 0.06, 0.0]) if reference == 'left_camera' else np.zeros(3)
    base_position = measured - optical_body_rotation @ left_offset - base_rotation @ np.array([0,0,0.015])
    # Recover RPY robustly; installation avoids the +/-90 degree singularity.
    pitch = math.asin(float(np.clip(-base_rotation[2,0], -1, 1)))
    if abs(math.cos(pitch)) < 1e-6:
        raise ValueError('Camera mounting rotation is singular')
    rpy = [math.atan2(base_rotation[2,1], base_rotation[2,2]), pitch,
           math.atan2(base_rotation[1,0], base_rotation[0,0])]
    optical_to_body = rotation_rpy(-math.pi/2, 0, -math.pi/2)
    left_position = measured if reference == 'left_camera' else measured + optical_body_rotation @ np.array([-0.01,0.06,0])
    return dict(base_position=base_position, base_rpy=rpy, left_position=left_position,
                optical_to_base=optical_body_rotation @ optical_to_body)


def bbox_iou(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    extent = np.maximum(0, np.minimum(a[2:], b[2:])-np.maximum(a[:2], b[:2]))
    intersection = float(np.prod(extent))
    return intersection / max(1e-9, float(np.prod(np.maximum(0,a[2:]-a[:2])) +
                                        np.prod(np.maximum(0,b[2:]-b[:2])) - intersection))


class ImageTracker:
    """Unique per-session IDs, class-aware one-to-one IoU association, no stale output."""
    def __init__(self, maximum_age=0.5, minimum_iou=0.2):
        self.maximum_age, self.minimum_iou = maximum_age, minimum_iou
        self.tracks, self.next_id, self.last_stamp = {}, 1, 0.0

    def update(self, detections, stamp):
        if not math.isfinite(stamp) or stamp <= self.last_stamp:
            raise ValueError('Non-increasing observation timestamp')
        self.last_stamp = stamp
        self.tracks = {i:t for i,t in self.tracks.items() if stamp-t['stamp'] <= self.maximum_age}
        candidates = sorted(((-bbox_iou(d['bbox'], t['bbox']), i, j)
                             for i,t in self.tracks.items() for j,d in enumerate(detections)
                             if d['class_id'] == t['class_id']), key=lambda x:x[0])
        assigned, used = {}, set()
        for negative_iou, i, j in candidates:
            if -negative_iou < self.minimum_iou:
                break
            if i not in used and j not in assigned:
                assigned[j] = i; used.add(i)
        result = []
        for j,d in enumerate(detections):
            i = assigned.get(j)
            if i is None:
                i = self.next_id; self.next_id += 1
            record = dict(d, object_id=i, stamp=stamp)
            self.tracks[i] = record
            result.append(record)
        return result


def obstacle_polygon(position, dimensions, yaw, padding=0.15):
    p, d = np.asarray(position,float), np.asarray(dimensions,float)
    if (p.shape != (3,) or d.shape != (3,) or not np.isfinite(p).all() or
            not np.isfinite(d).all() or not math.isfinite(yaw) or
            np.any(d <= 0) or np.any(d > 20) or not 0 <= padding <= 1):
        raise ValueError('Invalid 3D obstacle geometry')
    hx,hy = d[:2]/2 + padding
    xy = np.array([[hx,hy],[-hx,hy],[-hx,-hy],[hx,-hy]])
    return xy @ rotation_rpy(0,0,yaw)[:2,:2].T + p[:2]
