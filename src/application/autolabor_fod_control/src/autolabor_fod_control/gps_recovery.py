"""Depth-based memory for one FOD and its GPS endpoint after visual loss."""

from collections import deque
import math
from statistics import median

from gps_module.long_range import EARTH_RADIUS_M, rotate_xy, validate_latitude_longitude
from .visual_servo import same_target_observations


def local_to_gps(x, y, origin_lat, origin_lon, yaw_offset=0.0):
    """Exact inverse of the existing GPS goal manager's local conversion."""
    origin_lat, origin_lon = validate_latitude_longitude(origin_lat, origin_lon)
    east, north = rotate_xy(x, y, -yaw_offset)
    cosine = math.cos(math.radians(origin_lat))
    if abs(cosine) < 1e-8:
        raise ValueError("GPS origin is too close to a pole")
    return validate_latitude_longitude(
        origin_lat + math.degrees(north / EARTH_RADIUS_M),
        origin_lon + math.degrees(east / (EARTH_RADIUS_M * cosine)))


def depth_point_in_world(detection, projection, translation, quaternion):
    """Use the box centre, matching RegisteredDepthFusion's depth sample.

    ZED registered depth is optical Z, not radial range. P describes the
    rectified left image; TF must be evaluated at that image's source stamp.
    """
    fx, fy, cx, cy = (float(projection[i]) for i in (0, 5, 2, 6))
    z = detection.depth_m
    if not detection.depth_valid or min(fx, fy, z) <= 0:
        raise ValueError("valid registered depth and intrinsics are required")
    p = ((detection.x + detection.width * 0.5 - cx) * z / fx,
         (detection.y + detection.height * 0.5 - cy) * z / fy, z)
    values = (*p, *translation, *quaternion)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("non-finite camera geometry")
    norm = math.sqrt(sum(v*v for v in quaternion))
    if norm < 1e-8:
        raise ValueError("invalid camera rotation")
    x, y, z, w = (v/norm for v in quaternion)
    # Quaternion rotation: p + 2 w (q cross p) + 2 q cross (q cross p).
    cross = (y*p[2]-z*p[1], z*p[0]-x*p[2], x*p[1]-y*p[0])
    cross2 = (y*cross[2]-z*cross[1], z*cross[0]-x*cross[2], x*cross[1]-y*cross[0])
    return tuple(p[i]+2*w*cross[i]+2*cross2[i]+translation[i] for i in range(3))


def inside_vehicle_footprint(point, footprint):
    """The rectangular chassis envelope, in base_link, including its edges."""
    return (min(p[0] for p in footprint) <= point[0] <= max(p[0] for p in footprint)
            and min(p[1] for p in footprint) <= point[1] <= max(p[1] for p in footprint))


class FodGpsMemory:
    """Confirm the nearest identity, then retain only that object's world point.

    Six distinct consecutive observations within a 25 cm world-space radius
    establish a point. A lost box/depth never deletes an established point or
    selects a bystander. Closer stable observations refine the same point within 0.5 m of the
    original lock; slowly drifting depth cannot move it without bound.
    """

    def __init__(self, frames=6, max_range=6.0, stability_radius=0.25, crossing=0.8):
        self.frames = frames
        self.max_range = max_range
        self.stability_radius = stability_radius
        self.crossing = crossing
        self.reset()

    def reset(self):
        self.track_id = 0
        self.class_name = ""
        self.samples = deque(maxlen=self.frames)
        self.point = None
        self.direction = None
        self.last_stamp = None
        self.point_stamp = None
        self.range_m = None
        self.frozen = False
        self.initial_point = None
        self.point_range = None
        self.last_detection = None
        self.last_seen_stamp = None
        self.other_track_ids = set()

    def match_visible(self, stamp, observations):
        if self.last_detection is None or self.point is None:
            return False
        matches = same_target_observations(self.last_detection, observations, self.other_track_ids)
        if len(matches) != 1:
            return False
        d = matches[0]
        self.track_id = d.track_id
        self.last_detection = d
        self.last_seen_stamp = stamp
        self.other_track_ids.update(item.track_id for item in observations
                                    if item.track_id != d.track_id)
        return True

    def recently_visible(self, stamp, grace):
        return (self.point is not None and stamp is not None and self.last_seen_stamp is not None
                and 0 <= stamp-self.last_seen_stamp <= grace)

    def observe(self, stamp, observations, camera_xy):
        # observations: (PixelDetection, world_xyz). Repeated/stale frames
        # cannot manufacture the consecutive observations used for stability.
        if self.frozen or not math.isfinite(stamp) or stamp <= 0:
            return
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return
        self.last_stamp = stamp
        if self.point is not None:
            if not self.match_visible(stamp, [d for d, _p in observations]):
                self.samples.clear()
                return
            chosen = next(((d,p) for d,p in observations if d.track_id == self.track_id), None)
            if chosen is None:
                return
            d, p = chosen
            # Improve the same object's estimate as it gets closer. Every
            # update stays within 0.5 m of the first confirmation, not merely
            # within 0.5 m of the last update (which would allow unlimited drift).
            if (not d.depth_valid or not 0.8 < d.range_m < self.point_range - 0.10
                    or not all(math.isfinite(v) for v in p)
                    or math.dist(p[:2], self.initial_point[:2]) > 0.5):
                self.samples.clear()
                return
            self.samples.append(p)
            if len(self.samples) >= self.frames:
                centre = tuple(median(v[i] for v in self.samples) for i in range(3))
                if all(math.dist(v[:2], centre[:2]) <= self.stability_radius for v in self.samples):
                    self.point, self.point_stamp, self.point_range = centre, stamp, d.range_m
                    self.range_m = d.range_m
                    self.samples.clear()
            return
        candidates = [(d, p) for d, p in observations
                      if d.track_id > 0 and d.depth_valid
                      and 0 < d.range_m <= self.max_range
                      and all(math.isfinite(v) for v in (*p, d.range_m))]
        associated = (same_target_observations(self.last_detection, [d for d, _p in candidates],
                                              self.other_track_ids)
                      if self.last_detection is not None else ())
        matches = [(d, p) for d, p in candidates if d in associated]
        if not matches:
            if self.point is not None:
                return
            self.samples.clear()
            chosen = min(candidates, key=lambda item: (item[0].range_m, item[0].track_id), default=None)
            if chosen is None:
                self.track_id = 0
                return
        else:
            if len(matches) != 1:
                self.samples.clear()
                return
            chosen = matches[0]
        d, point = chosen
        self.track_id, self.class_name = d.track_id, d.class_name
        self.last_detection, self.last_seen_stamp = d, stamp
        self.other_track_ids.update(item.track_id for item, _p in candidates
                                    if item.track_id != d.track_id)
        self.range_m = d.range_m
        self.samples.append(point)
        if len(self.samples) < self.frames:
            return
        centre = tuple(median(p[i] for p in self.samples) for i in range(3))
        if any(math.dist(p[:2], centre[:2]) > self.stability_radius for p in self.samples):
            return
        if self.direction is None:
            dx, dy = centre[0]-camera_xy[0], centre[1]-camera_xy[1]
            length = math.hypot(dx, dy)
            if length < 1e-3:
                return
            self.direction = (dx/length, dy/length)
        self.point = centre
        self.initial_point = centre
        self.point_range = d.range_m
        self.samples.clear()
        self.point_stamp = stamp

    def snapshot(self, origin_lat, origin_lon, yaw_offset=0.0):
        if self.point is None:
            return None
        x, y = self.point[:2]
        end_x = x + self.crossing * self.direction[0]
        end_y = y + self.crossing * self.direction[1]
        lat, lon = local_to_gps(x, y, origin_lat, origin_lon, yaw_offset)
        end_lat, end_lon = local_to_gps(end_x, end_y, origin_lat, origin_lon, yaw_offset)
        return dict(track_id=self.track_id, class_name=self.class_name,
                    range_m=self.range_m, fod_x=x, fod_y=y,
                    fod_latitude=lat, fod_longitude=lon,
                    goal_x=end_x, goal_y=end_y,
                    goal_latitude=end_lat, goal_longitude=end_lon,
                    crossing_m=self.crossing, sample_stamp=self.point_stamp)


def depth_steering_curvature(detection, projection, translation, quaternion, max_curvature):
    """Pure pursuit through the observed FOD in base_link (x forward, y left)."""
    x, y, _z = depth_point_in_world(detection, projection, translation, quaternion)
    if x <= 0 or not math.isfinite(max_curvature) or max_curvature <= 0:
        raise ValueError("FOD must lie ahead of the chassis")
    curvature = 2.0*y / max(x*x+y*y, 0.25)
    return max(-max_curvature, min(max_curvature, curvature)), y
