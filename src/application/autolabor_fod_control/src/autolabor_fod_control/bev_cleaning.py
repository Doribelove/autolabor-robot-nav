"""Metric BEV observations and forward cleaning geometry, independent of ROS."""
import base64
from dataclasses import dataclass
import math
import zlib


def ground_objects(metadata, detections):
    """Use box centres in the *ungarnished* ground raster, including its mask.

    A planar representative point is an estimate, not object depth/height.
    Pixel centres follow GroundBev.pixel's half-pixel convention exactly.
    """
    width, height = metadata['dimensions']
    resolution = float(metadata['metres_per_pixel'])
    xmin, xmax = map(float, metadata['x_range_m'])
    ymin, ymax = map(float, metadata['y_range_m'])
    if (not all(math.isfinite(v) for v in (resolution, xmin, xmax, ymin, ymax))
            or not .005 <= resolution <= .2 or not 2 <= width <= 2048
            or not 2 <= height <= 2048 or xmin >= xmax or ymin >= ymax
            or abs(width-math.ceil((ymax-ymin)/resolution)) > 0
            or abs(height-math.ceil((xmax-xmin)/resolution)) > 0):
        raise ValueError('Invalid BEV raster geometry')
    decoder = zlib.decompressobj()
    mask = decoder.decompress(base64.b64decode(metadata['ground_support_zlib_b64'], validate=True),
                              width*height+1)
    if len(mask) != width*height or not decoder.eof or decoder.unused_data:
        raise ValueError('Invalid BEV ground support mask')
    objects = []
    for item in detections[:100]:
        x0, y0, x1, y1 = map(float, item['bbox'])
        confidence = float(item['confidence'])
        if (not all(math.isfinite(v) for v in (x0, y0, x1, y1, confidence))
                or not 0 <= confidence <= 1 or not 0 <= x0 < x1 <= width
                or not 0 <= y0 < y1 <= height):
            continue
        u, v = (x0+x1)/2, (y0+y1)/2
        if not mask[min(height-1, int(v))*width+min(width-1, int(u))]:
            continue
        x, y = xmax-(v+.5)*resolution, ymax-(u+.5)*resolution
        if xmin <= x <= xmax and ymin <= y <= ymax:
            objects.append(dict(x=x, y=y, label=str(item['class_name']), confidence=confidence))
    return objects


def transform_point(pose, point):
    x, y, yaw = pose
    px, py = point
    return (x+math.cos(yaw)*px-math.sin(yaw)*py,
            y+math.sin(yaw)*px+math.cos(yaw)*py)


def path_length(poses):
    return sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(poses, poses[1:]))


def sweep_candidates(robot, target, reference_offset=.52, lead=.8, tail=.35):
    """Forward base paths whose leading cleaning reference passes the point."""
    bearing = math.atan2(target[1]-robot[1], target[0]-robot[0])
    for delta in (0, math.pi/3, -math.pi/3, math.pi):
        yaw = math.atan2(math.sin(bearing+delta), math.cos(bearing+delta))
        c, s = math.cos(yaw), math.sin(yaw)
        start = (target[0]-(reference_offset+lead)*c,
                 target[1]-(reference_offset+lead)*s, yaw)
        end = (target[0]+(tail-reference_offset)*c,
               target[1]+(tail-reference_offset)*s, yaw)
        yield start, end


def forward_crossing(previous, current, target, reference_offset=.52, tolerance=.10):
    """Measured reference swept over point. Reject reverse, spin and TF jumps."""
    dx, dy = current[0]-previous[0], current[1]-previous[1]
    yaw_delta = math.atan2(math.sin(current[2]-previous[2]), math.cos(current[2]-previous[2]))
    if (not all(math.isfinite(v) for v in (*previous, *current, *target))
            or not .001 <= math.hypot(dx, dy) <= .25 or abs(yaw_delta) > .25
            or dx*math.cos(previous[2])+dy*math.sin(previous[2]) <= .001):
        return False
    a = transform_point(previous, (reference_offset, 0))
    b = transform_point(current, (reference_offset, 0))
    vx, vy = b[0]-a[0], b[1]-a[1]
    norm = vx*vx+vy*vy
    if norm < 1e-12:
        return False
    t = max(0., min(1., ((target[0]-a[0])*vx+(target[1]-a[1])*vy)/norm))
    return math.hypot(target[0]-a[0]-t*vx, target[1]-a[1]-t*vy) <= tolerance


@dataclass
class Target:
    identity: int
    x: float
    y: float
    label: str
    observed: float


class VisibleTargets:
    """Fresh observations while idle; keep unfinished map targets within a round.

Visibility still expires after ttl. A cleaning round's pending locations do not:
the vehicle can turn away from a target while cleaning another one.
"""
    def __init__(self, radius=.20, ttl=2.):
        self.radius, self.ttl = radius, ttl
        self.targets, self.passed = {}, []
        self.next_id = 1
        self.retain_missing = False

    def begin_round(self, now):
        self.targets = {t.identity: t for t in self.visible(now)}
        self.passed.clear()
        self.retain_missing = True

    def end_round(self, now):
        self.retain_missing = False
        self.targets = {t.identity: t for t in self.visible(now)}
        self.passed.clear()

    def pending(self):
        """Unfinished locations, including targets temporarily out of view."""
        return list(self.targets.values())

    def update(self, observations, stamp):
        used = set()
        accepted = []
        for obj in observations[:100]:
            x, y = obj['x'], obj['y']
            if any(math.hypot(x-p[0], y-p[1]) <= self.radius for p in accepted):
                continue
            accepted.append((x, y))
            if any(math.hypot(x-p[0], y-p[1]) <= self.radius for p in self.passed):
                continue
            candidates = [t for t in self.targets.values()
                          if t.identity not in used and math.hypot(t.x-x, t.y-y) <= self.radius]
            if candidates:
                target = min(candidates, key=lambda t: math.hypot(t.x-x, t.y-y))
                target.x, target.y, target.label, target.observed = x, y, obj['label'], stamp
            else:
                target = Target(self.next_id, x, y, obj['label'], stamp)
                self.targets[target.identity] = target
                self.next_id += 1
            used.add(target.identity)
        if not self.retain_missing:
            self.targets = {i: t for i, t in self.targets.items() if stamp-t.observed <= self.ttl}

    def visible(self, now):
        return [t for t in self.targets.values() if 0 <= now-t.observed <= self.ttl]

    def complete(self, target):
        self.passed.append((target.x, target.y))
        self.targets = {i: t for i, t in self.targets.items()
                        if math.hypot(t.x-target.x, t.y-target.y) > self.radius}
