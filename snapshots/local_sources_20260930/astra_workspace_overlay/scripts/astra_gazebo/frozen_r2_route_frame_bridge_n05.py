"""Pure SE(2) input view for a future, separately identified frozen-R2 wrapper.

No ROS node, TF broadcast, inference, clock read, source-time substitution or
algorithm change. The old policy adds spawn_world_x=-5.4 to both odometry and
laser TF x. Its private ``odom`` view must therefore be spawn-relative route
coordinates, NOT route coordinates. Real ROS frames remain untouched.

Only rigid copies of spawn(-5.4,0,0), gate(-4,0,0), goal(5,0,0) and a forward
straight nine-metre nominal task reference are supported. The separate common
guide may detour and remains audit-only. Curved/longer nominal tasks need
another method, not a change of coordinate names. Static boxes come exclusively from
the same bound canonical-route raster; never inverse-rotate world AABBs.
"""
from dataclasses import dataclass
import hashlib
import json
import math

from route_world_adapter import RouteFrame, wrap_angle

METHOD = 'frozen_r2_v060_sensor_route_se2_n05_v1'
SCHEMA = 'astra_frozen_r2_route_frame_bridge_n05_v1'
CANONICAL_MAP_FRAME = 'r2_canonical_route'
PRIVATE_SCOPE = 'inprocess_legacy_alias_never_publish_as_real_ros_frame'
MAX_AGE_NS = 200_000_000
SPAWN = (-5.4, 0., 0.)
GATE = (-4., 0., 0.)
GOAL = (5., 0., 0.)
GEOMETRY_TOLERANCE_M_RAD = 1e-8


def _real(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('finite built-in real required; bool/coercion forbidden')
    return value


def _vector(value, length):
    if type(value) is not tuple or len(value) != length:
        raise ValueError('fixed-size immutable numeric tuple required')
    for item in value: _real(item)
    return value


def _ns(value, positive=False):
    if type(value) is not int or not (1 if positive else 0) <= value < 2**63:
        raise ValueError('typed nonnegative int64 nanoseconds required')
    return value


def _frame(value):
    if type(value) is not str or not value or value.startswith('/') or any(c.isspace() for c in value):
        raise ValueError('explicit canonical frame name required')


@dataclass(frozen=True)
class SourceMetadata:
    source_ns: int
    received_source_ns: int
    available_source_ns: int
    receipt_steady_ns: int

    def __post_init__(self):
        _ns(self.source_ns, True); _ns(self.received_source_ns)
        _ns(self.available_source_ns, True); _ns(self.receipt_steady_ns, True)
        if self.available_source_ns < max(self.source_ns, self.received_source_ns):
            raise ValueError('availability cannot precede source/receipt')

    def require(self, now_ns):
        _ns(now_ns, True)
        if (now_ns < self.available_source_ns or now_ns-self.source_ns > MAX_AGE_NS or
                abs(self.received_source_ns-self.source_ns) > MAX_AGE_NS):
            raise ValueError('unavailable/future/stale original source')


@dataclass(frozen=True)
class SE2:
    x: float = 0.
    y: float = 0.
    yaw: float = 0.

    def __post_init__(self):
        _vector((self.x, self.y, self.yaw), 3)

    def point(self, xy):
        _vector(xy, 2)
        return RouteFrame(self.x, self.y, self.yaw).point_to_world(xy)

    def pose(self, xy_yaw):
        _vector(xy_yaw, 3)
        return RouteFrame(self.x, self.y, self.yaw).pose_to_world(xy_yaw)

    def inverse(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return SE2(-c*self.x-s*self.y, s*self.x-c*self.y, -self.yaw)


@dataclass(frozen=True)
class Pose3:
    xyz: tuple
    xyzw: tuple

    def __post_init__(self):
        _vector(self.xyz, 3); _vector(self.xyzw, 4)
        if abs(sum(q*q for q in self.xyzw)-1.) > 1e-5:
            raise ValueError('unit measured quaternion required; no normalization repair')

    def transformed(self, transform):
        if type(transform) is not SE2: raise ValueError('SE2 transform required')
        x, y = transform.point(self.xyz[:2])
        qx, qy, qz, qw = self.xyzw
        s, c = math.sin(transform.yaw/2), math.cos(transform.yaw/2)
        return Pose3((x, y, self.xyz[2]), (c*qx-s*qy, c*qy+s*qx, c*qz+s*qw, c*qw-s*qz))

    def planar(self):
        x, y, z, w = self.xyzw
        return self.xyz[0], self.xyz[1], math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


@dataclass(frozen=True)
class BodyTwist:
    linear_xyz: tuple
    angular_xyz: tuple

    def __post_init__(self):
        _vector(self.linear_xyz, 3); _vector(self.angular_xyz, 3)


@dataclass(frozen=True)
class TransformSample:
    metadata: SourceMetadata
    parent_frame: str
    child_frame: str
    pose: Pose3

    def __post_init__(self):
        if type(self.metadata) is not SourceMetadata or type(self.pose) is not Pose3:
            raise ValueError('typed source metadata and pose required')
        _frame(self.parent_frame); _frame(self.child_frame)


@dataclass(frozen=True)
class OdometrySample(TransformSample):
    body_twist: BodyTwist

    def __post_init__(self):
        super().__post_init__()
        if type(self.body_twist) is not BodyTwist: raise ValueError('body-frame twist required')


@dataclass(frozen=True)
class LaserSample:
    metadata: SourceMetadata
    frame_id: str
    ranges: tuple
    angle_min: float
    angle_max: float
    angle_increment: float
    range_min: float
    range_max: float
    time_increment: float = 0.
    scan_time: float = 0.
    intensities: tuple = ()

    def __post_init__(self):
        if type(self.metadata) is not SourceMetadata: raise ValueError('typed scan metadata required')
        _frame(self.frame_id)
        _vector((self.angle_min, self.angle_max, self.angle_increment, self.range_min,
                 self.range_max, self.time_increment, self.scan_time), 7)
        if self.angle_increment <= 0 or not 0 <= self.range_min < self.range_max or min(self.time_increment, self.scan_time) < 0:
            raise ValueError('invalid local scan geometry')
        if type(self.ranges) is not tuple or len(self.ranges) < 2 or any(
                type(v) not in (int, float) or math.isnan(v) or v < 0 for v in self.ranges):
            raise ValueError('immutable nonnegative scan ranges; positive infinity is preserved')
        if type(self.intensities) is not tuple: raise ValueError('immutable intensities required')
        for v in self.intensities: _real(v)


@dataclass(frozen=True)
class LegacyPoseView:
    metadata: SourceMetadata
    source_parent_frame: str
    source_child_frame: str
    pose: Pose3
    body_twist: object = None
    parent_alias: str = 'odom'
    coordinate_scope: str = PRIVATE_SCOPE


@dataclass(frozen=True)
class GateView:
    metadata: SourceMetadata
    event_json: bytes
    original_event_sha256: str
    coordinate_scope: str = PRIVATE_SCOPE

    def event(self):
        return json.loads(self.event_json)


class FrozenR2RouteFrameBridge:
    def __init__(self, *, world_from_route, world_from_odom, spawn_world, gate_world,
                 goal_world, nominal_reference_world, guide_world, run_id, seed, method=METHOD,
                 world_frame='map', odom_frame='odom'):
        if type(world_from_route) is not SE2 or type(world_from_odom) is not SE2:
            raise ValueError('explicit rigid frame transforms required')
        _frame(world_frame); _frame(odom_frame)
        if world_frame == odom_frame or type(run_id) is not str or not run_id or type(seed) is not int or seed <= 0 or method != METHOD:
            raise ValueError('separate real frames and new bridge method identity required')
        self.world_from_route, self.world_from_odom = world_from_route, world_from_odom
        self.world_frame, self.odom_frame = world_frame, odom_frame
        self.identity = (run_id, seed, method)
        for actual, expected in ((spawn_world, SPAWN), (gate_world, GATE), (goal_world, GOAL)):
            route = world_from_route.inverse().pose(_vector(actual, 3))
            if math.dist(route[:2], expected[:2]) > GEOMETRY_TOLERANCE_M_RAD or abs(wrap_angle(route[2]-expected[2])) > GEOMETRY_TOLERANCE_M_RAD:
                raise ValueError('only rigid copies of frozen spawn/gate/goal supported')
        if type(nominal_reference_world) is not tuple or len(nominal_reference_world) < 2:
            raise ValueError('explicit immutable straight nominal task reference required')
        reference = tuple(world_from_route.inverse().point(_vector(p, 2)) for p in nominal_reference_world)
        if (math.dist(reference[0], GATE[:2]) > GEOMETRY_TOLERANCE_M_RAD or
                math.dist(reference[-1], GOAL[:2]) > GEOMETRY_TOLERANCE_M_RAD or
                any(abs(p[1]) > GEOMETRY_TOLERANCE_M_RAD for p in reference) or
                any(b[0] <= a[0] for a, b in zip(reference, reference[1:]))):
            raise ValueError('curved, reversed, extended or rescaled nominal task is not SE2 support')
        if type(guide_world) is not tuple or len(guide_world) < 2:
            raise ValueError('explicit audit-only common guide required')
        guide = tuple(world_from_route.inverse().point(_vector(p, 2)) for p in guide_world)
        if (math.dist(guide[0], GATE[:2]) > GEOMETRY_TOLERANCE_M_RAD or
                math.dist(guide[-1], GOAL[:2]) > GEOMETRY_TOLERANCE_M_RAD or
                any(a == b for a, b in zip(guide, guide[1:]))):
            raise ValueError('common guide endpoints/degenerate segment mismatch')
        self.guide_route_points = guide
        self.guide_length_m = sum(math.dist(a, b) for a, b in zip(guide, guide[1:]))
        self.guide_use = 'audit_only'
        self.full_polyline_consumed = False

    def _route_pose(self, sample):
        if sample.parent_frame == self.world_frame: world = sample.pose
        elif sample.parent_frame == self.odom_frame: world = sample.pose.transformed(self.world_from_odom)
        else: raise ValueError('unregistered real parent frame; no implicit TF lookup')
        return world.transformed(self.world_from_route.inverse())

    def _private_view(self, sample, now_ns):
        sample.metadata.require(now_ns)
        route = self._route_pose(sample)
        # Exactly the inverse of frozen policy_pose/_scan's +spawn_world_x.
        private = Pose3((route.xyz[0]-SPAWN[0], route.xyz[1], route.xyz[2]), route.xyzw)
        return LegacyPoseView(sample.metadata, sample.parent_frame, sample.child_frame,
                              private, getattr(sample, 'body_twist', None))

    def odometry(self, sample, now_ns):
        if type(sample) is not OdometrySample or sample.child_frame != 'base_link':
            raise ValueError('actual base_link odometry required')
        return self._private_view(sample, now_ns)

    def laser_transform(self, sample, scan, now_ns):
        self.scan(scan, now_ns)
        if type(sample) is not TransformSample or sample.child_frame != scan.frame_id or sample.metadata.source_ns != scan.metadata.source_ns:
            raise ValueError('exact requested scan source TF required; latest is not a substitute')
        return self._private_view(sample, now_ns)

    def health_transform(self, sample, now_ns):
        """Fresh actually stamped health-only TF; never substitutes scan TF."""
        if type(sample) is not TransformSample or sample.child_frame != 'laser_link':
            raise ValueError('actual laser TF required')
        return self._private_view(sample, now_ns)

    def scan(self, sample, now_ns):
        if type(sample) is not LaserSample or sample.frame_id != 'laser_link':
            raise ValueError('actual local laser_link scan required')
        sample.metadata.require(now_ns)
        return sample  # All local data and metadata unchanged, no resampling.

    @staticmethod
    def command(body_twist):
        if type(body_twist) is not BodyTwist: raise ValueError('body-frame command required')
        return body_twist  # No rotation/scaling/steering projection.

    def goal(self, world_pose):
        result = self.world_from_route.inverse().pose(_vector(world_pose, 3))
        if math.dist(result[:2], GOAL[:2]) > GEOMETRY_TOLERANCE_M_RAD or abs(wrap_angle(result[2])) > GEOMETRY_TOLERANCE_M_RAD:
            raise ValueError('changed goal/length requires a different method')
        return GOAL  # Declared design constant, not snapped sensor odometry.

    def gate(self, event, metadata, now_ns):
        keys = {'schema', 'run_id', 'seed', 'method', 'source_ns', 'source', 'pose_map',
                'body_twist', 'activation_source_ns', 'goal_pose', 'deadline_source_ns', 'deadline_ns'}
        if type(event) is not dict or set(event) != keys or type(metadata) is not SourceMetadata:
            raise ValueError('closed shared gate event and actual availability required')
        if event['schema'] != 'astra_common_navigation_v1' or (event['run_id'], event['seed'], event['method']) != self.identity or type(event['seed']) is not int:
            raise ValueError('public new-method gate identity mismatch')
        metadata.require(now_ns)
        for key in ('source_ns', 'activation_source_ns', 'deadline_source_ns', 'deadline_ns'): _ns(event[key], True)
        if (event['source_ns'] != metadata.source_ns or event['source'] != 'actual_odom_header_first_crossing' or
                not 5_000_000_000 <= event['activation_source_ns'] <= min(5_200_000_000, event['source_ns']) or
                event['deadline_ns'] != event['deadline_source_ns'] or event['deadline_ns'] != event['source_ns']+22_000_000_000):
            raise ValueError('original gate source/activation/deadline changed')
        for key in ('pose_map', 'body_twist', 'goal_pose'):
            if type(event[key]) is not list or len(event[key]) != 3: raise ValueError('actual gate numeric triple required')
            _vector(tuple(event[key]), 3)
        pose = self.world_from_route.inverse().pose(tuple(event['pose_map']))
        if abs(pose[0]+4) > .03 or abs(pose[1]) > .03 or abs(pose[2]) > .03 or abs(event['body_twist'][0]-.8) > .04:
            raise ValueError('actual gate outside inherited tolerances')
        goal = self.goal(tuple(event['goal_pose']))
        original = json.dumps(event, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
        output = json.loads(original); output['pose_map'] = list(pose); output['goal_pose'] = list(goal)
        return GateView(metadata, json.dumps(output, sort_keys=True, separators=(',', ':'), allow_nan=False).encode(), hashlib.sha256(original).hexdigest())


@dataclass(frozen=True)
class CanonicalStaticPrior:
    image_sha256: str
    metadata_sha256: str
    boxes_xy_length_width: tuple
    blocked_cell_count: int
    frame_id: str = CANONICAL_MAP_FRAME

    def __post_init__(self):
        for digest in (self.image_sha256, self.metadata_sha256):
            if type(digest) is not str or len(digest) != 64 or any(c not in '0123456789abcdef' for c in digest):
                raise ValueError('canonical raster SHA256 bindings required')
        if self.frame_id != CANONICAL_MAP_FRAME or type(self.boxes_xy_length_width) is not tuple or type(self.blocked_cell_count) is not int or self.blocked_cell_count < 0:
            raise ValueError('typed canonical raster prior required')
        for row in self.boxes_xy_length_width:
            _vector(row, 4)
            if row[2] <= 0 or row[3] <= 0: raise ValueError('positive rectangle dimensions required')

    def legacy_scene_fields(self):
        return dict(static_boxes=[dict(zip(('x', 'y', 'length', 'width'), row)) for row in self.boxes_xy_length_width],
                    static_circles=[], corridor_half_width_m=None,
                    r2_static_input_adapter=dict(schema=SCHEMA, canonical_map_frame=self.frame_id,
                        image_sha256=self.image_sha256, metadata_sha256=self.metadata_sha256,
                        raster_union_exact=True, legacy_implicit_y_wall_m=20.,
                        unknown_outside_map_semantics_equal=False, full_polyline_consumed=False))

    def world_polygons(self, world_from_route):
        if type(world_from_route) is not SE2: raise ValueError('explicit canonical-to-world SE2 required')
        return tuple(tuple(world_from_route.point((x+dx*length/2, y+dy*width/2))
                     for dx, dy in ((-1., -1.), (1., -1.), (1., 1.), (-1., 1.)))
                     for x, y, length, width in self.boxes_xy_length_width)


def canonical_static_prior(pgm_bytes, yaml_bytes, *, image_sha256, metadata_sha256,
                           frame_id=CANONICAL_MAP_FRAME):
    """Bind canonical raster bytes and reuse the exact frozen rectangle union.

    No world boxes accepted, no raster rotation/resampling, no clipping unknown
    cells. Future common-map publication must use these same canonical cells
    and its declared rigid frame relation; world geometry equivalence remains
    a required separate scene audit, not established by a frame label.
    """
    if type(pgm_bytes) is not bytes or type(yaml_bytes) is not bytes or frame_id != CANONICAL_MAP_FRAME:
        raise ValueError('canonical route-frame raster byte inputs required')
    if hashlib.sha256(pgm_bytes).hexdigest() != image_sha256 or hashlib.sha256(yaml_bytes).hexdigest() != metadata_sha256:
        raise ValueError('canonical raster byte binding mismatch')
    from prepare_navigation_n01_scene import legacy_static_boxes
    from common_route import SharedStaticMap
    from frozen_r2_sensor_lifecycle import verify_static_union
    boxes, audit = legacy_static_boxes(pgm_bytes, yaml_bytes)
    verify_static_union(dict(static_boxes=boxes, static_circles=[], corridor_half_width_m=None), SharedStaticMap(pgm_bytes, yaml_bytes))
    return CanonicalStaticPrior(image_sha256, metadata_sha256,
        tuple(tuple(box[k] for k in ('x', 'y', 'length', 'width')) for box in boxes), audit['blocked_pixel_count'])
