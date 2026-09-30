"""Independent collision-geometry audit; no ROS, planner or simulator imports.

Distances use world XY projections of individual URDF collision primitives.
Box projections are exact convex hulls of eight transformed corners. Cylinders
use circumscribed regular prisms; ``approximation_error_m`` bounds their outward
Hausdorff error, including after rotation/projection. Keeping components separate
avoids filling the empty space between robot parts with a convex chassis hull.

A negative polygon clearance is minus the shortest *2D translation* that makes
two convex polygons nonpenetrating. This containment-aware SAT depth is not an
intersection width, a contact force, or a 3D penetration depth. Compound output
is the minimum component-pair value, not the compound's minimum translation.
XY overlap and overlapping Z intervals are only potential 3D intersection:
tilted shapes can be disjoint despite both tests passing. Gazebo contacts remain
separate evidence. No formal safety or calibrated real-vehicle claim is made.
"""
from dataclasses import asdict, dataclass
import math
import xml.etree.ElementTree as ET
from typing import Mapping, Optional, Sequence, Tuple

Point2 = Tuple[float, float]
Point3 = Tuple[float, float, float]


def _finite(values, length=None):
    result = tuple(float(x) for x in values)
    if length is not None and len(result) != length:
        raise ValueError('unexpected vector dimension')
    if not all(math.isfinite(x) for x in result):
        raise ValueError('nonfinite geometry')
    return result


def _positive(value, name):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(name + ' must be positive and finite')
    return value


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)


@dataclass(frozen=True)
class Pose3:
    """World or relative rigid pose, metres; quaternion ordering XYZW."""
    xyz: Point3 = (0.0, 0.0, 0.0)
    xyzw: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)

    def __post_init__(self):
        xyz, q = _finite(self.xyz, 3), _finite(self.xyzw, 4)
        norm = math.sqrt(sum(x*x for x in q))
        if norm < 1e-12:
            raise ValueError('zero quaternion')
        object.__setattr__(self, 'xyz', xyz)
        object.__setattr__(self, 'xyzw', tuple(x/norm for x in q))

    @classmethod
    def from_xyz_rpy(cls, xyz=(0, 0, 0), rpy=(0, 0, 0)):
        roll, pitch, yaw = _finite(rpy, 3)
        sr, cr = math.sin(roll/2), math.cos(roll/2)
        sp, cp = math.sin(pitch/2), math.cos(pitch/2)
        sy, cy = math.sin(yaw/2), math.cos(yaw/2)
        return cls(xyz, (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                         cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy))

    @classmethod
    def from_ros(cls, pose):
        """Duck-typed geometry_msgs/Pose; caller must verify frame and timestamp."""
        p, q = pose.position, pose.orientation
        return cls((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    @property
    def rotation(self):
        x, y, z, w = self.xyzw
        return ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
                (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
                (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))

    def rotate(self, point):
        point = _finite(point, 3)
        return tuple(sum(row[j]*point[j] for j in range(3)) for row in self.rotation)

    def transform(self, point):
        rotated = self.rotate(point)
        return tuple(rotated[i]+self.xyz[i] for i in range(3))

    def compose(self, child):
        return Pose3(self.transform(child.xyz), _qmul(self.xyzw, child.xyzw))


def _cross(o, a, b):
    return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])


def convex_hull(points):
    """CCW monotone-chain hull. Reject degenerate/nonfinite projections."""
    points = sorted(set(_finite(p, 2) for p in points))
    if len(points) < 3:
        raise ValueError('projection has fewer than three distinct points')
    lower, upper = [], []
    for p in points:
        while len(lower) > 1 and _cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(points):
        while len(upper) > 1 and _cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    hull = tuple(lower[:-1]+upper[:-1])
    if len(hull) < 3 or abs(_area2(hull)) < 1e-14:
        raise ValueError('degenerate projected polygon')
    return hull


def _area2(polygon):
    return sum(a[0]*b[1]-a[1]*b[0] for a, b in _edges(polygon))


def _edges(polygon):
    return zip(polygon, polygon[1:]+polygon[:1])


def _validated_polygon(points):
    polygon = tuple(_finite(p, 2) for p in points)
    if len(polygon) < 3 or abs(_area2(polygon)) < 1e-14:
        raise ValueError('degenerate polygon')
    hull = convex_hull(polygon)
    # Do not silently turn concave input or a self-crossing order into a hull.
    if not math.isclose(abs(_area2(polygon)), abs(_area2(hull)), rel_tol=1e-10, abs_tol=1e-12):
        raise ValueError('polygon must be convex and boundary-ordered')
    return hull


def _point_segment_distance(p, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    denominator = dx*dx+dy*dy
    t = 0.0 if denominator == 0 else max(0.0, min(1.0, ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/denominator))
    return math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dy)


@dataclass(frozen=True)
class PolygonClearance:
    signed_clearance_m: float
    intersects_or_touches: bool
    penetration_depth_2d_m: float
    definition: str = 'positive Euclidean gap; negative minimum 2D separating translation; touching zero'


def convex_polygon_clearance(first, second, tolerance_m=1e-10):
    """SAT intersection and exact nonintersecting polygon edge distance.

    If A is inside B, depth uses min(maxA-minB, maxB-minA), not the
    intersection interval width. Numeric contact tolerance is reported by the
    caller; it never inflates a positive distance into negative penetration.
    """
    a, b = _validated_polygon(first), _validated_polygon(second)
    tolerance_m = float(tolerance_m)
    if not math.isfinite(tolerance_m) or tolerance_m < 0:
        raise ValueError('invalid numerical tolerance')
    separated, depth = False, float('inf')
    for polygon in (a, b):
        for p, q in _edges(polygon):
            nx, ny = -(q[1]-p[1]), q[0]-p[0]
            norm = math.hypot(nx, ny)
            nx, ny = nx/norm, ny/norm
            pa, pb = [x*nx+y*ny for x, y in a], [x*nx+y*ny for x, y in b]
            low_a, high_a, low_b, high_b = min(pa), max(pa), min(pb), max(pb)
            if high_a < low_b or high_b < low_a:
                separated = True
            depth = min(depth, high_a-low_b, high_b-low_a)
    if separated:
        gap = min([_point_segment_distance(p, u, v) for p in a for u, v in _edges(b)] +
                  [_point_segment_distance(p, u, v) for p in b for u, v in _edges(a)])
        return PolygonClearance(gap, gap <= tolerance_m, 0.0)
    depth = max(0.0, depth)
    return PolygonClearance(-depth if depth else 0.0, True, depth)


@dataclass(frozen=True)
class CollisionProjection:
    name: str
    link_name: str
    polygon_xy: Tuple[Point2, ...]
    z_min_m: float
    z_max_m: float
    primitive_type: str
    approximation_error_m: float
    pose_source: str
    approximation: str

    def __post_init__(self):
        object.__setattr__(self, 'polygon_xy', _validated_polygon(self.polygon_xy))
        low, high, error = _finite((self.z_min_m, self.z_max_m, self.approximation_error_m), 3)
        if low > high or error < 0:
            raise ValueError('invalid height interval or approximation error')
        if not self.name:
            raise ValueError('collision name required')

    def to_dict(self):
        return asdict(self)


def _projection(name, link_name, vertices, primitive_type, error, pose_source, approximation):
    return CollisionProjection(str(name), str(link_name), convex_hull([p[:2] for p in vertices]),
                               min(p[2] for p in vertices), max(p[2] for p in vertices),
                               primitive_type, float(error), pose_source, approximation)


def project_box(name, size_xyz, pose=Pose3(), link_name='', pose_source='world_pose'):
    size = _finite(size_xyz, 3)
    for x in size:
        _positive(x, 'box size')
    vertices = [pose.transform((sx*size[0]/2, sy*size[1]/2, sz*size[2]/2))
                for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    return _projection(name, link_name, vertices, 'box', 0, pose_source, 'exact eight-corner projection')


def project_cylinder(name, radius_m, length_m, pose=Pose3(), sides=48,
                     link_name='', pose_source='world_pose'):
    radius, length = _positive(radius_m, 'radius'), _positive(length_m, 'length')
    if isinstance(sides, bool) or not isinstance(sides, int) or sides < 8:
        raise ValueError('cylinder requires an integer sides >= 8')
    outer_radius = radius/math.cos(math.pi/sides)
    vertices = [pose.transform((outer_radius*math.cos(2*math.pi*k/sides),
                                outer_radius*math.sin(2*math.pi*k/sides), z))
                for k in range(sides) for z in (-length/2, length/2)]
    projection = _projection(name, link_name, vertices, 'cylinder', outer_radius-radius, pose_source,
                             'outer circumscribed {}-sided XY projection; error r*(sec(pi/n)-1); exact analytic Z extrema'.format(sides))
    # Wheel-ground QC must not mistake polygon discretization for penetration.
    # The true cylinder's vertical support is analytic for any roll/pitch/yaw.
    rotation = pose.rotation
    half_z = radius*math.hypot(rotation[2][0], rotation[2][1])+length/2*abs(rotation[2][2])
    return CollisionProjection(projection.name, projection.link_name, projection.polygon_xy,
                               pose.xyz[2]-half_z, pose.xyz[2]+half_z,
                               projection.primitive_type, projection.approximation_error_m,
                               projection.pose_source, projection.approximation)


def project_actor_box(name, pose, size_xyz=(0.72, 0.72, 1.0)):
    """Convenience only: pose is collision-centre pose, never actor ground pose.

    Default XY is the historical 0.72 m actor box. Caller MUST supply the actual
    SDF collision height/origin when different; no world model is inferred.
    """
    return project_box(name, size_xyz, pose, pose_source='actor_collision_world_pose')


@dataclass(frozen=True)
class PairClearance:
    first: str
    second: str
    physical_projected_clearance_m: float
    conservative_projected_clearance_m: float
    margin_m: float
    penetration_depth_2d_m: float
    projected_intersection_or_touch: bool
    z_interval_overlap_m: float
    z_interval_gap_m: float
    z_intervals_intersect_or_touch: bool
    potential_3d_intersection: bool
    geometry_approximation_error_bound_m: float
    numerical_tolerance_m: float
    interpretation: str = 'XY projection plus Z interval screen; potential intersection is not a measured 3D contact'

    def to_dict(self):
        return asdict(self)


def pair_clearance(first, second, margin_m=0.33, tolerance_m=1e-10):
    margin_m = float(margin_m)
    if not math.isfinite(margin_m) or margin_m < 0:
        raise ValueError('invalid conservative margin')
    planar = convex_polygon_clearance(first.polygon_xy, second.polygon_xy, tolerance_m)
    overlap = min(first.z_max_m, second.z_max_m)-max(first.z_min_m, second.z_min_m)
    z_intersection = overlap >= -tolerance_m
    return PairClearance(first.name, second.name, planar.signed_clearance_m,
                         planar.signed_clearance_m-margin_m, margin_m,
                         planar.penetration_depth_2d_m, planar.intersects_or_touches,
                         max(0.0, overlap), max(0.0, -overlap), z_intersection,
                         planar.intersects_or_touches and z_intersection,
                         first.approximation_error_m+second.approximation_error_m,
                         float(tolerance_m))


def compound_clearance(first, second, margin_m=0.33, tolerance_m=1e-10):
    """Audit all primitive pairs, retaining pairs with disjoint height intervals.

    The unfiltered XY minimum helps compare to a legacy 2D metric. The
    Z-conditioned XY minimum removes definitely separated heights, but still
    must not be called true 3D Euclidean clearance or physical contact.
    Empty obstacle sets produce null minima (JSON finite), never +infinity.
    """
    first, second = tuple(first), tuple(second)
    if not first:
        raise ValueError('first compound must have collision components')
    if len({p.name for p in first}) != len(first) or len({p.name for p in second}) != len(second):
        raise ValueError('duplicate collision component identity')
    pairs = [pair_clearance(a, b, margin_m, tolerance_m) for a in first for b in second]
    eligible = [p for p in pairs if p.z_intervals_intersect_or_touch]
    minimum = min(pairs, key=lambda p: p.physical_projected_clearance_m) if pairs else None
    height_minimum = min(eligible, key=lambda p: p.physical_projected_clearance_m) if eligible else None
    return {
        'definition': 'minimum signed convex-component XY gap, not compound translation depth or 3D contact',
        'first_component_count': len(first), 'second_component_count': len(second),
        'pair_count': len(pairs), 'margin_m': float(margin_m),
        'minimum_physical_projected_clearance_m': minimum.physical_projected_clearance_m if minimum else None,
        'minimum_conservative_projected_clearance_m': minimum.conservative_projected_clearance_m if minimum else None,
        'minimum_z_conditioned_projected_clearance_m': height_minimum.physical_projected_clearance_m if height_minimum else None,
        'worst_projected_pair': [minimum.first, minimum.second] if minimum else None,
        'potential_3d_intersection_pairs': [[p.first, p.second] for p in pairs if p.potential_3d_intersection],
        'pairs': [p.to_dict() for p in pairs],
    }


def _xml_vector(text, default, length=3):
    return _finite(text.split() if text is not None else default, length)


def _origin(element):
    origin = element.find('origin')
    return Pose3() if origin is None else Pose3.from_xyz_rpy(
        _xml_vector(origin.get('xyz'), (0, 0, 0)), _xml_vector(origin.get('rpy'), (0, 0, 0)))


@dataclass(frozen=True)
class _Collision:
    name: str
    link: str
    origin: Pose3
    kind: str
    dimensions: tuple


@dataclass(frozen=True)
class _Joint:
    name: str
    parent: str
    child: str
    kind: str
    origin: Pose3
    axis: Point3
    mimic: Optional[tuple]


class RobotCollisionModel:
    """Expanded URDF collision tree; uses actual link poses or actual joint states.

    Missing movable-joint values fail closed: no implicit zero steering/wheel
    position. A supplied world pose for a collision link takes priority and
    avoids FK for that link. Joint timestamps and Gazebo/TF consistency are a
    caller responsibility; this library cannot verify time from bare values.
    Unsupported mesh/sphere/planar/floating geometry fails explicitly.
    """
    def __init__(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        if root.tag != 'robot':
            raise ValueError('expected expanded URDF robot')
        self.links = {e.get('name') for e in root.findall('link')}
        if None in self.links or len(self.links) != len(root.findall('link')):
            raise ValueError('missing or duplicate link name')
        self.collisions, self.joints, self.by_child = [], {}, {}
        for link in root.findall('link'):
            name = link.get('name')
            for index, collision in enumerate(link.findall('collision')):
                geometry = collision.find('geometry')
                if geometry is None or len(geometry) != 1:
                    raise ValueError('collision must contain one geometry')
                primitive = geometry[0]
                if primitive.tag == 'box':
                    dimensions = _xml_vector(primitive.get('size'), ())
                elif primitive.tag == 'cylinder':
                    dimensions = (float(primitive.get('radius')), float(primitive.get('length')))
                else:
                    raise ValueError('unsupported collision geometry: ' + primitive.tag)
                for dimension in dimensions:
                    _positive(dimension, 'primitive dimension')
                collision_id = name + '/' + collision.get('name', 'collision_{}'.format(index))
                if any(c.name == collision_id for c in self.collisions):
                    raise ValueError('duplicate collision name')
                self.collisions.append(_Collision(collision_id, name, _origin(collision), primitive.tag, dimensions))
        for element in root.findall('joint'):
            name, kind = element.get('name'), element.get('type')
            if not name or name in self.joints or kind not in {'fixed', 'revolute', 'continuous', 'prismatic'}:
                raise ValueError('unsupported or duplicate joint')
            parent, child = element.find('parent').get('link'), element.find('child').get('link')
            if parent not in self.links or child not in self.links or child in self.by_child:
                raise ValueError('invalid joint topology')
            axis_el = element.find('axis')
            axis = _xml_vector(axis_el.get('xyz') if axis_el is not None else None, (1, 0, 0))
            norm = math.sqrt(sum(x*x for x in axis))
            if kind != 'fixed' and norm < 1e-12:
                raise ValueError('zero joint axis')
            axis = tuple(x/norm for x in axis) if norm else (1, 0, 0)
            mimic_el = element.find('mimic')
            mimic = None if mimic_el is None else (mimic_el.get('joint'), float(mimic_el.get('multiplier', '1')),
                                                   float(mimic_el.get('offset', '0')))
            if mimic is not None:
                _finite(mimic[1:])
            joint = _Joint(name, parent, child, kind, _origin(element), axis, mimic)
            self.joints[name] = joint
            self.by_child[child] = joint
        roots = self.links-set(self.by_child)
        if len(roots) != 1 or not self.collisions:
            raise ValueError('URDF requires one rooted tree and collision primitives')
        self.root_link = next(iter(roots))
        for link in self.links:
            visited = set()
            while link in self.by_child:
                if link in visited:
                    raise ValueError('URDF joint cycle')
                visited.add(link)
                link = self.by_child[link].parent
            if link != self.root_link:
                raise ValueError('disconnected URDF tree')

    def project(self, root_world_pose=None, joint_positions=None, link_world_poses=None, cylinder_sides=48):
        positions, poses = dict(joint_positions or {}), dict(link_world_poses or {})
        if set(poses)-self.links:
            raise ValueError('unknown link world pose: ' + repr(sorted(set(poses)-self.links)))
        if any(not isinstance(p, Pose3) for p in poses.values()):
            raise ValueError('link poses must be Pose3')
        supplied_links = set(poses)
        if root_world_pose is not None:
            if not isinstance(root_world_pose, Pose3):
                raise ValueError('root pose must be Pose3')
            if self.root_link not in poses:
                poses[self.root_link] = root_world_pose

        def joint_value(name, visiting=()):
            if name in visiting or name not in self.joints:
                raise ValueError('invalid mimic joint dependency')
            if name in positions:
                return _finite([positions[name]], 1)[0]
            mimic = self.joints[name].mimic
            if mimic is not None:
                return mimic[1]*joint_value(mimic[0], visiting+(name,))+mimic[2]
            raise ValueError('missing actual joint position: '+name)

        def link_pose(link):
            if link in poses:
                return poses[link]
            if link not in self.by_child:
                raise ValueError('missing world pose for root link '+link)
            joint = self.by_child[link]
            motion = Pose3()
            if joint.kind in {'revolute', 'continuous'}:
                angle = joint_value(joint.name)
                motion = Pose3(xyzw=tuple(x*math.sin(angle/2) for x in joint.axis)+(math.cos(angle/2),))
            elif joint.kind == 'prismatic':
                distance = joint_value(joint.name)
                motion = Pose3(tuple(x*distance for x in joint.axis))
            poses[link] = link_pose(joint.parent).compose(joint.origin).compose(motion)
            return poses[link]

        result = []
        for c in self.collisions:
            pose = link_pose(c.link).compose(c.origin)
            source = 'actual_link_world_pose' if c.link in supplied_links else 'root_pose_and_actual_joint_state_fk'
            if c.kind == 'box':
                projection = project_box(c.name, c.dimensions, pose, c.link, source)
            else:
                projection = project_cylinder(c.name, c.dimensions[0], c.dimensions[1], pose,
                                              cylinder_sides, c.link, source)
            result.append(projection)
        return tuple(result)
