"""M08 additive batch-distance backend; frozen v1 tracking/schema are unchanged.

All support circles and latest baseline/tentative points remain in the minimum.
The common 1.35 m/s time slope factors out exactly. Batched AABB lower bounds
only select exact convex-footprint point distances; they never replace the
footprint. A declared 1e-10 m numerical subtraction is conservative relative to
v1's GEOS expression over the tested, bounded coordinate domain. Not a formal
floating-point proof or physical/continuous safety qualification.
"""
import math
import numpy as np
from shapely.geometry import Polygon
import sensor_tracks_v1 as _v1
SCHEMA = _v1.SCHEMA

BATCH_SCHEMA = 'astra_sensor_tracks_batch_distance_v2'
NUMERICAL_SUBTRACTION_M = 1e-10
MAXIMUM_QUERY_COUNT = 4096
QUERY_TILE = 32


def _point_polygon_distances(polygons, points):
    """One point per corresponding convex polygon; both [N,V,2]/[N,2]."""
    if len(points) == 0:
        return np.empty(0, dtype=float)
    edge = np.roll(polygons, -1, axis=1)-polygons
    rel = points[:, None, :]-polygons
    denominator = np.sum(edge*edge, axis=2)
    fraction = np.clip(np.sum(rel*edge, axis=2)/denominator, 0., 1.)
    nearest = rel-fraction[:, :, None]*edge
    distance = np.sqrt(np.min(np.sum(nearest*nearest, axis=2), axis=1))
    cross = edge[:, :, 0]*rel[:, :, 1]-edge[:, :, 1]*rel[:, :, 0]
    inside = np.all(cross >= 0., axis=1) | np.all(cross <= 0., axis=1)
    distance[inside] = 0.
    return distance


class ConvexFootprint:
    """The existing declared full-body hull, copied without dilation/change.

    Use RouteAwareRuntime.footprint.vertices. Convexity is a required geometric
    property, not an approximation of an arbitrary concave footprint. Holes,
    concavity, repeated edges and nonfinite/degenerate inputs are rejected.
    """
    def __init__(self, vertices):
        vertices = np.asarray(vertices, dtype=float)
        if vertices.ndim != 2 or vertices.shape[1] != 2 or not 3 <= len(vertices) <= 128 or not np.all(np.isfinite(vertices)):
            raise ValueError('3..128 finite full-body convex hull vertices required')
        if np.array_equal(vertices[0], vertices[-1]):
            vertices = vertices[:-1]
        edges = np.roll(vertices, -1, axis=0)-vertices
        if len(vertices) < 3 or np.any(np.linalg.norm(edges, axis=1) < 1e-9):
            raise ValueError('nondegenerate footprint edges required')
        polygon = Polygon(vertices)
        if not polygon.is_valid or polygon.area <= 1e-12 or not polygon.equals(polygon.convex_hull):
            raise ValueError('actual valid convex footprint required; no silent convexification')
        self.vertices = vertices.copy(); self.vertices.flags.writeable = False
        self.radius_m = float(np.linalg.norm(vertices, axis=1).max())


class BatchHardOccupancy:
    """Query-only adapter for a frozen v1 Snapshot. No track or policy mutation.

    Call once per fresh observation/decision snapshot. Input poses are base_link
    map poses, not rear axle poses. Prediction offsets are from now_ns, exactly
    as in v1. The caller retains v1 source health/deadline checks, the original
    external 0.33 m margin, static-map geometry and interval motion budgets.
    """
    def __init__(self, snapshot, now_ns):
        _require_available(snapshot, now_ns)
        if snapshot.schema != SCHEMA or not 0 <= now_ns-snapshot.source_ns <= snapshot.config.maximum_source_age_ns:
            raise ValueError('fresh frozen-v1 snapshot required')
        self.snapshot = snapshot; self.now_ns = now_ns; self.c = snapshot.config
        if self.c.dynamic_speed_bound_mps != 1.35:
            raise ValueError('same v1 common speed expansion required')
        centers = []; radii = []
        speed = self.c.dynamic_speed_bound_mps
        for track in snapshot.tracks:
            centers.append(track.center)
            radii.append(track.radius_m+speed*(now_ns-track.anchor_ns)/1e9)
        cell = self.c.sparse_cell_m
        for key, stamp in snapshot.sparse_supports:
            centers.append(((key[0]+.5)*cell, (key[1]+.5)*cell))
            radii.append(cell/math.sqrt(2)+speed*(now_ns-stamp)/1e9)
        age_radius = speed*(now_ns-snapshot.source_ns)/1e9
        # Keep every baseline/tentative return. No dominance assumption, spatial
        # downsampling, history expiry, CV shift or identity-based deletion.
        centers.extend(snapshot.baseline_points)
        radii.extend([age_radius]*len(snapshot.baseline_points))
        centers.extend(snapshot.tentative_map_points)
        radii.extend([age_radius+self.c.position_error_m]*len(snapshot.tentative_map_points))
        self.centers = np.asarray(centers, dtype=float).reshape(-1, 2)
        self.radii = np.asarray(radii, dtype=float)
        if not np.all(np.isfinite(self.centers)) or not np.all(np.isfinite(self.radii)) or np.any(self.radii < 0.):
            raise ValueError('finite causal support geometry required')
        self.centers.flags.writeable = False; self.radii.flags.writeable = False
        self.support_count = len(centers)
        self.identity = dict(schema=BATCH_SCHEMA,tracking_schema=SCHEMA,
                             predictor_request_schema_unchanged=True, source_ns=snapshot.source_ns,
                             numerical_subtraction_m=NUMERICAL_SUBTRACTION_M,
                             support_count=self.support_count, occupancy_removed=False,
                             learned_interaction=False, calibration_qualified=False,
                             physical_safety_certified=False, live_deadline_qualified=False)

    def distance_lower_bounds(self, footprint, base_poses, offsets_ns):
        """Return Q distances before caller's unchanged .33m margin.

        Uses fixed full-body hull and Q rigid transforms, without constructing
        Q Shapely geometries. All shapes are the exact same vertices/poses that
        the old FullFootprintEnvelope.at(base_pose) would construct.
        """
        if not isinstance(footprint, ConvexFootprint):
            raise ValueError('explicit validated shared ConvexFootprint required')
        poses = np.asarray(base_poses, dtype=float)
        offsets = np.asarray(offsets_ns)
        if poses.ndim != 2 or poses.shape[1] != 3 or not 1 <= len(poses) <= MAXIMUM_QUERY_COUNT or not np.all(np.isfinite(poses)):
            raise ValueError('bounded nonempty finite Q-by-3 base poses required')
        if offsets.shape != (len(poses),) or offsets.dtype.kind not in ('i', 'u') or np.any(offsets < 0) or np.any(offsets > self.c.maximum_prediction_ns):
            raise ValueError('same-count bounded integer nanosecond offsets required')
        if np.any(np.abs(poses[:, :2])+footprint.radius_m > self.c.maximum_abs_map_coordinate_m):
            raise ValueError('transformed hull outside declared numerical coordinate support')
        offsets = offsets.astype(np.int64, copy=False)
        if not self.support_count:
            return np.full(len(poses), math.inf)
        # Same shape at different horizons has an identical spatial minimum;
        # all v1 terms share exactly the same -1.35 * offset time contribution.
        unique, inverse = np.unique(poses, axis=0, return_inverse=True)
        cosine = np.cos(unique[:, 2]); sine = np.sin(unique[:, 2])
        vertices = footprint.vertices
        polygons = np.empty((len(unique), len(vertices), 2), dtype=float)
        polygons[:, :, 0] = unique[:, 0, None]+cosine[:, None]*vertices[:, 0]-sine[:, None]*vertices[:, 1]
        polygons[:, :, 1] = unique[:, 1, None]+sine[:, None]*vertices[:, 0]+cosine[:, None]*vertices[:, 1]
        spatial = self._spatial_minimum(polygons)
        return spatial[inverse]-self.c.dynamic_speed_bound_mps*(offsets/1e9)-NUMERICAL_SUBTRACTION_M

    def _spatial_minimum(self, polygons):
        best = np.empty(len(polygons), dtype=float)
        for start in range(0, len(polygons), QUERY_TILE):
            stop = min(start+QUERY_TILE, len(polygons)); shapes = polygons[start:stop]
            low = np.min(shapes, axis=1); high = np.max(shapes, axis=1)
            # Vectorized lower bound over every support; all are retained.
            dx = np.maximum(np.maximum(low[:, 0, None]-self.centers[None, :, 0],
                                       self.centers[None, :, 0]-high[:, 0, None]), 0.)
            dy = np.maximum(np.maximum(low[:, 1, None]-self.centers[None, :, 1],
                                       self.centers[None, :, 1]-high[:, 1, None]), 0.)
            lower = np.sqrt(dx*dx+dy*dy)-self.radii[None, :]-NUMERICAL_SUBTRACTION_M
            first = np.argmin(lower, axis=1); rows = np.arange(len(shapes))
            values = _point_polygon_distances(shapes, self.centers[first])-self.radii[first]
            possible_rows, possible_columns = np.nonzero(lower < values[:, None])
            # This may contain the first support again; duplicate work changes
            # no geometry. No lower-bound value itself becomes the answer.
            distances = _point_polygon_distances(shapes[possible_rows], self.centers[possible_columns])-self.radii[possible_columns]
            np.minimum.at(values, possible_rows, distances)
            best[start:stop] = values
        return best


def _require_available(snapshot, now_ns):
    _v1.ns(now_ns)
    if now_ns < snapshot.accepted_now_ns:
        raise ValueError('decision/query cannot predate actual snapshot acceptance')


class HardOccupancy(_v1.HardOccupancy):
    """Compatibility scalar query with v2 availability guard; v1 is unchanged."""
    def __init__(self, snapshot, now_ns):
        _require_available(snapshot, now_ns)
        super().__init__(snapshot, now_ns)


class SensorTracks(_v1.SensorTracks):
    """Same frozen v1 tracking; all new public occupancy paths are guarded."""
    def occupancy(self, now_ns):
        if self.health(now_ns):
            raise _v1.TrackerFault('fresh_accepted_scan_required_for_occupancy')
        return HardOccupancy(self.snapshot, now_ns)

    def batch_occupancy(self, now_ns):
        if self.health(now_ns):
            raise _v1.TrackerFault('fresh_accepted_scan_required_for_occupancy')
        return BatchHardOccupancy(self.snapshot, now_ns)

    def identity(self):
        value = super().identity()
        value.update(distance_backend=BATCH_SCHEMA, availability_guard='decision/query >= accepted_now_ns')
        return value


def prediction_request(snapshot, real_bucket_history, candidates, *, decision_ns):
    """Same valid request bytes/schema, with actual-availability guard added."""
    _require_available(snapshot, decision_ns)
    return _v1.prediction_request(snapshot, real_bucket_history, candidates, decision_ns=decision_ns)
