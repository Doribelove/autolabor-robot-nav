"""Isolated M12 candidate: lazy 32-row exact KD-neighborhood retrieval.

Only neighborhood retrieval differs from frozen sensor_tracks_ingestion_m08.
The accept body below is preserved byte-for-byte. No M08 globals are replaced.
The SciPy 1.10.1 compatible offline environment is the tested backend: one
worker, eps=0, sorted results, original radius and original scalar predicate.
Cached lists are broad-phase candidates, never an occupancy approximation.
No ROS integration, runtime qualification, training or physical-safety claim.
"""
from collections import deque
from dataclasses import asdict
import math
import time
import numpy as np
from scipy.spatial import cKDTree
from sensor_tracks_v1 import SCHEMA,Track,Snapshot,ns,canonical_sha,_finite_vector,_circle
from sensor_tracks_v2 import SensorTracks as BatchSensorTracks

TILE_ROWS = 32
IMPLEMENTATION_SCHEMA = 'astra_sensor_tracks_exact_kd_tiles_m12_v1'


class _LazyNeighbors32:
    """One tile only; traversal order may evict/refetch earlier tiles.

    The old tile is dropped BEFORE a new query, so retained neighbor lists do
    not accumulate. At the frozen 1440-point capacity, at most 46080 neighbor
    indices are cached. This is an element-count bound, not a Python-byte or
    native scratch-memory bound. Dense/interleaved traversal can be slower.
    """
    __slots__ = ('tree', 'points', 'radius', 'start', 'rows')

    def __init__(self, tree, points, radius):
        self.tree = tree
        self.points = points
        self.radius = radius
        self.start = -1
        self.rows = None

    def __call__(self, index):
        start = (index // TILE_ROWS) * TILE_ROWS
        if start != self.start:
            self.rows = None
            self.start = -1
            rows = self.tree.query_ball_point(
                self.points[start:start+TILE_ROWS], self.radius,
                eps=0., workers=1, return_sorted=True)
            self.rows = rows
            self.start = start
        return self.rows[index-start]


def _clusters(points,gap):
    if not len(points):return []
    tree=cKDTree(points);xy=points.tolist();visited=np.zeros(len(points),dtype=bool);visited_count=0;result=[]
    neighbors=_LazyNeighbors32(tree,points,gap+1e-9)
    for root in range(len(points)):
        if visited[root]:continue
        visited[root]=True;visited_count+=1;queue=[root];component=[]
        while queue:
            if visited_count==len(points):component.extend(queue);break
            i=queue.pop();component.append(i)
            for j in neighbors(i):
                if visited[j]:continue
                distance=math.hypot(xy[j][0]-xy[i][0],xy[j][1]-xy[i][1])
                connected=(np.linalg.norm(points[j]-points[i])<=gap) if abs(distance-gap)<=1e-12 else distance<=gap
                if connected:
                    visited[j]=True;visited_count+=1;queue.append(j)
        result.append(points[sorted(component)])
    return result


class IngestionSensorTracksM12(BatchSensorTracks):
    def accept(self, frame, *, tf_stamp_ns, odom_state, now_ns, frame_id='map'):
        started = time.perf_counter_ns(); self._clock(now_ns); c = self.config
        stamp = ns(frame.stamp_ns)
        if frame_id != 'map' or type(tf_stamp_ns) is not int or tf_stamp_ns != stamp:
            raise ValueError('exact scan source TF in map required; do not consume pending input')
        if not 0 <= now_ns-stamp <= c.maximum_source_age_ns:
            raise ValueError('future/stale scan cannot refresh tracking')
        if self.source_ns is not None and stamp <= self.source_ns:
            raise ValueError('replayed/out-of-order scan')
        odom_stamp = ns(odom_state['stamp_ns'])
        if not 0 <= stamp-odom_stamp <= c.maximum_source_age_ns:
            raise ValueError('causal at-or-before scan odometry required')
        pose = _finite_vector(odom_state['pose_map'], 3)
        twist = _finite_vector(odom_state['body_twist'], 3)
        laser_pose = _finite_vector(frame.laser_pose_map, 3)
        ranges = np.asarray(frame.model_ranges, dtype=float)
        if ranges.shape != (72,) or not np.all(np.isfinite(ranges)) or np.any(ranges < 0):
            raise ValueError('finite 72-beam real model scan required')
        points = np.asarray(frame.points_map, dtype=float)
        if points.shape == (0,):
            points = points.reshape(0, 2)
        if points.ndim != 2 or points.shape[1] != 2 or not np.all(np.isfinite(points)):
            raise ValueError('finite Nx2 exact-TF return cloud required')
        if np.any(np.abs(points) > c.maximum_abs_map_coordinate_m):
            raise ValueError('map coordinates outside declared numerical support')
        if len(points) > c.maximum_points:
            self._fail('point_capacity_exceeded_no_subsampling')
        if self.start_ns is not None and stamp-self.start_ns > c.maximum_episode_span_ns:
            self._fail('episode_span_exceeded_no_eviction')
        # This is the same clearance predicate used by RouteAwareRuntime.predict.
        distances = np.asarray([self.grid.clearance(p) for p in points], dtype=float)
        if np.any(np.isnan(distances)):
            raise ValueError('invalid static map clearance')
        map_known = distances <= c.static_tolerance_m
        dynamic = points[~map_known]
        keys = [tuple(np.floor(p/c.map_consistency_cell_m).astype(int)) for p in points[map_known]]
        streak = {key: self.map_streak.get(key, 0)+1 for key in set(keys)}
        confirmed = np.asarray([streak[key] >= c.static_confirmation_scans for key in keys], dtype=bool)
        tentative = points[map_known][~confirmed]
        # Map consistency is evidence of a static prior match, not actor truth.
        # Outside-map stationary returns are NEVER learned away as static.
        groups = _clusters(dynamic, c.cluster_gap_m); detections = []; sparse = dict(self.sparse)
        for group in groups:
            if len(group) >= c.minimum_cluster_points:
                center, radius = _circle(group, c.position_error_m)
                detections.append((center, radius, len(group)))
            else:
                for point in group:
                    key = tuple(np.floor(point/c.sparse_cell_m).astype(int))
                    # A whole cell at its oldest observation time encloses every
                    # later return in that cell. No lost singleton is discarded.
                    if key not in sparse:
                        sparse[key] = stamp
        if len(sparse) > c.maximum_sparse_cells:
            self._fail('sparse_capacity_exceeded_no_eviction')
        tracks = list(self.tracks); edges = []
        for i, old in enumerate(tracks):
            age = (stamp-old.anchor_ns)/1e9
            reach = old.radius_m+c.dynamic_speed_bound_mps*age+c.position_error_m
            for j, (center, radius, _) in enumerate(detections):
                if math.dist(center, old.center) <= reach+radius:
                    edges.append((i, j))
        # Bipartite components preserve alternatives. Ambiguous groups merge
        # into a labelled *union hypothesis*; no nearest-neighbour winner is
        # silently chosen. Once merged, identity remains unresolved this episode.
        adjacency = {}
        for i, j in edges:
            adjacency.setdefault(('t', i), set()).add(('d', j))
            adjacency.setdefault(('d', j), set()).add(('t', i))
        components = []; seen = set()
        for vertex in sorted(adjacency):
            if vertex in seen:
                continue
            pending = [vertex]; seen.add(vertex); component = set()
            while pending:
                v = pending.pop(); component.add(v)
                for other in adjacency[v]:
                    if other not in seen:
                        seen.add(other); pending.append(other)
            components.append(component)
        result = []; used_t = set(); used_d = set(); next_id = self.next_id; ambiguous_components = 0
        for component in components:
            ti = sorted(i for kind, i in component if kind == 't'); di = sorted(i for kind, i in component if kind == 'd')
            used_t.update(ti); used_d.update(di)
            old = [tracks[i] for i in ti]; new = [detections[i] for i in di]
            ambiguous = len(ti) != 1 or len(di) != 1 or old[0].ambiguous
            if not ambiguous:
                previous = old[0]; center, radius, _ = new[0]
                dt = (stamp-previous.last_detection_ns)/1e9
                velocity = np.subtract(center, previous.center)/dt
                magnitude = float(np.linalg.norm(velocity))
                # Uncalibrated centroid CV is only a bounded nominal diagnostic;
                # the hard tube retains the full speed disk, not this velocity.
                if magnitude > c.dynamic_speed_bound_mps:
                    velocity *= c.dynamic_speed_bound_mps/magnitude
                result.append(Track(previous.track_id, previous.member_ids, center, radius, stamp, stamp,
                                    tuple(float(v) for v in velocity), previous.observation_count+1, False))
            else:
                ambiguous_components += 1
                center = tuple(float(v) for v in np.mean([x[0] for x in new], axis=0))
                radius = max([math.dist(center, p.center)+p.radius_m+c.dynamic_speed_bound_mps*(stamp-p.anchor_ns)/1e9 for p in old]
                             +[math.dist(center, p[0])+p[1] for p in new])
                members = tuple(sorted({m for p in old for m in p.member_ids}))
                result.append(Track(min(members), members, center, radius, stamp, stamp, None,
                                    max(p.observation_count for p in old)+1, True))
        result.extend(p for i, p in enumerate(tracks) if i not in used_t)
        for j, (center, radius, _) in enumerate(detections):
            if j not in used_d:
                result.append(Track(next_id, (next_id,), center, radius, stamp, stamp, None, 1, False)); next_id += 1
        result.sort(key=lambda p: p.track_id)
        if len(result) > c.maximum_tracks:
            self._fail('track_capacity_exceeded_no_eviction')
        recent = tuple(self.recent)+(stamp,); recent = recent[-64:]
        frame_hash = canonical_sha(dict(source_ns=stamp, model_ranges=ranges.tolist(), points_map=points.tolist(), laser_pose_map=laser_pose))
        hashes = (tuple(self.recent_hashes)+((stamp, frame_hash),))[-64:]
        receipt = dict(schema=SCHEMA, source_ns=stamp, tf_source_ns=tf_stamp_ns, odom_source_ns=odom_stamp,
                       body_pose_map=pose, body_twist=twist, laser_pose_map=laser_pose, frame_id='map',
                       map_identity=self.map_identity, points_map=points.tolist(), model_ranges=ranges.tolist(), config=asdict(c))
        stats = dict(return_count=len(points), baseline_unknown_count=len(dynamic), map_match_count=int(map_known.sum()),
                     map_confirmed_count=int(confirmed.sum()), map_tentative_count=len(tentative), cluster_count=len(detections),
                     sparse_cells=len(sparse), track_count=len(result), ambiguous_components=ambiguous_components,
                     processing_ns=time.perf_counter_ns()-started)
        snapshot = Snapshot(SCHEMA, stamp, now_ns, 'map', self.map_identity, c, tuple(result),
                            tuple((key, value) for key, value in sorted(sparse.items())),
                            tuple(tuple(float(v) for v in p) for p in dynamic),
                            tuple(tuple(float(v) for v in p) for p in tentative), recent, hashes, odom_stamp, pose, twist,
                            tuple(sorted(stats.items())), canonical_sha(receipt))
        # Single commit after validation/capacity checks; invalid frames do not
        # advance freshness, associations, static confirmation or source history.
        self.tracks = snapshot.tracks; self.sparse = sparse; self.map_streak = streak
        self.source_ns = stamp; self.start_ns = stamp if self.start_ns is None else self.start_ns
        self.next_id = next_id; self.recent = deque(recent, maxlen=64); self.recent_hashes = deque(hashes, maxlen=64); self.snapshot = snapshot
        return snapshot
