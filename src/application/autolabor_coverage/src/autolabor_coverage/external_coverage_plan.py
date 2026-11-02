"""Load an explicitly selected ordered coverage plan for one immutable map."""
import copy
import hashlib
import json
import math

from .coverage_geometry import CoveragePlan, Point, Swath, estimate_route_time, polygon_area, sample_path

_cache = {}


def load_plan(filename, planner, polygon, width, overlap, origin, yaw, parameters):
    with open(filename, 'rb') as stream:
        raw = stream.read()
    data = json.loads(raw)
    digest = planner.grid.digest()
    if data['grid_digest'] != digest:
        raise ValueError('ordered coverage plan belongs to a different occupancy map')
    if abs(data['operation_width_m']-width) > 1e-6 or abs(data['overlap_ratio']-overlap) > 1e-6:
        raise ValueError('ordered coverage plan width/overlap does not match the request')
    expected = data['region']
    if len(polygon) != len(expected) or any(math.hypot(p.x-q[0], p.y-q[1]) > 1e-4
                                          for p,q in zip(polygon,expected)):
        raise ValueError('ordered coverage plan region does not match the requested region')
    key = (digest, hashlib.sha256(raw).hexdigest(), planner.front, planner.rear,
           planner.half_width, planner.minimum_turning_radius)
    cached = _cache.get(key)
    if cached is None:
        route = []
        collision_cache = {}
        for line in data['swaths']:
            values = [float(value) for point in line for value in point]
            if len(values) != 4 or not all(math.isfinite(v) for v in values):
                raise ValueError('invalid ordered coverage coordinates')
            a,b = Point(*values[:2]),Point(*values[2:])
            length = math.hypot(b.x-a.x,b.y-a.y)
            if length < planner.minimum_swath_length-1e-6:
                raise ValueError('ordered sweep is shorter than the configured minimum')
            heading = math.atan2(b.y-a.y,b.x-a.x)
            for point in sample_path(a,b,planner.grid.resolution):
                if not planner._pose_is_free(point,heading,collision_cache):
                    raise ValueError('ordered coverage sweep conflicts with the map footprint')
            route.append(Swath(a,b,0.,length))
        if not route:
            raise ValueError('ordered coverage plan has no sweep lines')
        transitions = None
        if data.get('format_version') == 2:
            from .validated_path import check_path, same_pose
            transitions = data['transitions']
            if len(transitions) != len(route):
                raise ValueError('one verified connector is required for each sweep')
            for i, (swath, poses) in enumerate(zip(route, transitions)):
                check_path(poses, planner.grid, planner.front, planner.rear,
                           planner.half_width, planner.minimum_turning_radius)
                expected_end = [swath.start.x, swath.start.y,
                                math.atan2(swath.end.y-swath.start.y, swath.end.x-swath.start.x)]
                if not same_pose(poses[-1], expected_end):
                    raise ValueError('connector does not end at the sweep entry pose')
                if i:
                    previous = route[i-1]
                    expected_start = [previous.end.x, previous.end.y,
                                      math.atan2(previous.end.y-previous.start.y, previous.end.x-previous.start.x)]
                    if not same_pose(poses[0], expected_start):
                        raise ValueError('connector does not start at the previous sweep exit pose')
            # Validate straight sweeps at the same resolution as connectors.
            import numpy as np
            for swath in route:
                heading = math.atan2(swath.end.y-swath.start.y, swath.end.x-swath.start.x)
                count = int(math.ceil(swath.length / min(.05, planner.grid.resolution/2)))+1
                poses = np.column_stack([np.linspace(swath.start.x, swath.end.x, count),
                                         np.linspace(swath.start.y, swath.end.y, count),
                                         np.full(count, heading)])
                check_path(poses, planner.grid, planner.front, planner.rear,
                           planner.half_width, planner.minimum_turning_radius)
        _cache.clear();_cache[key] = (route, transitions)
        cached = (route, transitions)
    route, transitions = copy.deepcopy(cached)
    estimate = estimate_route_time(route, origin or route[0].start, yaw,
                                   planner.minimum_turning_radius, parameters)
    area = polygon_area(polygon)
    covered = min(area,float(data['preview_swept_known_area_m2']))
    result = CoveragePlan(angle=0., spacing=width*(1-overlap), swaths=route,
        requested_area=area, reachable_area=covered, unreachable_area=max(0.,area-covered),
        score=estimate.total_time_sec,estimated_total_time_sec=estimate.total_time_sec,
        estimated_sweep_time_sec=estimate.sweep_time_sec,
        estimated_transit_time_sec=estimate.transit_time_sec,
        estimated_reverse_transitions=estimate.reverse_transitions)
    if transitions is not None:
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path
        result.transition_paths = []
        for poses in transitions:
            path = Path(); path.header.frame_id = 'map'
            for x, y, heading in poses:
                pose = PoseStamped(); pose.header.frame_id = 'map'
                pose.pose.position.x = x; pose.pose.position.y = y
                pose.pose.orientation.z = math.sin(heading/2)
                pose.pose.orientation.w = math.cos(heading/2)
                path.poses.append(pose)
            result.transition_paths.append(path)
        # A recorded entry is displayable only at its recorded origin. The
        # start service always computes a new entry from the current pose.
        from .validated_path import same_pose
        if origin is None or not same_pose([origin.x, origin.y, yaw], transitions[0][0], .05):
            result.transition_paths[0] = None
        result.verified_external_transitions = True
    return result
