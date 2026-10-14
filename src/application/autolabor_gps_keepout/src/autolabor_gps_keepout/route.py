"""Rolling GPS targets selected from a route around exclusion polygons."""
import math
from gps_module.long_range import RollingGoalRoute, RouteSegment


class KeepoutGoalRoute(RollingGoalRoute):
    def __init__(self, *args, geometry, **kwargs):
        super().__init__(*args, **kwargs)
        self.geometry = geometry

    def _candidate(self, x, y):
        start = (x, y)
        path = self.geometry.path(start, (self.final_x, self.final_y))
        remaining = self.lookahead_distance
        chosen = path[0]
        for a, b in zip(path, path[1:]):
            distance = math.hypot(b[0]-a[0], b[1]-a[1])
            fraction = min(1., remaining/max(distance, 1e-9))
            candidate = (a[0]+(b[0]-a[0])*fraction, a[1]+(b[1]-a[1])*fraction)
            if not self.geometry.clear_line(start, candidate):
                break
            chosen = candidate
            remaining -= distance*fraction
            if remaining <= 1e-6: break
        final = math.hypot(chosen[0]-self.final_x, chosen[1]-self.final_y) < 1e-6
        return chosen[0], chosen[1], final

    def final_is_within_horizon(self, x, y):
        return (super().final_is_within_horizon(x, y)
                and self.geometry.clear_line((x, y), (self.final_x, self.final_y)))

    def should_advance(self, x, y):
        segment = self.current_segment
        if segment is None or segment.is_final: return False
        if self.distance_to_segment(x, y) > self.advance_distance: return False
        nx, ny, final = self._candidate(x, y)
        # Avoid repeatedly resending the same corner target at 10 Hz.
        return final or math.hypot(nx-segment.x, ny-segment.y) >= 1.0

    def next_segment(self, x, y):
        if self.current_segment is not None and self.current_segment.is_final:
            return self.current_segment
        nx, ny, final = self._candidate(x, y)
        self.segment_index += 1
        self.current_segment = RouteSegment(self.segment_index,nx,ny,final,self.distance_to_final(x,y))
        return self.current_segment
