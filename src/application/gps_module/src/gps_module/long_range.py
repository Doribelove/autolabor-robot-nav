"""Pure geometry and GoalID helpers for rolling long-range GPS goals."""

from dataclasses import dataclass
import math
import re


EARTH_RADIUS_M = 6378137.0
ROUTE_GOAL_ID_PREFIX = "gps_long_range"
INTERMEDIATE_GOAL_KIND = "intermediate"
FINAL_GOAL_KIND = "final"
DISTANCE_EPSILON_M = 1e-6

_ROUTE_TOKEN_PATTERN = re.compile(r"^[0-9]+-[0-9]+$")


def _finite_number(name, value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("{} must be finite".format(name))
    return value


def validate_latitude_longitude(latitude, longitude):
    """Return validated WGS84 latitude/longitude as finite floats."""
    latitude = _finite_number("latitude", latitude)
    longitude = _finite_number("longitude", longitude)
    if not -90.0 <= latitude <= 90.0:
        raise ValueError("latitude must be in [-90, 90]")
    if not -180.0 <= longitude <= 180.0:
        raise ValueError("longitude must be in [-180, 180]")
    return latitude, longitude


def gps_to_xy(latitude, longitude, origin_latitude, origin_longitude):
    """Convert a nearby WGS84 point to the workspace's local ENU metres."""
    latitude, longitude = validate_latitude_longitude(latitude, longitude)
    origin_latitude, origin_longitude = validate_latitude_longitude(
        origin_latitude, origin_longitude
    )
    d_lat = math.radians(latitude - origin_latitude)
    d_lon = math.radians(longitude - origin_longitude)
    reference_latitude = math.radians(origin_latitude)
    x = EARTH_RADIUS_M * d_lon * math.cos(reference_latitude)
    y = EARTH_RADIUS_M * d_lat
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError("converted GPS goal must be finite")
    return x, y


def rotate_xy(x, y, yaw):
    """Rotate a local point by ``yaw`` radians."""
    x = _finite_number("x", x)
    y = _finite_number("y", y)
    yaw = _finite_number("yaw", yaw)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        cos_yaw * x - sin_yaw * y,
        sin_yaw * x + cos_yaw * y,
    )


def validate_route_distances(lookahead_distance, advance_distance):
    """Validate the rolling subgoal horizon and replacement threshold."""
    lookahead_distance = _finite_number(
        "lookahead_distance", lookahead_distance
    )
    advance_distance = _finite_number("advance_distance", advance_distance)
    if lookahead_distance <= 0.0:
        raise ValueError("lookahead_distance must be positive")
    if advance_distance <= 0.0:
        raise ValueError("advance_distance must be positive")
    if advance_distance >= lookahead_distance:
        raise ValueError(
            "advance_distance must be smaller than lookahead_distance"
        )
    return lookahead_distance, advance_distance


@dataclass(frozen=True)
class RouteSegment:
    """One local move_base target selected from a longer final route."""

    index: int
    x: float
    y: float
    is_final: bool
    final_distance: float


class RollingGoalRoute:
    """Select a bounded subgoal on the current-position-to-final line."""

    def __init__(
        self,
        final_x,
        final_y,
        lookahead_distance=15.0,
        advance_distance=5.0,
    ):
        self.final_x = _finite_number("final_x", final_x)
        self.final_y = _finite_number("final_y", final_y)
        (
            self.lookahead_distance,
            self.advance_distance,
        ) = validate_route_distances(lookahead_distance, advance_distance)
        self.segment_index = 0
        self.current_segment = None

    def distance_to_final(self, current_x, current_y):
        current_x = _finite_number("current_x", current_x)
        current_y = _finite_number("current_y", current_y)
        return math.hypot(
            self.final_x - current_x,
            self.final_y - current_y,
        )

    def distance_to_segment(self, current_x, current_y):
        if self.current_segment is None:
            return math.inf
        current_x = _finite_number("current_x", current_x)
        current_y = _finite_number("current_y", current_y)
        return math.hypot(
            self.current_segment.x - current_x,
            self.current_segment.y - current_y,
        )

    def should_advance(self, current_x, current_y):
        """Return true when an intermediate target should be replaced."""
        if self.current_segment is None or self.current_segment.is_final:
            return False
        current_x = _finite_number("current_x", current_x)
        current_y = _finite_number("current_y", current_y)
        if (
            self.distance_to_segment(current_x, current_y)
            <= self.advance_distance + DISTANCE_EPSILON_M
        ):
            return True

        # A detour can pass the subgoal outside its 5m radius. Advance once the
        # vehicle has made the same radial progress expected on the straight
        # route, or has crossed the plane through the subgoal perpendicular to
        # the remaining route, so move_base is never asked to turn back.
        expected_progress = self.lookahead_distance - self.advance_distance
        if (
            self.distance_to_final(current_x, current_y)
            <= self.current_segment.final_distance
            - expected_progress
            + DISTANCE_EPSILON_M
        ):
            return True
        remaining_x = self.final_x - self.current_segment.x
        remaining_y = self.final_y - self.current_segment.y
        passed_x = current_x - self.current_segment.x
        passed_y = current_y - self.current_segment.y
        return passed_x * remaining_x + passed_y * remaining_y >= 0.0

    def final_is_within_horizon(self, current_x, current_y):
        """Return whether the exact final point can be sent safely now."""
        return (
            self.distance_to_final(current_x, current_y)
            <= self.lookahead_distance + DISTANCE_EPSILON_M
        )

    def next_segment(self, current_x, current_y):
        """Create the next bounded target, or retain an already-final target."""
        current_x = _finite_number("current_x", current_x)
        current_y = _finite_number("current_y", current_y)
        if self.current_segment is not None and self.current_segment.is_final:
            return self.current_segment

        dx = self.final_x - current_x
        dy = self.final_y - current_y
        final_distance = math.hypot(dx, dy)
        is_final = (
            final_distance
            <= self.lookahead_distance + DISTANCE_EPSILON_M
        )
        if final_distance <= 1e-9:
            segment_x = self.final_x
            segment_y = self.final_y
            is_final = True
        elif is_final:
            segment_x = self.final_x
            segment_y = self.final_y
        else:
            scale = self.lookahead_distance / final_distance
            segment_x = current_x + dx * scale
            segment_y = current_y + dy * scale

        self.segment_index += 1
        self.current_segment = RouteSegment(
            index=self.segment_index,
            x=segment_x,
            y=segment_y,
            is_final=is_final,
            final_distance=final_distance,
        )
        return self.current_segment


def make_route_goal_id(route_token, segment_index, is_final):
    """Build the strict action GoalID understood by the goal-speed limiter."""
    route_token = str(route_token)
    if not _ROUTE_TOKEN_PATTERN.fullmatch(route_token):
        raise ValueError("route_token must contain '<timestamp>-<counter>'")
    segment_index = int(segment_index)
    if segment_index <= 0:
        raise ValueError("segment_index must be positive")
    kind = FINAL_GOAL_KIND if is_final else INTERMEDIATE_GOAL_KIND
    return "{}/{}/{}/{}".format(
        ROUTE_GOAL_ID_PREFIX,
        route_token,
        segment_index,
        kind,
    )


def parse_route_goal_id(identifier):
    """Return ``(route_token, segment_index, kind)`` for a managed GoalID."""
    parts = str(identifier).split("/")
    if len(parts) != 4 or parts[0] != ROUTE_GOAL_ID_PREFIX:
        return None
    if not _ROUTE_TOKEN_PATTERN.fullmatch(parts[1]):
        return None
    try:
        segment_index = int(parts[2])
    except ValueError:
        return None
    if segment_index <= 0:
        return None
    if parts[3] not in (INTERMEDIATE_GOAL_KIND, FINAL_GOAL_KIND):
        return None
    return parts[1], segment_index, parts[3]


def route_goal_kind(identifier):
    """Return ``intermediate``/``final`` for a strict managed GoalID."""
    identity = parse_route_goal_id(identifier)
    return None if identity is None else identity[2]


def is_contiguous_route_goal_transition(previous_identifier, next_identifier):
    """Return whether two IDs are consecutive segments of one active route."""
    previous = parse_route_goal_id(previous_identifier)
    following = parse_route_goal_id(next_identifier)
    return (
        previous is not None
        and following is not None
        and previous[0] == following[0]
        and previous[1] + 1 == following[1]
        and previous[2] == INTERMEDIATE_GOAL_KIND
    )


def is_intermediate_route_goal_id(identifier):
    """Return whether the action goal is a managed non-terminal subgoal."""
    return route_goal_kind(identifier) == INTERMEDIATE_GOAL_KIND
