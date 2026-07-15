import math

from nav_world_model.risk_evidence import (
    RelativeTrack,
    TTC_NO_CONFLICT,
    TTC_OBSERVED_CONFLICT,
    TTC_TRACKER_INVALID,
    classify_ttc_evidence,
    earliest_relative_ttc,
    oriented_box_clearance,
    rectangular_footprint_clearance,
)


class Scan:
    angle_min = -math.pi
    angle_increment = math.pi / 2.0
    range_min = 0.01
    range_max = 20.0

    def __init__(self, ranges):
        self.ranges = ranges


def test_unknown_relative_track_can_produce_finite_ttc():
    value = earliest_relative_ttc((RelativeTrack(
        x=3.0, y=0.2, vx=-1.0, vy=0.0, radius=0.3,
        confidence=0.9, motion_class="UNKNOWN",
    ),))
    assert value is not None
    assert 1.9 < value < 2.3


def test_departing_and_non_conflicting_tracks_have_no_ttc():
    departing = RelativeTrack(3.0, 0.0, 1.0, 0.0, 0.3, 0.9, "DEPARTING")
    passing = RelativeTrack(3.0, 3.0, -1.0, 0.0, 0.3, 0.9, "UNKNOWN")
    assert earliest_relative_ttc((departing, passing)) is None


def test_ttc_evidence_three_states_are_disjoint():
    assert classify_ttc_evidence(
        tracker_message_count=0, healthy_tracker_sample_count=0,
        finite_ttc_sample_count=0,
    ) == TTC_TRACKER_INVALID
    assert classify_ttc_evidence(
        tracker_message_count=5, healthy_tracker_sample_count=5,
        finite_ttc_sample_count=0,
    ) == TTC_NO_CONFLICT
    assert classify_ttc_evidence(
        tracker_message_count=5, healthy_tracker_sample_count=5,
        finite_ttc_sample_count=1,
    ) == TTC_OBSERVED_CONFLICT


def test_signed_clearance_preserves_intrusion_hidden_by_legacy_clip():
    evidence = rectangular_footprint_clearance(Scan((2.0, 0.30, 2.0, 2.0)))
    assert evidence.ray_index == 1
    assert evidence.signed_clearance_m < 0.0
    assert evidence.clipped_clearance_m == 0.0


def test_oriented_box_clearance_distinguishes_overlap_and_gap():
    assert oriented_box_clearance(
        (0.0, 0.0, 0.0), (1.04, 0.70),
        (0.7, 0.0, math.pi / 2.0), (0.55, 0.55),
    ) == 0.0
    gap = oriented_box_clearance(
        (0.0, 0.0, 0.0), (1.04, 0.70),
        (2.0, 0.0, 0.0), (0.55, 0.55),
    )
    assert abs(gap - 1.205) < 1.0e-9
