from teb_mode_manager.bounded_context_join import BoundedContextJoin


def _join(**changes):
    values = dict(
        maximum_entries=4,
        maximum_arrival_age_s=1.0,
        maximum_sequence_delta=2,
        maximum_timestamp_delta_s=0.45,
    )
    values.update(changes)
    return BoundedContextJoin(**values)


def test_exact_sequence_is_recovered_after_newer_geometry_arrives():
    join = _join()
    join.add(10, 1.00, 1.01, "g10")
    join.add(11, 1.10, 1.11, "g11")
    result = join.resolve(10, 1.20, 1.21)
    assert result.valid
    assert result.payload == "g10"
    assert result.reason == "EXACT_SEQUENCE_JOIN"
    assert result.sequence_delta == 0


def test_bounded_non_future_fallback_is_explicit():
    join = _join()
    join.add(20, 2.00, 2.01, "g20")
    result = join.resolve(21, 2.20, 2.21)
    assert result.valid
    assert result.payload == "g20"
    assert result.reason == "BOUNDED_SEQUENCE_TIME_JOIN"
    assert result.sequence_delta == 1


def test_invalid_exact_timestamp_can_fall_back_to_previous_bounded_entry():
    join = _join()
    join.add(20, 2.00, 2.01, "previous")
    join.add(21, 2.30, 2.31, "same_sequence_but_future_stamp")
    result = join.resolve(21, 2.20, 2.32)
    assert result.valid
    assert result.payload == "previous"
    assert result.reason == "BOUNDED_SEQUENCE_TIME_JOIN"
    assert result.sequence_delta == 1


def test_future_geometry_is_never_used_for_older_context():
    join = _join()
    join.add(31, 3.10, 3.11, "future")
    result = join.resolve(30, 3.20, 3.21)
    assert not result.valid
    assert result.reason == "NO_BOUNDED_GEOMETRY_MATCH"


def test_sequence_and_timestamp_bounds_fail_closed():
    join = _join()
    join.add(40, 4.00, 4.01, "old")
    assert not join.resolve(43, 4.20, 4.21).valid
    join.add(50, 5.00, 5.01, "late")
    assert not join.resolve(50, 5.50, 5.51).valid


def test_stale_entries_are_purged_and_time_reset_clears_cache():
    join = _join()
    join.add(60, 6.00, 6.01, "stale")
    assert join.resolve(60, 6.20, 7.02).reason == "GEOMETRY_CACHE_EMPTY"
    join.add(61, 7.10, 7.11, "before_reset")
    assert join.resolve(61, 7.20, 1.00).reason == "GEOMETRY_CACHE_EMPTY"


def test_cache_capacity_is_bounded():
    join = _join(maximum_entries=2)
    join.add(1, 1.0, 1.0, "one")
    join.add(2, 1.1, 1.1, "two")
    join.add(3, 1.2, 1.2, "three")
    assert join.size == 2
    assert join.resolve(1, 1.3, 1.3).reason == "NO_BOUNDED_GEOMETRY_MATCH"
