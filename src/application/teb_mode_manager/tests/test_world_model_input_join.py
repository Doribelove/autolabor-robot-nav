from teb_mode_manager.world_model_input_join import BoundedWorldModelInputJoin


def _join(**changes):
    values = dict(
        maximum_entries_per_stream=4,
        maximum_arrival_age_s=1.0,
        maximum_sequence_lag=2,
        maximum_timestamp_spread_s=0.05,
    )
    values.update(changes)
    return BoundedWorldModelInputJoin(**values)


def _add_triplet(join, sequence, source=1.0, arrivals=(1.01, 1.02, 1.03)):
    for stream, arrival in zip(("geometry", "tracks", "health"), arrivals):
        join.add(stream, sequence, source, arrival, "{}{}".format(stream, sequence))


def test_partial_new_publication_uses_previous_complete_atomic_triplet():
    join = _join()
    _add_triplet(join, 10)
    join.add("geometry", 11, 1.10, 1.11, "geometry11")
    result = join.resolve(1.12)
    assert result.valid
    assert result.world_model_sequence == 10
    assert result.newest_observed_sequence == 11
    assert result.sequence_lag == 1
    assert result.reason == "ATOMIC_EXACT_SEQUENCE_JOIN"
    assert result.payloads == {
        "geometry": "geometry10", "tracks": "tracks10", "health": "health10"
    }


def test_out_of_order_delivery_becomes_visible_only_when_triplet_is_complete():
    join = _join()
    join.add("health", 20, 2.0, 2.01, "h20")
    join.add("geometry", 20, 2.0, 2.02, "g20")
    assert not join.resolve(2.03).valid
    join.add("tracks", 20, 2.0, 2.04, "t20")
    result = join.resolve(2.05)
    assert result.valid
    assert result.world_model_sequence == 20


def test_dropped_stream_fails_closed_after_bounded_sequence_lag():
    join = _join(maximum_sequence_lag=1)
    _add_triplet(join, 30, source=3.0, arrivals=(3.01, 3.02, 3.03))
    join.add("geometry", 31, 3.1, 3.11, "g31")
    join.add("tracks", 31, 3.1, 3.12, "t31")
    join.add("geometry", 32, 3.2, 3.21, "g32")
    result = join.resolve(3.22)
    assert not result.valid
    assert result.reason == "WORLD_MODEL_SEQUENCE_LAG_EXCEEDED"
    assert result.sequence_lag == 2


def test_cross_sequence_synthesis_is_never_allowed():
    join = _join()
    join.add("geometry", 40, 4.0, 4.01, "g40")
    join.add("tracks", 41, 4.1, 4.11, "t41")
    join.add("health", 42, 4.2, 4.21, "h42")
    result = join.resolve(4.22)
    assert not result.valid
    assert result.reason == "NO_COMPLETE_WORLD_MODEL_SEQUENCE"


def test_source_timestamp_spread_is_bounded():
    join = _join(maximum_timestamp_spread_s=0.02)
    join.add("geometry", 50, 5.00, 5.01, "g50")
    join.add("tracks", 50, 5.01, 5.02, "t50")
    join.add("health", 50, 5.03, 5.04, "h50")
    result = join.resolve(5.05)
    assert not result.valid
    assert result.reason == "WORLD_MODEL_TIMESTAMP_SPREAD_EXCEEDED"


def test_duplicate_sequence_replaces_only_its_own_stream_entry():
    join = _join()
    _add_triplet(join, 60, source=6.0, arrivals=(6.01, 6.02, 6.03))
    join.add("tracks", 60, 6.0, 6.04, "tracks60-new")
    result = join.resolve(6.05)
    assert result.valid
    assert result.payloads["tracks"] == "tracks60-new"
    assert result.payloads["geometry"] == "geometry60"


def test_capacity_age_and_simulation_clock_rollback_clear_safely():
    join = _join(maximum_entries_per_stream=2, maximum_arrival_age_s=0.5)
    _add_triplet(join, 70, source=7.0, arrivals=(7.01, 7.02, 7.03))
    _add_triplet(join, 71, source=7.1, arrivals=(7.11, 7.12, 7.13))
    _add_triplet(join, 72, source=7.2, arrivals=(7.21, 7.22, 7.23))
    assert join.cache_sizes == {"geometry": 2, "tracks": 2, "health": 2}
    assert join.resolve(8.0).reason == "WORLD_MODEL_INPUT_STREAM_MISSING"
    _add_triplet(join, 73, source=8.0, arrivals=(8.01, 8.02, 8.03))
    assert join.resolve(1.0).reason == "WORLD_MODEL_INPUT_STREAM_MISSING"


def test_future_source_timestamp_is_rejected():
    join = _join()
    _add_triplet(join, 80, source=8.5, arrivals=(8.01, 8.02, 8.03))
    result = join.resolve(8.1)
    assert not result.valid
    assert result.reason == "WORLD_MODEL_SOURCE_TIMESTAMP_IN_FUTURE"
