"""M02 offline component tests; no ROS master, simulator, or trained policy.

The observation-parity reference is the unchanged R1 callback and real runtime,
not a second copy of the new queue implementation.
"""
import importlib.util
import json
import os
from pathlib import Path
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from v5_e2e_scope_rl.astra_input_sync import TimestampedScanQueue, TransformPending


def resolved(payload, stamp):
    return (payload, stamp)


def fill_history(queue):
    for stamp in (1.0, 1.25, 1.5, 1.75, 2.0, 2.25):
        assert queue.submit(stamp, stamp, stamp)
        assert queue.drain(stamp, resolved, lambda *args: None) == 1


@pytest.mark.parametrize('delay', [.002, .020])
def test_late_transform_wait_is_not_a_terminal_failure(delay):
    queue = TimestampedScanQueue()
    assert queue.submit(1.0, 'scan', 1.0)
    calls, commits = [], []

    def unavailable(payload, stamp):
        calls.append((payload, stamp))
        raise TransformPending('transform will arrive')

    assert queue.drain(1.0, unavailable, lambda *args: commits.append(args)) == 0
    assert queue.drain(1.001, unavailable, lambda *args: commits.append(args)) == 0
    assert queue.terminal_failure_count == 0
    assert queue.drain(1.0+delay, resolved, lambda *args: commits.append(args)) == 1
    assert commits == [('scan', ('scan', 1.0), 1.0)]
    assert calls == [('scan', 1.0), ('scan', 1.0)]
    assert queue.counts['tf_wait_scan_count'] == 1
    assert queue.counts['tf_wait_poll_count'] == 2
    assert queue.report()['maximum_completed_tf_wait_s'] == pytest.approx(delay)
    assert queue.terminal_failure_count == 0


@pytest.mark.parametrize('age,accepted', [(.199999999, True), (.2, True), (.200000001, False)])
def test_exact_age_boundary_on_arrival(age, accepted):
    queue = TimestampedScanQueue()
    assert queue.submit(1.0, 'scan', 1.0+age) is accepted
    committed = []
    queue.drain(1.0+age, resolved, lambda *args: committed.append(args))
    assert bool(committed) is accepted
    assert queue.counts['expired_count'] == (0 if accepted else 1)


def test_expiration_after_wait_does_not_invoke_late_transform_or_commit():
    queue = TimestampedScanQueue()
    queue.submit(1.0, 'scan', 1.0)

    def unavailable(*args):
        raise TransformPending()

    queue.drain(1.0, unavailable, lambda *args: pytest.fail('premature commit'))
    queue.drain(1.200000001, lambda *args: pytest.fail('expired lookup'),
                lambda *args: pytest.fail('expired commit'))
    assert queue.counts['expired_count'] == 1
    assert queue.terminal_failure_count == 1
    assert queue.events[-1]['reason'] == 'expired_waiting_for_transform'
    assert queue.events[-1]['tf_wait_s'] == pytest.approx(.200000001)


def test_six_actual_samples_and_stale_committed_history_are_separate_from_raw_scan():
    queue = TimestampedScanQueue()
    assert not queue.health(.5)['healthy']
    fill_history(queue)
    assert queue.health(2.45)['healthy']  # Exact 0.20 s inclusive boundary.
    assert queue.submit(2.5, 'new raw scan without TF', 2.5)

    def unavailable(*args):
        raise TransformPending()

    queue.drain(2.5, unavailable, lambda *args: pytest.fail('missing TF'))
    health = queue.health(2.5)
    assert not health['healthy']
    assert health['real_history_count'] == 6
    assert health['latest_committed_scan_age_s'] == .25
    assert 'committed_scan_stale' in health['failures']


@pytest.mark.parametrize('stamp', [0.0, -1.0, float('nan'), float('inf'), None])
def test_missing_or_invalid_stamp_never_becomes_latest_tf(stamp):
    queue = TimestampedScanQueue()
    assert not queue.submit(stamp, 'invalid', 1.0)
    queue.drain(1.0, lambda *args: pytest.fail('invalid TF request'), lambda *args: None)
    assert queue.counts['invalid_stamp_count'] == 1
    assert queue.terminal_failure_count == 1


def test_duplicate_and_out_of_order_are_visible_and_bounded_queue_drops_are_visible():
    queue = TimestampedScanQueue()
    assert queue.submit(1.0, 1.0, 1.0)
    assert not queue.submit(1.0, 'duplicate', 1.0)
    assert not queue.submit(.99, 'out of order', 1.0)
    for index in range(1, 8):
        assert queue.submit(1+index*.001, index, 1+index*.001)
    assert not queue.submit(1.008, 'overflow', 1.008)
    assert len(queue.pending) == queue.maximum_queue_depth == 8
    assert queue.counts['out_of_order_count'] == 2
    assert queue.counts['overflow_count'] == 1
    assert queue.terminal_failure_count == 3


def test_future_stamp_waits_for_clock_and_large_future_stamp_is_rejected():
    queue = TimestampedScanQueue()
    assert queue.submit(1.02, 'future', 1.0)
    queue.drain(1.0, lambda *args: pytest.fail('future TF lookup'), lambda *args: None)
    committed = []
    queue.drain(1.02, resolved, lambda *args: committed.append(args))
    assert committed[0][-1] == 1.02
    assert not queue.submit(1.5, 'far future', 1.02)
    assert queue.counts['future_stamp_rejected_count'] == 1


def test_clock_reset_flushes_history_and_pending_and_cannot_silently_resume():
    queue = TimestampedScanQueue()
    fill_history(queue)
    queue.submit(2.5, 'pending', 2.5)
    assert not queue.health(.5)['healthy']
    assert not queue.pending and not queue.history_stamps_ns
    assert queue.epoch == 1
    assert queue.counts['clock_reset_count'] == 1
    assert queue.counts['clock_reset_discarded_pending'] == 1
    assert not queue.submit(.6, 'new epoch', .6)
    assert queue.drain(.6, resolved, lambda *args: pytest.fail('resumed reset episode')) == 0
    assert queue.fault_reason == 'clock_reset_requires_new_episode'


@pytest.mark.parametrize('stage', ['resolve', 'commit'])
def test_unexpected_error_latches_fault_without_fake_commit(stage):
    queue = TimestampedScanQueue()
    queue.submit(1.0, 'scan', 1.0)

    def broken(*args):
        raise RuntimeError('fixture failure')

    queue.drain(1.0, broken if stage == 'resolve' else resolved,
                broken if stage == 'commit' else lambda *args: None)
    assert queue.fault_reason
    assert not queue.history_stamps_ns
    assert queue.counts['committed_count'] == 0
    assert queue.terminal_failure_count == 1
    json.dumps(queue.report(), allow_nan=False)


@pytest.fixture(scope='module')
def node_module():
    workspace = Path(__file__).resolve().parents[4]
    assert Path(os.environ['THESIS_WS']).resolve() == workspace
    path = workspace/'src/application/v5_e2e_scope_rl/scripts/astra_sync_episode.py'
    spec = importlib.util.spec_from_file_location('astra_sync_component_test_node', str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def offline_node(node_module, monkeypatch):
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan

    clock = SimpleNamespace(now=1.0)
    monkeypatch.setattr(node_module.rospy.Time, 'now',
                        staticmethod(lambda: node_module.rospy.Time.from_sec(clock.now)))
    node = node_module.AstraSyncEpisode.__new__(node_module.AstraSyncEpisode)
    node.input_sync_mode = 'synchronized'
    node._sync_lock = threading.RLock()
    node._scan_queue = TimestampedScanQueue()
    node._seen_epoch = 0
    node._node_fault = None
    node._command_gated_count = node._watchdog_stop_count = 0
    node._bootstrap_real_history_wait_wall_s = 0.0
    node._last_sync_health = None
    node._maximum_committed_scan_age_s = 0.0
    node.scan = node.odom = None
    node.scan_received_s = node.odom_received_s = None
    node.last_runtime_scan_stamp_s = None
    node.scan_stamps = []
    node.scan_transform_success_count = node.scan_transform_failure_count = 0
    node.tf_success_count = node.tf_failure_count = 0
    node.maximum_scan_age_s = node.maximum_odom_age_s = node.maximum_tf_age_s = 0.0
    node.minimum_scan_m = float('inf')
    node.collecting = False
    node.spawn_world_x = -5.4
    node.runtime = node_module.r2.r1.DirectionalBenchmarkRuntimeAdapter()
    node.transform_available = True
    node.lookup_calls = []

    def lookup(target, source, stamp, timeout):
        node.lookup_calls.append((target, source, stamp.to_nsec(), timeout.to_sec()))
        if not node.transform_available:
            raise node_module.tf2_ros.ExtrapolationException('future extrapolation fixture')
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = target
        transform.child_frame_id = source
        transform.transform.translation.x = .4*stamp.to_sec()
        transform.transform.rotation.w = 1.0
        return transform

    node.tf_buffer = SimpleNamespace(lookup_transform=lookup)

    def scan(stamp, frame='laser_link', index=0):
        message = LaserScan()
        message.header.stamp = node_module.rospy.Time.from_sec(stamp)
        message.header.frame_id = frame
        message.range_max = 8.0
        message.angle_min = -np.pi
        message.angle_increment = 2*np.pi/180
        message.ranges = (3+.2*np.sin(np.arange(180)*.1+index*.02)).tolist()
        return message

    def odom(stamp):
        node.odom = Odometry()
        node.odom.header.stamp = node_module.rospy.Time.from_sec(stamp)
        node.odom.header.frame_id = 'odom'
        node.odom_received_s = stamp

    node.make_scan, node.set_odom, node.clock = scan, odom, clock
    return node


def test_ros_adapter_uses_exact_positive_scan_stamp_and_zero_timeout(offline_node):
    node = offline_node
    node.transform_available = False
    node._scan(node.make_scan(1.0))
    assert node.scan_transform_failure_count == node.scan_transform_success_count == 0
    assert node._scan_queue.counts['tf_wait_scan_count'] == 1
    node.transform_available = True
    node.clock.now = 1.02
    node._drain_locked()
    assert node.scan_transform_success_count == 1
    assert node.scan_transform_failure_count == 0
    assert node.lookup_calls == [('odom', 'laser_link', 1_000_000_000, 0.0)]*2
    assert node.runtime.latest_scan_stamp_s == 1.0
    assert node.runtime.latest_scan_pose_world == pytest.approx((-5.0, 0.0, 0.0))


def test_wrong_timestamp_tf_cannot_be_committed(offline_node, node_module):
    node = offline_node
    original = node.tf_buffer.lookup_transform

    def wrong_stamp(*args):
        result = original(*args)
        result.header.stamp = node_module.rospy.Time.from_sec(1.01)
        return result

    node.tf_buffer.lookup_transform = wrong_stamp
    node._scan(node.make_scan(1.0))
    assert node.scan_transform_success_count == 0
    assert node.scan_transform_failure_count == 1
    assert node._scan_queue.fault_reason == 'transform_resolver_error:ValueError'


def test_no_delay_history_and_tracker_match_frozen_legacy_callback(offline_node, node_module):
    node = offline_node
    legacy = node_module.AstraSyncEpisode.__new__(node_module.AstraSyncEpisode)
    legacy.__dict__.update(node.__dict__)
    legacy.input_sync_mode = 'legacy'
    legacy.runtime = node_module.r2.r1.DirectionalBenchmarkRuntimeAdapter()
    legacy.scan_stamps = []
    # Real ROS nanosecond stamps exercise subtraction boundaries such as
    # 1.252-1.052 < .20; do not replace the inherited sampler by integer ceil.
    for index in range(100):
        stamp = (1_052_000_000 + index*50_000_000)/1e9
        node.clock.now = stamp
        message = node.make_scan(stamp, index=index)
        node._scan(message)
        legacy._scan(message)
        np.testing.assert_array_equal(node.runtime.scan_history, legacy.runtime.scan_history)
        assert node.last_runtime_scan_stamp_s == legacy.last_runtime_scan_stamp_s
        assert node.runtime.latest_scan_pose_world == legacy.runtime.latest_scan_pose_world
    assert node.scan_transform_success_count == legacy.scan_transform_success_count
    assert node.scan_transform_success_count > 6
    assert node._scan_queue.terminal_failure_count == 0
    np.testing.assert_array_equal(node.runtime.dynamic_velocity, legacy.runtime.dynamic_velocity)


def test_real_bootstrap_contract_and_fresh_history_required_for_nonzero_commands(
        offline_node, node_module, monkeypatch):
    node = offline_node
    commands = []
    monkeypatch.setattr(node_module.r2.R2BenchmarkEpisode, 'publish_command',
                        lambda self, speed=0., yaw_rate=0.: commands.append((speed, yaw_rate)))
    # The inherited runtime already contains six maximum-range placeholder rows.
    assert len(node.runtime.scan_history) == 6
    node._scan(node.make_scan(1.0))
    node.set_odom(1.0)
    assert 'transform_invalid' in node.input_health(1.0).failures(.20)
    node.publish_command(.8, .1)
    assert commands[-1] == (0.0, 0.0)
    for stamp in (1.25, 1.5, 1.75, 2.0, 2.25):
        node.clock.now = stamp
        node._scan(node.make_scan(stamp))
    node.set_odom(2.25)
    assert not node.input_health(2.25).failures(.20)
    node.publish_command(.8, .1)
    assert commands[-1] == (.8, .1)
    node.clock.now = 2.5
    node.set_odom(2.5)
    node.transform_available = False
    node._scan(node.make_scan(2.5))  # Fresh raw scan cannot hide stale history.
    assert 'scan_stale' in node.input_health(2.5).failures(.20)
    node.publish_command(.8, .1)
    assert commands[-1] == (0.0, 0.0)
    assert node._command_gated_count == 2
    node.collecting = True
    node._sync_tick(None)
    assert node._watchdog_stop_count == 1
    assert node.scan_transform_failure_count == 0  # Still awaiting unexpired TF.


def test_clock_reset_detected_by_health_clears_real_runtime_and_counts_failure(offline_node):
    node = offline_node
    node._scan(node.make_scan(1.0))
    node.clock.now = .5
    node.set_odom(.5)
    health = node._input_health_locked()
    assert health.failures(.20)
    assert node.scan_transform_failure_count == 1
    assert node.runtime.latest_scan_stamp_s is None
    assert node.runtime.latest_scan_pose_world is None
    assert node.runtime.track_history == [] or len(node.runtime.track_history) == 0
    assert np.all(node.runtime.scan_history == node.runtime.maximum_range_m)
    assert not node._scan_queue.submit(.6, 'new clock epoch', .6)


def test_zero_odom_header_stamp_is_not_replaced_by_receipt_time_in_health(offline_node):
    node = offline_node
    node.set_odom(0.0)
    node.odom_received_s = 1.0  # Frozen _odom's receipt fallback must not pass.
    assert 'odometry_age_invalid' in node._input_health_locked().failures(.20)


def test_legacy_mode_delegates_callbacks_and_commands_unchanged(offline_node, node_module, monkeypatch):
    node = offline_node
    node.input_sync_mode = 'legacy'
    calls = []
    parent = node_module.r2.R2BenchmarkEpisode
    monkeypatch.setattr(parent, '_scan', lambda self, message: calls.append(('scan', message)))
    monkeypatch.setattr(parent, 'input_health', lambda self, now: ('legacy_health', now))
    monkeypatch.setattr(parent, 'publish_command', lambda self, speed=0., yaw_rate=0.: calls.append(('cmd', speed, yaw_rate)))
    node._scan('original object')
    assert node.input_health(4.0) == ('legacy_health', 4.0)
    node.publish_command(.8, .2)
    assert calls == [('scan', 'original object'), ('cmd', .8, .2)]
    assert not node._scan_queue.counts


@pytest.mark.parametrize('method', ['actual_teb', 'dwa'])
def test_sync_rejects_external_move_base_publisher_before_ros_start(node_module, monkeypatch, method):
    parameters = {'~input_sync_mode': 'synchronized', '~method': method}
    monkeypatch.setattr(node_module.rospy, 'get_param', lambda name, default=None: parameters.get(name, default))
    with pytest.raises(ValueError, match='external move_base publisher'):
        node_module.AstraSyncEpisode()
