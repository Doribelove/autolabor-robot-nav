"""Independent bounded M08 checks; no ROS, benchmark, training, or GPU."""
import ast
import dataclasses
import hashlib
import inspect
import json
import math
from pathlib import Path
import sys
import threading
import time
import traceback
from types import SimpleNamespace

import numpy as np
from shapely.geometry import Point, Polygon

ROOT = Path(__file__).resolve().parent
STAGE = ROOT.parents[1]
SCRIPTS = STAGE/'workspace/scripts/astra_gazebo'
sys.path.insert(0, str(SCRIPTS))
import sensor_tracks_v1 as v1
from sensor_tracks_v2 import BatchHardOccupancy, ConvexFootprint
from sensor_tracks_ingestion_m08 import IngestionSensorTracks, _clusters
from sensor_tracks_query_m08 import DominanceBatchHardOccupancy
from route_sensor_buffer import ScanFrame
from route_tracks_sensor_io_m08 import TrackSensorBuffer, SharedLifecycle
from route_tracks_actor_adapter_m08 import DeadlineActor, ActorDeadlineExceeded, validate_metadata
from route_candidate_core_m08 import RouteTracksRuntime, prepublish_guard, RouteConfig
from route_candidate_core import RouteAwareRuntime, FullFootprintEnvelope

ROWS = []


def check(name, operation):
    try:
        details = operation()
        ROWS.append(dict(name=name, passed=True, details=details))
    except Exception:
        ROWS.append(dict(name=name, passed=False, error=traceback.format_exc()))


def raises(operation):
    try:
        operation()
    except (ValueError, RuntimeError):
        return
    raise AssertionError('invalid boundary unexpectedly accepted')


class Grid:
    image_sha256 = 'a'*64
    yaml_sha256 = 'b'*64
    def clearance(self, point):
        return 2.


def accept(tracker, stamp, points, now=None):
    frame = ScanFrame(stamp, np.full(72, 8.), np.asarray(points).reshape(-1, 2), (0., 0., 0.))
    snapshot = tracker.accept(frame, tf_stamp_ns=stamp,
        odom_state=dict(stamp_ns=stamp, pose_map=(0., 0., 0.), body_twist=(0., 0., 0.)),
        now_ns=stamp if now is None else now)
    return snapshot


def acceptance_parity():
    assert inspect.getsource(IngestionSensorTracks.accept) == inspect.getsource(v1.SensorTracks.accept)
    for name in ('state', 'step', 'rollout', 'goal_reached'):
        assert getattr(RouteTracksRuntime, name) is getattr(RouteAwareRuntime, name)
    return dict(accept_body_byte_equal=True, inherited_functions=4)


def cluster_reference():
    # Independent all-pairs adjacency/transitive closure, not KD traversal.
    rng = np.random.RandomState(8082026)
    cases = [rng.uniform(-1., 1., (24, 2))]
    for shift in (0., -9999., 9999.):
        for gap in (np.nextafter(.3, 0.), .3, np.nextafter(.3, 1.)):
            cases.append(np.asarray([(0., 0.), (gap, 0.), (2*gap, 0.), (2*gap, gap), (2., 2.), (2., 2.)])+shift)
    for points in cases:
        count = len(points)
        connected = np.eye(count, dtype=bool)
        for i in range(count):
            for j in range(count):
                connected[i, j] = np.linalg.norm(points[i]-points[j]) <= .3
        for k in range(count):
            connected |= connected[:, k, None] & connected[None, k, :]
        expected = []; used = set()
        for i in range(count):
            if i not in used:
                ids = list(np.flatnonzero(connected[i])); used.update(ids)
                expected.append(points[ids])
        got = _clusters(points, .3)
        assert len(expected) == len(got)
        for a, b in zip(expected, got):
            np.testing.assert_array_equal(a, b)
    return dict(cases=len(cases), reference='all_pairs_transitive_closure_original_scalar_norm')


def geometric_dominance():
    tracker = IngestionSensorTracks(Grid())
    points = [(x, y) for x in (2., 2.4, 2.8) for y in (2., 2.4, 2.8)]
    accept(tracker, 1_000_000_000, points)
    snapshot = accept(tracker, 1_100_000_000, points)
    fast = DominanceBatchHardOccupancy(snapshot, 1_150_000_000)
    full = BatchHardOccupancy(snapshot, 1_150_000_000)
    persistent = len(snapshot.tracks)+len(snapshot.sparse_supports)
    assert set(range(persistent)) <= set(fast.query_indices)
    assert len(fast.dominance_certificates) > 0
    for point, circle, slack in fast.dominance_certificates:
        distance = math.hypot(*(full.centers[point]-full.centers[circle]))
        assert circle < persistent and full.radii[circle] > distance+full.radii[point]+1e-9
    urdf = (STAGE/'runs/passive_sweep_q05_static/robot_description_runtime.xml').read_text()
    hull = FullFootprintEnvelope(urdf, RouteConfig())
    foot = ConvexFootprint(hull.vertices)
    poses = np.asarray([(1.5, 2., 0.), (2.4, 2.4, .6), (4., 4., -1.2), (2.8, 1., 2.1)])
    offsets = np.asarray([0, 50_000_000, 200_000_000, 1_000_000_000], dtype=np.int64)
    got = fast.distance_lower_bounds(foot, poses, offsets)
    reference = []
    for pose, dt in zip(poses, offsets):
        c, s = math.cos(pose[2]), math.sin(pose[2])
        poly = Polygon([(pose[0]+c*x-s*y, pose[1]+s*x+c*y) for x, y in foot.vertices])
        reference.append(min(poly.distance(Point(float(x), float(y)))-float(r)
                             for (x, y), r in zip(full.centers, full.radii))-1.35*int(dt)/1e9)
    error = got-np.asarray(reference)
    assert np.all(error <= 1e-12) and np.max(np.abs(error)) < 2e-10
    assert any(got < 0) and any(got > 0)
    return dict(full_supports=full.support_count, queried_supports=len(fast.centers),
        retained_persistent=persistent, independent_GEOS_queries=len(poses),
        delta_max=float(np.max(error)), delta_absmax=float(np.max(np.abs(error))))


def late_availability():
    tracker = IngestionSensorTracks(Grid())
    snapshot = accept(tracker, 1_000_000_000, [(2., 2.)], 1_190_000_000)
    raises(lambda: DominanceBatchHardOccupancy(snapshot, 1_189_999_999))
    DominanceBatchHardOccupancy(snapshot, 1_200_000_000)
    raises(lambda: DominanceBatchHardOccupancy(snapshot, 1_200_000_001))
    return dict(accepted_age_ns=190_000_000, original_expiry_ns=1_200_000_000, no_lifetime_extension=True)


def clock_reset():
    tracker = IngestionSensorTracks(Grid())
    snapshot = accept(tracker, 1_000_000_000, [(2., 2.)])
    tracker.health(1_100_000_000)
    raises(lambda: tracker.health(1_099_999_999))
    raises(lambda: accept(tracker, 1_150_000_000, []))
    assert tracker.snapshot is snapshot
    return dict(permanent_fault=tracker.fault, old_snapshot_preserved=True)


def causal_io_and_partial_commit():
    tracker = IngestionSensorTracks(Grid())
    core = SimpleNamespace(tracker=tracker)
    core.accept_scan = lambda frame, tf, odom, now: tracker.accept(frame, tf_stamp_ns=tf, odom_state=odom, now_ns=now)
    b = TrackSensorBuffer(core)
    scan = lambda stamp: b.scan(stamp, stamp, np.full(720, np.inf), -math.pi, 2*math.pi/720, .1, 30., (0., 0., 0.), stamp)
    b.odom(1_000_000_001, (0., 0., 0.), (0., 0., 0.))
    raises(lambda: scan(1_000_000_000))
    assert b.fresh is None and tracker.snapshot is None
    b.odom(1_000_000_000, (0., 0., 0.), (0., 0., 0.)); scan(1_000_000_000)
    old = tracker.snapshot
    def reject(*args):
        raise ValueError('independent injected tracker commit rejection')
    core.accept_scan = reject
    raises(lambda: scan(1_050_000_000))
    assert tracker.snapshot is old and b.fresh.stamp_ns == 1_050_000_000
    assert 'fresh_scan_tracker_commit_mismatch' in b.health(1_050_000_000)
    # The node catches the actual propagated error and permanently zero/faults;
    # the pure IO buffer is intentionally not a rollback transaction.
    return dict(causal_wait_does_not_consume=True, pure_IO_partial_commit_detected=True,
                standalone_buffer_atomicity=False, node_fault_required=True)


def exact_join_and_history():
    tracker = IngestionSensorTracks(Grid())
    core = SimpleNamespace(tracker=tracker)
    core.accept_scan = lambda frame, tf, odom, now: tracker.accept(frame, tf_stamp_ns=tf, odom_state=odom, now_ns=now)
    b = TrackSensorBuffer(core)
    names = ['front_left_steer_joint', 'front_right_steer_joint']
    for i in range(6):
        t = 1_000_000_000+i*200_000_000
        b.odom(t, (0., 0., 0.), (0., 0., 0.))
        b.joint(t+1, names, [0., 0.])
        if i == 0: assert b.state is None
        b.joint(t, names, [0., 0.])
        assert b.state['stamp_ns'] == t
        b.scan(t, t, np.full(720, np.inf), -math.pi, 2*math.pi/720, .1, 30., (0., 0., 0.), t)
        assert len(b.history) == i+1
    history, buckets, stamps = b.model_history()
    assert len(set(stamps)) == 6 and stamps == buckets and not b.health(t)
    return dict(exact_odom_joint_join=True, distinct_model_rows=len(set(stamps)))


def deadline_boundaries():
    config = RouteConfig(); command = (.3, .02, 3, None, .1)
    assert prepublish_guard(command, 1_200_000_000, 1_000_000_000, 1_000_000_000, 50., config) == command
    for now, stamp, elapsed in ((1_200_000_001, 1_000_000_000, 50.),
                                (1_200_000_000, 1_000_000_000, 50.000001),
                                (1_000_000_000, 1_000_000_001, 0.)):
        got = prepublish_guard(command, now, stamp, stamp, elapsed, config)
        assert got[:3] == (0., 0., 0)
    return dict(inclusive_200ms_50ms=True, one_ns_and_future_rejected=True)


def worker_private_result():
    release = threading.Event(); done = threading.Event()
    original = np.asarray([1., 2.]); retained = {}
    def blocked(observation):
        retained['observation'] = observation
        release.wait(.2); done.set(); return 7
    actor = DeadlineActor(blocked)
    raises(lambda: actor({'x': original}))
    original[0] = 99.
    assert retained['observation']['x'][0] == 1.
    raises(lambda: actor({'x': np.asarray([3.])}))
    release.set(); assert done.wait(.2)
    deadline = time.monotonic()+.2
    while actor.busy and time.monotonic() < deadline: time.sleep(.001)
    actor.actor = lambda observation: 2
    assert actor({'x': np.asarray([4.])}) == 2
    return dict(one_worker=True, old_output_discarded=True, observation_copy_isolated=True, timing_claim=False)


def namespace_and_terminal():
    raises(lambda: validate_metadata(dict(observation_schema='astra_route_observation_v1'), SimpleNamespace(semantics_sha256='a'*64)))
    method='astra_analytic_route_tracks_m08'; life=SharedLifecycle('test', 1, method)
    identity=dict(schema='astra_common_navigation_v1', run_id='test', seed=1, method=method)
    life.gate(dict(identity, source_ns=1_000_000_000, deadline_ns=23_000_000_000))
    life.fail('actual sensor failure')
    life.status(dict(identity, phase='complete', terminal_source_ns=2_000_000_000,
                     terminal_reason='shared_actual_goal', terminal_time_authority='actual_odom'))
    life.fail('late sensor failure')
    assert life.state['phase']=='complete' and life.state['fault']=='actual sensor failure'
    assert life.state['late_fault_count']==1 and life.state['selected_action']==0
    return dict(old_namespace_rejected=True, task_and_local_integrity_separate=True)


for name, function in (
    ('exact_accept_and_rollout_inheritance', acceptance_parity),
    ('KD_components_independent_transitive_reference', cluster_reference),
    ('strict_dominance_independent_GEOS_reference', geometric_dominance),
    ('late_accept_original_source_expiry', late_availability),
    ('clock_reset_permanent_fault', clock_reset),
    ('causal_wait_and_partial_commit_detection', causal_io_and_partial_commit),
    ('exact_joint_join_six_real_history', exact_join_and_history),
    ('deadline_and_source_age_boundary', deadline_boundaries),
    ('private_worker_expired_result', worker_private_result),
    ('checkpoint_namespace_and_shared_terminal', namespace_and_terminal)):
    check(name, function)

result = dict(schema='astra_m08_independent_small_fixtures_v1', interpreter=sys.executable,
    numpy_version=np.__version__, numpy_path=np.__file__, checks=ROWS,
    passed=sum(row['passed'] for row in ROWS), total=len(ROWS),
    no_ros=True, no_benchmark=True, no_gpu_import=('torch' not in sys.modules),
    frozen_source_manifest_sha256=hashlib.sha256((STAGE/'audits/route_tracks_runtime_m08/source_manifest.json').read_bytes()).hexdigest())
(ROOT/'small_fixtures.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
print(json.dumps(result, indent=2))
raise SystemExit(0 if result['passed']==result['total'] else 1)
