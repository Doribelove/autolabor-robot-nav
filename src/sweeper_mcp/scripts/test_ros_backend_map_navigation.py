#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline safety tests for absolute-map AI navigation.

The tests deliberately never initialise rospy.  ``ROSBackend`` receives plain
message-shaped objects, while the final publish boundary is replaced by an
in-memory recorder.  A failing preflight therefore cannot emit a ROS goal even
when these tests are accidentally run while the robot graph is online.
"""

import json
import math
import os
import sys
import threading
import time
from types import SimpleNamespace
from unittest import mock


_THIS = os.path.dirname(os.path.abspath(__file__))
_WORKSPACE_SRC = os.path.abspath(os.path.join(_THIS, "..", ".."))
_PKG_SRC = os.path.join(_WORKSPACE_SRC, "sweeper_mcp", "src")
_COVERAGE_SRC = os.path.join(
    _WORKSPACE_SRC, "application", "autolabor_coverage", "src")
for _path in (_PKG_SRC, _COVERAGE_SRC):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from autolabor_coverage.coverage_geometry import (  # noqa: E402
    occupancy_grid_digest,
)
from sweeper_mcp.ros_backend import ROSBackend  # noqa: E402
from sweeper_mcp.tools import ToolResult  # noqa: E402


class _OfflineRospy:
    """Only the shutdown query used by ``_wait_snapshot`` is implemented."""

    @staticmethod
    def is_shutdown():
        return False

    @staticmethod
    def logerr_throttle(*_args, **_kwargs):
        pass


class _CoverageServiceRospy(_OfflineRospy):
    class Time:
        @staticmethod
        def now():
            return 123.0

    def __init__(self, start_handler, cancel_handler):
        self.start_handler = start_handler
        self.cancel_handler = cancel_handler
        self.waited_services = []

    def wait_for_service(self, name, timeout):
        self.waited_services.append((name, timeout))

    def ServiceProxy(self, name, _service_type):
        if name == "/coverage/start_batch":
            return self.start_handler
        if name == "/coverage/cancel_batch":
            return self.cancel_handler
        raise AssertionError("unexpected service: %s" % name)


class _CoverageRegion:
    def __init__(self):
        self.id = ""
        self.name = ""
        self.region = SimpleNamespace(
            header=SimpleNamespace(stamp=None, frame_id=""),
            polygon=SimpleNamespace(points=[]),
        )


class _StartCoverageBatchRequest:
    def __init__(self):
        self.client_request_id = ""
        self.operation_width_m = 0.0
        self.overlap_ratio = 0.0
        self.allow_reverse_transit = False
        self.max_speed_mps = 0.0
        self.map_digest = ""
        self.regions = []


class _Point32(SimpleNamespace):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        super().__init__(x=x, y=y, z=z)


def _pose(x=0.0, y=0.0, z=0.0, yaw_deg=0.0):
    yaw = math.radians(yaw_deg)
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=z),
        orientation=SimpleNamespace(
            x=0.0, y=0.0,
            z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5)),
    )


def _grid(frame_id="map", width=6, height=5, resolution=1.0,
          origin_x=-2.0, origin_y=-1.0, origin_yaw_deg=0.0, data=None):
    if data is None:
        data = [0] * (width * height)
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id),
        info=SimpleNamespace(
            width=width,
            height=height,
            resolution=resolution,
            origin=_pose(origin_x, origin_y, yaw_deg=origin_yaw_deg),
        ),
        data=list(data),
    )


def _map_digest(grid):
    origin = grid.info.origin
    return occupancy_grid_digest(
        grid.header.frame_id,
        grid.info.width,
        grid.info.height,
        grid.info.resolution,
        (origin.position.x, origin.position.y, origin.position.z),
        (origin.orientation.x, origin.orientation.y,
         origin.orientation.z, origin.orientation.w),
        grid.data,
    )


def _coverage_status(grid, map_ready=True, digest=None):
    return SimpleNamespace(
        state="IDLE",
        active=False,
        batch_active=False,
        map_ready=map_ready,
        map_digest=_map_digest(grid) if digest is None else digest,
        localized=True,
    )


def _backend(grid=None, coverage=None, map_age_s=0.0):
    """Build a ROS-free backend and capture its final publish boundary."""
    grid = grid or _grid()
    coverage = coverage or _coverage_status(grid)
    backend = ROSBackend()
    # Prevent _snapshot/_wait_snapshot from importing rospy or opening topics.
    backend._ready = True
    backend._rospy = _OfflineRospy()
    backend._cache("map")(grid)
    backend._cache("localization")(
        SimpleNamespace(data="state=LOCALIZED;fitness=0.10;age=0.05"))
    backend._cache("coverage")(coverage)
    backend._received["map"] = time.monotonic() - float(map_age_s)

    published = []

    def record_publish(frame_id, x, y, yaw_rad, expected_map_digest=""):
        published.append(
            (frame_id, x, y, yaw_rad, expected_map_digest))
        return ToolResult("offline publish boundary reached")

    backend._publish_owned_goal = record_publish
    return backend, published


def _assert_rejected_without_publish(backend, published, x=0.25, y=1.25):
    result = backend.navigate_map_pose(x, y, 30.0)
    assert result.is_error, result
    assert published == [], "a rejected map target reached the publish boundary"
    return result


def test_latched_static_map_remains_valid_after_two_seconds():
    backend, published = _backend(map_age_s=60.0)

    result = backend.navigate_map_pose(0.25, 1.25, 30.0)

    assert not result.is_error, result
    assert len(published) == 1
    frame_id, x, y, yaw, expected_digest = published[0]
    assert frame_id == "map"
    assert abs(x - 0.25) < 1e-12
    assert abs(y - 1.25) < 1e-12
    assert abs(yaw - math.radians(30.0)) < 1e-12
    assert expected_digest == _map_digest(_grid())


def test_matching_coverage_map_digest_allows_arbitrary_free_target():
    grid = _grid()
    status = _coverage_status(grid, digest=_map_digest(grid))
    backend, published = _backend(grid=grid, coverage=status)

    result = backend.navigate_map_pose(2.75, 2.75, -135.0)

    assert not result.is_error, result
    assert published == [
        ("map", 2.75, 2.75, math.radians(-135.0), _map_digest(grid)),
    ]


def test_rotated_map_origin_uses_inverse_yaw_for_cell_lookup():
    grid = _grid(
        width=4, height=3, origin_x=10.0, origin_y=20.0,
        origin_yaw_deg=90.0)
    backend, published = _backend(
        grid=grid, coverage=_coverage_status(grid))

    # Cell-local centre (column=1,row=0) rotates to map point (9.5,21.5).
    result = backend.navigate_map_pose(9.5, 21.5, 45.0)

    assert not result.is_error, result
    assert published == [
        ("map", 9.5, 21.5, math.radians(45.0), _map_digest(grid)),
    ]


def test_non_planar_map_origin_is_rejected_without_publish():
    grid = _grid()
    grid.info.origin.orientation.x = math.sin(math.radians(10.0) * 0.5)
    grid.info.origin.orientation.w = math.cos(math.radians(10.0) * 0.5)
    backend, published = _backend(
        grid=grid, coverage=_coverage_status(grid))

    result = _assert_rejected_without_publish(backend, published)

    assert "平面旋转" in result.text


def test_mismatching_coverage_map_digest_is_rejected_without_publish():
    grid = _grid()
    status = _coverage_status(grid, digest="0" * 64)
    backend, published = _backend(grid=grid, coverage=status)

    result = _assert_rejected_without_publish(backend, published)

    assert "摘要" in result.text


def test_coverage_map_not_ready_is_rejected_without_publish():
    grid = _grid()
    status = _coverage_status(grid, map_ready=False)
    backend, published = _backend(grid=grid, coverage=status)

    result = _assert_rejected_without_publish(backend, published)

    assert "地图" in result.text
    assert "未加载" in result.text or "未就绪" in result.text


def test_non_map_frame_is_rejected_without_publish():
    grid = _grid(frame_id="odom")
    # Match the digest deliberately so the frame check, rather than an identity
    # mismatch, is what rejects this target.
    status = _coverage_status(grid)
    backend, published = _backend(grid=grid, coverage=status)

    result = _assert_rejected_without_publish(backend, published)

    assert "坐标系" in result.text or "frame" in result.text.lower()


def test_out_of_bounds_target_is_rejected_without_publish():
    backend, published = _backend()

    result = _assert_rejected_without_publish(
        backend, published, x=4.01, y=1.25)

    assert "超出" in result.text


def test_unknown_cell_is_rejected_without_publish():
    grid = _grid()
    # (0.25, 1.25) maps to column 2, row 2 for this map.
    grid.data[2 * grid.info.width + 2] = -1
    backend, published = _backend(
        grid=grid, coverage=_coverage_status(grid))

    result = _assert_rejected_without_publish(backend, published)

    assert "未知" in result.text or "占用" in result.text


def test_occupied_cell_is_rejected_without_publish():
    grid = _grid()
    grid.data[2 * grid.info.width + 2] = 80
    backend, published = _backend(
        grid=grid, coverage=_coverage_status(grid))

    result = _assert_rejected_without_publish(backend, published)

    assert "占用" in result.text


def test_invalid_new_map_callback_invalidates_old_cache_without_publish():
    valid_grid = _grid()
    backend, published = _backend(grid=valid_grid)
    invalid_grid = _grid(data=[0])

    # A malformed replacement must invalidate the previously cached identity.
    # Otherwise a map switch could leave AI navigation validating against stale
    # cells after the coverage manager has already rejected the new map.
    backend._cache("map")(invalid_grid)
    result = _assert_rejected_without_publish(backend, published)

    assert "地图" in result.text
    assert "无效" in result.text or "未收到" in result.text


def test_stale_dynamic_status_is_rejected_without_publish():
    backend, published = _backend()
    backend._received["coverage"] = time.monotonic() - 60.0

    result = _assert_rejected_without_publish(backend, published)

    assert "coverage/status" in result.text


def test_stale_localization_is_rejected_without_publish():
    backend, published = _backend()
    backend._received["localization"] = time.monotonic() - 60.0

    result = _assert_rejected_without_publish(backend, published)

    assert "LOCALIZED" in result.text


def test_map_switch_is_rejected_by_publish_boundary_digest_recheck():
    backend, published = _backend()
    _grid_before, expected_digest = backend._map_snapshot()
    replacement = _grid(origin_x=-3.0)
    backend._cache("map")(replacement)
    backend._cache("coverage")(_coverage_status(replacement))

    reason, _grid_after, _digest_after = backend._map_navigation_context(
        wait=False, expected_digest=expected_digest)

    assert "切换" in reason
    assert published == []


def test_coverage_region_name_and_uuid_cannot_select_same_region_twice():
    grid = _grid()
    coverage = _coverage_status(grid)
    coverage.chassis_ready = True
    coverage.avoidance_ready = True
    backend, _published = _backend(grid=grid, coverage=coverage)
    backend._load_regions = lambda _digest: [{
        "id": "region-a",
        "name": "A区",
        "points": [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)],
    }]

    result = backend.start_coverage_cleaning(["A区", "region-a"])

    assert result.is_error
    assert "重复加入" in result.text


def _coverage_service_modules():
    return {
        "autolabor_coverage.msg": SimpleNamespace(
            CoverageRegion=_CoverageRegion),
        "autolabor_coverage.srv": SimpleNamespace(
            StartCoverageBatch=object,
            StartCoverageBatchRequest=_StartCoverageBatchRequest,
            CancelCoverageBatch=object,
        ),
        "geometry_msgs.msg": SimpleNamespace(Point32=_Point32),
    }


def _coverage_start_backend(rospy_impl):
    grid = _grid()
    coverage = _coverage_status(grid)
    coverage.chassis_ready = True
    coverage.avoidance_ready = True
    backend, _published = _backend(grid=grid, coverage=coverage)
    backend._rospy = rospy_impl
    backend._load_regions = lambda _digest: [{
        "id": "region-a",
        "name": "A区",
        "points": [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)],
    }]
    return backend


def test_coverage_start_uses_client_generated_operation_id():
    requests = []

    def start(request):
        requests.append(request)
        return SimpleNamespace(
            accepted=True,
            message="accepted",
            batch_id=request.client_request_id,
        )

    rospy_impl = _CoverageServiceRospy(
        start, lambda _batch_id: AssertionError("cancel must not be called"))
    backend = _coverage_start_backend(rospy_impl)
    with mock.patch.dict(sys.modules, _coverage_service_modules()):
        result = backend.start_coverage_cleaning(["A区"])

    assert not result.is_error, result.text
    payload = json.loads(result.text)
    assert len(requests) == 1
    assert requests[0].client_request_id == payload["batch_id"]
    assert payload["batch_id"].startswith("coverage-batch-")
    assert len(payload["batch_id"]) == len("coverage-batch-") + 32
    assert backend._ai_batch_id == payload["batch_id"]


def test_lost_coverage_start_response_tombstones_exact_operation_id():
    attempted_ids = []
    canceled_ids = []

    def start(request):
        attempted_ids.append(request.client_request_id)
        raise RuntimeError("response lost after server acceptance")

    def cancel(batch_id):
        canceled_ids.append(batch_id)
        return SimpleNamespace(
            success=True,
            cancellation_requested=False,
            not_started=True,
            message="canceled before start",
            batch_id=batch_id,
        )

    backend = _coverage_start_backend(
        _CoverageServiceRospy(start, cancel))
    with mock.patch.dict(sys.modules, _coverage_service_modules()):
        result = backend.start_coverage_cleaning(["A区"])

    payload = json.loads(result.text)
    assert result.is_error
    assert attempted_ids == canceled_ids
    assert payload["batch_id"] == attempted_ids[0]
    assert payload["cancel_state"] == "confirmed_not_started"
    assert payload["outcome_uncertain"] is False
    assert backend._ai_batch_id == ""


def test_lost_coverage_start_response_retains_id_if_exact_cancel_unavailable():
    attempted_ids = []

    def start(request):
        attempted_ids.append(request.client_request_id)
        raise RuntimeError("response lost after server acceptance")

    def cancel(_batch_id):
        raise RuntimeError("cancel service disconnected")

    backend = _coverage_start_backend(
        _CoverageServiceRospy(start, cancel))
    with mock.patch.dict(sys.modules, _coverage_service_modules()):
        result = backend.start_coverage_cleaning(["A区"])

    payload = json.loads(result.text)
    assert result.is_error
    assert payload["cancel_state"] == "unavailable"
    assert payload["outcome_uncertain"] is True
    assert backend._ai_batch_id == attempted_ids[0]


def test_inflight_coverage_claim_cancel_is_not_mistaken_for_safe_tombstone():
    batch_id = "coverage-batch-" + "7" * 32

    def cancel(requested_id):
        return SimpleNamespace(
            success=True,
            cancellation_requested=True,
            not_started=True,
            message="owner claim is still settling",
            batch_id=requested_id,
        )

    backend = _coverage_start_backend(
        _CoverageServiceRospy(lambda _request: None, cancel))
    backend._ai_batch_id = batch_id
    with mock.patch.dict(sys.modules, _coverage_service_modules()):
        result = backend._cancel_coverage_batch_exact(batch_id)

    assert result["success"] is True
    assert result["not_started"] is True
    assert result["cancellation_requested"] is True
    assert result["cancel_state"] == "requested"
    assert result["safe"] is False
    assert backend._ai_batch_id == batch_id


def test_concurrent_coverage_submit_cannot_erase_accepted_owner_id():
    attempted_ids = []
    canceled_ids = []
    first_after_preflight = threading.Event()
    second_after_preflight = threading.Event()
    first_service_entered = threading.Event()
    second_finished = threading.Event()
    uuid_lock = threading.Lock()
    uuid_calls = [0]

    def deterministic_uuid():
        with uuid_lock:
            index = uuid_calls[0]
            uuid_calls[0] += 1
        if index == 0:
            first_after_preflight.set()
            # On the fixed path the second request is rejected by the submit
            # lock and never reaches UUID allocation, so use a bounded wait.
            second_after_preflight.wait(0.3)
            return SimpleNamespace(hex="a" * 32)
        second_after_preflight.set()
        assert first_service_entered.wait(1.0)
        return SimpleNamespace(hex="b" * 32)

    def start(request):
        attempted_ids.append(request.client_request_id)
        if request.client_request_id.endswith("a" * 32):
            first_service_entered.set()
            assert second_finished.wait(2.0)
            return SimpleNamespace(
                accepted=True,
                message="accepted A",
                batch_id=request.client_request_id,
            )
        return SimpleNamespace(
            accepted=False,
            message="manager rejected concurrent B",
            batch_id=request.client_request_id,
        )

    def cancel(batch_id):
        canceled_ids.append(batch_id)
        return SimpleNamespace(
            success=True,
            cancellation_requested=False,
            not_started=True,
            message="B never started",
            batch_id=batch_id,
        )

    backend = _coverage_start_backend(_CoverageServiceRospy(start, cancel))
    results = {}

    def invoke(name):
        try:
            results[name] = backend.start_coverage_cleaning(["A区"])
        finally:
            if name == "second":
                second_finished.set()

    first = threading.Thread(target=invoke, args=("first",))
    second = threading.Thread(target=invoke, args=("second",))
    with mock.patch.dict(sys.modules, _coverage_service_modules()), mock.patch(
        "sweeper_mcp.ros_backend.uuid.uuid4",
        side_effect=deterministic_uuid,
    ):
        first.start()
        assert first_after_preflight.wait(1.0)
        second.start()
        second.join(2.0)
        first.join(2.0)

    assert not first.is_alive()
    assert not second.is_alive()
    assert not results["first"].is_error, results["first"].text
    assert results["second"].is_error
    assert "正在提交或收敛" in results["second"].text
    first_payload = json.loads(results["first"].text)
    assert first_payload["accepted"] is True
    assert first_payload["batch_id"] == "coverage-batch-" + "a" * 32
    assert backend._ai_batch_id == first_payload["batch_id"]
    assert attempted_ids == [first_payload["batch_id"]]
    assert canceled_ids == []


def test_coverage_cancel_waits_for_submit_identity_to_settle():
    canceled_ids = []

    def cancel(batch_id):
        canceled_ids.append(batch_id)
        raise AssertionError("cancel must not race an in-flight submit")

    backend = _coverage_start_backend(
        _CoverageServiceRospy(lambda _request: None, cancel)
    )
    backend._ai_batch_id = "coverage-batch-" + "c" * 32
    assert backend._coverage_submit_lock.acquire(False)
    try:
        result = backend.cancel_coverage()
    finally:
        backend._coverage_submit_lock.release()

    assert result.is_error
    assert "提交或收敛" in result.text
    assert canceled_ids == []
    assert backend._ai_batch_id == "coverage-batch-" + "c" * 32


if __name__ == "__main__":
    tests = [value for name, value in sorted(globals().items())
             if name.startswith("test_")]
    for test in tests:
        test()
        print("PASS", test.__name__)
    print("%d map-navigation tests passed" % len(tests))
