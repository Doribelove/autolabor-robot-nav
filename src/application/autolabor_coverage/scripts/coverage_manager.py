#!/usr/bin/env python3
"""Plan and execute static-map coverage tasks through the existing move_base chain."""

import copy
import json
import math
import threading
import time
import uuid

import actionlib
from actionlib_msgs.msg import GoalStatus
from autolabor_coverage.coverage_geometry import (
    CoveragePlanner,
    GridMap,
    Point,
    order_swaths,
    sample_path,
)
from autolabor_coverage.msg import CoverageStatus, EnforcedPath
from autolabor_coverage.srv import (
    PlanCoverage,
    PlanCoverageResponse,
    StartCoverage,
    StartCoverageResponse,
)
from geometry_msgs.msg import Point as RosPoint
from geometry_msgs.msg import PolygonStamped, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rospy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray


TERMINAL_STATES = {"COMPLETED", "COMPLETED_PARTIAL", "CANCELED", "FAILED"}


class CoverageManager:
    def __init__(self):
        self.lock = threading.RLock()
        self.grid = None
        self.map_message = None
        self.map_digest = ""
        self.plan = None
        self.plan_id = ""
        self.plan_map_digest = ""
        self.region = PolygonStamped()
        self.operation_width = float(rospy.get_param("~operation_width_m", 0.70))
        self.overlap_ratio = float(rospy.get_param("~overlap_ratio", 0.15))
        self.allow_reverse_transit = self._strict_bool("~allow_reverse_transit", True)
        self.reverse_transit_speed = float(
            rospy.get_param("~reverse_transit_speed_mps", 0.15)
        )
        self.minimum_turning_radius = float(
            rospy.get_param("~minimum_turning_radius_m", 1.20)
        )
        self.path_sample_spacing = float(
            rospy.get_param("~path_sample_spacing_m", 0.10)
        )
        self.obstacle_wait_sec = float(rospy.get_param("~obstacle_wait_sec", 10.0))
        self.segment_retry_count = int(rospy.get_param("~segment_retry_count", 3))
        self.final_retry_count = int(rospy.get_param("~final_retry_count", 1))
        self.localization_fresh_sec = float(
            rospy.get_param("~localization_fresh_sec", 0.75)
        )
        self.watchdog_fresh_sec = float(rospy.get_param("~watchdog_fresh_sec", 1.0))
        self.goal_timeout_base_sec = float(
            rospy.get_param("~goal_timeout_base_sec", 20.0)
        )
        self.goal_timeout_per_meter_sec = float(
            rospy.get_param("~goal_timeout_per_meter_sec", 20.0)
        )
        self.state = "IDLE"
        self.detail = "waiting for a static map"
        self.localized = False
        self.localization_received_wall = 0.0
        self.watchdog_motion_enabled = False
        self.watchdog_received_wall = 0.0
        self.mode_goals_allowed = False
        self.mode_state = ""
        self.mode_received_wall = 0.0
        self.odom = None
        self.odom_received_wall = 0.0
        self.active = False
        self.manual_pause = False
        self.external_pause = False
        self.cancel_requested = False
        self.current_segment = 0
        self.total_segments = 0
        self.blocked_segments = []
        self.traversed_distance = 0.0
        self.last_tracked_point = None
        self.executed_path = Path()
        self.executed_path.header.frame_id = "map"
        self.worker = None
        self.original_teb = None
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.status_pub = rospy.Publisher(
            "/coverage/status", CoverageStatus, queue_size=1, latch=True
        )
        self.active_pub = rospy.Publisher(
            "/coverage/active", Bool, queue_size=1, latch=True
        )
        self.region_pub = rospy.Publisher(
            "/coverage/region", PolygonStamped, queue_size=1, latch=True
        )
        self.path_pub = rospy.Publisher(
            "/coverage/planned_path", Path, queue_size=1, latch=True
        )
        self.executed_path_pub = rospy.Publisher(
            "/coverage/executed_path", Path, queue_size=1, latch=True
        )
        self.marker_pub = rospy.Publisher(
            "/coverage/markers", MarkerArray, queue_size=1, latch=True
        )
        self.enforced_path_pub = rospy.Publisher(
            "/coverage/enforced_path", EnforcedPath, queue_size=1
        )

        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self._map_callback, queue_size=1)
        self.localization_sub = rospy.Subscriber(
            "/fast_lio/localization_status", String, self._localization_callback, queue_size=5
        )
        self.watchdog_sub = rospy.Subscriber(
            "/nvidia_cmd_vel_watchdog/status", String, self._watchdog_callback, queue_size=5
        )
        self.mode_sub = rospy.Subscriber(
            "/fod_navigation_mode/status", String, self._mode_callback, queue_size=5
        )
        self.pause_sub = rospy.Subscriber(
            "/navigation_pause/paused", Bool, self._external_pause_callback, queue_size=5
        )
        self.odom_sub = rospy.Subscriber("/Odometry", Odometry, self._odom_callback, queue_size=10)

        self.plan_service = rospy.Service("/coverage/plan", PlanCoverage, self._plan_service)
        self.start_service = rospy.Service(
            "/coverage/start", StartCoverage, self._start_service
        )
        self.pause_service = rospy.Service(
            "/coverage/set_paused", SetBool, self._pause_service
        )
        self.cancel_service = rospy.Service(
            "/coverage/cancel", Trigger, self._cancel_service
        )
        self.move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.status_timer = rospy.Timer(rospy.Duration(0.5), self._status_timer)
        self.tracking_timer = rospy.Timer(rospy.Duration(0.2), self._tracking_timer)
        self.active_pub.publish(Bool(data=False))
        self._publish_status()

    @staticmethod
    def _strict_bool(name, default):
        value = rospy.get_param(name, default)
        if type(value) is not bool:
            raise ValueError("{} must be a YAML boolean".format(name))
        return value

    def _map_callback(self, message):
        try:
            grid = GridMap(
                message.info.width,
                message.info.height,
                message.info.resolution,
                message.info.origin.position.x,
                message.info.origin.position.y,
                message.data,
            )
            digest = grid.digest()
        except ValueError as error:
            rospy.logerr_throttle(2.0, "coverage rejected map: %s", error)
            return
        changed_while_active = False
        with self.lock:
            previous = self.map_digest
            self.grid = grid
            self.map_message = copy.deepcopy(message)
            self.map_digest = digest
            if previous and previous != digest:
                self.plan = None
                self.plan_id = ""
                self.plan_map_digest = ""
                changed_while_active = self.active
                self.detail = "static map changed; coverage plan invalidated"
                if not self.active:
                    self.state = "IDLE"
            elif self.state == "IDLE":
                self.detail = "static map ready; select a coverage region"
        if changed_while_active:
            self._request_cancel("static map changed during coverage")
        if previous and previous != digest:
            self._clear_visualizations()
        self._publish_status()

    def _localization_callback(self, message):
        with self.lock:
            self.localized = message.data.startswith("state=LOCALIZED;")
            self.localization_received_wall = time.monotonic()

    def _watchdog_callback(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        with self.lock:
            self.watchdog_motion_enabled = payload.get("motion_enabled") is True
            self.watchdog_received_wall = time.monotonic()

    def _mode_callback(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        with self.lock:
            self.mode_state = str(payload.get("state", ""))
            self.mode_goals_allowed = payload.get("move_base_goals_allowed") is True
            self.mode_received_wall = time.monotonic()

    def _external_pause_callback(self, message):
        with self.lock:
            self.external_pause = bool(message.data)
            if self.active and self.external_pause:
                self.detail = "navigation paused by FOD safety arbitration"
        if message.data and self.active:
            self.move_base.cancel_goal()

    def _odom_callback(self, message):
        with self.lock:
            self.odom = copy.deepcopy(message)
            self.odom_received_wall = time.monotonic()

    def _localization_is_fresh(self):
        with self.lock:
            return self.localized and (
                time.monotonic() - self.localization_received_wall
                <= self.localization_fresh_sec
            )

    def _current_point(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", rospy.Time(0), rospy.Duration(0.5)
            )
        except Exception:
            return None
        return Point(transform.transform.translation.x,
                     transform.transform.translation.y)

    @staticmethod
    def _points_from_region(region):
        return [Point(float(point.x), float(point.y)) for point in region.polygon.points]

    @staticmethod
    def _pose(point, yaw, stamp=None):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = stamp if stamp is not None else rospy.Time.now()
        pose.pose.position.x = point.x
        pose.pose.position.y = point.y
        pose.pose.orientation.z = math.sin(0.5 * yaw)
        pose.pose.orientation.w = math.cos(0.5 * yaw)
        return pose

    def _path_for_points(self, points, yaw):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        path.poses = [self._pose(point, yaw, path.header.stamp) for point in points]
        return path

    def _planner(self):
        return CoveragePlanner(
            self.grid,
            footprint_front=float(rospy.get_param("~footprint_front_m", 0.62)),
            footprint_rear=float(rospy.get_param("~footprint_rear_m", 0.62)),
            footprint_half_width=float(
                rospy.get_param("~footprint_half_width_m", 0.45)
            ),
            minimum_swath_length=float(
                rospy.get_param("~minimum_swath_length_m", 1.20)
            ),
            angle_step_degrees=float(
                rospy.get_param("~candidate_angle_step_deg", 15.0)
            ),
        )

    def _plan_service(self, request):
        response = PlanCoverageResponse()
        with self.lock:
            if self.active:
                response.message = "cannot replace a plan while coverage is active"
                return response
            grid_ready = self.grid is not None
        if not grid_ready:
            response.message = "static map is not ready"
            return response
        if request.region.header.frame_id not in ("", "map"):
            response.message = "coverage region must use the map frame"
            return response
        operation_width = float(request.operation_width_m)
        overlap_ratio = float(request.overlap_ratio)
        current = self._current_point() if self._localization_is_fresh() else None
        try:
            points = self._points_from_region(request.region)
            plan = self._planner().plan(
                points, operation_width, overlap_ratio, reachable_seed=current
            )
        except ValueError as error:
            response.message = str(error)
            with self.lock:
                if self.plan is not None:
                    self.state = "READY"
                    self.detail = "new region rejected; previous plan kept: {}".format(
                        response.message
                    )
                else:
                    self.state = "IDLE"
                    self.detail = response.message
            self._publish_status()
            return response
        if current is None:
            current = points[0]
        route = order_swaths(
            plan.swaths, current, plan.spacing, self.minimum_turning_radius
        )
        plan_id = uuid.uuid4().hex
        region = copy.deepcopy(request.region)
        region.header.frame_id = "map"
        region.header.stamp = rospy.Time.now()
        planned_path = self._planned_path(route, current)
        with self.lock:
            self.plan = plan
            self.plan.swaths = route
            self.plan_id = plan_id
            self.plan_map_digest = self.map_digest
            self.region = region
            self.operation_width = operation_width
            self.overlap_ratio = overlap_ratio
            self.allow_reverse_transit = bool(request.allow_reverse_transit)
            self.state = "READY"
            self.detail = "coverage path is ready"
            self.current_segment = 0
            self.total_segments = len(route) * 2
            self.blocked_segments = []
        self.region_pub.publish(region)
        self.path_pub.publish(planned_path)
        self._publish_markers(route, region)
        self._publish_status()
        response.success = True
        response.message = "coverage plan ready"
        response.plan_id = plan_id
        response.planned_path = planned_path
        response.requested_area_m2 = plan.requested_area
        response.reachable_area_m2 = plan.reachable_area
        response.unreachable_area_m2 = plan.unreachable_area
        response.sweep_angle_rad = plan.angle
        response.swath_count = len(route)
        response.segment_count = len(route) * 2
        return response

    def _planned_path(self, route, current):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        cursor = current
        for swath in route:
            transit_yaw = math.atan2(swath.start.y - cursor.y, swath.start.x - cursor.x)
            for point in sample_path(cursor, swath.start, self.path_sample_spacing):
                path.poses.append(self._pose(point, transit_yaw, path.header.stamp))
            sweep_yaw = math.atan2(swath.end.y - swath.start.y,
                                   swath.end.x - swath.start.x)
            for point in sample_path(swath.start, swath.end, self.path_sample_spacing):
                path.poses.append(self._pose(point, sweep_yaw, path.header.stamp))
            cursor = swath.end
        return path

    def _publish_markers(self, route, region):
        array = MarkerArray()
        outline = Marker()
        outline.header.frame_id = "map"
        outline.header.stamp = rospy.Time.now()
        outline.ns = "coverage_region"
        outline.id = 0
        outline.type = Marker.LINE_STRIP
        outline.action = Marker.ADD
        outline.pose.orientation.w = 1.0
        outline.scale.x = 0.08
        outline.color.r = 1.0
        outline.color.g = 0.75
        outline.color.b = 0.1
        outline.color.a = 1.0
        for point in region.polygon.points:
            outline.points.append(RosPoint(x=point.x, y=point.y, z=0.05))
        if outline.points:
            outline.points.append(copy.deepcopy(outline.points[0]))
        array.markers.append(outline)
        lines = Marker()
        lines.header = outline.header
        lines.ns = "coverage_swaths"
        lines.id = 1
        lines.type = Marker.LINE_LIST
        lines.action = Marker.ADD
        lines.pose.orientation.w = 1.0
        lines.scale.x = 0.06
        lines.color.r = 0.1
        lines.color.g = 0.85
        lines.color.b = 1.0
        lines.color.a = 1.0
        for swath in route:
            lines.points.append(RosPoint(x=swath.start.x, y=swath.start.y, z=0.08))
            lines.points.append(RosPoint(x=swath.end.x, y=swath.end.y, z=0.08))
        array.markers.append(lines)
        self.marker_pub.publish(array)

    def _clear_visualizations(self):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        self.path_pub.publish(path)
        self.executed_path_pub.publish(copy.deepcopy(path))
        region = PolygonStamped()
        region.header.frame_id = "map"
        region.header.stamp = path.header.stamp
        self.region_pub.publish(region)
        markers = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        markers.markers.append(marker)
        self.marker_pub.publish(markers)

    def _start_service(self, request):
        with self.lock:
            if self.active:
                return StartCoverageResponse(False, "coverage is already active")
            if self.plan is None or request.plan_id != self.plan_id:
                return StartCoverageResponse(False, "coverage plan id is missing or stale")
            if self.plan_map_digest != self.map_digest:
                return StartCoverageResponse(False, "static map changed after planning")
            if not self._start_prechecks_locked():
                return StartCoverageResponse(False, self.detail)
            current = self._current_point()
            if current is None:
                return StartCoverageResponse(False, "map to base_link transform is unavailable")
            try:
                # Planning is allowed before localization so the operator can
                # preview a region.  Starting is stricter: recompute against
                # the vehicle's current known-free connected component so
                # disconnected rooms and map islands are clipped and reported.
                replanned = self._planner().plan(
                    self._points_from_region(self.region),
                    self.operation_width,
                    self.overlap_ratio,
                    reachable_seed=current,
                )
            except ValueError as error:
                self.detail = "coverage start replan failed: {}".format(error)
                return StartCoverageResponse(False, self.detail)
            self.plan = replanned
            route = order_swaths(
                self.plan.swaths,
                current,
                self.plan.spacing,
                self.minimum_turning_radius,
            )
            self.plan.swaths = route
            self.cancel_requested = False
            self.manual_pause = False
            self.active = True
            self.current_segment = 0
            self.total_segments = len(route) * 2
            self.blocked_segments = []
            self.traversed_distance = 0.0
            self.last_tracked_point = None
            self.executed_path = Path()
            self.executed_path.header.frame_id = "map"
            self.state = "GOING_TO_START"
            self.detail = "coverage task accepted"
            self.worker = threading.Thread(
                target=self._run_task, args=(copy.deepcopy(route), current), daemon=True
            )
            # Advertise mission ownership before the worker can submit its
            # first move_base goal.  The planner also treats a fresh enforced
            # path as authoritative to close cross-topic delivery races.
            self.active_pub.publish(Bool(data=True))
            self.worker.start()
        self.path_pub.publish(self._planned_path(route, current))
        self._publish_markers(route, self.region)
        self._publish_status()
        return StartCoverageResponse(True, "coverage task started")

    def _start_prechecks_locked(self):
        now = time.monotonic()
        if not self.localized or now - self.localization_received_wall > self.localization_fresh_sec:
            self.detail = "known-map localization is not LOCALIZED and fresh"
            return False
        if now - self.watchdog_received_wall > self.watchdog_fresh_sec:
            self.detail = "NVIDIA command watchdog status is stale"
            return False
        if not self.watchdog_motion_enabled:
            self.detail = "main motion gate is disabled"
            return False
        if now - self.mode_received_wall > 1.0 or not self.mode_goals_allowed:
            self.detail = "FOD navigation mode has not allowed move_base goals"
            return False
        if self.mode_state != "GPS_ACTIVE":
            self.detail = "navigation arbitration is not in GPS_ACTIVE"
            return False
        if self.odom is None or now - self.odom_received_wall > 0.5:
            self.detail = "odometry is not fresh"
            return False
        linear = self.odom.twist.twist.linear
        speed = math.sqrt(linear.x * linear.x + linear.y * linear.y)
        if speed > 0.02:
            self.detail = "vehicle must be stopped before starting coverage"
            return False
        if not self.move_base.wait_for_server(rospy.Duration(0.5)):
            self.detail = "move_base action server is unavailable"
            return False
        return True

    def _pause_service(self, request):
        with self.lock:
            if not self.active:
                return SetBoolResponse(False, "no active coverage task")
            if not request.data and not self._localization_is_fresh():
                return SetBoolResponse(False, "localization must be LOCALIZED before resume")
            self.manual_pause = bool(request.data)
            self.detail = "coverage paused by operator" if request.data else "coverage resuming"
            if request.data:
                self.state = "PAUSED"
        if request.data:
            self.move_base.cancel_goal()
        self._publish_status()
        return SetBoolResponse(True, self.detail)

    def _cancel_service(self, _request):
        with self.lock:
            if not self.active:
                return TriggerResponse(False, "no active coverage task")
        self._request_cancel("coverage canceled by operator")
        return TriggerResponse(True, "coverage cancellation requested")

    def _request_cancel(self, reason):
        with self.lock:
            self.cancel_requested = True
            self.detail = reason
        self.move_base.cancel_all_goals()
        self._publish_status()

    def _set_teb(self, backwards):
        try:
            import dynamic_reconfigure.client
            client = dynamic_reconfigure.client.Client(
                "/move_base/TebLocalPlannerROS", timeout=2.0
            )
            if self.original_teb is None:
                configuration = client.get_configuration(timeout=2.0)
                self.original_teb = {
                    "max_vel_x_backwards": configuration.get("max_vel_x_backwards", 0.3),
                    "xy_goal_tolerance": configuration.get("xy_goal_tolerance", 0.5),
                    "yaw_goal_tolerance": configuration.get("yaw_goal_tolerance", 0.3),
                }
            client.update_configuration({
                "max_vel_x_backwards": backwards,
                "xy_goal_tolerance": 0.20,
                "yaw_goal_tolerance": 0.20,
            })
            return True
        except Exception as error:
            rospy.logerr("coverage could not configure TEB: %s", error)
            return False

    def _restore_teb(self):
        if not self.original_teb:
            return
        try:
            import dynamic_reconfigure.client
            client = dynamic_reconfigure.client.Client(
                "/move_base/TebLocalPlannerROS", timeout=2.0
            )
            client.update_configuration(self.original_teb)
        except Exception as error:
            rospy.logerr("coverage could not restore TEB: %s", error)
        self.original_teb = None

    def _segments(self, route, current):
        segments = []
        cursor = current
        for swath in route:
            transit_distance = math.hypot(swath.start.x - cursor.x,
                                          swath.start.y - cursor.y)
            if transit_distance > 0.15:
                yaw = math.atan2(swath.end.y - swath.start.y,
                                 swath.end.x - swath.start.x)
                segments.append({
                    "type": "transit",
                    "start": cursor,
                    "end": swath.start,
                    "yaw": yaw,
                    "length": transit_distance,
                    "path": None,
                })
            yaw = math.atan2(swath.end.y - swath.start.y,
                             swath.end.x - swath.start.x)
            points = sample_path(swath.start, swath.end, self.path_sample_spacing)
            segments.append({
                "type": "sweep",
                "start": swath.start,
                "end": swath.end,
                "yaw": yaw,
                "length": swath.length,
                "path": self._path_for_points(points, yaw),
            })
            cursor = swath.end
        return segments

    def _wait_while_paused(self):
        while not rospy.is_shutdown():
            with self.lock:
                if self.cancel_requested:
                    return False
                paused = self.manual_pause or self.external_pause
                if paused:
                    self.state = "PAUSED"
            if not paused:
                return True
            time.sleep(0.1)
        return False

    def _execute_segment(self, segment, segment_index):
        if not self._wait_while_paused():
            return "canceled"
        if segment["type"] == "sweep":
            # A transit may move several metres while tracking is disabled.
            # Never compare the next swath's first sample with the previous
            # swath endpoint and misclassify that legitimate transit as a TF
            # localization jump.
            with self.lock:
                self.last_tracked_point = None
        backwards = self.reverse_transit_speed if (
            segment["type"] == "transit" and self.allow_reverse_transit
        ) else 0.0
        if not self._set_teb(backwards):
            return "failed"
        enforced = EnforcedPath()
        enforced.header.frame_id = "map"
        enforced.plan_id = self.plan_id
        enforced.segment_index = segment_index
        enforced.active = segment["type"] == "sweep"
        if enforced.active:
            enforced.path = segment["path"]
        goal = MoveBaseGoal()
        goal.target_pose = self._pose(segment["end"], segment["yaw"])
        timeout = self.goal_timeout_base_sec + (
            self.goal_timeout_per_meter_sec * segment["length"]
        )
        started = time.monotonic()
        # Publish the exact sweep before move_base accepts the goal.  Without
        # this ordering its first planner cycle could observe an inactive path
        # and use the Navfn fallback to shortcut a coverage swath.
        enforced.header.stamp = rospy.Time.now()
        if enforced.active:
            enforced.path.header.stamp = enforced.header.stamp
        self.enforced_path_pub.publish(enforced)
        self.move_base.send_goal(goal)
        while not rospy.is_shutdown():
            enforced.header.stamp = rospy.Time.now()
            if enforced.active:
                enforced.path.header.stamp = enforced.header.stamp
            self.enforced_path_pub.publish(enforced)
            if self.move_base.wait_for_result(rospy.Duration(0.2)):
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    return "succeeded"
                with self.lock:
                    if self.cancel_requested:
                        return "canceled"
                    if self.manual_pause or self.external_pause:
                        return "paused"
                return "blocked" if state in (
                    GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.LOST
                ) else "failed"
            with self.lock:
                if self.cancel_requested:
                    self.move_base.cancel_goal()
                    return "canceled"
                paused = self.manual_pause or self.external_pause
            if paused:
                self.move_base.cancel_goal()
                return "paused"
            if not self._localization_is_fresh():
                with self.lock:
                    self.manual_pause = True
                    self.state = "PAUSED"
                    self.detail = "localization lost; manual resume is required"
                self.move_base.cancel_goal()
                return "paused"
            if time.monotonic() - started > timeout:
                self.move_base.cancel_goal()
                return "blocked"
        return "canceled"

    def _run_task(self, route, current):
        terminal_state = "FAILED"
        try:
            segments = self._segments(route, current)
            with self.lock:
                self.total_segments = len(segments)
            blocked = []
            for index, segment in enumerate(segments):
                if rospy.is_shutdown():
                    terminal_state = "CANCELED"
                    break
                with self.lock:
                    self.current_segment = index + 1
                    self.state = "SWEEPING" if segment["type"] == "sweep" else "TRANSITING"
                    self.detail = "executing {} segment {} of {}".format(
                        segment["type"], index + 1, len(segments)
                    )
                result = "failed"
                attempts = 0
                while attempts <= self.segment_retry_count:
                    with self.lock:
                        self.state = (
                            "SWEEPING" if segment["type"] == "sweep"
                            else "TRANSITING"
                        )
                        self.detail = "executing {} segment {} of {}".format(
                            segment["type"], index + 1, len(segments)
                        )
                    result = self._execute_segment(segment, index)
                    if result == "succeeded":
                        break
                    if result == "paused":
                        if not self._wait_while_paused():
                            result = "canceled"
                            break
                        continue
                    if result in ("canceled", "failed"):
                        break
                    attempts += 1
                    if attempts <= self.segment_retry_count:
                        with self.lock:
                            self.state = "WAITING_OBSTACLE"
                            self.detail = "segment blocked; retry {} of {} in {:.0f}s".format(
                                attempts, self.segment_retry_count, self.obstacle_wait_sec
                            )
                        deadline = time.monotonic() + self.obstacle_wait_sec
                        while time.monotonic() < deadline and not rospy.is_shutdown():
                            with self.lock:
                                if self.cancel_requested:
                                    result = "canceled"
                                    break
                            time.sleep(0.1)
                if result == "succeeded":
                    continue
                if result == "canceled":
                    terminal_state = "CANCELED"
                    break
                if result == "failed":
                    terminal_state = "FAILED"
                    break
                blocked.append((index, segment))
                with self.lock:
                    self.blocked_segments.append(index)
            else:
                for index, segment in list(blocked):
                    for _attempt in range(self.final_retry_count):
                        with self.lock:
                            self.current_segment = index + 1
                            self.state = (
                                "SWEEPING" if segment["type"] == "sweep"
                                else "TRANSITING"
                            )
                            self.detail = "final retry for blocked segment {}".format(
                                index + 1
                            )
                        if self._execute_segment(segment, index) == "succeeded":
                            blocked.remove((index, segment))
                            with self.lock:
                                if index in self.blocked_segments:
                                    self.blocked_segments.remove(index)
                            break
                terminal_state = "COMPLETED_PARTIAL" if blocked else "COMPLETED"
        except Exception as error:
            rospy.logerr("coverage task failed: %s", error)
            with self.lock:
                self.detail = "coverage task exception: {}".format(error)
            terminal_state = "FAILED"
        finally:
            self.move_base.cancel_goal()
            off = EnforcedPath()
            off.header.frame_id = "map"
            off.plan_id = self.plan_id
            off.active = False
            self.enforced_path_pub.publish(off)
            self._restore_teb()
            with self.lock:
                self.active = False
                self.manual_pause = False
                self.state = terminal_state
                if terminal_state == "COMPLETED":
                    self.detail = "coverage route completed"
                elif terminal_state == "COMPLETED_PARTIAL":
                    self.detail = "coverage completed with blocked segments"
                elif terminal_state == "CANCELED":
                    self.detail = "coverage task canceled"
                elif not self.detail.startswith("coverage task exception"):
                    self.detail = "coverage task failed"
            self.active_pub.publish(Bool(data=False))
            self._publish_status()

    def _tracking_timer(self, _event):
        with self.lock:
            should_track = self.active and self.state == "SWEEPING" and not (
                self.manual_pause or self.external_pause
            )
        if not should_track or not self._localization_is_fresh():
            return
        point = self._current_point()
        if point is None:
            return
        with self.lock:
            if self.last_tracked_point is not None:
                step = math.hypot(point.x - self.last_tracked_point.x,
                                  point.y - self.last_tracked_point.y)
                if step > 0.5:
                    self.manual_pause = True
                    self.detail = "localization jump detected; manual resume is required"
                    self.move_base.cancel_goal()
                    return
                if step < 0.02:
                    return
                self.traversed_distance += step
            self.last_tracked_point = point
            pose = self._pose(point, 0.0)
            self.executed_path.header.stamp = pose.header.stamp
            self.executed_path.poses.append(pose)
            path = copy.deepcopy(self.executed_path)
        self.executed_path_pub.publish(path)

    def _status_timer(self, _event):
        self._publish_status()

    def _publish_status(self):
        message = CoverageStatus()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "map"
        with self.lock:
            plan = self.plan
            message.state = self.state
            message.plan_id = self.plan_id
            message.map_ready = self.grid is not None
            message.localized = self._localization_is_fresh()
            message.active = self.active
            message.paused = self.manual_pause or self.external_pause
            message.current_segment = self.current_segment
            message.total_segments = self.total_segments
            message.blocked_segments = len(self.blocked_segments)
            if plan is not None:
                message.requested_area_m2 = plan.requested_area
                message.reachable_area_m2 = plan.reachable_area
                message.unreachable_area_m2 = plan.unreachable_area
                message.traversed_area_m2 = min(
                    plan.reachable_area,
                    self.traversed_distance * self.operation_width,
                )
                if plan.reachable_area > 1.0e-6:
                    message.coverage_ratio = (
                        message.traversed_area_m2 / plan.reachable_area
                    )
            message.detail = self.detail
        self.status_pub.publish(message)


def main():
    rospy.init_node("coverage_manager", anonymous=False)
    CoverageManager()
    rospy.spin()


if __name__ == "__main__":
    main()
