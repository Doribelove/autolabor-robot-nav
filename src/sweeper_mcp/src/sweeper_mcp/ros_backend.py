# -*- coding: utf-8 -*-
"""ROS implementation of the authorised indoor sweeper MCP tools.

This backend deliberately reuses the same safety owners as the Qt console.
AI navigation uses a dedicated J6M-gated action request so NVIDIA can assign a
non-empty GoalID before submission; coverage goes through the J6M coverage
manager and visual spot cleaning goes through the FOD mode arbiter.  It never
publishes velocity or emergency-release commands.
"""

import configparser
import json
import math
import os
import threading
import time
import uuid

from autolabor_coverage.coverage_geometry import occupancy_grid_digest
from sweeper_mcp.tools import ToolResult


def _json(data):
    return json.dumps(data, ensure_ascii=False, sort_keys=True)


def _yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _public_mode_state(state):
    """Hide the legacy FOD arbiter's GPS-era label from the indoor API."""
    return "RELATIVE_NAV_ACTIVE" if state == "GPS_ACTIVE" else state


class ROSBackend:
    name = "ros"

    STATUS_NAMES = {
        0: "pending", 1: "active", 2: "preempted", 3: "succeeded",
        4: "aborted", 5: "rejected", 6: "preempting", 7: "recalling",
        8: "recalled", 9: "lost",
    }
    CANCEL_SAFE_STATUSES = {2, 3, 4, 5, 8}

    def __init__(self, topics=None):
        self._topics = {
            "simple_goal": "/move_base_simple/goal",
            "action_request": "/navigation_goal/action_request",
            "cancel_request": "/navigation_goal/cancel_request",
            "cancel_ack": "/navigation_goal/cancel_ack",
            "ai_heartbeat": "/navigation_goal/ai_heartbeat",
            "action_goal": "/move_base/goal",
            "cancel": "/move_base/cancel",
            "status": "/move_base/status",
            "odom": "/Odometry",
            "map": "/map",
            "localization": "/fast_lio/localization_status",
            "coverage_status": "/coverage/status",
            "mode_status": "/fod_navigation_mode/status",
            "visual_status": "/fod_visual_servo/status",
            "chassis": "/m2_driver/chassis_info",
        }
        if topics:
            self._topics.update(topics)
        self._coverage_root = os.environ.get(
            "SWEEPER_COVERAGE_REGION_ROOT", "").strip()
        self._coverage_legacy_root = os.environ.get(
            "SWEEPER_COVERAGE_REGION_LEGACY_ROOT", "").strip()
        self._source_mode = os.environ.get(
            "SWEEPER_STATIC_MAP_SOURCE_MODE", "fused")
        self._operator_settings_file = os.environ.get(
            "SWEEPER_OPERATOR_SETTINGS_FILE", "").strip()
        if not self._operator_settings_file:
            config_root = os.environ.get("XDG_CONFIG_HOME", "").strip()
            if not config_root:
                config_root = os.path.join(os.path.expanduser("~"), ".config")
            self._operator_settings_file = os.path.join(
                config_root, "Autolabor", "Autolabor Operator Console.conf"
            )
        self._lock = threading.RLock()
        self._ready = False
        self._rospy = None
        self._latest = {}
        self._received = {}
        self._map_digest = ""
        self._goal_messages = []
        self._cancel_ack_messages = []
        self._ai_goal_id = ""
        self._ai_batch_id = ""
        self._ai_visual_owned = False
        self._goal_submit_lock = threading.Lock()
        self._coverage_submit_lock = threading.Lock()
        self._goal_confirmation_timeout = 3.0
        self._goal_confirmation_poll = 0.03
        self._goal_cancel_confirmation_timeout = 2.0
        self._ai_goal_cancel_uncertain = False
        self._ai_goal_cancel_confirmed = False
        self._ai_goal_cancel_confirmed_state = ""
        self._ai_goal_failure_detail = ""
        self._cancel_cleanup_goal_ids = set()
        self._ai_goal_supervision_id = ""
        self._ai_goal_supervision_wall = 0.0
        self._ai_goal_supervision_timeout = 1.0

    # ---- ROS setup and snapshots -------------------------------------------------

    def _ensure(self):
        if self._ready:
            return
        import rospy
        if not rospy.core.is_initialized():
            # One fixed ROS identity makes duplicate/orphaned MCP control
            # backends visible and lets ROS evict the old registration instead
            # of leaving two anonymous navigation request sources alive.
            rospy.init_node("sweeper_mcp_backend", anonymous=False,
                            disable_signals=True)
        from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
        from autolabor_canbus_driver.msg import ChassisStatusInfo
        from autolabor_coverage.msg import CoverageStatus
        from geometry_msgs.msg import PoseStamped
        from move_base_msgs.msg import MoveBaseActionGoal
        from nav_msgs.msg import OccupancyGrid, Odometry
        from std_msgs.msg import String

        self._rospy = rospy
        self._types = {
            "GoalID": GoalID,
            "GoalStatus": GoalStatus,
            "GoalStatusArray": GoalStatusArray,
            "ChassisStatusInfo": ChassisStatusInfo,
            "CoverageStatus": CoverageStatus,
            "PoseStamped": PoseStamped,
            "MoveBaseActionGoal": MoveBaseActionGoal,
            "OccupancyGrid": OccupancyGrid,
            "Odometry": Odometry,
            "String": String,
        }
        self._action_request_pub = rospy.Publisher(
            self._topics["action_request"], MoveBaseActionGoal, queue_size=2)
        self._heartbeat_pub = rospy.Publisher(
            self._topics["ai_heartbeat"], GoalID, queue_size=1)
        self._cancel_request_pub = rospy.Publisher(
            self._topics["cancel_request"], GoalID, queue_size=5)
        self._status_sub = rospy.Subscriber(
            self._topics["status"], GoalStatusArray,
            self._cache("nav_status"), queue_size=10)
        self._action_goal_sub = rospy.Subscriber(
            self._topics["action_goal"], MoveBaseActionGoal,
            self._action_goal_callback, queue_size=20)
        self._cancel_ack_sub = rospy.Subscriber(
            self._topics["cancel_ack"], GoalStatus,
            self._cancel_ack_callback, queue_size=20)
        self._subscriptions = [
            rospy.Subscriber(self._topics["odom"], Odometry,
                             self._cache("odom"), queue_size=10),
            rospy.Subscriber(self._topics["map"], OccupancyGrid,
                             self._cache("map"), queue_size=1),
            rospy.Subscriber(self._topics["localization"], String,
                             self._cache("localization"), queue_size=5),
            rospy.Subscriber(self._topics["coverage_status"], CoverageStatus,
                             self._cache("coverage"), queue_size=5),
            rospy.Subscriber(self._topics["mode_status"], String,
                             self._cache("mode"), queue_size=5),
            rospy.Subscriber(self._topics["visual_status"], String,
                             self._cache("visual"), queue_size=5),
            rospy.Subscriber(self._topics["chassis"], ChassisStatusInfo,
                             self._cache("chassis"), queue_size=5),
            self._status_sub,
            self._action_goal_sub,
            self._cancel_ack_sub,
        ]
        self._ready = True
        self._heartbeat_timer = rospy.Timer(
            rospy.Duration(0.2), self._heartbeat_timer_callback)

    def _heartbeat_timer_callback(self, _event):
        with self._lock:
            goal_id = self._ai_goal_id
            supervised = bool(
                goal_id and self._ai_goal_supervision_id == goal_id and
                self._ai_goal_supervision_wall > 0.0 and
                time.monotonic() - self._ai_goal_supervision_wall <=
                self._ai_goal_supervision_timeout
            )
        if not supervised:
            # A freshly restarted backend owns no previous process's ID and
            # a child whose Agent thread stopped supervising navigation must
            # both be unable to extend the J6M execution lease.
            return
        try:
            heartbeat = self._types["GoalID"]()
            heartbeat.stamp = self._rospy.Time()
            heartbeat.id = goal_id
            self._heartbeat_pub.publish(heartbeat)
        except Exception:
            # Transport readiness is checked synchronously before a goal.  The
            # J6M lease watchdog independently cancels this exact active ID if
            # its periodic heartbeat stops.
            pass

    def _touch_ai_goal_supervision(self, goal_id=""):
        """Prove the Agent-side navigation loop still supervises this exact ID."""
        with self._lock:
            current = self._ai_goal_id
            supervised_id = str(goal_id or current)
            if not current or supervised_id != current:
                return False
            self._ai_goal_supervision_id = current
            self._ai_goal_supervision_wall = time.monotonic()
            return True

    def _cache(self, name):
        if name == "map":
            return self._cache_map

        def callback(message):
            with self._lock:
                self._latest[name] = message
                self._received[name] = time.monotonic()
        return callback

    def _cache_map(self, message):
        """Cache a validated static map and its canonical spatial identity.

        A map_server OccupancyGrid is normally latched and sent only once.  Its
        receive time therefore cannot be used as a freshness signal.  Compute
        the exact digest used by the coverage manager and atomically cache the
        message/digest pair instead; a later valid map callback replaces both.
        """
        try:
            origin = message.info.origin
            digest = occupancy_grid_digest(
                message.header.frame_id,
                message.info.width,
                message.info.height,
                message.info.resolution,
                (
                    origin.position.x,
                    origin.position.y,
                    origin.position.z,
                ),
                (
                    origin.orientation.x,
                    origin.orientation.y,
                    origin.orientation.z,
                    origin.orientation.w,
                ),
                message.data,
            )
        except (AttributeError, OverflowError, TypeError, ValueError) as exc:
            # A later map callback means the map source may be switching.  Do
            # not retain the previously valid map after rejecting its
            # replacement: that would let absolute navigation validate against
            # stale spatial data while the rest of the graph has moved on.
            with self._lock:
                self._latest.pop("map", None)
                self._received.pop("map", None)
                self._map_digest = ""
            self._rospy.logerr_throttle(
                2.0, "sweeper MCP rejected invalid /map: %s", exc)
            return
        with self._lock:
            self._latest["map"] = message
            self._received["map"] = time.monotonic()
            self._map_digest = digest

    def _action_goal_callback(self, message):
        with self._lock:
            now = time.monotonic()
            self._goal_messages.append((now, message))
            self._goal_messages = self._goal_messages[-50:]

    def _cancel_ack_callback(self, message):
        with self._lock:
            self._cancel_ack_messages.append((time.monotonic(), message))
            self._cancel_ack_messages = self._cancel_ack_messages[-50:]

    def _snapshot(self, name, max_age=None):
        self._ensure()
        with self._lock:
            message = self._latest.get(name)
            received = self._received.get(name)
        if message is None:
            return None
        if max_age is not None and (received is None or
                                    time.monotonic() - received > max_age):
            return None
        return message

    def _wait_snapshot(self, name, max_age, timeout=2.0):
        """Wait briefly for the first fresh callback after lazy ROS setup."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            message = self._snapshot(name, max_age)
            if message is not None:
                return message
            if self._rospy.is_shutdown():
                break
            time.sleep(0.03)
        return self._snapshot(name, max_age)

    def _map_snapshot(self):
        """Return the static map and matching digest without an age limit."""
        self._ensure()
        with self._lock:
            return self._latest.get("map"), self._map_digest

    def _wait_map_snapshot(self, timeout=2.0):
        """Wait for the first valid latched map, never for a periodic refresh."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            grid, digest = self._map_snapshot()
            if grid is not None and digest:
                return grid, digest
            if self._rospy.is_shutdown():
                break
            time.sleep(0.03)
        return self._map_snapshot()

    @staticmethod
    def _parse_json_message(message):
        if message is None:
            return {}
        try:
            value = json.loads(message.data)
            return value if isinstance(value, dict) else {}
        except (AttributeError, TypeError, ValueError):
            return {}

    # ---- preflight and goal ownership -------------------------------------------

    def _coverage_busy(self):
        status = self._snapshot("coverage", 2.0)
        return bool(status and (status.active or status.batch_active or
                                status.state in ("PLANNING", "PREPARING")))

    def _ordinary_navigation_ready(self):
        if self._coverage_root and self._wait_snapshot(
                "coverage", 2.0, 2.5) is None:
            return "静态地图覆盖管理器状态未就绪"
        if self._coverage_busy():
            return "覆盖清扫正在独占 move_base"
        odom = self._wait_snapshot("odom", 0.8, 1.5)
        if odom is None:
            return "/Odometry 未就绪或已超时"
        mode = self._parse_json_message(
            self._wait_snapshot("mode", 2.0, 2.5))
        if not mode:
            return "控制模式状态未就绪"
        if mode.get("state") != "GPS_ACTIVE":
            return "当前不是相对导航模式: %s" % mode.get("state", "UNKNOWN")
        if not mode.get("move_base_goals_allowed", False):
            return "模式仲裁器未放行普通导航目标"
        if self._action_request_pub.get_num_connections() < 1:
            return "%s 无 J6M 安全桥订阅者" % self._topics["action_request"]
        if self._heartbeat_pub.get_num_connections() < 1:
            return "%s 无 J6M 安全桥订阅者" % self._topics["ai_heartbeat"]
        if self._cancel_request_pub.get_num_connections() < 1:
            return "%s 无 J6M 安全桥订阅者" % self._topics["cancel_request"]
        if self._action_goal_sub.get_num_connections() < 1:
            return "%s 回显订阅尚未完成 TCPROS 连接" % self._topics["action_goal"]
        if self._status_sub.get_num_connections() < 1:
            return "%s 状态订阅尚未完成 TCPROS 连接" % self._topics["status"]
        if self._cancel_ack_sub.get_num_connections() < 1:
            return "%s 回执订阅尚未完成 TCPROS 连接" % self._topics["cancel_ack"]
        return ""

    def _map_navigation_context(self, wait=False, expected_digest=""):
        """Validate the dynamic gates and the identity of the cached map.

        Localization and coverage status are dynamic and must remain fresh.
        The OccupancyGrid itself is static: validity comes from the canonical
        digest agreeing with the fresh coverage-manager status, not from the
        age of its one latched callback.
        """
        if wait:
            localization = self._wait_snapshot("localization", 2.0, 2.5)
            coverage = self._wait_snapshot("coverage", 2.0, 2.5)
            grid, digest = self._wait_map_snapshot(2.5)
        else:
            localization = self._snapshot("localization", 2.0)
            coverage = self._snapshot("coverage", 2.0)
            grid, digest = self._map_snapshot()
        if localization is None or not getattr(
                localization, "data", "").startswith("state=LOCALIZED;"):
            return "静态地图三维 ICP 未提供新鲜的 LOCALIZED 状态", None, ""
        if coverage is None:
            return "/coverage/status 未就绪或已超时", None, ""
        if not bool(getattr(coverage, "map_ready", False)):
            return "覆盖管理器尚未加载静态地图", None, ""
        if not bool(getattr(coverage, "localized", False)):
            return "覆盖管理器报告全局定位尚未就绪", None, ""
        coverage_digest = str(getattr(coverage, "map_digest", "")).strip()
        if not coverage_digest:
            return "覆盖管理器未提供当前地图摘要", None, ""
        if grid is None or not digest:
            return "当前 /map 尚未收到有效的锁存地图", None, ""
        if getattr(grid.header, "frame_id", "") != "map":
            return "当前 /map 坐标系不是 map", None, ""
        if expected_digest and digest != expected_digest:
            return "导航预检期间静态地图已经切换", None, ""
        if digest != coverage_digest:
            return "本地 /map 与覆盖管理器的地图摘要不一致", None, ""
        return "", grid, digest

    @staticmethod
    def _stamp_nsec(stamp):
        """Return an integer ROS timestamp without relying on rospy helpers."""
        try:
            return int(stamp.secs) * 1000000000 + int(stamp.nsecs)
        except (AttributeError, TypeError, ValueError):
            return 0

    @classmethod
    def _target_matches(cls, target, frame_id, x, y, yaw_rad, stamp_nsec):
        """Match the immutable parts of an explicit action request.

        ``Header.seq`` is deliberately excluded.  rospy overwrites the seq of
        every top-level header during serialization.  The J6M bridge also
        normalizes timestamps to its local clock; the explicit GoalID, frame
        and complete planar pose remain the end-to-end identity.
        """
        try:
            if (stamp_nsec is not None and
                    cls._stamp_nsec(target.header.stamp) != stamp_nsec):
                return False
            if target.header.frame_id != frame_id:
                return False
            expected_z = math.sin(yaw_rad * 0.5)
            expected_w = math.cos(yaw_rad * 0.5)
            values = (
                (target.pose.position.x, x),
                (target.pose.position.y, y),
                (target.pose.position.z, 0.0),
                (target.pose.orientation.x, 0.0),
                (target.pose.orientation.y, 0.0),
                (target.pose.orientation.z, expected_z),
                (target.pose.orientation.w, expected_w),
            )
            return all(math.isfinite(float(actual)) and
                       abs(float(actual) - float(expected)) <= 1e-6
                       for actual, expected in values)
        except (AttributeError, TypeError, ValueError):
            return False

    @staticmethod
    def _status_items(status, goal_id):
        if status is None:
            return []
        return [
            item for item in getattr(status, "status_list", ())
            if str(getattr(item.goal_id, "id", "")) == goal_id
        ]

    @classmethod
    def _status_item(cls, status, goal_id):
        items = cls._status_items(status, goal_id)
        return items[0] if len(items) == 1 else None

    def _cancel_exact_goal(self, goal_id):
        """Cancel one explicit actionlib goal; never cancel unrelated owners."""
        message = self._types["GoalID"]()
        message.stamp = self._rospy.Time()
        message.id = goal_id
        self._cancel_request_pub.publish(message)

    def _wait_cancel_confirmation(self, goal_id, started, supervise=False):
        """Return a trusted terminal state after an exact cancel.

        RECALLING is deliberately not enough to transfer ownership: the bridge
        keeps retrying the exact ID until move_base reports a true terminal.
        LOST is likewise not accepted as cancellation proof.
        """
        deadline = time.monotonic() + self._goal_cancel_confirmation_timeout
        while time.monotonic() < deadline and not self._rospy.is_shutdown():
            if supervise:
                self._touch_ai_goal_supervision(goal_id)
            with self._lock:
                status = self._latest.get("nav_status")
                received = self._received.get("nav_status")
                cancel_acks = [
                    message for ack_received, message in self._cancel_ack_messages
                    if ack_received + 0.05 >= started and
                    str(getattr(message.goal_id, "id", "")) == goal_id and
                    int(getattr(message, "status", -1)) == 8 and
                    str(getattr(message, "text", "")) == "not_forwarded"
                ]
            if cancel_acks:
                # The J6M bridge emits this acknowledgement only while holding
                # the same lock used to accept an action request and only when
                # the ID has never been forwarded.  A later request with this
                # ID is permanently rejected by the bridge's replay cache.
                return "recalled_before_forward"
            if (status is not None and received is not None and
                    received + 0.05 >= started):
                items = self._status_items(status, goal_id)
                if (len(items) == 1 and
                        int(items[0].status) in self.CANCEL_SAFE_STATUSES):
                    return self.STATUS_NAMES.get(int(items[0].status), "unknown")
                if len(items) > 1:
                    return ""
            time.sleep(self._goal_confirmation_poll)
        return ""

    def _start_cancel_cleanup(self, goal_id):
        """Persistently retry exact cancellation until move_base proves safe."""
        with self._lock:
            if goal_id in self._cancel_cleanup_goal_ids:
                return
            self._cancel_cleanup_goal_ids.add(goal_id)

        def worker():
            try:
                while not self._rospy.is_shutdown():
                    with self._lock:
                        if (self._ai_goal_id != goal_id or
                                not self._ai_goal_cancel_uncertain):
                            return
                    started = time.monotonic()
                    try:
                        self._cancel_exact_goal(goal_id)
                    except Exception:
                        time.sleep(0.5)
                        continue
                    state = self._wait_cancel_confirmation(goal_id, started)
                    if state:
                        with self._lock:
                            if self._ai_goal_id == goal_id:
                                self._ai_goal_cancel_uncertain = False
                                self._ai_goal_cancel_confirmed = True
                                self._ai_goal_cancel_confirmed_state = state
                                self._ai_goal_failure_detail = ""
                        return
                    time.sleep(0.5)
            finally:
                with self._lock:
                    self._cancel_cleanup_goal_ids.discard(goal_id)

        threading.Thread(
            target=worker,
            name="sweeper-ai-cancel-%s" % goal_id[-8:],
            daemon=True,
        ).start()

    def _enter_cancel_uncertain(self, goal_id, reason):
        """Fail closed when move_base can no longer prove this goal stopped."""
        newly_uncertain = False
        with self._lock:
            if self._ai_goal_id != goal_id:
                return
            newly_uncertain = not self._ai_goal_cancel_uncertain
            self._ai_goal_cancel_uncertain = True
            self._ai_goal_cancel_confirmed = False
            self._ai_goal_cancel_confirmed_state = ""
            if reason and (newly_uncertain or not self._ai_goal_failure_detail):
                self._ai_goal_failure_detail = reason
        if newly_uncertain:
            try:
                self._cancel_exact_goal(goal_id)
            except Exception as exc:
                with self._lock:
                    if self._ai_goal_id == goal_id:
                        self._ai_goal_failure_detail = "%s；精确取消发布失败: %s" % (
                            reason, exc)
        self._start_cancel_cleanup(goal_id)

    def _failed_published_goal(self, goal_id, reason):
        cancel_error = ""
        cancel_started = time.monotonic()
        try:
            self._cancel_exact_goal(goal_id)
        except Exception as exc:
            cancel_error = "；精确取消发布失败: %s" % exc
        confirmed_state = "" if cancel_error else self._wait_cancel_confirmation(
            goal_id, cancel_started, supervise=True)
        with self._lock:
            if self._ai_goal_id == goal_id and confirmed_state:
                self._ai_goal_cancel_uncertain = False
                self._ai_goal_cancel_confirmed = True
                self._ai_goal_cancel_confirmed_state = confirmed_state
                self._ai_goal_failure_detail = ""
            elif self._ai_goal_id == goal_id:
                self._ai_goal_cancel_uncertain = True
                self._ai_goal_cancel_confirmed = False
                self._ai_goal_cancel_confirmed_state = ""
                self._ai_goal_failure_detail = reason
        if confirmed_state:
            detail = "；AI goal %s 的取消已由 move_base 确认为 %s" % (
                goal_id, confirmed_state)
        else:
            detail = (
                "；已对 AI goal %s 发布精确取消，但尚未收到同 ID 安全状态确认；"
                "已保留 GoalID 并锁住后续 AI 导航" % goal_id
            )
        if not confirmed_state:
            self._start_cancel_cleanup(goal_id)
        return ToolResult(_json({
            "accepted": False,
            "error": reason,
            "goal_id": goal_id,
            "cancel_state": "confirmed" if confirmed_state else "uncertain",
            "confirmed_state": confirmed_state,
            "detail": "%s%s" % (detail.lstrip("；"), cancel_error),
        }), True)

    def _existing_owned_goal_active(self, status):
        with self._lock:
            goal_id = self._ai_goal_id
            cancel_uncertain = self._ai_goal_cancel_uncertain
            cancel_confirmed = self._ai_goal_cancel_confirmed
        if not goal_id:
            return ""
        items = self._status_items(status, goal_id)
        item = items[0] if len(items) == 1 else None
        if cancel_uncertain:
            if (item is not None and
                    int(item.status) in self.CANCEL_SAFE_STATUSES):
                with self._lock:
                    if self._ai_goal_id == goal_id:
                        self._ai_goal_cancel_uncertain = False
                        self._ai_goal_cancel_confirmed = True
                        self._ai_goal_cancel_confirmed_state = self.STATUS_NAMES.get(
                            int(item.status), "unknown")
                        self._ai_goal_failure_detail = ""
                return ""
            return goal_id
        if cancel_confirmed:
            return ""
        if len(items) > 1:
            self._enter_cancel_uncertain(
                goal_id, "move_base 状态流包含重复 GoalID，无法证明目标已停止")
            return goal_id
        if item is not None and int(item.status) == 9:
            self._enter_cancel_uncertain(
                goal_id, "move_base 将该 GoalID 标记为 LOST，目标停止状态未知")
            return goal_id
        if item is not None and int(item.status) in (0, 1, 6, 7):
            return goal_id
        return ""

    def _publish_owned_goal(self, frame_id, x, y, yaw_rad,
                            expected_map_digest=""):
        if not self._goal_submit_lock.acquire(False):
            return ToolResult("另一个 AI 导航目标正在提交，当前目标未发送。", True)
        try:
            self._ensure()
            self._touch_ai_goal_supervision()
            if not str(frame_id).strip() or not all(math.isfinite(float(value))
                                                   for value in (x, y, yaw_rad)):
                return ToolResult("导航目标坐标系或位姿无效，目标未发送。", True)
            reason = self._ordinary_navigation_ready()
            if reason:
                return ToolResult(reason, True)
            if expected_map_digest:
                reason, _grid, _digest = self._map_navigation_context(
                    wait=False, expected_digest=expected_map_digest)
                if reason:
                    return ToolResult("地图目标提交前复核失败: %s" % reason, True)
            baseline_status = self._wait_snapshot("nav_status", 2.0, 1.5)
            if baseline_status is None:
                return ToolResult("move_base GoalID 状态流未就绪，目标未发送。", True)
            active_goal_id = self._existing_owned_goal_active(baseline_status)
            if active_goal_id:
                with self._lock:
                    cancel_uncertain = self._ai_goal_cancel_uncertain
                return ToolResult(
                    (
                        "AI 导航目标 %s 的精确取消尚未确认，当前目标未发送。"
                        if cancel_uncertain else
                        "AI 导航目标 %s 仍在执行，当前目标未发送。"
                    ) % active_goal_id,
                    True,
                )

            action = self._types["MoveBaseActionGoal"]()
            stamp = self._rospy.Time.now()
            request_stamp = self._stamp_nsec(stamp)
            if not request_stamp:
                return ToolResult("ROS 时间尚未就绪，目标未发送。", True)
            goal_id = "sweeper-ai-%s" % uuid.uuid4().hex
            action.header.stamp = stamp
            action.goal_id.stamp = stamp
            action.goal_id.id = goal_id
            target = action.goal.target_pose
            target.header.stamp = stamp
            target.header.frame_id = frame_id
            target.pose.position.x = float(x)
            target.pose.position.y = float(y)
            target.pose.orientation.z = math.sin(yaw_rad * 0.5)
            target.pose.orientation.w = math.cos(yaw_rad * 0.5)
            with self._lock:
                self._ai_goal_id = goal_id
                self._ai_goal_cancel_uncertain = False
                self._ai_goal_cancel_confirmed = False
                self._ai_goal_cancel_confirmed_state = ""
                self._ai_goal_failure_detail = ""
                self._ai_goal_supervision_id = goal_id
                self._ai_goal_supervision_wall = time.monotonic()
            started = time.monotonic()
            try:
                self._action_request_pub.publish(action)
            except Exception as exc:
                return self._failed_published_goal(
                    goal_id, "AI 导航目标提交失败: %s" % exc)

            deadline = time.monotonic() + self._goal_confirmation_timeout
            matching_echo = None
            matching_status = None
            while time.monotonic() < deadline and not self._rospy.is_shutdown():
                self._touch_ai_goal_supervision(goal_id)
                with self._lock:
                    action_candidates = [
                        message for received, message in self._goal_messages
                        if received + 0.05 >= started and
                        str(getattr(message.goal_id, "id", "")) == goal_id
                    ]
                    status = self._latest.get("nav_status")
                    status_received = self._received.get("nav_status")
                if len(action_candidates) > 1:
                    return self._failed_published_goal(
                        goal_id,
                        "收到重复的显式 move_base action 回显，拒绝确认目标所有权",
                    )
                if action_candidates:
                    candidate = action_candidates[0]
                    if not self._target_matches(
                            candidate.goal.target_pose, frame_id, x, y,
                            yaw_rad, None):
                        return self._failed_published_goal(
                            goal_id,
                            "显式 move_base action 回显的目标位姿与请求不一致",
                        )
                    matching_echo = candidate
                if (status is not None and status_received is not None and
                        status_received + 0.05 >= started):
                    status_candidates = self._status_items(status, goal_id)
                    if len(status_candidates) > 1:
                        return self._failed_published_goal(
                            goal_id,
                            "move_base 状态流包含重复的显式 GoalID，拒绝歧义状态",
                        )
                    matching_status = (
                        status_candidates[0] if status_candidates else None)
                if matching_echo is not None and matching_status is not None:
                    state = self.STATUS_NAMES.get(
                        int(matching_status.status), "unknown")
                    return ToolResult(_json({
                        "accepted": True,
                        "state": state,
                        "goal_id": goal_id,
                        "frame_id": frame_id,
                        "target": {
                            "x": x, "y": y,
                            "yaw_deg": math.degrees(yaw_rad),
                        },
                    }))
                time.sleep(self._goal_confirmation_poll)
            missing = []
            if matching_echo is None:
                missing.append("同 ID action 回显")
            if matching_status is None:
                missing.append("同 ID 状态")
            return self._failed_published_goal(
                goal_id,
                "目标未通过 move_base 显式 GoalID 闭环确认，缺少%s" %
                "、".join(missing),
            )
        finally:
            self._goal_submit_lock.release()

    # ---- read-only tools ---------------------------------------------------------

    def get_robot_status(self):
        self._ensure()
        odom = self._snapshot("odom", 2.0)
        chassis = self._snapshot("chassis", 2.0)
        localization = self._snapshot("localization", 2.0)
        coverage = self._snapshot("coverage", 2.0)
        mode = self._parse_json_message(self._snapshot("mode", 2.0))
        if mode:
            mode = dict(mode)
            mode["state"] = _public_mode_state(mode.get("state", ""))
        result = {
            "position": None,
            "battery_percent": None,
            "battery_voltage": None,
            "emergency": None,
            "localization": localization.data if localization else "UNAVAILABLE",
            "mode": mode,
            "coverage_active": bool(coverage and
                                    (coverage.active or coverage.batch_active)),
            "navigation": json.loads(self.get_navigation_status().text),
        }
        if odom is not None:
            result["position"] = {
                "frame_id": odom.header.frame_id,
                "x": round(odom.pose.pose.position.x, 3),
                "y": round(odom.pose.pose.position.y, 3),
                "yaw_deg": round(math.degrees(_yaw(odom.pose.pose.orientation)), 2),
            }
        if chassis is not None:
            result["battery_percent"] = round(chassis.battery_percent, 1)
            result["battery_voltage"] = round(chassis.battery_voltage, 2)
            result["emergency"] = {
                "hard": bool(chassis.hard_emergency),
                "soft": bool(chassis.soft_emergency),
                "gamepad": bool(chassis.gamepad_emergency),
                "robot": bool(chassis.robot_emergency),
            }
        return ToolResult(_json(result))

    def get_navigation_status(self):
        self._ensure()
        with self._lock:
            goal_id = self._ai_goal_id
            cancel_uncertain = self._ai_goal_cancel_uncertain
            cancel_confirmed = self._ai_goal_cancel_confirmed
            cancel_confirmed_state = self._ai_goal_cancel_confirmed_state
            failure_detail = self._ai_goal_failure_detail
        if goal_id:
            self._touch_ai_goal_supervision(goal_id)
        if not goal_id:
            return ToolResult(_json({"state": "idle", "goal_id": ""}))
        status = self._snapshot("nav_status", 2.0)
        if status is None:
            if cancel_confirmed and cancel_confirmed_state:
                return ToolResult(_json({
                    "state": cancel_confirmed_state,
                    "goal_id": goal_id,
                    "detail": "同 ID 精确取消已确认",
                }))
            return ToolResult(_json({
                "state": "cancel_uncertain" if cancel_uncertain else "unavailable",
                "goal_id": goal_id,
                "detail": (
                    "精确取消尚未由 move_base 确认；GoalID 已保留。%s" %
                    ((" 原始失败: " + failure_detail) if failure_detail else "")
                    if cancel_uncertain else "move_base 状态未就绪"
                ),
            }))
        matches = self._status_items(status, goal_id)
        if len(matches) > 1:
            self._enter_cancel_uncertain(
                goal_id, "move_base 状态流包含重复 GoalID，无法证明目标已停止")
            return ToolResult(_json({
                "state": "cancel_uncertain",
                "goal_id": goal_id,
                "detail": "move_base 状态流出现重复 GoalID，无法安全判定结果",
            }))
        for item in matches:
            state = self.STATUS_NAMES.get(item.status, "unknown")
            if int(item.status) == 9:
                self._enter_cancel_uncertain(
                    goal_id, "move_base 将该 GoalID 标记为 LOST，目标停止状态未知")
                return ToolResult(_json({
                    "state": "cancel_uncertain", "goal_id": goal_id,
                    "move_base_state": "lost",
                    "detail": (
                        "move_base 报告 LOST，不能据此证明车辆已经停止；"
                        "GoalID 已保留并持续精确取消"
                    ),
                }))
            if (cancel_uncertain and
                    int(item.status) not in self.CANCEL_SAFE_STATUSES):
                return ToolResult(_json({
                    "state": "cancel_uncertain", "goal_id": goal_id,
                    "move_base_state": state,
                    "detail": (
                        "精确取消尚未进入安全状态；GoalID 已保留。%s" %
                        (item.text or failure_detail or "")
                    ),
                }))
            if cancel_uncertain:
                with self._lock:
                    if self._ai_goal_id == goal_id:
                        self._ai_goal_cancel_uncertain = False
                        self._ai_goal_cancel_confirmed = True
                        self._ai_goal_cancel_confirmed_state = state
                        self._ai_goal_failure_detail = ""
            return ToolResult(_json({
                "state": state, "goal_id": goal_id,
                "detail": item.text or "",
            }))
        if cancel_uncertain:
            return ToolResult(_json({
                "state": "cancel_uncertain", "goal_id": goal_id,
                "detail": "状态流尚未出现该 GoalID；精确取消句柄仍被保留",
            }))
        if cancel_confirmed and cancel_confirmed_state:
            return ToolResult(_json({
                "state": cancel_confirmed_state,
                "goal_id": goal_id,
                "detail": "同 ID 精确取消已确认；move_base 已不再保留该状态项",
            }))
        return ToolResult(_json({
            "state": "pending", "goal_id": goal_id,
            "detail": "等待 move_base 发布该 goal ID 的状态",
        }))

    def get_coverage_status(self):
        status = self._snapshot("coverage", 2.0)
        if status is None:
            return ToolResult(_json({"state": "unavailable"}))
        with self._lock:
            owned = bool(self._ai_batch_id and
                         self._ai_batch_id == status.batch_id)
        result = {
            "state": status.state,
            "active": bool(status.active),
            "paused": bool(status.paused),
            "batch_active": bool(status.batch_active),
            "batch_id": status.batch_id,
            "ai_owned": owned,
            "current_region": status.current_region_name,
            "current_index": int(status.batch_current_index),
            "total_regions": int(status.batch_total_regions),
            "map_digest": status.map_digest,
            "localized": bool(status.localized),
            "chassis_ready": bool(status.chassis_ready),
            "avoidance_ready": bool(status.avoidance_ready),
            "detail": status.detail,
        }
        if (owned and status.state in (
                "COMPLETED", "COMPLETED_PARTIAL", "CANCELED", "FAILED") and
                not status.active and not status.batch_active):
            with self._lock:
                if self._ai_batch_id == status.batch_id:
                    self._ai_batch_id = ""
        return ToolResult(_json(result))

    def get_visual_servo_status(self):
        mode = self._parse_json_message(self._snapshot("mode", 2.0))
        visual = self._parse_json_message(self._snapshot("visual", 2.0))
        with self._lock:
            owned = self._ai_visual_owned
        return ToolResult(_json({
            "state": _public_mode_state(mode.get("state", "UNAVAILABLE")),
            "reason": mode.get("reason", ""),
            "visual_state": mode.get("visual_state", visual.get("state", "")),
            "visual_active": visual.get("active", False),
            "ai_owned": owned,
        }))

    # ---- navigation tools --------------------------------------------------------

    def navigate_relative(self, forward_m, left_m, delta_yaw_deg):
        self._ensure()
        if math.hypot(forward_m, left_m) < 0.01 and abs(delta_yaw_deg) < 0.1:
            return ToolResult("相对位移与转角均接近零，目标未发送。", True)
        odom = self._snapshot("odom", 0.8)
        if odom is None or not odom.header.frame_id:
            return ToolResult("无法读取新鲜且带坐标系的 /Odometry。", True)
        current_yaw = _yaw(odom.pose.pose.orientation)
        target_x = (odom.pose.pose.position.x +
                    math.cos(current_yaw) * forward_m -
                    math.sin(current_yaw) * left_m)
        target_y = (odom.pose.pose.position.y +
                    math.sin(current_yaw) * forward_m +
                    math.cos(current_yaw) * left_m)
        target_yaw = current_yaw + math.radians(delta_yaw_deg)
        return self._publish_owned_goal(
            odom.header.frame_id, target_x, target_y, target_yaw)

    def navigate_map_pose(self, x_m, y_m, yaw_deg):
        try:
            x_m = float(x_m)
            y_m = float(y_m)
            yaw_deg = float(yaw_deg)
        except (TypeError, ValueError):
            return ToolResult("地图目标坐标和朝向必须是数值。", True)
        if not all(math.isfinite(value) for value in (x_m, y_m, yaw_deg)):
            return ToolResult("地图目标坐标和朝向必须是有限值。", True)
        reason, grid, digest = self._map_navigation_context(wait=True)
        if reason:
            return ToolResult(reason + "。", True)
        resolution = float(grid.info.resolution)
        if not math.isfinite(resolution) or resolution <= 0.0:
            return ToolResult("当前地图分辨率无效。", True)
        width = int(grid.info.width)
        height = int(grid.info.height)
        if width <= 0 or height <= 0 or len(grid.data) != width * height:
            return ToolResult("当前地图尺寸或栅格数据长度无效。", True)
        origin_pose = grid.info.origin
        origin = origin_pose.position
        orientation = origin_pose.orientation
        origin_values = (
            float(origin.x), float(origin.y), float(origin.z),
            float(orientation.x), float(orientation.y),
            float(orientation.z), float(orientation.w),
        )
        if not all(math.isfinite(value) for value in origin_values):
            return ToolResult("当前地图原点位姿包含非有限值。", True)
        quaternion_norm = math.sqrt(sum(
            value * value for value in origin_values[3:]))
        if abs(quaternion_norm - 1.0) > 1e-3:
            return ToolResult("当前地图原点四元数未归一化。", True)
        if abs(orientation.x) > 1e-5 or abs(orientation.y) > 1e-5:
            return ToolResult("当前地图原点不是平面旋转。", True)

        # OccupancyGrid cells are expressed in the map origin's local axes.
        # Convert the requested world/map coordinate through the inverse origin
        # yaw before calculating its row and column.  Most map_server YAML files
        # use yaw=0, but silently assuming that would validate the wrong cell on
        # a rotated map.
        origin_yaw = _yaw(orientation)
        delta_x = x_m - origin.x
        delta_y = y_m - origin.y
        local_x = (math.cos(origin_yaw) * delta_x +
                   math.sin(origin_yaw) * delta_y)
        local_y = (-math.sin(origin_yaw) * delta_x +
                   math.cos(origin_yaw) * delta_y)
        column = int(math.floor(local_x / resolution))
        row = int(math.floor(local_y / resolution))
        if column < 0 or row < 0 or column >= width or row >= height:
            return ToolResult("地图目标超出当前占据栅格范围。", True)
        occupancy = grid.data[row * width + column]
        if occupancy < 0 or occupancy >= 50:
            return ToolResult("地图目标位于未知或占用栅格，已拒绝。", True)
        return self._publish_owned_goal(
            "map", x_m, y_m, math.radians(yaw_deg),
            expected_map_digest=digest)

    def cancel_navigation(self):
        self._ensure()
        with self._lock:
            goal_id = self._ai_goal_id
        if not goal_id:
            return ToolResult("当前没有 AI 所有的普通导航目标。", True)
        self._touch_ai_goal_supervision(goal_id)
        cancel_started = time.monotonic()
        try:
            self._cancel_exact_goal(goal_id)
        except Exception as exc:
            with self._lock:
                self._ai_goal_cancel_uncertain = True
                self._ai_goal_cancel_confirmed = False
                self._ai_goal_cancel_confirmed_state = ""
                self._ai_goal_failure_detail = "精确取消发布失败: %s" % exc
            self._start_cancel_cleanup(goal_id)
            return ToolResult(
                "精确取消发布失败，GoalID %s 已保留: %s" % (goal_id, exc),
                True,
            )
        confirmed_state = self._wait_cancel_confirmation(
            goal_id, cancel_started, supervise=True)
        with self._lock:
            if confirmed_state and self._ai_goal_id == goal_id:
                self._ai_goal_cancel_uncertain = False
                self._ai_goal_cancel_confirmed = True
                self._ai_goal_cancel_confirmed_state = confirmed_state
                self._ai_goal_failure_detail = ""
            elif self._ai_goal_id == goal_id:
                self._ai_goal_cancel_uncertain = True
                self._ai_goal_cancel_confirmed = False
                self._ai_goal_cancel_confirmed_state = ""
                self._ai_goal_failure_detail = "用户请求取消，但 move_base 尚未确认"
        if not confirmed_state:
            self._start_cancel_cleanup(goal_id)
            return ToolResult(
                "已发布精确取消，但 move_base 尚未确认；GoalID %s 已保留，"
                "后续 AI 导航保持锁定。" % goal_id,
                True,
            )
        return ToolResult(_json({
            "canceled": True,
            "goal_id": goal_id,
            "confirmed_state": confirmed_state,
        }))

    # ---- coverage tools ----------------------------------------------------------

    def _load_regions(self, digest):
        if (not self._coverage_root or not os.path.isabs(self._coverage_root) or
                os.path.normpath(self._coverage_root) == os.path.sep):
            raise ValueError("当前 map-set 区域库根目录未配置")
        canonical_root = os.path.realpath(self._coverage_root)
        if not os.path.isdir(canonical_root):
            raise ValueError("当前 map-set 目录不存在")
        path = os.path.join(
            canonical_root, "coverage_regions", self._source_mode,
            "regions.json")
        # Read the old digest-keyed location only as a compatibility fallback.
        # Qt owns the one-time migration into the map-set-local store.
        if not os.path.isfile(path) and self._coverage_legacy_root:
            legacy = os.path.join(
                self._coverage_legacy_root, "v1", digest,
                self._source_mode, "regions.json")
            if os.path.isfile(legacy):
                path = legacy
        canonical_path = os.path.realpath(path)
        allowed_roots = [canonical_root]
        if self._coverage_legacy_root:
            allowed_roots.append(os.path.realpath(self._coverage_legacy_root))
        if not any(canonical_path.startswith(root + os.path.sep)
                   for root in allowed_roots):
            raise ValueError("区域库路径越出了配置根目录")
        if os.path.islink(path):
            raise ValueError("区域库文件不能是符号链接")
        try:
            with open(path, "r", encoding="utf-8") as stream:
                document = json.load(stream)
        except (IOError, OSError, ValueError) as exc:
            raise ValueError("区域库读取失败: %s" % exc)
        if document.get("schema_version") != 1:
            raise ValueError("区域库 schema_version 不是 1")
        if document.get("map_digest") != digest:
            raise ValueError("区域库地图摘要不匹配")
        if document.get("source_mode") != self._source_mode:
            raise ValueError("区域库地图模式不匹配")
        stored_source = document.get("map_source", "")
        if (not stored_source or not os.path.isdir(stored_source) or
                os.path.realpath(stored_source) != canonical_root):
            raise ValueError("区域库不属于当前 map-set 目录")
        result = []
        for item in document.get("regions", []):
            if item.get("map_digest") != digest or item.get(
                    "source_mode") != self._source_mode:
                raise ValueError("区域记录地图身份不匹配")
            polygon = item.get("polygon")
            if not isinstance(polygon, list) or len(polygon) < 3:
                raise ValueError("区域 %s 的多边形无效" % item.get("name", ""))
            points = []
            for point in polygon:
                x, y = point.get("x"), point.get("y")
                if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
                    raise ValueError("区域坐标不是数值")
                if not math.isfinite(float(x)) or not math.isfinite(float(y)):
                    raise ValueError("区域坐标不是有限值")
                points.append((float(x), float(y)))
            area2 = sum(
                points[i][0] * points[(i + 1) % len(points)][1] -
                points[(i + 1) % len(points)][0] * points[i][1]
                for i in range(len(points)))
            if abs(area2) < 1e-4:
                raise ValueError("区域 %s 面积为零" % item.get("name", ""))
            result.append({
                "id": str(item.get("id", "")),
                "name": str(item.get("name", "")),
                "points": points,
            })
        return result

    def list_saved_coverage_regions(self):
        status = self._wait_snapshot("coverage", 2.0, 2.5)
        if status is None or not status.map_digest:
            return ToolResult(_json({"map_digest": "", "regions": []}))
        try:
            regions = self._load_regions(status.map_digest)
        except ValueError as exc:
            return ToolResult(str(exc), True)
        return ToolResult(_json({
            "map_digest": status.map_digest,
            "regions": [{"id": item["id"], "name": item["name"]}
                        for item in regions],
        }))

    def start_coverage_cleaning(self, regions, operation_width_m=None,
                                overlap_percent=None, max_speed_mps=None,
                                allow_reverse_transit=None,
                                reverse_speed_mps=None,
                                max_angular_speed_rps=None,
                                linear_accel_mps2=None,
                                angular_accel_rps2=None,
                                direction_change_penalty_sec=None,
                                segment_handoff_penalty_sec=None,
                                transit_replan_period_sec=None):
        if not self._coverage_submit_lock.acquire(False):
            return ToolResult(
                "另一个 AI 覆盖批次正在提交或收敛，当前请求未发送。",
                True,
            )
        try:
            return self._start_coverage_cleaning_serialized(
                regions,
                operation_width_m=operation_width_m,
                overlap_percent=overlap_percent,
                max_speed_mps=max_speed_mps,
                allow_reverse_transit=allow_reverse_transit,
                reverse_speed_mps=reverse_speed_mps,
                max_angular_speed_rps=max_angular_speed_rps,
                linear_accel_mps2=linear_accel_mps2,
                angular_accel_rps2=angular_accel_rps2,
                direction_change_penalty_sec=direction_change_penalty_sec,
                segment_handoff_penalty_sec=segment_handoff_penalty_sec,
                transit_replan_period_sec=transit_replan_period_sec,
            )
        finally:
            self._coverage_submit_lock.release()

    def _start_coverage_cleaning_serialized(
            self, regions, operation_width_m=None,
            overlap_percent=None, max_speed_mps=None,
            allow_reverse_transit=None, reverse_speed_mps=None,
            max_angular_speed_rps=None, linear_accel_mps2=None,
            angular_accel_rps2=None, direction_change_penalty_sec=None,
            segment_handoff_penalty_sec=None,
            transit_replan_period_sec=None):
        self._ensure()
        defaults = self._coverage_operator_defaults()
        operation_width_m = (
            defaults["operation_width_m"]
            if operation_width_m is None else operation_width_m
        )
        overlap_percent = (
            defaults["overlap_percent"]
            if overlap_percent is None else overlap_percent
        )
        max_speed_mps = (
            defaults["max_speed_mps"]
            if max_speed_mps is None else max_speed_mps
        )
        allow_reverse_transit = (
            defaults["allow_reverse_transit"]
            if allow_reverse_transit is None else allow_reverse_transit
        )
        reverse_speed_mps = (
            defaults["reverse_speed_mps"]
            if reverse_speed_mps is None else reverse_speed_mps
        )
        max_angular_speed_rps = (
            defaults["max_angular_speed_rps"]
            if max_angular_speed_rps is None else max_angular_speed_rps
        )
        linear_accel_mps2 = (
            defaults["linear_accel_mps2"]
            if linear_accel_mps2 is None else linear_accel_mps2
        )
        angular_accel_rps2 = (
            defaults["angular_accel_rps2"]
            if angular_accel_rps2 is None else angular_accel_rps2
        )
        direction_change_penalty_sec = (
            defaults["direction_change_penalty_sec"]
            if direction_change_penalty_sec is None
            else direction_change_penalty_sec
        )
        segment_handoff_penalty_sec = (
            defaults["segment_handoff_penalty_sec"]
            if segment_handoff_penalty_sec is None
            else segment_handoff_penalty_sec
        )
        transit_replan_period_sec = (
            defaults["transit_replan_period_sec"]
            if transit_replan_period_sec is None
            else transit_replan_period_sec
        )
        status = self._wait_snapshot("coverage", 2.0, 2.5)
        if status is None:
            return ToolResult("/coverage/status 未就绪。", True)
        if status.active or status.batch_active or status.state in (
                "PLANNING", "PREPARING"):
            return ToolResult("覆盖管理器已有任务，拒绝抢占。", True)
        if not status.localized or not status.chassis_ready or not status.avoidance_ready:
            return ToolResult(
                "覆盖安全门未就绪: localized=%s chassis=%s avoidance=%s" % (
                    status.localized, status.chassis_ready, status.avoidance_ready),
                True,
            )
        if not status.map_digest:
            return ToolResult("覆盖后端未提供当前地图摘要。", True)
        try:
            saved = self._load_regions(status.map_digest)
        except ValueError as exc:
            return ToolResult(str(exc), True)
        by_key = {}
        for item in saved:
            by_key[item["name"].strip()] = item
            by_key[item["id"].strip()] = item
        selected = []
        selected_ids = set()
        for requested in regions:
            item = by_key.get(requested.strip())
            if item is None:
                return ToolResult(
                    "未知已保存区域 %r；当前可用: %s" % (
                        requested, ", ".join(item["name"] for item in saved)),
                    True,
                )
            if item["id"] in selected_ids:
                return ToolResult(
                    "同一已保存区域不能同时通过名称和 UUID 重复加入批次: %s" %
                    item["name"],
                    True,
                )
            selected_ids.add(item["id"])
            selected.append(item)
        from autolabor_coverage.msg import CoverageRegion
        from autolabor_coverage.srv import StartCoverageBatch, StartCoverageBatchRequest
        from geometry_msgs.msg import Point32
        service = "/coverage/start_batch"
        try:
            self._rospy.wait_for_service(service, timeout=2.0)
        except Exception as exc:
            # The request has not been attempted yet, so there is no uncertain
            # remote operation to retain or cancel.
            return ToolResult("覆盖批次服务未就绪: %s" % exc, True)

        terminal_states = {
            "COMPLETED", "COMPLETED_PARTIAL", "CANCELED", "FAILED",
        }
        with self._lock:
            previous_batch_id = self._ai_batch_id
            if (previous_batch_id and status.batch_id == previous_batch_id and
                    status.state in terminal_states and not status.active and
                    not status.batch_active):
                self._ai_batch_id = ""
                previous_batch_id = ""
            if previous_batch_id:
                return ToolResult(_json({
                    "accepted": False,
                    "batch_id": previous_batch_id,
                    "error": (
                        "上一 AI 覆盖批次尚未确认安全终结，拒绝覆盖其所有权"
                    ),
                }), True)

        batch_id = "coverage-batch-%s" % uuid.uuid4().hex
        request_attempted = False
        try:
            request = StartCoverageBatchRequest()
            request.client_request_id = batch_id
            request.operation_width_m = operation_width_m
            request.overlap_ratio = overlap_percent / 100.0
            request.allow_reverse_transit = bool(allow_reverse_transit)
            request.max_speed_mps = max_speed_mps
            request.reverse_speed_mps = reverse_speed_mps
            request.max_angular_speed_rps = max_angular_speed_rps
            request.linear_accel_mps2 = linear_accel_mps2
            request.angular_accel_rps2 = angular_accel_rps2
            request.direction_change_penalty_sec = direction_change_penalty_sec
            request.segment_handoff_penalty_sec = segment_handoff_penalty_sec
            request.transit_replan_period_sec = transit_replan_period_sec
            request.map_digest = status.map_digest
            for item in selected:
                region = CoverageRegion()
                region.id = item["id"]
                region.name = item["name"]
                region.region.header.stamp = self._rospy.Time.now()
                region.region.header.frame_id = "map"
                region.region.polygon.points = [
                    Point32(x=x, y=y, z=0.0) for x, y in item["points"]]
                request.regions.append(region)
            proxy = self._rospy.ServiceProxy(service, StartCoverageBatch)
            # Retain the ID before the transport can hand the request to J6M.
            # If the response is lost, the caller can still query/cancel this
            # exact operation rather than guessing the newest batch.
            with self._lock:
                self._ai_batch_id = batch_id
            request_attempted = True
            response = proxy(request)
        except Exception as exc:
            if not request_attempted:
                with self._lock:
                    if self._ai_batch_id == batch_id:
                        self._ai_batch_id = ""
                return ToolResult("覆盖批次请求构造失败: %s" % exc, True)
            cleanup = self._cancel_coverage_after_uncertain_start(batch_id)
            return ToolResult(_json({
                "accepted": False,
                "batch_id": batch_id,
                "error": "覆盖批次服务调用结果不确定: %s" % exc,
                "cancel_state": cleanup["cancel_state"],
                "cancel_detail": cleanup["message"],
                "outcome_uncertain": not cleanup["safe"],
            }), True)

        if response.batch_id != batch_id:
            cleanup = self._cancel_coverage_after_uncertain_start(batch_id)
            return ToolResult(_json({
                "accepted": False,
                "batch_id": batch_id,
                "error": (
                    "覆盖管理器返回了不匹配的 batch_id %r" %
                    response.batch_id
                ),
                "cancel_state": cleanup["cancel_state"],
                "cancel_detail": cleanup["message"],
                "outcome_uncertain": not cleanup["safe"],
            }), True)
        if not response.accepted:
            cleanup = self._cancel_coverage_after_uncertain_start(batch_id)
            return ToolResult(_json({
                "accepted": False,
                "batch_id": batch_id,
                "error": response.message or "覆盖批次未被接受",
                "cancel_state": cleanup["cancel_state"],
                "cancel_detail": cleanup["message"],
                "outcome_uncertain": not cleanup["safe"],
            }), True)
        return ToolResult(_json({
            "accepted": True, "state": "active",
            "batch_id": batch_id,
            "regions": [item["name"] for item in selected],
            "planning_parameters": {
                "operation_width_m": operation_width_m,
                "overlap_percent": overlap_percent,
                "max_speed_mps": max_speed_mps,
                "allow_reverse_transit": bool(allow_reverse_transit),
                "reverse_speed_mps": reverse_speed_mps,
                "max_angular_speed_rps": max_angular_speed_rps,
                "linear_accel_mps2": linear_accel_mps2,
                "angular_accel_rps2": angular_accel_rps2,
                "direction_change_penalty_sec": direction_change_penalty_sec,
                "segment_handoff_penalty_sec": segment_handoff_penalty_sec,
                "transit_replan_period_sec": transit_replan_period_sec,
            },
            "message": response.message,
        }))

    def _coverage_operator_defaults(self):
        """Read the complete Qt planner preference set for omitted AI args.

        Qt writes QSettings synchronously on every planning-control change.
        The MCP backend runs on the same NVIDIA host, so reading that file at
        mission submission avoids a second, drifting set of AI defaults.  Each
        field is range-checked independently and falls back closed to the
        shipped value if the file is absent or partially malformed.
        """
        defaults = {
            "operation_width_m": 1.0,
            "overlap_percent": 15.0,
            "max_speed_mps": 0.8,
            "allow_reverse_transit": True,
            "reverse_speed_mps": 0.3,
            "max_angular_speed_rps": 0.6,
            "linear_accel_mps2": 1.0,
            "angular_accel_rps2": 0.5,
            "direction_change_penalty_sec": 1.0,
            "segment_handoff_penalty_sec": 0.5,
            "transit_replan_period_sec": 1.0,
        }
        path = self._operator_settings_file
        if not path:
            return defaults
        parser = configparser.ConfigParser(interpolation=None)
        try:
            with open(path, "r", encoding="utf-8") as stream:
                parser.read_file(stream)
        except (OSError, configparser.Error, UnicodeError):
            return defaults
        if not parser.has_section("coverage"):
            return defaults

        numeric = {
            "operation_width_m": ("operation_width_m", 0.30, 3.00),
            "overlap_percent": ("overlap_percent", 0.0, 50.0),
            "max_speed_mps": ("max_forward_speed_mps", 0.10, 1.60),
            "reverse_speed_mps": ("max_reverse_speed_mps", 0.05, 0.80),
            "max_angular_speed_rps": (
                "max_angular_speed_rps", 0.10, 1.00
            ),
            "linear_accel_mps2": ("linear_accel_mps2", 0.10, 2.00),
            "angular_accel_rps2": ("angular_accel_rps2", 0.10, 1.00),
            "direction_change_penalty_sec": (
                "direction_change_penalty_sec", 0.0, 30.0
            ),
            "segment_handoff_penalty_sec": (
                "segment_handoff_penalty_sec", 0.0, 30.0
            ),
            "transit_replan_period_sec": (
                "transit_replan_period_sec", 1.0, 10.0
            ),
        }
        for output_key, (settings_key, minimum, maximum) in numeric.items():
            option = "planning_parameters\\" + settings_key
            try:
                value = float(parser.get("coverage", option))
            except (configparser.Error, TypeError, ValueError, OverflowError):
                continue
            if math.isfinite(value) and minimum <= value <= maximum:
                # QSettings may serialize a spin-box value such as 1.2 as
                # 1.2000000000000002.  Preserve UI precision without exposing
                # that binary artefact in service requests or tool status.
                defaults[output_key] = round(value, 9)

        try:
            raw_reverse = parser.get(
                "coverage", "planning_parameters\\allow_reverse"
            ).strip().lower()
        except configparser.Error:
            raw_reverse = ""
        if raw_reverse in ("1", "true", "yes", "on"):
            defaults["allow_reverse_transit"] = True
        elif raw_reverse in ("0", "false", "no", "off"):
            defaults["allow_reverse_transit"] = False
        return defaults

    def _cancel_coverage_batch_exact(self, batch_id):
        """Cancel/tombstone one operation ID and classify the confirmation."""
        from autolabor_coverage.srv import CancelCoverageBatch

        service = "/coverage/cancel_batch"
        self._rospy.wait_for_service(service, timeout=2.0)
        response = self._rospy.ServiceProxy(
            service, CancelCoverageBatch)(batch_id)
        if response.batch_id != batch_id:
            raise RuntimeError(
                "coverage cancel response batch_id mismatch: %r" %
                response.batch_id
            )

        if (response.success and response.not_started and
                not response.cancellation_requested):
            cancel_state = "confirmed_not_started"
            safe = True
        elif response.success and not response.cancellation_requested:
            cancel_state = "confirmed_terminal"
            safe = True
        elif response.cancellation_requested:
            cancel_state = "requested" if response.success else "uncertain"
            safe = False
        else:
            cancel_state = "uncertain"
            safe = False
        if safe:
            with self._lock:
                if self._ai_batch_id == batch_id:
                    self._ai_batch_id = ""
        return {
            "success": bool(response.success),
            "cancellation_requested": bool(response.cancellation_requested),
            "not_started": bool(response.not_started),
            "batch_id": batch_id,
            "cancel_state": cancel_state,
            "safe": safe,
            "message": response.message,
        }

    def _cancel_coverage_after_uncertain_start(self, batch_id):
        try:
            return self._cancel_coverage_batch_exact(batch_id)
        except Exception as exc:
            return {
                "success": False,
                "cancellation_requested": False,
                "not_started": False,
                "batch_id": batch_id,
                "cancel_state": "unavailable",
                "safe": False,
                "message": "精确覆盖撤销服务不可用: %s" % exc,
            }

    def _coverage_set_paused(self, paused):
        self._ensure()
        status = self._snapshot("coverage", 2.0)
        with self._lock:
            owned = self._ai_batch_id
        if not status or not owned or status.batch_id != owned:
            return ToolResult("当前覆盖任务不属于本 AI 会话。", True)
        from std_srvs.srv import SetBool
        try:
            self._rospy.wait_for_service("/coverage/set_paused", timeout=2.0)
            response = self._rospy.ServiceProxy(
                "/coverage/set_paused", SetBool)(paused)
        except Exception as exc:
            return ToolResult("覆盖暂停服务调用失败: %s" % exc, True)
        return ToolResult(response.message, not response.success)

    def pause_coverage(self):
        return self._coverage_set_paused(True)

    def resume_coverage(self):
        return self._coverage_set_paused(False)

    def skip_coverage_region(self):
        return self._coverage_trigger("/coverage/skip_current", clear=False)

    def cancel_coverage(self):
        if not self._coverage_submit_lock.acquire(False):
            return ToolResult(
                "AI 覆盖批次仍在提交或收敛，请稍后再精确取消。",
                True,
            )
        try:
            return self._cancel_coverage_serialized()
        finally:
            self._coverage_submit_lock.release()

    def _cancel_coverage_serialized(self):
        self._ensure()
        with self._lock:
            owned = self._ai_batch_id
        if not owned:
            return ToolResult("当前 AI 会话没有可取消的覆盖 batch_id。", True)
        try:
            result = self._cancel_coverage_batch_exact(owned)
        except Exception as exc:
            return ToolResult(_json({
                "batch_id": owned,
                "cancel_state": "unavailable",
                "message": "精确覆盖撤销服务调用失败: %s" % exc,
            }), True)
        return ToolResult(_json(result), not result["success"])

    def _coverage_trigger(self, service, clear=False):
        self._ensure()
        status = self._snapshot("coverage", 2.0)
        with self._lock:
            owned = self._ai_batch_id
        if not status or not owned or status.batch_id != owned:
            return ToolResult("当前覆盖任务不属于本 AI 会话。", True)
        from std_srvs.srv import Trigger
        try:
            self._rospy.wait_for_service(service, timeout=2.0)
            response = self._rospy.ServiceProxy(service, Trigger)()
        except Exception as exc:
            return ToolResult("覆盖控制服务调用失败: %s" % exc, True)
        # Trigger success only means that the manager recorded the request.
        # Retain the exact batch ID until get_coverage_status proves both a
        # terminal state and inactive ownership; otherwise an uncertain cleanup
        # could no longer be queried or canceled by this AI session.
        return ToolResult(response.message, not response.success)

    # ---- visual spot-cleaning tools ---------------------------------------------

    def start_spot_cleaning(self):
        self._ensure()
        if self._coverage_busy():
            return ToolResult("覆盖清扫正在独占导航，不能启动视觉伺服。", True)
        mode = self._parse_json_message(
            self._wait_snapshot("mode", 2.0, 2.5))
        if mode.get("state") != "GPS_ACTIVE":
            return ToolResult("视觉伺服只能从相对导航模式启动。", True)
        from std_srvs.srv import SetBool
        service = "/fod_navigation_mode/set_fod_enabled"
        try:
            self._rospy.wait_for_service(service, timeout=2.0)
            response = self._rospy.ServiceProxy(service, SetBool)(True)
        except Exception as exc:
            return ToolResult("视觉模式服务调用失败: %s" % exc, True)
        if not response.success:
            return ToolResult(response.message, True)
        deadline = time.monotonic() + 2.0
        state = "GPS_ACTIVE"
        while time.monotonic() < deadline:
            state = self._parse_json_message(
                self._snapshot("mode", 2.0)).get("state", "UNAVAILABLE")
            if state != "GPS_ACTIVE":
                break
            time.sleep(0.05)
        if state == "GPS_ACTIVE":
            return ToolResult(
                response.message or "附近没有满足接管条件的新鲜 FOD 目标，未执行定点清扫。",
                True,
            )
        with self._lock:
            self._ai_visual_owned = True
        return ToolResult(_json({
            "accepted": True, "state": _public_mode_state(state),
            "message": response.message,
        }))

    def stop_spot_cleaning(self):
        self._ensure()
        with self._lock:
            owned = self._ai_visual_owned
        if not owned:
            return ToolResult("当前视觉任务不属于本 AI 会话。", True)
        from std_srvs.srv import SetBool
        service = "/fod_navigation_mode/set_fod_enabled"
        try:
            self._rospy.wait_for_service(service, timeout=2.0)
            response = self._rospy.ServiceProxy(service, SetBool)(False)
        except Exception as exc:
            return ToolResult("退出视觉模式失败: %s" % exc, True)
        if response.success:
            with self._lock:
                self._ai_visual_owned = False
        return ToolResult(response.message, not response.success)
