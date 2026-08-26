# -*- coding: utf-8 -*-
"""ROS 后端 —— 把清扫车真实 ROS 操作封装成工具 handler（MCP_BACKEND=ros）。

设计要点：
- 懒加载 rospy：工具被调用时才 init_node，避免无 ROS 环境下 server 无法启动。
  init_node 用 disable_signals=True，避免抢占 SIGINT 与 MCP stdio 主循环冲突。
- 所有导航类工具"发布即返回"（异步），绝不 spin/等待到达；到达与否由 navigation_status 轮询。
- 状态读取用 wait_for_message 短超时，话题缺失时优雅降级（返回 N/A），不崩溃。
- 消息类型懒 import 并容错（例如仿真无 autolabor_canbus_driver，电量显示 N/A）。
- 安全：导航一律走受控链（/move_base_simple/goal 或 /gps/goal_fix），不裸写 /cmd_vel。

工具方法签名与 mock_backend 一一对应（名字/参数一致），由 tools.build_registry 绑定。
"""

import json
import math
import time

from sweeper_mcp.tools import ToolResult


def _dumps(data):
    return json.dumps(data, ensure_ascii=False)


def _make_tf_listener(buffer):
    """创建包一层安全析构的 tf2_ros.TransformListener。

    tf2_ros.TransformListener 的 __del__ 会访问 self.tf_sub；若构造时订阅未
    完成（ROS 未就绪 / 进程在构造中途被 SIGTERM），__del__ 抛 AttributeError，
    在进程退出/GC 时被打印成恼人的"类型错误"。这里子类化并覆盖 __del__，
    用 __dict__.get 兜住 tf_sub 不存在的情况，unregister 也包 try。
    """
    import tf2_ros

    class _SafeTransformListener(tf2_ros.TransformListener):
        def __del__(self):
            try:
                sub = self.__dict__.get("tf_sub")
                if sub is not None:
                    sub.unregister()
            except Exception:
                pass

    return _SafeTransformListener(buffer)


class ROSBackend:
    name = "ros"

    # 默认话题（config/sweeper_mcp.yaml 的 ros 段覆盖后注入）
    DEFAULTS = {
        "goal_topic": "/move_base_simple/goal",
        "goal_frame": "camera_init",
        "gps_goal_topic": "/gps/goal_fix",
        "cancel_topic": "/move_base/cancel",
        "status_topic": "/move_base/status",
        "odom_topic": "/odom",
        "pose_source_topics": ["/gps/odom", "/gps/pose", "/Odometry", "/odom"],
        "chassis_info_topic": "/m2_driver/chassis_info",
        "emergency_stop_topic": "/m2_driver/emergency_stop",
        "fod_mode_service": "/fod_navigation_mode/set_fod_enabled",
    }

    GOAL_STATUS_NAMES = {
        0: "pending", 1: "active", 2: "preempted", 3: "succeeded",
        4: "aborted", 5: "rejected", 6: "preempting", 7: "recalling",
        8: "recalled", 9: "lost",
    }

    def __init__(self, topics=None):
        self._topics = dict(self.DEFAULTS)
        if topics:
            self._topics.update(topics)
        self._rospy = None
        self._msg = {}
        self._pubs = {}
        self._tf_buffer = None
        self._tf_listener = None

    def _tf(self):
        """懒加载 tf2_ros Buffer/Listener（TF 是模式无关的位姿来源）。

        必须保存 listener 引用，否则会被 GC 回收（buffer 也收不到 /tf 更新）。
        用 _make_tf_listener 包装，避免 __del__ 抛 AttributeError。
        """
        if self._tf_buffer is None:
            import tf2_ros
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = _make_tf_listener(self._tf_buffer)
        return self._tf_buffer

    def _move_base_odom_topic(self):
        """读 move_base 实际使用的 odom 话题（决定当前定位模式的权威来源）。

        navigation_arena.launch 把 localization_source 映射成
        /move_base/TebLocalPlannerROS/odom_topic 参数：
          gps 模式 → /gps/odom；fast_lio 模式 → /Odometry。
        """
        try:
            return self._rospy.get_param("/move_base/TebLocalPlannerROS/odom_topic", None)
        except Exception:
            return None

    # ---------- ROS 基础设施 ----------

    def _ros(self):
        if self._rospy is not None:
            return self._rospy
        try:
            import rospy
            rospy.init_node("sweeper_mcp_backend", anonymous=True, disable_signals=True)
        except Exception as exc:
            raise RuntimeError("ROS 不可达（未 source 环境或 roscore 未启动）: %s" % exc)
        self._rospy = rospy
        return rospy

    def _load_msgs(self):
        """懒加载消息类型，缺失的置 None（容错，如仿真无 autolabor_canbus_driver）。"""
        if self._msg:
            return self._msg
        rospy = self._ros()
        m = {}
        from std_msgs.msg import Bool
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import NavSatFix
        from nav_msgs.msg import Odometry
        from actionlib_msgs.msg import GoalID, GoalStatusArray
        m["Bool"] = Bool
        m["PoseStamped"] = PoseStamped
        m["NavSatFix"] = NavSatFix
        m["Odometry"] = Odometry
        m["GoalID"] = GoalID
        m["GoalStatusArray"] = GoalStatusArray
        try:
            from autolabor_canbus_driver.msg import ChassisStatusInfo
            m["ChassisStatusInfo"] = ChassisStatusInfo
        except Exception:
            m["ChassisStatusInfo"] = None
        try:
            from std_srvs.srv import SetBool
            m["SetBool"] = SetBool
        except Exception:
            m["SetBool"] = None
        self._msg = m
        return m

    def _pub(self, topic, msg_type, latch=False):
        if topic not in self._pubs:
            self._pubs[topic] = self._rospy.Publisher(topic, msg_type, queue_size=10, latch=latch)
        return self._pubs[topic]

    def _wait_msg(self, topic, msg_type, timeout=2.0):
        try:
            return self._rospy.wait_for_message(topic, msg_type, timeout=timeout)
        except Exception:
            return None

    def _publish_reliable(self, pub, msg, repeat=3, gap=0.2, wait=3.0):
        """发布导航目标并保证送达，规避两种"首条即丢"的坑（实测确认）：

        1. **仿真时钟未同步**：use_sim_time 下 server 进程刚启动时
           rospy.Time.now() 返回 0，此时发的目标带 0 时间戳，move_base 会
           误判"已到达"（瞬时 status=succeeded 且车不动）——可靠性测试中
           目标1 正是如此。必须先等 /clock 同步（Time.now() > 0）再发。
        2. **publisher 连接未建立**：新建 Publisher 后立即 publish，TCP 握手
           + master 注册是异步的，订阅方可能收不到首条。先发一条触发注册，
           再等 get_num_connections()>0，随后补发多次。

        每次补发前重打最新时间戳（导航目标重复发布是幂等的，move_base 以后到
        者为准；重复不会造成伤害）。非 PoseStamped 消息（无 header）跳过重打。
        """
        rospy = self._ros()
        # 1) 等仿真时钟同步
        t0 = time.time()
        while time.time() - t0 < wait:
            try:
                if rospy.Time.now().to_sec() > 0.0:
                    break
            except Exception:
                pass
            time.sleep(0.05)
        # 2) 首条触发 publisher 注册，等至少一个订阅者连上
        pub.publish(msg)
        t0 = time.time()
        while time.time() - t0 < wait:
            try:
                if pub.get_num_connections() > 0:
                    break
            except Exception:
                pass
            time.sleep(0.05)
        # 3) 补发多次（幂等），每次用最新时间戳保证 move_base 收到有效目标
        for i in range(repeat):
            if hasattr(msg, "header"):
                msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
            if i < repeat - 1:
                time.sleep(gap)

    def _current_pose(self):
        """读机器人当前位姿 (x, y, yaw)。**模式自适应**，绝不硬编码某个话题：

        1. TF 查 camera_init→base_link（GPS 与 FAST_LIO 两种模式都发布这条 TF 链，
           是模式无关且最准的来源）；
        2. 回退：读 move_base 的 TebLocalPlannerROS/odom_topic 参数（决定当前定位模式），
           用该话题的 Odometry；
        3. 最后回退：配置的 pose_source_topics 列表。

        TF/位姿来源不可达时返回 None，由调用方提示"无法读取当前位姿"。
        """
        pose = self._pose_from_tf()
        if pose is not None:
            return pose
        return self._pose_from_odom_topics()

    def _pose_from_tf(self):
        """TF 方式：查 camera_init→base_link（全局帧下 base_link 位姿），两模式通用。"""
        try:
            t = self._tf().lookup_transform(
                self._topics["goal_frame"], "base_link", self._rospy.Time(0),
                timeout=self._rospy.Duration(1.0))
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.transform.translation.x, t.transform.translation.y, yaw)

    def _pose_from_odom_topics(self):
        """话题方式：先 move_base 实际 odom_topic 参数，再配置的 pose_source_topics。"""
        m = self._load_msgs()
        topics = []
        mb_topic = self._move_base_odom_topic()
        if mb_topic:
            topics.append(mb_topic)
        topics.extend(self._topics["pose_source_topics"])
        seen = set()
        for topic in topics:
            if topic in seen:
                continue
            seen.add(topic)
            odom = self._wait_msg(topic, m["Odometry"], 1.5)
            if odom is not None:
                return self._odom_pose(odom)
        return None

    @staticmethod
    def _odom_pose(odom):
        p = odom.pose.pose
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (p.position.x, p.position.y, yaw)

    # ---------- 工具 handler ----------

    def get_robot_status(self):
        m = self._load_msgs()
        out = {}
        # 电量 / 急停（真车；仿真无此话题 → N/A）
        info = None
        if m.get("ChassisStatusInfo"):
            info = self._wait_msg(self._topics["chassis_info_topic"], m["ChassisStatusInfo"], 2.0)
        if info is not None:
            out["battery_percent"] = getattr(info, "battery_percent", None)
            out["battery_voltage"] = getattr(info, "battery_voltage", None)
            out["emergency"] = {
                "hard": bool(getattr(info, "hard_emergency", False)),
                "soft": bool(getattr(info, "soft_emergency", False)),
                "gamepad": bool(getattr(info, "gamepad_emergency", False)),
                "robot": bool(getattr(info, "robot_emergency", False)),
            }
        else:
            out["battery_percent"] = "N/A"
            out["emergency"] = "N/A"
        # 位姿
        pose = self._current_pose()
        if pose:
            out["position"] = {"x": round(pose[0], 3), "y": round(pose[1], 3), "yaw": round(pose[2], 3)}
        else:
            out["position"] = "N/A"
        # 导航状态
        out["navigation_status"] = self.navigation_status().text
        # 清扫状态
        try:
            from sweeper_mcp import sweep_backend
            out["sweep_state"] = sweep_backend.sweep_status_text()
        except Exception:
            out["sweep_state"] = "N/A"
        return ToolResult(_dumps(out), False)

    def navigate_pose(self, x, y, yaw=0.0, frame_id=None):
        m = self._load_msgs()
        frame_id = frame_id or self._topics["goal_frame"]
        goal = m["PoseStamped"]()
        goal.header.stamp = self._rospy.Time.now()
        goal.header.frame_id = frame_id
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        # 标准单位四元数：z=sin(yaw/2), w=cos(yaw/2)。
        # 注意：不能把 yaw 直接写进 orientation.z（模长≠1，move_base 判定"目标朝向非法"而 abort）。
        half = float(yaw) / 2.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = math.sin(half)
        goal.pose.orientation.w = math.cos(half)
        self._publish_reliable(self._pub(self._topics["goal_topic"], m["PoseStamped"]), goal)
        return ToolResult(
            "已发布导航目标: x=%.3f y=%.3f yaw=%.3f (frame=%s)，异步执行中（用 navigation_status 查询）。"
            % (x, y, yaw, frame_id), False)

    def navigate_relative(self, dx=0.0, dy=0.0, dyaw=0.0):
        pose = self._current_pose()
        if pose is None:
            return ToolResult("无法读取当前位姿（全局帧话题不可达），无法换算相对目标。", True)
        cx, cy, cyaw = pose
        tx = cx + dx * math.cos(cyaw) - dy * math.sin(cyaw)
        ty = cy + dx * math.sin(cyaw) + dy * math.cos(cyaw)
        tyaw = (cyaw + dyaw) % (2.0 * math.pi)
        return self.navigate_pose(tx, ty, tyaw)

    def navigate_gps(self, latitude, longitude, altitude=None):
        m = self._load_msgs()
        fix = m["NavSatFix"]()
        fix.header.stamp = self._rospy.Time.now()
        fix.header.frame_id = "gps"
        fix.latitude = float(latitude)
        fix.longitude = float(longitude)
        if altitude is not None:
            fix.altitude = float(altitude)
        fix.status.service = 1
        fix.status.status = 0
        self._publish_reliable(self._pub(self._topics["gps_goal_topic"], m["NavSatFix"]), fix)
        return ToolResult(
            "已发布 GPS 目标: lat=%.6f lon=%.6f（gps_goal_node 将转成 move_base 目标）。"
            % (latitude, longitude), False)

    def cancel_navigation(self):
        m = self._load_msgs()
        goal = m["GoalID"]()
        goal.stamp = self._rospy.Time.now()
        self._publish_reliable(self._pub(self._topics["cancel_topic"], m["GoalID"]), goal)
        return ToolResult("已取消当前导航目标。", False)

    def navigation_status(self):
        m = self._load_msgs()
        status = self._wait_msg(self._topics["status_topic"], m["GoalStatusArray"], 1.5)
        if status is None or not status.status_list:
            return ToolResult("idle（当前无导航目标或状态不可达）", False)
        # move_base 的 status_list 按"旧→新"排列，最后一条才是最近发布的目标。
        # 只报最近这条的状态：否则会把历史已成功/被抢占的目标混进来，造成
        # "旧目标 succeeded 被当成当前结果 → 监控秒判到达"的误报（实测复现过：
        # 顺序指令里第二条目标用指令1出发前的过期位姿计算）。
        # 状态文本(text)照常带上，如 "aborted: Goal position is in an obstacle"，
        # Agent 能据此如实转告用户。
        s = status.status_list[-1]
        name = self.GOAL_STATUS_NAMES.get(s.status, "?%d" % s.status)
        text = getattr(s, "text", "") or ""
        return ToolResult(("%s: %s" % (name, text)) if text else name, False)

    def emergency_stop(self, active, reason=None):
        m = self._load_msgs()
        msg = m["Bool"]()
        msg.data = bool(active)
        self._publish_reliable(self._pub(self._topics["emergency_stop_topic"], m["Bool"]), msg)
        txt = "已触发急停。" if active else "已解除急停。"
        if reason:
            txt += " 原因: %s" % reason
        return ToolResult(txt, False)

    def sweep_set(self, action):
        from sweeper_mcp import sweep_backend
        return sweep_backend.sweep_set(action)

    def sweep_coverage(self, area=None, pattern=None, duration=None, width=None):
        return ToolResult("全覆盖清扫尚未实现（接口已预留）。当前仅支持定点清扫开关 sweep_set(on/off/toggle)。", True)

    def set_fod_mode(self, enabled):
        m = self._load_msgs()
        if m.get("SetBool") is None:
            return ToolResult("std_srvs/SetBool 不可用。", True)
        service = self._topics["fod_mode_service"]
        try:
            if not self._rospy.wait_for_service(service, timeout=2.0):
                return ToolResult("服务 %s 不在线（FOD 模式管理器未启动）。" % service, True)
            req = m["SetBool"]()
            req.data = bool(enabled)
            resp = self._rospy.ServiceProxy(service, m["SetBool"])(req)
            return ToolResult("已切换 FOD 模式: enabled=%s → success=%s" % (enabled, resp.success), not resp.success)
        except Exception as exc:
            return ToolResult("切换 FOD 模式异常: %s" % exc, True)
