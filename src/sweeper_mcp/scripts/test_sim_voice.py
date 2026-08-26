#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仿真语音闭环测试 —— 验证「收到指令就移动，到达目标停下，等待下一条指令」。

前置：已启动 roslaunch sweeper_mcp sim_voice_demo.launch（标准仿真世界 + 语音 Agent）。
测试向 /voice/text 发送中文指令 → Agent(大模型+function calling) 调用 MCP 工具 →
仿真车移动 → 校验目标到达(move_base SUCCEEDED)后停下 → 发送下一条。

用法:
  python3 scripts/test_sim_voice.py                 # 跑内置指令序列
  python3 scripts/test_sim_voice.py --interactive   # 手动逐条输入指令并观察
"""

import math
import os
import sys
import time

_PKG_SRC = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
if _PKG_SRC not in sys.path:
    sys.path.insert(0, _PKG_SRC)

import rospy
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

INPUT_TOPIC = "/voice/text"
REPLY_TOPIC = "/voice/agent_reply"
STATUS_TOPIC = "/move_base/status"
GOAL_TOPIC = "/move_base_simple/goal"
POSE_TOPIC = "/gps/odom"          # 仿真里 camera_init 帧的位姿

GOAL_STATUS_NAMES = {0:"pending",1:"active",2:"preempted",3:"succeeded",
                     4:"aborted",5:"rejected",6:"preempting",7:"recalling",
                     8:"recalled",9:"lost"}

# 内置测试序列：(标签, 指令, 期望位移米或None)
# 说明：纯原地转向不做自动化用例 —— 本项目 TEB 配置 yaw_goal_tolerance=6.283(全圈)，
# move_base 不强制到达朝向，纯旋转目标会被当作"已到达"。转向请用带位移的指令
# （如"左前方走4米"），机器人会在行进中转向。
DEFAULT_CASES = [
    ("前进5米",  "往前走5米",         5.0),
    ("查询状态", "查询机器人当前状态", None),
    ("左平移3米", "往正左方向平移3米", 3.0),
    ("后退3米",  "往后退3米",         3.0),
]


class SimVoiceTester:
    def __init__(self, pose_topic=POSE_TOPIC):
        self._reply_count = 0
        self._latest_reply = None
        self._latest_status = None
        self._latest_goal = None
        self._pose = None          # (x, y, yaw)
        self._twist = (0.0, 0.0)   # (linear.x, angular.z)
        self._pub = rospy.Publisher(INPUT_TOPIC, String, queue_size=10)
        rospy.Subscriber(REPLY_TOPIC, String, self._on_reply, queue_size=10)
        rospy.Subscriber(STATUS_TOPIC, GoalStatusArray, self._on_status, queue_size=10)
        rospy.Subscriber(GOAL_TOPIC, PoseStamped, self._on_goal, queue_size=10)
        rospy.Subscriber(pose_topic, Odometry, self._on_odom, queue_size=10)

    # ---- 回调 ----
    def _on_reply(self, msg):
        self._reply_count += 1
        self._latest_reply = msg.data

    def _on_status(self, msg):
        self._latest_status = msg

    def _on_goal(self, msg):
        self._latest_goal = msg

    def _on_odom(self, msg):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._pose = (p.position.x, p.position.y, yaw)
        self._twist = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    # ---- 启动竞态兜底 ----
    def preflight(self):
        """确保 move_base 可用（否则重启一次）。

        项目仿真启动存在竞态：move_base costmap 初始化时若 camera_init→base_link
        TF 还没就绪，costmap 会放弃等待，此后所有目标"秒到达"、车不动。
        这里用小位移目标健康检查，车不动就重启 move_base（respawn 会自动拉起），
        这次 TF 已就绪，costmap 能正常初始化。
        """
        import subprocess
        if self.wait_pose(20.0) is None:
            print("✗ 等不到 /gps/odom 位姿（仿真未就绪？）")
            return False
        for attempt in range(2):
            from sweeper_mcp.mcp_client import MCPClient
            c = MCPClient(server_env_backend="ros")
            c.initialize()
            p0 = self._pose
            c.call_tool("navigate_relative", {"dx": 1.0})
            t = time.time()
            moved = 0.0
            while time.time() - t < 20:
                if self._pose:
                    moved = math.hypot(self._pose[0] - p0[0], self._pose[1] - p0[1])
                    if moved > 0.3:
                        break
                time.sleep(0.3)
            c.close()
            if moved > 0.3:
                print("  ✓ move_base 导航可用（车能动 %.2fm）" % moved)
                return True
            if attempt == 0:
                print("  → 车未动，重启 move_base（启动竞态兜底）...")
                subprocess.run(["rosnode", "kill", "/move_base"], capture_output=True)
                time.sleep(15)
        print("✗ move_base 始终无法导航（检查 /tf 与仿真状态）")
        return False

    # ---- 等待辅助 ----
    def wait_pose(self, timeout=10.0):
        start = time.time()
        while time.time() - start < timeout:
            if self._pose is not None:
                return self._pose
            time.sleep(0.2)
        return None

    def wait_reply(self, timeout=60.0):
        base = self._reply_count
        start = time.time()
        while time.time() - start < timeout:
            if self._reply_count > base:
                return self._latest_reply
            time.sleep(0.2)
        return None

    def current_status(self):
        if self._latest_status is None or not self._latest_status.status_list:
            return "idle"
        last = self._latest_status.status_list[-1]
        return GOAL_STATUS_NAMES.get(last.status, "?%d" % last.status)

    def wait_goal_success(self, timeout=90.0):
        start = time.time()
        while time.time() - start < timeout:
            if self.current_status() == "succeeded":
                return True
            time.sleep(0.5)
        return False

    def send_command(self, text):
        msg = String(data=text)
        # 等 /voice/text 订阅者连接建立后再发，且只发一次。
        # 之前重复发布会把同一条指令发两遍：Agent 并发执行两遍完整计划，
        # 顺序指令会交错执行、第一条总结到达时车仍在动（"未停下"假失败）。
        for _ in range(20):
            if self._pub.get_num_connections() > 0:
                break
            time.sleep(0.1)
        self._pub.publish(msg)

    def is_stopped(self, window=2.0, tol=0.15):
        """采样 window 秒，位姿漂移 < tol 米视为已停下。"""
        p0 = self.wait_pose(5.0)
        if p0 is None:
            return False
        time.sleep(window)
        p1 = self.wait_pose(1.0)
        if p1 is None:
            return False
        return math.hypot(p1[0] - p0[0], p1[1] - p0[1]) < tol

    # ---- 顺序指令用例（任务1：先...然后再...）----
    def run_sequence_case(self, label, command, expect_dist, timeout=180.0):
        """一条指令含多个连续动作（如'先往前走2米，然后再往前开3米'）。

        与 run_case 的区别：Agent 现在会先把指令拆成步骤列表、按顺序执行，
        全部完成后才发布最终回复，因此要等回复后再校验总位移，而不是靠
        move_base 状态（两条导航之间状态会 succeeded→active 切换）。
        """
        print("\n===== 用例: %s | 指令: %s =====" % (label, command))
        pose0 = self.wait_pose(10.0)
        if pose0 is None:
            print("  ✗ 等不到位姿（/gps/odom 未发布？）")
            return False

        self.send_command(command)
        reply = self.wait_reply(timeout)
        if reply is None:
            print("  ✗ 未收到 Agent 回复（/voice/agent_reply 超时）")
            return False
        print("  ✓ Agent 回复: %s" % reply[:120])

        # 顺序执行总位移校验（等待位移达到期望再量）
        start = time.time()
        moved = 0.0
        while time.time() - start < timeout:
            if self._pose:
                moved = math.hypot(self._pose[0] - pose0[0], self._pose[1] - pose0[1])
                if moved >= expect_dist * 0.9:
                    break
            time.sleep(0.5)

        ok = expect_dist * 0.5 <= moved <= expect_dist * 2.5
        if ok:
            print("  ✓ 顺序执行完成，总位移 %.2fm ≈ 期望 %.2fm" % (moved, expect_dist))
        else:
            print("  ✗ 总位移 %.2fm 不在期望 %.2fm 附近" % (moved, expect_dist))

        if not self.is_stopped():
            print("  ✗ 到达目标后未停下（仍有运动）")
            ok = False
        else:
            print("  ✓ 到达目标后已停下，等待下一条命令")
        return ok

    # ---- 单条用例 ----
    def run_case(self, label, command, expect_dist, retries=1):
        """执行一条指令并校验。移动用例允许重试一次（LLM 工具选择偶有波动）。"""
        print("\n===== 用例: %s | 指令: %s =====" % (label, command))
        pose0 = self.wait_pose(10.0)
        if pose0 is None:
            print("  ✗ 等不到位姿（/gps/odom 未发布？）")
            return False

        self.send_command(command)
        reply = self.wait_reply(90.0)
        if reply is None:
            print("  ✗ 未收到 Agent 回复（/voice/agent_reply 超时）")
            return False
        print("  ✓ Agent 回复: %s" % reply[:100])

        if expect_dist is None:
            # 纯查询，无需移动
            print("  ✓ 非移动指令")
            return True

        moved = 0.0
        for attempt in range(retries + 1):
            if attempt > 0:
                print("  → 位移不足，重发指令（第 %d 次）..." % attempt)
                self.send_command(command)
                self.wait_reply(60.0)
            if not self.wait_goal_success(120.0):
                print("  ✗ move_base 未到达目标，当前状态: %s" % self.current_status())
                return False
            time.sleep(1.0)
            pose1 = self.wait_pose(5.0)
            if pose1 is None:
                return False
            moved = math.hypot(pose1[0] - pose0[0], pose1[1] - pose0[1])
            if moved >= expect_dist * 0.4:
                break

        ok = True
        if moved < expect_dist * 0.5 or moved > expect_dist * 2.5:
            print("  ✗ 位移 %.2fm 不在期望 %.2fm 附近（疑似 LLM 工具选择波动）" % (moved, expect_dist))
            ok = False
        else:
            print("  ✓ move_base SUCCEEDED，位移 %.2fm ≈ 期望 %.2fm" % (moved, expect_dist))

        # 到达后必须停下（等待下一条命令）
        if not self.is_stopped():
            print("  ✗ 到达目标后未停下（仍有运动）")
            ok = False
        else:
            print("  ✓ 到达目标后已停下，等待下一条命令")
        return ok


def main():
    if "--interactive" in sys.argv:
        interactive = True
    else:
        interactive = False

    rospy.init_node("sim_voice_test", anonymous=True)
    tester = SimVoiceTester()
    # 等节点网络就绪
    time.sleep(1.5)

    # 启动竞态兜底：确保 move_base 能导航
    if not tester.preflight():
        print("preflight 失败，测试终止。")
        return 2

    if interactive:
        print("交互模式：输入指令回车（如 '往前走5米'），q 退出。")
        while True:
            try:
                line = input("指令> ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if line.lower() in ("q", "quit", "exit"):
                break
            if not line:
                continue
            tester.run_case("手动", line, None)
        return 0

    cases = list(DEFAULT_CASES)
    # 顺序指令用例（任务1/2：拆分+顺序执行+自动监控）
    SEQUENCE_CASES = [
        ("顺序:先2米再3米", "先往前走2米，然后再往前开3米", 5.0),
    ]
    total = len(cases) + len(SEQUENCE_CASES)
    passed = 0
    for label, command, dist in cases:
        if tester.run_case(label, command, dist):
            passed += 1
        time.sleep(1.0)
    for label, command, dist in SEQUENCE_CASES:
        if tester.run_sequence_case(label, command, dist):
            passed += 1
        time.sleep(1.0)

    print("\n====================")
    print("通过 %d/%d 个用例" % (passed, total))
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
