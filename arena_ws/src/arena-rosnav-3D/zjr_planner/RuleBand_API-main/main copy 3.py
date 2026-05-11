#!/usr/bin/env python3
# monitor_and_override.py
"""
监控最短激光距离并在阈值内触发一次 pipeline -> override 行为。

用法示例（rosrun 或直接运行）：
  rosrun <pkg> monitor_and_override.py
或
  python3 monitor_and_override.py --scan_topic /scan --threshold 0.3 --period 0.9 --goal_x 1.73 --goal_y -9.57

主要参数可通过命令行或 rosparam 覆盖：
  ~scan_topic, ~threshold, ~period, ~min_interval, ~final_goal_x, ~final_goal_y
"""

import os
import time
import math
import glob
import subprocess
import warnings
import argparse
import json

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf.transformations as tft

# pipeline modules (确保这些模块在你的环境可 import)
from pointcloud_generate import generate_pointcloud_grid
from merge_data import merge_start_and_paths
from ruleband_api import RuleBandAPI

# action / message imports
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"
warnings.filterwarnings("ignore", category=UserWarning, message=".*libiomp5md")


# ------------------ 工具函数（可按需替换为你已有实现） ------------------

def get_robot_pose(timeout=5.0):
    """从 /odom 读取机器人位姿 (x,y,yaw)。超时返回 None。"""
    try:
        msg = rospy.wait_for_message("/odom", Odometry, timeout=timeout)
    except rospy.ROSException:
        rospy.logerr("Timeout waiting for /odom (%.1fs)", timeout)
        return None
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
    return float(x), float(y), float(yaw)


def find_rrt_script():
    candidates = [
        "~/catkin_arena/src/zjr_planner/RuleBand_API-main/rrt_path_generate.py",
        "/home/robot/catkin_arena/src/zjr_planner/RuleBand_API-main/rrt_path_generate.py",
        "./rrt_path_generate.py",
    ]
    for p in candidates:
        p_exp = os.path.expanduser(p)
        if os.path.exists(p_exp):
            return os.path.abspath(p_exp)
    return None


def run_rrt_once(goal_x, goal_y, out_file=None, num_paths=3, step=0.3, max_iters=4000):
    """调用外部 RRT 脚本生成 paths.json"""
    rrt_script = find_rrt_script()
    if rrt_script is None:
        raise FileNotFoundError("Cannot find rrt_path_generate.py; please set correct path in find_rrt_script()")
    if out_file:
        out_file = os.path.expanduser(out_file)
        os.makedirs(os.path.dirname(out_file), exist_ok=True)

    cmd = [
        "python3", rrt_script,
        "--goal", str(float(goal_x)), str(float(goal_y)),
        "--num_paths", str(int(num_paths)),
        "--step", str(float(step)),
        "--max_iters", str(int(max_iters))
    ]
    if out_file:
        cmd.extend(["--out", out_file])

    rospy.loginfo("Running RRT: %s", " ".join(cmd))
    res = subprocess.run(cmd, capture_output=True, text=True)
    rospy.loginfo("RRT stdout: %s", res.stdout.strip() or "<empty>")
    if res.stderr:
        rospy.logwarn("RRT stderr: %s", res.stderr.strip())
    if res.returncode != 0:
        raise RuntimeError("RRT script failed (rc=%d)" % res.returncode)
    return out_file


def pipeline_generate_subgoal_from_pose(start_x, start_y, goal_x, goal_y):
    """
    执行点云 -> RRT -> merge -> RuleBand，返回 (x_sub, y_sub)。
    注意：这个函数会调用 generate_pointcloud_grid()（保存点云文件），
    再生成 paths.json，最后调用 merge_start_and_paths 并 feed 给 RuleBand。
    """
    rospy.loginfo("Pipeline: saving pointcloud...")
    saved_file = generate_pointcloud_grid(index=5)
    rospy.loginfo("Pointcloud saved to: %s", saved_file)

    out_paths = os.path.expanduser("~/catkin_arena/src/zjr_planner/RuleBand_API-main/data_json/paths.json")
    run_rrt_once(goal_x, goal_y, out_file=out_paths)

    merged_out = os.path.expanduser("~/catkin_arena/src/zjr_planner/RuleBand_API-main/data_json/example_data.json")
    merged_file = merge_start_and_paths((float(start_x), float(start_y)), out_path=merged_out)
    rospy.loginfo("Saved merged file to: %s", merged_file)

    api = RuleBandAPI(device="cpu")
    x_sub0, y_sub0 = api.predict_from_file(merged_file, debug=True)
    x_sub = 2 * start_x - x_sub0
    y_sub = 2 * start_y - y_sub0

    rospy.loginfo("Sampled rule-based sub-goal → (%.3f, %.3f)", x_sub, y_sub)
    return float(x_sub), float(y_sub)


def send_action_goal_and_cancel(x, y, yaw=0.0, duration=0.5, wait_server=2.0):
    """
    使用 move_base action 发送 goal 并在 duration 秒后 cancel。
    如果 action server 不可用，返回 False（上层调用 fallback 发布）。
    """
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server (%.1fs)...", wait_server)
    if not client.wait_for_server(rospy.Duration(wait_server)):
        rospy.logwarn("move_base action server not available after %.1fs", wait_server)
        return False

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = float(x)
    goal.target_pose.pose.position.y = float(y)
    q = tft.quaternion_from_euler(0.0, 0.0, float(yaw))
    goal.target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

    rospy.loginfo("Sending action goal to move_base: x=%.3f y=%.3f yaw=%.3f", x, y, yaw)
    client.send_goal(goal)

    start = time.time()
    try:
        while time.time() - start < float(duration) and not rospy.is_shutdown():
            rospy.sleep(0.02)
    except rospy.ROSInterruptException:
        rospy.logwarn("Interrupted while waiting to cancel goal")

    rospy.loginfo("Cancelling move_base goal after %.3fs", duration)
    client.cancel_goal()
    rospy.sleep(0.05)
    return True


def fallback_publish_goal_once(x, y, yaw=0.0, topic="/move_base_simple/goal", wait_conn=1.0):
    pub = rospy.Publisher(topic, PoseStamped, queue_size=1, latch=False)
    start_t = time.time()
    while pub.get_num_connections() == 0 and time.time() - start_t < wait_conn and not rospy.is_shutdown():
        rospy.sleep(0.02)
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()
    q = tft.quaternion_from_euler(0.0, 0.0, float(yaw))
    goal.pose = Pose(Point(float(x), float(y), 0.0), Quaternion(q[0], q[1], q[2], q[3]))
    pub.publish(goal)
    rospy.loginfo("Published fallback pose to %s: x=%.3f y=%.3f", topic, x, y)
    return True


# ------------------ 本次核心：检查最短距离并触发 pipeline 的函数 ------------------

def get_min_scan_distance(scan_topic="/scan", timeout=1.0):
    """
    一次性读取 scan_topic（sensor_msgs/LaserScan），返回最小距离（float）。
    忽略 NaN、inf、和小于 range_min / 大于 range_max 的点。
    如果读取失败或没有有效点，返回 None。
    """
    try:
        scan = rospy.wait_for_message(scan_topic, LaserScan, timeout=timeout)
    except rospy.ROSException:
        rospy.logwarn("Timeout waiting for scan on %s", scan_topic)
        return None

    ranges = scan.ranges
    if not ranges:
        return None

    valid = []
    for r in ranges:
        if r is None:
            continue
        # 排除 NaN / inf
        if math.isfinite(r):
            # 还可以限制在 sensor 的 range_min/range_max 之内
            if r >= getattr(scan, "range_min", 0.0) and r <= getattr(scan, "range_max", float("inf")):
                valid.append(r)
    if not valid:
        return None
    return float(min(valid))


def monitor_and_override_once(scan_topic,
                              threshold,
                              final_goal,
                              override_sub_dur=0.5,
                              override_final_dur=0.1,
                              rrt_out_paths=None):
    """
    检查一次扫描的最短距离；如果 < threshold 则触发 pipeline 并做 override 操作（sub 与 final）。
    final_goal: (x,y) tuple
    rrt_out_paths: 若指定，传给 run_rrt 的输出路径位置（字符串）
    返回 True 如果触发并成功执行（或者至少尝试发送），False 表示未触发或失败。
    """
    min_d = get_min_scan_distance(scan_topic, timeout=1.0)
    if min_d is None:
        rospy.logwarn("No valid scan readings")
        return False

    rospy.loginfo("Min scan distance: %.3f m (threshold %.3f)", min_d, threshold)
    if min_d >= threshold:
        return False  # 不触发

    # 触发：先读当前位姿
    pose = get_robot_pose(timeout=2.0)
    if pose is None:
        rospy.logerr("Cannot get robot pose; aborting trigger")
        return False
    cur_x, cur_y, cur_yaw = pose

    try:
        # pipeline: 生成子目标
        x_sub, y_sub = pipeline_generate_subgoal_from_pose(cur_x, cur_y, final_goal[0], final_goal[1])
    except Exception as e:
        rospy.logerr("Pipeline failed: %s", e)
        return False

    rospy.loginfo("Generated subgoal: (%.3f, %.3f)", x_sub, y_sub)

    # 1) 发布子目标 override（duration override_sub_dur）
    ok = send_action_goal_and_cancel(x_sub, y_sub, yaw=0.0, duration=override_sub_dur, wait_server=2.0)
    if not ok:
        rospy.logwarn("Action server unavailable; fallback publish for subgoal")
        fallback_publish_goal_once(x_sub, y_sub, yaw=0.0)

    rospy.sleep(0.05)  # 确保 cancel 传播

    # 2) 发布最终目标 override（duration override_final_dur）
    if final_goal is not None:
        fx, fy = float(final_goal[0]), float(final_goal[1])
        ok2 = send_action_goal_and_cancel(fx, fy, yaw=0.0, duration=override_final_dur, wait_server=2.0)
        if not ok2:
            rospy.logwarn("Action server unavailable; fallback publish for final goal")
            fallback_publish_goal_once(fx, fy, yaw=0.0)
    else:
        rospy.logwarn("final_goal not provided; skipping final goal publish")

    return True


# ------------------ 主节点内循环（示例） ------------------

def main_loop_monitor(scan_topic="/scan",
                      threshold=0.3,
                      period=0.9,
                      final_goal=(1.73, -9.57),
                      override_sub_dur=0.5,
                      override_final_dur=0.1):
    """
    主循环：每 period 秒检查一次最短距离；若触发则在该周期内执行一次 pipeline+override。
    使用 last_trigger_time 防止短时间内重复触发。
    """
    rospy.loginfo("monitor_and_override started: scan_topic=%s threshold=%.3f period=%.3f final_goal=(%.3f,%.3f)",
                  scan_topic, threshold, period, final_goal[0], final_goal[1])

    last_trigger_time = 0.0
    while not rospy.is_shutdown():
        loop_start = time.time()

        try:
            min_d = get_min_scan_distance(scan_topic, timeout=0.8)
        except Exception as e:
            rospy.logwarn("Error reading scan: %s", e)
            min_d = None

        triggered = False
        if min_d is not None and min_d < threshold:
            now = time.time()
            if now - last_trigger_time >= period - 1e-6:
                rospy.loginfo("Min distance %.3f < %.3f: triggering pipeline & overrides", min_d, threshold)
                # 执行一次监测触发（内部会自己调用 pipeline 等）
                try:
                    triggered = monitor_and_override_once(scan_topic, threshold, final_goal,
                                                          override_sub_dur=override_sub_dur,
                                                          override_final_dur=override_final_dur)
                except Exception as e:
                    rospy.logerr("monitor_and_override_once failed: %s", e)
                    triggered = False

                if triggered:
                    last_trigger_time = now
            else:
                rospy.loginfo("Detected small distance but last trigger %.3fs ago (< period %.3fs): skip", now - last_trigger_time, period)
        # else: 没有小于阈值，不触发

        # 休眠至下一个周期（保证周期为 period）
        elapsed = time.time() - loop_start
        to_sleep = period - elapsed
        if to_sleep > 0:
            rospy.sleep(to_sleep)
        else:
            rospy.logwarn("monitor loop overran period by %.3fs", -to_sleep)

    def symmetric_point(A, B):
        """
        输入：
            A: tuple/list, 点A的坐标 (x, y)
            B: tuple/list, 对称中心点B的坐标 (x, y)
        返回：
            C: tuple, 点C的坐标，使C关于B对称于A
        """
        Cx = 2 * B[0] - A[0]
        Cy = 2 * B[1] - A[1]
        return (Cx, Cy)


# ------------------ CLI / rosparam 启动部分 ------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--scan_topic", type=str, default=None)
    parser.add_argument("--threshold", type=float, default=None)
    parser.add_argument("--period", type=float, default=None)
    parser.add_argument("--goal_x", type=float, default=None)
    parser.add_argument("--goal_y", type=float, default=None)
    parser.add_argument("--scenario_file", type=str, default=None)
    args = parser.parse_args()

    rospy.init_node("monitor_and_override_node", anonymous=True)

    scan_topic = rospy.get_param("~scan_topic", args.scan_topic or "/scan")
    threshold = float(rospy.get_param("~threshold", args.threshold if args.threshold is not None else 0.3))
    period = float(rospy.get_param("~period", args.period if args.period is not None else 0.9))
    override_sub_dur = float(rospy.get_param("~override_sub_dur", 0.5))
    override_final_dur = float(rospy.get_param("~override_final_dur", 0.1))

    # final goal 来源：优先 CLI / rosparam，其次 scenario_file 中的 robot_goal 字段
    final_goal_x = rospy.get_param("~final_goal_x", args.goal_x if args.goal_x is not None else None)
    final_goal_y = rospy.get_param("~final_goal_y", args.goal_y if args.goal_y is not None else None)
    scenario_file = rospy.get_param("~scenario_file", args.scenario_file)

    if scenario_file:
        try:
            with open(os.path.expanduser(scenario_file), "r") as f:
                data = json.load(f)
            if "robot_goal" in data and isinstance(data["robot_goal"], (list, tuple)) and len(data["robot_goal"]) >= 2:
                final_goal_x, final_goal_y = float(data["robot_goal"][0]), float(data["robot_goal"][1])
                rospy.loginfo("Using robot_goal from scenario file: (%.3f, %.3f)", final_goal_x, final_goal_y)
        except Exception as e:
            rospy.logwarn("Cannot read scenario_file %s: %s", scenario_file, e)

    if final_goal_x is None or final_goal_y is None:
        rospy.logwarn("final_goal not fully specified; using default (1.73,-9.57)")
        final_goal = (1.73, -9.57)
    else:
        final_goal = (float(final_goal_x), float(final_goal_y))

    try:
        main_loop_monitor(scan_topic=scan_topic,
                          threshold=threshold,
                          period=period,
                          final_goal=final_goal,
                          override_sub_dur=override_sub_dur,
                          override_final_dur=override_final_dur)
    except rospy.ROSInterruptException:
        pass
