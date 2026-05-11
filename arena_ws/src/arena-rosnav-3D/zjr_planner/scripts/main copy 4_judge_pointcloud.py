#!/usr/bin/env python3
# monitor_and_override.py
"""
监控最短激光距离并在阈值内触发一次 pipeline -> override 行为（线程化实现）。

行为：
- 使用 /scan 订阅保存最新激光数据（不阻塞）
- 使用 rospy.Timer 以 0.2s 周期检查最短距离 —— 若 < threshold 且当前未在处理，则启动一个线程执行处理流程
- 处理流程（在独立线程中）：
    1) 读取 /odom 得到当前位姿
    2) 运行 pipeline (generate_pointcloud -> RRT -> merge -> RuleBand) 得到子目标
    3) 通过 move_base action 发送子目标并在 duration_sub 秒后 cancel（临时夺权）
    4) 发送最终目标（来自参数或 scenario 文件）并在 duration_final 秒后 cancel（释放控制权）
    5) 处理完成后，恢复 0.2s 的检测
- 采用 cancel_all_goals 来尽量清理 action server 的目标
- 在发送最终目标之前会尝试调用 /move_base/clear_costmaps（若该服务存在）

说明：
- 这里用线程（threading.Thread）而不是进程。线程开销小，且便于与 rospy 交互（rospy 在多进程时需更复杂处理）。
- 若你确实需要进程隔离（例如 pipeline 会导致 GIL 或内存冲突），可以改用 multiprocessing。现在先用线程。
"""

import os
import time
import math
import subprocess
import warnings
import argparse
import json
import threading

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
from std_srvs.srv import Empty

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"
warnings.filterwarnings("ignore", category=UserWarning, message=".*libiomp5md")


# ------------------ 辅助函数（与之前脚本兼容） ------------------

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
    rrt_script = find_rrt_script()
    if rrt_script is None:
        raise FileNotFoundError("Cannot find rrt_path_generate.py; set correct path in find_rrt_script()")
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

    # 如果你之前使用了关于某点的对称转换，可在此调整。这里保留原预测值（如需对称请自行替换）
    x_sub = float(x_sub0)
    y_sub = float(y_sub0)

    rospy.loginfo("Sampled rule-based sub-goal → (%.3f, %.3f)", x_sub, y_sub)
    return x_sub, y_sub


def get_robot_pose(timeout=5.0):
    """从 /odom 读取机器人位姿 (x,y,yaw)。"""
    try:
        msg = rospy.wait_for_message("/odom", Odometry, timeout=timeout)
    except rospy.ROSException:
        rospy.logerr("Timeout waiting for /odom (%.1fs)" % timeout)
        return None
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
    return float(x), float(y), float(yaw)


def send_action_goal_and_cancel(x, y, yaw=0.0, duration=0.5, wait_server=3.0):
    """
    使用 move_base action 发送 goal 并在 duration 秒后 cancel_all_goals（尽量保证释放控制权）。
    返回 True 表示找到 action server 并执行，False 表示 action server 不可用。
    """
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server (%.1fs)..." % wait_server)
    if not client.wait_for_server(rospy.Duration(wait_server)):
        rospy.logwarn("move_base action server not available after %.1fs" % wait_server)
        return False

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = float(x)
    goal.target_pose.pose.position.y = float(y)
    q = tft.quaternion_from_euler(0.0, 0.0, float(yaw))
    goal.target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

    rospy.loginfo("Sending action goal to move_base: x=%.3f y=%.3f yaw=%.3f" % (x, y, yaw))
    client.send_goal(goal)

    start = time.time()
    try:
        # 在 duration 时间中间隔短 sleep，避免长时间阻塞
        while time.time() - start < float(duration) and not rospy.is_shutdown():
            rospy.sleep(0.02)
    except rospy.ROSInterruptException:
        rospy.logwarn("Interrupted while waiting to cancel goal")

    rospy.loginfo("Cancelling move_base goal(s) after %.3fs" % duration)
    try:
        client.cancel_all_goals()
    except Exception as e:
        rospy.logwarn("cancel_all_goals() raised: %s" % e)

    # 给 action server / planner 一点时间稳定
    rospy.sleep(0.12)
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
    rospy.loginfo("Published fallback pose to %s: x=%.3f y=%.3f" % (topic, x, y))
    return True


# ------------------ MonitorNode 类（主逻辑） ------------------

class MonitorNode(object):
    def __init__(self,
                 scan_topic="/scan",
                 threshold=0.28,
                 check_period=0.2,
                 override_sub_dur=0.5,
                 override_final_dur=0.1,
                 final_goal=(1.73, -9.57),
                 rrt_out_paths=None):
        # params
        self.scan_topic = scan_topic
        self.threshold = float(threshold)
        self.check_period = float(check_period)
        self.override_sub_dur = float(override_sub_dur)
        self.override_final_dur = float(override_final_dur)
        self.final_goal = final_goal
        self.rrt_out_paths = rrt_out_paths

        # runtime state
        self.latest_scan = None       # store latest LaserScan.ranges
        self.latest_scan_time = 0.0
        self.processing_lock = threading.Lock()   # 用于避免并发处理
        self.timer = None
        self.sub_scan = None

        # subscribe to scan (store latest)
        self.sub_scan = rospy.Subscriber(self.scan_topic, LaserScan, self._scan_cb, queue_size=1)

        # start timer for periodic check
        self.timer = rospy.Timer(rospy.Duration(self.check_period), self._timer_cb)

        rospy.loginfo("MonitorNode initialized: scan_topic=%s threshold=%.3f check_period=%.3f final_goal=(%.3f,%.3f)",
                      self.scan_topic, self.threshold, self.check_period, self.final_goal[0], self.final_goal[1])

    def _scan_cb(self, msg: LaserScan):
        # 保存最新的 ranges（浅拷贝即可），只保留 floats
        try:
            self.latest_scan = list(msg.ranges)
            self.latest_scan_time = rospy.get_time()
        except Exception:
            self.latest_scan = None
            self.latest_scan_time = 0.0

    def _compute_min_from_latest(self):
        if not self.latest_scan:
            return None
        valid = []
        for r in self.latest_scan:
            if r is None:
                continue
            if math.isfinite(r):
                # 不知道 scan 对象的 range_min/max，这里不严格限制
                valid.append(r)
        if not valid:
            return None
        return float(min(valid))

    def _timer_cb(self, event):
        # 如果已经有线程在处理，跳过检测
        if self.processing_lock.locked():
            return

        # 读取最新最小值
        min_d = self._compute_min_from_latest()
        if min_d is None:
            # 如果没有可用扫描，直接返回
            return

        rospy.logdebug("Min scan distance: %.3f (threshold %.3f)" % (min_d, self.threshold))
        if min_d < self.threshold:
            # 触发处理（在新线程中）
            # 使用 lock 来阻止并发处理；线程在开始时会 acquire()
            t = threading.Thread(target=self._processing_worker, daemon=True)
            t.start()

    def _processing_worker(self):
        """
        在独立线程中执行 pipeline + override 流程。
        通过 processing_lock 防止重复触发。
        """
        got_lock = self.processing_lock.acquire(False)
        if not got_lock:
            # 另一个线程已经在处理
            return

        try:
            rospy.loginfo("Trigger detected: entering processing worker (pipeline + override)")

            # 1) 读取当前机器人位姿
            pose = get_robot_pose(timeout=3.0)
            if pose is None:
                rospy.logerr("Cannot read robot pose; aborting processing")
                return
            cur_x, cur_y, cur_yaw = pose
            rospy.loginfo("Current pose: x=%.3f y=%.3f yaw=%.3f" % (cur_x, cur_y, cur_yaw))

            # 2) pipeline -> 得到子目标（可能耗时）
            try:
                x_sub, y_sub = pipeline_generate_subgoal_from_pose(cur_x, cur_y, self.final_goal[0], self.final_goal[1])
            except Exception as e:
                rospy.logerr("Pipeline failed: %s" % e)
                return

            rospy.loginfo("Pipeline produced subgoal: (%.3f, %.3f)" % (x_sub, y_sub))

            # 3) 发送子目标并在 override_sub_dur 秒后取消（尝试 action）
            ok = send_action_goal_and_cancel(x_sub, y_sub, yaw=0.0, duration=self.override_sub_dur, wait_server=3.0)
            if not ok:
                rospy.logwarn("Action server unavailable; using fallback publish for subgoal")
                fallback_publish_goal_once(x_sub, y_sub, yaw=0.0)
                # 等待 override_sub_dur 的时长确保临时覆盖时间
                rospy.sleep(self.override_sub_dur)

            # 给系统短暂时间来完成 cancel/恢复传播
            rospy.sleep(0.05)

            # 尝试清理 costmap（如果服务存在）
            try:
                rospy.wait_for_service('/move_base/clear_costmaps', timeout=1.0)
                clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
                clear_costmaps()
                rospy.loginfo("Called /move_base/clear_costmaps before sending final goal")
            except Exception as e:
                rospy.logdebug("Could not call /move_base/clear_costmaps: %s" % e)

            rospy.sleep(0.08)

            # 4) 发送最终目标（短时覆盖），然后 cancel -> 释放控制权
            if self.final_goal is not None:
                fx, fy = float(self.final_goal[0]), float(self.final_goal[1])
                ok2 = send_action_goal_and_cancel(fx, fy, yaw=0.0, duration=self.override_final_dur, wait_server=3.0)
                if not ok2:
                    rospy.logwarn("Action server unavailable; using fallback publish for final goal")
                    fallback_publish_goal_once(fx, fy, yaw=0.0)
                    rospy.sleep(self.override_final_dur)
            else:
                rospy.logwarn("No final_goal provided; skipping final publication")

            rospy.loginfo("Processing worker finished; releasing control back to scenario and resuming scan checks")

        finally:
            # 释放锁，允许下次触发
            if self.processing_lock.locked():
                self.processing_lock.release()


# ------------------ CLI / rosparam 启动 ------------------

def read_final_goal_from_scenario(scenario_file):
    if not scenario_file:
        return None
    try:
        path = os.path.expanduser(scenario_file)
        with open(path, "r") as f:
            data = json.load(f)
        if isinstance(data, dict) and "robot_goal" in data:
            g = data["robot_goal"]
            if isinstance(g, (list, tuple)) and len(g) >= 2:
                return float(g[0]), float(g[1])
    except Exception as e:
        rospy.logwarn("Failed to read scenario_file '%s': %s" % (scenario_file, e))
    return None


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
    threshold = float(rospy.get_param("~threshold", args.threshold if args.threshold is not None else 0.28))
    period = float(rospy.get_param("~period", args.period if args.period is not None else 0.2))
    override_sub_dur = float(rospy.get_param("~override_sub_dur", 0.5))
    override_final_dur = float(rospy.get_param("~override_final_dur", 0.1))

    # final goal 来源：优先 CLI / rosparam，其次 scenario_file 中的 robot_goal 字段
    final_goal_x = rospy.get_param("~final_goal_x", args.goal_x if args.goal_x is not None else None)
    final_goal_y = rospy.get_param("~final_goal_y", args.goal_y if args.goal_y is not None else None)
    scenario_file = rospy.get_param("~scenario_file", args.scenario_file)

    if scenario_file:
        sgoal = read_final_goal_from_scenario(scenario_file)
        if sgoal:
            final_goal_x, final_goal_y = sgoal
            rospy.loginfo("Using robot_goal from scenario file: (%.3f, %.3f)" % (final_goal_x, final_goal_y))
        else:
            rospy.logwarn("scenario_file provided but robot_goal not found; using params/CLI for final goal")

    if final_goal_x is None or final_goal_y is None:
        rospy.logwarn("final_goal not fully specified; using default (1.73,-9.57)")
        final_goal = (1.73, -9.57)
    else:
        final_goal = (float(final_goal_x), float(final_goal_y))

    # instantiate node
    node = MonitorNode(scan_topic=scan_topic,
                       threshold=threshold,
                       check_period=period,
                       override_sub_dur=override_sub_dur,
                       override_final_dur=override_final_dur,
                       final_goal=final_goal)

    # keep alive
    rospy.spin()
