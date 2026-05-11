#!/usr/bin/env python3
# monitor_and_override_with_start_to_rrt.py
"""
Updated monitor_and_override: pass main-thread start pose to RRT via --start

Changes:
 - run_rrt_once_no_ros now accepts optional start_x/start_y and appends
   --start <x> <y> to the rrt_path_generate.py command line.
 - pipeline_generate_subgoal_no_ros passes the start coordinates it
   received into run_rrt_once_no_ros so RRT will use the same start.

Note: This requires a small change in rrt_path_generate.py to accept
an optional --start argument (see chat notes for the suggested patch).
"""

import os
import time
import math
import subprocess
import warnings
import argparse
import json
import threading
import multiprocessing

import rospy
from nav_msgs.msg import Odometry
import tf.transformations as tft

# pipeline modules (确保这些模块在你的环境可 import)
# 注意：子进程也需要这些模块可 import（且运行环境与主进程相同或兼容）
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


# ------------------ 子进程用的 pipeline（不依赖 rospy，使用 print） ------------------
# 这些函数在子进程里被 import/调用，不能使用 rospy.loginfo 等 ROS API。


def find_rrt_script_no_ros():
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


def run_rrt_once_no_ros(goal_x, goal_y, out_file=None, num_paths=3, step=0.3, max_iters=4000, start_x=None, start_y=None):
    """
    Launch rrt_path_generate.py in a subprocess.
    If start_x/start_y provided, append: --start <start_x> <start_y>
    """
    rrt_script = find_rrt_script_no_ros()
    if rrt_script is None:
        raise FileNotFoundError("Cannot find rrt_path_generate.py; set correct path in find_rrt_script_no_ros()")
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
    # add start if provided (this ensures RRT uses the same start as main thread)
    if start_x is not None and start_y is not None:
        cmd.extend(["--start", str(float(start_x)), str(float(start_y))])
    if out_file:
        cmd.extend(["--out", out_file])
    # 使用 print 而不是 rospy.loginfo
    print("[pipeline-child] Running RRT:", " ".join(cmd))
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.stdout and res.stdout.strip():
        print("[pipeline-child] RRT stdout:", res.stdout.strip())
    if res.stderr and res.stderr.strip():
        print("[pipeline-child] RRT stderr:", res.stderr.strip())
    if res.returncode != 0:
        raise RuntimeError("RRT script failed (rc=%d)" % res.returncode)
    return out_file


def pipeline_generate_subgoal_no_ros(start_x, start_y, goal_x, goal_y):
    """
    子进程内执行：点云 -> RRT -> merge -> RuleBand -> 返回 (x_sub, y_sub)
    使用 print 打日志，不依赖 rospy。
    """
    print("[pipeline-child] Saving pointcloud...")
    saved_file = generate_pointcloud_grid(index=5)
    print("[pipeline-child] Pointcloud saved to:", saved_file)

    out_paths = os.path.expanduser("~/catkin_arena/src/zjr_planner/RuleBand_API-main/data_json/paths.json")
    # pass start_x/start_y so RRT will use the same start pose
    run_rrt_once_no_ros(goal_x, goal_y, out_file=out_paths, start_x=start_x, start_y=start_y)

    merged_out = os.path.expanduser("~/catkin_arena/src/zjr_planner/RuleBand_API-main/data_json/example_data.json")
    merged_file = merge_start_and_paths((float(start_x), float(start_y)), out_path=merged_out)
    print("[pipeline-child] Saved merged file to:", merged_file)

    api = RuleBandAPI(device="cpu")
    x_sub0, y_sub0 = api.predict_from_file(merged_file, debug=True)

    x_sub = float(x_sub0)
    y_sub = float(y_sub0)

    print("[pipeline-child] Sampled subgoal -> (%.3f, %.3f)" % (x_sub, y_sub))
    return x_sub, y_sub


def pipeline_child_entry(start_x, start_y, goal_x, goal_y, q: multiprocessing.Queue):
    """
    子进程入口：执行 pipeline_generate_subgoal_no_ros 并把结果放入队列 q。
    q.put(("ok", x_sub, y_sub)) 或 q.put(("err", "message"))
    """
    try:
        x, y = pipeline_generate_subgoal_no_ros(start_x, start_y, goal_x, goal_y)
        q.put(("ok", float(x), float(y)))
    except Exception as e:
        import traceback
        tb = traceback.format_exc()
        # 把错误信息回传
        q.put(("err", str(e) + "\n" + tb))


# ------------------ 主进程中的 ROS 交互函数 ------------------


def get_robot_pose(timeout=5.0):
    """从 /odom 读取机器人位姿 (x,y,yaw)。超时返回 None。"""
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
        while time.time() - start < float(duration) and not rospy.is_shutdown():
            rospy.sleep(0.02)
    except rospy.ROSInterruptException:
        rospy.logwarn("Interrupted while waiting to cancel goal")

    rospy.loginfo("Cancelling move_base goal(s) after %.3fs" % duration)
    try:
        client.cancel_all_goals()
    except Exception as e:
        rospy.logwarn("cancel_all_goals() raised: %s" % e)

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


# ------------------ MonitorNode（主逻辑） ------------------

class MonitorNode(object):
    def __init__(self,
                 check_period=0.2,
                 dist_thresh=0.4,
                 time_thresh=1.0,
                 override_sub_dur=0.5,
                 final_goal=(6.0, 6.0),
                 pipeline_timeout=30.0):
        # parameters
        self.check_period = float(check_period)
        self.dist_thresh = float(dist_thresh)
        self.time_thresh = float(time_thresh)
        self.override_sub_dur = float(override_sub_dur)
        self.final_goal = final_goal
        self.pipeline_timeout = float(pipeline_timeout)

        # runtime state
        self.current_target = (float(self.final_goal[0]), float(self.final_goal[1]))
        self.target_assigned_time = rospy.get_time()
        self.processing_lock = threading.Lock()
        self.timer = None

        # latest robot pose from /odom
        self.latest_pose = None   # (x,y,yaw)
        self.latest_pose_time = 0.0

        # subscribe odom to keep pose up-to-date
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self._odom_cb, queue_size=1)

        # start periodic timer
        self.timer = rospy.Timer(rospy.Duration(self.check_period), self._timer_cb)

        rospy.loginfo("MonitorNode initialized: check_period=%.3fs dist_thresh=%.3fm time_thresh=%.3fs final_goal=(%.3f,%.3f) pipeline_timeout=%.1fs",
                      self.check_period, self.dist_thresh, self.time_thresh, self.final_goal[0], self.final_goal[1], self.pipeline_timeout)

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose
        q = p.orientation
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.latest_pose = (float(p.position.x), float(p.position.y), float(yaw))
        self.latest_pose_time = rospy.get_time()

    def _distance_to_target(self, pose, target):
        dx = pose[0] - target[0]
        dy = pose[1] - target[1]
        return math.hypot(dx, dy)

    def _timer_cb(self, event):
        if self.processing_lock.locked():
            return

        if not self.latest_pose:
            rospy.logdebug("No odom yet; skipping check")
            return

        dist = self._distance_to_target(self.latest_pose, self.current_target)
        elapsed = rospy.get_time() - self.target_assigned_time if self.target_assigned_time else float("inf")

        rospy.logdebug("Check target dist=%.3f (thresh=%.3f), elapsed=%.3f (thresh=%.3f)",
                       dist, self.dist_thresh, elapsed, self.time_thresh)

        trigger = False
        if dist <= self.dist_thresh:
            rospy.loginfo("Within distance threshold: dist=%.3f <= %.3f -> trigger processing", dist, self.dist_thresh)
            trigger = True
        elif elapsed >= self.time_thresh:
            rospy.loginfo("Target elapsed time exceeded: elapsed=%.3f >= %.3f -> trigger processing", elapsed, self.time_thresh)
            trigger = True

        if not trigger:
            return

        got = self.processing_lock.acquire(False)
        if not got:
            return

        self.target_assigned_time = 0.0
        t = threading.Thread(target=self._processing_worker, args=(), daemon=True)
        t.start()

    def _processing_worker(self):
        try:
            rospy.loginfo("Processing worker started (launch pipeline in child process).")

            if self.latest_pose:
                cur_x, cur_y, cur_yaw = self.latest_pose
            else:
                pose = get_robot_pose(timeout=3.0)
                if pose is None:
                    rospy.logerr("Cannot read odom for processing; aborting.")
                    return
                cur_x, cur_y, cur_yaw = pose

            rospy.loginfo("Current pose: x=%.3f y=%.3f yaw=%.3f" % (cur_x, cur_y, cur_yaw))

            ctx = multiprocessing.get_context('spawn')
            q = ctx.Queue()
            p = ctx.Process(target=pipeline_child_entry,
                            args=(cur_x, cur_y, self.final_goal[0], self.final_goal[1], q))
            p.start()

            rospy.loginfo("Pipeline child process started (pid=%s), waiting up to %.1fs..." % (str(p.pid), self.pipeline_timeout))

            got_result = False
            x_sub = y_sub = None
            try:
                start_wait = time.time()
                timeout = float(self.pipeline_timeout)
                while time.time() - start_wait < timeout:
                    try:
                        if not q.empty():
                            tup = q.get_nowait()
                            if tup and isinstance(tup, (list, tuple)) and len(tup) >= 1:
                                tag = tup[0]
                                if tag == "ok":
                                    _, x_sub, y_sub = tup
                                    got_result = True
                                    break
                                else:
                                    rospy.logerr("Pipeline child reported error: %s" % (tup[1] if len(tup) > 1 else "<no msg>"))
                                    break
                        time.sleep(0.1)
                    except Exception:
                        time.sleep(0.05)
                if not got_result:
                    try:
                        if not q.empty():
                            tup = q.get_nowait()
                            if tup and tup[0] == "ok":
                                _, x_sub, y_sub = tup
                                got_result = True
                    except Exception:
                        pass
            finally:
                if p.is_alive():
                    if got_result:
                        p.join(timeout=1.0)
                        if p.is_alive():
                            try:
                                p.terminate()
                            except Exception:
                                pass
                            p.join(timeout=1.0)
                    else:
                        rospy.logwarn("Pipeline child did not return in time (%.1fs). Terminating child." % self.pipeline_timeout)
                        try:
                            p.terminate()
                        except Exception as e:
                            rospy.logwarn("Error terminating pipeline child: %s" % e)
                        p.join(timeout=1.0)

            if not got_result or x_sub is None or y_sub is None:
                rospy.logerr("Pipeline failed or timed out; aborting processing worker.")
                return

            rospy.loginfo("Pipeline produced subgoal: (%.3f, %.3f)" % (x_sub, y_sub))

            ok = send_action_goal_and_cancel(x_sub, y_sub, yaw=0.0, duration=self.override_sub_dur, wait_server=3.0)
            if not ok:
                rospy.logwarn("Action server unavailable; fallback publish for subgoal.")
                fallback_publish_goal_once(x_sub, y_sub, yaw=0.0)
                rospy.sleep(self.override_sub_dur)

            rospy.sleep(0.05)

            try:
                rospy.wait_for_service('/move_base/clear_costmaps', timeout=1.0)
                clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
                clear_costmaps()
                rospy.loginfo("Called /move_base/clear_costmaps after override.")
            except Exception:
                pass

            self.current_target = (float(x_sub), float(y_sub))
            self.target_assigned_time = rospy.get_time()
            rospy.loginfo("Updated current_target to (%.3f, %.3f). Reset target_assigned_time." % (x_sub, y_sub))

            rospy.loginfo("Processing worker finished successfully.")

        finally:
            if self.processing_lock.locked():
                self.processing_lock.release()


# ------------------ scenario util ------------------


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


# ------------------ main / CLI ------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--check_period", type=float, default=None, help="period (s) to check distance (default 0.2)")
    parser.add_argument("--dist_thresh", type=float, default=None, help="distance threshold (m), default 0.4")
    parser.add_argument("--time_thresh", type=float, default=None, help="time threshold (s) default 1.0")
    parser.add_argument("--override_sub_dur", type=float, default=None, help="seconds to hold override subgoal (default 0.5)")
    parser.add_argument("--pipeline_timeout", type=float, default=None, help="max seconds to wait for pipeline child (default 30)")
    parser.add_argument("--goal_x", type=float, default=None, help="final goal x override")
    parser.add_argument("--goal_y", type=float, default=None, help="final goal y override")
    parser.add_argument("--scenario_file", type=str, default=None, help="optional scenario json file")
    args = parser.parse_args()

    rospy.init_node("monitor_and_override_node", anonymous=True)

    check_period = float(rospy.get_param("~check_period", args.check_period if args.check_period is not None else 0.2))
    dist_thresh = float(rospy.get_param("~dist_thresh", args.dist_thresh if args.dist_thresh is not None else 0.4))
    time_thresh = float(rospy.get_param("~time_thresh", args.time_thresh if args.time_thresh is not None else 1.0))
    override_sub_dur = float(rospy.get_param("~override_sub_dur", args.override_sub_dur if args.override_sub_dur is not None else 0.5))
    pipeline_timeout = float(rospy.get_param("~pipeline_timeout", args.pipeline_timeout if args.pipeline_timeout is not None else 30.0))

    final_goal_x = rospy.get_param("~final_goal_x", args.goal_x if args.goal_x is not None else None)
    final_goal_y = rospy.get_param("~final_goal_y", args.goal_y if args.goal_y is not None else None)
    scenario_file = rospy.get_param("~scenario_file", args.scenario_file if args.scenario_file is not None else None)

    if scenario_file:
        sgoal = read_final_goal_from_scenario(scenario_file)
        if sgoal:
            final_goal_x, final_goal_y = sgoal
            rospy.loginfo("Using robot_goal from scenario file: (%.3f, %.3f)" % (final_goal_x, final_goal_y))
        else:
            rospy.logwarn("scenario_file provided but robot_goal not found; using params/CLI for final goal")

    if final_goal_x is None or final_goal_y is None:
        rospy.logwarn("final_goal not fully specified; using default (1.73,-9.57)")
        final_goal = (6.0, 6.0)
    else:
        final_goal = (float(final_goal_x), float(final_goal_y))

    node = MonitorNode(check_period=check_period,
                       dist_thresh=dist_thresh,
                       time_thresh=time_thresh,
                       override_sub_dur=override_sub_dur,
                       final_goal=final_goal,
                       pipeline_timeout=pipeline_timeout)

    rospy.spin()
