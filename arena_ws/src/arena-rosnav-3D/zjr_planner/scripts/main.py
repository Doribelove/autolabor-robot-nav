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
import cmd
import time
import sys
import math
import subprocess
import warnings
import argparse
import json
import threading
import multiprocessing
# from queue import Empty
# from multiprocessing import Empty


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
from std_msgs.msg import Float32, Int32, String
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from std_srvs.srv import Empty as EmptySrv   # if you need the service type; otherwise, remove it
import queue as pyqueue                      # use pyqueue.Empty in except

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"
warnings.filterwarnings("ignore", category=UserWarning, message=".*libiomp5md")


# ------------------ 子进程用的 pipeline（不依赖 rospy，使用 print） ------------------
# 这些函数在子进程里被 import/调用，不能使用 rospy.loginfo 等 ROS API。


def find_rrt_script_no_ros():
    candidates = [
        "~/arena_ws/src/arena-rosnav-3D/zjr_planner/RuleBand_API-main/rrt_path_generate.py",
        "/home/robot/arena_ws/src/arena-rosnav-3D/zjr_planner/RuleBand_API-main/rrt_path_generate.py",
        "./rrt_path_generate.py",
    ]
    for p in candidates:
        p_exp = os.path.expanduser(p)
        if os.path.exists(p_exp):
            return os.path.abspath(p_exp)
    return None


def run_rrt_once_no_ros_old(goal_x, goal_y, out_file=None, num_paths=3, step=0.3, max_iters=4000, start_x=None, start_y=None):
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
    # print("[pipeline-child] Running RRT:", " ".join(cmd))
    res = subprocess.run(cmd, capture_output=True, text=True)
    # if res.stdout and res.stdout.strip():
        # print("[pipeline-child] RRT stdout:", res.stdout.strip())
    # if res.stderr and res.stderr.strip():
        # print("[pipeline-child] RRT stderr:", res.stderr.strip())
    if res.returncode != 0:
        raise RuntimeError("RRT script failed (rc=%d)" % res.returncode)
    return out_file

def run_rrt_once_no_ros(goal_x, goal_y, out_file=None, num_paths=3, step=0.3, max_iters=4000, start_x=None, start_y=None):
    rrt_script = find_rrt_script_no_ros()
    if rrt_script is None:
        raise FileNotFoundError("Cannot find rrt_path_generate.py; set correct path in find_rrt_script_no_ros()")
    if out_file:
        out_file = os.path.expanduser(out_file)
        os.makedirs(os.path.dirname(out_file), exist_ok=True)

    base_cmd = [
        "python3", rrt_script,
        "--goal", str(float(goal_x)), str(float(goal_y)),
        "--num_paths", str(int(num_paths)),
        "--step", str(float(step)),
        "--max_iters", str(int(max_iters))
    ]
    if out_file:
        base_cmd += ["--out", out_file]

    def run(cmd):
        # print("[pipeline-child] Running RRT:", " ".join(cmd))
        res = subprocess.run(cmd, capture_output=True, text=True)
        # if res.stdout.strip():
            # print("[pipeline-child] RRT stdout:", res.stdout.strip())
        # if res.stderr.strip():
            # print("[pipeline-child] RRT stderr:", res.stderr.strip())
        return res

    # Try with --start first (if provided)
    if start_x is not None and start_y is not None:
        cmd = base_cmd + ["--start", str(float(start_x)), str(float(start_y))]
        res = run(cmd)
        # If argparse rejected --start, retry without it
        if res.returncode != 0 and "unrecognized arguments: --start" in (res.stderr or ""):
            # print("[pipeline-child] RRT doesn't support --start; retrying without it.")
            res = run(base_cmd)
    else:
        res = run(base_cmd)

    if res.returncode != 0:
        raise RuntimeError("RRT script failed (rc=%d)" % res.returncode)
    return out_file


def pipeline_generate_subgoal_no_ros(start_x, start_y, goal_x, goal_y):
    """
    子进程内执行：点云 -> RRT -> merge -> RuleBand -> 返回 (x_sub, y_sub)
    使用 print 打日志，不依赖 rospy。
    """
    # print("[pipeline-child] Saving pointcloud...")
    saved_file = generate_pointcloud_grid(index=5)
    # print("[pipeline-child] Pointcloud saved to:", saved_file)

    out_paths = os.path.expanduser("~/arena_ws/src/arena-rosnav-3D/zjr_planner/RuleBand_API-main/data_json/paths.json")
    # pass start_x/start_y so RRT will use the same start pose
    run_rrt_once_no_ros(goal_x, goal_y, out_file=out_paths, start_x=start_x, start_y=start_y)

    merged_out = os.path.expanduser("~/arena_ws/src/arena-rosnav-3D/zjr_planner/RuleBand_API-main/data_json/example_data.json")
    merged_file = merge_start_and_paths((float(start_x), float(start_y)), out_path=merged_out)
    # print("[pipeline-child] Saved merged file to:", merged_file)

    api = RuleBandAPI(device="cpu")
    res = api.predict_from_file(merged_file, debug=True)

    # tolerate both (x,y) and (x,y,eri,band)
    if isinstance(res, (list, tuple)):
        x_sub0, y_sub0 = res[0], res[1]
        eri_rule       = res[2] if len(res) > 2 else None
        band_idx       = res[3] if len(res) > 3 else None
        eri_nn         = res[4] if len(res) > 4 else None      # NEW
        features_vec   = res[5] if len(res) > 5 else None
        acted_by       = res[6] if len(res) > 6 else None
        eri_act        = res[7] if len(res) > 7 else None
    else:
        x_sub0, y_sub0 = res
        eri_rule, band_idx, eri_nn, features_vec, acted_by, eri_act = None, None, None, None

    x_sub = float(x_sub0)
    y_sub = float(y_sub0)

    # print("[pipeline-child] Sampled subgoal -> (%.3f, %.3f)" % (x_sub, y_sub))
    return x_sub, y_sub, eri_rule, band_idx, eri_nn, features_vec, acted_by, eri_act


def pipeline_child_entry(start_x, start_y, goal_x, goal_y, q: multiprocessing.Queue):
    """
    子进程入口：执行 pipeline_generate_subgoal_no_ros 并把结果放入队列 q。
    q.put(("ok", x_sub, y_sub)) 或 q.put(("err", "message"))
    """

    try:
        # print("[pipeline-child] sys.argv seen by child:", sys.argv)
        sys.argv = [sys.argv[0]]          # (optional but belt-and-suspenders)
        rospy.init_node(
            "sderi_pipeline_child",
            anonymous=True,
            disable_signals=True,
            argv=[]                       # <— critical: ignore remaps
        )
        # rospy.loginfo("Parent ROS node up as: %s", rospy.get_name())
        # rospy.loginfo("Child ROS node up as: %s", rospy.get_name())
        rospy.sleep(0.1)
        x, y, eri_rule, band_idx, eri_nn, features_vec, acted_by, eri_act = pipeline_generate_subgoal_no_ros(start_x, start_y, goal_x, goal_y)
        q.put(("ok", float(x), float(y), eri_rule, band_idx, eri_nn, features_vec, acted_by, eri_act))
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
    # rospy.loginfo("Waiting for move_base action server (%.1fs)..." % wait_server)
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

    # rospy.loginfo("Sending action goal to move_base: x=%.3f y=%.3f yaw=%.3f" % (x, y, yaw))
    client.send_goal(goal)

    start = time.time()
    try:
        while time.time() - start < float(duration) and not rospy.is_shutdown():
            rospy.sleep(0.02)
    except rospy.ROSInterruptException:
        rospy.logwarn("Interrupted while waiting to cancel goal")

    # rospy.loginfo("Cancelling move_base goal(s) after %.3fs" % duration)
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
    # rospy.loginfo("Published fallback pose to %s: x=%.3f y=%.3f" % (topic, x, y))
    return True


# ------------------ MonitorNode（主逻辑） ------------------

class MonitorNode(object):
    def __init__(self,
                 check_period=0.2,
                 dist_thresh=0.4,
                 time_thresh=1.0,
                 override_sub_dur=0.5,
                 final_goal=(6.0, 6.0),
                 pipeline_timeout=30.0,
                 dry_run=True):
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

        # existing fields...
        self.dry_run = bool(dry_run)
        # debug pubs
        self.pub_eri        = rospy.Publisher("/sderi/eri_debug", Float32, queue_size=10)
        self.pub_eri_rule   = rospy.Publisher("/sderi/eri_rule_debug", Float32, queue_size=10)
        self.pub_sub        = rospy.Publisher("/sderi/subgoal_debug", PoseStamped, queue_size=10)
        self.pub_band       = rospy.Publisher("/sderi/band_idx_debug", Int32, queue_size=10)
        self.pub_diag       = rospy.Publisher("/sderi/diag", String, queue_size=10)
        self.pub_feat       = rospy.Publisher("/sderi/features_debug", Float32MultiArray, queue_size=10)
        self.pub_t_replan   = rospy.Publisher("/sderi/t_replan", Float32, queue_size=10)
        self.pub_Hk         = rospy.Publisher("/sderi/H_k", Int32, queue_size=10)
        self.pub_choice     = rospy.Publisher("/sderi/choice", String, queue_size=200)
        self.pub_goal       = rospy.Publisher("/sderi/final_goal", PoseStamped, queue_size=1, latch=True)

        self._publish_final_goal_once()

        # for replanning time
        self.t_min    = float(rospy.get_param("~t_min",   0.30))  # seconds
        self.t_max    = float(rospy.get_param("~t_max",   1.20))  # seconds
        self.gamma    = float(rospy.get_param("~gamma",   2.0))   # s(z)=z^gamma
        self.control_dt = float(rospy.get_param("~control_dt", 0.05))  # 20Hz micro-loop

        self.stop_evt = threading.Event()
        rospy.on_shutdown(self._on_shutdown)   # register clean shutdown handler
        self.worker_th = None
        self.child_proc = None                 # keep a handle where you create the child


        # rospy.loginfo("MonitorNode initialized: check_period=%.3fs dist_thresh=%.3fm time_thresh=%.3fs final_goal=(%.3f,%.3f) pipeline_timeout=%.1fs",
                      # self.check_period, self.dist_thresh, self.time_thresh, self.final_goal[0], self.final_goal[1], self.pipeline_timeout)

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose
        q = p.orientation
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.latest_pose = (float(p.position.x), float(p.position.y), float(yaw))
        self.latest_pose_time = rospy.get_time()

    def _publish_final_goal_once(self):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(self.final_goal[0])
        msg.pose.position.y = float(self.final_goal[1])
        msg.pose.orientation.w = 1.0
        self.pub_goal.publish(msg)

    def _distance_to_target(self, pose, target):
        dx = pose[0] - target[0]
        dy = pose[1] - target[1]
        return math.hypot(dx, dy)

    def _timer_cb(self, event):
        # do nothing if shutting down
        if self.stop_evt.is_set() or rospy.is_shutdown():
            return

        if self.processing_lock.locked():
            return

        if not self.latest_pose:
            rospy.logdebug("No odom yet; skipping check")
            return

        dist = self._distance_to_target(self.latest_pose, self.current_target)

        # SAFE elapsed computation (avoid inf/NaN on restarts)
        t0 = self.target_assigned_time
        now = rospy.get_time()
        if t0 is None or not math.isfinite(t0):
            elapsed = 0.0
        else:
            elapsed = max(0.0, now - t0)

        rospy.logdebug("Check target dist=%.3f (thresh=%.3f), elapsed=%.3f (thresh=%.3f)",
                    dist, self.dist_thresh, elapsed, self.time_thresh)

        trigger = False
        if dist <= self.dist_thresh:
            # rospy.loginfo("Within distance threshold: dist=%.3f <= %.3f -> trigger processing", dist, self.dist_thresh)
            trigger = True
        elif elapsed >= self.time_thresh:
            # rospy.loginfo("Target elapsed time exceeded: elapsed=%.3f >= %.3f -> trigger processing", elapsed, self.time_thresh)
            trigger = True

        if not trigger:
            return

        got = self.processing_lock.acquire(False)
        if not got:
            return

        self.target_assigned_time = rospy.get_time()   # throttle next trigger correctly

        # non-daemon thread; it will exit quickly and we can join on shutdown
        t = threading.Thread(target=self._processing_worker, args=())
        t.daemon = False
        t.start()
        self.worker_th = t


    def _processing_worker(self):
        try:  # Outer try block (covers entire worker logic)
            # ADD: early exit if shutting down
            if self.stop_evt.is_set() or rospy.is_shutdown():
                return
        
            # rospy.loginfo("Processing worker started (launch pipeline in child process).")

            # read curent pose
            if self.latest_pose:
                cur_x, cur_y, cur_yaw = self.latest_pose
            else:
                pose = get_robot_pose(timeout=3.0)
                if pose is None:
                    rospy.logerr("Cannot read odom for processing; aborting.")
                    return
                cur_x, cur_y, cur_yaw = pose

            # rospy.loginfo("Current pose: x=%.3f y=%.3f yaw=%.3f" % (cur_x, cur_y, cur_yaw))

            # launch child pipeline
            ctx = multiprocessing.get_context('spawn')
            q = ctx.Queue()
            # ADD: keep a handle on the child process
            self.child_proc = ctx.Process(target=pipeline_child_entry,
                                        args=(cur_x, cur_y, self.final_goal[0], self.final_goal[1], q),
                                        daemon=False)
            try:
                self.child_proc.start()
            except Exception as e:
                rospy.logerr("Failed to start pipeline child: %s", e)
                self.child_proc = None
                return

            # rospy.loginfo("Pipeline child process started (pid=%s), waiting up to %.1fs..." % (str(self.child_proc.pid), self.pipeline_timeout))

            # wait for result (reliable): block in small time slices
            got_result = False
            x_sub = y_sub = None
            eri_rule = None
            eri_nn   = None
            band_idx = None
            features_vec = None
            acted_by = None
            eri_act = None

            try:  # <-- this pairs with the finally below
                deadline = time.time() + float(self.pipeline_timeout)
                while (time.time() < deadline) and not self.stop_evt.is_set() and not rospy.is_shutdown():
                    remaining = max(0.0, deadline - time.time())
                    try:
                        tup = q.get(timeout=min(0.25, remaining))
                    except pyqueue.Empty:
                        continue
                    except Exception:
                        # transient queue errors—keep waiting
                        continue

                    if not tup:
                        continue

                    tag = tup[0]
                    if tag == "ok":
                        x_sub = float(tup[1]); y_sub = float(tup[2])
                        eri_rule = tup[3] if len(tup) > 3 else None
                        band_idx = tup[4] if len(tup) > 4 else None
                        eri_nn   = tup[5] if len(tup) > 5 else None
                        features_vec = tup[6] if len(tup) > 6 else None
                        acted_by   = tup[7] if len(tup) > 7 else None
                        eri_act  = tup[8] if len(tup) > 8 else None
                        got_result = True
                        break
                    else:
                        rospy.logerr("Pipeline child reported error: %s",
                                    (tup[1] if len(tup) > 1 else "<no msg>"))
                        break

            finally:  # Inner finally (child cleanup only)
                # ----- SAFE CHILD CLEANUP (None tolerant) -----
                proc = self.child_proc
                self.child_proc = None  # clear the handle first to avoid races

                try:
                    if proc is not None:
                        if got_result:
                            # we got a result — try to join, then terminate if it's still hanging
                            try:
                                proc.join(timeout=1.0)
                            except Exception:
                                pass
                            if proc.is_alive():
                                try:
                                    proc.terminate()
                                except Exception:
                                    pass
                                try:
                                    proc.join(timeout=1.0)
                                except Exception:
                                    pass
                        else:
                            rospy.logwarn("Pipeline child did not return in time (%.1fs). Terminating child.",
                                        self.pipeline_timeout)
                            try:
                                proc.terminate()
                            except Exception as e:
                                rospy.logwarn("Error terminating pipeline child: %s", e)
                            try:
                                proc.join(timeout=1.0)
                            except Exception:
                                pass
                except Exception as e:
                    rospy.logwarn("Child cleanup error: %s", e)

            # --- AFTER the inner `finally:` (child cleanup) ---

            # If we didn't get a result, just bail out gracefully
            if not got_result:
                rospy.logwarn("No subgoal produced (timeout or error).")
                return

            # 1) Publish diagnostics
            try:
                if eri_nn is not None:
                    self.pub_eri.publish(Float32(float(eri_nn)))            # NN
                if eri_rule is not None:
                    self.pub_eri_rule.publish(Float32(float(eri_rule)))     # Teacher

                if band_idx is not None:
                    self.pub_band.publish(Int32(int(band_idx)))
                # 2) Publish features vector
                if features_vec is not None:
                    # normalize to a flat list of float32
                    try:
                        arr = list(features_vec)  # handles plain lists
                    except TypeError:
                        import numpy as np
                        arr = np.asarray(features_vec).flatten().tolist()

                    msg = Float32MultiArray()
                    # optional: describe layout for tooling/rqt consumers
                    dim = MultiArrayDimension(label="features", size=len(arr), stride=len(arr))
                    msg.layout.dim = [dim]
                    msg.layout.data_offset = 0
                    msg.data = [float(v) for v in arr]  # ensure numeric

                    self.pub_feat.publish(msg)

                sub_msg = PoseStamped()
                sub_msg.header.frame_id = "map"
                sub_msg.header.stamp = rospy.Time.now()
                qz = tft.quaternion_from_euler(0.0, 0.0, 0.0)
                sub_msg.pose = Pose(Point(float(x_sub), float(y_sub), 0.0),
                                    Quaternion(qz[0], qz[1], qz[2], qz[3]))
                self.pub_sub.publish(sub_msg)

                self.pub_diag.publish(String(data="subgoal=(%.3f, %.3f) eri=%s band=%s" %
                                             (x_sub, y_sub,
                                              ("%.3f" % eri_rule) if isinstance(eri_rule, (int, float)) else str(eri_rule),
                                              str(band_idx))))
                payload = {
                        "tau": float(features_vec[0]) if features_vec else None,
                        "rho": float(features_vec[1]) if features_vec else None,
                        "eri_rule": float(eri_rule) if eri_rule is not None else None,
                        "eri_nn": float(eri_nn) if eri_nn is not None else None,
                        "eri_act": float(eri_act) if eri_act is not None else (
                            float(eri_rule) if (eri_nn is None) else float(eri_nn)
                        ),
                        "band": int(band_idx) if band_idx is not None else None,
                        "acted_by": acted_by if acted_by else ("teacher" if eri_nn is None else "student")
                    }
                self.pub_choice.publish(String(data=json.dumps(payload)))

            except Exception as e:
                rospy.logwarn("Publishing diagnostics failed: %s", e)

            # 2) Update current target & reset timer so the next check uses this subgoal
            self.current_target = (float(x_sub), float(y_sub))
            self.target_assigned_time = rospy.get_time()

            # 3) Compute ERI -> t_replan & H_k ONCE, always publish (dry-run or not)
            try:
                # prefer NN; fallback to rule
                if (eri_nn is not None) and math.isfinite(eri_nn):
                    xi = float(eri_nn)
                elif (eri_rule is not None) and math.isfinite(eri_rule):
                    xi = float(eri_rule)
                else:
                    xi = 0.0  # ultra-safe fallback

                # s(1 - xi) with gamma >= 1
                s = (1.0 - xi) ** self.gamma

                # continuous replan time, clipped
                t_replan = self.t_min + (self.t_max - self.t_min) * s
                t_replan = max(self.t_min, min(self.t_max, t_replan))

                # discrete micro-loop horizon (for visibility/logs)
                H_min = int(math.ceil(self.t_min / self.control_dt))
                H_max = int(math.floor(self.t_max / self.control_dt))
                H_k   = max(H_min, min(H_max, int(math.floor(t_replan / self.control_dt))))

                # ALWAYS publish these so rqt_plot can see them even in dry-run
                self.pub_diag.publish(String(f"[t_replan] ERI={xi:.3f}  t={t_replan:.2f}s  Hk={H_k}"))
                self.pub_t_replan.publish(Float32(t_replan))
                self.pub_Hk.publish(Int32(H_k))

            except Exception as e:
                rospy.logwarn("t_replan/H_k compute-publish failed: %s", e)
                # keep a safe default so the branch below can still run
                t_replan = self.t_min

            # 4) Command behavior: ONLY the act of sending the goal depends on dry_run
            try:
                if not self.dry_run:
                    ok = send_action_goal_and_cancel(
                        x_sub, y_sub,
                        yaw=0.0,
                        duration=t_replan,   # <— USE the computed t_replan
                        wait_server=3.0
                    )
                    if not ok:
                        # fallback publish to /move_base_simple/goal if action server not ready
                        fallback_publish_goal_once(
                            x_sub, y_sub, yaw=0.0,
                            topic="/move_base_simple/goal",
                            wait_conn=1.0
                        )
                else:
                    # Dry-run: preview only (still see /sderi/t_replan thanks to step 3)
                    fallback_publish_goal_once(
                        x_sub, y_sub, yaw=0.0,
                        topic="/sderi/override_goal_preview",
                        wait_conn=0.5
                    )
            except Exception as e:
                rospy.logwarn("Override/preview dispatch failed: %s", e)

        finally:  # Outer finally (ensures lock release)
            # ----- RELEASE REENTRANCY LOCK -----
            if self.processing_lock.locked():
                self.processing_lock.release()


    def _on_shutdown(self):
        # Signal all threads to stop producing work/logs
        try:
            self.stop_evt.set()
        except Exception:
            pass

        # Stop timer (optional; ROS timers stop on shutdown anyway)
        try:
            if self.timer is not None:
                self.timer.shutdown()
        except Exception:
            pass

        # Terminate child process if still alive
        try:
            if self.child_proc is not None and self.child_proc.is_alive():
                self.child_proc.terminate()
                self.child_proc.join(timeout=2.0)
        except Exception:
            pass
        finally:
            self.child_proc = None

        # Join the background worker so it can't log after finalization
        try:
            if self.worker_th is not None and self.worker_th.is_alive():
                self.worker_th.join(timeout=2.0)
        except Exception:
            pass


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
    # parser = argparse.ArgumentParser(allow_abbrev=False)
    args, unknown = parser.parse_known_args()
    if unknown:
        print("[main.py] Ignored unknown ROS args:", unknown)

    # rospy.init_node("monitor_and_override_node", anonymous=False)
    rospy.init_node("sderi_monitor", anonymous=True)
    # rospy.loginfo("Parent ROS node up as: %s", rospy.get_name())

    check_period = float(rospy.get_param("~check_period", args.check_period if args.check_period is not None else 0.2))
    dist_thresh = float(rospy.get_param("~dist_thresh", args.dist_thresh if args.dist_thresh is not None else 0.4))
    time_thresh = float(rospy.get_param("~time_thresh", args.time_thresh if args.time_thresh is not None else 1.0))
    override_sub_dur = float(rospy.get_param("~override_sub_dur", args.override_sub_dur if args.override_sub_dur is not None else 0.5))
    pipeline_timeout = float(rospy.get_param("~pipeline_timeout", args.pipeline_timeout if args.pipeline_timeout is not None else 30.0))
    dry_run = bool(rospy.get_param("~dry_run", True))

    final_goal_x = rospy.get_param("~final_goal_x", args.goal_x if args.goal_x is not None else None)
    final_goal_y = rospy.get_param("~final_goal_y", args.goal_y if args.goal_y is not None else None)
    # scenario_file = rospy.get_param("~scenario_file", args.scenario_file if args.scenario_file is not None else None)
    scenario_file = rospy.get_param("~scenario_file", rospy.get_param("/sderi/scenario_file", ""))

    if scenario_file:
        sgoal = read_final_goal_from_scenario(scenario_file)
        if sgoal:
            final_goal_x, final_goal_y = sgoal
            rospy.loginfo("Using robot_goal from scenario file: (%.3f, %.3f)" % (final_goal_x, final_goal_y))
        else:
            rospy.logwarn("scenario_file provided but robot_goal not found; using params/CLI for final goal")

    if final_goal_x is None or final_goal_y is None:
        rospy.logwarn("final_goal not fully specified; using default (6.0,6.0)")
        final_goal = (6.0, 6.0)
    else:
        final_goal = (float(final_goal_x), float(final_goal_y))

    node = MonitorNode(check_period=check_period,
                       dist_thresh=dist_thresh,
                       time_thresh=time_thresh,
                       override_sub_dur=override_sub_dur,
                       final_goal=final_goal,
                       pipeline_timeout=pipeline_timeout,
                       dry_run=dry_run)

    rospy.spin()
