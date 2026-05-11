#!/usr/bin/env python3
"""
generate_and_override_goal.py

每 0.9s 循环一次：
  - 读取 /odom 作为当前起点
  - 运行 pipeline -> RuleBand -> 得到子目标 (x_sub,y_sub)
  - 使用 move_base action 发送子目标并在 0.5s 后 cancel（临时覆盖 scenario）
  - 紧接发送最终目标（来自外部输入，如 scenario.json 的 robot_goal）并在 0.3s 后 cancel
如果 action server 不可用，回退到一次性的 /move_base_simple/goal 发布。
"""
import os
import glob
import subprocess
import warnings
import time
import math
import argparse
import json
import torch

import rospy
from nav_msgs.msg import Odometry
import tf.transformations as tft

# pipeline modules (ensure these are importable in your environment)
from pointcloud_generate import generate_pointcloud_grid
from merge_data import merge_start_and_paths
from ruleband_api import RuleBandAPI

# action / message imports for override
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"
warnings.filterwarnings("ignore", category=UserWarning, message=".*libiomp5md")

MIN_COMMIT_STEPS = 3
MAX_COMMIT_STEPS = 20
K_SIGMOID_SHAPE = 8.0
K_SIGMOID_CENTER = 0.5 

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
        raise FileNotFoundError("Cannot find rrt_path_generate.py; please set correct path in find_rrt_script()")

    if out_file:
        out_file = os.path.expanduser(out_file)
        os.makedirs(os.path.dirname(out_file), exist_ok=True)

    cmd = [
        "python3",
        rrt_script,
        "--goal", str(float(goal_x)), str(float(goal_y)),
        "--num_paths", str(int(num_paths)),
        "--step", str(float(step)),
        "--max_iters", str(int(max_iters))
    ]
    if out_file:
        cmd.extend(["--out", out_file])

    rospy.loginfo("Running RRT: %s", " ".join(cmd))
    res = subprocess.run(cmd, capture_output=True, text=True)
    rospy.loginfo("RRT stdout:\n%s", res.stdout.strip() or "<empty>")
    if res.stderr:
        rospy.logwarn("RRT stderr:\n%s", res.stderr.strip())
    if res.returncode != 0:
        raise RuntimeError(f"RRT script failed (rc={res.returncode})")
    return out_file


def pipeline_generate_subgoal_from_pose(start_x, start_y, goal_x, goal_y):
    """
    执行点云保存 -> RRT -> merge -> RuleBand，返回 (x_sub, y_sub)
    (会写入 RuleBand_API-main/data_json/*.json)
    """
    rospy.loginfo("Pipeline: saving pointcloud...")
    saved_file = generate_pointcloud_grid(index=5)
    rospy.loginfo("Pointcloud saved to: %s", saved_file)

    out_paths = os.path.expanduser("~/catkin_arena/src/zjr_planner/RuleBand_API-main/data_json/paths.json")
    try:
        run_rrt_once(goal_x, goal_y, out_file=out_paths)
    except Exception as e:
        rospy.logerr("RRT generation failed: %s", e)
        raise

    merged_out = os.path.expanduser("~/catkin_arena/src/zjr_planner/RuleBand_API-main/data_json/example_data.json")
    try:
        merged_file = merge_start_and_paths((float(start_x), float(start_y)), out_path=merged_out)
    except Exception as e:
        rospy.logerr("merge_start_and_paths failed: %s", e)
        raise
    rospy.loginfo("Saved merged file to: %s", merged_file)

    api = RuleBandAPI(device="cpu")
    try:
        x_sub, y_sub = api.predict_from_file(merged_file, debug=True)
    except Exception as e:
        rospy.logerr("RuleBand prediction failed: %s", e)
        raise

    rospy.loginfo("Sampled rule-based sub-goal → (%.3f, %.3f)" % (x_sub, y_sub))
    return float(x_sub), float(y_sub)


def get_robot_pose(timeout=5.0):
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


def send_action_goal_and_cancel(x, y, yaw=0.0, duration=0.5, wait_server=5.0):
    """
    使用 move_base action 发送 goal 并在 duration 秒后 cancel（干净地释放控制权）。
    返回 True 表示使用 action 成功（即 action server 可用并已 send/cancel），False 表示 action server 不可用。
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

    # 在 duration 内保持 goal（但不阻塞太长），然后取消
    start = time.time()
    try:
        while time.time() - start < float(duration) and not rospy.is_shutdown():
            rospy.sleep(0.02)
    except rospy.ROSInterruptException:
        rospy.logwarn("Interrupted while waiting to cancel goal")

    rospy.loginfo("Cancelling move_base goal after %.3fs", duration)
    client.cancel_goal()
    rospy.sleep(0.1)
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

def commit_steps_from_tau(tau_tensor, 
                          min_k=MIN_COMMIT_STEPS, 
                          max_k=MAX_COMMIT_STEPS,
                          shape=K_SIGMOID_SHAPE, 
                          center=K_SIGMOID_CENTER):
    """
    tau_tensor: torch.Tensor, shape (B,) or scalar
    return: torch.Tensor of commit steps (float型)
    """
    tau_tensor = tau_tensor.float()
    s = torch.sigmoid(shape * (center - tau_tensor))
    k = min_k + (max_k - min_k) * s
    return k

def read_final_goal_from_scenario(scenario_file):
    """
    如果 scenario_file 指定并存在，尝试读取 JSON 并返回 robot_goal [x,y]。
    返回 None 表示无法读取或字段不存在。
    """
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
        rospy.logwarn("Failed to read scenario_file '%s': %s", scenario_file, e)
    return None


def main_loop(period=1.2, final_goal_x=None, final_goal_y=None, one_shot=False):
    """
    主循环：每 period 秒运行一次（包含 pipeline 时间），每次:
      - 调用 pipeline -> 得到 x_sub,y_sub
      - send_action_goal_and_cancel(x_sub,y_sub,duration=0.5)
      - send_action_goal_and_cancel(final_goal_x,final_goal_y,duration=0.3)
    """
    rospy.loginfo("Starting main loop with period %.3fs", period)
    while not rospy.is_shutdown():
        loop_start = time.time()

        pose = get_robot_pose(timeout=5.0)
        if pose is None:
            rospy.logwarn("No /odom pose; skipping this iteration")
        else:
            cur_x, cur_y, cur_yaw = pose
            rospy.loginfo("Current robot pose: x=%.3f y=%.3f yaw=%.3f", cur_x, cur_y, cur_yaw)

            # 1) pipeline -> get subgoal
            try:
                x_sub, y_sub = pipeline_generate_subgoal_from_pose(cur_x, cur_y, final_goal_x, final_goal_y)
            except Exception as e:
                rospy.logerr("Pipeline failed: %s", e)
                x_sub = None

            if x_sub is not None:
                rospy.loginfo("Generated subgoal: (%.3f, %.3f)", x_sub, y_sub)
                # 2) override by sending subgoal (duration 0.5s)
                ok = send_action_goal_and_cancel(x_sub, y_sub, yaw=0.0, duration=0.5, wait_server=2.0)
                if not ok:
                    rospy.logwarn("Action server unavailable; fallback publish for subgoal")
                    fallback_publish_goal_once(x_sub, y_sub, yaw=0.0)

                # 3) immediately send final goal (duration 0.2s)
                if final_goal_x is not None and final_goal_y is not None:
                    ok2 = send_action_goal_and_cancel(final_goal_x, final_goal_y, yaw=0.0, duration=0.2, wait_server=2.0)
                    if not ok2:
                        rospy.logwarn("Action server unavailable; fallback publish for final goal")
                        fallback_publish_goal_once(final_goal_x, final_goal_y, yaw=0.0)
                else:
                    rospy.logwarn("Final goal not provided; skipping final-goal publish.")

        # 控制周期为 period 秒，与任务进度tau有关（减去本次循环实际耗时）
        elapsed = time.time() - loop_start
        to_sleep = period - elapsed
        if to_sleep > 0:
            rospy.sleep(to_sleep)
        else:
            # pipeline 或 action 超时导致本轮超时，直接进入下一轮（不 sleep）
            rospy.logwarn("Loop overran desired period by %.3fs", -to_sleep)

        if one_shot:
            rospy.loginfo("one_shot True -> exiting main loop after one iteration")
            return


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--period", type=float, default=1.2, help="loop period in seconds (default 0.9)")
    parser.add_argument("--one_shot", action="store_true", help="run exactly one iteration then exit")
    parser.add_argument("--goal_x", type=float, default=None, help="final goal x (override)")
    parser.add_argument("--goal_y", type=float, default=None, help="final goal y (override)")
    parser.add_argument("--scenario_file", type=str, default=None, help="optional scenario json file to read robot_goal from")
    args = parser.parse_args()

    # init rospy and read params (allow rosparam to override)
    rospy.init_node("generate_and_override_goal_args", anonymous=True)

    final_goal_x = rospy.get_param("~final_goal_x", 1.73)
    final_goal_y = rospy.get_param("~final_goal_y", -9.57)
    scenario_file = rospy.get_param("~scenario_file", args.scenario_file)

    # CLI overrides
    if args.goal_x is not None:
        final_goal_x = args.goal_x
    if args.goal_y is not None:
        final_goal_y = args.goal_y

    # If scenario_file provided, prefer robot_goal from that JSON
    if scenario_file:
        sgoal = read_final_goal_from_scenario(scenario_file)
        if sgoal:
            final_goal_x, final_goal_y = sgoal
            rospy.loginfo("Using robot_goal from scenario file %s -> (%.3f, %.3f)", scenario_file, final_goal_x, final_goal_y)
        else:
            rospy.logwarn("scenario_file provided but robot_goal not found; using params/CLI for final goal")

    try:
        main_loop(period=args.period, final_goal_x=final_goal_x, final_goal_y=final_goal_y, one_shot=args.one_shot)
    except rospy.ROSInterruptException:
        pass
