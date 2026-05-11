#!/home/robot/python_env/rosnav/bin/python3
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

import rospy
from nav_msgs.msg import Odometry
import tf.transformations as tft

# ROS services
from zjr_planner.srv import SaveScan, GenerateRRTPaths

# Pipeline modules
from pointcloud_generate import generate_pointcloud_grid
from merge_data import merge_start_and_paths
from ruleband_api import RuleBandAPI

# Action / message imports for move_base override
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"
warnings.filterwarnings("ignore", category=UserWarning, message=".*libiomp5md")

MIN_COMMIT_STEPS = 0.6         # 建议：3~5   （下限）
MAX_COMMIT_STEPS = 1.2       # 建议：15~30 （上限）
K_SIGMOID_SHAPE  = 8.0       # Sigmoid斜率，越大越敏感
K_SIGMOID_CENTER = 0.5       # Sigmoid中心位点（τ=0.5附近）

def request_save_pointcloud(index=1,
                            scan_topic="scan",
                            width=100,
                            height=100,
                            resolution=0.1,
                            timeout=10.0):
    rospy.wait_for_service('/save_pointcloud')
    try:
        save_pointcloud = rospy.ServiceProxy('/save_pointcloud', SaveScan)
        resp = save_pointcloud(index, scan_topic, width, height, resolution, timeout)
        if resp.success:
            print(f"[INFO] Saved pointcloud to: {resp.message}")
        else:
            print(f"[ERROR] Failed to save pointcloud: {resp.message}")
    except rospy.ServiceException as e:
        print(f"[ERROR] Service call failed: {e}")

def commit_steps_from_tau_scalar(tau,
                                 min_k=3,
                                 max_k=20,
                                 shape=8.0,
                                 center=0.5):
    """
    tau: float (0~1之间)
    return: float (commit steps)
    """
    s = 1 / (1 + math.exp(-shape * (center - tau)))  # sigmoid
    k = min_k + (max_k - min_k) * s
    return k

def call_rrt_service(goal_x, goal_y, num_paths=3, step=0.5, max_iters=5000):
    rospy.wait_for_service('/generate_rrt_paths')
    try:
        rrt_srv = rospy.ServiceProxy('/generate_rrt_paths', GenerateRRTPaths)
        resp = rrt_srv(goal_x, goal_y, num_paths, step, max_iters)
        if resp.success:
            print("RRT paths saved to:", resp.filename)
            # print("JSON data:", resp.json)
        else:
            print("RRT path generation failed.")
    except rospy.ServiceException as e:
        print("Service call failed:", e)


def pipeline_generate_subgoal_from_pose(start_x, start_y, goal_x, goal_y):
    """
    执行点云保存 -> RRT -> merge -> RuleBand，返回 (x_sub, y_sub)
    """
    rospy.loginfo("Pipeline: saving pointcloud...")

    # Step 1: 调用保存点云服务
    request_save_pointcloud()
    pc_file = os.path.expanduser(
        "~/catkin_arena/src/zjr_planner/scripts/data_json/data_start_1.json"
    )

    # Step 2: 调用 RRT 服务
    try:
        call_rrt_service(goal_x, goal_y)
    except Exception as e:
        rospy.logerr("RRT generation failed: %s", e)
        raise

    rrt_file = os.path.expanduser(
        "~/catkin_arena/src/zjr_planner/scripts/data_json/paths.json"
    )

    # Step 3: 等待两个文件生成
    rospy.loginfo("Waiting for pointcloud and RRT path JSON files to be ready...")
    timeout_sec = 30.0   # 最长等待时间，可调整
    check_interval = 0.2 # 每0.2秒检查一次
    start_time = time.time()

    while True:
        pc_exists = os.path.exists(pc_file)
        rrt_exists = os.path.exists(rrt_file)

        if pc_exists and rrt_exists:
            rospy.loginfo("Both JSON files found, proceeding to merge...")
            break

        if time.time() - start_time > timeout_sec:
            missing = []
            if not pc_exists:
                missing.append("data_start_1.json")
            if not rrt_exists:
                missing.append("paths.json")
            rospy.logerr(f"Timeout waiting for files: {', '.join(missing)}")
            raise TimeoutError(f"Timeout: {', '.join(missing)} not found within {timeout_sec}s")

        # 打印状态信息
        rospy.logdebug(f"Waiting... (pc:{pc_exists}, rrt:{rrt_exists})")
        time.sleep(check_interval)

    # Step 4: 合并数据
    merged_out = os.path.expanduser(
        "~/catkin_arena/src/zjr_planner/scripts/data_json/example_data.json"
    )
    try:
        merged_file = merge_start_and_paths(
            (float(start_x), float(start_y)), out_path=merged_out
        )
    except Exception as e:
        rospy.logerr("merge_start_and_paths failed: %s", e)
        raise

    # Step 5: 调用 RuleBand 推理模块
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


def fallback_publish_goal_once(x, y, yaw=0.0, topic="/move_base_simple/goal", wait_conn=0.9):
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


def main_loop(period=0.9, final_goal_x=None, final_goal_y=None, one_shot=False):
    """
    主循环：每 period 秒运行一次（包含 pipeline 时间），每次:
      - 调用 pipeline -> 得到 x_sub,y_sub
      - send_action_goal_and_cancel(x_sub,y_sub,duration=0.5)
      - send_action_goal_and_cancel(final_goal_x,final_goal_y,duration=0.3)
    """
    rospy.loginfo("Starting main loop with period %.3fs", period)
    pose = get_robot_pose(timeout=5.0)
    cur_x0, cur_y0, cur_yaw0 = pose
    sub_x, sub_y, sub_yaw = pose
    distance_total = math.hypot(final_goal_x - cur_x0, final_goal_y - cur_y0)
    end_time=time.time()
    while not rospy.is_shutdown():
        loop_start = time.time()
        pose = get_robot_pose(timeout=5.0)
        if pose is None:
            rospy.logwarn("No /odom pose; skipping this iteration")
        else:
            cur_x, cur_y, cur_yaw = pose
            x_sub, y_sub = pipeline_generate_subgoal_from_pose(cur_x, cur_y, final_goal_x, final_goal_y)
            rospy.loginfo("Current robot pose: x=%.3f y=%.3f yaw=%.3f", cur_x, cur_y, cur_yaw)

            # 1) pipeline -> get subgoal
            threshold = 2.0
            if math.hypot(cur_x - x_sub, cur_y - y_sub) < threshold or end_time-loop_start > 2.0:
                print("接近目标点，更新sub_xy")
                try:
                    x_sub, y_sub = pipeline_generate_subgoal_from_pose(cur_x, cur_y, final_goal_x, final_goal_y)
                    end_time=time.time()
                except Exception as e:
                    rospy.logerr("Pipeline failed: %s", e)
                    x_sub = None

            if x_sub is not None:
                rospy.loginfo("Generated subgoal: (%.3f, %.3f)", x_sub, y_sub)
                # 2) override by sending subgoal (duration 0.5s)
                ok = send_action_goal_and_cancel(x_sub, y_sub, yaw=0.0, duration=0.4, wait_server=2.0)
                if not ok:
                    rospy.logwarn("Action server unavailable; fallback publish for subgoal")
                    fallback_publish_goal_once(x_sub, y_sub, yaw=0.0)

                # # 3) immediately send final goal (duration 0.3s)
                # if final_goal_x is not None and final_goal_y is not None:
                #     ok2 = send_action_goal_and_cancel(final_goal_x, final_goal_y, yaw=0.0, duration=0.3, wait_server=2.0)
                #     if not ok2:
                #         rospy.logwarn("Action server unavailable; fallback publish for final goal")
                #         fallback_publish_goal_once(final_goal_x, final_goal_y, yaw=0.0)
                # else:
                #     rospy.logwarn("Final goal not provided; skipping final-goal publish.")

        # 控制周期为 period 秒（减去本次循环实际耗时）
        distance_cur = math.hypot(final_goal_x - cur_x, final_goal_y - cur_y)
        tau=(distance_total - distance_cur)/distance_total
        period = commit_steps_from_tau_scalar(tau)
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
    parser.add_argument("--period", type=float, default=0.9, help="loop period in seconds (default 0.9)")
    parser.add_argument("--one_shot", action="store_true", help="run exactly one iteration then exit")
    parser.add_argument("--goal_x", type=float, default=None, help="final goal x (override)")
    parser.add_argument("--goal_y", type=float, default=None, help="final goal y (override)")
    parser.add_argument("--scenario_file", type=str, default=None, help="optional scenario json file to read robot_goal from")
    args, unknown = parser.parse_known_args()

    # init rospy and read params (allow rosparam to override)
    rospy.init_node("generate_and_override_goal_args", anonymous=True)
    try:
        # 等待 /goal 话题发布一次消息
        msg = rospy.wait_for_message("/goal", PoseStamped, timeout=5.0)
        final_goal_x = msg.pose.position.x
        final_goal_y = msg.pose.position.y
        rospy.loginfo("Received final goal from /goal: x=%.3f y=%.3f", final_goal_x, final_goal_y)
    except rospy.ROSException:
        rospy.logwarn("No /goal message received, using default values")
        final_goal_x = 1.73
        final_goal_y = -9.57
    scenario_file = rospy.get_param("~scenario_file", args.scenario_file)

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
