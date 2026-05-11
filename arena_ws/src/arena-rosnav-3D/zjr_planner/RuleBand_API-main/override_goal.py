#!/usr/bin/env python3
"""
override_goal.py

短时间覆盖 scenario 的导航目标：把用户提供的目标以给定频率连续发布到
/move_base_simple/goal 持续 `duration` 秒，然后停止——从而在此时间窗口内
导航栈使用用户目标，窗口结束后恢复由 scenario 发布的目标（如果有的话）。

用法（命令行）:
  rosrun <your_pkg> override_goal.py --x 1.73 --y -9.57 --yaw 0.0 --duration 0.5

也可以在另一个 rospy 节点中导入并调用 temporary_override_goal(...)
"""

import rospy
from geometry_msgs.msg import PoseStamped
import tf.transformations as tft
import time
import argparse

def _make_goal_msg(x, y, yaw, frame_id="map"):
    """构造 PoseStamped 消息"""
    goal = PoseStamped()
    goal.header.frame_id = frame_id
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = float(x)
    goal.pose.position.y = float(y)
    goal.pose.position.z = 0.0
    q = tft.quaternion_from_euler(0.0, 0.0, float(yaw))
    goal.pose.orientation.x = q[0]
    goal.pose.orientation.y = q[1]
    goal.pose.orientation.z = q[2]
    goal.pose.orientation.w = q[3]
    return goal

def temporary_override_goal(x, y, yaw=0.0, duration=0.5, rate_hz=10.0,
                            topic="/move_base_simple/goal", frame_id="map",
                            wait_for_subscribers_timeout=2.0, quiet=False):
    """
    临时覆盖目标：在 duration 秒内以 rate_hz 频率重复发布目标到 topic。
    参数:
      x,y,yaw         - 目标位姿 (yaw 单位: 弧度)
      duration        - 覆盖时间（秒），建议 0.5
      rate_hz         - 发布频率 (Hz)，建议 5~20。频率越高覆盖效果越稳。
      topic           - 目标话题，默认 /move_base_simple/goal
      frame_id        - 目标坐标系，默认 "map"
      wait_for_subscribers_timeout - 等待发布器连接超时（秒）
      quiet           - 若为 True 则抑制日志输出
    返回:
      True 如果成功完成发布（发布过程无异常）
      False 如果在等待 publisher 连接阶段超时或发生异常
    """
    # 如果 rospy 未初始化则临时初始化（脚本直接运行时通常需要）
    if not rospy.core.is_initialized():
        rospy.init_node("override_goal_node_temp", anonymous=True)

    pub = rospy.Publisher(topic, PoseStamped, queue_size=1, latch=False)

    # 等待 publisher 被消费者（move_base）连接（可避免丢消息）
    start_wait = time.time()
    while pub.get_num_connections() == 0:
        if rospy.is_shutdown():
            if not quiet:
                rospy.logwarn("ROS shutdown requested while waiting for subscribers.")
            return False
        if (time.time() - start_wait) > wait_for_subscribers_timeout:
            # 仍然没连接，仍然继续（可能 move_base 通过 action 接收），但警告
            if not quiet:
                rospy.logwarn("No subscribers to %s after %.2fs; continuing to publish anyway.",
                              topic, wait_for_subscribers_timeout)
            break
        if not quiet:
            rospy.loginfo("Waiting for subscribers to %s ...", topic)
        rospy.sleep(0.05)

    rate = rospy.Rate(rate_hz)
    end_time = time.time() + float(duration)
    sent = 0
    try:
        while time.time() < end_time and not rospy.is_shutdown():
            msg = _make_goal_msg(x, y, yaw, frame_id=frame_id)
            pub.publish(msg)
            sent += 1
            # 小日志
            if not quiet and (sent % max(1, int(rate_hz/2)) == 0):
                rospy.loginfo("override_goal: published %d times, still overriding until %.3fs",
                              sent, end_time - time.time())
            rate.sleep()
    except rospy.ROSInterruptException:
        if not quiet:
            rospy.loginfo("override_goal: interrupted")
        return False
    except Exception as e:
        if not quiet:
            rospy.logerr("override_goal: exception while publishing: %s", e)
        return False

    if not quiet:
        rospy.loginfo("override_goal: finished publishing %d messages for %.3fs; releasing override",
                      sent, duration)
    return True

# Command-line interface
def _parse_args():
    p = argparse.ArgumentParser(description="Temporarily override scenario goal by repeatedly publishing to /move_base_simple/goal")
    p.add_argument("--x", type=float, required=True, help="goal x")
    p.add_argument("--y", type=float, required=True, help="goal y")
    p.add_argument("--yaw", type=float, default=0.0, help="goal yaw in radians")
    p.add_argument("--duration", type=float, default=0.5, help="override duration in seconds (default 0.5)")
    p.add_argument("--rate", type=float, default=10.0, help="publish rate in Hz (default 10)")
    p.add_argument("--topic", type=str, default="/move_base_simple/goal", help="topic to publish goal to")
    p.add_argument("--frame_id", type=str, default="map", help="goal frame_id (default 'map')")
    p.add_argument("--quiet", action="store_true", help="suppress info logs")
    return p.parse_args()

if __name__ == "__main__":
    args = _parse_args()
    # init node here if not already initialized
    if not rospy.core.is_initialized():
        rospy.init_node("override_goal_cli", anonymous=True)

    ok = temporary_override_goal(
        x=args.x, y=args.y, yaw=args.yaw,
        duration=args.duration, rate_hz=args.rate,
        topic=args.topic, frame_id=args.frame_id,
        quiet=args.quiet
    )
    if not ok:
        rospy.logwarn("override_goal: ended with warning/failure")
    else:
        rospy.loginfo("override_goal: success")
