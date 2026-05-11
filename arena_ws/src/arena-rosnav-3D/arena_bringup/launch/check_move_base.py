#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import sys

def check_amcl_pose():
    try:
        msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=5)
        print("AMCL Pose received:")
        print("  frame_id:", msg.header.frame_id)
        print("  position: x=%.3f y=%.3f z=%.3f" %
              (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z))
        return msg.header.frame_id
    except rospy.ROSException:
        print("[ERROR] No /amcl_pose received within 5s")
        return None

def check_scan_topic():
    try:
        msg = rospy.wait_for_message('/scan', LaserScan, timeout=5)
        print("Scan topic received:")
        print("  frame_id:", msg.header.frame_id)
        print("  ranges count:", len(msg.ranges))
        return msg.header.frame_id
    except rospy.ROSException:
        print("[ERROR] No /scan received within 5s")
        return None

def check_tf_chain(listener, from_frame, to_frame):
    try:
        t = listener.getLatestCommonTime(from_frame, to_frame)
        pos, rot = listener.lookupTransform(from_frame, to_frame, t)
        print(f"TF from {from_frame} -> {to_frame} exists. Translation: {pos}, Rotation: {rot}")
        return True
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print(f"[ERROR] TF from {from_frame} -> {to_frame} NOT available")
        return False

def main():
    rospy.init_node('move_base_checker', anonymous=True)
    listener = tf.TransformListener()

    rospy.sleep(1.0)  # 等 TF 缓存

    print("=== Checking AMCL Pose ===")
    amcl_frame = check_amcl_pose()

    print("\n=== Checking Scan Topic ===")
    scan_frame = check_scan_topic()

    print("\n=== Checking TF Chains ===")
    # map -> base_footprint
    check_tf_chain(listener, 'map', 'base_footprint')
    # base_footprint -> scan_frame
    if scan_frame:
        check_tf_chain(listener, 'base_footprint', scan_frame)
    # map -> scan_frame
    if scan_frame:
        check_tf_chain(listener, 'map', scan_frame)

    print("\n=== Move Base Config Check ===")
    # 尝试读取参数
    try:
        global_frame = rospy.get_param('/move_base/global_frame_id')
        base_frame = rospy.get_param('/move_base/base_local_planner/odom_topic', 'odom')
        print("Move Base global_frame_id:", global_frame)
        print("Move Base base_frame_id (approx.):", base_frame)
    except KeyError as e:
        print("[WARN] Move Base param not found:", e)

    print("\n=== Check complete ===")
    print("If TF chains from map->base_footprint->scan_frame exist, and AMCL is in map frame, Move Base should start properly.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

