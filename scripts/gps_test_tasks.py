#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import os
import random
import sys
import time

import rospy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray


EARTH_RADIUS_M = 6378137.0
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, os.pardir))
DEFAULT_FENCE_FILE = os.path.join(WORKSPACE_DIR, "config", "gps_test_fence.json")


def yaw_from_odom(msg):
    q = msg.pose.pose.orientation
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def gps_to_xy(lat, lon, origin_lat, origin_lon):
    d_lat = math.radians(lat - origin_lat)
    d_lon = math.radians(lon - origin_lon)
    ref_lat = math.radians(origin_lat)
    x = EARTH_RADIUS_M * d_lon * math.cos(ref_lat)
    y = EARTH_RADIUS_M * d_lat
    return x, y


def xy_to_gps(x, y, origin_lat, origin_lon):
    ref_lat = math.radians(origin_lat)
    lat = origin_lat + math.degrees(y / EARTH_RADIUS_M)
    lon = origin_lon + math.degrees(x / (EARTH_RADIUS_M * math.cos(ref_lat)))
    return lat, lon


def polygon_contains(point, polygon):
    x, y = point
    inside = False
    count = len(polygon)
    if count < 3:
        return False

    j = count - 1
    for i in range(count):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        intersects = ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
        )
        if intersects:
            inside = not inside
        j = i
    return inside


class GpsTestTasks:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/gps/odom")
        self.goal_fix_topic = rospy.get_param("~goal_fix_topic", "/gps/goal_fix")
        self.cancel_topic = rospy.get_param("~cancel_topic", "/move_base/cancel")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.marker_topic = rospy.get_param("~marker_topic", "/gps/test_fence_markers")
        self.marker_frame_id = rospy.get_param("~marker_frame_id", "camera_init")
        self.fence_file = rospy.get_param(
            "~fence_file",
            DEFAULT_FENCE_FILE,
        )
        self.forward_goal_m = float(rospy.get_param("~forward_goal_m", 8.0))
        self.fence_half_size_m = float(rospy.get_param("~fence_half_size_m", 10.0))
        self.monitor_rate_hz = float(rospy.get_param("~monitor_rate_hz", 5.0))

        self.latest_odom = None
        self.fence = None
        self.last_outside_warn = 0.0

        self.goal_pub = rospy.Publisher(self.goal_fix_topic, NavSatFix, queue_size=10)
        self.cancel_pub = rospy.Publisher(self.cancel_topic, GoalID, queue_size=10)
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1, latch=True)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=20)
        self.monitor_timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.monitor_rate_hz, 0.1)),
            self.monitor_fence,
        )
        self.marker_timer = rospy.Timer(rospy.Duration(1.0), self.publish_fence_markers_timer)

        self.load_fence()

    def odom_cb(self, msg):
        self.latest_odom = msg

    def get_origin(self):
        origin_lat = rospy.get_param("/gps/origin_lat", None)
        origin_lon = rospy.get_param("/gps/origin_lon", None)
        if origin_lat is None or origin_lon is None:
            raise RuntimeError("Missing /gps/origin_lat or /gps/origin_lon. Start GPS mode first.")
        return float(origin_lat), float(origin_lon)

    def require_pose(self):
        if self.latest_odom is None:
            rospy.loginfo("Waiting for %s ...", self.odom_topic)
            self.latest_odom = rospy.wait_for_message(self.odom_topic, Odometry, timeout=10.0)
        origin_lat, origin_lon = self.get_origin()
        p = self.latest_odom.pose.pose.position
        yaw = yaw_from_odom(self.latest_odom)
        return p.x, p.y, yaw, origin_lat, origin_lon

    def local_to_navsat(self, x, y):
        origin_lat, origin_lon = self.get_origin()
        lat, lon = xy_to_gps(x, y, origin_lat, origin_lon)
        return lat, lon

    def publish_goal_xy(self, x, y, label):
        if self.fence is not None and not self.point_inside_fence(x, y):
            print("[拒绝] 目标点在电子围栏外，不发布：%s x=%.3f y=%.3f" % (label, x, y))
            return False

        lat, lon = self.local_to_navsat(x, y)
        msg = NavSatFix()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "gps"
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = 0.0
        self.goal_pub.publish(msg)
        print("[发布] %s: x=%.3f y=%.3f lat=%.12f lon=%.12f -> %s" % (
            label,
            x,
            y,
            lat,
            lon,
            self.goal_fix_topic,
        ))
        return True

    def task_forward_8m(self):
        x, y, yaw, _origin_lat, _origin_lon = self.require_pose()
        target_x = x + self.forward_goal_m * math.cos(yaw)
        target_y = y + self.forward_goal_m * math.sin(yaw)
        print(
            "[诊断] 当前 x=%.3f y=%.3f yaw=%.1fdeg；目标 x=%.3f y=%.3f；直线距离=%.3fm"
            % (
                x,
                y,
                math.degrees(yaw),
                target_x,
                target_y,
                math.hypot(target_x - x, target_y - y),
            )
        )
        self.publish_goal_xy(target_x, target_y, "当前车头正前方 %.1fm GPS 目标" % self.forward_goal_m)

    def task_save_fence(self):
        x, y, yaw, origin_lat, origin_lon = self.require_pose()
        half = self.fence_half_size_m
        fx = math.cos(yaw)
        fy = math.sin(yaw)
        lx = -math.sin(yaw)
        ly = math.cos(yaw)

        # Rectangle corners in robot heading frame: front-left, front-right, back-right, back-left.
        corners_xy = [
            (x + half * fx + half * lx, y + half * fy + half * ly),
            (x + half * fx - half * lx, y + half * fy - half * ly),
            (x - half * fx - half * lx, y - half * fy - half * ly),
            (x - half * fx + half * lx, y - half * fy + half * ly),
        ]
        named_xy = {
            "front": (x + half * fx, y + half * fy),
            "back": (x - half * fx, y - half * fy),
            "left": (x + half * lx, y + half * ly),
            "right": (x - half * lx, y - half * ly),
        }

        corners_gps = []
        for px, py in corners_xy:
            lat, lon = xy_to_gps(px, py, origin_lat, origin_lon)
            corners_gps.append({"lat": lat, "lon": lon})

        named_gps = {}
        for name, (px, py) in named_xy.items():
            lat, lon = xy_to_gps(px, py, origin_lat, origin_lon)
            named_gps[name] = {"lat": lat, "lon": lon}

        self.fence = {
            "created_unix": time.time(),
            "frame": "gps_lat_lon",
            "origin_at_create": {"lat": origin_lat, "lon": origin_lon},
            "center": {"lat": xy_to_gps(x, y, origin_lat, origin_lon)[0],
                       "lon": xy_to_gps(x, y, origin_lat, origin_lon)[1]},
            "half_size_m": half,
            "heading_yaw_rad": yaw,
            "corners": corners_gps,
            "edge_midpoints": named_gps,
        }
        self.save_fence()
        self.publish_fence_markers()
        print("[保存] 电子围栏已写入：%s" % self.fence_file)
        self.print_fence()

    def task_random_goal(self):
        x, y, yaw, _origin_lat, _origin_lon = self.require_pose()
        half = self.fence_half_size_m
        fx = math.cos(yaw)
        fy = math.sin(yaw)
        lx = -math.sin(yaw)
        ly = math.cos(yaw)

        for _attempt in range(100):
            forward = random.uniform(-half, half)
            left = random.uniform(-half, half)
            target_x = x + forward * fx + left * lx
            target_y = y + forward * fy + left * ly
            if self.fence is None or self.point_inside_fence(target_x, target_y):
                self.publish_goal_xy(
                    target_x,
                    target_y,
                    "当前车体前后左右 %.1fm 范围内随机 GPS 目标" % half,
                )
                return

        print("[失败] 随机采样 100 次都在电子围栏外，请移动到围栏内部或重设围栏。")

    def load_fence(self):
        if not os.path.exists(self.fence_file):
            return
        with open(self.fence_file, "r", encoding="utf-8") as handle:
            self.fence = json.load(handle)
        print("[加载] 已加载电子围栏：%s" % self.fence_file)
        self.publish_fence_markers()

    def save_fence(self):
        os.makedirs(os.path.dirname(self.fence_file), exist_ok=True)
        with open(self.fence_file, "w", encoding="utf-8") as handle:
            json.dump(self.fence, handle, indent=2, ensure_ascii=False, sort_keys=True)

    def fence_polygon_xy(self):
        if self.fence is None:
            return None
        origin_lat, origin_lon = self.get_origin()
        polygon = []
        for point in self.fence.get("corners", []):
            polygon.append(gps_to_xy(point["lat"], point["lon"], origin_lat, origin_lon))
        return polygon

    def point_inside_fence(self, x, y):
        polygon = self.fence_polygon_xy()
        if polygon is None:
            return True
        return polygon_contains((x, y), polygon)

    def monitor_fence(self, _event):
        if self.fence is None or self.latest_odom is None or rospy.is_shutdown():
            return
        p = self.latest_odom.pose.pose.position
        try:
            inside = self.point_inside_fence(p.x, p.y)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Fence monitor unavailable: %s", exc)
            return
        if inside:
            return

        now = time.time()
        if now - self.last_outside_warn > 1.0:
            self.last_outside_warn = now
            rospy.logerr("Robot is outside GPS test fence: x=%.3f y=%.3f. Canceling move_base.", p.x, p.y)
        self.cancel_motion()

    def make_point(self, x, y, z):
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        return point

    def make_marker(self, marker_id, marker_type, stamp):
        marker = Marker()
        marker.header.frame_id = self.marker_frame_id
        marker.header.stamp = stamp
        marker.ns = "gps_test_fence"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration(0.0)
        return marker

    def delete_fence_markers(self):
        marker = Marker()
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(MarkerArray(markers=[marker]))

    def publish_fence_markers_timer(self, _event):
        if self.fence is not None:
            self.publish_fence_markers()

    def publish_fence_markers(self):
        if self.fence is None:
            self.delete_fence_markers()
            return

        try:
            polygon = self.fence_polygon_xy()
            origin_lat, origin_lon = self.get_origin()
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "GPS fence marker unavailable: %s", exc)
            return
        if not polygon:
            self.delete_fence_markers()
            return

        stamp = rospy.Time.now()
        markers = []

        line = self.make_marker(0, Marker.LINE_STRIP, stamp)
        line.scale.x = 0.08
        line.color.r = 0.0
        line.color.g = 1.0
        line.color.b = 0.2
        line.color.a = 0.95
        for x, y in polygon + [polygon[0]]:
            line.points.append(self.make_point(x, y, 0.05))
        markers.append(line)

        edge_midpoints = self.fence.get("edge_midpoints", {})
        points = self.make_marker(1, Marker.SPHERE_LIST, stamp)
        points.scale.x = 0.35
        points.scale.y = 0.35
        points.scale.z = 0.35
        points.color.r = 1.0
        points.color.g = 0.55
        points.color.b = 0.0
        points.color.a = 0.95

        for index, name in enumerate(("front", "back", "left", "right"), 2):
            gps_point = edge_midpoints.get(name)
            if gps_point is None:
                continue
            x, y = gps_to_xy(gps_point["lat"], gps_point["lon"], origin_lat, origin_lon)
            points.points.append(self.make_point(x, y, 0.15))

            label = self.make_marker(index, Marker.TEXT_VIEW_FACING, stamp)
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.8
            label.scale.z = 0.65
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 0.95
            label.text = name
            markers.append(label)

        if points.points:
            markers.append(points)

        self.marker_pub.publish(MarkerArray(markers=markers))

    def cancel_motion(self):
        self.cancel_pub.publish(GoalID())
        stop = Twist()
        for _ in range(3):
            self.cmd_pub.publish(stop)
            rospy.sleep(0.05)

    def print_fence(self):
        if self.fence is None:
            print("[围栏] 当前没有电子围栏。输入 2 可创建并永久保存。")
            return
        print("[围栏] half_size=%.1fm corners:" % float(self.fence.get("half_size_m", 0.0)))
        for index, point in enumerate(self.fence.get("corners", []), 1):
            print("  %d. lat=%.12f lon=%.12f" % (index, point["lat"], point["lon"]))
        edge_midpoints = self.fence.get("edge_midpoints", {})
        if edge_midpoints:
            print("  front/back/left/right:")
            for name in ("front", "back", "left", "right"):
                point = edge_midpoints.get(name)
                if point is not None:
                    print("  %s: lat=%.12f lon=%.12f" % (name, point["lat"], point["lon"]))
        print("  file: %s" % self.fence_file)

    def clear_fence(self):
        self.fence = None
        if os.path.exists(self.fence_file):
            os.remove(self.fence_file)
        self.delete_fence_markers()
        print("[清除] 电子围栏已清除。")

    def menu_loop(self):
        print("")
        print("GPS 测试任务已启动。普通 GPS 导航不会自动启用这个电子围栏，只有本脚本运行时会监控。")
        print("RViz 可添加 MarkerArray 订阅：%s" % self.marker_topic)
        self.print_fence()
        while not rospy.is_shutdown():
            print("")
            print("请选择任务：")
            print("  1: 发布当前车头正前方 %.1fm 的 GPS 目标" % self.forward_goal_m)
            print("  2: 以当前位置为中心，保存前后左右 %.1fm 的永久电子围栏" % self.fence_half_size_m)
            print("  3: 在当前位置前后左右 %.1fm 范围内随机发布 GPS 目标" % self.fence_half_size_m)
            print("  4: 显示当前电子围栏")
            print("  5: 清除永久电子围栏")
            print("  q: 退出")
            choice = input("> ").strip().lower()
            try:
                if choice == "1":
                    self.task_forward_8m()
                elif choice == "2":
                    self.task_save_fence()
                elif choice == "3":
                    self.task_random_goal()
                elif choice == "4":
                    self.print_fence()
                elif choice == "5":
                    self.clear_fence()
                elif choice in ("q", "quit", "exit"):
                    return
                else:
                    print("未知输入：%s" % choice)
            except (rospy.ROSException, RuntimeError, ValueError, OSError) as exc:
                print("[错误] %s" % exc)


def main():
    rospy.init_node("gps_test_tasks", anonymous=False)
    tasks = GpsTestTasks()
    try:
        tasks.menu_loop()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    sys.exit(main())
