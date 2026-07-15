#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import datetime
import json
import math
import os
import random
import sys
import threading
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
DEFAULT_FOD_RESULTS_DIR = os.path.join(WORKSPACE_DIR, "test_results")
FOD_JSONL_FILENAME = "fod_final_test_records.jsonl"
FOD_CSV_FILENAME = "fod_final_test_records.csv"
FOD_EFFICIENCY_THRESHOLD = 50.0

FOD_RUN_CASES = {
    "T02": {
        "name": "基础回收测试",
        "location": "车辆前方",
        "obstacles": 0,
        "instruction": "在车辆前方放置 1 个 FOD，由检测车识别并发送位置。",
    },
    "T03-1": {
        "name": "机坪处置效率测试",
        "location": "机坪中心",
        "obstacles": 0,
        "instruction": "车辆从机坪固定起点和朝向出发，FOD 放在中心区域。",
    },
    "T03-2": {
        "name": "机坪处置效率测试",
        "location": "机坪左上",
        "obstacles": 0,
        "instruction": "车辆回到相同起点和朝向，FOD 放在左上角区域。",
    },
    "T03-3": {
        "name": "机坪处置效率测试",
        "location": "机坪右下",
        "obstacles": 0,
        "instruction": "车辆回到相同起点和朝向，FOD 放在右下角区域。",
    },
    "T04-1": {
        "name": "滑行道处置效率测试",
        "location": "滑行道近端",
        "obstacles": 0,
        "instruction": "车辆从滑行道近端固定位置出发，FOD 放在近端。",
    },
    "T04-2": {
        "name": "滑行道处置效率测试",
        "location": "滑行道中端",
        "obstacles": 0,
        "instruction": "车辆回到滑行道近端固定位置，FOD 放在中端。",
    },
    "T04-3": {
        "name": "滑行道处置效率测试",
        "location": "滑行道远端",
        "obstacles": 0,
        "instruction": "车辆回到滑行道近端固定位置，FOD 放在远端。",
    },
    "T05": {
        "name": "单个固定障碍物测试",
        "location": "单障碍物路径",
        "obstacles": 1,
        "instruction": "在车辆与 FOD 的直线路径上放置 1 个固定障碍物。",
    },
    "T06": {
        "name": "多个固定障碍物测试",
        "location": "多障碍物路径",
        "obstacles": "2-3",
        "instruction": "在路径上放置 2～3 个固定障碍物，并保留车辆可通行空间。",
    },
}

FOD_CSV_FIELDS = [
    "recorded_at",
    "record_type",
    "task_id",
    "task_name",
    "location",
    "target_lat",
    "target_lon",
    "obstacle_count",
    "area_m2",
    "duration_s",
    "autonomous_arrival",
    "obstacle_avoidance",
    "cleaning_started",
    "fod_recovered",
    "collision",
    "boundary_violation",
    "manual_takeover",
    "vehicle_stopped",
    "efficiency_m2_s",
    "passed",
    "notes",
]


def yaw_from_odom(msg):
    q = msg.pose.pose.orientation
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def iso_time(unix_time):
    return datetime.datetime.fromtimestamp(unix_time).astimezone().isoformat(timespec="seconds")


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
        self.fod_results_dir = rospy.get_param("~fod_results_dir", DEFAULT_FOD_RESULTS_DIR)
        self.fod_goal_wait_timeout = float(rospy.get_param("~fod_goal_wait_timeout", 300.0))
        self.gps_stability_duration = float(rospy.get_param("~gps_stability_duration", 120.0))
        self.default_test_area_m2 = float(rospy.get_param("~default_test_area_m2", 1000.0))

        self.latest_odom = None
        self.odom_sequence = 0
        self.fence = None
        self.last_outside_warn = 0.0
        self.goal_condition = threading.Condition()
        self.goal_sequence = 0
        self.latest_observed_goal = None

        self.goal_pub = rospy.Publisher(self.goal_fix_topic, NavSatFix, queue_size=10)
        self.cancel_pub = rospy.Publisher(self.cancel_topic, GoalID, queue_size=10)
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1, latch=True)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=20)
        self.goal_observer_sub = rospy.Subscriber(
            self.goal_fix_topic,
            NavSatFix,
            self.goal_observed_cb,
            queue_size=20,
        )
        self.monitor_timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.monitor_rate_hz, 0.1)),
            self.monitor_fence,
        )
        self.marker_timer = rospy.Timer(rospy.Duration(1.0), self.publish_fence_markers_timer)

        self.load_fence()

    def odom_cb(self, msg):
        self.latest_odom = msg
        self.odom_sequence += 1

    def pose_snapshot(self):
        if self.latest_odom is None:
            return None
        pose = self.latest_odom.pose.pose
        return {
            "x": pose.position.x,
            "y": pose.position.y,
            "yaw_rad": yaw_from_odom(self.latest_odom),
        }

    def goal_observed_cb(self, msg):
        received_unix = time.time()
        observed = {
            "latitude": float(msg.latitude),
            "longitude": float(msg.longitude),
            "received_unix": received_unix,
            "received_at": iso_time(received_unix),
            "start_pose": self.pose_snapshot(),
        }
        with self.goal_condition:
            self.goal_sequence += 1
            observed["sequence"] = self.goal_sequence
            self.latest_observed_goal = observed
            self.goal_condition.notify_all()

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

    @property
    def fod_jsonl_file(self):
        return os.path.join(self.fod_results_dir, FOD_JSONL_FILENAME)

    @property
    def fod_csv_file(self):
        return os.path.join(self.fod_results_dir, FOD_CSV_FILENAME)

    def ask_float(self, prompt, default, minimum=0.0):
        while not rospy.is_shutdown():
            raw = input("%s [默认 %.3f]: " % (prompt, default)).strip()
            if not raw:
                value = float(default)
            else:
                try:
                    value = float(raw)
                except ValueError:
                    print("请输入有效数字。")
                    continue
            if value < minimum:
                print("输入值不能小于 %.3f。" % minimum)
                continue
            return value
        raise RuntimeError("ROS is shutting down")

    def ask_yes_no(self, prompt):
        while not rospy.is_shutdown():
            raw = input("%s [y/n]: " % prompt).strip().lower()
            if raw in ("y", "yes", "1", "是", "通过"):
                return True
            if raw in ("n", "no", "0", "否", "失败"):
                return False
            print("请输入 y 或 n。")
        raise RuntimeError("ROS is shutting down")

    def ask_pass_fail_skip(self, prompt):
        while not rospy.is_shutdown():
            raw = input("%s [p=通过/f=失败/s=跳过]: " % prompt).strip().lower()
            if raw in ("p", "pass", "y", "yes", "通过"):
                return "pass"
            if raw in ("f", "fail", "n", "no", "失败"):
                return "fail"
            if raw in ("s", "skip", "跳过"):
                return "skip"
            print("请输入 p、f 或 s。")
        raise RuntimeError("ROS is shutting down")

    def save_fod_record(self, record):
        os.makedirs(self.fod_results_dir, exist_ok=True)
        record = dict(record)
        record.setdefault("recorded_at", iso_time(time.time()))

        with open(self.fod_jsonl_file, "a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n")

        goal = record.get("goal") or {}
        row = {
            "recorded_at": record.get("recorded_at", ""),
            "record_type": record.get("record_type", ""),
            "task_id": record.get("task_id", ""),
            "task_name": record.get("task_name", ""),
            "location": record.get("location", ""),
            "target_lat": goal.get("latitude", ""),
            "target_lon": goal.get("longitude", ""),
            "obstacle_count": record.get("obstacle_count", ""),
            "area_m2": record.get("area_m2", ""),
            "duration_s": record.get("duration_s", ""),
            "autonomous_arrival": record.get("autonomous_arrival", ""),
            "obstacle_avoidance": record.get("obstacle_avoidance", ""),
            "cleaning_started": record.get("cleaning_started", ""),
            "fod_recovered": record.get("fod_recovered", ""),
            "collision": record.get("collision", ""),
            "boundary_violation": record.get("boundary_violation", ""),
            "manual_takeover": record.get("manual_takeover", ""),
            "vehicle_stopped": record.get("vehicle_stopped", ""),
            "efficiency_m2_s": record.get("efficiency_m2_s", ""),
            "passed": record.get("passed", ""),
            "notes": record.get("notes", ""),
        }
        csv_exists = os.path.exists(self.fod_csv_file) and os.path.getsize(self.fod_csv_file) > 0
        with open(self.fod_csv_file, "a", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=FOD_CSV_FIELDS)
            if not csv_exists:
                writer.writeheader()
            writer.writerow(row)

        print("[记录] JSONL: %s" % self.fod_jsonl_file)
        print("[记录] CSV:   %s" % self.fod_csv_file)
        return record

    def load_fod_records(self):
        if not os.path.exists(self.fod_jsonl_file):
            return []
        records = []
        with open(self.fod_jsonl_file, "r", encoding="utf-8") as handle:
            for line_number, line in enumerate(handle, 1):
                line = line.strip()
                if not line:
                    continue
                try:
                    records.append(json.loads(line))
                except json.JSONDecodeError as exc:
                    print("[警告] 跳过损坏记录第 %d 行：%s" % (line_number, exc))
        return records

    def published_topic_status(self):
        published = {name for name, _type_name in rospy.get_published_topics(namespace="/")}
        required = (
            "/canbus_msg",
            "/gps/fix",
            "/gps/odom",
            "/gps/heading",
            "/scan",
            "/move_base/status",
        )
        return {topic: topic in published for topic in required}

    def task_t01_device_and_gps_check(self):
        print("\n========== T01 设备与 GPS 检查 ==========")
        topic_status = self.published_topic_status()
        for topic, available in topic_status.items():
            print("  %s %s" % ("[正常]" if available else "[缺失]", topic))

        self.require_pose()
        duration = self.ask_float(
            "GPS 静稳采样时长（秒）",
            self.gps_stability_duration,
            minimum=1.0,
        )
        print("车辆应保持静止，开始采样 %.1f 秒。" % duration)
        samples = []
        last_sequence = -1
        started_monotonic = time.monotonic()
        deadline = started_monotonic + duration
        next_progress = started_monotonic

        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.latest_odom is not None and self.odom_sequence != last_sequence:
                last_sequence = self.odom_sequence
                snapshot = self.pose_snapshot()
                if snapshot is not None:
                    samples.append(snapshot)
            now = time.monotonic()
            if now >= next_progress:
                remaining = max(0.0, deadline - now)
                print("[采样] 剩余 %.0f 秒，已收集 %d 帧" % (remaining, len(samples)))
                next_progress = now + 10.0
            rospy.sleep(0.05)

        if len(samples) < 2:
            raise RuntimeError("GPS 静稳样本不足，检查 /gps/odom 是否持续发布。")

        reference = samples[0]
        squared_errors = []
        yaw_errors = []
        for sample in samples:
            dx = sample["x"] - reference["x"]
            dy = sample["y"] - reference["y"]
            squared_errors.append(dx * dx + dy * dy)
            yaw_errors.append(abs(normalize_angle(sample["yaw_rad"] - reference["yaw_rad"])))
        rms_m = math.sqrt(sum(squared_errors) / len(squared_errors))
        max_m = math.sqrt(max(squared_errors))
        max_yaw_deg = math.degrees(max(yaw_errors))

        print("[T01] 样本数: %d" % len(samples))
        print("[T01] 相对首帧位置 RMS: %.3f m" % rms_m)
        print("[T01] 相对首帧最大偏移: %.3f m" % max_m)
        print("[T01] 相对首帧最大航向变化: %.3f deg" % max_yaw_deg)
        gps_acceptable = self.ask_yes_no("GPS 定位和航向稳定性是否满足现场要求")
        cleaning_ready = self.ask_yes_no("清扫装置是否已上电并进入待机状态")
        passed = all(topic_status.values()) and gps_acceptable and cleaning_ready
        notes = input("备注（可留空）: ").strip()

        self.save_fod_record({
            "record_type": "device_check",
            "task_id": "T01",
            "task_name": "设备与 GPS 检查",
            "topic_status": topic_status,
            "sample_duration_s": duration,
            "sample_count": len(samples),
            "position_rms_m": rms_m,
            "position_max_m": max_m,
            "heading_max_change_deg": max_yaw_deg,
            "gps_acceptable": gps_acceptable,
            "cleaning_ready": cleaning_ready,
            "passed": passed,
            "notes": notes,
        })
        print("[T01] 结果：%s" % ("通过" if passed else "不通过"))

    def wait_for_fod_goal(self):
        input("准备完成后按回车进入等待；随后让检测车发送 FOD 点，并在 RabbitMQ 桥接终端输入 1。")
        with self.goal_condition:
            baseline = self.goal_sequence
            deadline = time.monotonic() + self.fod_goal_wait_timeout
            print("[等待] 新的 %s，超时 %.0f 秒..." % (
                self.goal_fix_topic,
                self.fod_goal_wait_timeout,
            ))
            while not rospy.is_shutdown() and self.goal_sequence <= baseline:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise RuntimeError("等待新的 %s 超时。" % self.goal_fix_topic)
                self.goal_condition.wait(timeout=min(0.5, remaining))
            if rospy.is_shutdown():
                raise RuntimeError("ROS is shutting down")
            observed = dict(self.latest_observed_goal)

        print("[开始计时] %s" % observed["received_at"])
        print("[目标] lat=%.12f lon=%.12f" % (
            observed["latitude"],
            observed["longitude"],
        ))
        return observed

    def run_fod_case(self, task_id, area_m2=None, case_config=None):
        case = dict(case_config or FOD_RUN_CASES[task_id])
        print("\n========== %s %s ==========" % (task_id, case["name"]))
        print("位置：%s" % case["location"])
        print("现场准备：%s" % case["instruction"])
        print("开始定义：收到 /gps/goal_fix，即 RabbitMQ 桥接终端输入 1 后开始计时。")
        print("结束定义：车辆到达、清扫动作完成且确认 FOD 完全回收后按回车。")
        if self.fence is not None:
            print("[警告] 当前测试脚本已加载电子围栏；越界时会取消导航并发送零速度。")

        if area_m2 is None:
            area_m2 = self.ask_float(
                "测试区域面积 S（m²）",
                self.default_test_area_m2,
                minimum=1000.0,
            )
        goal = self.wait_for_fod_goal()
        completion = input("FOD 完全回收后按回车结束计时；输入 c 取消本次任务: ").strip().lower()
        end_unix = time.time()
        if completion in ("c", "cancel", "取消"):
            self.cancel_motion()
            self.save_fod_record({
                "record_type": "run",
                "task_id": task_id,
                "task_name": case["name"],
                "location": case["location"],
                "obstacle_count": case["obstacles"],
                "area_m2": area_m2,
                "goal": goal,
                "start_unix": goal["received_unix"],
                "end_unix": end_unix,
                "duration_s": end_unix - goal["received_unix"],
                "end_pose": self.pose_snapshot(),
                "passed": False,
                "aborted": True,
                "notes": "operator canceled",
            })
            print("[取消] 已取消 move_base 并记录本次中止。")
            return None

        duration_s = max(0.0, end_unix - goal["received_unix"])
        autonomous_arrival = self.ask_yes_no("车辆是否自主到达 FOD 位置")
        if case["obstacles"] == 0:
            obstacle_avoidance = True
        else:
            obstacle_avoidance = self.ask_yes_no("车辆是否成功避开全部固定障碍物")
        cleaning_started = self.ask_yes_no("清扫装置是否正常启动并完成动作")
        fod_recovered = self.ask_yes_no("FOD 是否被完全回收")
        collision = self.ask_yes_no("是否发生碰撞")
        boundary_violation = self.ask_yes_no("是否发生越界")
        manual_takeover = self.ask_yes_no("是否发生人工接管")
        vehicle_stopped = self.ask_yes_no("任务完成后车辆是否停止")
        notes = input("备注（漏扫、推移、未吸入等，可留空）: ").strip()

        efficiency = area_m2 / duration_s if duration_s > 0.0 else None
        criteria_pass = (
            autonomous_arrival
            and obstacle_avoidance
            and cleaning_started
            and fod_recovered
            and not collision
            and not boundary_violation
            and not manual_takeover
            and vehicle_stopped
        )
        efficiency_required = task_id.startswith("T03-") or task_id.startswith("T04-")
        efficiency_pass = efficiency is not None and efficiency >= FOD_EFFICIENCY_THRESHOLD
        # The outline defines the T03/T04 efficiency result from three-run
        # average time (S / t_avg), so a single run is judged only on its
        # field criteria. The group threshold is applied in print_efficiency_group().
        passed = criteria_pass

        record = self.save_fod_record({
            "record_type": "run",
            "task_id": task_id,
            "task_name": case["name"],
            "location": case["location"],
            "obstacle_count": case["obstacles"],
            "area_m2": area_m2,
            "goal": goal,
            "start_unix": goal["received_unix"],
            "start_at": goal["received_at"],
            "end_unix": end_unix,
            "end_at": iso_time(end_unix),
            "duration_s": duration_s,
            "end_pose": self.pose_snapshot(),
            "autonomous_arrival": autonomous_arrival,
            "obstacle_avoidance": obstacle_avoidance,
            "cleaning_started": cleaning_started,
            "fod_recovered": fod_recovered,
            "collision": collision,
            "boundary_violation": boundary_violation,
            "manual_takeover": manual_takeover,
            "vehicle_stopped": vehicle_stopped,
            "criteria_pass": criteria_pass,
            "efficiency_m2_s": efficiency,
            "efficiency_threshold_m2_s": FOD_EFFICIENCY_THRESHOLD,
            "individual_efficiency_meets_threshold": efficiency_pass,
            "efficiency_required": efficiency_required,
            "passed": passed,
            "notes": notes,
        })
        print("[用时] %.3f s" % duration_s)
        if efficiency is not None:
            print("[效率] %.3f m²/s" % efficiency)
        if efficiency_required:
            print("[单次现场判定] %s；最终效率判定等待本组三次测试完成。" % (
                "通过" if passed else "不通过",
            ))
        else:
            print("[结果] %s" % ("通过" if passed else "不通过"))
        return record

    def run_fod_series(self, task_ids):
        area_m2 = self.ask_float(
            "本组测试统一使用的区域面积 S（m²）",
            self.default_test_area_m2,
            minimum=1000.0,
        )
        for index, task_id in enumerate(task_ids, 1):
            if index > 1:
                input("车辆回到规定的相同起点和朝向、重新放置 FOD 后按回车继续。")
            self.run_fod_case(task_id, area_m2=area_m2)
        self.print_fod_summary()

    def task_t07_repeat_recovery(self):
        print("\n========== T07 重复回收测试 ==========")
        area_m2 = self.ask_float(
            "测试区域面积 S（m²）",
            self.default_test_area_m2,
            minimum=1000.0,
        )
        for repeat in range(1, 4):
            if repeat > 1:
                input("在同一位置重新放置 FOD，车辆恢复到测试起点后按回车继续。")
            case = {
                "name": "重复回收测试",
                "location": "同一位置第 %d 次" % repeat,
                "obstacles": 0,
                "instruction": "同一位置连续测试第 %d/3 次，重新放置 FOD。" % repeat,
            }
            self.run_fod_case("T07-%d" % repeat, area_m2=area_m2, case_config=case)
        self.print_fod_summary()

    def task_t08_exception_checklist(self):
        print("\n========== T08 异常与急停测试 ==========")
        print("所有项目必须在低速、急停人员就位的条件下执行。")
        scenarios = (
            ("sudden_obstacle", "固定障碍物突然占用路径"),
            ("duplicate_goal", "FOD 位置重复发送"),
            ("communication_loss", "通信中断"),
            ("manual_cancel", "人工取消任务"),
            ("emergency_stop", "紧急停车"),
        )
        results = {}
        for key, label in scenarios:
            print("\n[T08] %s" % label)
            results[key] = self.ask_pass_fail_skip("该项响应是否符合预期")
        notes = input("T08 备注（可留空）: ").strip()
        completed = all(value != "skip" for value in results.values())
        passed = completed and all(value == "pass" for value in results.values())
        self.save_fod_record({
            "record_type": "exception_checklist",
            "task_id": "T08",
            "task_name": "异常与急停测试",
            "scenario_results": results,
            "completed": completed,
            "passed": passed,
            "notes": notes,
        })
        if not completed:
            print("[T08] 结果：未完成（存在跳过项）")
        else:
            print("[T08] 结果：%s" % ("通过" if passed else "不通过"))

    def latest_records_for_ids(self, records, task_ids):
        wanted = set(task_ids)
        latest = {}
        for record in reversed(records):
            task_id = record.get("task_id")
            if task_id in wanted and task_id not in latest and not record.get("aborted", False):
                latest[task_id] = record
        return latest

    def print_efficiency_group(self, records, label, task_ids):
        latest = self.latest_records_for_ids(records, task_ids)
        if len(latest) != len(task_ids):
            print("[%s] 记录不完整：%d/%d" % (label, len(latest), len(task_ids)))
            return
        selected = [latest[task_id] for task_id in task_ids]
        areas = {round(float(record["area_m2"]), 6) for record in selected}
        if len(areas) != 1:
            print("[%s] 三次区域面积不一致，无法按 S/t_avg 计算最终效率。" % label)
            return
        average_duration = sum(float(record["duration_s"]) for record in selected) / len(selected)
        area_m2 = float(selected[0]["area_m2"])
        efficiency = area_m2 / average_duration if average_duration > 0.0 else 0.0
        criteria_pass = all(record.get("criteria_pass", False) for record in selected)
        passed = criteria_pass and efficiency >= FOD_EFFICIENCY_THRESHOLD
        print("[%s] 平均用时 %.3fs，最终效率 %.3fm²/s，判定：%s" % (
            label,
            average_duration,
            efficiency,
            "通过" if passed else "不通过",
        ))

    def print_fod_summary(self):
        records = self.load_fod_records()
        print("\n========== FOD 最终测试记录汇总 ==========")
        if not records:
            print("当前没有测试记录。")
            return
        run_records = [record for record in records if record.get("record_type") == "run"]
        for record in run_records[-20:]:
            result = "中止" if record.get("aborted") else ("通过" if record.get("passed") else "不通过")
            print("  %-6s %-12s 用时=%7.3fs 效率=%s 结果=%s" % (
                record.get("task_id", ""),
                record.get("location", ""),
                float(record.get("duration_s", 0.0)),
                "%.3f" % float(record["efficiency_m2_s"])
                if record.get("efficiency_m2_s") is not None else "-",
                result,
            ))

        self.print_efficiency_group(records, "T03 机坪", ("T03-1", "T03-2", "T03-3"))
        self.print_efficiency_group(records, "T04 滑行道", ("T04-1", "T04-2", "T04-3"))

        repeats = self.latest_records_for_ids(records, ("T07-1", "T07-2", "T07-3"))
        if len(repeats) == 3:
            selected = [repeats["T07-%d" % index] for index in range(1, 4)]
            navigation_rate = sum(bool(record.get("autonomous_arrival")) for record in selected) / 3.0
            recovery_rate = sum(bool(record.get("fod_recovered")) for record in selected) / 3.0
            average_duration = sum(float(record.get("duration_s", 0.0)) for record in selected) / 3.0
            print("[T07 重复] 导航成功率 %.1f%%，回收成功率 %.1f%%，平均用时 %.3fs" % (
                navigation_rate * 100.0,
                recovery_rate * 100.0,
                average_duration,
            ))
        else:
            print("[T07 重复] 记录不完整：%d/3" % len(repeats))
        print("JSONL: %s" % self.fod_jsonl_file)
        print("CSV:   %s" % self.fod_csv_file)

    def fod_final_test_menu(self):
        while not rospy.is_shutdown():
            print("\n========== FOD 回收装备最终测试 ==========")
            print("  1: T01 设备与 GPS 检查（默认静止采样 120 秒）")
            print("  2: T02 基础回收测试")
            print("  3: T03 机坪处置效率测试（中心/左上/右下，共 3 次）")
            print("  4: T04 滑行道处置效率测试（近/中/远，共 3 次）")
            print("  5: T05 单个固定障碍物测试")
            print("  6: T06 多个固定障碍物测试")
            print("  7: T07 同一位置重复回收测试（共 3 次）")
            print("  8: T08 异常与急停检查表")
            print("  9: 显示测试记录和效率汇总")
            print("  0: 返回 GPS 测试主菜单")
            choice = input("FOD测试> ").strip().lower()
            try:
                if choice == "1":
                    self.task_t01_device_and_gps_check()
                elif choice == "2":
                    self.run_fod_case("T02")
                elif choice == "3":
                    self.run_fod_series(("T03-1", "T03-2", "T03-3"))
                elif choice == "4":
                    self.run_fod_series(("T04-1", "T04-2", "T04-3"))
                elif choice == "5":
                    self.run_fod_case("T05")
                elif choice == "6":
                    self.run_fod_case("T06")
                elif choice == "7":
                    self.task_t07_repeat_recovery()
                elif choice == "8":
                    self.task_t08_exception_checklist()
                elif choice == "9":
                    self.print_fod_summary()
                elif choice in ("0", "b", "back", "返回"):
                    return
                else:
                    print("未知输入：%s" % choice)
            except (rospy.ROSException, RuntimeError, ValueError, OSError) as exc:
                print("[错误] %s" % exc)

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
            print("  6: FOD 回收装备最终测试（T01-T08）")
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
                elif choice in ("6", "f", "fod"):
                    self.fod_final_test_menu()
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
