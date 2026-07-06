#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
import serial
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def parse_nmea_lat_lon(lat_value, lat_dir, lon_value, lon_dir):
    if lat_value == "" or lon_value == "":
        return None

    lat_raw = float(lat_value)
    lat = int(lat_raw / 100) + (lat_raw % 100) / 60.0
    if lat_dir == "S":
        lat = -lat

    lon_raw = float(lon_value)
    lon = int(lon_raw / 100) + (lon_raw % 100) / 60.0
    if lon_dir == "W":
        lon = -lon

    return lat, lon


def parse_gga(line):
    parts = line.split(",")
    if len(parts) < 10:
        return None
    if parts[2] == "" or parts[4] == "" or parts[6] == "0":
        return None

    lat_lon = parse_nmea_lat_lon(parts[2], parts[3], parts[4], parts[5])
    if lat_lon is None:
        return None

    lat, lon = lat_lon
    fix_quality = int(parts[6]) if parts[6].isdigit() else 0
    satellites = int(parts[7]) if parts[7].isdigit() else 0
    altitude = float(parts[9]) if parts[9] else 0.0
    return {
        "lat": lat,
        "lon": lon,
        "altitude": altitude,
        "fix_quality": fix_quality,
        "satellites": satellites,
        "course_deg": None,
        "speed_mps": None,
    }


def parse_rmc(line):
    parts = line.split(",")
    if len(parts) < 10:
        return None
    if parts[2] != "A":
        return None

    lat_lon = parse_nmea_lat_lon(parts[3], parts[4], parts[5], parts[6])
    if lat_lon is None:
        return None

    course_deg = None
    if parts[8] != "":
        course_deg = float(parts[8])
    speed_mps = None
    if parts[7] != "":
        speed_mps = float(parts[7]) * 0.514444

    lat, lon = lat_lon
    return {
        "lat": lat,
        "lon": lon,
        "altitude": None,
        "fix_quality": 1,
        "satellites": 0,
        "course_deg": course_deg,
        "speed_mps": speed_mps,
    }


def parse_nmea_fix(line):
    if line.startswith("$GNGGA") or line.startswith("$GPGGA"):
        return parse_gga(line)
    if line.startswith("$GNRMC") or line.startswith("$GPRMC"):
        return parse_rmc(line)
    return None


def gps_to_xy(lat, lon, origin_lat, origin_lon):
    radius = 6378137.0
    d_lat = math.radians(lat - origin_lat)
    d_lon = math.radians(lon - origin_lon)
    ref_lat = math.radians(origin_lat)
    x = radius * d_lon * math.cos(ref_lat)
    y = radius * d_lat
    return x, y


def yaw_from_course_deg(course_deg):
    # NMEA course is degrees clockwise from north. ROS yaw here is radians
    # counter-clockwise from the local x axis, where x is east and y is north.
    return math.radians(90.0 - course_deg)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


class GpsLocalizationNode:
    def __init__(self):
        self.port = rospy.get_param("~port", "/dev/ttyUSB1")
        self.baud_rate = int(rospy.get_param("~baud_rate", 115200))
        self.map_frame = rospy.get_param("~map_frame", "camera_init")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.pose_topic = rospy.get_param("~pose_topic", "/gps/pose")
        self.odom_topic = rospy.get_param("~odom_topic", "/gps/odom")
        self.fix_topic = rospy.get_param("~fix_topic", "/gps/fix")
        self.gps_frame = rospy.get_param("~gps_frame", "gps")
        self.origin_lat = rospy.get_param("~origin_lat", None)
        self.origin_lon = rospy.get_param("~origin_lon", None)
        if self.origin_lat == "":
            self.origin_lat = None
        if self.origin_lon == "":
            self.origin_lon = None
        if self.origin_lat is not None:
            self.origin_lat = float(self.origin_lat)
        if self.origin_lon is not None:
            self.origin_lon = float(self.origin_lon)
        if self.origin_lat is not None and self.origin_lon is not None:
            rospy.set_param("/gps/origin_lat", self.origin_lat)
            rospy.set_param("/gps/origin_lon", self.origin_lon)
        self.publish_rate = float(rospy.get_param("~publish_rate", 10.0))
        self.serial_timeout = float(rospy.get_param("~serial_timeout", 0.2))
        self.broadcast_tf = bool(rospy.get_param("~broadcast_tf", True))
        self.heading_source = rospy.get_param("~heading_source", "gps_course")
        self.min_course_distance = float(rospy.get_param("~min_course_distance", 0.5))
        self.initial_yaw = float(rospy.get_param("~initial_yaw", 0.0))
        self.position_filter_alpha = float(rospy.get_param("~position_filter_alpha", 0.25))
        self.stationary_filter_alpha = float(rospy.get_param("~stationary_filter_alpha", 0.05))
        self.stationary_speed_threshold = float(rospy.get_param("~stationary_speed_threshold", 0.05))
        self.stationary_hold_radius = float(rospy.get_param("~stationary_hold_radius", 0.8))
        self.max_fix_jump = float(rospy.get_param("~max_fix_jump", 5.0))
        self.heading_min_speed = float(rospy.get_param("~heading_min_speed", 0.15))
        self.use_wheel_odom = bool(rospy.get_param("~use_wheel_odom", False))
        self.wheel_odom_topic = rospy.get_param("~wheel_odom_topic", "/odom")
        self.gps_correction_alpha = float(rospy.get_param("~gps_correction_alpha", 0.0))
        self.gps_correction_max_step = float(rospy.get_param("~gps_correction_max_step", 0.05))
        self.gps_correction_reset_distance = float(rospy.get_param("~gps_correction_reset_distance", 20.0))

        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=10)
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self.fix_pub = rospy.Publisher(self.fix_topic, NavSatFix, queue_size=10)
        self.wheel_odom_sub = None
        if self.use_wheel_odom:
            self.wheel_odom_sub = rospy.Subscriber(
                self.wheel_odom_topic,
                Odometry,
                self.wheel_odom_cb,
                queue_size=10,
            )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.last_x = None
        self.last_y = None
        self.last_stamp = None
        self.last_altitude = 0.0
        self.last_yaw = self.initial_yaw
        self.last_linear_speed = 0.0
        self.last_gps_speed = None
        self.filtered_x = None
        self.filtered_y = None
        self.latest_wheel_odom = None
        self.last_wheel_x = None
        self.last_wheel_y = None
        self.last_wheel_yaw = None
        self.local_x = None
        self.local_y = None
        self.local_yaw = self.initial_yaw

        self.ser = serial.Serial(self.port, self.baud_rate, timeout=self.serial_timeout)
        rospy.loginfo(
            "GPS localization reading %s at %d baud; direct frame %s -> %s",
            self.port,
            self.baud_rate,
            self.map_frame,
            self.base_frame,
        )
        if self.use_wheel_odom:
            rospy.loginfo(
                "GPS localization using wheel odom %s for continuous local pose; gps_correction_alpha=%.3f",
                self.wheel_odom_topic,
                self.gps_correction_alpha,
            )

    def wheel_odom_cb(self, msg):
        self.latest_wheel_odom = msg

    def update_local_pose_from_wheel_odom(self, gps_x, gps_y, gps_yaw, gps_speed):
        if not self.use_wheel_odom:
            return None
        if self.latest_wheel_odom is None:
            rospy.logwarn_throttle(2.0, "Waiting for wheel odom on %s", self.wheel_odom_topic)
            return None

        pose = self.latest_wheel_odom.pose.pose
        wheel_x = pose.position.x
        wheel_y = pose.position.y
        wheel_yaw = yaw_from_quaternion(pose.orientation)

        if self.local_x is None or self.local_y is None:
            self.local_x = gps_x
            self.local_y = gps_y
            self.local_yaw = gps_yaw
            self.last_wheel_x = wheel_x
            self.last_wheel_y = wheel_y
            self.last_wheel_yaw = wheel_yaw
            return self.local_x, self.local_y, self.local_yaw, 0.0

        dx = wheel_x - self.last_wheel_x
        dy = wheel_y - self.last_wheel_y
        dyaw = normalize_angle(wheel_yaw - self.last_wheel_yaw)
        self.last_wheel_x = wheel_x
        self.last_wheel_y = wheel_y
        self.last_wheel_yaw = wheel_yaw

        self.local_x += dx
        self.local_y += dy
        self.local_yaw = normalize_angle(self.local_yaw + dyaw)

        correction_x = gps_x - self.local_x
        correction_y = gps_y - self.local_y
        correction_distance = math.hypot(correction_x, correction_y)
        if (
            self.gps_correction_reset_distance > 0.0
            and correction_distance > self.gps_correction_reset_distance
        ):
            rospy.logwarn(
                "Wheel/GPS pose diverged by %.3f m; resetting local pose to GPS",
                correction_distance,
            )
            self.local_x = gps_x
            self.local_y = gps_y
        elif self.gps_correction_alpha > 0.0 and correction_distance > self.stationary_hold_radius:
            step_x = self.gps_correction_alpha * correction_x
            step_y = self.gps_correction_alpha * correction_y
            step = math.hypot(step_x, step_y)
            if self.gps_correction_max_step > 0.0 and step > self.gps_correction_max_step:
                scale = self.gps_correction_max_step / step
                step_x *= scale
                step_y *= scale
            self.local_x += step_x
            self.local_y += step_y

        wheel_speed = self.latest_wheel_odom.twist.twist.linear.x
        if gps_speed is not None and abs(gps_speed) > abs(wheel_speed):
            linear_speed = gps_speed
        else:
            linear_speed = wheel_speed
        return self.local_x, self.local_y, self.local_yaw, linear_speed

    def filter_position(self, raw_x, raw_y, speed_mps):
        if self.filtered_x is None or self.filtered_y is None:
            self.filtered_x = raw_x
            self.filtered_y = raw_y
            return self.filtered_x, self.filtered_y

        dx = raw_x - self.filtered_x
        dy = raw_y - self.filtered_y
        distance = math.hypot(dx, dy)

        if self.max_fix_jump > 0.0 and distance > self.max_fix_jump:
            rospy.logwarn_throttle(
                2.0,
                "GPS fix jump rejected: raw=(%.3f, %.3f) filtered=(%.3f, %.3f) jump=%.3f m",
                raw_x,
                raw_y,
                self.filtered_x,
                self.filtered_y,
                distance,
            )
            return self.filtered_x, self.filtered_y

        is_stationary = speed_mps is not None and speed_mps < self.stationary_speed_threshold
        if is_stationary and distance <= self.stationary_hold_radius:
            return self.filtered_x, self.filtered_y

        alpha = self.stationary_filter_alpha if is_stationary else self.position_filter_alpha
        alpha = max(0.0, min(1.0, alpha))
        self.filtered_x += alpha * dx
        self.filtered_y += alpha * dy
        return self.filtered_x, self.filtered_y

    def publish_fix(self, stamp, lat, lon, altitude, fix_quality):
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = self.gps_frame
        msg.status.status = NavSatStatus.STATUS_FIX if fix_quality > 0 else NavSatStatus.STATUS_NO_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = altitude
        self.fix_pub.publish(msg)

    def update_motion(self, stamp, x, y, course_deg, speed_mps):
        yaw = self.last_yaw
        linear_speed = speed_mps if speed_mps is not None else 0.0
        if (
            course_deg is not None
            and speed_mps is not None
            and speed_mps >= self.heading_min_speed
            and self.heading_source in ("gps_course", "auto")
        ):
            yaw = yaw_from_course_deg(course_deg)
        elif self.last_x is not None and self.last_y is not None:
            dx = x - self.last_x
            dy = y - self.last_y
            distance = math.hypot(dx, dy)
            dt = (stamp - self.last_stamp).to_sec() if self.last_stamp is not None else 0.0
            if speed_mps is None and dt > 0.0:
                linear_speed = distance / dt
            if (
                self.heading_source in ("gps_course", "auto")
                and distance >= self.min_course_distance
                and (speed_mps is None or speed_mps >= self.stationary_speed_threshold)
            ):
                yaw = math.atan2(dy, dx)

        self.last_x = x
        self.last_y = y
        self.last_stamp = stamp
        self.last_yaw = yaw
        self.last_linear_speed = linear_speed
        return yaw, linear_speed

    def publish_pose_and_tf(self, stamp, x, y, yaw, linear_speed):
        q = quaternion_from_euler(0.0, 0.0, yaw)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        # Keep the navigation frame planar. GPS altitude is preserved in
        # /gps/fix, but using it in TF places the robot hundreds of meters
        # above the 2D costmap and prevents obstacle marking.
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        self.pose_pub.publish(pose)

        gps_odom = Odometry()
        gps_odom.header = pose.header
        gps_odom.child_frame_id = self.base_frame
        gps_odom.pose.pose = pose.pose
        gps_odom.twist.twist.linear.x = linear_speed
        self.odom_pub.publish(gps_odom)

        if not self.broadcast_tf:
            return

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.map_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(tf_msg)

    def spin(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            raw = self.ser.readline().decode("utf-8", errors="ignore").strip()
            parsed = parse_nmea_fix(raw)
            if parsed is None:
                rate.sleep()
                continue

            lat = parsed["lat"]
            lon = parsed["lon"]
            altitude = parsed["altitude"]
            fix_quality = parsed["fix_quality"]
            satellites = parsed["satellites"]
            course_deg = parsed["course_deg"]
            speed_mps = parsed["speed_mps"]
            if speed_mps is None:
                speed_mps = self.last_gps_speed
            else:
                self.last_gps_speed = speed_mps
            if altitude is None:
                altitude = self.last_altitude
            else:
                self.last_altitude = altitude

            if self.origin_lat is None or self.origin_lon is None:
                self.origin_lat = lat
                self.origin_lon = lon
                rospy.set_param("/gps/origin_lat", self.origin_lat)
                rospy.set_param("/gps/origin_lon", self.origin_lon)
                rospy.loginfo(
                    "GPS origin set to lat=%.8f lon=%.8f; satellites=%d",
                    self.origin_lat,
                    self.origin_lon,
                    satellites,
                )

            raw_x, raw_y = gps_to_xy(lat, lon, self.origin_lat, self.origin_lon)
            gps_x, gps_y = self.filter_position(raw_x, raw_y, speed_mps)
            stamp = rospy.Time.now()
            gps_yaw, gps_linear_speed = self.update_motion(stamp, gps_x, gps_y, course_deg, speed_mps)
            local_pose = self.update_local_pose_from_wheel_odom(gps_x, gps_y, gps_yaw, speed_mps)
            if local_pose is None:
                x, y, yaw, linear_speed = gps_x, gps_y, gps_yaw, gps_linear_speed
            else:
                x, y, yaw, linear_speed = local_pose
            self.publish_fix(stamp, lat, lon, altitude, fix_quality)
            self.publish_pose_and_tf(stamp, x, y, yaw, linear_speed)
            rate.sleep()

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    rospy.init_node("gps_localization_node")
    node = None
    try:
        node = GpsLocalizationNode()
        node.spin()
    finally:
        if node is not None:
            node.close()
