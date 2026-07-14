#!/usr/bin/env python3
"""Simulation-gated V2-03 local geometry, tracking, prediction, and health node."""

import math
from pathlib import Path
import threading

import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import Point32
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from sensor_msgs.msg import LaserScan

from nav_world_model.core import (
    MultiObjectTracker,
    Point2,
    RobotState,
    ScanFrame,
    compute_local_geometry,
    compute_path_metrics,
    extract_detections,
    scan_to_local_points,
    transform_points,
    validate_scan,
)
from nav_world_model.msg import (
    LocalGeometry,
    PredictedState,
    TrackedObstacle,
    TrackedObstacleArray,
    WorldModelHealth,
)


MOTION_CLASS = {
    "UNKNOWN": TrackedObstacle.MOTION_UNKNOWN,
    "STATIONARY": TrackedObstacle.MOTION_STATIONARY,
    "CROSSING": TrackedObstacle.MOTION_CROSSING,
    "HEAD_ON": TrackedObstacle.MOTION_HEAD_ON,
    "FOLLOWING": TrackedObstacle.MOTION_FOLLOWING,
    "DEPARTING": TrackedObstacle.MOTION_DEPARTING,
}


def _yaw(quaternion):
    siny = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny, cosy)


def _load_config(path):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("world-model config root must be a mapping")
    required = {
        "schema_version", "architecture_generation", "profile_id", "status",
        "simulation_only", "runtime_ready", "training_allowed",
        "real_vehicle_use_forbidden", "allow_unfrozen_simulation_candidate_required",
        "topics", "frames", "scan", "detection", "tracker", "prediction",
        "corridor", "health", "policy_boundary",
    }
    if set(data) != required:
        raise ValueError("world-model config keys drifted")
    if str(data["schema_version"]) != "2.0" or data["architecture_generation"] != "v2":
        raise ValueError("world-model schema drifted")
    if not (data["simulation_only"] is True
            and data["runtime_ready"] is False
            and data["training_allowed"] is False
            and data["real_vehicle_use_forbidden"] is True):
        raise ValueError("world-model candidate safety boundary drifted")
    boundary = data["policy_boundary"]
    if boundary["scene_manifest_access"] is not False:
        raise ValueError("scene manifest access is forbidden")
    if boundary["gazebo_or_pedsim_truth_evaluator_only"] is not True:
        raise ValueError("truth must remain evaluator-only")
    return data


class WorldModelNode:
    def __init__(self):
        config_path = rospy.get_param("~config")
        self.config = _load_config(config_path)
        if not rospy.get_param("/m2_gazebo/simulation_only", False):
            raise RuntimeError("V2-03 world model requires the simulation-only marker")
        if not rospy.get_param("~allow_unfrozen_simulation_candidate", False):
            raise RuntimeError("unfrozen V2-03 candidate requires explicit simulation opt-in")
        self.lock = threading.RLock()
        self.sequence = 0
        self.latest_odom = None
        self.latest_path = None
        self.latest_costmap_stamp = None
        self.last_scan_stamp_s = None
        self.last_tf_age_s = float("inf")
        self.last_tracker_stamp_s = None
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        tracker = self.config["tracker"]
        prediction = self.config["prediction"]
        self.tracker = MultiObjectTracker(
            association_gate_m=tracker["association_gate_m"],
            alpha=tracker["alpha"],
            beta=tracker["beta"],
            minimum_confirmed_hits=tracker["minimum_confirmed_hits"],
            maximum_misses=tracker["maximum_misses"],
            maximum_dt_s=tracker["maximum_dt_s"],
            stationary_speed_max_mps=tracker["stationary_speed_max_mps"],
            dynamic_speed_min_mps=tracker["dynamic_speed_min_mps"],
            prediction_horizon_s=prediction["horizon_s"],
            prediction_step_s=prediction["step_s"],
            confidence_decay_per_s=prediction["confidence_decay_per_s"],
            crossing_lateral_speed_min_mps=tracker["crossing_lateral_speed_min_mps"],
            crossing_path_half_width_m=tracker["crossing_path_half_width_m"],
        )
        topics = self.config["topics"]
        self.geometry_publisher = rospy.Publisher(
            topics["local_geometry"], LocalGeometry, queue_size=2
        )
        self.track_publisher = rospy.Publisher(
            topics["tracks"], TrackedObstacleArray, queue_size=2
        )
        self.health_publisher = rospy.Publisher(
            topics["health"], WorldModelHealth, queue_size=2, latch=True
        )
        rospy.Subscriber(topics["odometry"], Odometry, self._odom, queue_size=5)
        rospy.Subscriber(topics["global_path"], NavPath, self._path, queue_size=2)
        rospy.Subscriber(topics["local_costmap"], OccupancyGrid, self._costmap, queue_size=2)
        rospy.Subscriber(topics["scan"], LaserScan, self._scan, queue_size=2)
        self.health_timer = rospy.Timer(rospy.Duration(0.10), self._health_watchdog)

    def _odom(self, message):
        with self.lock:
            self.latest_odom = message

    def _path(self, message):
        with self.lock:
            self.latest_path = message

    def _costmap(self, message):
        with self.lock:
            self.latest_costmap_stamp = message.header.stamp

    def _scan(self, message):
        with self.lock:
            self.sequence += 1
            sequence = self.sequence
            try:
                self._process_scan(message, sequence)
            except Exception as exc:
                rospy.logwarn_throttle(1.0, "V2-03 world model rejected scan: %s", exc)
                self._publish_fault(message.header.stamp, sequence, "scan_processing_fault:{}".format(exc))

    def _process_scan(self, message, sequence):
        scan = ScanFrame(
            stamp_s=message.header.stamp.to_sec(),
            frame_id=message.header.frame_id,
            angle_min=message.angle_min,
            angle_max=message.angle_max,
            angle_increment=message.angle_increment,
            range_min=message.range_min,
            range_max=message.range_max,
            ranges=tuple(message.ranges),
        )
        validate_scan(scan, self.config["scan"]["required_ray_count"])
        if self.last_scan_stamp_s is not None and scan.stamp_s <= self.last_scan_stamp_s:
            self.tracker.reset()
        self.last_scan_stamp_s = scan.stamp_s
        odometry = self.latest_odom
        if odometry is None:
            raise RuntimeError("odometry_missing")
        health_config = self.config["health"]
        transform = self.tf_buffer.lookup_transform(
            self.config["frames"]["fixed"],
            scan.frame_id,
            message.header.stamp,
            rospy.Duration(health_config["tf_lookup_timeout_s"]),
        )
        transform_yaw = _yaw(transform.transform.rotation)
        local_points = scan_to_local_points(scan)
        fixed_points = transform_points(
            local_points,
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform_yaw,
        )
        odom_yaw = _yaw(odometry.pose.pose.orientation)
        robot = RobotState(
            x=odometry.pose.pose.position.x,
            y=odometry.pose.pose.position.y,
            yaw=odom_yaw,
            linear_velocity=odometry.twist.twist.linear.x,
        )
        detection_config = self.config["detection"]
        detections = extract_detections(
            fixed_points,
            maximum_gap_m=detection_config["maximum_cluster_gap_m"],
            minimum_points=detection_config["minimum_cluster_points"],
            maximum_diameter_m=detection_config["maximum_cluster_diameter_m"],
            maximum_range_m=detection_config["maximum_tracking_range_m"],
            origin=Point2(robot.x, robot.y),
        )
        tracks = self.tracker.update(detections, scan.stamp_s, robot)
        self.last_tracker_stamp_s = scan.stamp_s
        path_points = []
        if self.latest_path is not None and self.latest_path.header.frame_id == self.config["frames"]["fixed"]:
            path_points = [Point2(item.pose.position.x, item.pose.position.y)
                           for item in self.latest_path.poses]
        path_metrics = compute_path_metrics(robot, path_points)
        geometry = compute_local_geometry(
            scan,
            local_points,
            tracks,
            path_metrics,
            sector_half_width_rad=self.config["scan"]["sector_half_width_rad"],
            density_radius_m=self.config["scan"]["density_radius_m"],
            robot_radius_m=self.config["scan"]["robot_radius_m"],
            corridor_parameters=self.config["corridor"],
        )
        now = rospy.Time.now()
        scan_age = max(0.0, (now - message.header.stamp).to_sec())
        odom_age = max(0.0, (now - odometry.header.stamp).to_sec())
        tf_stamp = transform.header.stamp
        self.last_tf_age_s = 0.0 if tf_stamp == rospy.Time(0) else max(
            0.0, (message.header.stamp - tf_stamp).to_sec()
        )
        health_valid = (
            scan_age <= health_config["maximum_scan_age_s"]
            and odom_age <= health_config["maximum_odometry_age_s"]
            and geometry.front_covered and geometry.left_covered
            and geometry.right_covered and geometry.rear_covered
        )
        if health_config["require_costmap"] and self.latest_costmap_stamp is None:
            health_valid = False
        fault_reason = "" if health_valid else "input_stale_or_directional_coverage_missing"
        self._publish_geometry(message.header, sequence, geometry, health_valid)
        self._publish_tracks(message.header, sequence, tracks, robot)
        self._publish_health(
            message.header.stamp,
            sequence,
            valid=health_valid,
            stale=not health_valid,
            scan_valid=True,
            tf_valid=True,
            localization_valid=odom_age <= health_config["maximum_odometry_age_s"],
            costmap_valid=self.latest_costmap_stamp is not None,
            tracker_valid=True,
            scan_age_s=scan_age,
            tf_age_s=self.last_tf_age_s,
            tracker_age_s=max(0.0, now.to_sec() - scan.stamp_s),
            fault_reason=fault_reason,
        )

    def _publish_geometry(self, header, sequence, estimate, valid):
        message = LocalGeometry()
        message.header = header
        message.header.frame_id = self.config["frames"]["robot"]
        message.schema_version = "2.0"
        message.world_model_seq = sequence
        message.valid = valid
        message.stale = not valid
        for field in (
            "front_clearance_m", "left_clearance_m", "right_clearance_m",
            "rear_clearance_m", "footprint_clearance_m", "obstacle_density",
            "static_persistence", "corridor_width_m", "corridor_axis_yaw_rad",
            "corridor_parallel_confidence", "corridor_center_offset_m",
            "dead_end_score", "path_curvature", "signed_cross_track_error_m",
            "signed_heading_error_rad", "goal_direction_stability",
            "front_covered", "left_covered", "right_covered", "rear_covered",
        ):
            setattr(message, field, getattr(estimate, field))
        self.geometry_publisher.publish(message)

    def _publish_tracks(self, header, sequence, estimates, robot):
        array = TrackedObstacleArray()
        array.header = header
        array.header.frame_id = self.config["frames"]["robot"]
        array.schema_version = "2.0"
        array.world_model_seq = sequence
        cosine, sine = math.cos(robot.yaw), math.sin(robot.yaw)
        robot_vx, robot_vy = robot.linear_velocity * cosine, robot.linear_velocity * sine
        for estimate in estimates:
            obstacle = TrackedObstacle()
            obstacle.track_id = estimate.track_id
            obstacle.motion_class = MOTION_CLASS[estimate.motion_class]
            local_x = cosine * (estimate.x - robot.x) + sine * (estimate.y - robot.y)
            local_y = -sine * (estimate.x - robot.x) + cosine * (estimate.y - robot.y)
            relative_vx_world, relative_vy_world = estimate.vx - robot_vx, estimate.vy - robot_vy
            local_vx = cosine * relative_vx_world + sine * relative_vy_world
            local_vy = -sine * relative_vx_world + cosine * relative_vy_world
            obstacle.pose.pose.position.x = local_x
            obstacle.pose.pose.position.y = local_y
            obstacle.pose.pose.orientation.w = 1.0
            obstacle.pose.covariance[0] = 0.02
            obstacle.pose.covariance[7] = 0.02
            obstacle.velocity.twist.linear.x = local_vx
            obstacle.velocity.twist.linear.y = local_vy
            obstacle.velocity.covariance[0] = 0.05
            obstacle.velocity.covariance[7] = 0.05
            obstacle.confidence = estimate.confidence
            obstacle.age = rospy.Duration(estimate.age_s)
            obstacle.miss_count = estimate.miss_count
            obstacle.last_update = rospy.Time.from_sec(estimate.last_update_s)
            radius = estimate.radius
            for x, y in ((-radius, -radius), (radius, -radius),
                         (radius, radius), (-radius, radius)):
                obstacle.footprint.points.append(Point32(x=x, y=y, z=0.0))
            for prediction in estimate.predictions:
                predicted = PredictedState()
                predicted.time_from_start = rospy.Duration(prediction.time_from_start_s)
                predicted.pose.pose.position.x = (
                    cosine * (prediction.x - robot.x) + sine * (prediction.y - robot.y)
                )
                predicted.pose.pose.position.y = (
                    -sine * (prediction.x - robot.x) + cosine * (prediction.y - robot.y)
                )
                predicted.pose.pose.orientation.w = 1.0
                predicted.pose.covariance[0] = prediction.position_variance
                predicted.pose.covariance[7] = prediction.position_variance
                predicted.velocity.twist.linear.x = local_vx
                predicted.velocity.twist.linear.y = local_vy
                predicted.velocity.covariance[0] = 0.05 + prediction.position_variance
                predicted.velocity.covariance[7] = 0.05 + prediction.position_variance
                predicted.confidence = prediction.confidence
                obstacle.predictions.append(predicted)
            array.obstacles.append(obstacle)
        self.track_publisher.publish(array)

    def _publish_health(self, stamp, sequence, **values):
        message = WorldModelHealth()
        message.header.stamp = stamp
        message.header.frame_id = self.config["frames"]["fixed"]
        message.schema_version = "2.0"
        message.world_model_seq = sequence
        for key, value in values.items():
            setattr(message, key, value)
        self.health_publisher.publish(message)

    def _publish_fault(self, stamp, sequence, reason):
        header = type("HeaderProxy", (), {})()
        header.stamp = stamp
        header.frame_id = self.config["frames"]["robot"]
        geometry = LocalGeometry()
        geometry.header.stamp = stamp
        geometry.header.frame_id = self.config["frames"]["robot"]
        geometry.schema_version = "2.0"
        geometry.world_model_seq = sequence
        geometry.valid = False
        geometry.stale = True
        self.geometry_publisher.publish(geometry)
        tracks = TrackedObstacleArray()
        tracks.header.stamp = stamp
        tracks.header.frame_id = self.config["frames"]["robot"]
        tracks.schema_version = "2.0"
        tracks.world_model_seq = sequence
        self.track_publisher.publish(tracks)
        self._publish_health(
            stamp, sequence, valid=False, stale=True, scan_valid=False,
            tf_valid=False, localization_valid=self.latest_odom is not None,
            costmap_valid=self.latest_costmap_stamp is not None, tracker_valid=False,
            scan_age_s=float("inf"), tf_age_s=float("inf"), tracker_age_s=float("inf"),
            fault_reason=reason,
        )

    def _health_watchdog(self, _event):
        with self.lock:
            if self.last_scan_stamp_s is None:
                return
            age = max(0.0, rospy.Time.now().to_sec() - self.last_scan_stamp_s)
            if age <= self.config["health"]["maximum_scan_age_s"]:
                return
            self._publish_health(
                rospy.Time.from_sec(self.last_scan_stamp_s), self.sequence,
                valid=False, stale=True, scan_valid=False, tf_valid=False,
                localization_valid=self.latest_odom is not None,
                costmap_valid=self.latest_costmap_stamp is not None,
                tracker_valid=False, scan_age_s=age, tf_age_s=self.last_tf_age_s,
                tracker_age_s=age, fault_reason="scan_watchdog_timeout",
            )


def main():
    rospy.init_node("nav_world_model")
    WorldModelNode()
    rospy.spin()


if __name__ == "__main__":
    main()
