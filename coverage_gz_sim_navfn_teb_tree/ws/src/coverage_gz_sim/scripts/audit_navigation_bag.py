#!/usr/bin/env python3
"""Audit an experiment bag against the static map and M2 kinematic limits."""

import argparse
import hashlib
import json
import math
import os
import struct

import numpy as np
import rosbag
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt


ACTIVE_STATES = {"GOING_TO_START", "TRANSITING", "SWEEPING"}
MODE_NAMES = {
    0: "ORDINARY_NAVFN",
    1: "COVERAGE_ENTRY_NAVFN",
    2: "KINEMATIC_HYBRID_ASTAR_TRANSIT",
    3: "ENFORCED_SWEEP",
}


def quaternion_yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    return float(ordered[int(fraction * (len(ordered) - 1))])


def file_sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


class StaticMapFootprintAudit:
    def __init__(self, yaml_path, front, rear, half_width):
        self.yaml_path = os.path.realpath(yaml_path)
        with open(self.yaml_path, "r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        image_path = config["image"]
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(self.yaml_path), image_path)
        self.image_path = os.path.realpath(image_path)
        self.image = np.asarray(Image.open(self.image_path), dtype=np.uint8)
        if self.image.ndim != 2:
            raise ValueError("static map image must be an 8-bit grayscale PGM")
        self.resolution = float(config["resolution"])
        self.origin_x, self.origin_y, self.origin_yaw = map(float, config["origin"])
        self.negate = bool(config.get("negate", 0))
        pixels = self.image.astype(np.float64) / 255.0
        occupancy = pixels if self.negate else 1.0 - pixels
        self.free = occupancy < float(config["free_thresh"])
        self.occupied = occupancy > float(config["occupied_thresh"])
        self.unknown = ~(self.free | self.occupied)
        self.clearance = distance_transform_edt(self.free) * self.resolution

        # Sampling at no more than half a map cell prevents a one-cell wall
        # from falling between samples as the rectangle rotates over the grid.
        sample_step = min(0.05, 0.5 * self.resolution)
        x_count = max(2, int(math.ceil((front + rear) / sample_step)) + 1)
        y_count = max(2, int(math.ceil(2.0 * half_width / sample_step)) + 1)
        local_x, local_y = np.meshgrid(
            np.linspace(-rear, front, x_count),
            np.linspace(-half_width, half_width, y_count),
        )
        self.local_x = local_x.ravel()
        self.local_y = local_y.ravel()

    def inspect(self, x, y, yaw):
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        world_x = x + cosine * self.local_x - sine * self.local_y
        world_y = y + sine * self.local_x + cosine * self.local_y
        dx = world_x - self.origin_x
        dy = world_y - self.origin_y
        origin_cosine = math.cos(self.origin_yaw)
        origin_sine = math.sin(self.origin_yaw)
        map_x_m = origin_cosine * dx + origin_sine * dy
        map_y_m = -origin_sine * dx + origin_cosine * dy
        columns = np.floor(map_x_m / self.resolution).astype(np.int64)
        map_rows = np.floor(map_y_m / self.resolution).astype(np.int64)
        inside = (
            (columns >= 0)
            & (columns < self.image.shape[1])
            & (map_rows >= 0)
            & (map_rows < self.image.shape[0])
        )
        rows = self.image.shape[0] - 1 - map_rows[inside]
        valid_columns = columns[inside]
        is_free = np.zeros(columns.shape, dtype=bool)
        is_occupied = np.zeros(columns.shape, dtype=bool)
        is_unknown = np.ones(columns.shape, dtype=bool)
        is_free[inside] = self.free[rows, valid_columns]
        is_occupied[inside] = self.occupied[rows, valid_columns]
        is_unknown[inside] = self.unknown[rows, valid_columns]
        distances = np.zeros(columns.shape, dtype=np.float64)
        distances[inside] = self.clearance[rows, valid_columns]
        return {
            "unsafe_points": int(np.count_nonzero(~is_free)),
            "occupied_points": int(np.count_nonzero(is_occupied)),
            "unknown_points": int(np.count_nonzero(is_unknown)),
            "minimum_grid_center_clearance_m": float(np.min(distances)),
        }

    def metadata(self):
        return {
            "yaml": self.yaml_path,
            "image": self.image_path,
            "yaml_sha256": file_sha256(self.yaml_path),
            "image_sha256": file_sha256(self.image_path),
            "width_cells": int(self.image.shape[1]),
            "height_cells": int(self.image.shape[0]),
            "resolution_m": self.resolution,
            "origin": [self.origin_x, self.origin_y, self.origin_yaw],
        }


def path_signature(message):
    digest = hashlib.sha256()
    digest.update(str(message.plan_id).encode("utf-8"))
    digest.update(struct.pack("<II", int(message.segment_index), int(message.planner_mode)))
    for pose in message.path.poses:
        digest.update(struct.pack(
            "<ddd",
            float(pose.pose.position.x),
            float(pose.pose.position.y),
            quaternion_yaw(pose.pose.orientation),
        ))
    return digest.hexdigest()


def audit(args):
    static_map = StaticMapFootprintAudit(
        args.map_yaml, args.footprint_front, args.footprint_rear,
        args.footprint_half_width,
    )
    state = ""
    region = ""
    segment = 0
    active = False
    planner_mode = -1
    final_state = ""
    last_pose_stamp = -float("inf")
    last_motion_pose = None
    distance_m = 0.0
    actual_pose_samples = 0
    unsafe_actual_samples = 0
    occupied_actual_samples = 0
    unknown_actual_samples = 0
    actual_clearances = []
    first_unsafe_actual = []
    unique_hybrid_paths = set()
    hybrid_path_pose_samples = 0
    unsafe_hybrid_path_poses = 0
    first_unsafe_path = []
    tracking_errors = []
    curvature_samples = 0
    curvature_violations = 0
    minimum_command_radius = float("inf")
    direction_changes = 0
    direction_changes_without_zero = 0
    last_motion_sign = 0
    zero_seen = True

    topics = [
        "/coverage/status",
        "/coverage/enforced_path",
        "/coverage_gz_sim/hybrid_tracking_error",
        "/cmd_vel_sim",
        "/odom",
    ]
    with rosbag.Bag(args.bag) as bag:
        bag_start = bag.get_start_time()
        bag_end = bag.get_end_time()
        for topic, message, recorded_stamp in bag.read_messages(topics=topics):
            stamp = recorded_stamp.to_sec()
            if topic == "/coverage/status":
                state = str(message.state)
                final_state = state
                region = str(message.current_region_name)
                segment = int(message.current_segment)
                active = bool(message.active)
                continue
            if topic == "/coverage/enforced_path":
                planner_mode = int(message.planner_mode)
                if (
                    planner_mode == 2
                    and len(message.path.poses) >= 2
                    and path_signature(message) not in unique_hybrid_paths
                ):
                    signature = path_signature(message)
                    unique_hybrid_paths.add(signature)
                    for index, pose in enumerate(message.path.poses):
                        hybrid_path_pose_samples += 1
                        result = static_map.inspect(
                            float(pose.pose.position.x),
                            float(pose.pose.position.y),
                            quaternion_yaw(pose.pose.orientation),
                        )
                        if result["unsafe_points"]:
                            unsafe_hybrid_path_poses += 1
                            if len(first_unsafe_path) < 10:
                                first_unsafe_path.append({
                                    "signature": signature,
                                    "pose_index": index,
                                    "x": float(pose.pose.position.x),
                                    "y": float(pose.pose.position.y),
                                    **result,
                                })
                continue
            if topic == "/coverage_gz_sim/hybrid_tracking_error":
                value = float(message.data)
                if math.isfinite(value):
                    tracking_errors.append(value)
                continue
            if topic == "/cmd_vel_sim":
                velocity = float(message.linear.x)
                omega = float(message.angular.z)
                if abs(velocity) > 0.01 and abs(omega) > 0.01:
                    curvature_samples += 1
                    radius = abs(velocity / omega)
                    minimum_command_radius = min(minimum_command_radius, radius)
                    if radius + 1.0e-6 < args.minimum_turning_radius:
                        curvature_violations += 1
                sign = 1 if velocity > 0.02 else -1 if velocity < -0.02 else 0
                if sign == 0:
                    zero_seen = True
                elif last_motion_sign and sign != last_motion_sign:
                    direction_changes += 1
                    if not zero_seen:
                        direction_changes_without_zero += 1
                    last_motion_sign = sign
                    zero_seen = False
                elif sign:
                    last_motion_sign = sign
                    zero_seen = False
                continue
            if not active or state not in ACTIVE_STATES:
                last_motion_pose = None
                continue
            pose = message.pose.pose
            x = float(pose.position.x)
            y = float(pose.position.y)
            if last_motion_pose is not None:
                distance_m += math.hypot(x - last_motion_pose[0], y - last_motion_pose[1])
            last_motion_pose = (x, y)
            if stamp - last_pose_stamp + 1.0e-9 < args.pose_sample_period:
                continue
            last_pose_stamp = stamp
            actual_pose_samples += 1
            result = static_map.inspect(x, y, quaternion_yaw(pose.orientation))
            actual_clearances.append(result["minimum_grid_center_clearance_m"])
            if result["unsafe_points"]:
                unsafe_actual_samples += 1
                occupied_actual_samples += int(result["occupied_points"] > 0)
                unknown_actual_samples += int(result["unknown_points"] > 0)
                if len(first_unsafe_actual) < 10:
                    first_unsafe_actual.append({
                        "stamp": stamp,
                        "state": state,
                        "region": region,
                        "segment": segment,
                        "planner_mode": MODE_NAMES.get(planner_mode, str(planner_mode)),
                        "x": x,
                        "y": y,
                        **result,
                    })

    tracking_summary = {
        "samples": len(tracking_errors),
        "mean_m": float(np.mean(tracking_errors)) if tracking_errors else None,
        "median_m": percentile(tracking_errors, 0.50),
        "p95_m": percentile(tracking_errors, 0.95),
        "p99_m": percentile(tracking_errors, 0.99),
        "maximum_m": max(tracking_errors) if tracking_errors else None,
    }
    result = {
        "passed": (
            final_state == "COMPLETED"
            and unsafe_actual_samples == 0
            and unsafe_hybrid_path_poses == 0
            and curvature_violations == 0
            and direction_changes_without_zero == 0
        ),
        "bag": os.path.realpath(args.bag),
        "bag_duration_sec": bag_end - bag_start,
        "final_state": final_state,
        "map": static_map.metadata(),
        "footprint": {
            "front_m": args.footprint_front,
            "rear_m": args.footprint_rear,
            "half_width_m": args.footprint_half_width,
        },
        "actual_trajectory": {
            "distance_m": distance_m,
            "pose_samples": actual_pose_samples,
            "unsafe_pose_samples": unsafe_actual_samples,
            "occupied_pose_samples": occupied_actual_samples,
            "unknown_pose_samples": unknown_actual_samples,
            "minimum_grid_center_clearance_m": min(actual_clearances)
            if actual_clearances else None,
            "p01_grid_center_clearance_m": percentile(actual_clearances, 0.01),
            "first_unsafe": first_unsafe_actual,
        },
        "hybrid_paths": {
            "unique_fixed_gear_parts": len(unique_hybrid_paths),
            "pose_samples": hybrid_path_pose_samples,
            "unsafe_pose_samples": unsafe_hybrid_path_poses,
            "first_unsafe": first_unsafe_path,
        },
        "hybrid_tracking_error": tracking_summary,
        "commands": {
            "curvature_samples": curvature_samples,
            "curvature_violations": curvature_violations,
            "minimum_command_radius_m": minimum_command_radius
            if math.isfinite(minimum_command_radius) else None,
            "direction_changes": direction_changes,
            "direction_changes_without_zero": direction_changes_without_zero,
        },
    }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--map-yaml", required=True)
    parser.add_argument("--output")
    parser.add_argument("--minimum-turning-radius", type=float, default=1.35)
    parser.add_argument("--footprint-front", type=float, default=0.62)
    parser.add_argument("--footprint-rear", type=float, default=0.62)
    parser.add_argument("--footprint-half-width", type=float, default=0.45)
    parser.add_argument("--pose-sample-period", type=float, default=0.05)
    args = parser.parse_args()
    result = audit(args)
    rendered = json.dumps(result, indent=2, ensure_ascii=False, sort_keys=True)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as stream:
            stream.write(rendered)
            stream.write("\n")
    print(rendered)
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
