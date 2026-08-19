#!/usr/bin/env python3
"""Build a persistent 2-D occupancy map from the fused avoidance scan.

The mapper deliberately consumes the final ``/scan`` contract rather than
individual lidar topics.  Consequently the same implementation works with a
MID360-only degraded scan and with the normal MID360 + dual-LD19 fusion.  A
FAST-LIO odometry pose supplies the continuous camera_init-frame trajectory.

Two execution modes are supported:

* ``--ros`` subscribes to live LaserScan/Odometry topics and saves on shutdown;
* ``--bag`` reads an existing bag directly without starting a ROS master.

The saved map follows the ROS map_server PGM/YAML convention.
"""

import argparse
import datetime
import math
import os
import sys
import threading

import yaml


UNKNOWN_PIXEL = 205
FREE_PIXEL = 254
OCCUPIED_PIXEL = 0


def yaw_from_quaternion(quaternion):
    """Return planar yaw without depending on tf.transformations."""
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def bresenham(start_x, start_y, end_x, end_y):
    """Yield integer cells on a line, including both endpoints."""
    x = int(start_x)
    y = int(start_y)
    end_x = int(end_x)
    end_y = int(end_y)
    dx = abs(end_x - x)
    sx = 1 if x < end_x else -1
    dy = -abs(end_y - y)
    sy = 1 if y < end_y else -1
    error = dx + dy
    while True:
        yield x, y
        if x == end_x and y == end_y:
            break
        doubled = 2 * error
        if doubled >= dy:
            error += dy
            x += sx
        if doubled <= dx:
            error += dx
            y += sy


class SparseOccupancyMapper:
    """Sparse log-odds grid suitable for long FAST-LIO trajectories."""

    def __init__(
        self,
        resolution=0.10,
        beam_stride=4,
        scan_stride=2,
        min_range=0.10,
        max_range=12.0,
        free_space_range=8.0,
        base_offset_x=-0.20,
        base_offset_y=0.0,
        free_log_odds=-0.35,
        occupied_log_odds=0.85,
        min_log_odds=-4.0,
        max_log_odds=4.0,
    ):
        if resolution <= 0.0:
            raise ValueError("resolution must be positive")
        if beam_stride < 1 or scan_stride < 1:
            raise ValueError("beam_stride and scan_stride must be positive")
        if min_range < 0.0 or max_range <= min_range:
            raise ValueError("invalid min_range/max_range")
        if free_space_range <= min_range or free_space_range > max_range:
            raise ValueError("free_space_range must be within mapper range")
        self.resolution = float(resolution)
        self.beam_stride = int(beam_stride)
        self.scan_stride = int(scan_stride)
        self.min_range = float(min_range)
        self.max_range = float(max_range)
        self.free_space_range = float(free_space_range)
        self.base_offset_x = float(base_offset_x)
        self.base_offset_y = float(base_offset_y)
        self.free_log_odds = float(free_log_odds)
        self.occupied_log_odds = float(occupied_log_odds)
        self.min_log_odds = float(min_log_odds)
        self.max_log_odds = float(max_log_odds)
        self.cells = {}
        self.scan_messages = 0
        self.integrated_scans = 0
        self.integrated_beams = 0
        self.occupied_endpoints = 0
        self.skipped_pose_age = 0
        self.frame_rejections = 0
        self.first_base_pose = None
        self.final_base_pose = None

    def _cell(self, coordinate):
        return int(math.floor(coordinate / self.resolution))

    def _update(self, cell, increment):
        previous = self.cells.get(cell, 0.0)
        self.cells[cell] = min(
            self.max_log_odds,
            max(self.min_log_odds, previous + increment),
        )

    def integrate_scan(self, scan, pose_x, pose_y, pose_yaw):
        """Integrate one scan whose frame is base_link into camera_init."""
        self.scan_messages += 1
        if (self.scan_messages - 1) % self.scan_stride:
            return False
        if scan.angle_increment <= 0.0 or not scan.ranges:
            return False
        frame = str(scan.header.frame_id).lstrip("/")
        if frame != "base_link":
            self.frame_rejections += 1
            return False

        cos_yaw = math.cos(pose_yaw)
        sin_yaw = math.sin(pose_yaw)
        origin_x = (
            pose_x
            + cos_yaw * self.base_offset_x
            - sin_yaw * self.base_offset_y
        )
        origin_y = (
            pose_y
            + sin_yaw * self.base_offset_x
            + cos_yaw * self.base_offset_y
        )
        origin_cell = (self._cell(origin_x), self._cell(origin_y))

        beams = 0
        endpoints = 0
        sensor_min = max(self.min_range, float(scan.range_min))
        sensor_max = min(self.max_range, float(scan.range_max))
        for index in range(0, len(scan.ranges), self.beam_stride):
            measured_range = float(scan.ranges[index])
            hit = math.isfinite(measured_range)
            if hit:
                if measured_range < sensor_min:
                    continue
                if measured_range > sensor_max:
                    ray_range = self.free_space_range
                    hit = False
                else:
                    ray_range = measured_range
            elif math.isinf(measured_range) and measured_range > 0.0:
                ray_range = self.free_space_range
            else:
                continue

            if not hit:
                ray_range = min(ray_range, self.free_space_range)
            angle = pose_yaw + scan.angle_min + index * scan.angle_increment
            end_x = origin_x + ray_range * math.cos(angle)
            end_y = origin_y + ray_range * math.sin(angle)
            end_cell = (self._cell(end_x), self._cell(end_y))
            ray_cells = list(
                bresenham(origin_cell[0], origin_cell[1], end_cell[0], end_cell[1])
            )
            free_cells = ray_cells[:-1] if hit else ray_cells
            # A set prevents the same discretized ray from updating a cell twice.
            for cell in set(free_cells):
                self._update(cell, self.free_log_odds)
            if hit:
                self._update(end_cell, self.occupied_log_odds)
                endpoints += 1
            beams += 1

        if beams:
            self.integrated_scans += 1
            self.integrated_beams += beams
            self.occupied_endpoints += endpoints
            base_pose = [origin_x, origin_y, pose_yaw]
            if self.first_base_pose is None:
                self.first_base_pose = base_pose
            self.final_base_pose = base_pose
            return True
        return False

    @staticmethod
    def probability(log_odds):
        return 1.0 / (1.0 + math.exp(-log_odds))

    def save(self, output_dir, map_name="map", padding_m=1.0):
        if not self.cells or self.integrated_scans == 0:
            raise RuntimeError("no fused scans were integrated; refusing to save empty map")
        output_dir = os.path.abspath(output_dir)
        os.makedirs(output_dir, exist_ok=True)
        padding_cells = max(0, int(math.ceil(padding_m / self.resolution)))
        xs = [cell[0] for cell in self.cells]
        ys = [cell[1] for cell in self.cells]
        min_x = min(xs) - padding_cells
        max_x = max(xs) + padding_cells
        min_y = min(ys) - padding_cells
        max_y = max(ys) + padding_cells
        width = max_x - min_x + 1
        height = max_y - min_y + 1
        if width <= 0 or height <= 0 or width * height > 250000000:
            raise RuntimeError("refusing unreasonable map dimensions {}x{}".format(width, height))

        pixels = bytearray([UNKNOWN_PIXEL]) * (width * height)
        occupied_cells = 0
        free_cells = 0
        for (cell_x, cell_y), log_odds in self.cells.items():
            probability = self.probability(log_odds)
            if probability >= 0.65:
                pixel = OCCUPIED_PIXEL
                occupied_cells += 1
            elif probability <= 0.35:
                pixel = FREE_PIXEL
                free_cells += 1
            else:
                continue
            column = cell_x - min_x
            row_from_bottom = cell_y - min_y
            row = height - 1 - row_from_bottom
            pixels[row * width + column] = pixel

        pgm_path = os.path.join(output_dir, map_name + ".pgm")
        yaml_path = os.path.join(output_dir, map_name + ".yaml")
        metadata_path = os.path.join(output_dir, "mapping_info.yaml")
        temporary_pgm = pgm_path + ".tmp"
        with open(temporary_pgm, "wb") as stream:
            stream.write("P5\n{} {}\n255\n".format(width, height).encode("ascii"))
            stream.write(pixels)
        os.replace(temporary_pgm, pgm_path)

        map_yaml = {
            "image": os.path.basename(pgm_path),
            "resolution": self.resolution,
            "origin": [min_x * self.resolution, min_y * self.resolution, 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,
        }
        temporary_yaml = yaml_path + ".tmp"
        with open(temporary_yaml, "w", encoding="utf-8") as stream:
            yaml.safe_dump(map_yaml, stream, default_flow_style=False, sort_keys=False)
        os.replace(temporary_yaml, yaml_path)

        metadata = {
            "status": "complete",
            "generated_at": datetime.datetime.now().astimezone().isoformat(),
            "frame_id": "map",
            "trajectory_frame": "camera_init",
            "scan_frame": "base_link",
            "scan_topic": "/scan",
            "odom_topic": "/Odometry",
            "resolution_m": self.resolution,
            "width_cells": width,
            "height_cells": height,
            "origin": map_yaml["origin"],
            "scan_messages": self.scan_messages,
            "integrated_scans": self.integrated_scans,
            "integrated_beams": self.integrated_beams,
            "occupied_endpoints": self.occupied_endpoints,
            "occupied_cells": occupied_cells,
            "free_cells": free_cells,
            "skipped_pose_age": self.skipped_pose_age,
            "frame_rejections": self.frame_rejections,
            "base_offset_xy_m": [self.base_offset_x, self.base_offset_y],
            "first_base_pose_xyyaw": self.first_base_pose,
            "final_base_pose_xyyaw": self.final_base_pose,
            "beam_stride": self.beam_stride,
            "scan_stride": self.scan_stride,
            "max_range_m": self.max_range,
            "free_space_range_m": self.free_space_range,
        }
        temporary_metadata = metadata_path + ".tmp"
        with open(temporary_metadata, "w", encoding="utf-8") as stream:
            yaml.safe_dump(metadata, stream, default_flow_style=False, sort_keys=False)
        os.replace(temporary_metadata, metadata_path)
        return {
            "pgm": pgm_path,
            "yaml": yaml_path,
            "metadata": metadata_path,
            "width": width,
            "height": height,
        }


def mapper_from_arguments(arguments):
    return SparseOccupancyMapper(
        resolution=arguments.resolution,
        beam_stride=arguments.beam_stride,
        scan_stride=arguments.scan_stride,
        min_range=arguments.min_range,
        max_range=arguments.max_range,
        free_space_range=arguments.free_space_range,
        base_offset_x=arguments.base_offset_x,
        base_offset_y=arguments.base_offset_y,
    )


def process_bag(arguments):
    import rosbag

    mapper = mapper_from_arguments(arguments)
    latest_odometry = None
    latest_odom_time = None
    first_message_time = None
    final_message_time = None
    topics = [arguments.odom_topic, arguments.scan_topic]
    last_progress_count = 0
    with rosbag.Bag(arguments.bag, "r") as bag:
        for topic, message, bag_time in bag.read_messages(topics=topics):
            timestamp = message.header.stamp
            if timestamp.to_sec() <= 0.0:
                timestamp = bag_time
            if first_message_time is None:
                first_message_time = timestamp.to_sec()
            final_message_time = timestamp.to_sec()
            if topic == arguments.odom_topic:
                latest_odometry = message
                latest_odom_time = timestamp
                continue
            if latest_odometry is None or latest_odom_time is None:
                mapper.skipped_pose_age += 1
                continue
            age = abs((timestamp - latest_odom_time).to_sec())
            if age > arguments.max_pose_age:
                mapper.skipped_pose_age += 1
                continue
            pose = latest_odometry.pose.pose
            mapper.integrate_scan(
                message,
                pose.position.x,
                pose.position.y,
                yaw_from_quaternion(pose.orientation),
            )
            if (
                mapper.integrated_scans
                and mapper.integrated_scans % 250 == 0
                and mapper.integrated_scans != last_progress_count
            ):
                print(
                    "integrated {} scans, {} sparse cells".format(
                        mapper.integrated_scans, len(mapper.cells)
                    ),
                    flush=True,
                )
                last_progress_count = mapper.integrated_scans
    result = mapper.save(arguments.output_dir, arguments.map_name, arguments.padding)
    metadata_path = result["metadata"]
    with open(metadata_path, "r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    metadata["source_bag"] = os.path.abspath(arguments.bag)
    if first_message_time is not None and final_message_time is not None:
        metadata["bag_duration_sec"] = final_message_time - first_message_time
    temporary_metadata = metadata_path + ".tmp"
    with open(temporary_metadata, "w", encoding="utf-8") as stream:
        yaml.safe_dump(metadata, stream, default_flow_style=False, sort_keys=False)
    os.replace(temporary_metadata, metadata_path)
    print("MAP_SAVED={}".format(result["yaml"]), flush=True)
    print("MAP_SIZE={}x{}".format(result["width"], result["height"]), flush=True)
    return 0


class LiveMapperNode:
    def __init__(self, arguments):
        import rospy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan

        self.rospy = rospy
        self.arguments = arguments
        self.mapper = mapper_from_arguments(arguments)
        self.lock = threading.Lock()
        self.latest_odometry = None
        self.latest_odom_stamp = None
        self.saved = False
        self.odom_subscriber = rospy.Subscriber(
            arguments.odom_topic,
            Odometry,
            self._odom_callback,
            queue_size=20,
            tcp_nodelay=True,
        )
        self.scan_subscriber = rospy.Subscriber(
            arguments.scan_topic,
            LaserScan,
            self._scan_callback,
            queue_size=5,
            tcp_nodelay=True,
        )
        rospy.on_shutdown(self.save)
        rospy.loginfo(
            "fused_scan_mapper: %s + %s -> %s (resolution %.3f m)",
            arguments.scan_topic,
            arguments.odom_topic,
            arguments.output_dir,
            arguments.resolution,
        )

    def _odom_callback(self, message):
        with self.lock:
            self.latest_odometry = message
            self.latest_odom_stamp = message.header.stamp

    def _scan_callback(self, scan):
        with self.lock:
            odometry = self.latest_odometry
            odom_stamp = self.latest_odom_stamp
            if odometry is None or odom_stamp is None:
                self.mapper.skipped_pose_age += 1
                return
            scan_stamp = scan.header.stamp
            age = abs((scan_stamp - odom_stamp).to_sec())
            if age > self.arguments.max_pose_age:
                self.mapper.skipped_pose_age += 1
                self.rospy.logwarn_throttle(
                    5.0,
                    "fused_scan_mapper: odometry age %.3f s exceeds %.3f s",
                    age,
                    self.arguments.max_pose_age,
                )
                return
            pose = odometry.pose.pose
            self.mapper.integrate_scan(
                scan,
                pose.position.x,
                pose.position.y,
                yaw_from_quaternion(pose.orientation),
            )
        self.rospy.loginfo_throttle(
            10.0,
            "fused_scan_mapper: integrated %d scans, %d sparse cells",
            self.mapper.integrated_scans,
            len(self.mapper.cells),
        )

    def save(self):
        with self.lock:
            if self.saved:
                return
            self.saved = True
            try:
                result = self.mapper.save(
                    self.arguments.output_dir,
                    self.arguments.map_name,
                    self.arguments.padding,
                )
            except Exception as error:  # shutdown path must leave explicit evidence
                print("MAP_SAVE_FAILED={}".format(error), file=sys.stderr, flush=True)
                self.rospy.logerr("fused_scan_mapper: map save failed: %s", error)
                return
        try:
            print("MAP_SAVED={}".format(result["yaml"]), flush=True)
            self.rospy.loginfo("fused_scan_mapper: saved %s", result["yaml"])
        except Exception as error:
            print("MAP_SAVE_REPORT_FAILED={}".format(error), file=sys.stderr, flush=True)


def process_live(arguments):
    import rospy

    rospy.init_node("fused_scan_mapper", disable_signals=False)
    LiveMapperNode(arguments)
    rospy.spin()
    return 0


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--ros", action="store_true", help="subscribe to live ROS topics")
    mode.add_argument("--bag", help="read an existing rosbag directly")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--map-name", default="map")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--odom-topic", default="/Odometry")
    parser.add_argument("--resolution", type=float, default=0.10)
    parser.add_argument("--beam-stride", type=int, default=4)
    parser.add_argument("--scan-stride", type=int, default=2)
    parser.add_argument("--min-range", type=float, default=0.10)
    parser.add_argument("--max-range", type=float, default=12.0)
    parser.add_argument("--free-space-range", type=float, default=8.0)
    parser.add_argument("--base-offset-x", type=float, default=-0.20)
    parser.add_argument("--base-offset-y", type=float, default=0.0)
    parser.add_argument("--max-pose-age", type=float, default=0.20)
    parser.add_argument("--padding", type=float, default=1.0)
    return parser.parse_args(argv)


def main(argv=None):
    arguments = parse_arguments(argv)
    if arguments.max_pose_age <= 0.0:
        raise ValueError("max_pose_age must be positive")
    if arguments.bag:
        return process_bag(arguments)
    return process_live(arguments)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError, ValueError) as error:
        print("ERROR: {}".format(error), file=sys.stderr)
        sys.exit(1)
