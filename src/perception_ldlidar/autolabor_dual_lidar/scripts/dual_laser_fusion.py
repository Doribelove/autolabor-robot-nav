#!/usr/bin/env python3
"""Fuse two planar LaserScan streams into one scan in a vehicle frame."""

import math

import message_filters
import rospy
from sensor_msgs.msg import LaserScan


TAU = 2.0 * math.pi


def _valid_range(value, scan):
    return (
        math.isfinite(value)
        and value > 0.0
        and value >= scan.range_min
        and value <= scan.range_max
    )


def add_scan_to_bins(scan, x_offset, y_offset, yaw, angle_min,
                     angle_increment, ranges, intensities):
    """Transform scan points into the target frame and keep the nearest/bin."""
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    bin_count = len(ranges)

    for index, distance in enumerate(scan.ranges):
        if not _valid_range(distance, scan):
            continue

        source_angle = scan.angle_min + index * scan.angle_increment
        source_x = distance * math.cos(source_angle)
        source_y = distance * math.sin(source_angle)
        target_x = x_offset + cos_yaw * source_x - sin_yaw * source_y
        target_y = y_offset + sin_yaw * source_x + cos_yaw * source_y
        target_range = math.hypot(target_x, target_y)
        target_angle = math.atan2(target_y, target_x)

        # The output scan covers a complete circle. Wrap +pi into the -pi bin.
        relative_angle = (target_angle - angle_min) % TAU
        target_index = int(math.floor(relative_angle / angle_increment + 0.5)) % bin_count

        if target_range < ranges[target_index]:
            ranges[target_index] = target_range
            if index < len(scan.intensities):
                intensities[target_index] = scan.intensities[index]
            else:
                intensities[target_index] = 0.0


class DualLaserFusion:
    def __init__(self):
        self.target_frame = rospy.get_param("~target_frame", "base_link")
        self.output_topic = rospy.get_param("~output_topic", "/scan")
        self.angle_increment = rospy.get_param(
            "~angle_increment", math.radians(0.5))
        if self.angle_increment <= 0.0:
            raise ValueError("~angle_increment must be positive")

        self.angle_min = -math.pi
        self.bin_count = int(round(TAU / self.angle_increment))
        if self.bin_count < 2:
            raise ValueError("~angle_increment is too large")
        self.angle_increment = TAU / self.bin_count
        self.angle_max = self.angle_min + (self.bin_count - 1) * self.angle_increment

        self.front_pose = self._read_pose("front")
        self.rear_pose = self._read_pose("rear")
        self.output_range_min = rospy.get_param("~range_min", 0.02)
        self.output_range_max = rospy.get_param("~range_max", 13.0)

        front_topic = rospy.get_param("~front_topic", "/lidar/front/scan_raw")
        rear_topic = rospy.get_param("~rear_topic", "/lidar/rear/scan_raw")
        queue_size = rospy.get_param("~queue_size", 10)
        sync_slop = rospy.get_param("~sync_slop", 0.08)

        self.publisher = rospy.Publisher(
            self.output_topic, LaserScan, queue_size=1)
        self.front_subscriber = message_filters.Subscriber(front_topic, LaserScan)
        self.rear_subscriber = message_filters.Subscriber(rear_topic, LaserScan)
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.front_subscriber, self.rear_subscriber],
            queue_size,
            sync_slop,
            allow_headerless=False,
        )
        self.synchronizer.registerCallback(self._callback)

        rospy.loginfo(
            "Fusing %s and %s into %s (%s, %d bins)",
            front_topic,
            rear_topic,
            self.output_topic,
            self.target_frame,
            self.bin_count,
        )

    @staticmethod
    def _read_pose(prefix):
        return (
            float(rospy.get_param("~{}_x".format(prefix))),
            float(rospy.get_param("~{}_y".format(prefix), 0.0)),
            float(rospy.get_param("~{}_yaw".format(prefix))),
        )

    def _callback(self, front_scan, rear_scan):
        ranges = [math.inf] * self.bin_count
        intensities = [0.0] * self.bin_count

        add_scan_to_bins(
            front_scan,
            *self.front_pose,
            self.angle_min,
            self.angle_increment,
            ranges,
            intensities
        )
        add_scan_to_bins(
            rear_scan,
            *self.rear_pose,
            self.angle_min,
            self.angle_increment,
            ranges,
            intensities
        )

        output = LaserScan()
        output.header.stamp = max(front_scan.header.stamp, rear_scan.header.stamp)
        output.header.frame_id = self.target_frame
        output.angle_min = self.angle_min
        output.angle_max = self.angle_max
        output.angle_increment = self.angle_increment
        output.time_increment = 0.0
        output.scan_time = max(front_scan.scan_time, rear_scan.scan_time)
        output.range_min = self.output_range_min
        output.range_max = self.output_range_max
        output.ranges = ranges
        output.intensities = intensities
        self.publisher.publish(output)


def main():
    rospy.init_node("dual_laser_fusion")
    try:
        DualLaserFusion()
    except (KeyError, ValueError) as error:
        rospy.logfatal("Invalid dual-lidar configuration: %s", error)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()

