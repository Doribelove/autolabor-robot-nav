#!/usr/bin/env python3

"""Conservatively merge an optional LaserScan into a required primary scan.

The primary scan drives publication.  If the optional scan is absent, stale or
uses another frame, the output is a copy of the primary scan.  Consequently a
USB lidar can be plugged in or removed without changing the MID360 avoidance
path or creating a second publisher on /scan.
"""

import copy
import math

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


def valid_range(value, range_min, range_max):
    return math.isfinite(value) and range_min <= value <= range_max


def merge_scans(primary, optional):
    """Return primary with closer valid optional returns rebinned into it."""
    if primary.angle_increment <= 0.0:
        raise ValueError("primary angle_increment must be positive")
    if optional.angle_increment <= 0.0:
        raise ValueError("optional angle_increment must be positive")

    output = copy.deepcopy(primary)
    # rospy deserializes variable-length primitive arrays as tuples. Convert
    # them before replacing bins, while keeping the published ROS message type.
    output.ranges = list(output.ranges)
    output.intensities = list(output.intensities)
    # A fused scan represents the union of both sensors' measurement ranges.
    # In particular, keep the LD19's useful near field instead of discarding
    # returns below the MID360 minimum range. MID360-only passthrough still
    # retains the original metadata because merge_scans is not called then.
    output.range_min = min(primary.range_min, optional.range_min)
    output.range_max = max(primary.range_max, optional.range_max)
    bin_count = len(output.ranges)
    if bin_count == 0:
        return output

    primary_has_intensity = len(output.intensities) == bin_count
    optional_has_intensity = len(optional.intensities) == len(optional.ranges)

    for optional_index, optional_range in enumerate(optional.ranges):
        if not valid_range(optional_range, optional.range_min, optional.range_max):
            continue
        angle = optional.angle_min + optional_index * optional.angle_increment
        primary_index = int(round((angle - output.angle_min) / output.angle_increment))
        if primary_index < 0 or primary_index >= bin_count:
            continue

        primary_range = output.ranges[primary_index]
        if (not valid_range(primary_range, output.range_min, output.range_max)
                or optional_range < primary_range):
            output.ranges[primary_index] = optional_range
            if primary_has_intensity:
                output.intensities[primary_index] = (
                    optional.intensities[optional_index]
                    if optional_has_intensity else 0.0)

    return output


class OptionalLaserScanFusion:
    def __init__(self):
        self.primary_topic = rospy.get_param("~primary_topic", "/mid360/scan")
        self.optional_topic = rospy.get_param("~optional_topic", "/dual_lidar/scan")
        self.output_topic = rospy.get_param("~output_topic", "/scan")
        self.optional_enabled = rospy.get_param("~optional_enabled", True)
        self.optional_timeout = rospy.get_param("~optional_timeout", 0.35)

        if self.optional_timeout <= 0.0:
            raise ValueError("~optional_timeout must be positive")

        self.latest_optional = None
        self.latest_optional_receipt = None
        self.last_active = None

        self.publisher = rospy.Publisher(
            self.output_topic, LaserScan, queue_size=1)
        self.active_publisher = rospy.Publisher(
            "/avoidance/dual_lidar_active", Bool, queue_size=1, latch=True)
        self.mode_publisher = rospy.Publisher(
            "/avoidance/source_mode", String, queue_size=1, latch=True)

        if self.optional_enabled:
            self.optional_subscriber = rospy.Subscriber(
                self.optional_topic,
                LaserScan,
                self.optional_callback,
                queue_size=1,
                tcp_nodelay=True,
            )
        else:
            self.optional_subscriber = None

        self.primary_subscriber = rospy.Subscriber(
            self.primary_topic,
            LaserScan,
            self.primary_callback,
            queue_size=5,
            tcp_nodelay=True,
        )

        rospy.loginfo(
            "optional_laserscan_fusion: %s + optional %s -> %s (enabled=%s, timeout=%.3fs)",
            self.primary_topic,
            self.optional_topic,
            self.output_topic,
            self.optional_enabled,
            self.optional_timeout,
        )
        self.publish_status(False)

    def optional_callback(self, message):
        self.latest_optional = message
        self.latest_optional_receipt = rospy.Time.now()

    def optional_is_fresh(self, primary):
        if (not self.optional_enabled or self.latest_optional is None
                or self.latest_optional_receipt is None):
            return False

        if self.latest_optional.header.frame_id != primary.header.frame_id:
            rospy.logwarn_throttle(
                5.0,
                "optional_laserscan_fusion: frame mismatch primary=%s optional=%s; using MID360 only",
                primary.header.frame_id,
                self.latest_optional.header.frame_id,
            )
            return False

        age = (rospy.Time.now() - self.latest_optional_receipt).to_sec()
        return -0.05 <= age <= self.optional_timeout

    def publish_status(self, active):
        self.active_publisher.publish(Bool(data=active))
        self.mode_publisher.publish(String(
            data="mid360+dual_ld19" if active else "mid360"))
        if active != self.last_active:
            if active:
                rospy.loginfo("avoidance scan now uses MID360 + front/rear LD19")
            elif self.last_active is not None:
                rospy.logwarn("dual LD19 unavailable or stale; avoidance fell back to MID360")
            self.last_active = active

    def primary_callback(self, primary):
        active = self.optional_is_fresh(primary)
        if active:
            try:
                output = merge_scans(primary, self.latest_optional)
            except ValueError as error:
                rospy.logwarn_throttle(
                    5.0,
                    "optional_laserscan_fusion: %s; using MID360 only",
                    str(error),
                )
                output = copy.deepcopy(primary)
                active = False
        else:
            output = copy.deepcopy(primary)

        self.publisher.publish(output)
        self.publish_status(active)


def main():
    rospy.init_node("avoidance_scan_fusion")
    OptionalLaserScanFusion()
    rospy.spin()


if __name__ == "__main__":
    main()
