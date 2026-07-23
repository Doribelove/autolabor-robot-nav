#!/usr/bin/env python3
"""Deterministic pinhole-camera/FOD simulator for projection and tracker tests."""

from math import pi, sin

import cv2
import numpy as np
import rospy
import tf2_ros
from autolabor_fod_msgs.msg import FodDetection, FodDetectionArray
from cv_bridge import CvBridge
from geometry_msgs.msg import Point32, PointStamped, TransformStamped
from sensor_msgs.msg import CameraInfo, Image, RegionOfInterest
from tf.transformations import quaternion_from_matrix

from autolabor_fod_vision.projection import (
    ProjectionError,
    optical_rotation_base,
    project_ground_point_to_pixel,
)


class SimSourceNode:
    def __init__(self):
        self.width = int(rospy.get_param("~width", 960))
        self.height = int(rospy.get_param("~height", 540))
        self.fps = float(rospy.get_param("~fps", 10.0))
        self.fx = float(rospy.get_param("~fx", 700.0))
        self.fy = float(rospy.get_param("~fy", 700.0))
        self.cx = float(rospy.get_param("~cx", self.width / 2.0))
        self.cy = float(rospy.get_param("~cy", self.height / 2.0))
        self.target_x = float(rospy.get_param("~target_x", 2.0))
        self.target_y = float(rospy.get_param("~target_y", 0.25))
        self.target_y_amplitude = float(
            rospy.get_param("~target_y_amplitude", 0.0)
        )
        self.camera_x = float(rospy.get_param("~camera_x", 0.30))
        self.camera_y = float(rospy.get_param("~camera_y", 0.0))
        self.camera_z = float(rospy.get_param("~camera_z", 1.0))
        pitch_degrees = float(rospy.get_param("~camera_pitch_down_deg", 25.0))
        self.camera_pitch = pitch_degrees * pi / 180.0
        self.frame_id = str(
            rospy.get_param("~camera_frame", "fod_camera_optical_frame")
        )
        self.base_frame = str(rospy.get_param("~base_frame", "base_link"))
        self.tracking_frame = str(rospy.get_param("~tracking_frame", "odom"))
        self.publish_detection = bool(
            rospy.get_param("~publish_detection", True)
        )
        self.bbox_width = int(rospy.get_param("~bbox_width", 80))
        self.bbox_height = int(rospy.get_param("~bbox_height", 50))

        self.k = np.array(
            [
                [self.fx, 0.0, self.cx],
                [0.0, self.fy, self.cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        self.rotation_base_from_camera = optical_rotation_base(self.camera_pitch)
        self.camera_origin = np.array(
            [self.camera_x, self.camera_y, self.camera_z], dtype=np.float64
        )
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher(
            "/fod_camera/image_raw", Image, queue_size=1
        )
        self.info_pub = rospy.Publisher(
            "/fod_camera/camera_info", CameraInfo, queue_size=1
        )
        self.detection_pub = rospy.Publisher(
            "/fod/detections", FodDetectionArray, queue_size=1
        )
        self.debug_pub = rospy.Publisher(
            "/fod/debug/sim_image", Image, queue_size=1
        )
        self.ground_truth_pub = rospy.Publisher(
            "/fod/sim/ground_truth", PointStamped, queue_size=1
        )
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self._publish_static_transforms()
        self.start_time = rospy.Time.now()

    def _publish_static_transforms(self):
        camera_transform = TransformStamped()
        camera_transform.header.stamp = rospy.Time.now()
        camera_transform.header.frame_id = self.base_frame
        camera_transform.child_frame_id = self.frame_id
        camera_transform.transform.translation.x = self.camera_x
        camera_transform.transform.translation.y = self.camera_y
        camera_transform.transform.translation.z = self.camera_z
        matrix = np.eye(4)
        matrix[:3, :3] = self.rotation_base_from_camera
        quaternion = quaternion_from_matrix(matrix)
        camera_transform.transform.rotation.x = quaternion[0]
        camera_transform.transform.rotation.y = quaternion[1]
        camera_transform.transform.rotation.z = quaternion[2]
        camera_transform.transform.rotation.w = quaternion[3]

        transforms = [camera_transform]
        if self.tracking_frame and self.tracking_frame != self.base_frame:
            base_transform = TransformStamped()
            base_transform.header.stamp = camera_transform.header.stamp
            base_transform.header.frame_id = self.tracking_frame
            base_transform.child_frame_id = self.base_frame
            base_transform.transform.rotation.w = 1.0
            transforms.append(base_transform)
        self.static_broadcaster.sendTransform(transforms)

    def _camera_info(self, stamp):
        message = CameraInfo()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        message.width = self.width
        message.height = self.height
        message.distortion_model = "plumb_bob"
        message.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        message.K = self.k.reshape(-1).tolist()
        message.R = np.eye(3).reshape(-1).tolist()
        message.P = [
            self.fx,
            0.0,
            self.cx,
            0.0,
            0.0,
            self.fy,
            self.cy,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        return message

    def _background(self):
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        horizon = int(self.height * 0.28)
        image[:horizon, :, :] = (205, 175, 130)
        for row in range(horizon, self.height):
            ratio = (row - horizon) / max(1, self.height - horizon)
            value = int(80 + 65 * ratio)
            image[row, :, :] = (value, value, value)
        for row in range(horizon + 35, self.height, 55):
            cv2.line(image, (0, row), (self.width - 1, row), (105, 105, 105), 1)
        for column in range(0, self.width, 120):
            cv2.line(
                image,
                (int(self.cx), horizon),
                (column, self.height - 1),
                (105, 105, 105),
                1,
            )
        cv2.putText(
            image,
            "SYNTHETIC - NOT CAMERA DATA",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (20, 20, 220),
            2,
            cv2.LINE_AA,
        )
        return image

    def _build_frame(self, stamp):
        elapsed = (stamp - self.start_time).to_sec()
        target_y = self.target_y + self.target_y_amplitude * sin(0.5 * elapsed)
        target = np.array([self.target_x, target_y, 0.0], dtype=np.float64)
        image = self._background()
        detections = FodDetectionArray()
        detections.header.stamp = stamp
        detections.header.frame_id = self.frame_id
        detections.image_width = self.width
        detections.image_height = self.height
        detections.model_name = "synthetic_ground_truth"
        detections.model_task = "simulation"
        detections.inference_ms = 0.0

        try:
            u, v = project_ground_point_to_pixel(
                target,
                self.k,
                self.rotation_base_from_camera,
                self.camera_origin,
            )
            x1 = int(round(u - self.bbox_width / 2.0))
            x2 = int(round(u + self.bbox_width / 2.0))
            y2 = int(round(v))
            y1 = y2 - self.bbox_height
            visible = x2 >= 0 and x1 < self.width and y2 >= 0 and y1 < self.height
            if visible:
                x1_clip = max(0, min(self.width - 1, x1))
                x2_clip = max(x1_clip, min(self.width - 1, x2))
                y1_clip = max(0, min(self.height - 1, y1))
                y2_clip = max(y1_clip, min(self.height - 1, y2))
                polygon = np.array(
                    [
                        [x1_clip, y2_clip],
                        [x1_clip + 12, y1_clip],
                        [x2_clip - 12, y1_clip],
                        [x2_clip, y2_clip],
                    ],
                    dtype=np.int32,
                )
                cv2.fillConvexPoly(image, polygon, (0, 145, 255))
                cv2.polylines(image, [polygon], True, (0, 60, 180), 2)
                cv2.circle(image, (int(round(u)), int(round(v))), 5, (0, 0, 255), -1)

                if self.publish_detection:
                    detection = FodDetection()
                    detection.class_id = 0
                    detection.class_name = "fod"
                    detection.confidence = 0.99
                    detection.bbox = RegionOfInterest(
                        x_offset=x1_clip,
                        y_offset=y1_clip,
                        width=x2_clip - x1_clip + 1,
                        height=y2_clip - y1_clip + 1,
                        do_rectify=False,
                    )
                    detection.anchor_px = Point32(x=float(u), y=float(v), z=0.0)
                    detection.mask_px.points = [
                        Point32(x=float(point[0]), y=float(point[1]), z=0.0)
                        for point in polygon
                    ]
                    detections.detections = [detection]
        except ProjectionError as error:
            rospy.logwarn_throttle(5.0, "Synthetic target is not visible: %s", error)

        ground_truth = PointStamped()
        ground_truth.header.stamp = stamp
        ground_truth.header.frame_id = self.tracking_frame or self.base_frame
        ground_truth.point.x = float(target[0])
        ground_truth.point.y = float(target[1])
        ground_truth.point.z = 0.0
        return image, detections, ground_truth

    def run(self):
        rate = rospy.Rate(self.fps)
        while not rospy.is_shutdown():
            stamp = rospy.Time.now()
            image, detections, ground_truth = self._build_frame(stamp)
            image_message = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            image_message.header = detections.header
            self.image_pub.publish(image_message)
            self.debug_pub.publish(image_message)
            self.info_pub.publish(self._camera_info(stamp))
            self.detection_pub.publish(detections)
            self.ground_truth_pub.publish(ground_truth)
            rate.sleep()


def main():
    rospy.init_node("fod_sim_source")
    SimSourceNode().run()


if __name__ == "__main__":
    main()
