#!/usr/bin/env python3
"""Attach backend identity and display-only depth to legacy vision results.

LocateAnything remains ineligible for motion.  Its synchronized depth is
calculated only on the Qt result topic, leaving ``/fod/detections`` unchanged
for the J6M motion gate.
"""

from collections import deque
import math
import threading
import time

import numpy as np
import rospy
from autolabor_fod_msgs.msg import (
    FodDetectionArray,
    FodVisionDetection,
    FodVisionDetectionArray,
)
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import CameraInfo, Image

from autolabor_fod_vision.depth_fusion import nearest_synchronized_message
from autolabor_fod_vision.two_stage import estimate_clustered_depth


class LegacyResultAdapter:
    def __init__(self):
        self.backend_id = str(rospy.get_param("~backend_id")).strip().lower()
        if self.backend_id not in ("yolo", "locateanything"):
            raise ValueError(
                "legacy result adapter accepts only yolo or locateanything"
            )
        self.received = 0
        self.bridge = CvBridge()
        self.display_depth_enabled = bool(
            rospy.get_param("~display_depth/enabled", True)
        ) and self.backend_id == "locateanything"
        self.depth_topic = str(
            rospy.get_param(
                "~display_depth/depth_topic", "/fod_camera/depth_registered"
            )
        )
        self.camera_info_topic = str(
            rospy.get_param(
                "~display_depth/camera_info_topic", "/fod_camera/camera_info"
            )
        )
        self.depth_sync_tolerance_sec = float(
            rospy.get_param("~display_depth/sync_tolerance_sec", 0.06)
        )
        self.depth_wait_sec = float(
            rospy.get_param("~display_depth/wait_sec", 0.03)
        )
        self.depth_buffer_size = int(
            rospy.get_param("~display_depth/buffer_size", 120)
        )
        self.depth_min_m = float(
            rospy.get_param("~display_depth/min_m", 0.30)
        )
        self.depth_max_m = float(
            rospy.get_param("~display_depth/max_m", 15.0)
        )
        self.depth_min_samples = int(
            rospy.get_param("~display_depth/min_samples", 20)
        )
        self.depth_min_valid_fraction = float(
            rospy.get_param("~display_depth/min_valid_fraction", 0.12)
        )
        self.depth_bbox_inset_fraction = float(
            rospy.get_param("~display_depth/bbox_inset_fraction", 0.10)
        )
        self.depth_aggregation = str(
            rospy.get_param("~display_depth/aggregation", "median")
        ).strip().lower()
        if not self.depth_topic.startswith("/"):
            raise ValueError("depth_topic must be an absolute ROS topic")
        if not self.camera_info_topic.startswith("/"):
            raise ValueError("camera_info_topic must be an absolute ROS topic")
        if not 0.0 < self.depth_sync_tolerance_sec <= 0.20:
            raise ValueError("depth_sync_tolerance_sec must be in (0, 0.20]")
        if not 0.0 <= self.depth_wait_sec <= 0.10:
            raise ValueError("depth_wait_sec must be in [0, 0.10]")
        if not 2 <= self.depth_buffer_size <= 600:
            raise ValueError("depth_buffer_size must be in [2, 600]")
        if not 0.0 < self.depth_min_m < self.depth_max_m:
            raise ValueError("depth range is invalid")
        if self.depth_min_samples < 1:
            raise ValueError("depth_min_samples must be positive")
        if not 0.0 <= self.depth_min_valid_fraction <= 1.0:
            raise ValueError("depth_min_valid_fraction must be in [0, 1]")
        if not 0.0 <= self.depth_bbox_inset_fraction < 0.5:
            raise ValueError("depth_bbox_inset_fraction must be in [0, 0.5)")
        if self.depth_aggregation != "median":
            raise ValueError("legacy display depth aggregation must be median")

        self._sensor_condition = threading.Condition()
        self._depth_messages = deque(maxlen=self.depth_buffer_size)
        self._camera_info_messages = deque(maxlen=self.depth_buffer_size)
        self.received_depth_frames = 0
        self.received_camera_info = 0
        self.camera_info_duplicates_dropped = 0
        self.synchronized_frames = 0
        self.depth_sync_misses = 0
        self.cluster_attempts = 0
        self.valid_depth_detections = 0
        self.last_sync_delta_sec = float("nan")
        self.last_cluster_reason = ""
        self.last_error = ""
        self.publisher = rospy.Publisher(
            "/fod/vision/results", FodVisionDetectionArray, queue_size=1
        )
        self.diagnostic_publisher = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=2
        )
        self.depth_subscriber = None
        self.camera_info_subscriber = None
        if self.display_depth_enabled:
            self.depth_subscriber = rospy.Subscriber(
                self.depth_topic,
                Image,
                self._depth_callback,
                queue_size=10,
                buff_size=16 * 1024 * 1024,
            )
            self.camera_info_subscriber = rospy.Subscriber(
                self.camera_info_topic,
                CameraInfo,
                self._camera_info_callback,
                queue_size=10,
            )
        self.subscriber = rospy.Subscriber(
            "/fod/detections",
            FodDetectionArray,
            self._callback,
            queue_size=1,
        )
        rospy.set_param("~display_depth_enabled", self.display_depth_enabled)
        rospy.set_param("~display_depth_motion_isolated", True)
        rospy.set_param("~depth_buffer_size", self.depth_buffer_size)
        rospy.set_param("~depth_aggregation", self.depth_aggregation)
        rospy.set_param("~depth_cluster_method", "organized_point_cloud_geometry")
        rospy.loginfo(
            "FOD UI result adapter ready: backend_id=%s display_depth=%s "
            "depth=%s camera_info=%s buffer=%d tolerance=%.1fms "
            "aggregation=%s output=/fod/vision/results only",
            self.backend_id,
            self.display_depth_enabled,
            self.depth_topic,
            self.camera_info_topic,
            self.depth_buffer_size,
            self.depth_sync_tolerance_sec * 1000.0,
            self.depth_aggregation,
        )

    def _depth_callback(self, message: Image) -> None:
        # Keep this callback deliberately light: retain only a bounded reference
        # to the source-stamped registered depth message.
        with self._sensor_condition:
            self.received_depth_frames += 1
            self._depth_messages.append(message)
            self._sensor_condition.notify_all()

    def _camera_info_callback(self, message: CameraInfo) -> None:
        with self._sensor_condition:
            self.received_camera_info += 1
            if self._camera_info_messages:
                previous = self._camera_info_messages[-1]
                if (
                    previous.header.frame_id == message.header.frame_id
                    and previous.header.stamp == message.header.stamp
                ):
                    # The ZED alias currently emits the same CameraInfo stamp up
                    # to three times.  Deduplicate it so a 120-entry history
                    # represents 120 RGB source frames rather than ~40 frames.
                    self.camera_info_duplicates_dropped += 1
                    return
            self._camera_info_messages.append(message)
            self._sensor_condition.notify_all()

    def _matching_sensor_bundle(self, source: FodDetectionArray):
        source_stamp = float(source.header.stamp.to_sec())
        source_frame = str(source.header.frame_id)
        deadline = time.monotonic() + self.depth_wait_sec
        with self._sensor_condition:
            while True:
                depth, depth_delta = nearest_synchronized_message(
                    self._depth_messages,
                    source_stamp,
                    source_frame,
                    self.depth_sync_tolerance_sec,
                )
                camera_info, info_delta = nearest_synchronized_message(
                    self._camera_info_messages,
                    source_stamp,
                    source_frame,
                    self.depth_sync_tolerance_sec,
                )
                if depth is not None and camera_info is not None:
                    return depth, camera_info, max(depth_delta, info_delta)
                remaining = deadline - time.monotonic()
                if remaining <= 0.0 or rospy.is_shutdown():
                    return None, None, float("nan")
                self._sensor_condition.wait(timeout=remaining)

    def _decode_depth(self, message: Image) -> np.ndarray:
        image = np.asarray(
            self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
        )
        if message.encoding == "32FC1":
            return image.astype(np.float32, copy=False)
        if message.encoding in ("16UC1", "mono16"):
            return image.astype(np.float32) * 0.001
        raise ValueError(
            "unsupported registered depth encoding {}".format(message.encoding)
        )

    def _publish_diagnostic(self, synchronized: bool) -> None:
        status = DiagnosticStatus()
        status.name = "fod_vision/display_depth_adapter"
        status.hardware_id = self.backend_id
        if not self.display_depth_enabled:
            status.level = DiagnosticStatus.OK
            status.message = "legacy source depth passthrough active"
        elif synchronized:
            status.level = DiagnosticStatus.OK
            status.message = (
                "source-stamped display-only point-cloud depth fusion active"
            )
        else:
            status.level = DiagnosticStatus.WARN
            status.message = (
                "display-only depth unavailable for the RGB source timestamp"
            )
        status.values = [
            KeyValue("backend_id", self.backend_id),
            KeyValue("display_depth_enabled", str(self.display_depth_enabled)),
            KeyValue("display_depth_motion_isolated", "True"),
            KeyValue("depth_topic", self.depth_topic),
            KeyValue("camera_info_topic", self.camera_info_topic),
            KeyValue("depth_buffer_size", str(self.depth_buffer_size)),
            KeyValue("depth_aggregation", self.depth_aggregation),
            KeyValue("depth_cluster_method", "organized_point_cloud_geometry"),
            KeyValue("received_depth_frames", str(self.received_depth_frames)),
            KeyValue("received_camera_info", str(self.received_camera_info)),
            KeyValue(
                "camera_info_duplicates_dropped",
                str(self.camera_info_duplicates_dropped),
            ),
            KeyValue(
                "buffered_camera_info", str(len(self._camera_info_messages))
            ),
            KeyValue("synchronized_frames", str(self.synchronized_frames)),
            KeyValue("depth_sync_misses", str(self.depth_sync_misses)),
            KeyValue("cluster_attempts", str(self.cluster_attempts)),
            KeyValue(
                "valid_depth_detections", str(self.valid_depth_detections)
            ),
            KeyValue(
                "last_depth_sync_delta_ms",
                "{:.3f}".format(self.last_sync_delta_sec * 1000.0)
                if math.isfinite(self.last_sync_delta_sec)
                else "N/A",
            ),
            KeyValue("last_cluster_reason", self.last_cluster_reason or "N/A"),
            KeyValue("last_error", self.last_error or "N/A"),
        ]
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status = [status]
        self.diagnostic_publisher.publish(array)

    def _callback(self, source: FodDetectionArray) -> None:
        self.received += 1
        depth_message = None
        camera_info = None
        depth_image = None
        depth_delta = float("nan")
        self.last_error = ""
        self.last_cluster_reason = ""
        if self.display_depth_enabled:
            depth_message, camera_info, depth_delta = self._matching_sensor_bundle(
                source
            )
            if depth_message is not None and camera_info is not None:
                try:
                    depth_image = self._decode_depth(depth_message)
                    expected_shape = (int(source.image_height), int(source.image_width))
                    if depth_image.shape != expected_shape:
                        raise ValueError(
                            "registered depth shape {} does not match RGB {}".format(
                                depth_image.shape, expected_shape
                            )
                        )
                    if (
                        int(camera_info.width) != int(source.image_width)
                        or int(camera_info.height) != int(source.image_height)
                    ):
                        raise ValueError(
                            "CameraInfo resolution {}x{} does not match RGB {}x{}".format(
                                camera_info.width,
                                camera_info.height,
                                source.image_width,
                                source.image_height,
                            )
                        )
                except Exception as error:
                    self.last_error = str(error)
                    depth_message = None
                    camera_info = None
                    depth_image = None
                    depth_delta = float("nan")
            if depth_image is None:
                self.depth_sync_misses += 1
                self.last_sync_delta_sec = float("nan")
            else:
                self.synchronized_frames += 1
                self.last_sync_delta_sec = depth_delta

        output = FodVisionDetectionArray()
        output.header = source.header
        output.backend_id = self.backend_id
        output.image_width = source.image_width
        output.image_height = source.image_height
        output.model_name = source.model_name
        output.detector_model_sha256 = source.model_sha256
        output.classifier_model_sha256 = ""
        output.detector_inference_ms = source.inference_ms
        output.classifier_inference_ms = 0.0
        output.total_latency_ms = source.inference_ms
        output.received_frames = self.received
        output.processed_frames = self.received
        output.dropped_frames = 0
        output.expired_frames = 0
        if self.display_depth_enabled:
            output.depth_synchronized = depth_image is not None
            output.depth_sync_delta_sec = (
                depth_delta if depth_image is not None else float("nan")
            )
            if depth_image is not None:
                output.depth_header = depth_message.header
        else:
            output.depth_synchronized = source.depth_synchronized
            output.depth_header = source.depth_header
            output.depth_sync_delta_sec = source.depth_sync_delta_sec

        frame_valid_depths = 0
        for detection in source.detections:
            result = FodVisionDetection()
            result.backend_id = self.backend_id
            result.object_id = 0
            result.track_id = 0
            result.detector_class_id = detection.class_id
            result.material_class = detection.class_name
            result.detect_confidence = detection.confidence
            # Existing backends are one-stage detectors.  Do not mislabel their
            # detector confidence as a classifier confidence.
            result.classify_confidence = float("nan")
            result.class_probabilities = [0.0] * 5
            result.bbox = detection.bbox
            if self.display_depth_enabled:
                result.depth_valid = False
                result.depth_m = float("nan")
                result.depth_mad_m = float("nan")
                result.depth_sample_count = 0
                if depth_image is not None and camera_info is not None:
                    self.cluster_attempts += 1
                    x1 = float(detection.bbox.x_offset)
                    y1 = float(detection.bbox.y_offset)
                    x2 = x1 + float(detection.bbox.width)
                    y2 = y1 + float(detection.bbox.height)
                    estimate = estimate_clustered_depth(
                        depth_image,
                        (x1, y1, x2, y2),
                        camera_info.K,
                        minimum_depth_m=self.depth_min_m,
                        maximum_depth_m=self.depth_max_m,
                        inset_fraction=self.depth_bbox_inset_fraction,
                        minimum_samples=self.depth_min_samples,
                        minimum_valid_fraction=self.depth_min_valid_fraction,
                        aggregation=self.depth_aggregation,
                    )
                    self.last_cluster_reason = estimate.reason
                    result.depth_valid = bool(
                        estimate.valid and math.isfinite(estimate.depth_m)
                    )
                    result.depth_m = float(estimate.depth_m)
                    result.depth_mad_m = float(estimate.mad_m)
                    result.depth_sample_count = int(estimate.sample_count)
            else:
                result.depth_valid = bool(
                    source.depth_synchronized
                    and detection.depth_valid
                    and math.isfinite(detection.depth_m)
                )
                result.depth_m = detection.depth_m
                result.depth_mad_m = detection.depth_mad_m
                result.depth_sample_count = detection.depth_sample_count
            if result.depth_valid:
                frame_valid_depths += 1
            result.world_position_valid = False
            result.world_frame = ""
            result.state = "ACTIVE"
            result.last_observed = source.header.stamp
            output.detections.append(result)
        self.valid_depth_detections += frame_valid_depths
        self.publisher.publish(output)
        self._publish_diagnostic(output.depth_synchronized)


def main():
    rospy.init_node("fod_vision_result_adapter", anonymous=False)
    LegacyResultAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
