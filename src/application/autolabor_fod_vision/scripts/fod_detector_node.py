#!/usr/bin/env python3
"""Latest-frame YOLO inference with registered ZED depth fusion."""

from collections import deque
import math
import os
import threading
import time
import traceback

import rospy
from autolabor_fod_msgs.msg import FodDetection, FodDetectionArray
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point32
from sensor_msgs.msg import Image, RegionOfInterest

from autolabor_fod_vision.clip_filter import (
    ClipDetectionFilter,
    ClipFilterStats,
    OfficialClipRuntime,
)
from autolabor_fod_vision.detector import UltralyticsDetector, annotate_image
from autolabor_fod_vision.depth_fusion import estimate_detection_depth


def _csv_ints(value):
    if isinstance(value, list):
        return [int(item) for item in value]
    return [int(item.strip()) for item in str(value).split(",") if item.strip()]


def _csv_strings(value):
    if isinstance(value, list):
        return [str(item).strip() for item in value if str(item).strip()]
    return [item.strip() for item in str(value).split(",") if item.strip()]


def _prompt_strings(value, name):
    values = value if isinstance(value, list) else [value]
    prompts = [str(item).strip() for item in values if str(item).strip()]
    if not prompts:
        raise ValueError("{} must contain at least one prompt".format(name))
    return prompts


class DetectorNode:
    def __init__(self):
        weights = str(rospy.get_param("~weights"))
        ultralytics_root = str(
            rospy.get_param(
                "~ultralytics_root",
                os.environ.get("AUTOLABOR_FOD_ULTRALYTICS_ROOT", ""),
            )
        )
        require_gam = bool(rospy.get_param("~require_gam", False))
        self.runtime_token = str(rospy.get_param("~runtime_token", ""))
        self.smoke_test_only = bool(rospy.get_param("~smoke_test_only", True))
        required_names = _csv_strings(
            rospy.get_param("~required_class_names", "fod")
        )
        self.debug_every_n = max(1, int(rospy.get_param("~debug_every_n", 1)))
        self.enable_depth_fusion = bool(
            rospy.get_param("~enable_depth_fusion", False)
        )
        self.depth_topic = str(
            rospy.get_param("~depth_topic", "/fod_camera/depth_registered")
        )
        self.depth_sync_tolerance_sec = float(
            rospy.get_param("~depth_sync_tolerance_sec", 0.06)
        )
        self.depth_wait_sec = float(rospy.get_param("~depth_wait_sec", 0.03))
        self.depth_min_m = float(rospy.get_param("~depth_min_m", 0.30))
        self.depth_max_m = float(rospy.get_param("~depth_max_m", 15.0))
        self.depth_min_samples = int(rospy.get_param("~depth_min_samples", 20))
        self.depth_min_valid_fraction = float(
            rospy.get_param("~depth_min_valid_fraction", 0.20)
        )
        self.depth_bbox_inset_fraction = float(
            rospy.get_param("~depth_bbox_inset_fraction", 0.18)
        )
        if not self.depth_topic.startswith("/"):
            raise ValueError("depth_topic must be an absolute ROS topic")
        if not 0.0 < self.depth_sync_tolerance_sec <= 0.20:
            raise ValueError("depth_sync_tolerance_sec must be in (0, 0.20]")
        if not 0.0 <= self.depth_wait_sec <= 0.10:
            raise ValueError("depth_wait_sec must be in [0, 0.10]")
        if not 0.0 < self.depth_min_m < self.depth_max_m:
            raise ValueError("depth range is invalid")
        if self.depth_min_samples < 1:
            raise ValueError("depth_min_samples must be positive")
        if not 0.0 <= self.depth_min_valid_fraction <= 1.0:
            raise ValueError("depth_min_valid_fraction must be in [0, 1]")
        if not 0.0 <= self.depth_bbox_inset_fraction < 0.5:
            raise ValueError("depth_bbox_inset_fraction must be in [0, 0.5)")
        classes = _csv_ints(rospy.get_param("~classes", ""))
        self.bridge = CvBridge()
        self.detector = UltralyticsDetector(
            weights=weights,
            device=str(rospy.get_param("~device", "auto")),
            image_size=int(rospy.get_param("~image_size", 640)),
            confidence=float(rospy.get_param("~confidence", 0.25)),
            iou=float(rospy.get_param("~iou", 0.45)),
            max_detections=int(rospy.get_param("~max_detections", 100)),
            classes=classes,
            half=bool(rospy.get_param("~half", True)),
            warmup=bool(rospy.get_param("~warmup", True)),
            expected_sha256=str(
                rospy.get_param("~expected_model_sha256", "")
            ),
            ultralytics_root=ultralytics_root,
            require_gam=require_gam,
        )
        available = set(self.detector.names.values())
        missing = [name for name in required_names if name not in available]
        if missing and not self.smoke_test_only:
            raise RuntimeError(
                "production model is missing required classes: {}".format(
                    ", ".join(missing)
                )
            )
        if missing:
            rospy.logwarn(
                "SMOKE TEST ONLY: model has no required class(es) %s",
                ", ".join(missing),
            )

        self.enable_clip_filter = bool(
            rospy.get_param("~enable_clip_filter", False)
        )
        self.clip_filter = None
        self.clip_runtime = None
        if self.enable_clip_filter:
            positive_prompts = _prompt_strings(
                rospy.get_param("~clip_positive_prompts"),
                "clip_positive_prompts",
            )
            negative_prompts = _prompt_strings(
                rospy.get_param("~clip_negative_prompts"),
                "clip_negative_prompts",
            )
            self.clip_runtime = OfficialClipRuntime(
                weights=str(rospy.get_param("~clip_weights")),
                expected_sha256=str(
                    rospy.get_param("~clip_expected_model_sha256")
                ),
                clip_python_root=str(rospy.get_param("~clip_python_root")),
                positive_prompts=positive_prompts,
                negative_prompts=negative_prompts,
                model_name=str(
                    rospy.get_param("~clip_model_name", "ViT-B/32")
                ),
                source_commit=str(
                    rospy.get_param("~clip_source_commit", "")
                ),
                device=str(rospy.get_param("~clip_device", "auto")),
                warmup=bool(rospy.get_param("~clip_warmup", True)),
            )
            self.clip_filter = ClipDetectionFilter(
                self.clip_runtime,
                low_confidence=float(
                    rospy.get_param("~clip_low_confidence", 0.20)
                ),
                high_confidence=float(
                    rospy.get_param("~clip_high_confidence", 0.60)
                ),
                positive_probability=float(
                    rospy.get_param("~clip_positive_probability", 0.50)
                ),
                crop_padding_fraction=float(
                    rospy.get_param("~clip_crop_padding_fraction", 0.10)
                ),
            )

        self.detection_pub = rospy.Publisher(
            "/fod/detections", FodDetectionArray, queue_size=1
        )
        self.debug_pub = rospy.Publisher(
            "/fod/debug/image", Image, queue_size=1
        )
        self.diagnostic_pub = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=2
        )
        self._condition = threading.Condition()
        self._latest_message = None
        self._depth_condition = threading.Condition()
        self._depth_messages = deque(maxlen=45)
        self._stopping = False
        self.received_frames = 0
        self.processed_frames = 0
        self.dropped_frames = 0
        self.last_inference_ms = 0.0
        self.last_clip_stats = ClipFilterStats()
        self.last_error = ""
        self.received_depth_frames = 0
        self.synchronized_depth_frames = 0
        self.depth_missing_frames = 0
        self.last_depth_sync_delta_sec = float("nan")
        self.last_depth_valid_detections = 0
        self.depth_subscriber = None
        if self.enable_depth_fusion:
            self.depth_subscriber = rospy.Subscriber(
                self.depth_topic,
                Image,
                self._depth_callback,
                queue_size=10,
                buff_size=16 * 1024 * 1024,
            )
        self.subscriber = rospy.Subscriber(
            "/fod_camera/image_raw",
            Image,
            self._image_callback,
            queue_size=1,
            buff_size=16 * 1024 * 1024,
        )
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()
        rospy.on_shutdown(self.shutdown)
        rospy.set_param(
            "~ultralytics_import_path", self.detector.ultralytics_path
        )
        rospy.set_param(
            "~ultralytics_version", self.detector.ultralytics_version
        )
        rospy.set_param("~gam_layer_count", self.detector.gam_layer_count)
        rospy.set_param("~clip_filter_active", self.enable_clip_filter)
        if self.clip_runtime is not None:
            rospy.set_param(
                "~clip_import_path", self.clip_runtime.clip_import_path
            )
            rospy.set_param(
                "~clip_model_sha256", self.clip_runtime.weights_sha256
            )
        rospy.set_param("~ready_token", self.runtime_token)
        rospy.loginfo(
            "FOD detector ready: model=%s task=%s device=%s classes=%s sha256=%s "
            "ultralytics_version=%s ultralytics_path=%s gam_layers=%d "
            "depth_fusion=%s depth_topic=%s",
            self.detector.model_name,
            self.detector.task,
            self.detector.device,
            ",".join(self.detector.names.values()),
            self.detector.model_sha256,
            self.detector.ultralytics_version,
            self.detector.ultralytics_path,
            self.detector.gam_layer_count,
            self.enable_depth_fusion,
            self.depth_topic,
        )
        if self.clip_runtime is not None:
            rospy.loginfo(
                "CLIP post-filter ready: model=%s device=%s weights_sha256=%s "
                "source_commit=%s import_path=%s gates=<%.2f/%.2f> "
                "positive_probability=%.2f prompts=%d+%d",
                self.clip_runtime.model_name,
                self.clip_runtime.device,
                self.clip_runtime.weights_sha256,
                self.clip_runtime.source_commit or "unknown",
                self.clip_runtime.clip_import_path,
                self.clip_filter.low_confidence,
                self.clip_filter.high_confidence,
                self.clip_filter.positive_probability,
                len(self.clip_runtime.positive_prompts),
                len(self.clip_runtime.negative_prompts),
            )

    def _image_callback(self, message):
        with self._condition:
            self.received_frames += 1
            if self._latest_message is not None:
                self.dropped_frames += 1
            self._latest_message = message
            self._condition.notify()

    def _depth_callback(self, message):
        with self._depth_condition:
            self.received_depth_frames += 1
            self._depth_messages.append(message)
            self._depth_condition.notify_all()

    def _matching_depth_message(self, image_message):
        if not self.enable_depth_fusion:
            return None, float("nan")
        image_stamp = float(image_message.header.stamp.to_sec())
        if not math.isfinite(image_stamp) or image_stamp <= 0.0:
            return None, float("nan")
        deadline = time.monotonic() + self.depth_wait_sec
        with self._depth_condition:
            while True:
                compatible = [
                    item
                    for item in self._depth_messages
                    if item.header.frame_id == image_message.header.frame_id
                ]
                if compatible:
                    best = min(
                        compatible,
                        key=lambda item: abs(
                            float(item.header.stamp.to_sec()) - image_stamp
                        ),
                    )
                    delta = abs(float(best.header.stamp.to_sec()) - image_stamp)
                    if math.isfinite(delta) and delta <= self.depth_sync_tolerance_sec:
                        return best, delta
                remaining = deadline - time.monotonic()
                if remaining <= 0.0 or self._stopping:
                    return None, float("nan")
                self._depth_condition.wait(timeout=remaining)

    def _to_ros_detection(self, detection, width, height):
        message = FodDetection()
        message.class_id = detection.class_id
        message.class_name = detection.class_name
        message.confidence = detection.confidence
        x1 = max(0, min(width - 1, int(round(detection.xmin))))
        y1 = max(0, min(height - 1, int(round(detection.ymin))))
        x2 = max(x1, min(width - 1, int(round(detection.xmax))))
        y2 = max(y1, min(height - 1, int(round(detection.ymax))))
        message.bbox = RegionOfInterest(
            x_offset=x1,
            y_offset=y1,
            height=y2 - y1 + 1,
            width=x2 - x1 + 1,
            do_rectify=False,
        )
        message.anchor_px = Point32(
            x=float(max(0.0, min(width - 1.0, detection.anchor_u))),
            y=float(max(0.0, min(height - 1.0, detection.anchor_v))),
            z=0.0,
        )
        message.mask_px.points = [
            Point32(
                x=float(max(0.0, min(width - 1.0, point[0]))),
                y=float(max(0.0, min(height - 1.0, point[1]))),
                z=0.0,
            )
            for point in detection.mask
        ]
        message.depth_valid = bool(detection.depth_valid)
        message.depth_m = float(detection.depth_m)
        message.depth_mad_m = float(detection.depth_mad_m)
        message.depth_sample_count = int(detection.depth_sample_count)
        message.depth_valid_fraction = float(detection.depth_valid_fraction)
        return message

    def _publish_diagnostic(self, level, text):
        status = DiagnosticStatus()
        status.name = "fod_vision/detector"
        status.hardware_id = self.detector.model_sha256
        status.level = level
        status.message = text
        status.values = [
            KeyValue("model", self.detector.model_name),
            KeyValue("task", self.detector.task),
            KeyValue("device", self.detector.device),
            KeyValue(
                "ultralytics_version", self.detector.ultralytics_version
            ),
            KeyValue(
                "ultralytics_import_path", self.detector.ultralytics_path
            ),
            KeyValue("gam_layer_count", str(self.detector.gam_layer_count)),
            KeyValue("classes", ",".join(self.detector.names.values())),
            KeyValue("smoke_test_only", str(self.smoke_test_only)),
            KeyValue("received_frames", str(self.received_frames)),
            KeyValue("processed_frames", str(self.processed_frames)),
            KeyValue("dropped_frames", str(self.dropped_frames)),
            KeyValue("last_inference_ms", "{:.2f}".format(self.last_inference_ms)),
            KeyValue("clip_filter_enabled", str(self.enable_clip_filter)),
            KeyValue(
                "clip_candidates", str(self.last_clip_stats.clip_candidates)
            ),
            KeyValue(
                "clip_high_confidence_kept",
                str(self.last_clip_stats.high_confidence_kept),
            ),
            KeyValue("clip_kept", str(self.last_clip_stats.clip_kept)),
            KeyValue("clip_dropped", str(self.last_clip_stats.clip_dropped)),
            KeyValue(
                "clip_low_confidence_dropped",
                str(self.last_clip_stats.low_confidence_dropped),
            ),
            KeyValue(
                "clip_invalid_crop_dropped",
                str(self.last_clip_stats.invalid_crop_dropped),
            ),
            KeyValue(
                "last_clip_inference_ms",
                "{:.2f}".format(self.last_clip_stats.clip_inference_ms),
            ),
            KeyValue("depth_fusion_enabled", str(self.enable_depth_fusion)),
            KeyValue("depth_topic", self.depth_topic),
            KeyValue("received_depth_frames", str(self.received_depth_frames)),
            KeyValue(
                "synchronized_depth_frames", str(self.synchronized_depth_frames)
            ),
            KeyValue("depth_missing_frames", str(self.depth_missing_frames)),
            KeyValue(
                "last_depth_sync_delta_ms",
                "{:.2f}".format(1000.0 * self.last_depth_sync_delta_sec)
                if math.isfinite(self.last_depth_sync_delta_sec)
                else "N/A",
            ),
            KeyValue(
                "last_depth_valid_detections",
                str(self.last_depth_valid_detections),
            ),
        ]
        if self.clip_runtime is not None:
            status.values.extend(
                [
                    KeyValue("clip_model", self.clip_runtime.model_name),
                    KeyValue("clip_device", self.clip_runtime.device),
                    KeyValue(
                        "clip_weights_sha256",
                        self.clip_runtime.weights_sha256,
                    ),
                    KeyValue(
                        "clip_source_commit",
                        self.clip_runtime.source_commit or "unknown",
                    ),
                    KeyValue(
                        "clip_import_path",
                        self.clip_runtime.clip_import_path,
                    ),
                ]
            )
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status = [status]
        self.diagnostic_pub.publish(array)

    def _process(self, image_message):
        image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
        result = self.detector.predict(image)
        self.last_inference_ms = result.inference_ms
        detections = result.detections
        if self.clip_filter is not None:
            clip_result = self.clip_filter.filter(image, detections)
            detections = clip_result.detections
            self.last_clip_stats = clip_result.stats
        else:
            self.last_clip_stats = ClipFilterStats(
                input_count=len(detections),
                output_count=len(detections),
            )
        depth_message, depth_delta = self._matching_depth_message(image_message)
        depth_image = None
        depth_error = ""
        if depth_message is not None:
            try:
                converted = self.bridge.imgmsg_to_cv2(
                    depth_message, desired_encoding="32FC1"
                )
                if converted.shape != image.shape[:2]:
                    raise ValueError(
                        "registered depth is {}x{}, RGB is {}x{}".format(
                            converted.shape[1],
                            converted.shape[0],
                            image.shape[1],
                            image.shape[0],
                        )
                    )
                depth_image = converted
            except Exception as error:
                depth_error = str(error)

        valid_depth_detections = 0
        for item in detections:
            if depth_image is None:
                continue
            estimate = estimate_detection_depth(
                depth_image,
                (item.xmin, item.ymin, item.xmax, item.ymax),
                item.mask,
                min_depth_m=self.depth_min_m,
                max_depth_m=self.depth_max_m,
                min_samples=self.depth_min_samples,
                min_valid_fraction=self.depth_min_valid_fraction,
                bbox_inset_fraction=self.depth_bbox_inset_fraction,
            )
            item.depth_valid = estimate.valid
            item.depth_m = estimate.depth_m
            item.depth_mad_m = estimate.mad_m
            item.depth_sample_count = estimate.sample_count
            item.depth_valid_fraction = estimate.valid_fraction
            if estimate.valid:
                valid_depth_detections += 1

        depth_synchronized = depth_image is not None
        if self.enable_depth_fusion:
            if depth_synchronized:
                self.synchronized_depth_frames += 1
                self.last_depth_sync_delta_sec = depth_delta
            else:
                self.depth_missing_frames += 1
                self.last_depth_sync_delta_sec = float("nan")
        self.last_depth_valid_detections = valid_depth_detections
        output = FodDetectionArray()
        output.header = image_message.header
        output.image_width = image.shape[1]
        output.image_height = image.shape[0]
        output.model_name = self.detector.model_name
        output.model_sha256 = self.detector.model_sha256
        output.model_task = self.detector.task
        output.inference_ms = result.inference_ms
        output.depth_synchronized = depth_synchronized
        output.depth_sync_delta_sec = (
            depth_delta if depth_synchronized else float("nan")
        )
        if depth_synchronized:
            output.depth_header = depth_message.header
        output.detections = [
            self._to_ros_detection(item, image.shape[1], image.shape[0])
            for item in detections
        ]
        self.detection_pub.publish(output)

        self.processed_frames += 1
        if self.processed_frames % self.debug_every_n == 0:
            banner = (
                "SMOKE ONLY {}".format(self.detector.model_name)
                if self.smoke_test_only
                else self.detector.model_name
            )
            if self.enable_depth_fusion:
                banner += " | DEPTH {}".format(
                    "SYNC" if depth_synchronized else "MISSING"
                )
            if self.enable_clip_filter:
                banner += " | CLIP"
            debug = annotate_image(
                image, detections, result.inference_ms, banner
            )
            debug_message = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            debug_message.header = image_message.header
            self.debug_pub.publish(debug_message)
        diagnostic_level = DiagnosticStatus.OK
        diagnostic_text = "inference and depth fusion active"
        if self.enable_depth_fusion and not depth_synchronized:
            diagnostic_level = DiagnosticStatus.WARN
            diagnostic_text = "inference active; registered depth unavailable"
            if depth_error:
                diagnostic_text += ": " + depth_error
        elif not self.enable_depth_fusion:
            diagnostic_text = "inference active; depth fusion disabled"
        if self.enable_clip_filter:
            diagnostic_text = "YOLO and CLIP filtering active; " + diagnostic_text
        self._publish_diagnostic(diagnostic_level, diagnostic_text)

    def _worker_loop(self):
        while not rospy.is_shutdown():
            with self._condition:
                while self._latest_message is None and not self._stopping:
                    self._condition.wait(timeout=0.5)
                if self._stopping:
                    return
                message = self._latest_message
                self._latest_message = None
            try:
                self._process(message)
            except Exception as error:
                self.last_error = str(error)
                rospy.logerr_throttle(
                    5.0, "FOD detector inference failed: %s", traceback.format_exc()
                )
                self._publish_diagnostic(DiagnosticStatus.ERROR, str(error))

    def shutdown(self):
        with self._condition:
            self._stopping = True
            self._condition.notify_all()
        with self._depth_condition:
            self._depth_condition.notify_all()
        if (
            hasattr(self, "worker")
            and self.worker.is_alive()
            and threading.current_thread() is not self.worker
        ):
            self.worker.join(timeout=2.0)


def main():
    rospy.init_node("fod_detector")
    try:
        DetectorNode()
        rospy.spin()
    except Exception as error:
        rospy.logfatal(
            "FOD detector failed to start: %s\n%s",
            error,
            traceback.format_exc(),
        )
        raise


if __name__ == "__main__":
    main()
