#!/usr/bin/env python3
"""Latest-frame two-stage trash detection, material classification and depth map."""

from collections import deque
from dataclasses import dataclass
import math
import os
from pathlib import Path
import threading
import time
import traceback
from typing import Sequence, Tuple

import cv2
import numpy as np
import rospy
import tf2_ros
from autolabor_fod_msgs.msg import (
    FodDetection,
    FodDetectionArray,
    FodVisionDetection,
    FodVisionDetectionArray,
)
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from dynamic_reconfigure.srv import Reconfigure
from geometry_msgs.msg import Point, Point32
from sensor_msgs.msg import CameraInfo, Image, RegionOfInterest
from tf.transformations import quaternion_matrix

from autolabor_fod_vision.confidence_control import (
    CONFIDENCE_SERVICE,
    GLOBAL_CONFIDENCE_PARAM,
    DetectionConfidenceController,
    validate_detection_confidence,
)
from autolabor_fod_vision.two_stage import (
    DepthClusterEstimate,
    LatestFrameSlot,
    MATERIAL_CLASSES,
    ObjectObservation,
    WorldObjectMap,
    appearance_histogram,
    context_crop,
    crop_sharpness,
    estimate_clustered_depth,
)
from autolabor_fod_vision.two_stage_runtime import TwoStageUltralyticsRuntime


BACKEND_ID = "detect_and_classify"


@dataclass(frozen=True)
class FrameEnvelope:
    message: Image
    stamp_sec: float
    frame_id: str


@dataclass(frozen=True)
class SourceTransform:
    matrix: np.ndarray
    target_frame: str


def _finite_stamp(message) -> float:
    value = float(message.header.stamp.to_sec())
    return value if math.isfinite(value) and value > 0.0 else float("nan")


def _percentile(values: Sequence[float], percentile: float) -> float:
    if not values:
        return float("nan")
    return float(np.percentile(np.asarray(values, dtype=np.float64), percentile))


class DetectAndClassifyNode:
    def __init__(self):
        self.bridge = CvBridge()
        self._shutdown_lock = threading.Lock()
        self._stopping = False
        self._models_released = False
        self._p = lambda name, default=None: rospy.get_param(
            "~detect_and_classify/{}".format(name), default
        )

        if not bool(self._p("enabled", True)):
            raise RuntimeError("detect_and_classify is disabled in configuration")
        queue_size = int(self._p("runtime/frame_queue_size", 1))
        drop_old = bool(self._p("runtime/drop_old_frames", True))
        if queue_size != 1 or not drop_old:
            raise ValueError(
                "detect_and_classify requires frame_queue_size=1 and drop_old_frames=true"
            )
        self.target_fps = float(self._p("runtime/target_fps", 8.0))
        self.minimum_fps = float(self._p("runtime/minimum_fps", 5.0))
        self.max_frame_age_sec = float(
            self._p("runtime/max_frame_age_ms", 500.0)
        ) / 1000.0
        self.max_result_age_sec = float(
            self._p("runtime/max_result_age_ms", 500.0)
        ) / 1000.0
        self.warmup_frames = int(self._p("runtime/warmup_frames", 3))
        if self.target_fps <= 0.0 or self.minimum_fps <= 0.0:
            raise ValueError("runtime FPS values must be positive")
        if not 0.0 < self.max_frame_age_sec <= self.max_result_age_sec:
            raise ValueError("frame/result age limits are inconsistent")

        self.ground_roi_enabled = bool(self._p("ground_roi/enabled", False))
        self.ground_roi_polygon = self._p("ground_roi/polygon", [])
        self.roi_mask_outside = bool(self._p("ground_roi/mask_outside", True))
        self.roi_coordinate_mode = str(
            self._p("ground_roi/coordinate_mode", "normalized")
        )
        if self.roi_coordinate_mode not in ("normalized", "pixels"):
            raise ValueError("ground_roi.coordinate_mode must be normalized or pixels")

        self.context_fraction = float(self._p("detector/context", 0.20))
        self.max_classifier_batch = int(
            self._p("classifier/max_live_batch", 8)
        )
        self.minimum_crop_sharpness = float(
            self._p("classifier/minimum_crop_sharpness", 18.0)
        )
        if self.minimum_crop_sharpness < 0.0:
            raise ValueError("classifier minimum crop sharpness cannot be negative")
        expected_materials = tuple(
            str(value) for value in self._p("classifier/class_names", [])
        )
        if expected_materials != MATERIAL_CLASSES:
            raise RuntimeError(
                "configured material order {} does not match {}".format(
                    expected_materials, MATERIAL_CLASSES
                )
            )
        required_materials_value = rospy.get_param(
            "~required_class_names", ""
        )
        if isinstance(required_materials_value, list):
            required_materials = tuple(
                str(value).strip()
                for value in required_materials_value
                if str(value).strip()
            )
        else:
            required_materials = tuple(
                value.strip()
                for value in str(required_materials_value).split(",")
                if value.strip()
            )
        if required_materials != MATERIAL_CLASSES:
            raise RuntimeError(
                "required motion classes {} do not match classifier order {}".format(
                    required_materials, MATERIAL_CLASSES
                )
            )

        self.depth_enabled = bool(self._p("depth/enabled", True))
        require_depth_timestamp_match = self._p(
            "depth/require_timestamp_match", True
        )
        if type(require_depth_timestamp_match) is not bool:
            raise ValueError(
                "depth.require_timestamp_match must be a YAML boolean"
            )
        self.require_depth_timestamp_match = require_depth_timestamp_match
        self.depth_topic = str(
            self._p("depth/topic", "/fod_camera/depth_registered")
        )
        self.camera_info_topic = str(
            self._p("depth/camera_info_topic", "/fod_camera/camera_info")
        )
        self.depth_tolerance_sec = float(
            self._p("depth/timestamp_tolerance_ms", 20.0)
        ) / 1000.0
        self.depth_wait_sec = float(self._p("depth/wait_ms", 35.0)) / 1000.0
        self.depth_min_m = float(self._p("depth/min_m", 0.30))
        self.depth_max_m = float(self._p("depth/max_m", 15.0))
        self.depth_inset_fraction = float(
            self._p("depth/bbox_inset_fraction", 0.10)
        )
        self.depth_min_samples = int(self._p("depth/min_samples", 24))
        self.depth_min_valid_fraction = float(
            self._p("depth/min_valid_fraction", 0.12)
        )
        self.depth_aggregation = str(self._p("depth/aggregation", "median"))
        self.depth_buffer_size = int(self._p("depth/buffer_size", 45))
        self.camera_info_buffer_size = int(
            self._p("depth/camera_info_buffer_size", 45)
        )
        self.depth_lock_samples = int(
            self._p("depth/lock_valid_samples", 5)
        )
        self.depth_lock_min_inliers = int(
            self._p("depth/lock_min_inliers", 3)
        )
        self.depth_validation_interval_frames = int(
            self._p("depth/validation_interval_frames", 12)
        )
        if not 0.0 < self.depth_tolerance_sec < 0.0334:
            raise ValueError(
                "depth timestamp tolerance must be positive and below one 30 Hz frame"
            )
        if self.depth_buffer_size < 1 or self.camera_info_buffer_size < 1:
            raise ValueError("sensor synchronization buffers must be non-empty")
        if self.depth_lock_samples != 5:
            raise ValueError("depth lock requires exactly five valid observations")
        if not 2 <= self.depth_lock_min_inliers <= self.depth_lock_samples:
            raise ValueError("depth lock inlier count is inconsistent")
        if self.depth_validation_interval_frames < 1:
            raise ValueError("depth validation interval must be positive")

        motion_enabled = self._p("motion/enabled", False)
        if type(motion_enabled) is not bool:
            raise ValueError("motion.enabled must be a YAML boolean")
        self.motion_enabled = motion_enabled
        if self.motion_enabled and not self.depth_enabled:
            raise ValueError(
                "detect_and_classify motion requires registered depth"
            )
        if self.motion_enabled and not self.require_depth_timestamp_match:
            raise ValueError(
                "detect_and_classify motion requires source-stamped depth matching"
            )

        self.transform_target = str(self._p("transform/target_frame", "map"))
        self.transform_fallback = str(
            self._p("transform/fallback_target_frame", "odom")
        )
        self.transform_timeout_sec = float(
            self._p("transform/timeout_ms", 30.0)
        ) / 1000.0
        if not bool(self._p("transform/require_source_timestamp", True)):
            raise ValueError("source-timestamp TF matching may not be disabled")
        if not bool(self._p("transform/per_source_frame_lookup_cache", True)):
            raise ValueError("per-source-frame TF lookup sharing may not be disabled")
        self.world_lock_samples = int(
            self._p("transform/world_lock_samples", 3)
        )
        self.tf_max_consecutive_failures = int(
            self._p("transform/max_consecutive_failures", 10)
        )
        self.tf_failure_backoff_sec = float(
            self._p("transform/failure_backoff_sec", 2.0)
        )
        if self.world_lock_samples != 3:
            raise ValueError("world position lock requires exactly three TF samples")
        if self.tf_max_consecutive_failures != 10:
            raise ValueError("TF failure backoff requires exactly ten failed lookups")
        if self.tf_failure_backoff_sec <= 0.0:
            raise ValueError("TF failure backoff must be positive")

        self.object_map = WorldObjectMap(
            max_world_distance_m=float(
                self._p("object_identity/max_world_distance_m", 0.30)
            ),
            memory_timeout_sec=float(
                self._p("object_identity/object_memory_timeout_sec", 30.0)
            ),
            vote_window=int(self._p("classifier/vote_window", 5)),
            minimum_stable_votes=int(
                self._p("classifier/minimum_stable_votes", 3)
            ),
            stable_confidence=float(
                self._p("classifier/stable_confidence", 0.55)
            ),
            reclassify_interval_frames=int(
                self._p("classifier/reclassify_interval_frames", 5)
            ),
            appearance_change_threshold=float(
                self._p("classifier/appearance_change_threshold", 0.35)
            ),
            depth_lock_samples=self.depth_lock_samples,
            depth_lock_min_inliers=self.depth_lock_min_inliers,
            depth_outlier_mad_scale=float(
                self._p("depth/lock_outlier_mad_scale", 3.0)
            ),
            depth_outlier_min_m=float(
                self._p("depth/lock_outlier_min_m", 0.08)
            ),
            depth_validation_interval_frames=(
                self.depth_validation_interval_frames
            ),
            depth_validation_max_abs_change_m=float(
                self._p("depth/validation_max_abs_change_m", 0.15)
            ),
            depth_validation_max_relative_change=float(
                self._p("depth/validation_max_relative_change", 0.10)
            ),
            depth_validation_failures_before_reacquire=int(
                self._p("depth/validation_failures_before_reacquire", 2)
            ),
            depth_bbox_area_change_ratio=float(
                self._p("depth/bbox_area_change_ratio", 0.40)
            ),
            world_lock_samples=self.world_lock_samples,
            world_outlier_mad_scale=float(
                self._p("transform/world_outlier_mad_scale", 3.0)
            ),
            world_outlier_min_m=float(
                self._p("transform/world_outlier_min_m", 0.08)
            ),
        )
        if not bool(self._p("object_identity/use_one_to_one_assignment", True)):
            raise ValueError("object identity requires one-to-one assignment")
        if not bool(
            self._p("object_identity/inherit_classification_history", True)
        ):
            raise ValueError("classification history inheritance may not be disabled")

        ultralytics_root = str(
            rospy.get_param(
                "~ultralytics_root",
                os.environ.get(
                    "AUTOLABOR_FOD_ULTRALYTICS_ROOT",
                    "/home/slam/yolo11/yolo11_GAM",
                ),
            )
        )
        tracker_config = str(
            rospy.get_param(
                "~two_stage_tracker_config",
                self._p("tracking/tracker_config", ""),
            )
        )
        if not tracker_config:
            raise ValueError("two_stage_tracker_config is required")
        if str(self._p("tracking/tracker", "botsort")) != "botsort":
            raise ValueError("detect_and_classify requires BoT-SORT")
        if not bool(
            self._p("tracking/use_camera_motion_compensation", True)
        ):
            raise ValueError("BoT-SORT camera motion compensation may not be disabled")

        rospy.loginfo(
            "detect_and_classify: loading material classifier first for smoke test"
        )
        configured_device = str(rospy.get_param("~device", "cuda:0"))
        if configured_device == "auto":
            configured_device = "cuda:0"
        configured_detector_confidence = validate_detection_confidence(
            self._p("detector/conf", 0.20)
        )
        try:
            initial_detector_confidence = validate_detection_confidence(
                rospy.get_param(
                    GLOBAL_CONFIDENCE_PARAM,
                    configured_detector_confidence,
                )
            )
        except (TypeError, ValueError) as error:
            rospy.logwarn(
                "Ignoring invalid global detector confidence: %s; using %.2f",
                error,
                configured_detector_confidence,
            )
            initial_detector_confidence = configured_detector_confidence
        self.runtime = TwoStageUltralyticsRuntime(
            ultralytics_root=ultralytics_root,
            detector_weights=str(self._p("detector/model")),
            detector_sha256=str(self._p("detector/expected_sha256")),
            classifier_weights=str(self._p("classifier/model")),
            classifier_sha256=str(
                self._p("classifier/expected_sha256")
            ),
            tracker_config=tracker_config,
            device=configured_device,
            detector_imgsz=int(self._p("detector/imgsz", 768)),
            detector_confidence=initial_detector_confidence,
            detector_iou=float(self._p("detector/iou", 0.60)),
            detector_max_detections=int(self._p("detector/max_det", 50)),
            classifier_imgsz=int(self._p("classifier/imgsz", 224)),
            classifier_max_batch=self.max_classifier_batch,
            half=bool(self._p("detector/half", True))
            and bool(self._p("classifier/half", True)),
            warmup_frames=self.warmup_frames,
            track_buffer_frames=int(
                self._p("tracking/track_buffer_frames", 45)
            ),
        )

        self.frame_slot = LatestFrameSlot()
        self._sensor_condition = threading.Condition()
        self._depth_messages = deque(maxlen=self.depth_buffer_size)
        self._camera_info_messages = deque(maxlen=self.camera_info_buffer_size)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(40.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.received_frames = 0
        self.processed_frames = 0
        self.expired_frames = 0
        self.inference_errors = 0
        self.depth_received = 0
        self.camera_info_received = 0
        self.depth_synchronized_frames = 0
        self.depth_sync_requests = 0
        self.depth_sync_misses = 0
        self.depth_cluster_attempts = 0
        self.depth_cluster_skips_locked = 0
        self.depth_lock_events = 0
        self.depth_reacquire_events = 0
        self.tf_valid_detections = 0
        self.tf_lookup_frames = 0
        self.tf_target_attempts = 0
        self.tf_lookup_successes = 0
        self.tf_lookup_failures = 0
        self.tf_consecutive_failures = 0
        self.tf_backoff_activations = 0
        self.tf_backoff_skips = 0
        self.tf_backoff_until_monotonic = 0.0
        self.world_lock_events = 0
        self.frame_index = 0
        self.last_error = ""
        self.last_publish_monotonic = 0.0
        self.stage_metrics = {
            "detector": deque(maxlen=300),
            "classifier": deque(maxlen=300),
            "total": deque(maxlen=300),
            "capture_to_publish": deque(maxlen=300),
        }

        self.results_pub = rospy.Publisher(
            "/fod/vision/results", FodVisionDetectionArray, queue_size=1
        )
        # Motion output remains fail-closed per frame: only confirmed material
        # classifications are projected into the legacy interface, and the
        # controller independently rejects every candidate without current,
        # source-stamped registered depth.
        self.legacy_pub = rospy.Publisher(
            "/fod/detections", FodDetectionArray, queue_size=1
        )
        self.debug_pub = rospy.Publisher(
            "/fod/debug/image", Image, queue_size=1
        )
        self.diagnostic_pub = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=2
        )
        self.confidence_control = DetectionConfidenceController(
            BACKEND_ID,
            initial_detector_confidence,
            self.runtime.set_detector_confidence,
        )
        self.confidence_service = rospy.Service(
            CONFIDENCE_SERVICE,
            Reconfigure,
            self._set_detection_confidence,
        )
        rospy.set_param(
            GLOBAL_CONFIDENCE_PARAM, self.confidence_control.current
        )
        rospy.set_param(
            "~detect_and_classify/detector/conf",
            self.confidence_control.current,
        )
        self.depth_subscriber = None
        self.camera_info_subscriber = None
        if self.depth_enabled:
            self.depth_subscriber = rospy.Subscriber(
                self.depth_topic,
                Image,
                self._depth_callback,
                queue_size=1,
                buff_size=16 * 1024 * 1024,
            )
            self.camera_info_subscriber = rospy.Subscriber(
                self.camera_info_topic,
                CameraInfo,
                self._camera_info_callback,
                queue_size=1,
            )
        self.image_subscriber = rospy.Subscriber(
            "/fod_camera/image_raw",
            Image,
            self._image_callback,
            queue_size=1,
            buff_size=16 * 1024 * 1024,
        )

        self.runtime_token = str(rospy.get_param("~runtime_token", ""))
        rospy.set_param("~backend", BACKEND_ID)
        rospy.set_param("~runtime_path", self.runtime.runtime_path)
        rospy.set_param("~runtime_version", self.runtime.runtime_version)
        rospy.set_param("~motion_eligible", self.motion_enabled)
        rospy.set_param("~ultralytics_import_path", self.runtime.ultralytics_path)
        rospy.set_param("~ultralytics_version", self.runtime.ultralytics_version)
        rospy.set_param("~gam_layer_count", self.runtime.gam_layer_count)
        rospy.set_param("~detector_task", "detect")
        rospy.set_param("~classifier_task", "classify")
        rospy.set_param("~classifier_probability_dimensions", 5)
        rospy.set_param("~model_load_count_detector", 1)
        rospy.set_param("~model_load_count_classifier", 1)
        rospy.set_param("~ready_token", self.runtime_token)

        self.worker = threading.Thread(
            target=self._worker_loop,
            name="detect_and_classify_inference",
            daemon=False,
        )
        self.worker.start()
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(
            "detect_and_classify ready: ultralytics=%s version=%s detector=%s "
            "detector_sha=%s classifier=%s classifier_sha=%s GAM=%d classes=%s "
            "queue=1 target_fps=%.1f ground_roi=%s motion_eligible=%s "
            "detector_confidence=%.2f confidence_service=%s",
            self.runtime.ultralytics_path,
            self.runtime.ultralytics_version,
            self.runtime.detector_path,
            self.runtime.detector_model_sha256,
            self.runtime.classifier_path,
            self.runtime.classifier_model_sha256,
            self.runtime.gam_layer_count,
            ",".join(MATERIAL_CLASSES),
            self.target_fps,
            self.ground_roi_enabled,
            self.motion_enabled,
            self.confidence_control.current,
            rospy.resolve_name(CONFIDENCE_SERVICE),
        )
        rospy.loginfo(
            "detect_and_classify depth/TF cache: depth_samples=%d "
            "depth_validate_every=%d frames world_samples=%d "
            "TF_failures_before_backoff=%d backoff=%.1fs",
            self.depth_lock_samples,
            self.depth_validation_interval_frames,
            self.world_lock_samples,
            self.tf_max_consecutive_failures,
            self.tf_failure_backoff_sec,
        )

    def _set_detection_confidence(self, request):
        previous = self.confidence_control.current
        response = self.confidence_control.service_response(request)
        rospy.set_param(
            GLOBAL_CONFIDENCE_PARAM, self.confidence_control.current
        )
        rospy.set_param(
            "~detect_and_classify/detector/conf",
            self.confidence_control.current,
        )
        if abs(self.confidence_control.current - previous) > 1e-9:
            rospy.loginfo(
                "detect_and_classify detector confidence changed %.3f -> %.3f",
                previous,
                self.confidence_control.current,
            )
        return response

    def _image_callback(self, message: Image) -> None:
        # Deliberately no CvBridge conversion, depth lookup, TF lookup or model
        # work here.  The only queued payload is the latest ROS image reference
        # together with its acquisition stamp and frame ID.
        envelope = FrameEnvelope(
            message=message,
            stamp_sec=_finite_stamp(message),
            frame_id=str(message.header.frame_id),
        )
        self.received_frames += 1
        self.frame_slot.put(envelope)

    def _depth_callback(self, message: Image) -> None:
        with self._sensor_condition:
            self.depth_received += 1
            self._depth_messages.append(message)
            self._sensor_condition.notify_all()

    def _camera_info_callback(self, message: CameraInfo) -> None:
        with self._sensor_condition:
            self.camera_info_received += 1
            self._camera_info_messages.append(message)
            self._sensor_condition.notify_all()

    @staticmethod
    def _nearest(
        messages,
        stamp_sec: float,
        tolerance_sec: float,
        required_frame: str,
    ):
        compatible = []
        for message in messages:
            if str(message.header.frame_id) != required_frame:
                continue
            candidate = _finite_stamp(message)
            if math.isfinite(candidate):
                compatible.append((abs(candidate - stamp_sec), message))
        if not compatible:
            return None, float("nan")
        delta, message = min(compatible, key=lambda item: item[0])
        if delta > tolerance_sec:
            return None, float("nan")
        return message, float(delta)

    def _matching_sensor_bundle(self, frame: FrameEnvelope):
        if not self.depth_enabled or not math.isfinite(frame.stamp_sec):
            return None, None, float("nan")
        deadline = time.monotonic() + self.depth_wait_sec
        with self._sensor_condition:
            while not self._stopping:
                depth, depth_delta = self._nearest(
                    self._depth_messages,
                    frame.stamp_sec,
                    self.depth_tolerance_sec,
                    frame.frame_id,
                )
                camera_info, info_delta = self._nearest(
                    self._camera_info_messages,
                    frame.stamp_sec,
                    self.depth_tolerance_sec,
                    frame.frame_id,
                )
                if depth is not None and camera_info is not None:
                    return depth, camera_info, max(depth_delta, info_delta)
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None, None, float("nan")
                self._sensor_condition.wait(timeout=remaining)
        return None, None, float("nan")

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

    def _apply_ground_roi(self, image_bgr: np.ndarray) -> np.ndarray:
        if not self.ground_roi_enabled:
            return image_bgr
        if len(self.ground_roi_polygon) < 3:
            raise ValueError("enabled ground ROI requires at least three points")
        height, width = image_bgr.shape[:2]
        points = []
        for value in self.ground_roi_polygon:
            if not isinstance(value, (list, tuple)) or len(value) != 2:
                raise ValueError("ground ROI polygon entries must be [x, y]")
            x, y = float(value[0]), float(value[1])
            if self.roi_coordinate_mode == "normalized":
                x *= width
                y *= height
            points.append((int(round(x)), int(round(y))))
        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.fillPoly(mask, [np.asarray(points, dtype=np.int32)], 255)
        if not self.roi_mask_outside:
            return image_bgr
        return cv2.bitwise_and(image_bgr, image_bgr, mask=mask)

    def _source_age(self, stamp_sec: float) -> float:
        if not math.isfinite(stamp_sec):
            return float("inf")
        now = float(rospy.Time.now().to_sec())
        if not math.isfinite(now) or now <= 0.0:
            return float("inf")
        age = now - stamp_sec
        return age if age >= -0.02 else float("inf")

    def _lookup_source_transform(
        self, source_frame: str, stamp_sec: float
    ):
        """Look up one exact source-frame transform for all boxes in this RGB frame."""
        if not source_frame or not math.isfinite(stamp_sec):
            return None
        now_monotonic = time.monotonic()
        if now_monotonic < self.tf_backoff_until_monotonic:
            self.tf_backoff_skips += 1
            return None

        self.tf_lookup_frames += 1
        stamp = rospy.Time.from_sec(stamp_sec)
        target_frames = tuple(
            dict.fromkeys((self.transform_target, self.transform_fallback))
        )
        for target_frame in target_frames:
            if not target_frame:
                continue
            self.tf_target_attempts += 1
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    stamp,
                    rospy.Duration(self.transform_timeout_sec),
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                continue
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            matrix = quaternion_matrix(
                [rotation.x, rotation.y, rotation.z, rotation.w]
            )
            matrix[:3, 3] = np.asarray(
                [translation.x, translation.y, translation.z],
                dtype=np.float64,
            )
            if np.all(np.isfinite(matrix)):
                self.tf_lookup_successes += 1
                self.tf_consecutive_failures = 0
                return SourceTransform(matrix=matrix, target_frame=target_frame)

        self.tf_lookup_failures += 1
        self.tf_consecutive_failures += 1
        if self.tf_consecutive_failures >= self.tf_max_consecutive_failures:
            self.tf_backoff_activations += 1
            self.tf_consecutive_failures = 0
            self.tf_backoff_until_monotonic = (
                now_monotonic + self.tf_failure_backoff_sec
            )
            rospy.logwarn(
                "detect_and_classify: exact source-stamped TF failed %d times; "
                "backing off for %.1fs before retrying",
                self.tf_max_consecutive_failures,
                self.tf_failure_backoff_sec,
            )
        return None

    @staticmethod
    def _apply_source_transform(
        camera_point: Tuple[float, float, float],
        source_transform: SourceTransform,
    ):
        source = np.asarray(
            [camera_point[0], camera_point[1], camera_point[2], 1.0],
            dtype=np.float64,
        )
        target = source_transform.matrix.dot(source)
        return target[:3] if np.all(np.isfinite(target[:3])) else None

    @staticmethod
    def _roi_for_box(box, width: int, height: int) -> RegionOfInterest:
        x1 = max(0, min(width - 1, int(math.floor(box[0]))))
        y1 = max(0, min(height - 1, int(math.floor(box[1]))))
        x2 = max(x1 + 1, min(width, int(math.ceil(box[2]))))
        y2 = max(y1 + 1, min(height, int(math.ceil(box[3]))))
        return RegionOfInterest(
            x_offset=x1,
            y_offset=y1,
            width=x2 - x1,
            height=y2 - y1,
            do_rectify=False,
        )

    def _legacy_detection(
        self,
        detection,
        target,
        estimate: DepthClusterEstimate,
        width: int,
        height: int,
    ):
        if target.stable_material not in MATERIAL_CLASSES:
            return None
        detect_confidence = float(detection.confidence)
        classify_confidence = float(target.classify_confidence)
        if not all(
            math.isfinite(value)
            and 0.0 <= value <= 1.0
            for value in (detect_confidence, classify_confidence)
        ):
            return None

        roi = self._roi_for_box(detection.bbox, width, height)
        message = FodDetection()
        message.class_id = MATERIAL_CLASSES.index(target.stable_material)
        message.class_name = str(target.stable_material)
        # Require both stages to be confident; a high score from one model may
        # never hide a weak score from the other model at the motion gate.
        message.confidence = min(detect_confidence, classify_confidence)
        message.bbox = roi
        message.anchor_px = Point32(
            x=float(
                min(
                    width - 1,
                    roi.x_offset + 0.5 * max(0, roi.width - 1),
                )
            ),
            y=float(min(height - 1, roi.y_offset + roi.height - 1)),
            z=0.0,
        )
        message.depth_valid = bool(estimate.valid)
        message.depth_m = float(estimate.depth_m)
        message.depth_mad_m = float(estimate.mad_m)
        message.depth_sample_count = int(estimate.sample_count)
        message.depth_valid_fraction = float(estimate.valid_fraction)
        return message

    def _publish_diagnostic(self, level: int, message: str) -> None:
        now = rospy.Time.now()
        elapsed = max(1e-6, time.monotonic() - self.last_publish_monotonic)
        instantaneous_fps = 1.0 / elapsed if self.last_publish_monotonic else 0.0
        cuda_memory = self.runtime.cuda_memory_stats()
        live_objects = [
            target
            for target in self.object_map.objects.values()
            if target.state not in ("CLEANED", "EXPIRED")
        ]
        tf_backoff_remaining = max(
            0.0, self.tf_backoff_until_monotonic - time.monotonic()
        )
        status = DiagnosticStatus()
        status.name = "fod_vision/detect_and_classify"
        status.hardware_id = self.runtime.detector_model_sha256
        status.level = level
        status.message = message
        values = {
            "backend_id": BACKEND_ID,
            "ultralytics_import_path": self.runtime.ultralytics_path,
            "ultralytics_version": self.runtime.ultralytics_version,
            "detector_sha256": self.runtime.detector_model_sha256,
            "classifier_sha256": self.runtime.classifier_model_sha256,
            "detector_task": "detect",
            "classifier_task": "classify",
            "material_classes": ",".join(MATERIAL_CLASSES),
            "detector_confidence_supported": str(
                self.confidence_control.supported
            ),
            "detector_confidence": "{:.3f}".format(
                self.confidence_control.current
            ),
            "detector_confidence_service": rospy.resolve_name(
                CONFIDENCE_SERVICE
            ),
            "gam_layer_count": str(self.runtime.gam_layer_count),
            "model_load_count_detector": str(
                self.runtime.load_counts["detector"]
            ),
            "model_load_count_classifier": str(
                self.runtime.load_counts["classifier"]
            ),
            "frame_queue_capacity": "1",
            "frame_queue_pending": str(self.frame_slot.pending),
            "received_frames": str(self.received_frames),
            "processed_frames": str(self.processed_frames),
            "dropped_frames": str(self.frame_slot.overwritten),
            "expired_frames": str(self.expired_frames),
            "inference_errors": str(self.inference_errors),
            "depth_received": str(self.depth_received),
            "camera_info_received": str(self.camera_info_received),
            "depth_synchronized_frames": str(self.depth_synchronized_frames),
            "depth_sync_requests": str(self.depth_sync_requests),
            "depth_sync_misses": str(self.depth_sync_misses),
            "depth_cluster_attempts": str(self.depth_cluster_attempts),
            "depth_cluster_skips_locked": str(
                self.depth_cluster_skips_locked
            ),
            "depth_lock_events": str(self.depth_lock_events),
            "depth_reacquire_events": str(self.depth_reacquire_events),
            "live_objects": str(len(live_objects)),
            "depth_locked_objects": str(
                sum(1 for target in live_objects if target.depth_locked)
            ),
            "world_locked_objects": str(
                sum(1 for target in live_objects if target.world_locked)
            ),
            "tf_valid_detections": str(self.tf_valid_detections),
            "tf_lookup_frames": str(self.tf_lookup_frames),
            "tf_target_attempts": str(self.tf_target_attempts),
            "tf_lookup_successes": str(self.tf_lookup_successes),
            "tf_lookup_failures": str(self.tf_lookup_failures),
            "tf_consecutive_failures": str(self.tf_consecutive_failures),
            "tf_backoff_activations": str(self.tf_backoff_activations),
            "tf_backoff_skips": str(self.tf_backoff_skips),
            "tf_backoff_remaining_sec": "{:.3f}".format(
                tf_backoff_remaining
            ),
            "world_lock_events": str(self.world_lock_events),
            "instantaneous_publish_fps": "{:.3f}".format(instantaneous_fps),
            "ground_roi_enabled": str(self.ground_roi_enabled),
            "motion_eligible": str(self.motion_enabled),
            "temporary_jpg_writes": "0",
            "cuda_memory_allocated_bytes": str(cuda_memory["allocated"]),
            "cuda_memory_reserved_bytes": str(cuda_memory["reserved"]),
            "cuda_max_memory_allocated_bytes": str(
                cuda_memory["max_allocated"]
            ),
            "cuda_max_memory_reserved_bytes": str(cuda_memory["max_reserved"]),
            "last_error": self.last_error,
        }
        for stage, samples in self.stage_metrics.items():
            values["{}_p50_ms".format(stage)] = "{:.3f}".format(
                _percentile(samples, 50.0)
            )
            values["{}_p95_ms".format(stage)] = "{:.3f}".format(
                _percentile(samples, 95.0)
            )
        status.values = [KeyValue(key, value) for key, value in values.items()]
        array = DiagnosticArray()
        array.header.stamp = now
        array.status = [status]
        self.diagnostic_pub.publish(array)

    def _debug_image(self, image_bgr, detections, targets, depth_estimates):
        output = image_bgr.copy()
        for detection, target, depth in zip(
            detections, targets, depth_estimates
        ):
            x1, y1, x2, y2 = [int(round(value)) for value in detection.bbox]
            color = (80, 220, 80)
            cv2.rectangle(output, (x1, y1), (x2, y2), color, 2)
            classify_text = (
                "{:.2f}".format(target.classify_confidence)
                if math.isfinite(target.classify_confidence)
                else "N/A"
            )
            depth_text = (
                "{:.2f}m".format(depth.depth_m) if depth.valid else "N/A"
            )
            label = "{} D:{:.2f} C:{} depth:{}".format(
                target.stable_material,
                detection.confidence,
                classify_text,
                depth_text,
            )
            cv2.putText(
                output,
                label,
                (max(0, x1), max(18, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
                cv2.LINE_AA,
            )
        return output

    def _process_frame(self, frame: FrameEnvelope) -> None:
        processing_started = time.perf_counter()
        image_bgr = np.asarray(
            self.bridge.imgmsg_to_cv2(frame.message, desired_encoding="bgr8")
        )
        detector_input = self._apply_ground_roi(image_bgr)
        self.frame_index += 1
        detections, detector_ms = self.runtime.detect(
            detector_input, self.frame_index
        )

        crops = []
        appearances = []
        for detection in detections:
            crop, _ = context_crop(
                image_bgr, detection.bbox, self.context_fraction
            )
            crops.append(crop)
            appearances.append(appearance_histogram(crop))

        # Decide whether each object needs a full depth calculation before
        # touching the registered depth image.  A locked object skips both the
        # full-frame CvBridge conversion and the per-box clustering until its
        # scheduled validation (or a material box/appearance change) is due.
        known_targets = []
        depth_sample_requested = []
        for detection, appearance in zip(detections, appearances):
            known_target = self.object_map.target_for_track(
                detection.track_id, frame.stamp_sec
            )
            known_targets.append(known_target)
            should_sample = bool(
                self.depth_enabled
                and (
                    self.motion_enabled
                    or
                    known_target is None
                    or self.object_map.should_sample_depth(
                        known_target,
                        detection.bbox,
                        appearance,
                        self.frame_index,
                    )
                )
            )
            depth_sample_requested.append(should_sample)
            if (
                self.depth_enabled
                and known_target is not None
                and known_target.depth_locked
                and not should_sample
            ):
                self.depth_cluster_skips_locked += 1

        depth_message = None
        camera_info = None
        sync_delta = float("nan")
        depth_array = None
        if any(depth_sample_requested):
            self.depth_sync_requests += 1
            depth_message, camera_info, sync_delta = (
                self._matching_sensor_bundle(frame)
            )
            if depth_message is not None and camera_info is not None:
                try:
                    depth_array = self._decode_depth(depth_message)
                    if depth_array.shape != image_bgr.shape[:2]:
                        raise ValueError(
                            "registered depth is {}x{}, RGB is {}x{}".format(
                                depth_array.shape[1],
                                depth_array.shape[0],
                                image_bgr.shape[1],
                                image_bgr.shape[0],
                            )
                        )
                    if (
                        int(camera_info.width) != int(frame.message.width)
                        or int(camera_info.height) != int(frame.message.height)
                    ):
                        raise ValueError(
                            "CameraInfo is {}x{}, RGB is {}x{}".format(
                                camera_info.width,
                                camera_info.height,
                                frame.message.width,
                                frame.message.height,
                            )
                        )
                    self.depth_synchronized_frames += 1
                except Exception as error:
                    self.last_error = "depth decode failed: {}".format(error)
                    depth_message = None
                    camera_info = None
                    sync_delta = float("nan")
            if depth_array is None:
                self.depth_sync_misses += 1

        depth_estimates = []
        cluster_attempted = []
        for detection, should_sample in zip(
            detections, depth_sample_requested
        ):
            if not self.depth_enabled:
                estimate = DepthClusterEstimate(
                    valid=False, reason="depth processing is disabled"
                )
            elif not should_sample:
                # Do not publish the cached camera range as a current-frame
                # measurement.  The stable value remains internal to object_id
                # association; Qt receives N/A until a fresh validation frame.
                estimate = DepthClusterEstimate(
                    valid=False,
                    reason=(
                        "stable object depth cached; scheduled validation not due"
                    ),
                )
            else:
                estimate = DepthClusterEstimate(
                    valid=False, reason="no synchronized depth"
                )
            attempted = bool(
                should_sample
                and depth_array is not None
                and camera_info is not None
            )
            if attempted:
                self.depth_cluster_attempts += 1
                estimate = estimate_clustered_depth(
                    depth_array,
                    detection.bbox,
                    camera_info.K,
                    minimum_depth_m=self.depth_min_m,
                    maximum_depth_m=self.depth_max_m,
                    inset_fraction=self.depth_inset_fraction,
                    minimum_samples=self.depth_min_samples,
                    minimum_valid_fraction=self.depth_min_valid_fraction,
                    aggregation=self.depth_aggregation,
                )
            depth_estimates.append(estimate)
            cluster_attempted.append(attempted)

        # Only objects without a locked world point need another TF sample.
        # The exact source-stamped lookup is performed once for this RGB frame
        # and the resulting matrix is shared by every qualifying box.
        needs_source_transform = any(
            estimate.valid
            and (known_target is None or not known_target.world_locked)
            for estimate, known_target in zip(depth_estimates, known_targets)
        )
        source_transform = (
            self._lookup_source_transform(frame.frame_id, frame.stamp_sec)
            if needs_source_transform
            else None
        )

        observations = []
        world_positions = []
        world_frames = []
        for detection, appearance, estimate, known_target in zip(
            detections, appearances, depth_estimates, known_targets
        ):
            world_position = None
            world_frame = ""
            if (
                estimate.valid
                and source_transform is not None
                and (known_target is None or not known_target.world_locked)
            ):
                world_position = self._apply_source_transform(
                    estimate.camera_point, source_transform
                )
                if world_position is not None:
                    world_frame = source_transform.target_frame
                    self.tf_valid_detections += 1
            observations.append(
                ObjectObservation(
                    track_id=detection.track_id,
                    bbox=detection.bbox,
                    detect_confidence=detection.confidence,
                    depth_valid=estimate.valid,
                    depth_m=estimate.depth_m,
                    world_position=world_position,
                    world_frame=world_frame,
                    appearance=appearance,
                )
            )
            world_positions.append(world_position)
            world_frames.append(world_frame)

        targets = self.object_map.associate(observations, frame.stamp_sec)
        classification_candidates = []
        for index, target in enumerate(targets):
            crop = crops[index]
            appearance = appearances[index]
            if (
                crop_sharpness(crop) >= self.minimum_crop_sharpness
                and self.object_map.should_classify(
                    target, appearance, self.frame_index
                )
            ):
                classification_candidates.append(
                    (len(target.votes), target.last_classified_frame, index, crop, appearance)
                )
        classification_candidates.sort(key=lambda value: (value[0], value[1]))
        classification_candidates = classification_candidates[
            : self.max_classifier_batch
        ]
        classifier_ms = 0.0
        if classification_candidates:
            probabilities, classifier_ms = self.runtime.classify(
                [value[3] for value in classification_candidates]
            )
            for candidate, probability in zip(
                classification_candidates, probabilities
            ):
                _, _, index, _, appearance = candidate
                self.object_map.add_classification(
                    targets[index], probability, appearance, self.frame_index
                )

        # A paper-like class without a separable local depth cluster must never
        # acquire a distance/world identity from the ground plane.  Apply this
        # before updating either persistent cache.
        for index, target in enumerate(targets):
            estimate = depth_estimates[index]
            if (
                target.stable_material == "paper"
                and estimate.valid
                and not estimate.separated_from_background
            ):
                depth_estimates[index] = DepthClusterEstimate(
                    valid=False,
                    sample_count=estimate.sample_count,
                    valid_fraction=estimate.valid_fraction,
                    reason="paper is not separable from the local ground surface",
                )
                world_positions[index] = None
                world_frames[index] = ""
                self.object_map.clear_depth_lock(target, clear_world=True)
            if (
                target.stable_material == "paper"
                and target.depth_locked
                and not target.depth_separated_from_background
            ):
                self.object_map.clear_depth_lock(target, clear_world=True)

            estimate = depth_estimates[index]
            target.depth_valid = bool(estimate.valid)
            target.depth_m = float(estimate.depth_m)
            if cluster_attempted[index]:
                if estimate.valid:
                    was_depth_locked = target.depth_locked
                    was_world_locked = target.world_locked
                    event = self.object_map.record_depth_observation(
                        target,
                        estimate,
                        frame.stamp_sec,
                        self.frame_index,
                        world_positions[index],
                        world_frames[index],
                    )
                    if not was_depth_locked and target.depth_locked:
                        self.depth_lock_events += 1
                    if not was_world_locked and target.world_locked:
                        self.world_lock_events += 1
                    if event == "REACQUIRING":
                        self.depth_reacquire_events += 1
                elif self.object_map.note_depth_failure(
                    target, self.frame_index
                ):
                    self.depth_reacquire_events += 1

            # A world-locked point is the persistent object-map coordinate,
            # not a replacement for a missing current-frame camera depth.  It
            # may be displayed for object identity while depth remains N/A.
            if (
                world_positions[index] is None
                and target.world_locked
                and target.world_position is not None
            ):
                world_positions[index] = target.world_position.copy()
                world_frames[index] = target.world_frame

        processing_ms = (time.perf_counter() - processing_started) * 1000.0
        capture_age = self._source_age(frame.stamp_sec)
        if not math.isfinite(capture_age) or capture_age > self.max_result_age_sec:
            self.expired_frames += 1
            return

        result = FodVisionDetectionArray()
        result.header = frame.message.header
        result.backend_id = BACKEND_ID
        result.image_width = int(frame.message.width)
        result.image_height = int(frame.message.height)
        result.model_name = self.runtime.model_name
        result.detector_model_sha256 = self.runtime.detector_model_sha256
        result.classifier_model_sha256 = self.runtime.classifier_model_sha256
        result.detector_inference_ms = float(detector_ms)
        result.classifier_inference_ms = float(classifier_ms)
        result.total_latency_ms = float(processing_ms)
        result.received_frames = int(self.received_frames)
        result.processed_frames = int(self.processed_frames + 1)
        result.dropped_frames = int(self.frame_slot.overwritten)
        result.expired_frames = int(self.expired_frames)
        result.depth_synchronized = bool(
            depth_message is not None and camera_info is not None
        )
        if depth_message is not None:
            result.depth_header = depth_message.header
        result.depth_sync_delta_sec = float(sync_delta)

        for detection, target, estimate, world_position, world_frame in zip(
            detections,
            targets,
            depth_estimates,
            world_positions,
            world_frames,
        ):
            message = FodVisionDetection()
            message.backend_id = BACKEND_ID
            message.object_id = int(target.object_id)
            message.track_id = int(detection.track_id)
            message.detector_class_id = 0
            message.material_class = str(target.stable_material)
            message.detect_confidence = float(detection.confidence)
            message.classify_confidence = float(target.classify_confidence)
            aggregate = target.aggregate_probabilities()
            message.class_probabilities = (
                [0.0] * len(MATERIAL_CLASSES)
                if aggregate is None
                else [float(value) for value in aggregate]
            )
            message.bbox = self._roi_for_box(
                detection.bbox,
                int(frame.message.width),
                int(frame.message.height),
            )
            message.depth_valid = bool(estimate.valid)
            message.depth_m = float(estimate.depth_m)
            message.depth_mad_m = float(estimate.mad_m)
            message.depth_sample_count = int(estimate.sample_count)
            message.world_position_valid = bool(world_position is not None)
            if world_position is not None:
                message.world_position = Point(
                    x=float(world_position[0]),
                    y=float(world_position[1]),
                    z=float(world_position[2]),
                )
            message.world_frame = str(world_frame)
            message.state = str(target.state)
            message.last_observed = frame.message.header.stamp
            result.detections.append(message)

        legacy = FodDetectionArray()
        legacy.header = frame.message.header
        legacy.image_width = int(frame.message.width)
        legacy.image_height = int(frame.message.height)
        legacy.model_name = self.runtime.model_name
        legacy.model_sha256 = self.runtime.detector_model_sha256
        legacy.model_task = "detect"
        legacy.inference_ms = float(processing_ms)
        depth_synchronized = bool(
            depth_array is not None
            and depth_message is not None
            and camera_info is not None
        )
        legacy.depth_synchronized = depth_synchronized
        legacy.depth_sync_delta_sec = (
            float(sync_delta) if depth_synchronized else float("nan")
        )
        if depth_synchronized:
            legacy.depth_header = depth_message.header
        legacy.detections = [
            message
            for message in (
                self._legacy_detection(
                    detection,
                    target,
                    estimate,
                    int(frame.message.width),
                    int(frame.message.height),
                )
                for detection, target, estimate in zip(
                    detections, targets, depth_estimates
                )
            )
            if message is not None
        ]

        self.results_pub.publish(result)
        self.legacy_pub.publish(legacy)
        if self.debug_pub.get_num_connections() > 0:
            debug = self._debug_image(
                image_bgr, detections, targets, depth_estimates
            )
            debug_message = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            debug_message.header = frame.message.header
            self.debug_pub.publish(debug_message)
        self.processed_frames += 1
        self.stage_metrics["detector"].append(float(detector_ms))
        self.stage_metrics["classifier"].append(float(classifier_ms))
        self.stage_metrics["total"].append(float(processing_ms))
        self.stage_metrics["capture_to_publish"].append(
            float(capture_age * 1000.0)
        )
        self._publish_diagnostic(
            DiagnosticStatus.OK,
            (
                "detect_and_classify running; synchronized motion output enabled"
                if self.motion_enabled
                else "detect_and_classify running; recognition-only"
            ),
        )
        self.last_publish_monotonic = time.monotonic()

    def _worker_loop(self) -> None:
        period = 1.0 / self.target_fps
        next_start = 0.0
        while not self._stopping and not rospy.is_shutdown():
            frame = self.frame_slot.take(timeout=0.20)
            if frame is None:
                continue
            now = time.monotonic()
            if next_start > now:
                time.sleep(min(period, next_start - now))
                replacement = self.frame_slot.take(timeout=0.0)
                if replacement is not None:
                    frame = replacement
            source_age = self._source_age(frame.stamp_sec)
            if not math.isfinite(source_age) or source_age > self.max_frame_age_sec:
                self.expired_frames += 1
                continue
            next_start = time.monotonic() + period
            try:
                self._process_frame(frame)
                self.last_error = ""
            except Exception as error:
                self.inference_errors += 1
                self.last_error = "{}: {}".format(type(error).__name__, error)
                rospy.logerr_throttle(
                    1.0,
                    "detect_and_classify frame failed: %s\n%s",
                    error,
                    traceback.format_exc(),
                )
                self._publish_diagnostic(
                    DiagnosticStatus.ERROR, self.last_error
                )

    def shutdown(self) -> None:
        with self._shutdown_lock:
            if self._stopping:
                return
            self._stopping = True
            self.frame_slot.stop()
            with self._sensor_condition:
                self._sensor_condition.notify_all()
        if (
            getattr(self, "worker", None) is not None
            and threading.current_thread() is not self.worker
        ):
            self.worker.join(timeout=15.0)
            if self.worker.is_alive():
                rospy.logerr(
                    "detect_and_classify inference thread did not stop within 15 seconds"
                )
                return
        with self._shutdown_lock:
            if not self._models_released:
                self.runtime.close()
                self.object_map.reset()
                self._models_released = True
                rospy.loginfo(
                    "detect_and_classify stopped; inference thread joined and both models released"
                )


def main():
    rospy.init_node("fod_detector", anonymous=False)
    try:
        DetectAndClassifyNode()
    except Exception as error:
        rospy.logfatal(
            "detect_and_classify failed to start: %s\n%s",
            error,
            traceback.format_exc(),
        )
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
