#!/usr/bin/env python3
"""Latest-frame YOLO inference node with explicit smoke-test safeguards."""

import json
import threading
import traceback

import rospy
from autolabor_fod_msgs.msg import FodDetection, FodDetectionArray
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point32
from sensor_msgs.msg import CameraInfo, Image, RegionOfInterest
from std_msgs.msg import String

from autolabor_fod_vision.detector import UltralyticsDetector, annotate_image
from autolabor_fod_vision.preview_tracking import PreviewStabilizer
from autolabor_fod_vision.detection_roi import DetectionRoi, mark_recovery_region
from autolabor_fod_vision.depth import RegisteredDepthFusion, DetectionTracker


def _csv_ints(value):
    if isinstance(value, list):
        return [int(item) for item in value]
    return [int(item.strip()) for item in str(value).split(",") if item.strip()]


def _csv_strings(value):
    if isinstance(value, list):
        return [str(item).strip() for item in value if str(item).strip()]
    return [item.strip() for item in str(value).split(",") if item.strip()]


class DetectorNode:
    def __init__(self):
        weights = str(rospy.get_param("~weights"))
        self.smoke_test_only = bool(rospy.get_param("~smoke_test_only", True))
        required_names = _csv_strings(
            rospy.get_param("~required_class_names", "fod")
        )
        self.debug_every_n = max(1, int(rospy.get_param("~debug_every_n", 1)))
        classes = _csv_ints(rospy.get_param("~classes", ""))
        self.bridge = CvBridge()
        self.depth_enabled = bool(rospy.get_param("~use_registered_depth", False))
        self.depth_fusion = RegisteredDepthFusion() if self.depth_enabled else None
        self.detection_tracker = DetectionTracker() if self.depth_enabled else None
        self.selected_track_id = 0
        self.recovery_overlay = None
        self.depth_box_count = 0
        self.depth_error = ""
        self.detector_profile = str(rospy.get_param("~detector_profile", "original"))
        self.roi = DetectionRoi(rospy.get_param("~roi_top_fraction", 0.0),
                                rospy.get_param("~roi_bottom_fraction", 1.0))
        self.preview_stabilizer = (
            PreviewStabilizer(
                alpha=float(rospy.get_param("~preview_smoothing_alpha", 0.65)),
                hold_sec=float(rospy.get_param("~preview_hold_sec", 0.30)),
            ) if bool(rospy.get_param("~stabilize_preview", False)) else None
        )
        self.current_box_count = 0
        self.preview_box_count = 0
        self.preview_held_count = 0
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
            agnostic_nms=bool(rospy.get_param("~agnostic_nms", False)),
            expected_sha256=str(
                rospy.get_param("~expected_model_sha256", "")
            ),
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
        self._stopping = False
        self.received_frames = 0
        self.processed_frames = 0
        self.dropped_frames = 0
        self.last_inference_ms = 0.0
        self.last_error = ""
        if self.depth_enabled:
            self.depth_subscriber = rospy.Subscriber(
                "/fod_camera/depth_registered", Image, self._depth_callback,
                queue_size=1, buff_size=16 * 1024 * 1024)
            self.info_subscriber = rospy.Subscriber(
                "/fod_camera/camera_info", CameraInfo, self._info_callback, queue_size=1)
        self.selection_subscriber = rospy.Subscriber(
            "/fod_visual_servo/status", String, self._selection_callback, queue_size=1)
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
        rospy.loginfo(
            "FOD detector ready: model=%s task=%s device=%s classes=%s sha256=%s",
            self.detector.model_name,
            self.detector.task,
            self.detector.device,
            ",".join(self.detector.names.values()),
            self.detector.model_sha256,
        )

    def _depth_callback(self, message):
        try:
            depth = self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
            self.depth_fusion.add_depth(message.header.stamp.to_sec(),
                                        message.header.frame_id, depth, message.encoding)
            self.depth_error = ""
        except (ValueError, TypeError, RuntimeError) as exc:
            self.depth_error = str(exc)
            rospy.logwarn_throttle(5.0, "FOD depth unavailable: %s", exc)

    def _info_callback(self, message):
        self.depth_fusion.set_camera(message.header.stamp.to_sec(), message.header.frame_id,
                                     message.width, message.height, message.P)

    def _selection_callback(self, message):
        try:
            state = json.loads(message.data)
            self.selected_track_id = int(state.get("target_track_id", 0)) if state.get("active") else 0
            self.recovery_overlay = state
        except (ValueError, TypeError, AttributeError):
            self.selected_track_id = 0
            self.recovery_overlay = None

    def _image_callback(self, message):
        with self._condition:
            self.received_frames += 1
            if self._latest_message is not None:
                self.dropped_frames += 1
            self._latest_message = message
            self._condition.notify()

    def _to_ros_detection(self, detection, width, height):
        message = FodDetection()
        message.class_id = detection.class_id
        message.class_name = detection.class_name
        message.confidence = detection.confidence
        message.track_id = detection.track_id
        message.depth_valid = detection.depth_valid
        message.depth_m = detection.depth_m
        message.range_m = detection.range_m
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
            KeyValue("classes", ",".join(self.detector.names.values())),
            KeyValue("smoke_test_only", str(self.smoke_test_only)),
            KeyValue("received_frames", str(self.received_frames)),
            KeyValue("processed_frames", str(self.processed_frames)),
            KeyValue("dropped_frames", str(self.dropped_frames)),
            KeyValue("last_inference_ms", "{:.2f}".format(self.last_inference_ms)),
            KeyValue("detector_profile", self.detector_profile),
            KeyValue("confidence_threshold", str(self.detector.confidence)),
            KeyValue("image_size", str(self.detector.image_size)),
            KeyValue("current_box_count", str(self.current_box_count)),
            KeyValue("preview_box_count", str(self.preview_box_count)),
            KeyValue("preview_held_count", str(self.preview_held_count)),
            KeyValue("roi_top_fraction", str(self.roi.top)),
            KeyValue("roi_bottom_fraction", str(self.roi.bottom)),
            KeyValue("registered_depth_enabled", str(self.depth_enabled)),
            KeyValue("depth_box_count", str(self.depth_box_count)),
            KeyValue("depth_error", self.depth_error),
        ]
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status = [status]
        self.diagnostic_pub.publish(array)

    def _process(self, image_message):
        image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
        result = self.roi.predict(self.detector, image)
        if self.depth_fusion is not None:
            result.detections = self.depth_fusion.enrich(
                result.detections, image_message.header.stamp.to_sec(),
                image_message.header.frame_id, image.shape[1], image.shape[0])
            result.detections = self.detection_tracker.update(
                result.detections, image_message.header.stamp.to_sec(),
                (image_message.header.frame_id, image.shape[1], image.shape[0]))
        self.depth_box_count = sum(d.depth_valid for d in result.detections)
        self.last_inference_ms = result.inference_ms
        self.current_box_count = len(result.detections)
        preview_detections = result.detections
        held_for_sec = None
        if self.preview_stabilizer is not None:
            preview = self.preview_stabilizer.update(
                result.detections, image_message.header.stamp.to_sec(),
                (image.shape[1], image.shape[0]),
            )
            preview_detections = [box.detection for box in preview]
            held_for_sec = [box.held_for_sec for box in preview]
        self.preview_box_count = len(preview_detections)
        self.preview_held_count = sum(age > 0.0 for age in (held_for_sec or []))
        # Control receives only this frame's actual model results, with original
        # scores and geometry. Preview smoothing/holds never enter this topic.
        output = FodDetectionArray()
        output.header = image_message.header
        output.image_width = image.shape[1]
        output.image_height = image.shape[0]
        output.model_name = self.detector.model_name
        output.model_sha256 = self.detector.model_sha256
        output.model_task = self.detector.task
        output.inference_ms = result.inference_ms
        output.detections = [
            self._to_ros_detection(item, image.shape[1], image.shape[0])
            for item in result.detections
        ]
        self.detection_pub.publish(output)

        self.processed_frames += 1
        if self.processed_frames % self.debug_every_n == 0:
            banner = (
                "SMOKE ONLY {}".format(self.detector.model_name)
                if self.smoke_test_only
                else self.detector.model_name
            )
            debug = annotate_image(
                image, preview_detections, result.inference_ms,
                banner + " / " + self.detector_profile, held_for_sec=held_for_sec,
                selected_track_id=self.selected_track_id,
            )
            debug = self.roi.mark_preview(debug)
            debug = mark_recovery_region(debug, self.recovery_overlay)
            debug_message = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            debug_message.header = image_message.header
            self.debug_pub.publish(debug_message)
        self._publish_diagnostic(DiagnosticStatus.OK, "inference active")

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
        rospy.logfatal("FOD detector failed to start: %s", error)
        raise


if __name__ == "__main__":
    main()
