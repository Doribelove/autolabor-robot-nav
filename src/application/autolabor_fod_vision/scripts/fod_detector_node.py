#!/usr/bin/env python3
"""Latest-frame YOLO inference node with explicit smoke-test safeguards."""

import threading
import traceback

import rospy
from autolabor_fod_msgs.msg import FodDetection, FodDetectionArray
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point32
from sensor_msgs.msg import Image, RegionOfInterest

from autolabor_fod_vision.detector import UltralyticsDetector, annotate_image


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
        ]
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status = [status]
        self.diagnostic_pub.publish(array)

    def _process(self, image_message):
        image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
        result = self.detector.predict(image)
        self.last_inference_ms = result.inference_ms
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
                image, result.detections, result.inference_ms, banner
            )
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
