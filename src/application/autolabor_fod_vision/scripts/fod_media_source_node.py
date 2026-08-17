#!/usr/bin/env python3
"""Publish a V4L2 camera, image, or video as timestamped ROS image topics."""

import threading
from pathlib import Path

import cv2
import rospy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool

from autolabor_fod_vision.camera_info import (
    load_camera_calibration,
    uncalibrated,
)


class MediaSourceNode:
    def __init__(self):
        self.source_type = str(rospy.get_param("~source_type", "device")).lower()
        self.source = str(rospy.get_param("~source", "/dev/video0"))
        self.frame_id = str(
            rospy.get_param("~frame_id", "fod_camera_optical_frame")
        )
        self.camera_name = str(rospy.get_param("~camera_name", "fod_camera"))
        self.camera_info_url = str(rospy.get_param("~camera_info_url", ""))
        self.request_width = int(rospy.get_param("~width", 0))
        self.request_height = int(rospy.get_param("~height", 0))
        self.request_fps = float(rospy.get_param("~fps", 30.0))
        self.loop_media = bool(rospy.get_param("~loop", True))
        self.use_mjpeg = bool(rospy.get_param("~use_mjpeg", True))
        self.reconnect_delay = float(rospy.get_param("~reconnect_delay", 1.0))

        if self.source_type not in ("device", "video", "image"):
            raise ValueError("source_type must be device, video, or image")
        if self.request_fps <= 0.0:
            raise ValueError("fps must be positive")

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher(
            "/fod_camera/image_raw", Image, queue_size=1
        )
        self.info_pub = rospy.Publisher(
            "/fod_camera/camera_info", CameraInfo, queue_size=1
        )
        self.alive_pub = rospy.Publisher(
            "/fod_camera/source_alive", Bool, queue_size=1, latch=True
        )
        self.diagnostic_pub = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=2
        )
        self.capture = None
        self.static_image = None
        self.calibration = None
        self.frames = 0
        self.failures = 0
        self.last_error = ""
        self.started_at = rospy.Time.now()
        self.last_diagnostic = rospy.Time(0)
        self._lock = threading.Lock()

        if self.camera_info_url:
            self.calibration = load_camera_calibration(self.camera_info_url)
            rospy.loginfo(
                "Loaded camera calibration '%s' (%dx%d)",
                self.calibration.camera_name,
                self.calibration.width,
                self.calibration.height,
            )
        rospy.on_shutdown(self.close)
        try:
            self._open_source()
        except RuntimeError as error:
            if self.source_type != "device":
                raise
            self.last_error = str(error)
            rospy.logwarn(
                "FOD camera is not ready; waiting and retrying: %s", error
            )
            self._publish_diagnostic(force=True)

    def _capture_source(self):
        if self.source_type == "device":
            return int(self.source) if self.source.isdigit() else self.source
        return self.source

    def _open_source(self):
        self.close()
        if self.source_type == "image":
            self.static_image = cv2.imread(
                str(Path(self.source).expanduser()), cv2.IMREAD_COLOR
            )
            if self.static_image is None:
                raise RuntimeError("cannot read image: {}".format(self.source))
            rospy.loginfo("FOD image source opened: %s", self.source)
            self.alive_pub.publish(Bool(data=True))
            return

        source = self._capture_source()
        if self.source_type == "device":
            self.capture = cv2.VideoCapture(source, cv2.CAP_V4L2)
            if not self.capture.isOpened():
                self.capture.release()
                self.capture = cv2.VideoCapture(source)
            if self.use_mjpeg:
                self.capture.set(
                    cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")
                )
            if self.request_width > 0:
                self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.request_width)
            if self.request_height > 0:
                self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.request_height)
            self.capture.set(cv2.CAP_PROP_FPS, self.request_fps)
        else:
            self.capture = cv2.VideoCapture(source)

        if not self.capture.isOpened():
            self.capture.release()
            self.capture = None
            self.alive_pub.publish(Bool(data=False))
            raise RuntimeError(
                "cannot open {} source: {}".format(self.source_type, self.source)
            )
        rospy.loginfo("FOD %s source opened: %s", self.source_type, self.source)
        self.alive_pub.publish(Bool(data=True))

    def close(self):
        with self._lock:
            if self.capture is not None:
                self.capture.release()
                self.capture = None

    def _read(self):
        if self.source_type == "image":
            return (
                self.static_image.copy()
                if self.static_image is not None
                else None
            )
        capture = self.capture
        if capture is None or not capture.isOpened():
            return None
        ok, frame = capture.read()
        if ok and frame is not None:
            return frame
        if self.source_type == "video" and self.loop_media:
            capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = capture.read()
            if ok and frame is not None:
                return frame
        return None

    def _resize_if_requested(self, frame):
        if self.request_width <= 0 or self.request_height <= 0:
            return frame
        if (
            frame.shape[1] == self.request_width
            and frame.shape[0] == self.request_height
        ):
            return frame
        return cv2.resize(
            frame,
            (self.request_width, self.request_height),
            interpolation=cv2.INTER_AREA,
        )

    def _camera_info(self, width, height, stamp):
        calibration = (
            self.calibration.scaled(width, height)
            if self.calibration is not None
            else uncalibrated(width, height, self.camera_name)
        )
        message = CameraInfo()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        message.width = calibration.width
        message.height = calibration.height
        message.distortion_model = calibration.distortion_model
        message.D = calibration.d
        message.K = calibration.k
        message.R = calibration.r
        message.P = calibration.p
        return message

    def _publish_diagnostic(self, force=False):
        now = rospy.Time.now()
        if not force and (now - self.last_diagnostic).to_sec() < 1.0:
            return
        self.last_diagnostic = now
        alive = self.static_image is not None or (
            self.capture is not None and self.capture.isOpened()
        )
        status = DiagnosticStatus()
        status.name = "fod_vision/media_source"
        status.hardware_id = self.source
        status.level = (
            DiagnosticStatus.OK if alive else DiagnosticStatus.ERROR
        )
        status.message = "streaming" if alive else (self.last_error or "offline")
        elapsed = max(1e-3, (now - self.started_at).to_sec())
        status.values = [
            KeyValue("source_type", self.source_type),
            KeyValue("source", self.source),
            KeyValue("frame_id", self.frame_id),
            KeyValue("frames", str(self.frames)),
            KeyValue("failures", str(self.failures)),
            KeyValue("average_fps", "{:.2f}".format(self.frames / elapsed)),
            KeyValue(
                "calibrated",
                str(bool(self.calibration and self.calibration.calibrated)),
            ),
        ]
        array = DiagnosticArray()
        array.header.stamp = now
        array.status = [status]
        self.diagnostic_pub.publish(array)

    def run(self):
        rate = rospy.Rate(self.request_fps)
        while not rospy.is_shutdown():
            frame = self._read()
            if frame is None:
                self.failures += 1
                self.last_error = "frame read failed"
                self.alive_pub.publish(Bool(data=False))
                self._publish_diagnostic(force=True)
                if self.source_type == "video" and not self.loop_media:
                    rospy.loginfo("FOD video source reached end of stream")
                    break
                if self.source_type == "device":
                    rospy.logwarn_throttle(
                        5.0, "FOD camera frame read failed; reconnecting"
                    )
                    rospy.sleep(self.reconnect_delay)
                    if rospy.is_shutdown():
                        break
                    try:
                        self._open_source()
                    except Exception as error:
                        self.last_error = str(error)
                        self.alive_pub.publish(Bool(data=False))
                    continue
                rate.sleep()
                continue

            frame = self._resize_if_requested(frame)
            stamp = rospy.Time.now()
            image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            image_message.header.stamp = stamp
            image_message.header.frame_id = self.frame_id
            info_message = self._camera_info(frame.shape[1], frame.shape[0], stamp)
            self.image_pub.publish(image_message)
            self.info_pub.publish(info_message)
            self.frames += 1
            self.alive_pub.publish(Bool(data=True))
            self._publish_diagnostic()
            rate.sleep()


def main():
    rospy.init_node("fod_media_source")
    try:
        MediaSourceNode().run()
    except Exception as error:
        rospy.logfatal("FOD media source failed: %s", error)
        raise


if __name__ == "__main__":
    main()
