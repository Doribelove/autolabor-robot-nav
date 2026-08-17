#!/usr/bin/env python3
"""ROI-based exposure/gain control and input-quality diagnostics."""

import threading
from types import SimpleNamespace

import rospy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from dynamic_reconfigure.client import Client as DynamicReconfigureClient
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, SetBoolResponse

from autolabor_fod_vision.image_quality import (
    ControllerConfig,
    ExposureGainController,
    ImagingControlBounds,
    NormalizedRoi,
    measure_image_quality,
    quality_flags,
)


def _text(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        return "{:.4f}".format(value)
    return str(value)


class ImageQualityControllerNode:
    def __init__(self):
        self.enabled = bool(rospy.get_param("~enabled", True))
        self.monitor_only = bool(rospy.get_param("~monitor_only", False))
        self.image_topic = str(
            rospy.get_param("~image_topic", "/fod_camera/image_raw")
        )
        self.dynamic_server = str(
            rospy.get_param("~dynamic_reconfigure_server", "/zed2/zed_node")
        )
        self.control_rate_hz = max(
            0.1, float(rospy.get_param("~control_rate_hz", 2.0))
        )
        self.image_timeout_sec = max(
            0.1, float(rospy.get_param("~image_timeout_sec", 1.0))
        )
        self.settle_cycles = max(0, int(rospy.get_param("~settle_cycles", 2)))
        self.max_sample_width = max(
            32, int(rospy.get_param("~max_sample_width", 320))
        )
        self.restore_auto_on_shutdown = bool(
            rospy.get_param("~restore_auto_on_shutdown", True)
        )

        self.roi = NormalizedRoi(
            x_min=float(rospy.get_param("~roi_x_min", 0.05)),
            x_max=float(rospy.get_param("~roi_x_max", 0.95)),
            y_min=float(rospy.get_param("~roi_y_min", 0.25)),
            y_max=float(rospy.get_param("~roi_y_max", 1.0)),
        )
        self.config = ControllerConfig(
            target_median=float(rospy.get_param("~target_median", 115.0)),
            median_tolerance=float(
                rospy.get_param("~median_tolerance", 8.0)
            ),
            dark_threshold=int(rospy.get_param("~dark_threshold", 5)),
            bright_threshold=int(rospy.get_param("~bright_threshold", 250)),
            max_dark_fraction=float(
                rospy.get_param("~max_dark_fraction", 0.08)
            ),
            max_bright_fraction=float(
                rospy.get_param("~max_bright_fraction", 0.02)
            ),
            dynamic_range_dark_fraction=float(
                rospy.get_param("~dynamic_range_dark_fraction", 0.08)
            ),
            dynamic_range_bright_fraction=float(
                rospy.get_param("~dynamic_range_bright_fraction", 0.02)
            ),
            sharpness_warn_threshold=float(
                rospy.get_param("~sharpness_warn_threshold", 45.0)
            ),
            color_mean_spread_warn=float(
                rospy.get_param("~color_mean_spread_warn", 45.0)
            ),
            controller_gain=float(rospy.get_param("~controller_gain", 0.60)),
            max_exposure_step_ratio=float(
                rospy.get_param("~max_exposure_step_ratio", 1.35)
            ),
            exposure_min_us=float(
                rospy.get_param("~exposure_min_percent", 1.0)
            ),
            exposure_max_us=float(
                rospy.get_param("~exposure_max_percent", 100.0)
            ),
            gain_min=float(rospy.get_param("~gain_min", 0.0)),
            gain_max=float(rospy.get_param("~gain_max", 100.0)),
            gain_step=float(rospy.get_param("~gain_step", 2.0)),
        )
        self.controller = ExposureGainController(self.config)
        self.bridge = CvBridge()

        self._image_lock = threading.Lock()
        self._control_lock = threading.RLock()
        self._latest_image = None
        self._latest_receipt = None
        self._manual_control_active = False
        self._remaining_settle_cycles = 0
        self._last_controls = None
        self._last_action = "starting"
        self._shutdown = False

        # ZED camera initialization takes several seconds. Connect lazily so
        # roslaunch startup order cannot make this sidecar exit permanently.
        self.dynamic_client = None
        self.diagnostic_pub = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=2
        )
        self.image_subscriber = rospy.Subscriber(
            self.image_topic,
            Image,
            self._image_callback,
            queue_size=1,
            buff_size=16 * 1024 * 1024,
        )
        self.enable_service = rospy.Service(
            "~set_enabled", SetBool, self._set_enabled
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate_hz), self._timer_callback
        )
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(
            "FOD image-quality controller ready: enabled=%s monitor_only=%s "
            "ROI=(%.2f,%.2f)-(%.2f,%.2f), ZED exposure cap=%.0f%%",
            self.enabled,
            self.monitor_only,
            self.roi.x_min,
            self.roi.y_min,
            self.roi.x_max,
            self.roi.y_max,
            self.config.exposure_max_us,
        )

    def _image_callback(self, message):
        with self._image_lock:
            self._latest_image = message
            self._latest_receipt = rospy.Time.now()

    def _snapshot_image(self):
        with self._image_lock:
            return self._latest_image, self._latest_receipt

    @staticmethod
    def _controls(configuration):
        automatic = bool(configuration["auto_exposure_gain"])
        return SimpleNamespace(
            success=True,
            message="ZED dynamic-reconfigure values",
            exposure_auto=automatic,
            exposure_time_us=float(configuration["exposure"]),
            exposure_min_us=1.0,
            exposure_max_us=100.0,
            gain_auto=automatic,
            gain=float(configuration["gain"]),
            gain_min=0.0,
            gain_max=100.0,
        )

    def _client(self):
        if self.dynamic_client is None:
            self.dynamic_client = DynamicReconfigureClient(
                self.dynamic_server, timeout=0.75
            )
        return self.dynamic_client

    def _get_controls(self):
        return self._controls(self._client().get_configuration(timeout=1.0))

    def _update_field(self, name, value):
        # Send the complete current configuration. The upstream ROS1 wrapper
        # initializes omitted service fields to defaults, which can otherwise
        # revert auto mode while setting exposure or gain.
        configuration = self._client().get_configuration(timeout=1.0)
        configuration[name] = value
        return self._client().update_configuration(configuration)

    def _request_set(self, exposure_auto, exposure_percent, gain_auto, gain):
        if bool(exposure_auto) != bool(gain_auto):
            raise ValueError("ZED uses one shared automatic exposure/gain mode")
        # The ROS1 ZED wrapper assigns non-bitmask dynamic-reconfigure levels
        # to these fields and handles exactly one level per callback. Send
        # ordered single-field updates so manual values reach the hardware.
        configuration = self._update_field(
            "auto_exposure_gain", bool(exposure_auto)
        )
        if not exposure_auto:
            configuration = self._update_field(
                "exposure", int(round(exposure_percent))
            )
            configuration = self._update_field(
                "gain", int(round(gain))
            )
        return self._controls(configuration)

    def _restore_auto(self):
        if not self._manual_control_active:
            return True, "camera control was not taken"
        exposure = self.config.exposure_min_us
        gain = self.config.gain_min
        if self._last_controls is not None:
            exposure = self._last_controls.exposure_time_us
            gain = self._last_controls.gain
        try:
            self._request_set(True, exposure, True, gain)
            self._manual_control_active = False
            self._remaining_settle_cycles = 0
            self._last_action = "restored_camera_auto"
            rospy.loginfo("Restored ZED native automatic exposure/gain")
            return True, "ZED automatic exposure/gain restored"
        except Exception as error:
            rospy.logerr("Failed to restore camera automatic controls: %s", error)
            return False, str(error)

    def _set_enabled(self, request):
        with self._control_lock:
            self.enabled = bool(request.data)
            if self.enabled:
                self._remaining_settle_cycles = 0
                self._last_action = "control_enabled"
                return SetBoolResponse(
                    success=True,
                    message=(
                        "automatic image-quality control enabled"
                        if not self.monitor_only
                        else "monitor-only mode enabled; camera will not change"
                    ),
                )
            success, message = self._restore_auto()
            self._last_action = "control_disabled"
            return SetBoolResponse(success=success, message=message)

    def _publish_diagnostic(
        self, level, message, metrics=None, controls=None, action=None
    ):
        status = DiagnosticStatus()
        status.name = "fod_vision/image_quality_controller"
        status.hardware_id = "zed2"
        status.level = level
        status.message = message
        values = [
            ("enabled", self.enabled),
            ("monitor_only", self.monitor_only),
            ("image_topic", self.image_topic),
            ("action", action or self._last_action),
            (
                "roi_normalized",
                "{:.3f},{:.3f},{:.3f},{:.3f}".format(
                    self.roi.x_min,
                    self.roi.y_min,
                    self.roi.x_max,
                    self.roi.y_max,
                ),
            ),
        ]
        if metrics is not None:
            values.extend(
                [
                    ("roi_pixels", "{}x{}".format(metrics.roi_width, metrics.roi_height)),
                    ("p10", metrics.p10),
                    ("median", metrics.median),
                    ("p90", metrics.p90),
                    ("p99", metrics.p99),
                    ("dark_fraction", metrics.dark_fraction),
                    ("bright_fraction", metrics.bright_fraction),
                    ("sharpness", metrics.sharpness),
                    ("mean_b", metrics.mean_b),
                    ("mean_g", metrics.mean_g),
                    ("mean_r", metrics.mean_r),
                ]
            )
        if controls is not None:
            values.extend(
                [
                    ("auto_exposure_gain", controls.exposure_auto),
                    ("exposure_percent", controls.exposure_time_us),
                    ("exposure_hw_min_percent", controls.exposure_min_us),
                    ("exposure_hw_max_percent", controls.exposure_max_us),
                    ("gain_percent", controls.gain),
                    ("gain_hw_min", controls.gain_min),
                    ("gain_hw_max", controls.gain_max),
                ]
            )
        status.values = [KeyValue(str(key), _text(value)) for key, value in values]
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status = [status]
        self.diagnostic_pub.publish(array)

    def _timer_callback(self, _event):
        with self._control_lock:
            if self._shutdown:
                return
            message, receipt = self._snapshot_image()
            if message is None or receipt is None:
                self._publish_diagnostic(
                    DiagnosticStatus.ERROR, "waiting for camera image"
                )
                return
            age = max(0.0, (rospy.Time.now() - receipt).to_sec())
            if age > self.image_timeout_sec:
                self._publish_diagnostic(
                    DiagnosticStatus.ERROR,
                    "camera image is stale ({:.2f} s)".format(age),
                )
                return

            try:
                image = self.bridge.imgmsg_to_cv2(
                    message, desired_encoding="bgr8"
                )
                metrics = measure_image_quality(
                    image,
                    roi=self.roi,
                    dark_threshold=self.config.dark_threshold,
                    bright_threshold=self.config.bright_threshold,
                    max_sample_width=self.max_sample_width,
                )
                controls = self._get_controls()
                self._last_controls = controls
                hardware = ImagingControlBounds(
                    exposure_min_us=controls.exposure_min_us,
                    exposure_max_us=controls.exposure_max_us,
                    gain_min=controls.gain_min,
                    gain_max=controls.gain_max,
                )
                recommendation = self.controller.recommend(
                    metrics,
                    controls.exposure_time_us,
                    controls.gain,
                    hardware,
                )
                action = recommendation.reason

                if self.enabled and not self.monitor_only:
                    if self._remaining_settle_cycles > 0:
                        self._remaining_settle_cycles -= 1
                        action = "settling_after_camera_change"
                    elif (
                        recommendation.reason == "dynamic_range_conflict"
                        and not self._manual_control_active
                        and (controls.exposure_auto or controls.gain_auto)
                    ):
                        action = "keep_native_auto_dynamic_range_conflict"
                    elif (
                        recommendation.changed
                        or controls.exposure_auto
                        or controls.gain_auto
                    ):
                        # Mark ownership before the service call: even a
                        # transport error could happen after the camera
                        # accepted part of the request, so shutdown must try
                        # to restore its native automatic modes.
                        self._manual_control_active = True
                        try:
                            applied = self._request_set(
                                False,
                                recommendation.exposure_time_us,
                                False,
                                recommendation.gain,
                            )
                        except Exception:
                            self._restore_auto()
                            raise
                        self._remaining_settle_cycles = self.settle_cycles
                        action = recommendation.reason
                        rospy.loginfo(
                            "Image-quality control: %s, ZED exposure %.0f%% -> %.0f%%, "
                            "gain %.0f%% -> %.0f%%",
                            action,
                            controls.exposure_time_us,
                            applied.exposure_time_us,
                            controls.gain,
                            applied.gain,
                        )
                elif self.monitor_only:
                    action = "monitor_only:" + recommendation.reason
                else:
                    action = "disabled:" + recommendation.reason

                self._last_action = action
                flags = quality_flags(metrics, self.config)
                if flags:
                    level = DiagnosticStatus.WARN
                    status_message = ",".join(flags)
                else:
                    level = DiagnosticStatus.OK
                    status_message = "camera input quality within configured limits"
                self._publish_diagnostic(
                    level,
                    status_message,
                    metrics=metrics,
                    controls=controls,
                    action=action,
                )
            except Exception as error:
                rospy.logerr_throttle(
                    5.0, "Image-quality controller failed: %s", error
                )
                self._publish_diagnostic(
                    DiagnosticStatus.ERROR,
                    "controller error: {}".format(error),
                )

    def shutdown(self):
        with self._control_lock:
            if self._shutdown:
                return
            self._shutdown = True
            if self.restore_auto_on_shutdown:
                self._restore_auto()


def main():
    rospy.init_node("fod_image_quality_controller")
    try:
        ImageQualityControllerNode()
        rospy.spin()
    except Exception as error:
        rospy.logfatal("FOD image-quality controller failed to start: %s", error)
        raise


if __name__ == "__main__":
    main()
