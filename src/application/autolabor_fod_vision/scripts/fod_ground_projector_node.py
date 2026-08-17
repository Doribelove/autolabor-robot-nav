#!/usr/bin/env python3
"""Project detection anchors onto the local ground and into a stable frame."""

from math import hypot

import numpy as np
import rospy
import tf2_ros
from autolabor_fod_msgs.msg import (
    FodDetectionArray,
    FodGroundObservation,
    FodGroundObservationArray,
)
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from tf.transformations import quaternion_matrix
from visualization_msgs.msg import Marker, MarkerArray

from autolabor_fod_vision.projection import ProjectionError, project_pixel_to_ground


def _csv_ints(value):
    if isinstance(value, list):
        return {int(item) for item in value}
    return {int(item.strip()) for item in str(value).split(",") if item.strip()}


def _csv_strings(value):
    if isinstance(value, list):
        return {str(item).strip() for item in value if str(item).strip()}
    return {item.strip() for item in str(value).split(",") if item.strip()}


def _transform_matrix(transform):
    rotation = transform.transform.rotation
    matrix = quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
    translation = transform.transform.translation
    matrix[0, 3] = translation.x
    matrix[1, 3] = translation.y
    matrix[2, 3] = translation.z
    return matrix


class GroundProjectorNode:
    def __init__(self):
        self.base_frame = str(rospy.get_param("~base_frame", "base_link"))
        self.output_frame = str(
            rospy.get_param("~output_frame", "camera_init")
        )
        self.ground_z = float(rospy.get_param("~ground_z", 0.0))
        self.min_range = float(rospy.get_param("~min_range", 0.15))
        self.max_range = float(rospy.get_param("~max_range", 12.0))
        self.min_confidence = float(rospy.get_param("~min_confidence", 0.25))
        self.require_forward = bool(rospy.get_param("~require_forward", True))
        self.tf_timeout = float(rospy.get_param("~tf_timeout", 0.08))
        self.base_stddev = float(rospy.get_param("~base_stddev", 0.05))
        self.range_stddev_scale = float(
            rospy.get_param("~range_stddev_scale", 0.03)
        )
        self.allowed_class_ids = _csv_ints(
            rospy.get_param("~allowed_class_ids", "")
        )
        self.allowed_class_names = _csv_strings(
            rospy.get_param("~allowed_class_names", "fod")
        )
        self.camera_info = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.observation_pub = rospy.Publisher(
            "/fod/ground_observations",
            FodGroundObservationArray,
            queue_size=1,
        )
        self.marker_pub = rospy.Publisher(
            "/fod/debug/projected_markers", MarkerArray, queue_size=1
        )
        self.status_pub = rospy.Publisher(
            "/fod/projection_status", String, queue_size=1, latch=True
        )
        rospy.Subscriber(
            "/fod_camera/camera_info",
            CameraInfo,
            self._camera_info_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            "/fod/detections",
            FodDetectionArray,
            self._detections_callback,
            queue_size=1,
        )

    def _camera_info_callback(self, message):
        self.camera_info = message

    def _publish(self, message):
        self.observation_pub.publish(message)
        self.status_pub.publish(String(data=message.status))
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for index, observation in enumerate(message.observations):
            marker = Marker()
            marker.header = message.header
            marker.ns = "fod_ground_observations"
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = observation.point
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.14
            marker.scale.y = 0.14
            marker.scale.z = 0.05
            marker.color.r = 1.0
            marker.color.g = 0.55
            marker.color.b = 0.0
            marker.color.a = 0.9
            marker.lifetime = rospy.Duration(0.5)
            markers.markers.append(marker)
        self.marker_pub.publish(markers)

    def _empty_output(self, detections, status):
        output = FodGroundObservationArray()
        output.header.stamp = detections.header.stamp
        output.header.frame_id = self.output_frame
        output.source_frame = detections.header.frame_id
        output.image_width = detections.image_width
        output.image_height = detections.image_height
        output.status = status
        return output

    def _class_allowed(self, detection):
        if detection.confidence < self.min_confidence:
            return False
        if self.allowed_class_ids and detection.class_id not in self.allowed_class_ids:
            return False
        if (
            self.allowed_class_names
            and detection.class_name not in self.allowed_class_names
        ):
            return False
        return True

    def _detections_callback(self, detections):
        output = self._empty_output(detections, "INITIALIZING")
        info = self.camera_info
        if info is None:
            output.status = "NO_CAMERA_INFO"
            self._publish(output)
            return
        if info.K[0] <= 0.0 or info.K[4] <= 0.0:
            output.status = "UNCALIBRATED"
            self._publish(output)
            return
        if info.width != detections.image_width or info.height != detections.image_height:
            output.status = "CAMERA_INFO_SIZE_MISMATCH"
            self._publish(output)
            return
        if info.header.frame_id and info.header.frame_id != detections.header.frame_id:
            output.status = "CAMERA_INFO_FRAME_MISMATCH"
            self._publish(output)
            return

        candidates = [
            (index, detection)
            for index, detection in enumerate(detections.detections)
            if self._class_allowed(detection)
        ]
        if not candidates:
            output.status = "NO_MATCHING_DETECTIONS"
            self._publish(output)
            return

        stamp = detections.header.stamp
        if stamp == rospy.Time(0):
            stamp = rospy.Time.now()
            output.header.stamp = stamp
        try:
            base_from_camera = self.tf_buffer.lookup_transform(
                self.base_frame,
                detections.header.frame_id,
                stamp,
                rospy.Duration(self.tf_timeout),
            )
            base_from_camera_matrix = _transform_matrix(base_from_camera)
            if self.output_frame == self.base_frame:
                output_from_base_matrix = np.eye(4)
            else:
                output_from_base = self.tf_buffer.lookup_transform(
                    self.output_frame,
                    self.base_frame,
                    stamp,
                    rospy.Duration(self.tf_timeout),
                )
                output_from_base_matrix = _transform_matrix(output_from_base)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as error:
            output.status = "TF_ERROR"
            rospy.logwarn_throttle(5.0, "FOD projection TF unavailable: %s", error)
            self._publish(output)
            return

        camera_origin = base_from_camera_matrix[:3, 3]
        rotation = base_from_camera_matrix[:3, :3]
        rejected = 0
        for detection_index, detection in candidates:
            try:
                u = float(detection.anchor_px.x)
                v = float(detection.anchor_px.y)
                if not (0.0 <= u < info.width and 0.0 <= v < info.height):
                    raise ProjectionError("anchor is outside the image")
                point_base = project_pixel_to_ground(
                    u=u,
                    v=v,
                    camera_matrix=info.K,
                    distortion=info.D,
                    rotation_base_from_camera=rotation,
                    camera_origin_in_base=camera_origin,
                    ground_z=self.ground_z,
                    distortion_model=info.distortion_model,
                )
                distance = hypot(point_base[0], point_base[1])
                if self.require_forward and point_base[0] <= 0.0:
                    raise ProjectionError("target is not in front of base_link")
                if distance < self.min_range or distance > self.max_range:
                    raise ProjectionError("target is outside configured range")

                point_output = output_from_base_matrix.dot(
                    np.array([point_base[0], point_base[1], point_base[2], 1.0])
                )[:3]
                standard_deviation = (
                    self.base_stddev + self.range_stddev_scale * distance
                )
                covariance_base = np.diag(
                    [standard_deviation ** 2, standard_deviation ** 2]
                )
                rotation_xy = output_from_base_matrix[:2, :2]
                covariance_output = rotation_xy.dot(covariance_base).dot(
                    rotation_xy.T
                )

                observation = FodGroundObservation()
                observation.source_detection_index = detection_index
                observation.class_id = detection.class_id
                observation.class_name = detection.class_name
                observation.confidence = detection.confidence
                observation.point = Point(
                    x=float(point_output[0]),
                    y=float(point_output[1]),
                    z=float(point_output[2]),
                )
                observation.covariance_xy = [
                    float(covariance_output[0, 0]),
                    float(covariance_output[0, 1]),
                    float(covariance_output[1, 0]),
                    float(covariance_output[1, 1]),
                ]
                observation.bbox = detection.bbox
                observation.anchor_px = detection.anchor_px
                output.observations.append(observation)
            except ProjectionError as error:
                rejected += 1
                rospy.logdebug("Rejected FOD detection projection: %s", error)

        output.status = "OK" if output.observations else "NO_VALID_PROJECTIONS"
        if rejected:
            output.status += "_REJECTED_{}".format(rejected)
        self._publish(output)


def main():
    rospy.init_node("fod_ground_projector")
    GroundProjectorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
