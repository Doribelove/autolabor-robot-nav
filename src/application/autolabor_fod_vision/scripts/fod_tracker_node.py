#!/usr/bin/env python3
"""Track projected FOD observations in a stable frame and select a safe target."""

import threading

import rospy
from autolabor_fod_msgs.msg import (
    FodGroundObservationArray,
    FodTarget,
    FodTrack,
    FodTrackArray,
)
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from autolabor_fod_vision.tracking import GroundObservation, MultiTargetTracker


def _fill_pose(pose, x, y, covariance_xx, covariance_xy, covariance_yy):
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation.w = 1.0
    pose.covariance = [0.0] * 36
    pose.covariance[0] = max(1e-6, float(covariance_xx))
    pose.covariance[1] = float(covariance_xy)
    pose.covariance[6] = float(covariance_xy)
    pose.covariance[7] = max(1e-6, float(covariance_yy))
    pose.covariance[14] = 0.0025
    pose.covariance[21] = 1e6
    pose.covariance[28] = 1e6
    pose.covariance[35] = 1e6


class TrackerNode:
    def __init__(self):
        self.tracking_frame = str(rospy.get_param("~tracking_frame", "camera_init"))
        self.selection_policy = str(
            rospy.get_param("~selection_policy", "reject_ambiguous")
        )
        self.publish_rate = float(rospy.get_param("~publish_rate", 10.0))
        self.tracker = MultiTargetTracker(
            association_distance=float(
                rospy.get_param("~association_distance", 0.6)
            ),
            alpha=float(rospy.get_param("~alpha", 0.45)),
            min_hits=int(rospy.get_param("~min_hits", 3)),
            max_age=float(rospy.get_param("~max_age", 0.8)),
            max_misses=int(rospy.get_param("~max_misses", 10)),
        )
        self.lock = threading.Lock()
        self.current_frame = self.tracking_frame
        self.last_input_status = "WAITING_FOR_OBSERVATIONS"
        self.track_pub = rospy.Publisher(
            "/fod/tracks", FodTrackArray, queue_size=1
        )
        self.target_pub = rospy.Publisher(
            "/fod/target", FodTarget, queue_size=1
        )
        self.target_pose_pub = rospy.Publisher(
            "/fod/target_pose", PoseWithCovarianceStamped, queue_size=1
        )
        self.marker_pub = rospy.Publisher(
            "/fod/debug/track_markers", MarkerArray, queue_size=1
        )
        rospy.Subscriber(
            "/fod/ground_observations",
            FodGroundObservationArray,
            self._observations_callback,
            queue_size=1,
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate), self._timer_callback
        )

    def _observations_callback(self, message):
        stamp = (
            message.header.stamp.to_sec()
            if message.header.stamp != rospy.Time(0)
            else rospy.Time.now().to_sec()
        )
        observations = [
            GroundObservation(
                class_id=item.class_id,
                class_name=item.class_name,
                confidence=item.confidence,
                x=item.point.x,
                y=item.point.y,
                covariance_xx=item.covariance_xy[0],
                covariance_xy=0.5
                * (item.covariance_xy[1] + item.covariance_xy[2]),
                covariance_yy=item.covariance_xy[3],
            )
            for item in message.observations
        ]
        with self.lock:
            if self.current_frame and message.header.frame_id != self.current_frame:
                rospy.logerr(
                    "FOD tracking frame changed from %s to %s; resetting tracks",
                    self.current_frame,
                    message.header.frame_id,
                )
                self.tracker.tracks = []
                self.tracker._last_update = None
            self.current_frame = message.header.frame_id or self.tracking_frame
            self.last_input_status = message.status
            self.tracker.update(observations, stamp)

    def _track_message(self, track):
        message = FodTrack()
        message.track_id = track.track_id
        message.class_id = track.class_id
        message.class_name = track.class_name
        message.confidence = track.confidence
        _fill_pose(
            message.pose,
            track.x,
            track.y,
            track.covariance_xx,
            track.covariance_xy,
            track.covariance_yy,
        )
        message.first_observed = rospy.Time.from_sec(track.first_observed)
        message.last_observed = rospy.Time.from_sec(track.last_observed)
        message.hit_count = track.hit_count
        message.miss_count = track.miss_count
        message.confirmed = track.confirmed(self.tracker.min_hits)
        return message

    def _publish_markers(self, header, tracks):
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        for track in tracks:
            marker = Marker()
            marker.header = header
            marker.ns = "fod_tracks"
            marker.id = track.track_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = track.x
            marker.pose.position.y = track.y
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.18
            marker.scale.y = 0.18
            marker.scale.z = 0.05
            marker.color.r = 0.1 if track.confirmed(self.tracker.min_hits) else 0.8
            marker.color.g = 0.9 if track.confirmed(self.tracker.min_hits) else 0.8
            marker.color.b = 0.2
            marker.color.a = 0.9
            marker.lifetime = rospy.Duration(0.3)
            array.markers.append(marker)
        self.marker_pub.publish(array)

    def _timer_callback(self, _event):
        now = rospy.Time.now()
        now_sec = now.to_sec()
        with self.lock:
            self.tracker.prune(now_sec)
            tracks = list(self.tracker.tracks)
            target_track, status, candidate_count = self.tracker.select_target(
                now_sec, self.selection_policy
            )
            frame_id = self.current_frame or self.tracking_frame
            input_status = self.last_input_status

        header = Header(stamp=now, frame_id=frame_id)
        track_array = FodTrackArray()
        track_array.header = header
        track_array.tracks = [self._track_message(track) for track in tracks]
        self.track_pub.publish(track_array)

        target = FodTarget()
        target.header = header
        target.valid = target_track is not None
        if status == "NO_CONFIRMED_TARGET":
            if tracks:
                target.status = "CONFIRMING"
            elif input_status.startswith("OK"):
                target.status = "NO_TARGET"
            else:
                target.status = input_status
        else:
            target.status = status
        target.candidate_count = candidate_count
        if target_track is not None:
            target.track_id = target_track.track_id
            target.class_id = target_track.class_id
            target.class_name = target_track.class_name
            target.confidence = target_track.confidence
            _fill_pose(
                target.pose,
                target_track.x,
                target_track.y,
                target_track.covariance_xx,
                target_track.covariance_xy,
                target_track.covariance_yy,
            )
            target.last_observed = rospy.Time.from_sec(
                target_track.last_observed
            )
            target.age_sec = max(0.0, now_sec - target_track.last_observed)
            pose = PoseWithCovarianceStamped()
            pose.header = header
            pose.pose = target.pose
            self.target_pose_pub.publish(pose)
        else:
            target.class_id = -1
        self.target_pub.publish(target)
        self._publish_markers(header, tracks)


def main():
    rospy.init_node("fod_tracker")
    TrackerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
