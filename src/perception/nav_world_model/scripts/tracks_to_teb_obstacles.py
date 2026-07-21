#!/usr/bin/env python3
"""Publish healthy laser tracks as velocity-aware custom TEB obstacles."""

import threading

import rospy
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
from nav_world_model.msg import TrackedObstacle, TrackedObstacleArray, WorldModelHealth
from nav_world_model.teb_bridge import local_track_to_fixed
from tf.transformations import euler_from_quaternion


DYNAMIC_CLASSES = {
    TrackedObstacle.MOTION_CROSSING,
    TrackedObstacle.MOTION_HEAD_ON,
    TrackedObstacle.MOTION_FOLLOWING,
    TrackedObstacle.MOTION_DEPARTING,
}


class TracksToTebObstacles:
    def __init__(self):
        self.lock = threading.RLock()
        self.odom = None
        self.tracks = None
        self.health_valid = False
        self.health_sequence = None
        self.minimum_confidence = float(rospy.get_param("~minimum_confidence", 0.55))
        self.fixed_frame = str(rospy.get_param("~fixed_frame", "odom"))
        input_tracks = str(rospy.get_param("~input_tracks", "/nav_world_model/tracks"))
        input_health = str(rospy.get_param("~input_health", "/nav_world_model/health"))
        input_odometry = str(rospy.get_param("~input_odometry", "/odom"))
        output = str(rospy.get_param(
            "~output_obstacles", "/move_base/TebLocalPlannerROS/obstacles"
        ))
        self.publisher = rospy.Publisher(output, ObstacleArrayMsg, queue_size=1)
        rospy.Subscriber(input_odometry, Odometry, self._odom, queue_size=5)
        rospy.Subscriber(input_health, WorldModelHealth, self._health, queue_size=5)
        rospy.Subscriber(input_tracks, TrackedObstacleArray, self._tracks, queue_size=2)

    def _odom(self, message):
        with self.lock:
            self.odom = message

    def _health(self, message):
        with self.lock:
            self.health_valid = bool(
                message.valid and not message.stale and message.tracker_valid
            )
            self.health_sequence = int(message.world_model_seq)
        self._publish_if_ready()

    @staticmethod
    def _radius(track):
        return max(
            (max(abs(point.x), abs(point.y)) for point in track.footprint.points),
            default=0.25,
        )

    def _empty(self, stamp):
        result = ObstacleArrayMsg()
        result.header.stamp = stamp
        result.header.frame_id = self.fixed_frame
        self.publisher.publish(result)

    def _tracks(self, message):
        with self.lock:
            self.tracks = message
        self._publish_if_ready()

    def _publish_if_ready(self):
        with self.lock:
            message = self.tracks
            odom = self.odom
            if (
                message is None or self.health_sequence is None
                or int(message.world_model_seq) != self.health_sequence
            ):
                return
            healthy = self.health_valid
        if odom is None or not healthy:
            self._empty(message.header.stamp)
            return
        orientation = odom.pose.pose.orientation
        yaw = euler_from_quaternion((
            orientation.x, orientation.y, orientation.z, orientation.w
        ))[2]
        robot_speed = float(odom.twist.twist.linear.x)
        result = ObstacleArrayMsg()
        result.header.stamp = message.header.stamp
        result.header.frame_id = self.fixed_frame
        for track in message.obstacles:
            if (
                track.motion_class not in DYNAMIC_CLASSES
                or track.confidence < self.minimum_confidence
            ):
                continue
            converted = local_track_to_fixed(
                track_id=track.track_id,
                local_x=track.pose.pose.position.x,
                local_y=track.pose.pose.position.y,
                relative_vx=track.velocity.twist.linear.x,
                relative_vy=track.velocity.twist.linear.y,
                radius=self._radius(track),
                robot_x=odom.pose.pose.position.x,
                robot_y=odom.pose.pose.position.y,
                robot_yaw=yaw,
                robot_linear_velocity=robot_speed,
            )
            obstacle = ObstacleMsg()
            obstacle.id = converted.track_id
            obstacle.polygon.points = [Point32(x=converted.x, y=converted.y, z=0.0)]
            obstacle.radius = converted.radius
            obstacle.orientation.w = 1.0
            obstacle.velocities.twist.linear.x = converted.vx
            obstacle.velocities.twist.linear.y = converted.vy
            result.obstacles.append(obstacle)
        self.publisher.publish(result)


def main():
    rospy.init_node("tracks_to_teb_obstacles")
    TracksToTebObstacles()
    rospy.spin()


if __name__ == "__main__":
    main()
