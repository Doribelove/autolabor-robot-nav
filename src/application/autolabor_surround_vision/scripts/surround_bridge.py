#!/usr/bin/env python3
"""Read TF/localization; publish scoped observations. No navigation/control outputs."""
from pathlib import Path
import sys
import threading
import time
import uuid

import numpy as np
import rospy
import tf2_ros
from tf.transformations import quaternion_matrix
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from autolabor_surround_vision.msg import SurroundObservationArray, SurroundTargetArray


def main():
    rospy.init_node("surround_observation_bridge")
    root = Path(rospy.get_param("~code_root")).resolve()
    sys.path.insert(0, str(root))
    from distribution.j6m_bpu_navigation.surround.contracts import atomic_json, read_json, transform
    from distribution.j6m_bpu_navigation.surround.ros_adapter import messages, legacy_ground
    directory = Path(rospy.get_param("~session_dir"))
    pose_output = Path(rospy.get_param("~pose_output"))
    map_id = rospy.get_param("~map_id")
    floor = float(rospy.get_param("~map_covariance_floor_m2"))
    if not map_id or not np.isfinite(floor) or floor <= 0:
        raise ValueError("A real map version and measured covariance floor are required")
    base = rospy.get_param("~base_frame", "base_link")
    buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10))
    listener = tf2_ros.TransformListener(buffer)
    observations = rospy.Publisher("observations", SurroundObservationArray, queue_size=1)
    targets = rospy.Publisher("targets", SurroundTargetArray, queue_size=1)
    status_pub = rospy.Publisher("status", String, queue_size=1)
    old = None
    if rospy.get_param("~publish_legacy_observations", False):
        from autolabor_fod_msgs.msg import FodGroundObservationArray
        old = rospy.Publisher("fod_ground_observations", FodGroundObservationArray, queue_size=1)
    guard = threading.RLock()
    status = {"text": "", "received": 0.}
    epoch = {"session": uuid.uuid4().hex, "valid": False, "stamp": 0, "correction": None}

    def localization(msg):
        with guard:
            status.update(text=msg.data, received=time.monotonic())

    def matrix_tf(target, source, stamp):
        value = buffer.lookup_transform(target, source, stamp, rospy.Duration(.02)).transform
        q = [value.rotation.x, value.rotation.y, value.rotation.z, value.rotation.w]
        if not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > .001:
            raise ValueError("Invalid source-time TF quaternion")
        t = quaternion_matrix(q)
        t[:3, 3] = [value.translation.x, value.translation.y, value.translation.z]
        return transform(t)

    def odometry(msg):
        with guard:
            stamp_ns = msg.header.stamp.to_nsec()
            now_ns = rospy.Time.now().to_nsec()
            ready = ("state=LOCALIZED" in status["text"].split(";")
                     and 0 <= time.monotonic() - status["received"] <= .5
                     and 0 <= now_ns - stamp_ns <= 500_000_000)
            try:
                if msg.header.frame_id.lstrip("/") != "camera_init":
                    raise ValueError("Expected FAST-LIO odometry in camera_init")
                map_base = matrix_tf("map", base, msg.header.stamp)
                correction = matrix_tf("map", "camera_init", msg.header.stamp)
                if epoch["correction"] is not None:
                    delta = np.linalg.inv(epoch["correction"]) @ correction
                    angle = np.arccos(np.clip((np.trace(delta[:3, :3]) - 1) / 2, -1, 1))
                    if np.linalg.norm(delta[:3, 3]) > .25 or angle > .05:
                        epoch["session"] = uuid.uuid4().hex
                if ready and (not epoch["valid"] or stamp_ns <= epoch["stamp"]):
                    epoch["session"] = uuid.uuid4().hex
                covariance = np.asarray(msg.pose.covariance).reshape(6, 6)[:3, :3]
                if not np.isfinite(covariance).all() or np.linalg.eigvalsh(covariance).min() < -1e-8:
                    raise ValueError("Invalid odometry covariance")
                # FAST-LIO covariance lives in camera_init; rotate to map and
                # add a separately measured map-localization uncertainty floor.
                r = correction[:3, :3]
                xy = (r @ covariance @ r.T)[:2, :2] + np.eye(2) * floor
                atomic_json(pose_output, dict(stamp_ns=stamp_ns, map_id=map_id,
                    localization_session=epoch["session"], T_map_base=map_base.tolist(),
                    covariance_xy=xy.tolist(), valid=bool(ready)))
                epoch.update(valid=ready, stamp=stamp_ns, correction=correction)
            except (tf2_ros.TransformException, ValueError, OSError) as e:
                # No invented identity transform. A stale pose file cannot
                # bracket new frames and will be rejected by the native service.
                epoch["valid"] = False
                rospy.logwarn_throttle(2, "surround pose unavailable: %s", str(e))

    rospy.Subscriber(rospy.get_param("~localization_status_topic", "/fast_lio/localization_status"),
                     String, localization, queue_size=1)
    rospy.Subscriber(rospy.get_param("~odom_topic", "/Odometry"), Odometry, odometry, queue_size=1)
    rate, last_source = rospy.Rate(10), None
    while not rospy.is_shutdown():
        try:
            state = read_json(directory / "state.json")
            obs, target_list = messages(state, rospy.Time.now().to_nsec())
            identity = (state["session"], obs.header.stamp.to_nsec(),
                        tuple(o.observation_id for o in obs.observations))
            if identity != last_source:
                observations.publish(obs)
                if old:
                    for camera in {o.source.camera for o in obs.observations}:
                        geometry = state["camera_results"][camera]["inference_image"]
                        old.publish(legacy_ground(obs, (geometry["width"], geometry["height"]), camera))
                last_source = identity
            targets.publish(target_list)
            status_pub.publish(String("OBSERVATION_ONLY;production_ready=false;motion_eligible=false"))
        except (OSError, ValueError, KeyError) as e:
            status_pub.publish(String("UNAVAILABLE;" + str(e)))
        rate.sleep()


if __name__ == "__main__":
    main()
