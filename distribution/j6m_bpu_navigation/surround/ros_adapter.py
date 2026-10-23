"""Explicit versioned messages. Imports generated ROS messages only on demand."""
import math

from . import SCHEMA


def stamp(ns):
    import rospy
    return rospy.Time(int(ns) // 1_000_000_000, int(ns) % 1_000_000_000)


def source(value):
    from autolabor_surround_vision.msg import SurroundSource
    message = SurroundSource()
    message.camera = value["camera"]
    message.frame_key = value["frame_key"]
    message.sequence = int(value["frame_key"].rsplit("/", 1)[1])
    message.raw_stamp = value["raw_stamp"]
    message.exposure_stamp = stamp(value["stamp_ns"])
    message.clock_id = value["clock_id"]
    message.clock_mapping_id = value["clock_mapping_id"]
    message.calibration_id = value["calibration_id"]
    message.rectification_id = value["rectification_id"]
    x1, y1, x2, y2 = value["bbox"]
    message.bbox.x_offset, message.bbox.y_offset = max(0, math.floor(x1)), max(0, math.floor(y1))
    message.bbox.width = max(1, math.ceil(x2) - message.bbox.x_offset)
    message.bbox.height = max(1, math.ceil(y2) - message.bbox.y_offset)
    return message


def messages(state, now_ns):
    from autolabor_surround_vision.msg import (
        SurroundObservation, SurroundObservationArray, SurroundTarget, SurroundTargetArray)
    from geometry_msgs.msg import Point
    if (state.get("schema") != SCHEMA or state.get("synthetic") or state.get("replay")
            or state.get("service_stopped") or state.get("localization_valid") is not True
            or not 0 <= now_ns - state["produced_ns"] <= 500_000_000):
        raise ValueError("Offline, stopped, stale or unsupported state cannot enter live ROS")
    observations, targets = SurroundObservationArray(), SurroundTargetArray()
    identity = state.get("map_identity") or ("", "", state["calibration_id"])
    for msg in (observations, targets):
        msg.schema, msg.session = SCHEMA, state["session"]
        msg.header.frame_id = "map"
        msg.map_id, msg.localization_session, msg.calibration_id = identity
        msg.detector_model_sha256 = state["model_identity"]["detector_zed"]["sha256"]
        msg.classifier_model_sha256 = state["model_identity"]["classifier"]["sha256"]
        msg.production_ready = msg.motion_eligible = False
    targets.header.stamp = stamp(now_ns)
    results = list(state.get("camera_results", {}).values())
    if not results and state.get("latest_result"):
        results = [state["latest_result"]]
    fresh = [r for r in results if 0 <= now_ns - r.get("source", {}).get("stamp_ns", 0) <= 500_000_000]
    observations.header.stamp = stamp(max((r["source"]["stamp_ns"] for r in fresh), default=0))
    for result in fresh:
        for o in result.get("observations", []):
            if (o["map_id"], o["localization_session"], o["calibration_id"]) != tuple(identity):
                continue
            m = SurroundObservation()
            m.observation_id, m.position_source = o["id"], o["position_source"]
            m.detector_class_id, m.material = o["class_id"], o["material"]
            m.detect_confidence, m.classify_confidence = o["detect_confidence"], o["classify_confidence"]
            m.material_valid = o["probabilities"] is not None
            m.material_probabilities = o["probabilities"] or [0.] * 5
            m.point_map = Point(*o["map_xyz"])
            m.covariance_xy = [v for row in o["covariance_xy"] for v in row]
            m.source = source(o)
            m.depth_valid = False
            observations.observations.append(m)
    for target in state["targets"]:
        m = SurroundTarget()
        m.target_id, m.status, m.material = target["id"], target["status"], target["material"]
        m.detect_confidence, m.classify_confidence = target["detect_confidence"], target["classify_confidence"]
        m.point_map = Point(*target["map_xyz"])
        m.covariance_xy = [v for row in target["covariance_xy"] for v in row]
        m.first_observed, m.last_observed = stamp(target["first_observed_ns"]), stamp(target["last_observed_ns"])
        m.independent_observations, m.visible_misses = target["hits"], target["visible_misses"]
        m.sources = [source(s) for s in target["sources"]]
        m.production_ready = m.motion_eligible = False
        targets.targets.append(m)
    return observations, targets


def legacy_ground(observations, image_size, camera=None):
    """Existing FOD observation type on a separate experimental topic only."""
    from autolabor_fod_msgs.msg import FodGroundObservation, FodGroundObservationArray
    import copy
    names = {o.source.camera for o in observations.observations}
    if camera is None:
        if len(names) != 1:
            raise ValueError("Legacy array needs one camera with one exposure")
        camera = next(iter(names))
    chosen = [o for o in observations.observations if o.source.camera == camera]
    if len({o.source.exposure_stamp.to_nsec() for o in chosen}) > 1:
        raise ValueError("Mixed exposure in legacy observation array")
    result = FodGroundObservationArray()
    result.header = copy.deepcopy(observations.header)
    if chosen:
        result.header.stamp = chosen[0].source.exposure_stamp
    result.image_width, result.image_height = image_size
    result.source_frame = "surround_%s_rectified_optical" % camera
    result.status = "OBSERVATION_ONLY_GROUND_PLANE_NO_MEASURED_DEPTH"
    for i, o in enumerate(chosen):
        m = FodGroundObservation()
        m.source_detection_index, m.class_id, m.class_name = i, o.detector_class_id, "trash"
        m.confidence, m.point, m.covariance_xy, m.bbox = o.detect_confidence, o.point_map, o.covariance_xy, o.source.bbox
        m.anchor_px.x = m.bbox.x_offset + .5 * m.bbox.width
        m.anchor_px.y = m.bbox.y_offset + m.bbox.height
        result.observations.append(m)
    return result
