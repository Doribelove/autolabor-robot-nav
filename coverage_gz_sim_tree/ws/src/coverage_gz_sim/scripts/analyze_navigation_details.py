#!/usr/bin/env python3
"""Extract reproducible transition, sweep, and fault details from a sim bag."""

import argparse
import hashlib
import json
import math
import os

import rosbag


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def quaternion_yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z
               + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y
                     + quaternion.z * quaternion.z),
    )


def polyline_length(points):
    return sum(
        math.hypot(second[0] - first[0], second[1] - first[1])
        for first, second in zip(points, points[1:])
    )


def maximum_chord_deviation(points):
    if len(points) < 2:
        return 0.0
    start = points[0]
    end = points[-1]
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    if length <= 1.0e-9:
        return max(
            math.hypot(point[0] - start[0], point[1] - start[1])
            for point in points
        )
    return max(
        abs((point[0] - start[0]) * dy - (point[1] - start[1]) * dx)
        / length
        for point in points
    )


def path_signature(poses):
    digest = hashlib.sha256()
    for pose in poses:
        position = pose.pose.position
        yaw = quaternion_yaw(pose.pose.orientation)
        digest.update(
            ("{:.3f},{:.3f},{:.3f};".format(
                float(position.x), float(position.y), yaw
            )).encode("ascii")
        )
    return digest.hexdigest()


def path_geometry(poses):
    samples = [
        (
            float(pose.pose.position.x),
            float(pose.pose.position.y),
            quaternion_yaw(pose.pose.orientation),
        )
        for pose in poses
    ]
    cleaned = []
    for sample in samples:
        if (
            not cleaned
            or math.hypot(
                sample[0] - cleaned[-1][0], sample[1] - cleaned[-1][1]
            ) > 1.0e-5
        ):
            cleaned.append(sample)
    if len(cleaned) < 2:
        return {
            "pose_count": len(samples),
            "length_m": 0.0,
            "direct_distance_m": 0.0,
            "path_to_direct_ratio": None,
            "maximum_chord_deviation_m": 0.0,
            "gear_changes": 0,
            "parts": [],
            "shape": "stationary",
        }

    edge_gears = []
    for first, second in zip(cleaned, cleaned[1:]):
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        yaw_step = wrap_angle(second[2] - first[2])
        mid_yaw = first[2] + 0.5 * yaw_step
        projection = dx * math.cos(mid_yaw) + dy * math.sin(mid_yaw)
        edge_gears.append(1 if projection >= 0.0 else -1)

    runs = []
    run_start = 0
    for edge_index in range(1, len(edge_gears)):
        if edge_gears[edge_index] != edge_gears[run_start]:
            runs.append((run_start, edge_index, edge_gears[run_start]))
            run_start = edge_index
    runs.append((run_start, len(edge_gears), edge_gears[run_start]))

    parts = []
    shape_parts = []
    for first_edge, end_edge, gear in runs:
        part_samples = cleaned[first_edge:end_edge + 1]
        net_heading = sum(
            wrap_angle(second[2] - first[2])
            for first, second in zip(part_samples, part_samples[1:])
        )
        absolute_heading = sum(
            abs(wrap_angle(second[2] - first[2]))
            for first, second in zip(part_samples, part_samples[1:])
        )
        length = polyline_length(part_samples)
        if absolute_heading < math.radians(5.0):
            turn = "straight"
        elif net_heading > math.radians(2.0):
            turn = "left_arc"
        elif net_heading < -math.radians(2.0):
            turn = "right_arc"
        else:
            turn = "compound_curve"
        gear_name = "forward" if gear > 0 else "reverse"
        parts.append({
            "gear": gear_name,
            "length_m": length,
            "net_heading_change_deg": math.degrees(net_heading),
            "absolute_heading_change_deg": math.degrees(absolute_heading),
            "turn": turn,
            "maximum_chord_deviation_m": maximum_chord_deviation(
                part_samples
            ),
        })
        shape_parts.append("{}_{}_{:.2f}m".format(gear_name, turn, length))

    points = [(sample[0], sample[1]) for sample in cleaned]
    direct = math.hypot(
        cleaned[-1][0] - cleaned[0][0],
        cleaned[-1][1] - cleaned[0][1],
    )
    length = polyline_length(cleaned)
    return {
        "pose_count": len(samples),
        "length_m": length,
        "direct_distance_m": direct,
        "path_to_direct_ratio": length / direct if direct > 1.0e-6 else None,
        "maximum_chord_deviation_m": maximum_chord_deviation(points),
        "gear_changes": max(0, len(parts) - 1),
        "parts": parts,
        "shape": " -> ".join(shape_parts),
        "start": {"x": cleaned[0][0], "y": cleaned[0][1],
                  "yaw_deg": math.degrees(cleaned[0][2])},
        "end": {"x": cleaned[-1][0], "y": cleaned[-1][1],
                "yaw_deg": math.degrees(cleaned[-1][2])},
    }


def trajectory_geometry(samples):
    if not samples:
        return {
            "sample_count": 0,
            "distance_m": 0.0,
            "direct_distance_m": 0.0,
            "maximum_chord_deviation_m": 0.0,
        }
    points = [(sample[1], sample[2]) for sample in samples]
    return {
        "sample_count": len(samples),
        "distance_m": polyline_length(points),
        "direct_distance_m": math.hypot(
            points[-1][0] - points[0][0], points[-1][1] - points[0][1]
        ),
        "maximum_chord_deviation_m": maximum_chord_deviation(points),
        "net_heading_change_deg": math.degrees(wrap_angle(
            samples[-1][3] - samples[0][3]
        )),
        "start": {"x": points[0][0], "y": points[0][1],
                  "yaw_deg": math.degrees(samples[0][3])},
        "end": {"x": points[-1][0], "y": points[-1][1],
                "yaw_deg": math.degrees(samples[-1][3])},
    }


def samples_between(samples, start, end):
    return [sample for sample in samples if start <= sample[0] <= end]


def signed_motion_distances(samples):
    """Split measured odometry arc length by the commanded chassis gear."""
    forward = 0.0
    reverse = 0.0
    stationary = 0.0
    for first, second in zip(samples, samples[1:]):
        distance = math.hypot(second[1] - first[1], second[2] - first[2])
        speed = 0.5 * (first[4] + second[4])
        if speed > 1.0e-3:
            forward += distance
        elif speed < -1.0e-3:
            reverse += distance
        else:
            stationary += distance
    return {
        "forward_distance_m": forward,
        "reverse_distance_m": reverse,
        "stationary_pose_change_m": stationary,
    }


def motion_timing(samples):
    """Measure where a transition spent time instead of inferring from length."""
    forward_time = 0.0
    reverse_time = 0.0
    stopped_time = 0.0
    maximum_forward_speed = 0.0
    maximum_reverse_speed = 0.0
    for first, second in zip(samples, samples[1:]):
        duration = max(0.0, second[0] - first[0])
        speed = 0.5 * (first[4] + second[4])
        maximum_forward_speed = max(maximum_forward_speed, speed)
        maximum_reverse_speed = max(maximum_reverse_speed, -speed)
        if speed > 0.02:
            forward_time += duration
        elif speed < -0.02:
            reverse_time += duration
        else:
            stopped_time += duration
    return {
        "forward_motion_sec": forward_time,
        "reverse_motion_sec": reverse_time,
        "stopped_sec": stopped_time,
        "maximum_forward_speed_mps": maximum_forward_speed,
        "maximum_reverse_speed_mps": maximum_reverse_speed,
    }


def path_for_interval(paths, start, end):
    return [
        path for path in paths
        if start - 1.00 <= path["first_stamp"] <= end + 0.05
    ]


def project_pose_to_path(path, pose):
    poses = path.get("poses", [])
    if len(poses) < 2 or pose is None:
        return None
    first = poses[0].pose.position
    last = poses[-1].pose.position
    dx = float(last.x) - float(first.x)
    dy = float(last.y) - float(first.y)
    length = math.hypot(dx, dy)
    if length <= 1.0e-9:
        return None
    tx = dx / length
    ty = dy / length
    relative_x = pose[1] - float(first.x)
    relative_y = pose[2] - float(first.y)
    return {
        "stamp": pose[0],
        "x": pose[1],
        "y": pose[2],
        "yaw_deg": math.degrees(pose[3]),
        "along_m": relative_x * tx + relative_y * ty,
        "cross_track_m": relative_x * ty - relative_y * tx,
        "heading_error_deg": math.degrees(abs(wrap_angle(
            pose[3] - math.atan2(dy, dx)
        ))),
        "speed_mps": pose[4],
    }


def entry_fault_effects(data):
    effects = []
    for event in data["faults"]:
        if event.get("kind") != "entry_offset_injected":
            continue
        stamp = float(event.get("stamp", 0.0))
        pre_x = float(event.get("pre_x", 0.0))
        pre_y = float(event.get("pre_y", 0.0))
        pre_yaw = float(event.get("pre_yaw", 0.0))
        requested_distance = abs(float(event.get("lateral_offset_m", 0.0)))
        requested_yaw = abs(float(event.get("yaw_offset_rad", 0.0)))
        post = None
        for sample in data["odometry"]:
            if sample[0] < stamp:
                continue
            displacement = math.hypot(sample[1] - pre_x, sample[2] - pre_y)
            yaw_change = abs(wrap_angle(sample[3] - pre_yaw))
            if (
                displacement >= max(0.01, 0.80 * requested_distance)
                or yaw_change >= max(0.01, 0.80 * requested_yaw)
            ):
                post = sample
                break
            if sample[0] > stamp + 1.0:
                break
        candidates = [
            path for path in data["sweep_paths"]
            if path["segment_index"] + 1 == int(event.get("segment", -1))
        ]
        path = (
            min(candidates, key=lambda item: abs(
                item["first_stamp"] - stamp
            ))
            if candidates else None
        )
        pre = (stamp, pre_x, pre_y, pre_yaw, 0.0)
        recovery_state = next((
            status for status in data["statuses"]
            if status["stamp"] >= stamp
            and status["state"] == "TRANSITING"
            and status["segment"] == int(event.get("segment", -1))
        ), None)
        sweep_retry = next((
            status for status in data["statuses"]
            if status["stamp"] > stamp
            and status["state"] == "SWEEPING"
            and status["segment"] == int(event.get("segment", -1))
        ), None)
        recovery_deadline = (
            sweep_retry["stamp"] if sweep_retry is not None
            else data["bag_end"]
        )
        recovery_paths = sorted((
            item for item in data["hybrid_paths"]
            if stamp + 0.05 <= item["first_stamp"] <= recovery_deadline
        ), key=lambda item: item["first_stamp"])
        recovery_actions = sorted((
            item for item in data.get("hybrid_action_paths", [])
            if stamp + 0.05 <= item["first_stamp"] <= recovery_deadline
        ), key=lambda item: item["first_stamp"])
        recovery_path = recovery_paths[0] if recovery_paths else None
        recovery_action = recovery_actions[0] if recovery_actions else None
        recovery_start_stamp = (
            recovery_action["first_stamp"] if recovery_action is not None
            else (
                recovery_path["first_stamp"]
                if recovery_path is not None else (
                    recovery_state["stamp"] if recovery_state else None
                )
            )
        )
        deviation_log = next((
            log for log in data["relevant_logs"]
            if log["stamp"] >= stamp
            and log["stamp"] <= recovery_deadline
            and "left its fixed-gear path" in log["message"]
        ), None)
        actual_recovery_samples = (
            samples_between(
                data["odometry"], recovery_start_stamp,
                sweep_retry["stamp"],
            )
            if recovery_start_stamp is not None and sweep_retry is not None
            else []
        )
        record = {
            "event": event,
            "measured_map_displacement_m": (
                math.hypot(post[1] - pre_x, post[2] - pre_y)
                if post is not None else None
            ),
            "measured_yaw_change_deg": (
                math.degrees(abs(wrap_angle(post[3] - pre_yaw)))
                if post is not None else None
            ),
            "post_sample_delay_sec": (
                post[0] - stamp if post is not None else None
            ),
            "stale_path_canceled_stamp": (
                deviation_log["stamp"] if deviation_log else None
            ),
            "fault_to_stale_path_cancel_sec": (
                deviation_log["stamp"] - stamp if deviation_log else None
            ),
            "recovery_planned_stamp": (
                recovery_path["first_stamp"] if recovery_path else None
            ),
            "recovery_started_stamp": recovery_start_stamp,
            "recovery_state_stamp": (
                recovery_state["stamp"] if recovery_state else None
            ),
            "sweep_retried_stamp": (
                sweep_retry["stamp"] if sweep_retry else None
            ),
            "fault_to_recovery_sec": (
                recovery_start_stamp - stamp
                if recovery_start_stamp is not None else None
            ),
            "recovery_duration_sec": (
                sweep_retry["stamp"] - recovery_start_stamp
                if recovery_start_stamp is not None and sweep_retry else None
            ),
            "fault_to_sweep_retry_sec": (
                sweep_retry["stamp"] - stamp if sweep_retry else None
            ),
        }
        if path is not None:
            record["pre_pose_on_sweep"] = project_pose_to_path(path, pre)
            record["post_pose_on_sweep"] = project_pose_to_path(path, post)
        if recovery_path is not None:
            record["planned_recovery"] = path_geometry(
                recovery_path["poses"]
            )
        if actual_recovery_samples:
            record["actual_recovery"] = {
                **trajectory_geometry(actual_recovery_samples),
                **signed_motion_distances(actual_recovery_samples),
            }
        effects.append(record)
    return effects


def sweep_projection(path, samples):
    poses = path.get("poses", [])
    if len(poses) < 2 or not samples:
        return {}
    first = poses[0].pose.position
    last = poses[-1].pose.position
    dx = float(last.x) - float(first.x)
    dy = float(last.y) - float(first.y)
    length = math.hypot(dx, dy)
    if length <= 1.0e-9:
        return {}
    tx = dx / length
    ty = dy / length
    values = []
    for sample in samples:
        relative_x = sample[1] - float(first.x)
        relative_y = sample[2] - float(first.y)
        along = relative_x * tx + relative_y * ty
        cross = relative_x * ty - relative_y * tx
        heading = abs(wrap_angle(sample[3] - math.atan2(dy, dx)))
        values.append((sample[0], along, cross, heading, sample[4]))
    settled_values = [
        value for value in values if 1.0 <= value[1] <= length
    ]
    return {
        "entry_along_m": values[0][1],
        "entry_cross_track_m": values[0][2],
        "entry_heading_error_deg": math.degrees(values[0][3]),
        "maximum_abs_cross_track_m": max(abs(value[2]) for value in values),
        "maximum_heading_error_deg": math.degrees(max(value[3] for value in values)),
        "maximum_abs_cross_track_after_1m_m": (
            max(abs(value[2]) for value in settled_values)
            if settled_values else None
        ),
        "maximum_heading_error_after_1m_deg": (
            math.degrees(max(value[3] for value in settled_values))
            if settled_values else None
        ),
        "maximum_longitudinal_overshoot_m": max(
            value[1] - length for value in values
        ),
        "terminal_along_m": values[-1][1],
        "terminal_cross_track_m": values[-1][2],
        "terminal_heading_error_deg": math.degrees(values[-1][3]),
        "terminal_speed_mps": values[-1][4],
    }


def load_bag(path):
    statuses = []
    odometry = []
    hybrid_paths = {}
    hybrid_action_paths = {}
    sweep_paths = {}
    faults = []
    fault_keys = set()
    relevant_logs = []
    bag_start = None
    bag_end = None
    topics = [
        "/coverage/status",
        "/coverage/enforced_path",
        "/coverage/hybrid_transition_path",
        "/coverage_gz_sim/fault_event",
        "/odom",
        "/rosout",
    ]
    with rosbag.Bag(path, "r") as bag:
        for topic, message, bag_stamp in bag.read_messages(topics=topics):
            stamp = bag_stamp.to_sec()
            bag_start = stamp if bag_start is None else min(bag_start, stamp)
            bag_end = stamp if bag_end is None else max(bag_end, stamp)
            if topic == "/coverage/status":
                key = (
                    str(message.state), int(message.current_segment),
                    str(message.current_region_name),
                )
                if not statuses or statuses[-1]["key"] != key:
                    statuses.append({
                        "stamp": stamp,
                        "state": key[0],
                        "segment": key[1],
                        "region": key[2],
                        "detail": str(message.detail),
                        "key": key,
                    })
            elif topic == "/odom":
                odometry.append((
                    stamp,
                    float(message.pose.pose.position.x),
                    float(message.pose.pose.position.y),
                    quaternion_yaw(message.pose.pose.orientation),
                    float(message.twist.twist.linear.x),
                ))
            elif topic == "/coverage/hybrid_transition_path":
                if len(message.poses) >= 2:
                    signature = path_signature(message.poses)
                    if signature not in hybrid_paths:
                        hybrid_paths[signature] = {
                            "signature": signature,
                            "first_stamp": stamp,
                            "poses": list(message.poses),
                        }
            elif topic == "/coverage/enforced_path":
                if bool(message.active) and len(message.path.poses) >= 2:
                    signature = path_signature(message.path.poses)
                    if signature not in sweep_paths:
                        sweep_paths[signature] = {
                            "signature": signature,
                            "first_stamp": stamp,
                            "segment_index": int(message.segment_index),
                            "poses": list(message.path.poses),
                        }
                elif len(message.path.poses) >= 2:
                    signature = path_signature(message.path.poses)
                    if signature not in hybrid_action_paths:
                        hybrid_action_paths[signature] = {
                            "signature": signature,
                            "first_stamp": stamp,
                            "segment_index": int(message.segment_index),
                            "poses": list(message.path.poses),
                        }
            elif topic == "/coverage_gz_sim/fault_event":
                try:
                    event = json.loads(message.data)
                except (TypeError, ValueError):
                    event = {"kind": "unparsed", "data": str(message.data)}
                key = json.dumps(event, ensure_ascii=False, sort_keys=True)
                if key not in fault_keys:
                    fault_keys.add(key)
                    faults.append(event)
            elif topic == "/rosout":
                rendered = str(message.msg)
                if any(token in rendered for token in (
                    "coverage planned rolling Hybrid chunk",
                    "coverage planned direct Hybrid connector",
                    "coverage Hybrid A*",
                    "has not acquired its entrance",
                    "entry-alignment Hybrid path",
                    "entry left its acquisition basin",
                    "replanning disturbed sweep",
                    "coverage connector",
                    "coverage sweep",
                    "left its fixed-gear path",
                    "possible oscillation",
                    "invalidated its cached suffix",
                    "kept its cached suffix",
                    "remaining-transition replan",
                )):
                    relevant_logs.append({"stamp": stamp, "message": rendered})
    return {
        "bag_start": bag_start or 0.0,
        "bag_end": bag_end or 0.0,
        "statuses": statuses,
        "odometry": odometry,
        "hybrid_paths": list(hybrid_paths.values()),
        "hybrid_action_paths": list(hybrid_action_paths.values()),
        "sweep_paths": list(sweep_paths.values()),
        "faults": faults,
        "relevant_logs": relevant_logs,
    }


def build_report(bag_path, data):
    statuses = data["statuses"]
    intervals = []
    for index, status in enumerate(statuses):
        end = (
            statuses[index + 1]["stamp"]
            if index + 1 < len(statuses) else data["bag_end"]
        )
        intervals.append({
            "state": status["state"],
            "segment": status["segment"],
            "region": status["region"],
            "detail": status["detail"],
            "start_stamp": status["stamp"],
            "end_stamp": end,
            "duration_sec": max(0.0, end - status["stamp"]),
        })

    transitions = []
    sweeps = []
    for interval in intervals:
        samples = samples_between(
            data["odometry"], interval["start_stamp"], interval["end_stamp"]
        )
        if interval["state"] == "TRANSITING":
            planned = path_for_interval(
                data["hybrid_paths"],
                interval["start_stamp"], interval["end_stamp"],
            )
            transitions.append({
                **interval,
                "actual": trajectory_geometry(samples),
                "signed_motion": signed_motion_distances(samples),
                "motion_timing": motion_timing(samples),
                "planned_paths": [
                    {
                        "first_stamp": path["first_stamp"],
                        **path_geometry(path["poses"]),
                    }
                    for path in planned
                ],
            })
        elif interval["state"] == "SWEEPING":
            candidates = path_for_interval(
                data["sweep_paths"],
                interval["start_stamp"], interval["end_stamp"],
            )
            matching = [
                path for path in data["sweep_paths"]
                if path["segment_index"] + 1 == interval["segment"]
            ]
            path = min(
                matching,
                key=lambda item: abs(
                    item["first_stamp"] - interval["start_stamp"]
                ),
            ) if matching else (
                candidates[0] if candidates else None
            )
            faults = [
                event for event in data["faults"]
                if interval["start_stamp"] <= float(event.get("stamp", -1.0))
                <= interval["end_stamp"]
            ]
            sweep = {
                **interval,
                "actual": trajectory_geometry(samples),
                "faults": faults,
            }
            if path is not None:
                sweep["planned"] = path_geometry(path["poses"])
                sweep["tracking"] = sweep_projection(path, samples)
            sweeps.append(sweep)

    whole_actual = trajectory_geometry(data["odometry"])
    terminal = statuses[-1]["state"] if statuses else "UNKNOWN"
    return {
        "bag": os.path.abspath(bag_path),
        "bag_start_stamp": data["bag_start"],
        "bag_end_stamp": data["bag_end"],
        "bag_duration_sec": data["bag_end"] - data["bag_start"],
        "terminal_state": terminal,
        "whole_run_actual": whole_actual,
        "fault_events": data["faults"],
        "entry_fault_effects": entry_fault_effects(data),
        "state_intervals": intervals,
        "transitions": transitions,
        "sweeps": sweeps,
        "relevant_logs": data["relevant_logs"],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--output", required=True)
    arguments = parser.parse_args()
    report = build_report(arguments.bag, load_bag(arguments.bag))
    with open(arguments.output, "w", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps({
        "output": os.path.abspath(arguments.output),
        "terminal_state": report["terminal_state"],
        "transition_intervals": len(report["transitions"]),
        "sweep_intervals": len(report["sweeps"]),
        "fault_events": len(report["fault_events"]),
    }, ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
