#!/usr/bin/env python3
"""Measure TEB local-plan deviation from its latest global plan by state."""

import argparse
import bisect
import json
import math
import os
import statistics

import rosbag


STATUS_TOPIC = "/coverage/status"
GLOBAL_TOPIC = "/move_base/TebLocalPlannerROS/global_plan"
LOCAL_TOPIC = "/move_base/TebLocalPlannerROS/local_plan"


def point_to_segment(px, py, ax, ay, bx, by):
    dx = bx - ax
    dy = by - ay
    squared = dx * dx + dy * dy
    if squared <= 1.0e-12:
        return math.hypot(px - ax, py - ay)
    fraction = max(0.0, min(
        1.0, ((px - ax) * dx + (py - ay) * dy) / squared
    ))
    return math.hypot(
        px - (ax + fraction * dx), py - (ay + fraction * dy)
    )


def local_to_global_distances(local_path, global_path):
    global_points = [
        (pose.pose.position.x, pose.pose.position.y)
        for pose in global_path.poses
    ]
    edges = list(zip(global_points, global_points[1:]))
    return [
        min(point_to_segment(
            pose.pose.position.x,
            pose.pose.position.y,
            first[0], first[1], second[0], second[1],
        ) for first, second in edges)
        for pose in local_path.poses
    ]


def percentile(values, ratio):
    ordered = sorted(values)
    if not ordered:
        return None
    index = int(math.ceil(ratio * len(ordered))) - 1
    return ordered[max(0, min(index, len(ordered) - 1))]


def summarize(rows):
    if not rows:
        return {
            "paired_local_plans": 0,
            "maximum_local_point_deviation_m": None,
            "p95_local_plan_maximum_deviation_m": None,
            "mean_local_point_deviation_m": None,
        }
    maxima = [row["maximum_deviation_m"] for row in rows]
    means = [row["mean_deviation_m"] for row in rows]
    return {
        "paired_local_plans": len(rows),
        "maximum_local_point_deviation_m": max(maxima),
        "p95_local_plan_maximum_deviation_m": percentile(maxima, 0.95),
        "mean_local_point_deviation_m": statistics.mean(means),
    }


def analyze(path, maximum_pair_age):
    statuses = []
    global_paths = []
    local_paths = []
    with rosbag.Bag(path) as bag:
        for topic, message, stamp in bag.read_messages(topics=[
                STATUS_TOPIC, GLOBAL_TOPIC, LOCAL_TOPIC]):
            seconds = stamp.to_sec()
            if topic == STATUS_TOPIC:
                statuses.append((
                    seconds,
                    str(message.state),
                    int(message.current_segment),
                    str(message.current_region_name),
                ))
            elif topic == GLOBAL_TOPIC and len(message.poses) >= 2:
                global_paths.append((seconds, message))
            elif topic == LOCAL_TOPIC and len(message.poses) >= 2:
                local_paths.append((seconds, message))

    status_stamps = [record[0] for record in statuses]
    global_stamps = [record[0] for record in global_paths]
    rows = []
    for stamp, local_path in local_paths:
        global_index = bisect.bisect_right(global_stamps, stamp) - 1
        status_index = bisect.bisect_right(status_stamps, stamp) - 1
        if global_index < 0 or status_index < 0:
            continue
        global_stamp, global_path = global_paths[global_index]
        if stamp - global_stamp > maximum_pair_age:
            continue
        distances = local_to_global_distances(local_path, global_path)
        _, state, segment, region = statuses[status_index]
        rows.append({
            "stamp": stamp,
            "state": state,
            "segment": segment,
            "region": region,
            "global_plan_age_sec": stamp - global_stamp,
            "maximum_deviation_m": max(distances),
            "mean_deviation_m": statistics.mean(distances),
        })

    states = {}
    segments = {}
    for state in sorted(set(row["state"] for row in rows)):
        states[state] = summarize([row for row in rows if row["state"] == state])
    for row in rows:
        key = "{}:{}:{}".format(row["region"], row["state"], row["segment"])
        segments.setdefault(key, []).append(row)
    return {
        "bag": os.path.abspath(path),
        "metric": "directed distance from every TEB local-plan pose to the latest TEB global-plan polyline",
        "maximum_global_plan_age_sec": maximum_pair_age,
        "global_plan_messages": len(global_paths),
        "local_plan_messages": len(local_paths),
        "paired_local_plans": len(rows),
        "by_state": states,
        "by_segment": {
            key: summarize(value) for key, value in sorted(segments.items())
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--maximum-pair-age", type=float, default=2.0)
    arguments = parser.parse_args()
    report = analyze(arguments.bag, arguments.maximum_pair_age)
    with open(arguments.output, "w", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps({
        "output": os.path.abspath(arguments.output),
        "paired_local_plans": report["paired_local_plans"],
        "by_state": report["by_state"],
    }, ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
