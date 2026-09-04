#!/usr/bin/env python3
"""Enforce and summarize the line-to-line transition wall-time contract."""

import argparse
import json
import os


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--details", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--limit-sec", type=float, default=10.0)
    parser.add_argument("--minimum-count", type=int, default=1)
    parser.add_argument(
        "--timing-advisory", action="store_true",
        help="report the time threshold without failing an otherwise valid run",
    )
    args = parser.parse_args()

    with open(args.details, "r", encoding="utf-8") as stream:
        details = json.load(stream)

    # Segment 1 is the independent first-line entry of a coverage task.  It is
    # intentionally excluded: the 10 s contract applies only between two
    # consecutive cleaning lines in the same region/task.
    transition_intervals = [
        item for item in details.get("transitions", [])
        if int(item.get("segment", 0)) > 1
    ]
    grouped = {}
    for item in transition_intervals:
        key = (str(item.get("region", "")), int(item.get("segment", 0)))
        grouped.setdefault(key, []).append(item)
    measurements = []
    for (region, segment), items in sorted(grouped.items()):
        related_states = [
            state for state in details.get("state_intervals", [])
            if str(state.get("region", "")) == region
            and int(state.get("segment", 0)) == segment
        ]
        start_stamp = min(
            float(item.get("start_stamp", 0.0)) for item in items
        )
        end_stamp = max(
            [float(item.get("end_stamp", start_stamp)) for item in items]
            + [float(state.get("end_stamp", start_stamp))
               for state in related_states]
        )
        duration = max(0.0, end_stamp - start_stamp)
        paths = []
        path_keys = set()
        for item in items:
            for path in item.get("planned_paths", []):
                key = (
                    round(float(path.get("length_m", 0.0)), 3),
                    str(path.get("shape", "")),
                )
                if key not in path_keys:
                    path_keys.add(key)
                    paths.append(path)

        def sum_nested(section, field):
            return sum(
                float(item.get(section, {}).get(field, 0.0) or 0.0)
                for item in items
            )

        measurements.append({
            "region": region,
            "segment": segment,
            "duration_sec": duration,
            "within_budget": duration <= args.limit_sec + 1.0e-9,
            "actual_distance_m": sum_nested("actual", "distance_m"),
            "signed_motion": {
                field: sum_nested("signed_motion", field)
                for field in (
                    "forward_distance_m", "reverse_distance_m",
                    "stationary_pose_change_m",
                )
            },
            "motion_timing": {
                "forward_motion_sec": sum_nested(
                    "motion_timing", "forward_motion_sec"
                ),
                "reverse_motion_sec": sum_nested(
                    "motion_timing", "reverse_motion_sec"
                ),
                "stopped_sec": sum_nested("motion_timing", "stopped_sec"),
                "maximum_forward_speed_mps": max(
                    float(item.get("motion_timing", {}).get(
                        "maximum_forward_speed_mps", 0.0
                    ) or 0.0) for item in items
                ),
                "maximum_reverse_speed_mps": max(
                    float(item.get("motion_timing", {}).get(
                        "maximum_reverse_speed_mps", 0.0
                    ) or 0.0) for item in items
                ),
            },
            "planned": [
                {
                    "length_m": path.get("length_m"),
                    "shape": path.get("shape"),
                    "gear_changes": path.get("gear_changes"),
                    "direct_distance_m": path.get("direct_distance_m"),
                    "maximum_chord_deviation_m": path.get(
                        "maximum_chord_deviation_m"
                    ),
                }
                for path in paths
            ],
        })

    durations = [item["duration_sec"] for item in measurements]
    one_hz_searches = sum(
        "1 Hz online replan" in str(item.get("message", ""))
        for item in details.get("relevant_logs", [])
    )
    event_searches = sum(
        any(token in str(item.get("message", "")) for token in (
            "blocked/deviation replan",
            "invalidated its cached suffix",
            "remaining-transition replan",
        ))
        for item in details.get("relevant_logs", [])
    )
    direct_requests = sum(
        "coverage planned direct Hybrid connector" in str(
            item.get("message", "")
        )
        for item in details.get("relevant_logs", [])
    )
    passed = (
        details.get("terminal_state") == "COMPLETED"
        and len(measurements) >= args.minimum_count
        and (
            args.timing_advisory
            or all(item["within_budget"] for item in measurements)
        )
    )
    report = {
        "passed": passed,
        "terminal_state": details.get("terminal_state"),
        "contract": "same-region line-to-line TRANSITING interval <= {:.3f}s".format(
            args.limit_sec
        ),
        "timing_enforced": not args.timing_advisory,
        "first_line_entry_excluded": True,
        "transition_count": len(measurements),
        "maximum_duration_sec": max(durations) if durations else None,
        "mean_duration_sec": (
            sum(durations) / len(durations) if durations else None
        ),
        "planner_search_counts": {
            "direct_service_requests": direct_requests,
            "event_replans": event_searches,
            "unconditional_1hz_replans": one_hz_searches,
        },
        "transitions": measurements,
        "source": os.path.realpath(args.details),
    }
    rendered = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
    with open(args.output, "w", encoding="utf-8") as stream:
        stream.write(rendered)
        stream.write("\n")
    print(rendered)
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
