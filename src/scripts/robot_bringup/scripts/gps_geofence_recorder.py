#!/usr/bin/env python3
"""Record live GPS boundary samples and persist conservative convex keepouts."""

import argparse
import math
import os
from pathlib import Path
import re
import tempfile
import time

import yaml


DEFAULT_CONFIG = (
    Path(os.path.realpath(__file__)).parents[1] / "config" / "gps_geofences.yaml"
)
NAME_PATTERN = re.compile(r"^[A-Za-z0-9_.-]+$")


def empty_document():
    return {"hard_margin_m": 1.0, "regions": []}


def load_document(path):
    path = Path(path)
    if not path.exists():
        return empty_document()
    with path.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if document is None:
        document = empty_document()
    if not isinstance(document, dict):
        raise ValueError("geofence file root must be a YAML mapping")
    regions = document.setdefault("regions", [])
    if not isinstance(regions, list):
        raise ValueError("regions must be a YAML list")
    document.setdefault("hard_margin_m", 1.0)
    return document


def save_document(path, document):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=path.name + ".", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            yaml.safe_dump(
                document,
                stream,
                allow_unicode=True,
                default_flow_style=False,
                sort_keys=False,
            )
        os.replace(temporary_name, path)
    except Exception:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def validate_name(name):
    name = str(name).strip()
    if not NAME_PATTERN.fullmatch(name):
        raise ValueError("region name may contain only letters, numbers, '.', '_' and '-'")
    return name


def find_region(document, name):
    name = validate_name(name)
    for region in document["regions"]:
        if region.get("name") == name:
            return region
    return None


def begin_region(document, name, replace=False):
    name = validate_name(name)
    existing = find_region(document, name)
    if existing is not None:
        if not replace:
            raise ValueError("region {!r} already exists; use --replace to overwrite it".format(name))
        document["regions"].remove(existing)
    region = {"name": name, "enabled": False, "samples": [], "vertices": []}
    document["regions"].append(region)
    return region


def append_sample(document, name, latitude, longitude):
    region = find_region(document, name)
    if region is None:
        raise ValueError("region {!r} does not exist; run begin first".format(name))
    if region.get("enabled"):
        raise ValueError("region {!r} is closed; begin it again with --replace to recollect".format(name))
    latitude = float(latitude)
    longitude = float(longitude)
    if not math.isfinite(latitude) or not -90.0 <= latitude <= 90.0:
        raise ValueError("latitude must be finite and within [-90, 90]")
    if not math.isfinite(longitude) or not -180.0 <= longitude <= 180.0:
        raise ValueError("longitude must be finite and within [-180, 180]")
    region.setdefault("samples", []).append(
        {"latitude": latitude, "longitude": longitude}
    )
    return len(region["samples"])


def _cross(origin, first, second):
    return ((first[0] - origin[0]) * (second[1] - origin[1]) -
            (first[1] - origin[1]) * (second[0] - origin[0]))


def _is_non_left_turn(origin, first, second):
    first_x = first[0] - origin[0]
    first_y = first[1] - origin[1]
    second_x = second[0] - origin[0]
    second_y = second[1] - origin[1]
    cross = first_x * second_y - first_y * second_x
    scale = abs(first_x * second_y) + abs(first_y * second_x)
    return cross <= max(1e-24, 1e-9 * scale)


def convex_hull(samples):
    """Return the maximum outer polygon in counter-clockwise GPS order."""
    points = sorted(
        set((float(sample["longitude"]), float(sample["latitude"])) for sample in samples)
    )
    if len(points) < 3:
        raise ValueError("at least three distinct GPS points are required")

    lower = []
    for point in points:
        while len(lower) >= 2 and _is_non_left_turn(lower[-2], lower[-1], point):
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(points):
        while len(upper) >= 2 and _is_non_left_turn(upper[-2], upper[-1], point):
            upper.pop()
        upper.append(point)
    hull = lower[:-1] + upper[:-1]
    if len(hull) < 3:
        raise ValueError("GPS points are collinear and cannot form a polygon")
    return [
        {"latitude": latitude, "longitude": longitude}
        for longitude, latitude in hull
    ]


def close_region(document, name):
    region = find_region(document, name)
    if region is None:
        raise ValueError("region {!r} does not exist".format(name))
    vertices = convex_hull(region.get("samples", []))
    region["vertices"] = vertices
    region["enabled"] = True
    return vertices


def set_region_enabled(document, name, enabled):
    region = find_region(document, name)
    if region is None:
        raise ValueError("region {!r} does not exist".format(name))
    if enabled and len(region.get("vertices", [])) < 3:
        raise ValueError("region {!r} has not been closed".format(name))
    region["enabled"] = bool(enabled)


def remove_region(document, name):
    region = find_region(document, name)
    if region is None:
        raise ValueError("region {!r} does not exist".format(name))
    document["regions"].remove(region)


def read_live_fix(topic, timeout):
    import rospy
    from sensor_msgs.msg import NavSatFix, NavSatStatus

    if not rospy.core.is_initialized():
        rospy.init_node("gps_geofence_recorder", anonymous=True)
    message = rospy.wait_for_message(topic, NavSatFix, timeout=timeout)
    if message.status.status < NavSatStatus.STATUS_FIX:
        raise RuntimeError("received /gps/fix without a valid GNSS fix")
    return float(message.latitude), float(message.longitude)


def apply_document_live(document, topic="/gps/geofence/reload", timeout=1.5):
    """Replace both running costmap layer parameters and request a reload."""
    import rospy
    from std_msgs.msg import Empty

    if not rospy.core.is_initialized():
        rospy.init_node("gps_geofence_recorder", anonymous=True, disable_signals=True)
    parameters = {
        "hard_margin_m": float(document.get("hard_margin_m", 1.0)),
        "regions": document.get("regions", []),
    }
    for namespace in (
        "/move_base/global_costmap/gps_geofence_layer",
        "/move_base/local_costmap/gps_geofence_layer",
    ):
        for key, value in parameters.items():
            rospy.set_param(namespace + "/" + key, value)

    publisher = rospy.Publisher(topic, Empty, queue_size=1)
    deadline = time.monotonic() + max(0.0, float(timeout))
    # Both layer objects live inside one move_base process, so rospy normally
    # reports one transport connection even though both callbacks receive it.
    while publisher.get_num_connections() < 1 and time.monotonic() < deadline:
        rospy.sleep(0.05)
    connections = publisher.get_num_connections()
    # A few short-spaced publications avoid losing the one-shot request while a
    # costmap is reconnecting. Reload is idempotent in both layer instances.
    for _ in range(3):
        publisher.publish(Empty())
        rospy.sleep(0.05)
    return connections


def print_regions(document):
    regions = document.get("regions", [])
    print("hard_margin_m={:.3f}".format(float(document.get("hard_margin_m", 1.0))))
    if not regions:
        print("regions: none")
        return
    for region in regions:
        print(
            "{}: enabled={} samples={} vertices={}".format(
                region.get("name", "<unnamed>"),
                bool(region.get("enabled", False)),
                len(region.get("samples", [])),
                len(region.get("vertices", [])),
            )
        )


def make_parser():
    parser = argparse.ArgumentParser(
        description="Record multiple GPS forbidden regions as conservative convex polygons."
    )
    parser.add_argument("--file", default=str(DEFAULT_CONFIG), help="geofence YAML file")
    parser.add_argument(
        "--apply-live",
        action="store_true",
        help="reload both running move_base costmap layers after a file change",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    begin = subparsers.add_parser("begin", help="start collecting a named region")
    begin.add_argument("name")
    begin.add_argument("--replace", action="store_true")

    add = subparsers.add_parser("add", help="append the current /gps/fix")
    add.add_argument("name")
    add.add_argument("--topic", default="/gps/fix")
    add.add_argument("--timeout", type=float, default=10.0)
    add.add_argument("--latitude", type=float)
    add.add_argument("--longitude", type=float)

    close = subparsers.add_parser("close", help="build and enable the maximum outer polygon")
    close.add_argument("name")
    for command in ("enable", "disable", "remove"):
        child = subparsers.add_parser(command)
        child.add_argument("name")
    subparsers.add_parser("list")
    return parser


def main():
    arguments = make_parser().parse_args()
    document = load_document(arguments.file)
    if arguments.command == "begin":
        begin_region(document, arguments.name, replace=arguments.replace)
    elif arguments.command == "add":
        explicit_latitude = arguments.latitude is not None
        explicit_longitude = arguments.longitude is not None
        if explicit_latitude != explicit_longitude:
            raise ValueError("--latitude and --longitude must be provided together")
        if explicit_latitude:
            latitude, longitude = arguments.latitude, arguments.longitude
        else:
            latitude, longitude = read_live_fix(arguments.topic, arguments.timeout)
        count = append_sample(document, arguments.name, latitude, longitude)
        print("recorded point {}: latitude={:.12f} longitude={:.12f}".format(
            count, latitude, longitude
        ))
    elif arguments.command == "close":
        vertices = close_region(document, arguments.name)
        print("closed region {!r} with {} convex-hull vertices".format(
            arguments.name, len(vertices)
        ))
    elif arguments.command == "enable":
        set_region_enabled(document, arguments.name, True)
    elif arguments.command == "disable":
        set_region_enabled(document, arguments.name, False)
    elif arguments.command == "remove":
        remove_region(document, arguments.name)
    elif arguments.command == "list":
        print_regions(document)
        return
    save_document(arguments.file, document)
    print("saved {}".format(arguments.file))
    if arguments.apply_live:
        connections = apply_document_live(document)
        if connections >= 1:
            print("live reload requested for global and local costmaps")
        else:
            print(
                "warning: file saved, but only {} geofence layer subscriber(s) were online; "
                "restart GPS navigation if the running costmaps did not update".format(connections)
            )


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        raise SystemExit("gps geofence error: {}".format(error))
