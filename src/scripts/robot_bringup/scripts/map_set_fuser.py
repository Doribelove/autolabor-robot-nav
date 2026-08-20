#!/usr/bin/env python3
"""Project a horizontal 3-D map slice into a dual-LD19 occupancy map."""

import argparse
import collections
import datetime
import math
import os
import struct
import sys

import yaml


UNKNOWN_PIXEL = 205
FREE_PIXEL = 254
OCCUPIED_PIXEL = 0


def read_pgm(path):
    with open(path, "rb") as stream:
        if stream.readline().strip() != b"P5":
            raise ValueError("only binary P5 PGM maps are supported")

        tokens = []
        while len(tokens) < 3:
            line = stream.readline()
            if not line:
                raise ValueError("truncated PGM header")
            line = line.split(b"#", 1)[0]
            tokens.extend(line.split())
        width, height, maximum = (int(token) for token in tokens[:3])
        if width <= 0 or height <= 0 or maximum != 255:
            raise ValueError("invalid PGM dimensions or maximum value")
        pixels = stream.read(width * height)
        if len(pixels) != width * height:
            raise ValueError("truncated PGM pixel data")
    return width, height, pixels


def read_binary_pcd_xyz(path):
    header = {}
    with open(path, "rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PCD DATA line is missing")
            decoded = line.decode("ascii").strip()
            if not decoded or decoded.startswith("#"):
                continue
            key, _, value = decoded.partition(" ")
            header[key.upper()] = value.strip()
            if key.upper() == "DATA":
                break
        if header.get("DATA", "").lower() != "binary":
            raise ValueError("only binary PCD maps are supported")
        fields = header.get("FIELDS", "").split()
        sizes = [int(item) for item in header.get("SIZE", "").split()]
        types = header.get("TYPE", "").split()
        counts = [int(item) for item in header.get("COUNT", "").split()]
        if not counts:
            counts = [1] * len(fields)
        if not (len(fields) == len(sizes) == len(types) == len(counts)):
            raise ValueError("inconsistent PCD field metadata")
        for coordinate in ("x", "y", "z"):
            if coordinate not in fields:
                raise ValueError("PCD lacks {} field".format(coordinate))
        offsets = {}
        point_step = 0
        for field, size, field_type, count in zip(fields, sizes, types, counts):
            if count != 1:
                raise ValueError("array PCD fields are not supported")
            offsets[field] = point_step
            if field in ("x", "y", "z") and (size != 4 or field_type != "F"):
                raise ValueError("PCD coordinates must be float32")
            point_step += size
        point_count = int(header.get("POINTS", header.get("WIDTH", "0")))
        payload = stream.read(point_count * point_step)
        if len(payload) != point_count * point_step:
            raise ValueError("truncated PCD payload")
    for index in range(point_count):
        base = index * point_step
        yield (
            struct.unpack_from("<f", payload, base + offsets["x"])[0],
            struct.unpack_from("<f", payload, base + offsets["y"])[0],
            struct.unpack_from("<f", payload, base + offsets["z"])[0],
        )


def load_map(map_yaml_path):
    with open(map_yaml_path, "r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    resolution = float(metadata["resolution"])
    origin = metadata["origin"]
    if resolution <= 0.0 or len(origin) < 2:
        raise ValueError("invalid map resolution/origin")
    image_path = metadata["image"]
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(map_yaml_path), image_path)
    width, height, pixels = read_pgm(image_path)
    cells = {}
    min_cell_x = int(round(float(origin[0]) / resolution))
    min_cell_y = int(round(float(origin[1]) / resolution))
    for row in range(height):
        cell_y = min_cell_y + height - 1 - row
        for column in range(width):
            pixel = pixels[row * width + column]
            if pixel != UNKNOWN_PIXEL:
                cells[(min_cell_x + column, cell_y)] = pixel
    return metadata, cells, resolution


def write_map(output_dir, name, cells, resolution, metadata):
    if not cells:
        raise RuntimeError("refusing to save an empty fused map")
    os.makedirs(output_dir, exist_ok=True)
    min_x = min(cell[0] for cell in cells)
    max_x = max(cell[0] for cell in cells)
    min_y = min(cell[1] for cell in cells)
    max_y = max(cell[1] for cell in cells)
    width = max_x - min_x + 1
    height = max_y - min_y + 1
    pixels = bytearray([UNKNOWN_PIXEL]) * (width * height)
    for (cell_x, cell_y), pixel in cells.items():
        column = cell_x - min_x
        row = height - 1 - (cell_y - min_y)
        pixels[row * width + column] = pixel

    pgm_path = os.path.join(output_dir, name + ".pgm")
    yaml_path = os.path.join(output_dir, name + ".yaml")
    pgm_temporary = pgm_path + ".tmp"
    with open(pgm_temporary, "wb") as stream:
        stream.write("P5\n{} {}\n255\n".format(width, height).encode("ascii"))
        stream.write(pixels)
    os.replace(pgm_temporary, pgm_path)

    map_yaml = {
        "image": os.path.basename(pgm_path),
        "resolution": resolution,
        "origin": [min_x * resolution, min_y * resolution, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    yaml_temporary = yaml_path + ".tmp"
    with open(yaml_temporary, "w", encoding="utf-8") as stream:
        yaml.safe_dump(map_yaml, stream, default_flow_style=False, sort_keys=False)
    os.replace(yaml_temporary, yaml_path)

    info_path = os.path.join(output_dir, "config.yaml")
    info = dict(metadata)
    info.update({
        "status": "complete",
        "generated_at": datetime.datetime.now().astimezone().isoformat(),
        "width_cells": width,
        "height_cells": height,
        "origin": map_yaml["origin"],
    })
    with open(info_path + ".tmp", "w", encoding="utf-8") as stream:
        yaml.safe_dump(info, stream, default_flow_style=False, sort_keys=False)
    os.replace(info_path + ".tmp", info_path)
    return yaml_path


def fuse(arguments):
    _, cells, resolution = load_map(arguments.map_2d)
    if arguments.resolution is not None and not math.isclose(
            resolution, arguments.resolution, rel_tol=0.0, abs_tol=1e-9):
        raise ValueError("2-D map resolution does not match requested resolution")
    hits = collections.Counter()
    slice_points = 0
    lower = arguments.slice_center_z - arguments.slice_half_width
    upper = arguments.slice_center_z + arguments.slice_half_width
    for x, y, z in read_binary_pcd_xyz(arguments.map_3d):
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        if lower <= z <= upper:
            hits[(int(math.floor(x / resolution)), int(math.floor(y / resolution)))] += 1
            slice_points += 1

    projected_cells = 0
    newly_occupied = 0
    for cell, count in hits.items():
        if count < arguments.min_points_per_cell:
            continue
        projected_cells += 1
        if cells.get(cell) != OCCUPIED_PIXEL:
            newly_occupied += 1
        cells[cell] = OCCUPIED_PIXEL
    if projected_cells == 0:
        raise RuntimeError("3-D height slice contains no accepted occupied cells")

    output = write_map(
        arguments.output_dir,
        arguments.map_name,
        cells,
        resolution,
        {
            "frame_id": "map",
            "fusion_policy": "occupied_union",
            "map_2d": os.path.abspath(arguments.map_2d),
            "map_3d": os.path.abspath(arguments.map_3d),
            "slice_center_z_m": arguments.slice_center_z,
            "slice_half_width_m": arguments.slice_half_width,
            "slice_bounds_z_m": [lower, upper],
            "min_points_per_cell": arguments.min_points_per_cell,
            "slice_points": slice_points,
            "projected_occupied_cells": projected_cells,
            "newly_occupied_cells": newly_occupied,
        },
    )
    print("FUSED_MAP_SAVED={}".format(output), flush=True)


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-2d", required=True)
    parser.add_argument("--map-3d", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--map-name", default="map")
    parser.add_argument("--slice-center-z", type=float, required=True)
    parser.add_argument("--slice-half-width", type=float, default=0.10)
    parser.add_argument("--min-points-per-cell", type=int, default=2)
    parser.add_argument("--resolution", type=float)
    arguments = parser.parse_args(argv)
    if arguments.slice_half_width <= 0.0:
        parser.error("--slice-half-width must be positive")
    if arguments.min_points_per_cell < 1:
        parser.error("--min-points-per-cell must be positive")
    return arguments


def main(argv=None):
    fuse(parse_arguments(argv))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError, ValueError, yaml.YAMLError) as error:
        print("ERROR={}".format(error), file=sys.stderr, flush=True)
        sys.exit(1)
