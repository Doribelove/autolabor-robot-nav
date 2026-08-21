#!/usr/bin/env python3

import importlib.util
import pathlib
import struct
import tempfile
import types
import unittest

import yaml


SCRIPT = pathlib.Path(__file__).parents[1] / "scripts" / "map_set_fuser.py"
SPEC = importlib.util.spec_from_file_location("map_set_fuser", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def write_pcd(path, points):
    with open(path, "wb") as stream:
        stream.write(
            (
                "VERSION 0.7\n"
                "FIELDS x y z intensity\n"
                "SIZE 4 4 4 4\n"
                "TYPE F F F F\n"
                "COUNT 1 1 1 1\n"
                "WIDTH {}\nHEIGHT 1\nPOINTS {}\nDATA binary\n"
            ).format(len(points), len(points)).encode("ascii")
        )
        for x, y, z in points:
            stream.write(struct.pack("<ffff", x, y, z, 1.0))


class MapSetFuserTest(unittest.TestCase):
    def test_height_slice_adds_only_accepted_occupied_cells(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = pathlib.Path(temporary)
            base_dir = root / "base"
            fused_dir = root / "fused"
            MODULE.write_map(
                str(base_dir),
                "map",
                {(0, 0): MODULE.FREE_PIXEL, (1, 0): MODULE.OCCUPIED_PIXEL},
                1.0,
                {"source": "dual_ld19_only"},
            )
            pcd = root / "map.pcd"
            write_pcd(
                pcd,
                [
                    (2.2, 0.2, -0.42),
                    (2.3, 0.3, -0.41),
                    (3.2, 0.2, 1.0),
                    (3.3, 0.3, 1.0),
                ],
            )
            arguments = types.SimpleNamespace(
                map_2d=str(base_dir / "map.yaml"),
                map_3d=str(pcd),
                output_dir=str(fused_dir),
                map_name="map",
                slice_center_z=-0.42,
                slice_half_width=0.10,
                min_points_per_cell=2,
                resolution=1.0,
            )
            MODULE.fuse(arguments)
            _, cells, resolution = MODULE.load_map(str(fused_dir / "map.yaml"))
            self.assertEqual(1.0, resolution)
            self.assertEqual(MODULE.FREE_PIXEL, cells[(0, 0)])
            self.assertEqual(MODULE.OCCUPIED_PIXEL, cells[(1, 0)])
            self.assertEqual(MODULE.OCCUPIED_PIXEL, cells[(2, 0)])
            self.assertNotIn((3, 0), cells)
            config = yaml.safe_load((fused_dir / "config.yaml").read_text())
            self.assertEqual("occupied_union", config["fusion_policy"])
            self.assertEqual(1, config["newly_occupied_cells"])

    def test_rejects_empty_slice(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = pathlib.Path(temporary)
            base_dir = root / "base"
            MODULE.write_map(
                str(base_dir), "map", {(0, 0): MODULE.FREE_PIXEL}, 1.0, {}
            )
            pcd = root / "map.pcd"
            write_pcd(pcd, [(0.0, 0.0, 2.0)])
            arguments = types.SimpleNamespace(
                map_2d=str(base_dir / "map.yaml"),
                map_3d=str(pcd),
                output_dir=str(root / "fused"),
                map_name="map",
                slice_center_z=0.0,
                slice_half_width=0.1,
                min_points_per_cell=1,
                resolution=1.0,
            )
            with self.assertRaises(RuntimeError):
                MODULE.fuse(arguments)

    def test_persistent_slice_uses_distinct_frame_cells(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = pathlib.Path(temporary)
            base_dir = root / "base"
            fused_dir = root / "fused"
            MODULE.write_map(
                str(base_dir),
                "map",
                {(0, 0): MODULE.FREE_PIXEL, (1, 0): MODULE.OCCUPIED_PIXEL},
                1.0,
                {"source": "dual_ld19_only"},
            )
            pcd = root / "map.pcd"
            write_pcd(pcd, [(9.0, 9.0, -0.756)])
            observations = root / "slice.yaml"
            observations.write_text(
                yaml.safe_dump({
                    "schema_version": 1,
                    "frame_id": "camera_init",
                    "resolution_m": 1.0,
                    "slice_center_z_m": -0.756,
                    "slice_half_width_m": 0.10,
                    "min_frame_observations": 20,
                    "observed_clouds": 100,
                    "candidate_cells": 2,
                    "accepted_cells": 1,
                    "cells": [[2, 0, 25]],
                }),
                encoding="utf-8",
            )
            arguments = types.SimpleNamespace(
                map_2d=str(base_dir / "map.yaml"),
                map_3d=str(pcd),
                slice_observations=str(observations),
                output_dir=str(fused_dir),
                map_name="map",
                slice_center_z=-0.756,
                slice_half_width=0.10,
                min_points_per_cell=2,
                resolution=1.0,
            )
            MODULE.fuse(arguments)
            _, cells, _ = MODULE.load_map(str(fused_dir / "map.yaml"))
            self.assertEqual(MODULE.OCCUPIED_PIXEL, cells[(2, 0)])
            self.assertNotIn((9, 9), cells)
            config = yaml.safe_load((fused_dir / "config.yaml").read_text())
            self.assertEqual("persistent_occupied_union", config["fusion_policy"])
            self.assertEqual(20, config["min_frame_observations"])


if __name__ == "__main__":
    unittest.main()
