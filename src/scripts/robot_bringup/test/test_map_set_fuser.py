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
            write_pcd(pcd, [(9.0, 9.0, -0.4)])
            observations = root / "slice.yaml"
            observations.write_text(
                yaml.safe_dump({
                    "schema_version": 2,
                    "frame_id": "camera_init",
                    "resolution_m": 1.0,
                    "slice_center_z_m": -0.4,
                    "slice_half_width_m": 0.20,
                    "min_frame_observations": 20,
                    "observed_clouds": 100,
                    "candidate_cells": 2,
                    "accepted_cells_before_sweep": 2,
                    "accepted_cells": 1,
                    "moving_self_crop": {
                        "enabled": True,
                        "point_frame_id": "base_link",
                        "point_bounds_xy_m": [-0.75, 0.75, -0.50, 0.50],
                        "body_to_base_xyz_m": [-0.211, -0.02329, -0.95588],
                        "exact_time_sync": True,
                        "point_rejected_count": 3,
                        "sweep_frame_id": "base_link",
                        "sweep_bounds_xy_m": [-0.62, 0.62, -0.45, 0.45],
                        "sweep_linear_step_m": 0.05,
                        "sweep_angular_step_rad": 0.03490658503988659,
                        "sweep_pose_samples": 4,
                        "swept_cells": 1,
                        "swept_accepted_cells_filtered": 1,
                    },
                    "cells": [[2, 0, 25]],
                    "swept_cells": [[1, 1]],
                }),
                encoding="utf-8",
            )
            arguments = types.SimpleNamespace(
                map_2d=str(base_dir / "map.yaml"),
                map_3d=str(pcd),
                slice_observations=str(observations),
                output_dir=str(fused_dir),
                map_name="map",
                slice_center_z=-0.4,
                slice_half_width=0.20,
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
            self.assertTrue(config["moving_self_crop"]["enabled"])
            self.assertEqual(1, config["swept_accepted_cells_filtered"])

    def test_rejects_legacy_persistent_slice_without_moving_crop_evidence(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = pathlib.Path(temporary)
            observations = root / "legacy_slice.yaml"
            observations.write_text(
                yaml.safe_dump({
                    "schema_version": 1,
                    "frame_id": "camera_init",
                    "resolution_m": 1.0,
                    "slice_center_z_m": -0.4,
                    "slice_half_width_m": 0.20,
                    "min_frame_observations": 20,
                    "observed_clouds": 100,
                    "accepted_cells": 1,
                    "cells": [[2, 0, 25]],
                }),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "moving self-crop evidence"):
                MODULE.load_slice_observations(
                    str(observations), 1.0, -0.4, 0.20
                )

    def test_rejects_schema_without_coordinate_frame(self):
        with tempfile.TemporaryDirectory() as temporary:
            observations = pathlib.Path(temporary) / "missing_frame.yaml"
            observations.write_text(
                yaml.safe_dump({"schema_version": 2}), encoding="utf-8"
            )
            with self.assertRaisesRegex(ValueError, "frame_id"):
                MODULE.load_slice_observations(
                    str(observations), 1.0, -0.4, 0.20
                )

    def test_rejects_accepted_cell_inside_swept_footprint(self):
        with tempfile.TemporaryDirectory() as temporary:
            observations = pathlib.Path(temporary) / "overlap_slice.yaml"
            observations.write_text(
                yaml.safe_dump({
                    "schema_version": 2,
                    "frame_id": "camera_init",
                    "resolution_m": 1.0,
                    "slice_center_z_m": -0.4,
                    "slice_half_width_m": 0.20,
                    "min_frame_observations": 20,
                    "observed_clouds": 100,
                    "accepted_cells_before_sweep": 1,
                    "accepted_cells": 1,
                    "moving_self_crop": {
                        "enabled": True,
                        "point_frame_id": "base_link",
                        "point_bounds_xy_m": [-0.75, 0.75, -0.50, 0.50],
                        "body_to_base_xyz_m": [-0.211, -0.02329, -0.95588],
                        "exact_time_sync": True,
                        "sweep_frame_id": "base_link",
                        "sweep_bounds_xy_m": [-0.62, 0.62, -0.45, 0.45],
                        "sweep_linear_step_m": 0.05,
                        "sweep_angular_step_rad": 0.03490658503988659,
                        "sweep_pose_samples": 4,
                        "swept_cells": 1,
                        "swept_accepted_cells_filtered": 0,
                    },
                    "cells": [[2, 0, 25]],
                    "swept_cells": [[2, 0]],
                }),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "inside the swept footprint"):
                MODULE.load_slice_observations(
                    str(observations), 1.0, -0.4, 0.20
                )


if __name__ == "__main__":
    unittest.main()
