"""Current map geometry and MID360-origin planar positions; no remembered-pose fallback."""
from dataclasses import dataclass
import hashlib
import json
import math

import numpy as np

from autolabor_bpu_perception.core import zed_mount


def rigid_matrix(matrix):
    value = np.asarray(matrix, dtype=np.float64)
    if (value.shape != (4, 4) or not np.isfinite(value).all()
            or not np.allclose(value[3], [0, 0, 0, 1], atol=1e-6)
            or not np.allclose(value[:3, :3].T @ value[:3, :3], np.eye(3), atol=1e-5)
            or not math.isclose(float(np.linalg.det(value[:3, :3])), 1.0, abs_tol=1e-5)):
        raise ValueError("INVALID_TRANSFORM")
    return value


def rectified_intrinsics(projection):
    """P belongs to the rectified LEFT image, with no stereo translation."""
    p = np.asarray(projection, dtype=float)
    if (p.shape != (12,) or not np.isfinite(p).all() or p[0] <= 0 or p[5] <= 0
            or not np.allclose(p[[1, 3, 4, 7, 8, 9, 11]], 0, atol=1e-7)
            or not math.isclose(p[10], 1.0, abs_tol=1e-7)):
        raise ValueError("INVALID_LEFT_RECTIFIED_INTRINSICS")
    return [p[0], 0, p[2], 0, p[5], p[6], 0, 0, 1]


@dataclass(frozen=True)
class MapTransform:
    matrix: np.ndarray
    target_frame: str = "map"


class MapLocalizer:
    def __init__(self, mount_config):
        if mount_config.get("reference") != "left_camera":
            raise ValueError("map localization requires measured left-camera extrinsics")
        mount = zed_mount(mount_config)
        self.mid360_in_base = np.asarray(
            mount_config["mid360_in_base_m"], dtype=float
        ).copy()
        self.expected = np.eye(4)
        self.expected[:3, :3] = mount["optical_to_base"]
        self.expected[:3, 3] = mount["left_position"]
        self.calibration_sha256 = hashlib.sha256(json.dumps(
            mount_config, sort_keys=True, separators=(",", ":"), allow_nan=False
        ).encode()).hexdigest()

    def base_transform(self, source_frame, lookup):
        """lookup(target, source) must use the exact RGB acquisition stamp."""
        if not source_frame.endswith("_left_camera_optical_frame"):
            raise ValueError("UNSUPPORTED_CAMERA_FRAME")
        base_from_camera = rigid_matrix(lookup("base_link", source_frame))
        displacement = np.linalg.norm(
            base_from_camera[:3, 3] - self.expected[:3, 3]
        )
        relative_rotation = base_from_camera[:3, :3].T @ self.expected[:3, :3]
        angle = math.acos(float(np.clip(
            (np.trace(relative_rotation) - 1) / 2, -1, 1
        )))
        # The tolerance detects wrong/default/old installation TF; it is not
        # a claim of metrology accuracy for the user's approximate calibration.
        if displacement > 0.01 or angle > math.radians(0.5):
            raise ValueError("EXTRINSIC_MISMATCH")
        return MapTransform(base_from_camera, "base_link")

    def map_transform(self, base_transform, lookup):
        if base_transform.target_frame != "base_link":
            raise ValueError("INVALID_BASE_FRAME")
        map_from_base = rigid_matrix(lookup("map", "base_link"))
        return MapTransform(
            map_from_base @ rigid_matrix(base_transform.matrix)
        )

    def radar_transform(self, base_transform):
        """MID360 origin, vehicle axes: X forward, Y left, Z up."""
        if base_transform.target_frame != "base_link":
            raise ValueError("INVALID_BASE_FRAME")
        matrix = rigid_matrix(base_transform.matrix).copy()
        matrix[:3, 3] -= self.mid360_in_base
        return MapTransform(matrix, "mid360")

    def transform(self, source_frame, lookup):
        return self.map_transform(
            self.base_transform(source_frame, lookup), lookup
        )


def localization_ready(status, received_monotonic, now_monotonic, maximum_age=0.5):
    fields = dict(part.split("=", 1) for part in status.split(";") if "=" in part)
    age = now_monotonic - received_monotonic
    return fields.get("state") == "LOCALIZED" and 0 <= age <= maximum_age


def current_point(estimate, transform, target_frame, dimensions=3):
    if not estimate.valid or transform is None or transform.target_frame != target_frame:
        return None
    point = np.asarray(estimate.camera_point, dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all() or point[2] <= 0:
        return None
    result = rigid_matrix(transform.matrix)[:dimensions] @ np.r_[point, 1.0]
    return result if np.isfinite(result).all() else None


def current_map_point(estimate, transform):
    return current_point(estimate, transform, "map")


def current_local_point(estimate, transform):
    return current_point(estimate, transform, "mid360", dimensions=2)
