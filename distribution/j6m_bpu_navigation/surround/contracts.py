"""Validated frame, calibration and source-time contracts. No device imports."""
from dataclasses import dataclass, field
import hashlib
import json
from pathlib import Path

import numpy as np

from . import CAMERAS


def read_json(path):
    return json.loads(Path(path).read_text(), parse_constant=lambda x: (_ for _ in ()).throw(
        ValueError("Non-finite JSON: " + x)))


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for block in iter(lambda: f.read(4 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def atomic_json(path, obj):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(obj, ensure_ascii=False, indent=2, allow_nan=False))
    temporary.replace(path)


def matrix(value, shape, name):
    a = np.asarray(value, dtype=float)
    if a.shape != shape or not np.isfinite(a).all():
        raise ValueError(name + ": wrong shape or non-finite values")
    return a


def transform(value, name="transform"):
    a = matrix(value, (4, 4), name)
    r = a[:3, :3]
    if (not np.allclose(a[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(r), 1, atol=1e-5)):
        raise ValueError(name + ": expected a rigid right-handed transform")
    return a


@dataclass
class Frame:
    camera: str
    session: str
    sequence: int
    raw_stamp: int
    stamp_ns: int
    received_ns: int
    clock_id: str
    clock_mapping_id: str
    calibration_id: str
    image: np.ndarray = field(repr=False)
    rectified: bool = False
    rectification_id: str = ""
    synthetic: bool = False

    @property
    def key(self):
        return "%s/%s/%d" % (self.session, self.camera, self.sequence)

    def validate(self, now_ns, max_age_ns=500_000_000):
        if self.camera not in CAMERAS or not self.session or not self.clock_mapping_id:
            raise ValueError("Missing camera/session/clock identity")
        if any(type(x) is not int or x < 0 for x in
               (self.sequence, self.raw_stamp, self.stamp_ns, self.received_ns)):
            raise ValueError("Invalid integer timestamp/sequence")
        if self.stamp_ns <= 0 or not 0 <= now_ns - self.stamp_ns <= max_age_ns:
            raise ValueError("Source exposure is stale or in the future")
        if (not isinstance(self.image, np.ndarray) or self.image.dtype != np.uint8
                or self.image.ndim != 3 or self.image.shape[2] != 3
                or not 1 <= min(self.image.shape[:2]) <= max(self.image.shape[:2]) <= 4096):
            raise ValueError("Expected a bounded BGR uint8 perspective image")
        return self

    def identity(self):
        return {k: getattr(self, k) for k in (
            "camera", "session", "sequence", "raw_stamp", "stamp_ns", "received_ns",
            "clock_id", "clock_mapping_id", "calibration_id", "rectified",
            "rectification_id", "synthetic")}


class ClockMapping:
    """Explicit measured raw-counter -> exposure midpoint -> ROS epoch mapping.

    No arrival-time substitution. The validity interval is checked on each frame;
    live producers can reload a periodically renewed mapping file.
    """
    def __init__(self, data):
        self.data = data
        if (data.get("verified") is not True or not data.get("id")
                or not data.get("clock_id") or not data.get("evidence")
                or data.get("source_semantics") not in ("frame_start", "exposure_midpoint")):
            raise ValueError("Clock/exposure mapping has not been measured")
        for name in ("raw_origin", "epoch_origin_ns", "scale_num", "scale_den",
                     "exposure_offset_ns", "valid_from_ns", "valid_until_ns", "uncertainty_ns"):
            if type(data.get(name)) is not int:
                raise ValueError("Clock field must be integer: " + name)
        if (data["scale_num"] <= 0 or data["scale_den"] <= 0
                or not 0 <= data["uncertainty_ns"] <= 5_000_000
                or data["valid_until_ns"] <= data["valid_from_ns"]):
            raise ValueError("Invalid clock scale, uncertainty or validity")

    def convert(self, raw, now_ns):
        c = self.data
        if type(raw) is not int or raw <= 0:
            raise ValueError("Hardware timestamp missing")
        if not c["valid_from_ns"] <= now_ns <= c["valid_until_ns"]:
            raise ValueError("Clock mapping expired")
        stamp = (c["epoch_origin_ns"] + (raw - c["raw_origin"]) * c["scale_num"]
                 // c["scale_den"] + c["exposure_offset_ns"])
        if not 0 <= now_ns - stamp <= 500_000_000:
            raise ValueError("Mapped exposure outside freshness limit")
        return stamp


class Calibration:
    def __init__(self, data):
        self.data = data
        if data.get("schema") != "j6m.surround.calibration.v1":
            raise ValueError("Unknown calibration schema")
        if set(data.get("cameras", {})) != set(CAMERAS):
            raise ValueError("Exactly front/right/rear/left calibration entries required")
        self.id = data.get("id", "")
        self.synthetic = data.get("synthetic") is True
        self.cameras = data["cameras"]
        for name, c in self.cameras.items():
            if c.get("intrinsics_valid"):
                self.intrinsics(name)
            if c.get("extrinsics_valid"):
                transform(c["T_base_camera_optical"], name)
        if data.get("ground_valid"):
            plane = matrix(data.get("ground_plane_base"), (4,), "ground plane")
            if not np.isclose(np.linalg.norm(plane[:3]), 1, atol=1e-5):
                raise ValueError("Ground plane normal must have unit length")

    def intrinsics(self, name):
        c = self.cameras[name]
        if not c.get("intrinsics_valid"):
            raise ValueError("Intrinsics unavailable: " + name)
        for key in ("raw_size", "rectified_size"):
            size = c[key]
            if len(size) != 2 or any(type(x) is not int or not 2 <= x <= 4096 for x in size):
                raise ValueError("Invalid image size")
        k = matrix(c["K_raw"], (3, 3), "K_raw")
        kr = matrix(c["K_rectified"], (3, 3), "K_rectified")
        for a in (k, kr):
            if a[0, 0] <= 0 or a[1, 1] <= 0 or not np.allclose(a[2], [0, 0, 1]):
                raise ValueError("Invalid camera matrix")
        if c["lens_model"] not in ("fisheye", "pinhole"):
            raise ValueError("Unsupported lens model")
        d = np.asarray(c["D"], dtype=float)
        if d.ndim != 1 or len(d) not in ((4,) if c["lens_model"] == "fisheye" else (4, 5, 8)):
            raise ValueError("Distortion coefficient count mismatch")
        if not np.isfinite(d).all() or not c.get("rectification_id"):
            raise ValueError("Invalid distortion/rectification identity")
        return k, d, kr

    def ground_camera(self, name):
        self.intrinsics(name)
        c = self.cameras[name]
        if (not self.id or not c.get("extrinsics_valid") or not self.data.get("ground_valid")
                or not c.get("valid_ground_polygon_px") or not c.get("calibration_evidence")):
            raise ValueError("Ground calibration incomplete: " + name)
        polygon = matrix(c["valid_ground_polygon_px"],
                         (len(c["valid_ground_polygon_px"]), 2), "valid polygon")
        if len(polygon) < 3:
            raise ValueError("At least three ground polygon vertices required")
        for key in ("pixel_sigma", "extrinsic_angle_sigma_rad", "position_sigma_m"):
            if not np.isfinite(c.get(key, float("nan"))) or c[key] <= 0:
                raise ValueError("Measured positive uncertainty required: " + key)
        return c

    def issues(self):
        out = []
        for name in CAMERAS:
            try:
                self.ground_camera(name)
            except (ValueError, KeyError, TypeError) as e:
                out.append(str(e))
        if self.synthetic:
            out.append("Synthetic calibration: offline validation only")
        return out
