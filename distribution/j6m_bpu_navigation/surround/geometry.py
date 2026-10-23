"""Perspective rectification, ray/ground intersection and source-time SE(3)."""
from collections import deque
from dataclasses import dataclass
import bisect
import math
import threading

import cv2
import numpy as np

from .contracts import matrix, transform


class Rectifier:
    def __init__(self, calibration, *, offline_reference=False):
        self.calibration = calibration
        self.offline_reference = offline_reference
        self.maps = {}

    def apply(self, frame):
        c = self.calibration.cameras[frame.camera]
        self.calibration.intrinsics(frame.camera)
        if frame.calibration_id != self.calibration.id:
            raise ValueError("Frame/calibration identity mismatch")
        if frame.rectified:
            if (frame.rectification_id != c["rectification_id"]
                    or list(frame.image.shape[1::-1]) != c["rectified_size"]):
                raise ValueError("GDC perspective output/calibration mismatch")
            return frame.image
        return self.raw(frame.camera, frame.image)

    def raw(self, camera, image):
        """Explicit offline reference; live callers must supply GDC output."""
        if not self.offline_reference:
            raise ValueError("GDC perspective frame required; CPU rectification is offline only")
        c = self.calibration.cameras[camera]
        k, d, kr = self.calibration.intrinsics(camera)
        if list(image.shape[1::-1]) != c["raw_size"]:
            raise ValueError("Raw image dimensions differ from calibration")
        if camera not in self.maps:
            fn = (cv2.fisheye.initUndistortRectifyMap if c["lens_model"] == "fisheye"
                  else cv2.initUndistortRectifyMap)
            # Keep the optical axes unchanged; T_base_camera_optical also applies
            # to this perspective output. GDC configurations must match this R=I.
            self.maps[camera] = fn(k, d, np.eye(3), kr, tuple(c["rectified_size"]), cv2.CV_32FC1)
        return cv2.remap(image, *self.maps[camera], cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT)


@dataclass
class Pose:
    stamp_ns: int
    map_id: str
    localization_session: str
    T_map_base: np.ndarray
    covariance_xy: np.ndarray
    valid: bool = True

    @classmethod
    def from_dict(cls, d):
        cov = matrix(d["covariance_xy"], (2, 2), "pose covariance")
        if not np.allclose(cov, cov.T) or np.linalg.eigvalsh(cov).min() < 0:
            raise ValueError("Pose covariance must be positive semidefinite")
        if (type(d["stamp_ns"]) is not int or d["stamp_ns"] <= 0
                or not d["map_id"] or not d["localization_session"]):
            raise ValueError("Pose identity/time missing")
        return cls(d["stamp_ns"], d["map_id"], d["localization_session"],
                   transform(d["T_map_base"], "T_map_base"), cov, d["valid"] is True)


class PoseBuffer:
    """Never extrapolate and never replace a missing source pose with latest TF."""
    def __init__(self, max_gap_ns=100_000_000):
        self.poses = deque(maxlen=600)
        self.max_gap_ns = max_gap_ns
        self.lock = threading.RLock()
        self.identity = None

    def put(self, pose):
        identity = (pose.map_id, pose.localization_session)
        with self.lock:
            if identity != self.identity:
                self.poses.clear()
                self.identity = identity
            if self.poses and pose.stamp_ns <= self.poses[-1].stamp_ns:
                raise ValueError("Out-of-order pose or unannounced clock reset")
            self.poses.append(pose)

    def at(self, stamp_ns):
        with self.lock:
            values = list(self.poses)
        i = bisect.bisect_left([p.stamp_ns for p in values], stamp_ns)
        if i < len(values) and values[i].stamp_ns == stamp_ns:
            if not values[i].valid:
                raise ValueError("Localization invalid")
            return values[i]
        if i == 0 or i == len(values):
            raise ValueError("Source-time pose not bracketed")
        a, b = values[i - 1:i + 1]
        if not a.valid or not b.valid or b.stamp_ns - a.stamp_ns > self.max_gap_ns:
            raise ValueError("Localization invalid or interpolation gap too large")
        fraction = (stamp_ns - a.stamp_ns) / (b.stamp_ns - a.stamp_ns)
        t = np.eye(4)
        rvec = cv2.Rodrigues(a.T_map_base[:3, :3].T @ b.T_map_base[:3, :3])[0]
        t[:3, :3] = a.T_map_base[:3, :3] @ cv2.Rodrigues(rvec * fraction)[0]
        t[:3, 3] = (1 - fraction) * a.T_map_base[:3, 3] + fraction * b.T_map_base[:3, 3]
        return Pose(stamp_ns, a.map_id, a.localization_session, t,
                    (1 - fraction) * a.covariance_xy + fraction * b.covariance_xy)

    def latest(self):
        with self.lock:
            return self.poses[-1] if self.poses else None


def pixel_inside(pixel, polygon):
    return cv2.pointPolygonTest(np.asarray(polygon, np.float32), tuple(map(float, pixel)), False) >= 0


class GroundProjector:
    def __init__(self, calibration, min_range=.3, max_range=10., max_sigma=.5):
        self.calibration = calibration
        self.min_range, self.max_range, self.max_sigma = min_range, max_range, max_sigma

    def ray(self, camera, pixel, rotation_noise=None):
        c = self.calibration.ground_camera(camera)
        k = np.asarray(c["K_rectified"], dtype=float)
        t = transform(c["T_base_camera_optical"])
        ray = np.linalg.solve(k, [pixel[0], pixel[1], 1.])
        ray = t[:3, :3] @ ray
        if rotation_noise is not None:
            ray = cv2.Rodrigues(np.asarray(rotation_noise, float))[0] @ ray
        normal = np.asarray(self.calibration.data["ground_plane_base"][:3])
        d = self.calibration.data["ground_plane_base"][3]
        denominator = float(normal @ ray)
        if abs(denominator) / np.linalg.norm(ray) < .025:
            raise ValueError("Ray too close to ground horizon")
        scale = -float(normal @ t[:3, 3] + d) / denominator
        if scale <= 0:
            raise ValueError("Ground is behind camera ray")
        point = t[:3, 3] + scale * ray
        if not self.min_range <= np.linalg.norm(point[:2]) <= self.max_range:
            raise ValueError("Ground point outside calibrated range")
        return point

    def project(self, camera, bbox, pose):
        c = self.calibration.ground_camera(camera)
        width, height = c["rectified_size"]
        box = matrix(bbox, (4,), "bbox")
        x1, y1, x2, y2 = box
        if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
            raise ValueError("Box outside perspective image")
        pixel = ((x1 + x2) * .5, y2)
        if (not 2 < pixel[0] < width - 2 or not 2 < pixel[1] < height - 2
                or not pixel_inside(pixel, c["valid_ground_polygon_px"])):
            raise ValueError("Contact point in invalid/occluded ground region")
        p = self.ray(camera, pixel)
        # Pixel jitter plus angular extrinsic errors. A non-shrinking calibration
        # floor avoids falsely precise long-range monocular estimates.
        cov = np.eye(3) * c["position_sigma_m"] ** 2
        for delta in ((c["pixel_sigma"], 0), (0, c["pixel_sigma"])):
            q = self.ray(camera, np.asarray(pixel) + delta)
            cov += np.outer(q - p, q - p)
        for axis in range(3):
            delta = np.zeros(3)
            delta[axis] = c["extrinsic_angle_sigma_rad"]
            q = self.ray(camera, pixel, delta)
            cov += np.outer(q - p, q - p)
        r = pose.T_map_base[:3, :3]
        covariance = (r @ cov @ r.T)[:2, :2] + pose.covariance_xy
        if math.sqrt(float(np.linalg.eigvalsh(covariance).max())) > self.max_sigma:
            raise ValueError("Ground estimate too uncertain")
        world = pose.T_map_base[:3, :3] @ p + pose.T_map_base[:3, 3]
        return p, world, covariance, pixel

    def visible(self, camera, world, pose):
        try:
            c = self.calibration.ground_camera(camera)
            base = np.linalg.inv(pose.T_map_base) @ np.r_[world, 1.]
            opt = np.linalg.inv(np.asarray(c["T_base_camera_optical"])) @ base
            if opt[2] <= 0 or not self.min_range <= np.linalg.norm(base[:2]) <= self.max_range:
                return False
            pixel = np.asarray(c["K_rectified"]) @ opt[:3]
            return pixel_inside(pixel[:2] / pixel[2], c["valid_ground_polygon_px"])
        except (ValueError, KeyError):
            return False
