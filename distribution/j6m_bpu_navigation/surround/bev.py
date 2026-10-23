"""Camera-to-ground BEV compositor. No ROS, detector, BPU or device dependency."""
import hashlib
import json
import time

import cv2
import numpy as np

from . import CAMERAS


class BevComposer:
    """Precompute raw/perspective -> BEV maps, then remap each input once."""
    def __init__(self, calibration, config, *, perspective_only=False):
        begin = time.perf_counter()
        self.calibration, self.config = calibration, dict(config)
        self.perspective_only = perspective_only
        if config.get("schema") != "j6m.surround.bev.config.v1":
            raise ValueError("Unknown BEV configuration")
        for key in ("x_range_m", "y_range_m"):
            limits = np.asarray(config[key], dtype=float)
            if limits.shape != (2,) or not np.isfinite(limits).all() or not limits[0] < limits[1]:
                raise ValueError("Invalid BEV range: " + key)
        self.resolution = float(config["metres_per_pixel"])
        if not .005 <= self.resolution <= .2:
            raise ValueError("BEV metres_per_pixel must be in [0.005, 0.2]")
        self.width = int(np.ceil(np.diff(config["y_range_m"])[0] / self.resolution))
        self.height = int(np.ceil(np.diff(config["x_range_m"])[0] / self.resolution))
        if not 2 <= min(self.width, self.height) <= max(self.width, self.height) <= 2048:
            raise ValueError("BEV dimensions must be in [2, 2048]")
        if (not 0 < config["max_source_age_ms"] <= 500
                or not 0 <= config["max_skew_ms"] <= 100
                or not 0 < config["feather_pixels"] <= 512
                or not 0 <= config["min_range_m"] < config["max_range_m"] <= 10
                or type(config["require_all_cameras"]) is not bool
                or not 0 < config["output_hz"] <= 30):
            raise ValueError("Invalid BEV freshness/blending/range configuration")
        for name in CAMERAS:
            calibration.ground_camera(name)
        plane = np.asarray(calibration.data["ground_plane_base"])
        if abs(plane[2]) < .1:
            raise ValueError("BEV ground plane is nearly vertical")
        row, col = np.indices((self.height, self.width), dtype=float)
        x = config["x_range_m"][1] - (row + .5) * self.resolution
        y = config["y_range_m"][1] - (col + .5) * self.resolution
        z = -(plane[0] * x + plane[1] * y + plane[3]) / plane[2]
        world = np.stack((x, y, z, np.ones_like(x)), axis=0).reshape(4, -1)
        radius = np.hypot(x, y)
        self.maps = {}
        for name in CAMERAS:
            c = calibration.cameras[name]
            k, d, kr = calibration.intrinsics(name)
            optical = (np.linalg.inv(np.asarray(c["T_base_camera_optical"])) @ world)[:3].T
            valid = (optical[:, 2] > .05).reshape(x.shape)
            # Avoid enormous or non-finite projections behind the camera.
            optical[~valid.ravel()] = [0, 0, 1]
            rectified = optical @ kr.T
            rectified = rectified[:, :2] / rectified[:, 2:3]
            rx, ry = [a.reshape(x.shape).astype(np.float32) for a in rectified.T]
            width, height = c["rectified_size"]
            mask = np.zeros((height, width), np.uint8)
            cv2.fillPoly(mask, [np.asarray(c["valid_ground_polygon_px"], np.int32)], 255)
            distances = cv2.distanceTransform(mask, cv2.DIST_L2, 3)
            weight = cv2.remap(distances, rx, ry, cv2.INTER_LINEAR,
                               borderMode=cv2.BORDER_CONSTANT)
            valid &= ((radius >= config["min_range_m"]) & (radius <= config["max_range_m"])
                      & (rx >= 0) & (rx < width - 1) & (ry >= 0) & (ry < height - 1))
            weight = np.minimum(weight / config["feather_pixels"], 1).astype(np.float32) * valid
            self.maps[(name, True)] = rx, ry, weight
            if perspective_only:
                # Live GDC input needs only the perspective -> ground mapping.
                # Do not even construct CPU fisheye maps for the live service.
                continue
            args = (optical.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), k, d)
            raw = (cv2.fisheye.projectPoints(*args, alpha=float(k[0, 1] / k[0, 0]))[0]
                   if c["lens_model"] == "fisheye" else cv2.projectPoints(*args)[0]).reshape(-1, 2)
            mx, my = [a.reshape(x.shape).astype(np.float32) for a in raw.T]
            rw, rh = c["raw_size"]
            raw_valid = np.isfinite(mx) & np.isfinite(my) & (mx >= 0) & (mx < rw - 1) & (my >= 0) & (my < rh - 1)
            mx[~raw_valid], my[~raw_valid] = -1, -1
            self.maps[(name, False)] = mx, my, weight * raw_valid
        self.calibration_sha256 = hashlib.sha256(
            json.dumps(calibration.data, sort_keys=True, allow_nan=False).encode()).hexdigest()
        self.config_sha256 = hashlib.sha256(
            json.dumps(config, sort_keys=True, allow_nan=False).encode()).hexdigest()
        self.map_build_ms = (time.perf_counter() - begin) * 1000

    def pixel(self, x, y):
        """base_link x forward/y left -> image u right/v down, pixel centres."""
        return ((self.config["y_range_m"][1] - y) / self.resolution - .5,
                (self.config["x_range_m"][1] - x) / self.resolution - .5)

    def compose(self, frames, now_ns):
        begin = time.perf_counter()
        if not frames or not set(frames).issubset(CAMERAS):
            raise ValueError("Expected named front/right/rear/left frames")
        if self.config["require_all_cameras"] and set(frames) != set(CAMERAS):
            raise ValueError("Waiting for all four camera frames")
        max_age = int(self.config["max_source_age_ms"] * 1e6)
        for name, f in frames.items():
            f.validate(now_ns, max_age)
            if self.perspective_only and not f.rectified:
                raise ValueError("Live BEV requires GDC perspective input; raw images are offline only")
            c = self.calibration.cameras[name]
            if f.camera != name or f.calibration_id != self.calibration.id:
                raise ValueError("Camera/calibration identity mismatch")
            if f.synthetic != self.calibration.synthetic:
                raise ValueError("Synthetic and real inputs cannot be mixed")
            if list(f.image.shape[1::-1]) != c["rectified_size" if f.rectified else "raw_size"]:
                raise ValueError("BEV image dimensions differ from calibration")
            if f.rectified and f.rectification_id != c["rectification_id"]:
                raise ValueError("Rectified image identity mismatch")
        stamps = [f.stamp_ns for f in frames.values()]
        skew = max(stamps) - min(stamps)
        if skew > self.config["max_skew_ms"] * 1e6:
            raise ValueError("Camera exposure skew exceeds BEV limit")
        total = np.zeros((self.height, self.width, 3), np.float32)
        weights = np.zeros((self.height, self.width), np.float32)
        counts = np.zeros((self.height, self.width), np.uint8)
        for name in CAMERAS:
            if name not in frames:
                continue
            frame = frames[name]
            mx, my, weight = self.maps[(name, frame.rectified)]
            image = cv2.remap(frame.image, mx, my, cv2.INTER_LINEAR,
                              borderMode=cv2.BORDER_CONSTANT)
            total += image.astype(np.float32) * weight[:, :, None]
            weights += weight
            counts += (weight > 0).astype(np.uint8)
        covered = weights > 0
        result = np.clip(total / np.maximum(weights[:, :, None], 1e-8), 0, 255).astype(np.uint8)
        meta = dict(schema="j6m.surround.bev.v1", frame_id="base_link",
                    calibration_id=self.calibration.id, calibration_sha256=self.calibration_sha256,
                    config_sha256=self.config_sha256, synthetic=self.calibration.synthetic,
                    sources={name: f.identity() for name, f in frames.items()},
                    source_oldest_ns=min(stamps), source_newest_ns=max(stamps),
                    max_source_age_ms=self.config["max_source_age_ms"],
                    source_age_ms=(now_ns - min(stamps)) / 1e6, exposure_skew_ms=skew / 1e6,
                    dimensions=[self.width, self.height], metres_per_pixel=self.resolution,
                    x_range_m=self.config["x_range_m"], y_range_m=self.config["y_range_m"],
                    pixel_centres=True, axes="image top: +base X; image left: +base Y",
                    coverage_fraction=float(covered.mean()), max_contributors=int(counts.max()),
                    processing_ms=(time.perf_counter() - begin) * 1000,
                    map_build_ms=self.map_build_ms, backend="cpu_cached_remap",
                    production_ready=False, motion_eligible=False)
        return result, counts, meta


def preview(bev, frames, meta):
    """Debug composite only; exported bev.png remains free of annotations."""
    tiles = []
    for name in CAMERAS:
        panel = (cv2.resize(frames[name].image, (320, 240)) if name in frames
                 else np.zeros((240, 320, 3), np.uint8))
        cv2.rectangle(panel, (0, 0), (320, 28), (24, 24, 24), -1)
        cv2.putText(panel, name, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, .6, (240, 240, 240), 1)
        tiles.append(panel)
    canvas = np.full((560, 1140, 3), 24, np.uint8)
    canvas[:480, :640] = np.vstack((np.hstack(tiles[:2]), np.hstack(tiles[2:])))
    scale = min(500 / bev.shape[1], 500 / bev.shape[0])
    bird = cv2.resize(bev, (round(bev.shape[1] * scale), round(bev.shape[0] * scale)))
    canvas[:bird.shape[0], 640:640 + bird.shape[1]] = bird
    label = "SYNTHETIC / OFFLINE" if meta["synthetic"] else "CALIBRATED GROUND BEV"
    cv2.putText(canvas, label, (655, 30), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 210, 255), 2)
    cv2.putText(canvas, "X forward (up) / Y left | CPU remap | no detector / no ROS",
                (12, 520), cv2.FONT_HERSHEY_SIMPLEX, .55, (220, 220, 220), 1)
    cv2.putText(canvas, "Observation only | production_ready=false | motion_eligible=false",
                (12, 546), cv2.FONT_HERSHEY_SIMPLEX, .5, (190, 190, 190), 1)
    return canvas
