"""Offline calibration helpers; results remain candidates until measured checks."""
import glob
from pathlib import Path

import cv2
import numpy as np

from .contracts import atomic_json, read_json, sha256


def intrinsics(pattern, columns, rows, square_m, output):
    if min(columns, rows) < 3 or not 0 < square_m < 1:
        raise ValueError("Specify checkerboard inner corners and measured square size in metres")
    template = np.zeros((1, columns * rows, 3), np.float64)
    template[0, :, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2) * square_m
    objects, points, evidence, size = [], [], [], None
    for filename in sorted(glob.glob(pattern)):
        image = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError("Unreadable calibration image: " + filename)
        current = image.shape[::-1]
        if size is not None and current != size:
            raise ValueError("Mixed image resolutions")
        size = current
        found, corners = cv2.findChessboardCorners(image, (columns, rows))
        if not found:
            continue
        corners = cv2.cornerSubPix(image, corners, (5, 5), (-1, -1),
                                  (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 40, .001))
        objects.append(template.copy())
        points.append(corners.astype(np.float64).reshape(1, -1, 2))
        evidence.append(dict(file=str(Path(filename).resolve()), sha256=sha256(filename)))
    if len(points) < 15:
        raise ValueError("Need at least 15 successful, diverse fisheye checkerboard views")
    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW | cv2.fisheye.CALIB_CHECK_COND
    rms, k, d, rvecs, tvecs = cv2.fisheye.calibrate(
        objects, points, size, np.eye(3), np.zeros((4, 1)), flags=flags,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 100, 1e-8))
    per_view = []
    for obj, measured, r, t in zip(objects, points, rvecs, tvecs):
        predicted = cv2.fisheye.projectPoints(obj, r, t, k, d)[0]
        per_view.append(float(np.sqrt(np.mean(np.sum((predicted - measured) ** 2, axis=-1)))))
    # Preserve native resolution; choose final perspective K/FOV intentionally,
    # then generate the matching GDC bin. This candidate is not auto-activated.
    kr = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(k, d, size, np.eye(3), balance=0)
    atomic_json(output, dict(kind="fisheye_intrinsics_candidate", activated=False,
                            rms_px=float(rms), per_view_rms_px=per_view,
                            raw_size=list(size), rectified_size=list(size),
                            K_raw=k.tolist(), D=d.ravel().tolist(), K_rectified=kr.tolist(),
                            R_rectified=np.eye(3).tolist(), evidence=evidence,
                            note="Validate independent images, image edges and ground-distance errors before activation."))


def extrinsics(correspondences, output):
    data = read_json(correspondences)
    world = np.asarray(data["points_base_m"], np.float64)
    pixels = np.asarray(data["points_rectified_px"], np.float64)
    k = np.asarray(data["K_rectified"], np.float64)
    if (world.ndim != 2 or world.shape[1] != 3 or len(world) < 6
            or pixels.shape != (len(world), 2) or k.shape != (3, 3)
            or not all(np.isfinite(a).all() for a in (world, pixels, k))
            or np.linalg.matrix_rank(world - world.mean(axis=0)) < 2):
        raise ValueError("Need >=6 measured, non-collinear 3D base / rectified 2D correspondences")
    ok, rvec, tvec = cv2.solvePnP(world, pixels, k, np.zeros(5), flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise ValueError("Extrinsic solve failed")
    camera_from_base = np.eye(4)
    camera_from_base[:3, :3] = cv2.Rodrigues(rvec)[0]
    camera_from_base[:3, 3] = tvec.ravel()
    optical = (camera_from_base @ np.c_[world, np.ones(len(world))].T).T
    if (optical[:, 2] <= 0).any():
        raise ValueError("Extrinsic solution puts control points behind camera")
    predicted = cv2.projectPoints(world, rvec, tvec, k, np.zeros(5))[0].reshape(-1, 2)
    residuals = np.linalg.norm(predicted - pixels, axis=1)
    atomic_json(output, dict(kind="extrinsics_candidate", activated=False,
                            T_base_camera_optical=np.linalg.inv(camera_from_base).tolist(),
                            reprojection_rms_px=float(np.sqrt(np.mean(residuals ** 2))),
                            residuals_px=residuals.tolist(), input_sha256=sha256(correspondences),
                            note="Planar pose can be ambiguous. Check measured height/orientation and held-out ground points; do not auto-enable."))
