"""Pinhole projection helpers for intersecting an image ray with the ground."""

from typing import Iterable, Tuple

import cv2
import numpy as np


class ProjectionError(ValueError):
    """Raised when a pixel cannot produce a physically valid ground point."""


def _as_finite_array(values: Iterable[float], shape: Tuple[int, ...], name: str):
    try:
        array = np.asarray(values, dtype=np.float64).reshape(shape)
    except (TypeError, ValueError) as error:
        raise ProjectionError(
            "{} does not have the required shape {}".format(name, shape)
        ) from error
    if not np.all(np.isfinite(array)):
        raise ProjectionError("{} contains non-finite values".format(name))
    return array


def pixel_to_camera_ray(
    u: float,
    v: float,
    camera_matrix: Iterable[float],
    distortion: Iterable[float],
    distortion_model: str = "plumb_bob",
) -> np.ndarray:
    """Return an optical-frame ray (x right, y down, z forward)."""
    if not np.isfinite(u) or not np.isfinite(v):
        raise ProjectionError("pixel is not finite")
    k = _as_finite_array(camera_matrix, (3, 3), "camera matrix")
    if k[0, 0] <= 0.0 or k[1, 1] <= 0.0:
        raise ProjectionError("camera is uncalibrated")
    d = np.asarray(list(distortion), dtype=np.float64)
    if d.size and not np.all(np.isfinite(d)):
        raise ProjectionError("distortion contains non-finite values")

    pixel = np.array([[[float(u), float(v)]]], dtype=np.float64)
    if distortion_model in ("plumb_bob", "rational_polynomial", ""):
        if d.size not in (0, 4, 5, 8, 12, 14):
            raise ProjectionError(
                "pinhole calibration has an invalid distortion length"
            )
        try:
            normalized = cv2.undistortPoints(
                pixel, k, d if d.size else None
            )
        except cv2.error as error:
            raise ProjectionError(
                "OpenCV rejected the pinhole calibration"
            ) from error
    elif distortion_model in ("equidistant", "fisheye"):
        if d.size != 4:
            raise ProjectionError("fisheye calibration requires four coefficients")
        try:
            normalized = cv2.fisheye.undistortPoints(
                pixel, k, d.reshape(4, 1)
            )
        except cv2.error as error:
            raise ProjectionError(
                "OpenCV rejected the fisheye calibration"
            ) from error
    else:
        raise ProjectionError(
            "unsupported distortion model: {}".format(distortion_model)
        )
    try:
        x, y = normalized.reshape(2)
    except ValueError as error:
        raise ProjectionError("undistortion returned an invalid ray") from error
    if not np.isfinite(x) or not np.isfinite(y):
        raise ProjectionError("undistortion returned a non-finite ray")
    ray = np.array([x, y, 1.0], dtype=np.float64)
    ray /= np.linalg.norm(ray)
    return ray


def intersect_ray_with_ground(
    camera_origin_in_base: Iterable[float],
    ray_in_base: Iterable[float],
    ground_z: float = 0.0,
    parallel_epsilon: float = 1e-6,
) -> np.ndarray:
    origin = _as_finite_array(camera_origin_in_base, (3,), "camera origin")
    ray = _as_finite_array(ray_in_base, (3,), "camera ray")
    if not np.isfinite(ground_z):
        raise ProjectionError("ground height is not finite")
    if abs(ray[2]) <= parallel_epsilon:
        raise ProjectionError("camera ray is parallel to the ground")
    scale = (float(ground_z) - origin[2]) / ray[2]
    if scale <= 0.0:
        raise ProjectionError("camera ray intersects the ground behind the camera")
    point = origin + scale * ray
    point[2] = float(ground_z)
    if not np.all(np.isfinite(point)):
        raise ProjectionError("projected point is not finite")
    return point


def project_pixel_to_ground(
    u: float,
    v: float,
    camera_matrix: Iterable[float],
    distortion: Iterable[float],
    rotation_base_from_camera: Iterable[float],
    camera_origin_in_base: Iterable[float],
    ground_z: float = 0.0,
    distortion_model: str = "plumb_bob",
) -> np.ndarray:
    ray_camera = pixel_to_camera_ray(
        u, v, camera_matrix, distortion, distortion_model
    )
    rotation = _as_finite_array(
        rotation_base_from_camera, (3, 3), "camera rotation"
    )
    ray_base = rotation.dot(ray_camera)
    return intersect_ray_with_ground(camera_origin_in_base, ray_base, ground_z)


def project_ground_point_to_pixel(
    point_in_base: Iterable[float],
    camera_matrix: Iterable[float],
    rotation_base_from_camera: Iterable[float],
    camera_origin_in_base: Iterable[float],
) -> Tuple[float, float]:
    """Ideal, undistorted forward projection used by the deterministic simulator."""
    point = _as_finite_array(point_in_base, (3,), "ground point")
    origin = _as_finite_array(camera_origin_in_base, (3,), "camera origin")
    rotation = _as_finite_array(
        rotation_base_from_camera, (3, 3), "camera rotation"
    )
    k = _as_finite_array(camera_matrix, (3, 3), "camera matrix")
    camera_point = rotation.T.dot(point - origin)
    if camera_point[2] <= 0.0:
        raise ProjectionError("point is behind the camera")
    pixel = k.dot(camera_point / camera_point[2])
    return float(pixel[0]), float(pixel[1])


def optical_rotation_base(camera_pitch_down_radians: float) -> np.ndarray:
    """Map ROS optical axes into base axes for a forward, downward-pitched camera."""
    pitch = float(camera_pitch_down_radians)
    c = np.cos(pitch)
    s = np.sin(pitch)
    base_pitch = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    optical_level = np.array(
        [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
    )
    return base_pitch.dot(optical_level)
