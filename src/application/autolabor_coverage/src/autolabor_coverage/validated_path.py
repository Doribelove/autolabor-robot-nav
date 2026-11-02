"""Validate dense, signed vehicle trajectories imported with a static-map plan."""
import math
import numpy as np


def check_path(values, grid, front, rear, half_width, turning_radius):
    p = np.asarray(values, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3 or len(p) < 2 or not np.isfinite(p).all():
        raise ValueError('trajectory must contain finite x/y/yaw poses')
    delta = np.diff(p[:, :2], axis=0)
    lengths = np.linalg.norm(delta, axis=1)
    angles = np.arctan2(np.sin(np.diff(p[:, 2])), np.cos(np.diff(p[:, 2])))
    nonzero = lengths > 1e-7
    if np.any((~nonzero) & (np.abs(angles) > 1e-6)):
        raise ValueError('trajectory contains in-place rotation')
    if np.any(lengths > min(.05, grid.resolution / 2) + 1e-5):
        raise ValueError('trajectory must be sampled at half-cell spacing or finer')
    curvature = np.zeros_like(lengths)
    curvature[nonzero] = 2 * np.sin(np.abs(angles[nonzero]) / 2) / lengths[nonzero]
    if np.any(curvature > 1 / turning_radius + 1e-4):
        raise ValueError('trajectory violates the minimum turning radius')
    mid = p[:-1, 2] + angles / 2
    lateral = np.abs(-delta[:, 0] * np.sin(mid) + delta[:, 1] * np.cos(mid))
    if np.any(lateral > np.maximum(.002, lengths * .01)):
        raise ValueError('trajectory contains lateral vehicle motion')
    # Full asymmetric footprint, including its interior. A perimeter-only
    # test can miss a pillar or an occupied island inside the rectangle.
    spacing = min(.05, grid.resolution / 2)
    xx, yy = np.meshgrid(np.linspace(-rear, front, int(math.ceil((front + rear) / spacing)) + 1),
                         np.linspace(-half_width, half_width, int(math.ceil(2 * half_width / spacing)) + 1))
    xx, yy = xx.ravel(), yy.ravel()
    cells = np.asarray(grid.data, dtype=np.int8).reshape(grid.height, grid.width)
    for begin in range(0, len(p), 512):
        points = p[begin:begin + 512]
        c, s = np.cos(points[:, 2, None]), np.sin(points[:, 2, None])
        ix = np.floor((points[:, 0, None] + c * xx - s * yy - grid.origin_x) / grid.resolution).astype(int)
        iy = np.floor((points[:, 1, None] + s * xx + c * yy - grid.origin_y) / grid.resolution).astype(int)
        inside = (ix >= 0) & (iy >= 0) & (ix < grid.width) & (iy < grid.height)
        free = cells[np.clip(iy, 0, grid.height - 1), np.clip(ix, 0, grid.width - 1)] == 0
        if not np.all(inside & free):
            raise ValueError('trajectory footprint intersects occupied/unknown space')
    return p


def same_pose(a, b, tolerance=1e-4):
    return (math.hypot(a[0] - b[0], a[1] - b[1]) <= tolerance and
            abs(math.atan2(math.sin(a[2] - b[2]), math.cos(a[2] - b[2]))) <= tolerance)
