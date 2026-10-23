"""Analytic ground texture -> four synthetic fisheye images for offline BEV."""
import copy
from pathlib import Path
import math

import cv2
import numpy as np

from . import CAMERAS
from .contracts import atomic_json, read_json

STAMP = 1_792_500_000_000_000_000


def make_scene(directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    template = Path(__file__).resolve().parent / "config/calibration.template.json"
    data = copy.deepcopy(read_json(template))
    data.update(id="SYNTHETIC-BEV-ONLY", synthetic=True, ground_valid=True,
                ground_plane_base=[0., 0., 1., 0.])
    k = np.array([[200., 0, 320.], [0, 200., 240.], [0, 0, 1.]])
    d = np.array([-.03, .003, 0., 0.])
    pitch = math.pi / 4
    r0 = np.array([[0, -math.sin(pitch), math.cos(pitch)],
                   [-1., 0, 0], [0, -math.cos(pitch), -math.sin(pitch)]])
    r, u = np.indices((1200, 1200))
    ground_x, ground_y = 6 - (r + .5) * .01, 6 - (u + .5) * .01
    texture = np.zeros((1200, 1200, 3), np.uint8)
    texture[:] = [65, 80, 65]
    checker = ((np.floor(ground_x) + np.floor(ground_y)) % 2) == 0
    texture[checker] = [90, 100, 90]
    grid = (np.mod(ground_x, 1) < .025) | (np.mod(ground_y, 1) < .025)
    texture[grid] = [160, 160, 160]
    for x, y, colour in ((2., 0., (30, 80, 240)), (0., -2., (40, 220, 70)),
                         (-2., 0., (230, 100, 30)), (0., 2., (40, 220, 230))):
        texture[(abs(ground_x - x) < .25) & (abs(ground_y - y) < .25)] = colour
    cv2.imwrite(str(directory / "ground_truth.png"), texture)
    rows, cols = np.indices((480, 640), dtype=float)
    pixels = np.stack((cols, rows), axis=-1).reshape(-1, 1, 2)
    normalized = cv2.fisheye.undistortPoints(pixels, k, d).reshape(-1, 2)
    rays = np.c_[normalized, np.ones(len(normalized))]
    frames = {}
    for name, yaw in zip(CAMERAS, (0, -math.pi / 2, math.pi, math.pi / 2)):
        cy, sy = math.cos(yaw), math.sin(yaw)
        t = np.eye(4)
        t[:3, :3] = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ r0
        t[:3, 3] = [.25 * cy, .25 * sy, .9]
        c = data["cameras"][name]
        c.update(intrinsics_valid=True, extrinsics_valid=True, raw_size=[640, 480],
                 rectified_size=[640, 480], K_raw=k.tolist(), D=d.tolist(),
                 K_rectified=k.tolist(), rectification_id="SYNTHETIC-BEV-" + name,
                 T_base_camera_optical=t.tolist(),
                 valid_ground_polygon_px=[[4, 4], [635, 4], [635, 475], [4, 475]],
                 pixel_sigma=1., extrinsic_angle_sigma_rad=.001, position_sigma_m=.02,
                 calibration_evidence="Analytic synthetic scene; never install on vehicle")
        base_rays = rays @ t[:3, :3].T
        valid = base_rays[:, 2] < -1e-3
        scale = -.9 / np.where(valid, base_rays[:, 2], -1)
        xyz = t[:3, 3] + base_rays * scale[:, None]
        mx = ((6 - xyz[:, 1]) / .01 - .5).reshape(480, 640).astype(np.float32)
        my = ((6 - xyz[:, 0]) / .01 - .5).reshape(480, 640).astype(np.float32)
        image = cv2.remap(texture, mx, my, cv2.INTER_LINEAR)
        image[~valid.reshape(480, 640)] = [24, 24, 24]
        cv2.imwrite(str(directory / (name + ".png")), image)
        frames[name] = dict(camera=name, image=name + ".png", session="synthetic-bev",
                            sequence=0, raw_stamp=STAMP, stamp_ns=STAMP,
                            received_ns=STAMP + 10_000_000, clock_id="synthetic",
                            clock_mapping_id="synthetic-identity", calibration_id=data["id"],
                            rectified=False, rectification_id="", synthetic=True)
    atomic_json(directory / "calibration.json", data)
    atomic_json(directory / "frames.json", dict(schema="j6m.surround.bev.frames.v1",
                                               now_ns=STAMP + 20_000_000, frames=frames))
    return directory / "frames.json", directory / "calibration.json"
