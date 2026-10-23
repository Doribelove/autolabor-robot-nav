"""Deterministic synthetic input, clearly separated from hardware evidence."""
import copy
import json
import math
from pathlib import Path

import cv2
import numpy as np

from . import CAMERAS
from .contracts import atomic_json, read_json


def make_demo(directory, template):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    calibration = copy.deepcopy(template)
    calibration.update(id="SYNTHETIC-FIXTURE-ONLY", synthetic=True, ground_valid=True,
                       ground_plane_base=[0., 0., 1., 0.])
    events = []
    k = np.array([[240., 0, 320.], [0, 240., 240.], [0, 0, 1.]])
    pitch = math.pi / 4
    r0 = np.array([[0, -math.sin(pitch), math.cos(pitch)],
                   [-1., 0, 0], [0, -math.cos(pitch), -math.sin(pitch)]])
    for name, yaw in zip(CAMERAS, (0, -math.pi / 2, math.pi, math.pi / 2)):
        cy, sy = math.cos(yaw), math.sin(yaw)
        t = np.eye(4)
        t[:3, :3] = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ r0
        t[:3, 3] = [0, 0, .9]
        calibration["cameras"][name].update(
            intrinsics_valid=True, extrinsics_valid=True, raw_size=[640, 480],
            rectified_size=[640, 480], K_raw=k.tolist(), D=[0., 0., 0., 0.],
            K_rectified=k.tolist(), rectification_id="SYNTHETIC-perspective-" + name,
            T_base_camera_optical=t.tolist(),
            valid_ground_polygon_px=[[4, 4], [635, 4], [635, 475], [4, 475]],
            pixel_sigma=1., extrinsic_angle_sigma_rad=.001, position_sigma_m=.02,
            calibration_evidence="Synthetic fixture, never install on vehicle")
    start = 1_792_500_000_000_000_000
    world_points = [np.array([2., 1.2, 0.]), np.array([3., -.7, 0.])]
    for index in range(18):
        stamp = start + index * 80_000_000
        pose = np.eye(4)
        pose[0, 3] = index * .01
        events.append(dict(type="pose", stamp_ns=stamp, map_id="synthetic-map",
                           localization_session="synthetic-localization", T_map_base=pose.tolist(),
                           covariance_xy=[[.0004, 0], [0, .0004]], valid=True))
        for name in CAMERAS:
            image = np.full((480, 640, 3), (45, 60, 60), np.uint8)
            c = calibration["cameras"][name]
            objects = []
            for i, world in enumerate(world_points):
                point = np.linalg.inv(np.asarray(c["T_base_camera_optical"])) @ np.linalg.inv(pose) @ np.r_[world, 1]
                if point[2] <= 0:
                    continue
                pixel = k @ point[:3]
                u, v = pixel[:2] / pixel[2]
                if not (24 <= u < 616 and 40 < v < 475):
                    continue
                box = [u - 14, v - 22, u + 14, v]
                cv2.rectangle(image, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])),
                              (70, 180, 250) if i == 0 else (190, 100, 80), -1)
                p = [.02] * 5
                p[i] = .92
                objects.append(dict(bbox=box, detect_confidence=.92 - .03 * i,
                                    class_id=0, probabilities=p))
            cv2.putText(image, "SYNTHETIC " + name, (20, 440), cv2.FONT_HERSHEY_SIMPLEX,
                        .8, (220, 220, 220), 2)
            filename = "%s_%03d.png" % (name, index)
            cv2.imwrite(str(directory / filename), image)
            events.append(dict(type="frame", camera=name, session="synthetic-capture",
                               sequence=index, raw_stamp=stamp, stamp_ns=stamp,
                               received_ns=stamp + 15_000_000, now_ns=stamp + 20_000_000,
                               clock_id="synthetic", clock_mapping_id="synthetic-identity",
                               calibration_id=calibration["id"], rectified=True,
                               rectification_id=c["rectification_id"], synthetic=True,
                               image=filename, detections=objects))
    atomic_json(directory / "calibration.json", calibration)
    with (directory / "events.jsonl").open("w") as stream:
        for event in events:
            stream.write(json.dumps(event, allow_nan=False) + "\n")
    return directory / "events.jsonl", directory / "calibration.json"
