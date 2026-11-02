"""CPU preparation for the pinned official BEV-VTv2 six-slot HBM.

Runs the supplied HEAL's pure geometry and box-decoding methods directly,
without importing its x86-only training/quantization plugin on the Jetson.
This does not replace, emulate, or train the neural network: that runs on J6M.
"""
import ast
import hashlib
from pathlib import Path
from types import SimpleNamespace, MethodType

import cv2
import numpy as np
import torch
from torchvision.transforms import functional as TVF

ORDER = ('front', 'left', 'right', 'rear')
CLASSES = ('car', 'truck', 'construction_vehicle', 'bus', 'trailer', 'barrier',
           'motorcycle', 'bicycle', 'pedestrian', 'traffic_cone')
MODEL_SHA = '05443372d993714db0b226de6771a114876217fb02248c7bb1ec077a8b1af2e6'
MODEL_NAME = 'bev_vtv2_resnet50_nuscenes'


class HealCPU:
    def __init__(self, source):
        self.source = Path(source)
        self.hashes = {}
        self.scope = dict(torch=torch, np=np)
        self._load('heal/core/nus_box3d_utils.py', 'adjust_coords')
        self._load('heal/models/task_modules/bev/view_transformer.py',
                   '_gen_3d_points', 'ViewTransformer')
        self._load('heal/models/task_modules/bev/view_transformer.py',
                   '_gen_reference_point', 'LSSTransformer')
        self._load('heal/models/task_modules/centerpoint/bbox_coders.py',
                   'CenterPointBBoxCoder')
        self.geometry = SimpleNamespace(num_views=6, bev_size=(51.2, 51.2, .8),
            grid_size=(128, 128), z_range=(-5., 3., .125), depth=60,
            num_points=10, cal_minmax=False)
        self.geometry._gen_3d_points = MethodType(self.scope['_gen_3d_points'], self.geometry)
        self.decoder = self.scope['CenterPointBBoxCoder'](
            pc_range=[-51.2, -51.2], out_size_factor=8, voxel_size=[.1, .1],
            post_center_range=[-61.2, -61.2, -10., 61.2, 61.2, 10.],
            max_num=500, score_threshold=.1)

    def _load(self, relative, name, parent=None):
        path = self.source / relative
        data = path.read_bytes()
        self.hashes[relative] = hashlib.sha256(data).hexdigest()
        tree = ast.parse(data, filename=str(path))
        nodes = tree.body
        if parent:
            nodes = next(n for n in nodes if isinstance(n, ast.ClassDef) and n.name == parent).body
        node = next(n for n in nodes if isinstance(n, (ast.FunctionDef, ast.ClassDef)) and n.name == name)
        # Only registry/decorator dependencies are removed; method bodies remain unchanged.
        node.decorator_list = []
        future = ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0)
        module = ast.fix_missing_locations(ast.Module(body=[future, node], type_ignores=[]))
        exec(compile(module, str(path), 'exec'), self.scope)

    def grids(self, meta):
        homographies = []
        for name in ORDER:
            width, height = meta['camera_image_sizes'][name]
            k = np.eye(4)
            k[:3, :3] = meta['projection_intrinsics'][name]
            transform = np.array(meta['T_ground_from_upright_optical'][name], dtype=np.float64)
            # Official resize 396x704, bottom crop 256x704, then stride-16 feature map.
            resize = np.diag([704/width, 396/height, 1., 1.])
            crop = np.eye(4); crop[1, 2] = -140
            feature = np.diag([1/16., 1/16., 1., 1.])
            homographies.append(feature @ crop @ resize @ k @ np.linalg.inv(transform))
        # A negative camera depth makes both absent views invalid in HEAL's own code.
        missing = np.eye(4); missing[2] = [0, 0, 0, -100]
        homographies.extend([missing, missing])
        with torch.no_grad():
            points = self.scope['_gen_reference_point'](
                self.geometry, torch.tensor(np.array(homographies)), (16, 44))
        raw = [p.numpy() for p in points]
        yy, xx = np.indices((128, 128))
        xy = raw[0] + np.stack([xx, yy], axis=-1)
        valid = (xy[..., 0] >= 0) & (xy[..., 0] < 44) & (xy[..., 1] >= 0) & (xy[..., 1] < 96)
        cameras = np.floor_divide(xy[..., 1], 16).astype(np.int64)
        if np.any(valid & (cameras >= 4)):
            raise ValueError('Absent camera selected by HEAL geometry')
        statistics = dict(selected_points_per_camera={n: int(np.sum(valid & (cameras == i)))
                          for i, n in enumerate(ORDER)}, absent_camera_selected=0,
                          sampled_bev_cells=int(np.any(valid, axis=0).sum()))
        return raw, statistics, valid.sum(axis=0)

    def decode(self, outputs):
        reg, height, dim, rot, vel, heatmap = [torch.from_numpy(a) for a in outputs]
        if any(not torch.isfinite(a).all() for a in (reg, height, dim, rot, vel, heatmap)):
            raise ValueError('Nonfinite model output')
        with torch.no_grad():
            heat = heatmap.sigmoid()
            decoded = self.decoder.decode(heat, rot[:, :1], rot[:, 1:2], height,
                                           dim.exp(), vel, reg=reg)[0]
        # Preview reports official pre-NMS candidates explicitly; no substitute NMS
        # is presented as validated equivalence to Horizon's rotated-NMS extension.
        rows = [dict(class_name=CLASSES[int(c)], score=float(s),
                     x_ground=float(b[0]), y_ground=float(b[1]), box=b.tolist())
                for b, s, c in zip(decoded['bboxes'], decoded['scores'], decoded['labels'])]
        return rows, heat.numpy()[0]


def image_planes(bgr):
    rgb = torch.from_numpy(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)).permute(2, 0, 1)
    # HEAL 0.6.0 targets torchvision with antialias=True by default.
    rgb = TVF.resize(rgb, [396, 704], antialias=True)[:, 140:, :]
    image = rgb.permute(1, 2, 0).contiguous().numpy()
    i420 = cv2.cvtColor(image, cv2.COLOR_RGB2YUV_I420).reshape(-1)
    y = i420[:256*704].reshape(1, 256, 704, 1)
    uv = i420[256*704:].reshape(2, -1).T.copy().reshape(1, 128, 352, 2)
    return [y, uv], image


def quantize_grids(grids, specs):
    result = []
    for p, spec in zip(grids, specs):
        if (spec['type'] != 5 or spec['quant'] != 1 or len(spec['scales']) != 1
                or spec['zero_points'] != [0] or list(p.shape) != spec['shape']):
            raise ValueError('Unsupported official sampling ABI')
        # Saturation is essential: wrapping the invalid 65535 sentinel can select
        # actual image features. Saturated coordinates remain outside both sources.
        q = np.clip(np.rint(p/spec['scales'][0]), -32768, 32767).astype(np.int16)
        result.append(q)
    return result


def prepare_images(images):
    inputs, crops = [], {}
    for name in ORDER:
        planes, crops[name] = image_planes(images[name])
        inputs.extend(planes)
    for _ in range(2):
        planes, _ = image_planes(np.full((256, 704, 3), 128, dtype=np.uint8))
        inputs.extend(planes)
    return inputs, crops
