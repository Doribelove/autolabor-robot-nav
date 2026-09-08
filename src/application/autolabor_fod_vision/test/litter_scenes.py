"""Ray-cast fixtures: synthetic geometry, not measured recognition accuracy."""
from pathlib import Path
import numpy as np
import yaml
from autolabor_bpu_perception.core import zed_mount


def scene(objects=(), width=640, height=360, slope=0.):
    config = Path(__file__).resolve().parents[2] / 'autolabor_bpu_perception/config/zed_mount.yaml'
    mount = zed_mount(yaml.safe_load(config.read_text()))
    k = [250*width/640, 0, width/2, 0, 250*height/360, height/2, 0, 0, 1]
    v, u = np.mgrid[:height, :width]
    rays = np.stack(((u-k[2])/k[0], (v-k[5])/k[4], np.ones_like(u)), axis=-1)
    rays = rays @ mount['optical_to_base'].T
    origin = mount['left_position']
    # Floor z = slope*x, plus a wall at x=7 m.
    denominator = rays[..., 2]-slope*rays[..., 0]
    with np.errstate(divide='ignore', invalid='ignore'):
        ground = (slope*origin[0]-origin[2])/denominator
        wall = (7-origin[0])/rays[..., 0]
    ground = np.where(ground > .3, ground, np.inf)
    wall = np.where(wall > .3, wall, np.inf)
    depth = np.minimum(ground, wall)
    visible = []
    for low, high, class_name in objects:
        with np.errstate(divide='ignore', invalid='ignore'):
            first = (np.asarray(low)-origin)/rays
            second = (np.asarray(high)-origin)/rays
        enter = np.max(np.minimum(first, second), axis=-1)
        leave = np.min(np.maximum(first, second), axis=-1)
        mask = (enter > .3) & (leave >= enter) & (enter < depth)
        depth[mask] = enter[mask]
        rows, columns = np.nonzero(mask)
        if not len(rows):
            raise ValueError('Synthetic object outside camera view')
        class_id = {'bottle':39, 'suitcase':28, 'potted plant':58, 'paper':73}[class_name]
        visible.append(dict(bbox=[float(columns.min()),float(rows.min()),float(columns.max()+1),float(rows.max()+1)],
                            class_id=class_id,class_name=class_name,confidence=.7,object_id=len(visible)+1))
    return depth.astype(np.float32), k, mount, visible


BOTTLE = ([1.8, -.08, .0], [1.9, .02, .28], 'suitcase')
PLANT = ([1.7, -.6, .0], [2.05, -.25, .80], 'potted plant')
