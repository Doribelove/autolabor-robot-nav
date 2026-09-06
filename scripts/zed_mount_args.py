#!/usr/bin/env python3
"""Print launch arguments from the single measured left-lens calibration."""
from pathlib import Path
import sys
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/application/autolabor_bpu_perception/src'))
from autolabor_bpu_perception.core import zed_mount

mount=zed_mount(yaml.safe_load((Path(__file__).resolve().parents[1]/
    'src/application/autolabor_bpu_perception/config/zed_mount.yaml').read_text()))
for name,value in zip(('cam_pos_x','cam_pos_y','cam_pos_z','cam_roll','cam_pitch','cam_yaw'),
                      list(mount['base_position'])+list(mount['base_rpy'])):
    print('%s:=%.12f'%(name,value))
