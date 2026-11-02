#!/usr/bin/env python3
"""Read-only dataset/toolchain checks. Does not invoke an invented HEAL API."""
import argparse
import importlib.metadata
import json
import platform
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'src/application/autolabor_fod_vision/src'))

def main():
    p = argparse.ArgumentParser()
    p.add_argument('mode', choices=['toolchain','data','bag'])
    p.add_argument('--heal-root')
    p.add_argument('--dataset')
    p.add_argument('--bag')
    a = p.parse_args()
    errors = []
    result = dict(host_arch=platform.machine(), python=sys.executable, mode=a.mode)
    if a.mode == 'toolchain':
        if platform.machine() != 'x86_64': errors.append('training/quantization/compiler host must be x86_64 Linux')
        required = {'heal':'0.0.2','hbdk4-compiler':'4.11.11','horizon-plugin-pytorch':'3.3.10'}
        result['packages'] = {}
        for name, want in required.items():
            try: have = importlib.metadata.version(name)
            except importlib.metadata.PackageNotFoundError: have = None
            result['packages'][name] = dict(required=want, installed=have)
            # Official plugin wheels include a PEP 440 local CUDA/Torch build tag,
            # e.g. 3.3.10+cu128.torch280. Keep the tag in the report.
            if have is None or have.split('+', 1)[0] != want:
                errors.append(name+' matching version missing from this interpreter')
        root = Path(a.heal_root) if a.heal_root else None
        for name in ['configs/bev_vtv2_resnet50_nuscenes','heal/models/task_modules/bev/view_transformer.py']:
            if root is None or not (root/name).exists(): errors.append('source missing: '+name)
        result['compile_executed'] = False
        result['note'] = 'Availability check only; four-view HEAL port and measured calibration dataset still required'
    elif a.mode == 'data':
        from autolabor_fod_vision.bev_feature.data import validate, images, targets
        from autolabor_fod_vision.bev_feature.geometry import DEFAULT, sampling
        if not a.dataset: p.error('--dataset required')
        path = Path(a.dataset)
        count = 0
        for line, text in enumerate(path.read_text().splitlines(), 1):
            if not text.strip(): continue
            count += 1
            try:
                sample = json.loads(text); validate(sample, path.parent)
                rig = json.loads((path.parent/sample['calibration']).read_text())
                sampling(rig, DEFAULT, sample['rig_to_base_xyyaw'])
                images(sample, path.parent, rig, DEFAULT); targets(sample['objects'], DEFAULT)
            except (ValueError, KeyError, TypeError, OSError) as e:
                errors.append('line %d: %s' % (line, e))
        result['samples'] = count
        if not count: errors.append('empty dataset')
    else:
        import rosbag
        if not a.bag: p.error('--bag required')
        with rosbag.Bag(a.bag) as bag:
            topics = bag.get_type_and_topic_info().topics
            result['topics'] = {k:dict(type=v.msg_type, count=v.message_count) for k,v in topics.items()}
            result['images'] = [k for k,v in topics.items() if v.msg_type in ('sensor_msgs/Image','sensor_msgs/CompressedImage')]
            result['camera_info'] = [k for k,v in topics.items() if v.msg_type == 'sensor_msgs/CameraInfo']
            result['tf'] = [k for k,v in topics.items() if v.msg_type == 'tf2_msgs/TFMessage']
        if len(result['images']) < 4: errors.append('no four-camera training/replay images in this bag')
    result['errors'] = errors
    print(json.dumps(result, indent=2))
    return 2 if errors else 0

if __name__ == '__main__':
    sys.exit(main())
