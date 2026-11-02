#!/usr/bin/env python3
"""Interleave old/new projection on identical images; check blind areas and pixels."""
import argparse
import importlib.util
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'distribution/j6m_bpu_navigation'))
from surround.ground_bev import GroundBev


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference-module', type=Path, required=True)
    parser.add_argument('--images', type=Path, required=True,
                        help='Directory containing front/left/right/rear.png, already upright')
    parser.add_argument('--config', type=Path, default=ROOT/'config/four_camera_bev.json')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location('reference_ground_bev', args.reference_module)
    reference = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(reference)
    config = json.loads(args.config.read_text())
    old, new = reference.GroundBev(config), GroundBev(config)
    frames = {name: cv2.imread(str(args.images/(name+'.png'))) for name in config['cameras']}
    if any(frame is None for frame in frames.values()):
        raise ValueError('Missing upright source image')
    cv2.setNumThreads(1)
    samples = {'before': [], 'after': []}
    for iteration in range(60):
        for name, bev in [('before', old), ('after', new)]:
            begin = time.perf_counter()
            bev.compose(frames)
            elapsed = (time.perf_counter()-begin)*1000
            if iteration >= 10:
                samples[name].append(elapsed)
    image_old, counts_old, layers_old = old.compose(frames)
    image_new, counts_new, layers_new = new.compose(frames)
    difference = np.abs(image_old.astype(int)-image_new.astype(int))
    report = {name: dict(p50=float(np.median(values)), p95=float(np.percentile(values, 95)))
              for name, values in samples.items()}
    report.update(max_pixel_change=int(difference.max()), mean_pixel_change=float(difference.mean()),
                  support_equal=bool(np.array_equal(counts_old, counts_new)),
                  layers_equal=all(np.array_equal(layers_old[n], layers_new[n]) for n in layers_old))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    if report['max_pixel_change'] > 1 or not report['support_equal'] or not report['layers_equal']:
        raise SystemExit('Projection changed beyond expected floating-point rounding')


if __name__ == '__main__':
    main()
