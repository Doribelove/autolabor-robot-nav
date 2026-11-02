#!/usr/bin/env python3
"""Compare fusion on one unchanged four-view snapshot; no camera/control access."""
import argparse
import copy
import hashlib
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
    parser.add_argument('--snapshot', type=Path, required=True,
                        help='Directory containing before_config.json and front/left/right/rear.png')
    args = parser.parse_args()
    cv2.setNumThreads(1)
    config = json.loads((args.snapshot/'before_config.json').read_text())
    config['blend_mode'] = 'weighted'
    before = GroundBev(config)
    selected_config = copy.deepcopy(config)
    selected_config['blend_mode'] = 'single_source'
    after = GroundBev(selected_config)
    frames = {n: cv2.imread(str(args.snapshot/(n+'.png'))) for n in before.cameras}
    images, timings = [], {}
    for name, bev in [('weighted', before), ('single_source', after)]:
        times = []
        for i in range(16):
            begin = time.monotonic()
            image, support, layers = bev.compose(frames)
            if i > 0: times.append((time.monotonic()-begin)*1000)
        timings[name] = float(np.median(times))
        cv2.imwrite(str(args.snapshot/(name+'.png')), image)
        preview = cv2.copyMakeBorder(image, 30, 0, 0, 0, cv2.BORDER_CONSTANT)
        cv2.putText(preview, name, (12, 22), cv2.FONT_HERSHEY_SIMPLEX, .6, (255,255,255), 1)
        images.append(preview)
    cv2.imwrite(str(args.snapshot/'comparison.jpg'), cv2.hconcat(images))
    owner = after.source_owner
    colors = np.array([[70,160,240],[130,220,80],[240,120,80],[160,70,220]], np.uint8)
    display = np.zeros_like(images[0][30:])
    for index, name in enumerate(after.source_names):
        display[owner == index] = colors[index]
    cv2.imwrite(str(args.snapshot/'source_owner.png'), display)
    assert np.array_equal(before.support_counts, after.support_counts)
    assert all(np.array_equal(before.maps[n][axis], after.maps[n][axis])
               for n in frames for axis in (0,1))
    for index, name in enumerate(after.source_names):
        assert np.array_equal(image[owner == index], layers[name][owner == index])
    report = dict(
        same_images=True,
        image_sha256={n:hashlib.sha256(im.tobytes()).hexdigest() for n,im in frames.items()},
        ground_mapping_unchanged=True, geometric_support_unchanged=True,
        geometric_overlap_pixels=int((support>1).sum()),
        mixed_camera_pixels_after=0,
        source_owner_names=after.source_names,
        selected_pixels={n:int((owner==i).sum()) for i,n in enumerate(after.source_names)},
        projection_median_ms=timings,
        actual_metric_accuracy_verified=False,
        note='Removes alpha double exposure; non-ground perspective stretch and calibration errors remain.')
    (args.snapshot/'comparison.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
