#!/usr/bin/env python3
"""J6M-only replay of pixel BEV through the existing, older trash HBM.

This is a model reuse diagnostic, NOT four-camera learned feature fusion.
Uses the deployed executor/SDK read-only and never publishes ROS observations.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import sys
import time

VENDOR = Path('/map/robot_j6m_navigation_20260907/bpu_lab_20260905/vendor')
MODELS = Path('/map/j6m-dev-b/vision-bpu/conversion_runs/20261019_j4liv6zx')
LIBRARY = Path('/map/robot_j6m_navigation_20260907/bpu_production_20261010/build_v5/libj6m_tensor.so')
LIBRARY_SHA = '5c99bafb0531fac9f8e84c49c8c2d0a6d8e852e5c2217e62750dce93b4a14e3d'
MODEL_SHA = 'f2f1f7e9753b53b0494495be44af92a0603b3ee6c71e69e0f58a0d50544a8cb8'
CURRENT_CHECKPOINT_SHA = '5efaafa1503db11c2ba261b4429389d96335b4eef4d0fc44d6ca41e7431f2d0f'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    runtime = str(VENDOR/'runtime_4_9_2/lib')+':/usr/hobot/lib'
    if os.environ.get('LD_LIBRARY_PATH') != runtime:
        os.execve(sys.executable, [sys.executable, '-B', *sys.argv],
                  dict(os.environ, LD_LIBRARY_PATH=runtime, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1'))
    sys.path[:0] = [str(MODELS/'tools'), str(MODELS/'deps'), str(VENDOR/'python311')]
    import cv2
    import numpy as np
    from native_tensor import TensorEngine, sha256
    from board_preprocess import detector_input
    from production_io import detector_output, validate_production_interface
    cv2.setNumThreads(1)
    if sha256(LIBRARY) != LIBRARY_SHA:
        raise ValueError('Deployed executor does not match the verified build')
    manifest = json.loads((args.directory/'samples.json').read_text())
    contract = json.loads((MODELS/'production_models.json').read_text())
    source_checkpoint = contract['models']['detector_zed']['checkpoint_sha256']
    timings, results = [], []
    with (VENDOR.parent/'preview_worker.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with TensorEngine(LIBRARY, MODELS/'models/detector_zed_attempt03.hbm',
                          'detector_zed', MODEL_SHA) as engine:
            validate_production_interface(engine.metadata, 'detector_zed')
            for sample in manifest:
                path = args.directory/sample['file']
                if path.parent != args.directory or sha256(path) != sample['jpeg_sha256']:
                    raise ValueError('Replay image hash/path mismatch')
                image = cv2.imread(str(path))
                tensor, geometry = detector_input(image)
                if not timings:
                    for _ in range(3):
                        engine.infer([tensor], timeout_ms=1000)
                begin = time.monotonic()
                raw, timing = engine.infer([tensor], timeout_ms=1000)
                boxes = detector_output(raw[0], geometry, confidence=.25, iou=.45)
                timing = dict(timing, infer_decode_ms=(time.monotonic()-begin)*1000)
                timings.append(timing)
                for x1, y1, x2, y2, confidence in boxes:
                    cv2.rectangle(image, (round(x1), round(y1)), (round(x2), round(y2)), (0, 255, 255), 2)
                    cv2.putText(image, 'trash %.2f'%confidence, (round(x1), max(12, round(y1)-4)),
                                cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 255, 255), 1)
                cv2.imwrite(str(args.directory/(path.stem+'_bpu.jpg')), image)
                results.append(dict(source_id=sample['source_id'], file=sample['file'],
                                    boxes=boxes.tolist(), timing=timing))
            report = dict(scope='older 2D trash detector on pixel BEV; NOT learned multi-camera fusion',
                          model_sha256=MODEL_SHA, source_checkpoint_sha256=source_checkpoint,
                          current_cuda_checkpoint_sha256=CURRENT_CHECKPOINT_SHA,
                          same_checkpoint_as_current_cuda=source_checkpoint == CURRENT_CHECKPOINT_SHA,
                          runtime='DNN 3.14.5 / HBRT 4.9.2', metadata=engine.metadata,
                          samples=len(results), warmup=3,
                          vendor_infer_ms={key: float(v) for key, v in zip(('p50', 'p95', 'max'),
                              np.percentile([t['vendor_infer_ms'] for t in timings], [50, 95, 100]))},
                          capture_to_map_latency_ms=None, bev_detection_accuracy_verified=False,
                          results=results)
    (args.directory/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ('metadata', 'results')}))


if __name__ == '__main__':
    main()
