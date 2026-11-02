#!/usr/bin/env python3
"""Resident J6M image detector: JPEG in, original-image trash boxes out."""
import fcntl
import hashlib
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


def main():
    runtime = str(VENDOR/'runtime_4_9_2/lib')+':/usr/hobot/lib'
    if os.environ.get('LD_LIBRARY_PATH') != runtime:
        os.execve(sys.executable, [sys.executable, '-B', *sys.argv],
                  dict(os.environ, LD_LIBRARY_PATH=runtime, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1'))
    # Keep protocol output separate from vendor C stdout.
    wire = os.fdopen(os.dup(sys.stdout.fileno()), 'wb', buffering=0)
    os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
    from tensor_wire import send, receive
    sys.path[:0] = [str(MODELS/'tools'), str(MODELS/'deps'), str(VENDOR/'python311')]
    import cv2
    import numpy as np
    from native_tensor import TensorEngine, sha256
    from board_preprocess import detector_input
    from production_io import detector_output, validate_production_interface
    cv2.setNumThreads(1)
    if sha256(LIBRARY) != LIBRARY_SHA:
        raise ValueError('Unexpected resident executor identity')
    checkpoint = json.loads((MODELS/'production_models.json').read_text())['models']['detector_zed']['checkpoint_sha256']
    with (VENDOR.parent/'preview_worker.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with TensorEngine(LIBRARY, MODELS/'models/detector_zed_attempt03.hbm', 'detector_zed', MODEL_SHA) as engine:
            validate_production_interface(engine.metadata, 'detector_zed')
            tensor, _ = detector_input(np.zeros((540,960,3), np.uint8))
            for _ in range(3):
                engine.infer([tensor], timeout_ms=1000)
            send(wire, dict(ready=True, model_sha256=MODEL_SHA, checkpoint_sha256=checkpoint,
                            metadata=engine.metadata, backend='BPU'))
            while True:
                try:
                    header, payload = receive(sys.stdin.buffer, timeout=60)
                except EOFError:
                    return
                rows = header.get('views', [])
                if not 1 <= len(rows) <= 4 or len({r['camera'] for r in rows}) != len(rows):
                    raise ValueError('Invalid camera set')
                confidence = float(header['confidence'])
                if not 0 <= confidence <= 1:
                    raise ValueError('Invalid confidence')
                cursor, results = 0, {}
                for row in rows:
                    count = row['bytes']
                    if type(count) is not int or not 0 < count <= 2*1024*1024 or cursor+count > len(payload):
                        raise ValueError('Invalid JPEG size')
                    jpeg = payload[cursor:cursor+count]; cursor += count
                    if hashlib.sha256(jpeg).hexdigest() != row['sha256']:
                        raise ValueError('JPEG identity mismatch')
                    image = cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
                    if image is None or list(image.shape[1::-1]) != row['size']:
                        raise ValueError('Image dimensions mismatch')
                    begin = time.monotonic()
                    tensor, geometry = detector_input(image)
                    raw, timing = engine.infer([tensor], timeout_ms=1000)
                    boxes = detector_output(raw[0], geometry, confidence=confidence, iou=.45, max_detections=25)
                    results[row['camera']] = dict(boxes=boxes.tolist(), timing=timing,
                                                  image_pipeline_ms=(time.monotonic()-begin)*1000)
                if cursor != len(payload):
                    raise ValueError('Trailing input bytes')
                send(wire, dict(source_id=header['source_id'], model_sha256=MODEL_SHA, results=results))


if __name__ == '__main__':
    main()
