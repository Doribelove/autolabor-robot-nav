#!/usr/bin/env python3
"""J6M resident FCOS: four original RGB JPEGs in, COCO80 boxes out; no depth."""
import fcntl
import hashlib
import os
from pathlib import Path
import sys
import time

LAB = Path('/map/robot_j6m_navigation_20260907/bpu_lab_20260905')
MODEL = LAB/'models/fcos_nash_m/model.hbm'
MODEL_SHA = '1daacc7dfde181b65872c47f4fe5db84fd31f19d3ad56aa1fa1d902bb5ffd7fa'
LIBRARY_SHA = '3f3534566e73f19bf10963131c8cbff206aa7a6f98ff1f65dd45b2b13e4fa8aa'


def main():
    runtime = str(LAB/'vendor/runtime_4_9_2/lib')+':/usr/hobot/lib'
    if os.environ.get('LD_LIBRARY_PATH') != runtime:
        os.execve(sys.executable, [sys.executable, '-B', *sys.argv],
                  dict(os.environ, LD_LIBRARY_PATH=runtime, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1'))
    wire = os.fdopen(os.dup(sys.stdout.fileno()), 'wb', buffering=0)
    os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
    from tensor_wire import send, receive
    sys.path.append(str(LAB/'vendor/python311'))
    sys.path.append('/map/j6m-dev-b/vision-bpu/conversion_runs/20261019_j4liv6zx/deps')
    import cv2
    import numpy as np
    from resident_fcos import ResidentFCOS
    from decode_fcos_demo import decode
    from fcos_image import prepare_rgb, source_boxes
    cv2.setNumThreads(1)
    for path, expected in ((MODEL, MODEL_SHA), (LAB/'bin/libfcos_resident.so', LIBRARY_SHA)):
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError('Unexpected FCOS model/runtime identity: '+str(path))
    with (LAB/'preview_worker.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with ResidentFCOS(LAB, MODEL) as engine:
            blank, _ = prepare_rgb(np.zeros((540,960,3), np.uint8))
            for _ in range(3):
                engine.infer(blank)
            metadata = dict(model_name='fcos_efficientnetb3_mscoco', library_sha256=LIBRARY_SHA,
                input_shapes=[[1,896,896,1],[1,448,448,2]], input_format='NV12', output_tensors=15,
                task_core='BPU_CORE_0', runtime='DNN 3.14.5 / private runtime_4_9_2',
                preprocessing='aspect resize, top-left black BGR pad, OpenCV I420 to NV12',
                decoder='sigmoid(class)*sigmoid(centerness), LTRB*stride, classwise NMS 0.6',
                exact_HEAL_decoder_verified=False, training_preprocess_equivalence_verified=False,
                depth_used=False)
            # No verified floating checkpoint identity exists for this installed HBM.
            send(wire, dict(ready=True, model_sha256=MODEL_SHA, checkpoint_sha256=None,
                            metadata=metadata, backend='BPU'))
            while True:
                try:
                    header, payload = receive(sys.stdin.buffer, timeout=60)
                except EOFError:
                    return
                rows = header.get('views', [])
                if not 1 <= len(rows) <= 4 or len({r['camera'] for r in rows}) != len(rows):
                    raise ValueError('Invalid camera set')
                confidence = float(header['confidence'])
                if not 0 < confidence < 1:
                    raise ValueError('Invalid confidence')
                cursor, results = 0, {}
                for row in rows:
                    begin = time.monotonic()
                    count = row['bytes']
                    if type(count) is not int or not 0 < count <= 2*1024*1024 or cursor+count > len(payload):
                        raise ValueError('Invalid JPEG size')
                    jpeg = payload[cursor:cursor+count]; cursor += count
                    if hashlib.sha256(jpeg).hexdigest() != row['sha256']:
                        raise ValueError('JPEG identity mismatch')
                    image = cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
                    if image is None or list(image.shape[1::-1]) != row['size']:
                        raise ValueError('Image dimensions mismatch')
                    nv12, geometry = prepare_rgb(image)
                    prepared = time.monotonic()
                    raw, timing = engine.infer(nv12)
                    inferred = time.monotonic()
                    boxes = source_boxes(decode(raw, threshold=confidence, nms_iou=.6, maximum=100), geometry)
                    timing.update(preprocess_ms=(prepared-begin)*1000,
                        decode_ms=(time.monotonic()-inferred)*1000, image_pipeline_ms=(time.monotonic()-begin)*1000)
                    results[row['camera']] = dict(boxes=boxes, timing=timing)
                if cursor != len(payload):
                    raise ValueError('Trailing input bytes')
                send(wire, dict(source_id=header['source_id'], model_sha256=MODEL_SHA, results=results))


if __name__ == '__main__':
    main()
