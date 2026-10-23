#!/usr/bin/env python3
"""Board-only executor comparison on synthetic NV12; not a perception accuracy test."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path('/map/robot_j6m_navigation_20260907')
REFERENCE = ROOT / 'bpu_lab_20260905'
CANDIDATE = ROOT / 'bpu_production_20261010'
RUNTIME = str(REFERENCE / 'vendor/runtime_4_9_2/lib') + ':/usr/hobot/lib'
SHA = '1daacc7dfde181b65872c47f4fe5db84fd31f19d3ad56aa1fa1d902bb5ffd7fa'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('LD_LIBRARY_PATH') != RUNTIME:
        os.execve(sys.executable, [sys.executable, '-u'] + sys.argv,
                  dict(os.environ, LD_LIBRARY_PATH=RUNTIME, PYTHONDONTWRITEBYTECODE='1',
                       OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1'))
    sys.path[:0] = [str(REFERENCE / 'vendor/python311'), str(REFERENCE / 'tools')]
    import numpy as np
    from native_tensor import TensorEngine
    from resident_fcos import ResidentFCOS
    model = REFERENCE / 'models/fcos_nash_m/model.hbm'
    results = []
    with (REFERENCE / 'preview_worker.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with TensorEngine(args.library, model,
                          'fcos_efficientnetb3_mscoco', SHA) as engine:
            metadata = engine.metadata
            with ResidentFCOS(REFERENCE, model) as old:
                for index in range(3):
                    y = np.full((1, 896, 896, 1), 16, dtype=np.uint8)
                    if index:
                        ramp = np.arange(896, dtype=np.uint16) % 220 + 16
                        y[0, :, :, 0] = ramp[None, :] if index == 1 else ramp[:, None]
                    uv = np.full((1, 448, 448, 2), 128, dtype=np.uint8)
                    outputs, timing = engine.infer([y, uv])
                    expected, old_timing = old.infer(y.tobytes() + uv.tobytes())
                    errors = [float(np.max(np.abs(a[0] - b))) for a, b in zip(outputs, expected)]
                    matched = all(np.allclose(a[0], b, rtol=1e-6, atol=1e-6)
                                  for a, b in zip(outputs, expected))
                    if not matched:
                        raise RuntimeError('Generic executor differs from validated FCOS adapter: ' + str(errors))
                    results.append({'fixture': index, 'matched': matched,
                                    'max_abs_error': max(errors), 'timing': timing,
                                    'old_timing': old_timing})
            owned = [value.copy() for value in outputs]
            # A new inference must not overwrite previously returned host arrays.
            engine.infer([y, uv])
            if not all(np.array_equal(a, b) for a, b in zip(outputs, owned)):
                raise RuntimeError('Output ownership regression')
    report = {'reference_model_sha256': SHA, 'metadata': metadata, 'comparisons': results,
              'output_ownership_passed': True, 'synthetic_inputs': True,
              'formal_trash_bpu_migrated': False, 'production_ready': False,
              'motion_eligible': False}
    print('J6M_REFERENCE_RESULT ' + json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
