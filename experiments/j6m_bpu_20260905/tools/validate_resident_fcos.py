#!/usr/bin/env python3
"""J6M resident-vs-vendor tensor equivalence; run with the private runtime env."""
import os
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'
import fcntl
import hashlib
import json
from pathlib import Path
import sys
import time

LAB = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(LAB / 'vendor/python311'))
import numpy as np
from resident_fcos import ResidentFCOS


def main():
    if LAB != Path('/map/robot_j6m_optimized_20260905/bpu_lab_20260905'):
        raise RuntimeError('Run only in the private J6M lab')
    model = LAB / 'models/fcos_nash_m/model.hbm'
    if hashlib.sha256(model.read_bytes()).hexdigest() != '1daacc7dfde181b65872c47f4fe5db84fd31f19d3ad56aa1fa1d902bb5ffd7fa':
        raise RuntimeError('Reference model hash changed')
    with (LAB / 'preview_worker.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        report = []
        with ResidentFCOS(LAB, model) as engine:
            for repeat in range(6):
                for case in ('bus', 'zed_saved'):
                    inputs = LAB / 'golden_20260906/inputs' / case
                    payload = (inputs / 'y.bin').read_bytes() + (inputs / 'uv.bin').read_bytes()
                    started = time.monotonic()
                    outputs, timing = engine.infer(payload)
                    worst = 0.0
                    for i, output in enumerate(outputs):
                        path = LAB / 'golden_20260906/results' / ('fcos_' + case) / 'dump' / (
                            'model_infer_output_%d__output_%d.bin' % (i, i))
                        reference = np.fromfile(str(path), dtype='<f4').reshape(output.shape)
                        np.testing.assert_array_equal(output, reference)
                        worst = max(worst, float(np.max(np.abs(output - reference))))
                    report.append(dict(repeat=repeat, case=case, tensor_count=15,
                                       maximum_absolute_difference=worst, timing=timing,
                                       test_ms=(time.monotonic() - started) * 1000))
        result = dict(passed=True, comparisons=180, exact_float32_equality=True, frames=report)
        (LAB / 'golden_20260906/resident_equivalence.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
