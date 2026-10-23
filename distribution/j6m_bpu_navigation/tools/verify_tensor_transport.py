#!/usr/bin/env python3
"""Exercise a real SSH/BPU round trip using the pinned reference FCOS only."""
import argparse
import json
from pathlib import Path
import time

import numpy as np

from production_ptq import write_json
from tensor_transport import TensorClient, ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target', default='root@192.168.10.100')
    parser.add_argument('--library', required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    if args.report.exists():
        raise FileExistsError('Preserve prior round-trip evidence')
    records = []
    with TensorClient(args.target, ROOT + '/bpu_production_20261010/tools/tensor_stdio.py',
                      args.library, ROOT + '/bpu_lab_20260905/models/fcos_nash_m/model.hbm',
                      'fcos_efficientnetb3_mscoco',
                      '1daacc7dfde181b65872c47f4fe5db84fd31f19d3ad56aa1fa1d902bb5ffd7fa',
                      timeout=8) as client:
        metadata, load_ms = client.metadata, client.load_ms
        for index in range(3):
            tensors = [np.full((1, 896, 896, 1), 16 + index * 40, dtype=np.uint8),
                       np.full((1, 448, 448, 2), 128, dtype=np.uint8)]
            started = time.monotonic()
            outputs, timing = client.infer(tensors, stamp_ns=time.time_ns(), frame_id='synthetic_nv12',
                                           max_age_ms=500)
            if len(outputs) != 15 or not all(np.isfinite(value).all() for value in outputs):
                raise RuntimeError('Invalid FCOS outputs across the tensor bridge')
            records.append({'sequence': index + 1, 'source_identity_verified': True,
                            'result_within_500ms': True, 'outputs': len(outputs),
                            'round_trip_ms': (time.monotonic() - started) * 1000, 'timing': timing})
    report = {'metadata': metadata, 'load_ms': load_ms, 'frames': records, 'worker_closed': True,
              'synthetic_inputs': True, 'formal_trash_bpu_migrated': False,
              'production_ready': False, 'motion_eligible': False}
    write_json(args.report, report)
    print(json.dumps(report, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
