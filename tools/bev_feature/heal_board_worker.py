#!/usr/bin/env python3
"""Private OE3.9.1 resident HBM worker; reuses existing tensor executor/protocol."""
import argparse
import fcntl
import os
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime', type=Path, required=True)
    parser.add_argument('--sha256', required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    runtime = str(args.runtime/'lib')+':/usr/hobot/lib'
    if os.environ.get('LD_LIBRARY_PATH') != runtime:
        os.execve(sys.executable, [sys.executable, '-u', '-B', *sys.argv],
                  dict(os.environ, LD_LIBRARY_PATH=runtime,
                       OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1'))
    sys.path.insert(0, '/map/robot_j6m_navigation_20260907/bpu_lab_20260905/vendor/python311')
    from native_tensor import TensorEngine
    from tensor_wire import send, receive, unpack_arrays, pack_arrays
    wire = os.fdopen(os.dup(sys.stdout.fileno()), 'wb', buffering=0)
    os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
    with open('/map/robot_j6m_navigation_20260907/bpu_lab_20260905/preview_worker.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with TensorEngine(root/'libj6m_tensor.so', args.runtime/'model.hbm',
                          'bev_vtv2_resnet50_nuscenes', args.sha256) as engine:
            send(wire, dict(ready=True, metadata=engine.metadata, model_sha256=engine.model_sha256))
            while True:
                try:
                    header, payload = receive(sys.stdin.buffer, timeout=60)
                except EOFError:
                    return
                tensors = unpack_arrays(header['tensors'], payload, engine.metadata['inputs'])
                outputs, timing = engine.infer(tensors, timeout_ms=2000)
                descriptors, raw = pack_arrays(outputs)
                send(wire, dict(sequence=header['sequence'], tensors=descriptors, timing=timing), raw)


if __name__ == '__main__':
    main()
