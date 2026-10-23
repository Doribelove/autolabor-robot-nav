#!/usr/bin/env python3
"""Private observation-only tensor worker; no ROS or control publishers."""
import argparse
import fcntl
import os
from pathlib import Path
import sys

REFERENCE = Path('/map/robot_j6m_navigation_20260907/bpu_lab_20260905')
RUNTIME = str(REFERENCE / 'vendor/runtime_4_9_2/lib') + ':/usr/hobot/lib'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', type=Path, required=True)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--model-name', required=True)
    parser.add_argument('--sha256', required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    if root != Path('/map/robot_j6m_navigation_20260907/bpu_production_20261010'):
        raise RuntimeError('Use the isolated J6M production-adaptation directory')
    if root not in args.library.resolve(strict=True).parents or args.library.is_symlink():
        raise ValueError('Executor library must be a regular file in this candidate')
    if os.environ.get('LD_LIBRARY_PATH') != RUNTIME:
        os.execve(sys.executable, [sys.executable, '-u'] + sys.argv,
                  dict(os.environ, LD_LIBRARY_PATH=RUNTIME, PYTHONDONTWRITEBYTECODE='1',
                       OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1'))
    sys.path.insert(0, str(REFERENCE / 'vendor/python311'))
    from native_tensor import TensorEngine
    from tensor_wire import send, receive, identity, unpack_arrays, pack_arrays
    # Vendor C stdout must never mix with the binary protocol.
    wire = os.fdopen(os.dup(sys.stdout.fileno()), 'wb', buffering=0)
    os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
    with (REFERENCE / 'preview_worker.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with TensorEngine(args.library, args.model, args.model_name, args.sha256) as engine:
            send(wire, {'ready': True, 'model_sha256': engine.model_sha256,
                        'metadata': engine.metadata, 'load_ms': engine.load_ms,
                        'production_ready': False, 'motion_eligible': False})
            previous = 0
            while True:
                try:
                    header, payload = receive(sys.stdin.buffer, timeout=60)
                except EOFError:
                    return
                source = identity(header)
                if header.get('op') != 'infer' or source['sequence'] <= previous:
                    raise ValueError('Invalid/replayed inference request')
                previous = source['sequence']
                timeout = header.get('timeout_ms')
                if type(timeout) is not int or not 1 <= timeout <= 8000:
                    raise ValueError('Invalid device deadline')
                tensors = unpack_arrays(header.get('tensors'), payload, engine.metadata['inputs'])
                outputs, timing = engine.infer(tensors, timeout_ms=timeout)
                descriptors, raw = pack_arrays(outputs)
                send(wire, dict(source, ok=True, model_sha256=engine.model_sha256,
                                production_ready=False, motion_eligible=False,
                                tensors=descriptors, timing=timing), raw)


if __name__ == '__main__':
    main()
