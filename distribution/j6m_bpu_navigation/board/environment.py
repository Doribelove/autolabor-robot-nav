#!/usr/bin/env python3
"""Inspect and select the private YOLO11 preparation bundle; never start inference."""
import argparse
import ctypes
import hashlib
import json
import os
from pathlib import Path
import platform
import sys

BASE = Path('/map/j6m-dev-b/vision-bpu')
REFERENCE = Path('/map/robot_j6m_navigation_20260907/bpu_lab_20260905')
RELEASE = Path(__file__).resolve().parent


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def status():
    metadata = json.loads((RELEASE / 'DEPLOYMENT.json').read_text())
    current = BASE / 'current'
    return {
        'release': str(RELEASE),
        'selected': current.is_symlink() and current.resolve() == RELEASE,
        'stage': 'onnx_and_board_environment_prepared',
        'models': metadata['models'],
        'formal_hbm_present': False,
        'production_ready': False,
        'motion_eligible': False,
        'inference_service_started_by_this_tool': False,
        'remaining': metadata['remaining'],
    }


def verify():
    manifest = json.loads((RELEASE / 'FILES.json').read_text())
    for item in manifest['files']:
        path = RELEASE / item['path']
        if (Path(item['path']).is_absolute() or '..' in Path(item['path']).parts
                or path.is_symlink() or RELEASE not in path.resolve().parents
                or path.stat().st_size != item['bytes'] or digest(path) != item['sha256']):
            raise ValueError('Candidate file mismatch: ' + str(path))
    dependencies = json.loads((RELEASE / 'DEPENDENCIES.json').read_text())
    for item in dependencies['files']:
        path = Path(item['path'])
        if path.stat().st_size != item['bytes'] or digest(path) != item['sha256']:
            raise ValueError('Pinned board dependency mismatch: ' + str(path))
    return dict(status(), candidate_files_verified=len(manifest['files']),
                dependency_files_verified=len(dependencies['files']))


def environment():
    return dict(os.environ,
                LD_LIBRARY_PATH=str(REFERENCE / 'vendor/runtime_4_9_2/lib') + ':/usr/hobot/lib',
                PYTHONPATH=str(RELEASE / 'tools') + ':' + str(REFERENCE / 'vendor/python311'),
                PYTHONDONTWRITEBYTECODE='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')


def self_test():
    result = verify()
    import numpy as np
    from production_ptq import verify_workspace, validate_tensor, contract
    verify_workspace(RELEASE / 'ptq', require_calibration=False)
    for task, spec in contract()['models'].items():
        validate_tensor(RELEASE / 'samples' / (task + '.npy'), spec)
    dependencies = json.loads((RELEASE / 'DEPENDENCIES.json').read_text())
    library = ctypes.CDLL(dependencies['executor'])
    for name in ['create', 'metadata', 'input', 'infer', 'output', 'destroy']:
        getattr(library, 'j6m_tensor_' + name)
    dnn = ctypes.CDLL(str(REFERENCE / 'vendor/runtime_4_9_2/lib/libdnn.so'))
    dnn.hbDNNGetVersion.argtypes = []
    dnn.hbDNNGetVersion.restype = ctypes.c_char_p
    result['dnn_version_api'] = dnn.hbDNNGetVersion().decode()
    if result['dnn_version_api'] != dependencies['expected_dnn_version']:
        raise ValueError('DNN runtime API version differs from the pinned executor contract')
    result['numpy_version'] = np.__version__
    result['executor_loaded'] = True
    result['ptq_model_and_config_hashes_verified'] = True
    result['sample_tensors_verified'] = 2
    result['bpu_inference_executed'] = False
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=['status', 'verify', 'self-test',
                                            'check-models', 'activate', 'rollback'])
    args = parser.parse_args()
    if (platform.machine() != 'aarch64' or RELEASE.parent != BASE / 'releases'
            or Path(__file__).is_symlink()):
        raise RuntimeError('Run the installed J6M candidate only')
    current = BASE / 'current'
    if args.operation == 'status':
        result = status()
    elif args.operation == 'verify':
        result = verify()
    elif args.operation == 'self-test':
        required = environment()
        if any(os.environ.get(k) != required[k] for k in
               ['LD_LIBRARY_PATH', 'PYTHONPATH', 'PYTHONDONTWRITEBYTECODE',
                'OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS']):
            os.execve(sys.executable, [sys.executable, '-B', str(Path(__file__).resolve()),
                                      'self-test'], required)
        result = self_test()
    elif args.operation == 'check-models':
        result = dict(verify(), error='MISSING_FORMAL_HBM',
                      explanation='Compile both pinned ONNX models on the matched x86_64 toolchain first.')
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 2
    elif args.operation == 'activate':
        verify()
        if current.exists() or current.is_symlink():
            if not current.is_symlink() or current.resolve() != RELEASE:
                raise FileExistsError('Preserve the existing candidate selector')
        else:
            current.symlink_to(Path('releases') / RELEASE.name)
        result = status()
    else:
        if current.exists() or current.is_symlink():
            if not current.is_symlink() or current.resolve() != RELEASE:
                raise RuntimeError('Refuse to unlink a different candidate selector')
            current.unlink()
        result = dict(status(), rollback='Preparation selector removed; all files retained.')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError) as exc:
        print(json.dumps({'error': str(exc), 'production_ready': False}, ensure_ascii=False),
              file=sys.stderr)
        raise SystemExit(1)
