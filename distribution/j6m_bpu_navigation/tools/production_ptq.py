#!/usr/bin/env python3
"""Prepare and validate private nash-m PTQ jobs; never change a ROS backend."""
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import shutil
import subprocess

PACKAGE = Path(__file__).resolve().parents[1]
REGISTRY = PACKAGE / 'production_models.json'


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def write_json(path, value):
    with Path(path).open('x', encoding='utf-8') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def inside(root, relative):
    path = root / relative
    if Path(relative).is_absolute() or '..' in Path(relative).parts or path.is_symlink():
        raise ValueError('Expected a relative regular file: ' + str(relative))
    if root.resolve() not in path.resolve().parents:
        raise ValueError('Path escapes workspace: ' + str(relative))
    return path


def contract():
    value = json.loads(REGISTRY.read_text())
    if value['target'] != 'nash-m' or value['production_ready'] is not False:
        raise ValueError('Expected the unaccepted J6M draft contract')
    return value


def compile_config(kind, spec):
    # Already preprocessed float32 inputs preserve the FP32 export boundary.
    # JSON is a YAML subset. No implicit NV12 conversion or second normalization.
    return {
        'model_parameters': {
            'onnx_model': '../../models/' + kind + '.onnx', 'march': 'nash-m',
            'working_dir': './build', 'output_model_file_prefix': kind},
        'input_parameters': {
            'input_name': spec['input_name'],
            'input_shape': 'x'.join(map(str, spec['input_shape'])),
            'input_type_rt': 'featuremap', 'input_type_train': 'featuremap',
            'input_layout_train': 'NCHW'},
        'calibration_parameters': {'cal_data_dir': '../../calibration/' + kind},
        'compiler_parameters': {
            'compile_mode': 'latency', 'core_num': 1, 'optimize_level': 'O2',
            'input_source': {spec['input_name']: 'ddr'}}}


def verify_export(directory, spec):
    import onnx
    path = directory / 'best.onnx'
    report = json.loads((directory / 'export_manifest.json').read_text())
    if (digest(path) != spec['onnx_sha256'] or report['onnx_sha256'] != spec['onnx_sha256']
            or report['sha256'] != spec['checkpoint_sha256']
            or report['source_sha256_after'] != spec['checkpoint_sha256']
            or report['classes'] != spec['classes'] or report['task'] != spec['task']
            or report.get('onnx_checker_passed') is not True
            or (spec['gam_required'] and report.get('gam_layers', 0) < 1)):
        raise ValueError('Production export identity/class/GAM mismatch: ' + str(directory))
    graph = onnx.load(str(path), load_external_data=False)
    onnx.checker.check_model(graph, full_check=True)
    for tensors, name, shape in ((graph.graph.input, spec['input_name'], spec['input_shape']),
                                 (graph.graph.output, spec['output_name'], spec['output_shape'])):
        if (len(tensors) != 1 or tensors[0].name != name
                or tensors[0].type.tensor_type.elem_type != onnx.TensorProto.FLOAT
                or [d.dim_value for d in tensors[0].type.tensor_type.shape.dim] != shape):
            raise ValueError('Unexpected ONNX interface')
    if any(item.data_location == onnx.TensorProto.EXTERNAL for item in graph.graph.initializer):
        raise ValueError('External ONNX data is unavailable in this fixed export')
    return path, report


def prepare(exports, output):
    registry = contract()
    exports, output = exports.resolve(), output.absolute()
    if output.exists() or output.is_symlink():
        raise FileExistsError('Use a new PTQ output directory')
    checked = {kind: verify_export(exports / kind, spec)
               for kind, spec in registry['models'].items()}
    output.mkdir(parents=True)
    (output / 'models').mkdir()
    for kind, (path, report) in checked.items():
        shutil.copyfile(path, output / 'models' / (kind + '.onnx'))
        job = output / 'jobs' / kind
        job.mkdir(parents=True)
        (output / 'calibration' / kind).mkdir(parents=True)
        write_json(job / 'compile.yaml', compile_config(kind, registry['models'][kind]))
        write_json(job / 'export_manifest.json', report)
    write_json(output / 'contract.json', registry)
    write_json(output / 'STATUS.json', {
        'stage': 'onnx_verified_ptq_prepared', 'calibration_at_preparation': False,
        'int8_accuracy_accepted': False, 'hbm_board_accepted': False,
        'production_ready': False})
    print(json.dumps({'output': str(output), 'models': list(checked), 'compiled': False}))


def validate_tensor(path, spec):
    import numpy as np
    # Refuse object pickles, oversized files and memmaps with unexpected shape.
    limit = 4
    for dimension in spec['input_shape']:
        limit *= dimension
    if path.stat().st_size > limit + 4096:
        raise ValueError('Oversized calibration tensor')
    tensor = np.load(path, allow_pickle=False)
    if (tensor.dtype != np.float32 or list(tensor.shape) != spec['input_shape']
            or not np.isfinite(tensor).all() or tensor.min() < 0 or tensor.max() > 1):
        raise ValueError('Expected finite normalized float32 NCHW calibration input')


def calibration_records(manifest, spec, kind):
    data = json.loads(manifest.read_text())
    if (data.get('model') != kind or data.get('split') != 'train'
            or data.get('preprocess_id') != spec['preprocess_id']):
        raise ValueError('Calibration requires declared train split and exact preprocessing')
    records = data.get('samples', [])
    if not isinstance(records, list) or len(records) < spec['minimum_calibration_samples']:
        raise ValueError('Insufficient calibration samples')
    seen = set()
    classes = set()
    checked = []
    for record in records:
        path = inside(manifest.parent, record['path'])
        if (not path.is_file() or path.suffix != '.npy' or digest(path) != record['sha256']
                or record['sha256'] in seen or record.get('class_name') not in spec['classes']):
            raise ValueError('Invalid, duplicate or mislabelled calibration sample')
        validate_tensor(path, spec)
        seen.add(record['sha256'])
        classes.add(record['class_name'])
        checked.append((path, record))
    if classes != set(spec['classes']):
        raise ValueError('Calibration must cover every material class')
    return data, checked


def verify_workspace(workspace, require_calibration=False):
    registry = contract()
    if json.loads((workspace / 'contract.json').read_text()) != registry:
        raise ValueError('PTQ workspace contract changed')
    for kind, spec in registry['models'].items():
        if digest(workspace / 'models' / (kind + '.onnx')) != spec['onnx_sha256']:
            raise ValueError('PTQ model hash changed: ' + kind)
        config = json.loads((workspace / 'jobs' / kind / 'compile.yaml').read_text())
        if config != compile_config(kind, spec):
            raise ValueError('PTQ configuration changed: ' + kind)
        if require_calibration:
            manifest = workspace / 'calibration' / kind / 'manifest.json'
            _, records = calibration_records(manifest, spec, kind)
            actual = {p.name for p in manifest.parent.iterdir()}
            expected = {'manifest.json'} | {p.name for p, _ in records}
            if actual != expected:
                raise ValueError('Unrecorded calibration files: ' + kind)
    return registry


def attach_calibration(workspace, kind, manifest):
    registry = verify_workspace(workspace)
    data, records = calibration_records(manifest, registry['models'][kind], kind)
    target = workspace / 'calibration' / kind
    if any(target.iterdir()):
        raise FileExistsError('Calibration directory must be empty; preserve earlier data')
    copied = []
    for index, (path, record) in enumerate(records):
        name = '%05d.npy' % index
        shutil.copyfile(path, target / name)
        copied.append(dict(record, path=name))
    # run() copies only these tensors into its separate compiler input folder.
    write_json(target / 'manifest.json', dict(data, samples=copied))
    print(json.dumps({'model': kind, 'samples': len(copied), 'compiled': False}))


def run(workspace, lock_path):
    if platform.machine() != 'x86_64':
        raise RuntimeError('Run hb_compile in the isolated x86_64 OpenExplorer environment')
    verify_workspace(workspace, require_calibration=True)
    lock = json.loads(lock_path.read_text())
    if (not isinstance(lock.get('packages'), dict) or not lock.get('environment_id')
            or not {'horizon-tc-ui', 'hbdk4'}.issubset(lock['packages'])):
        raise ValueError('Supply exact SDK package versions and environment_id/image digest')
    for package, version in lock['packages'].items():
        if not version or importlib.metadata.version(package) != version:
            raise ValueError('Toolchain package version mismatch: ' + package)
    compiler = shutil.which('hb_compile')
    if not compiler:
        raise RuntimeError('hb_compile is missing')
    for kind in contract()['models']:
        job = workspace / 'jobs' / kind
        if (job / 'build').exists() or (job / 'compile.log').exists():
            raise FileExistsError('Use a fresh PTQ workspace to retain previous compiler evidence')
    write_json(workspace / 'TOOLCHAIN_LOCK.json', lock)
    for kind in contract()['models']:
        job = workspace / 'jobs' / kind
        # Only .npy tensors are visible to the calibration loader.
        calibration = workspace / 'calibration' / kind
        config = compile_config(kind, contract()['models'][kind])
        inputs = job / 'calibration_inputs'
        inputs.mkdir()
        for path in sorted(calibration.glob('*.npy')):
            shutil.copyfile(path, inputs / path.name)
        config['calibration_parameters']['cal_data_dir'] = './calibration_inputs'
        write_json(job / 'run.yaml', config)
        command = [compiler, '--config', str((job / 'run.yaml').resolve())]
        with (job / 'compile.log').open('xb') as stream:
            result = subprocess.run(command, cwd=job, stdout=stream, stderr=subprocess.STDOUT)
        hbm = list((job / 'build').rglob('*.hbm')) if (job / 'build').exists() else []
        write_json(job / 'compile_result.json', {
            'returncode': result.returncode,
            'hbm_files': [{'path': str(p.relative_to(job)), 'sha256': digest(p)} for p in hbm],
            'bpu_operator_assignment_reviewed': False, 'int8_accuracy_accepted': False,
            'hbm_board_accepted': False, 'production_ready': False})
        if result.returncode or not hbm:
            raise RuntimeError('Compilation failed or no HBM: ' + kind + '; inspect compile.log')
    print('HBM compilation finished; accuracy, operator placement and board acceptance are pending.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='operation', required=True)
    prep = commands.add_parser('prepare')
    prep.add_argument('--exports', type=Path, required=True)
    prep.add_argument('--output', type=Path, required=True)
    attach = commands.add_parser('attach-calibration')
    attach.add_argument('--workspace', type=Path, required=True)
    attach.add_argument('--model', choices=list(contract()['models']), required=True)
    attach.add_argument('--manifest', type=Path, required=True)
    check = commands.add_parser('verify')
    check.add_argument('--workspace', type=Path, required=True)
    check.add_argument('--require-calibration', action='store_true')
    execute = commands.add_parser('run')
    execute.add_argument('--workspace', type=Path, required=True)
    execute.add_argument('--toolchain-lock', type=Path, required=True)
    args = parser.parse_args()
    if args.operation == 'prepare':
        prepare(args.exports, args.output)
    elif args.operation == 'attach-calibration':
        attach_calibration(args.workspace.resolve(), args.model, args.manifest.resolve())
    elif args.operation == 'verify':
        verify_workspace(args.workspace.resolve(), args.require_calibration)
        print('PTQ workspace verified; production_ready=false')
    else:
        run(args.workspace.resolve(), args.toolchain_lock.resolve())


if __name__ == '__main__':
    main()
