#!/usr/bin/env python3
"""Separate SDK file inventory from actual x86 container import verification."""
import argparse
import email
import hashlib
import importlib
import importlib.metadata
import json
from pathlib import Path
import platform
import re
import sys
import zipfile

EXPECTED = {'hbdk4-compiler': '4.11.11', 'hbdk4-march': '4.11.11',
            'horizon-plugin-pytorch': '3.3.10', 'heal': '0.0.2'}


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for data in iter(lambda: stream.read(8*1024*1024), b''):
            digest.update(data)
    return digest.hexdigest()


def matching(have, want):
    return isinstance(have, str) and have.split('+', 1)[0] == want


def package_report(sdk):
    sdk = sdk.resolve(strict=True)
    wheels, errors = [], []
    for path in sorted((sdk/'package/host/ai_toolchain').glob('*.whl')):
        if not path.name.startswith(('hbdk4_compiler-', 'hbdk4_march-', 'horizon_plugin_pytorch-')):
            continue
        with zipfile.ZipFile(path) as wheel:
            metadata = email.message_from_bytes(wheel.read(next(
                n for n in wheel.namelist() if n.endswith('.dist-info/METADATA'))))
            tags = wheel.read(next(n for n in wheel.namelist() if n.endswith('.dist-info/WHEEL'))).decode()
            native = next((n for n in wheel.namelist() if n.endswith('.so')), None)
            elf_machine = None
            if native:
                with wheel.open(native) as stream:
                    header = stream.read(20)
                if header[:4] == b'\x7fELF':
                    elf_machine = int.from_bytes(header[18:20], 'little' if header[5] == 1 else 'big')
            name, version = metadata['Name'], metadata['Version']
            wheels.append(dict(path=str(path.relative_to(sdk)), name=name, version=version,
                               bytes=path.stat().st_size, sha256=sha256(path),
                               tags=[line[5:] for line in tags.splitlines() if line.startswith('Tag: ')],
                               native_example=native, elf_machine=elf_machine,
                               requires=metadata.get_all('Requires-Dist', [])))
            if name in EXPECTED and not matching(version, EXPECTED[name]):
                errors.append('Unexpected package version: '+name+' '+version)
    for name in ('hbdk4-compiler', 'hbdk4-march', 'horizon-plugin-pytorch'):
        if not any(row['name'] == name for row in wheels):
            errors.append('Missing wheel: '+name)
    launcher = sdk/'run_docker.sh'
    if not launcher.is_file():
        errors.append('Official run_docker.sh missing')
    return dict(mode='package', sdk=str(sdk), host_arch=platform.machine(), wheels=wheels,
                launcher_sha256=sha256(launcher) if launcher.is_file() else None,
                heal_configs=[str(p.relative_to(sdk)) for p in sdk.rglob('bev_vtv2_resnet50_nuscenes')],
                runtime_imports_tested=False, errors=errors)


def runtime_report(stage='training', heal_root=None):
    result = dict(mode='runtime', stage=stage, host_arch=platform.machine(),
                  python=sys.version, packages={}, imports={}, errors=[])
    errors = result['errors']
    if platform.machine() != 'x86_64' or platform.system() != 'Linux':
        errors.append('A native x86_64 Linux environment is required for training/compilation')
    if sys.version_info[:2] != (3, 10):
        errors.append('Use the py310 environment from this OE package')
    required = ('hbdk4-compiler', 'hbdk4-march') if stage == 'compiler' else tuple(EXPECTED)
    for name, wanted in EXPECTED.items():
        try:
            actual = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            actual = None
        result['packages'][name] = dict(required=wanted, installed=actual)
        if name in required and not matching(actual, wanted):
            errors.append('Missing/mismatched package: '+name)
    # Do not attempt to load an x86 shared library into this Jetson interpreter.
    if not errors:
        for name in (('hbdk4.compiler',) if stage == 'compiler' else
                     ('hbdk4.compiler', 'torch', 'horizon_plugin_pytorch', 'heal')):
            try:
                module = importlib.import_module(name)
                result['imports'][name] = dict(file=getattr(module, '__file__', None), loaded=True)
                if name == 'hbdk4.compiler':
                    result['march'] = str(module.March.nash_m)
                if name == 'torch':
                    result['torch'] = dict(version=module.__version__, cuda=module.version.cuda,
                                           cuda_available=module.cuda.is_available())
            except Exception as exc:
                result['imports'][name] = dict(loaded=False, error=str(exc))
                errors.append('Import failed: '+name)
        if stage == 'training' and 'torch' in result:
            plugin = result['packages']['horizon-plugin-pytorch']['installed']
            tag = re.search(r'\+cu(\d+)\.torch(\d+)', plugin)
            torch_version = result['torch']['version'].split('+', 1)[0].replace('.', '')
            cuda = (result['torch']['cuda'] or '').replace('.', '')
            if not tag or (tag[1], tag[2]) != (cuda, torch_version):
                errors.append('Torch/CUDA does not match the plugin binary build tag')
            if not result['torch']['cuda_available']:
                result['notes'] = ['CUDA device unavailable; only CPU-supported operations can run']
    if stage == 'training':
        for relative in ('configs/bev_vtv2_resnet50_nuscenes',
                         'heal/models/task_modules/bev/view_transformer.py'):
            if heal_root is None or not (heal_root/relative).exists():
                errors.append('HEAL source missing: '+relative)
    result['runtime_imports_tested'] = bool(result['imports'])
    result['ready'] = not errors
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('package', 'runtime'))
    parser.add_argument('--sdk', type=Path)
    parser.add_argument('--stage', choices=('compiler', 'training'), default='training')
    parser.add_argument('--heal-root', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.mode == 'package' and args.sdk is None:
        parser.error('--sdk is required for package inventory')
    result = package_report(args.sdk) if args.mode == 'package' else runtime_report(args.stage, args.heal_root)
    text = json.dumps(result, indent=2)+'\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text)
    print(text)
    return 2 if result['errors'] else 0


if __name__ == '__main__':
    sys.exit(main())
