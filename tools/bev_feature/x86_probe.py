#!/usr/bin/env python3
"""Bounded, local OE compiler compatibility probe under ARM-to-x86 emulation.

This is separate from the native x86 training/compilation workflow. The old
workspace supplies read-only emulator/Python/dependencies; new SDK wheels,
caches and reports live in this workspace. No system binfmt registration.
"""
import argparse
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
OLD = Path('/home/slam/j6m-dev-b/runtime/yolo11_compile_20261019_j4liv6zx')
WORK = ROOT/'runtime/x86_solution_20261101'


def guest(action, output):
    import ctypes
    import importlib.metadata
    report = dict(execution_mode=os.environ['BEV_EXECUTION_MODE'],
                  physical_host_arch='aarch64', guest_arch=platform.machine(),
                  native_x86_validated=False, training_validated=False,
                  action=action, status='started')
    start = time.monotonic()
    try:
        report['packages'] = {n: importlib.metadata.version(n)
                              for n in ('hbdk4-compiler', 'hbdk4-march', 'numpy')}
        assert report['packages']['hbdk4-compiler'] == '4.11.11'
        assert report['packages']['hbdk4-march'] == '4.11.11'
        ctypes.CDLL(str(WORK/'site/hbdk4/compiler/_mlir_libs/libhbtl.so'), mode=ctypes.RTLD_GLOBAL)
        import hbdk4.compiler as compiler
        assert str(compiler.__file__).startswith(str(WORK/'site/'))
        report.update(compiler_file=compiler.__file__, march=str(compiler.March.nash_m))
        print('Compiler 4.11.11 imported under '+report['execution_mode'], flush=True)
        if action == 'load':
            module = compiler.load(str(ROOT/'runtime/bev_feature_20261101/official/nash_m/compile/quantized_deploy.bc'))
            report['functions'] = [dict(name=f.name,
                inputs=[dict(name=v.name, shape=v.type.shape, dtype=str(v.type.np_dtype)) for v in f.flatten_inputs],
                outputs=[dict(name=v.name, shape=v.type.shape, dtype=str(v.type.np_dtype)) for v in f.flatten_outputs])
                for f in module.functions]
        report['status'] = 'passed'
    except Exception as exc:
        report.update(status='failed', error=str(exc))
        raise
    finally:
        report['elapsed_s'] = time.monotonic()-start
        output.write_text(json.dumps(report, indent=2)+'\n')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--backend', choices=('box64', 'qemu'), default='qemu')
    p.add_argument('--action', choices=('import', 'load'), default='import')
    p.add_argument('--timeout', type=int, default=120)
    p.add_argument('--guest', action='store_true', help=argparse.SUPPRESS)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    a.output = a.output.resolve()
    if a.guest:
        return guest(a.action, a.output)
    if platform.machine() != 'aarch64':
        p.error('This diagnostic uses the local ARM64 emulator; use oe391.sh on native x86')
    if not 1 <= a.timeout <= 300:
        p.error('--timeout must be 1..300 seconds')
    if ROOT not in a.output.parents:
        p.error('--output must be inside this workspace')
    a.output.parent.mkdir(parents=True, exist_ok=True)
    site = WORK/'site'
    if not (site/'hbdk4/compiler/_mlir_libs/libhbtl.so').is_file():
        p.error('Install the delivered cp310 compiler/march wheels with pip --target '+str(site))
    sysroot = OLD/'emulation/x86-jammy'
    libraries = [OLD/'emulation/python310/python/lib',
                 sysroot/'lib/x86_64-linux-gnu', sysroot/'usr/lib/x86_64-linux-gnu',
                 site/'hbdk4/compiler/_mlir_libs', OLD/'sdk-python/numpy.libs']
    library_path = ':'.join(map(str, libraries))
    env = {k:v for k,v in os.environ.items()
           if k not in ('LD_LIBRARY_PATH', 'PYTHONHOME', 'PYTHONPATH') and not k.startswith('BOX64_')}
    env.update(PYTHONPATH=str(site)+':'+str(OLD/'sdk-python'), PYTHONNOUSERSITE='1',
               PYTHONDONTWRITEBYTECODE='1', BEV_EXECUTION_MODE='emulated-'+a.backend,
               BOX64_LD_LIBRARY_PATH=library_path, BOX64_NOBANNER='1', BOX64_LOG='0',
               BOX64_EMULATED_LIBS='libgomp.so.1', OPENBLAS_NUM_THREADS='1',
               OMP_NUM_THREADS='2', MKL_NUM_THREADS='2', CUDA_VISIBLE_DEVICES='',
               TMPDIR=str(WORK/'temp'), XDG_CACHE_HOME=str(WORK/'cache'),
               MPLCONFIGDIR=str(WORK/'matplotlib'))
    for name in ('temp', 'cache', 'matplotlib'):
        (WORK/name).mkdir(parents=True, exist_ok=True)
    python = OLD/'emulation/python310/python/bin/python3'
    if a.backend == 'box64':
        command = [str(OLD/'emulation/box64-build/box64'), str(python)]
    else:
        command = [str(OLD/'emulation/qemu10/usr/bin/qemu-x86_64'), '-cpu', 'max',
                   '-L', str(sysroot), '-E', 'LD_LIBRARY_PATH='+library_path, str(python)]
    command = ['bwrap', '--die-with-parent', '--ro-bind', '/', '/', '--bind', str(ROOT), str(ROOT),
               '--dev', '/dev', '--proc', '/proc', '--unshare-net',
               '--bind', str(WORK/'temp'), '/tmp', '--chdir', str(WORK), *command,
               '-B', str(Path(__file__).resolve()), '--guest', '--action', a.action,
               '--output', str(a.output)]
    started = time.monotonic()
    with a.output.with_suffix('.log').open('w') as log:
        try:
            result = subprocess.run(command, env=env, timeout=a.timeout,
                                    stdout=log, stderr=subprocess.STDOUT)
            code = result.returncode
        except subprocess.TimeoutExpired:
            code = 124
    a.output.with_suffix('.process.json').write_text(json.dumps(dict(
        execution_mode='emulated-'+a.backend, physical_host_arch=platform.machine(),
        action=a.action, command=command, returncode=code, elapsed_s=time.monotonic()-started), indent=2)+'\n')
    print(json.dumps(dict(returncode=code, report=str(a.output))))
    return code


if __name__ == '__main__':
    sys.exit(main())
