#!/usr/bin/env python3
"""Validate the delivered OE runtime privately on J6M with the six-camera HBM.

Reuses hrt_model_exec and rebuilds the resident adapter with the SDK headers.
Only their process-local shared libraries change. No service is started/stopped,
and no system library, camera, navigation, or formal current link is changed.
"""
import argparse
import json
from pathlib import Path
import platform
import re
import shlex
import subprocess

from oe391_environment import sha256
from prepare_official import HASHES

ROOT = Path(__file__).resolve().parents[2]
REFERENCE = '/map/robot_j6m_navigation_20260907/bpu_lab_20260905'
LIBRARIES = ('libdnn.so', 'libhbrt4.so', 'libhbucp.so', 'libhb_arm_rpc.so',
             'libhbtl.so', 'libhlog_wrapper.so', 'libperfetto_sdk.so')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdk', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--frames', type=int, default=100)
    parser.add_argument('--cxx', default='g++' if platform.machine() == 'aarch64'
                        else 'aarch64-linux-gnu-g++', help='AArch64 C++ compiler')
    args = parser.parse_args()
    if not 1 <= args.frames <= 1000:
        parser.error('--frames must be 1..1000')
    args.output.mkdir(parents=True, exist_ok=True)
    libs = args.sdk/'samples/ucp_tutorial/deps_aarch64/ucp/lib'
    model = ROOT/'runtime/bev_feature_20261101/official/nash_m/compile/model.hbm'
    if sha256(model) != HASHES['nash_m/compile/model.hbm']:
        raise ValueError('Official six-camera HBM identity mismatch')
    manifest = {}
    for name in LIBRARIES:
        path = libs/name
        with path.open('rb') as stream:
            header = stream.read(20)
        if header[:4] != b'\x7fELF' or int.from_bytes(header[18:20], 'little') != 183:
            raise ValueError('Not an AArch64 shared library: '+str(path))
        manifest[name] = sha256(path)
    # This builds the small board adapter, not the x86 model compiler or HEAL.
    include = args.sdk/'samples/ucp_tutorial/deps_aarch64/ucp/include'
    source = ROOT/'tools/bev_feature/native_reference.cpp'
    executor = ROOT/'distribution/j6m_bpu_navigation/native/tensor_executor.cpp'
    executable = args.output.resolve()/'native_reference'
    build = [args.cxx, '-std=c++14', '-O3', '-Wall', '-Wextra', '-Werror',
             '-DJ6M_DNN_EXPECTED_VERSION="3.15.8_(4.11.11 HBRT)"',
             '-I'+str(include), '-I'+str(executor.parent), str(source),
             '-L'+str(libs), '-ldnn', '-lhbucp', '-Wl,--allow-shlib-undefined',
             '-o', str(executable)]
    with (args.output/'build.log').open('w') as stream:
        subprocess.run(build, check=True, timeout=60, stdout=stream, stderr=subprocess.STDOUT)
    with executable.open('rb') as stream:
        header = stream.read(20)
    if header[:4] != b'\x7fELF' or int.from_bytes(header[18:20], 'little') != 183:
        raise ValueError('Compiler must produce an AArch64 adapter executable')
    (args.output/'build.json').write_text(json.dumps(dict(command=build,
        sources={str(p.relative_to(ROOT)): sha256(p) for p in (source, executor)},
        headers={str(p.relative_to(include)): sha256(p) for p in include.rglob('*.h')},
        executable_sha256=sha256(executable)), indent=2)+'\n')
    ssh = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', 'root@192.168.10.100']

    def remote(arguments, **kwargs):
        return subprocess.run(ssh+[shlex.join(arguments)], check=True, timeout=60, **kwargs)

    board = remote(['mktemp', '-d', '/tmp/conbine-oe391-XXXXXXXX'], capture_output=True, text=True).stdout.strip()
    if not re.fullmatch(r'/tmp/conbine-oe391-[A-Za-z0-9]+', board):
        raise ValueError('Unexpected temporary directory')
    remote(['mkdir', board+'/lib'])
    subprocess.run(['scp', '-q', *[str(libs/n) for n in LIBRARIES],
                    'root@192.168.10.100:'+board+'/lib/'], check=True, timeout=60)
    subprocess.run(['scp', '-q', str(model), 'root@192.168.10.100:'+board+'/model.hbm'],
                   check=True, timeout=60)
    subprocess.run(['scp', '-q', str(executable), 'root@192.168.10.100:'+board+'/native_reference'],
                   check=True, timeout=30)
    hashes = remote(['sha256sum', *[board+'/lib/'+n for n in LIBRARIES], board+'/model.hbm',
                     board+'/native_reference', REFERENCE+'/vendor/runtime_4_9_2/bin/hrt_model_exec'],
                    capture_output=True, text=True).stdout
    actual = {Path(line.split()[1]).name: line.split()[0] for line in hashes.splitlines()}
    if (any(actual[n] != h for n, h in manifest.items()) or actual['model.hbm'] != sha256(model)
            or actual['native_reference'] != sha256(executable)):
        raise ValueError('Board upload digest mismatch')
    (args.output/'runtime_manifest.json').write_text(json.dumps(dict(
        sdk=str(args.sdk.resolve()), board=board, libraries=manifest, model_sha256=sha256(model),
        native_sha256=sha256(executable), hrt_model_exec=REFERENCE+'/vendor/runtime_4_9_2/bin/hrt_model_exec',
        hrt_model_exec_sha256=actual['hrt_model_exec']),
        indent=2)+'\n')
    library_path = board+'/lib:/usr/hobot/lib'
    tool = REFERENCE+'/vendor/runtime_4_9_2/bin/hrt_model_exec'
    native = board+'/native_reference'
    state_args = ['systemctl', 'show', 'j6m-b-camera-receiver.service', 'j6m-b-clock.service',
                  '-p', 'Id', '-p', 'MainPID', '-p', 'ActiveState']
    before = remote(state_args, capture_output=True, text=True).stdout
    prefix = ['flock', '-n', REFERENCE+'/preview_worker.lock', 'env', 'LD_LIBRARY_PATH='+library_path]
    try:
        with (args.output/'loader.log').open('w') as stream:
            remote(['env', 'LD_LIBRARY_PATH='+library_path, 'ldd', tool], stdout=stream, stderr=subprocess.STDOUT)
        remote(['python3', '-c', 'from pathlib import Path; p=Path('+repr(board)+'); '
                '(p/"y.bin").write_bytes(bytes([128])*180224); '
                '(p/"uv.bin").write_bytes(bytes([128])*90112); '
                '(p/"points.bin").write_bytes(bytes(655360))'])
        with (args.output/'model_info.log').open('w') as stream:
            remote(prefix+[tool, 'model_info', '--model_file', board+'/model.hbm'],
                   stdout=stream, stderr=subprocess.STDOUT)
        inputs = ','.join([board+'/y.bin', board+'/uv.bin']*6+[board+'/points.bin']*2)
        with (args.output/'perf.log').open('w') as stream:
            remote(prefix+[tool, 'perf', '--model_file', board+'/model.hbm', '--input_file', inputs,
                           '--core_id', '1', '--thread_num', '1', '--frame_count', str(args.frames),
                           '--profile_path', board+'/profile'], stdout=stream, stderr=subprocess.STDOUT)
        subprocess.run(['scp', '-qr', 'root@192.168.10.100:'+board+'/profile', str(args.output)],
                       check=True, timeout=30)
        with (args.output/'native.log').open('w') as stream:
            remote(prefix+[native, board+'/model.hbm', board+'/native_result.json'],
                   stdout=stream, stderr=subprocess.STDOUT)
        subprocess.run(['scp', '-q', 'root@192.168.10.100:'+board+'/native_result.json', str(args.output)],
                       check=True, timeout=30)
        print('OE runtime + existing tool + SDK-matched resident adapter verified:', board)
    finally:
        after = remote(state_args, capture_output=True, text=True).stdout
        (args.output/'services.json').write_text(json.dumps(dict(before=before, after=after,
                                                               unchanged=before == after), indent=2)+'\n')


if __name__ == '__main__':
    main()
