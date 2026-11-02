#!/usr/bin/env python3
"""Official SIX-camera benchmark and resident binding test; no ROS commands.
Temporarily stop the old BPU receiver, restoring its active state in finally.
All uploads/builds go to private /tmp; formal board current is unchanged.
"""
import argparse
import json
import shlex
import subprocess
from pathlib import Path
from prepare_official import digest, HASHES

ROOT = Path(__file__).resolve().parents[2]

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', required=True)
    p.add_argument('--frames', type=int, default=100)
    p.add_argument('--host', default='root@192.168.10.100')
    p.add_argument('--model', default=str(ROOT/'runtime/bev_feature_20261101/official/nash_m/compile/model.hbm'))
    p.add_argument('--native', action='store_true', help='also compile/test resident executor')
    a = p.parse_args()
    if a.frames < 1 or digest(Path(a.model)) != HASHES['nash_m/compile/model.hbm']:
        raise ValueError('Invalid frame count or unexpected official HBM SHA256')
    out = Path(a.output); out.mkdir(parents=True, exist_ok=True)
    b = '/tmp/conbine_bev_vtv2_20261101'
    ref = '/map/robot_j6m_navigation_20260907/bpu_lab_20260905'
    lib = ref+'/vendor/runtime_4_9_2'
    ssh = ['ssh', '-o', 'BatchMode=yes', a.host]
    def remote(args, **kwargs):
        return subprocess.run(ssh+[shlex.join(args)], check=True, timeout=60, **kwargs)
    def upload(path, name):
        subprocess.run(['scp','-q',str(path),a.host+':'+b+'/'+name],check=True,timeout=60)
    remote(['mkdir','-p',b])
    upload(a.model,'model.hbm')
    remote(['python3','-c',
        "from pathlib import Path; p=Path("+repr(b)+"); "
        "(p/'y.bin').write_bytes(bytes([128])*180224); "
        "(p/'uv.bin').write_bytes(bytes([128])*90112); "
        "(p/'points.bin').write_bytes(bytes(655360))"])
    if a.native:
        upload(ROOT/'distribution/j6m_bpu_navigation/native/tensor_executor.cpp','tensor_executor.cpp')
        upload(ROOT/'tools/bev_feature/native_reference.cpp','native_reference.cpp')
        sysroot = '/map/autolabor_runtime/rootfs'
        remote(['mkdir','-p',b+'/compiler_lib'])
        for name in ['libisl.so.22','libmpc.so.3','libopcodes-2.34-system.so','libbfd-2.34-system.so','libctf.so.0']:
            remote(['cp','-L',sysroot+'/usr/lib/aarch64-linux-gnu/'+name,b+'/compiler_lib/'+name])
        with (out/'native_build.log').open('w') as f:
            remote(['env','LD_LIBRARY_PATH='+b+'/compiler_lib',sysroot+'/usr/bin/aarch64-linux-gnu-g++-9',
                '--sysroot='+sysroot,'-B'+sysroot+'/usr/bin/','-std=c++14','-O3','-Wall','-Wextra','-Werror',
                '-I'+ref+'/vendor/ucp_include',b+'/native_reference.cpp','-L'+lib+'/lib','-ldnn','-lhbucp',
                '-Wl,-rpath,'+lib+'/lib','-Wl,--allow-shlib-undefined',
                '-o',b+'/native_reference'],stdout=f,stderr=subprocess.STDOUT)
    prefix = ['flock','-n',ref+'/preview_worker.lock','env','LD_LIBRARY_PATH='+lib+'/lib:/usr/hobot/lib']
    unit = 'j6m-b-camera-receiver.service'
    was = subprocess.run(ssh+[shlex.join(['systemctl','is-active','--quiet',unit])]).returncode == 0
    try:
        if was: remote(['systemctl','stop',unit])
        with (out/'official_hbm_model_info.log').open('w') as f:
            remote(prefix+[lib+'/bin/hrt_model_exec','model_info','--model_file',b+'/model.hbm'],stdout=f,stderr=subprocess.STDOUT)
        inputs = ','.join([b+'/y.bin',b+'/uv.bin']*6+[b+'/points.bin']*2)
        with (out/'official_hbm_perf_valid.log').open('w') as f:
            remote(prefix+[lib+'/bin/hrt_model_exec','perf','--model_file',b+'/model.hbm','--input_file',inputs,
                '--core_id','1','--thread_num','1','--frame_count',str(a.frames),
                '--profile_path',b+'/profile_valid'],stdout=f,stderr=subprocess.STDOUT)
        if a.native:
            with (out/'native_run.log').open('w') as f:
                remote(prefix+[b+'/native_reference',b+'/model.hbm',b+'/native_result.json'],stdout=f,stderr=subprocess.STDOUT)
            subprocess.run(['scp','-q',a.host+':'+b+'/native_result.json',str(out)],check=True,timeout=30)
        subprocess.run(['scp','-qr',a.host+':'+b+'/profile_valid',str(out)],check=True,timeout=30)
    finally:
        if was: remote(['systemctl','start',unit])
        state = subprocess.run(ssh+[shlex.join(['systemctl','is-active',unit])],capture_output=True,text=True,timeout=30)
        (out/'receiver_restoration.json').write_text(json.dumps(dict(was_active=was,after=state.stdout.strip()),indent=2))

if __name__ == '__main__':
    main()
