#!/usr/bin/env python3
"""Preparation check only: imports, small tensor algebra/autograd; zero learning."""
import argparse
import hashlib
import importlib
import importlib.metadata as md
import json
import os
from pathlib import Path
import platform
import random
import subprocess
import sys

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);args=parser.parse_args()
    result={'schema':'astra_train_cuda_environment_v1','python':platform.python_version(),'executable':sys.executable,
        'platform':platform.platform(),'training_steps':0,'environment_steps':0,'optimizer_updates':0,'errors':[],
        'modules':{},'seed':260926,'thread_count':1}
    for name in ['numpy','pandas','matplotlib','gymnasium','stable_baselines3','torch','scipy.spatial',
                 'scipy.ndimage','networkx','sympy','cloudpickle','numexpr','yaml','rospy','rosbag']:
        try:
            module=importlib.import_module(name)
            result['modules'][name]={'origin':getattr(module,'__file__',None),'version':getattr(module,'__version__',None)}
        except Exception as error:result['errors'].append(name+': '+type(error).__name__+': '+str(error))
    try:
        import numpy as np
        import torch
        random.seed(260926);np.random.seed(260926);torch.manual_seed(260926);torch.cuda.manual_seed_all(260926)
        torch.set_num_threads(1);torch.use_deterministic_algorithms(True)
        torch.backends.cudnn.benchmark=False;torch.backends.cudnn.deterministic=True
        torch.backends.cuda.matmul.allow_tf32=False;torch.backends.cudnn.allow_tf32=False
        result['cuda']={'available':torch.cuda.is_available(),'torch_version':torch.__version__,
            'compiled_runtime':torch.version.cuda,'cudnn':torch.backends.cudnn.version(),
            'arch_list':torch.cuda.get_arch_list(),'device_count':torch.cuda.device_count()}
        if not torch.cuda.is_available():raise RuntimeError('CUDA unavailable')
        if torch.__version__!='2.4.1+cu121':raise RuntimeError('unexpected Torch build')
        device=torch.cuda.get_device_properties(0)
        result['cuda']['device']={'name':device.name,'total_memory_bytes':device.total_memory,
                                 'compute_capability':[device.major,device.minor]}
        if '.venv_train_cuda/' not in str(torch.__file__):raise RuntimeError('Torch did not originate from CUDA venv')
        for name in ['numpy','gymnasium','stable_baselines3']:
            if '.venv_train_cuda/' not in result['modules'][name]['origin']:raise RuntimeError(name+' not inside CUDA venv')
        torch.cuda.reset_peak_memory_stats()
        outcomes=[]
        for repeat in range(2):
            torch.manual_seed(260926);torch.cuda.manual_seed_all(260926)
            a=torch.randn(16,8,device='cuda',dtype=torch.float64,requires_grad=True)
            b=torch.randn(8,4,device='cuda',dtype=torch.float64)
            product=a@b;loss=product.square().sum();loss.backward();torch.cuda.synchronize()
            reference=2*(a.detach().cpu()@b.cpu())@b.cpu().T
            error=(a.grad.cpu()-reference).abs().max().item()
            if not torch.isfinite(a.grad).all() or error>1e-9:raise RuntimeError('CUDA autograd analytic check failed')
            payload=product.detach().cpu().numpy().tobytes()+a.grad.cpu().numpy().tobytes()
            outcomes.append({'sha256':hashlib.sha256(payload).hexdigest(),'gradient_max_abs_error':error,'loss':loss.item()})
        result['tensor_autograd']=outcomes
        if outcomes[0]!=outcomes[1]:raise RuntimeError('same-seed deterministic CUDA repeat differs')
        # Also load/use cuDNN in a small differentiable convolution, no optimizer.
        x=torch.arange(64,dtype=torch.float32,device='cuda').reshape(1,1,8,8).requires_grad_(True)
        weight=torch.ones(1,1,3,3,device='cuda');y=torch.nn.functional.conv2d(x,weight);y.sum().backward()
        expected=torch.nn.functional.conv2d(x.detach().cpu(),weight.cpu())
        result['cudnn_convolution']={'output_matches_cpu':torch.equal(y.detach().cpu(),expected),
                                    'finite_input_gradient':bool(torch.isfinite(x.grad).all())}
        if not all(result['cudnn_convolution'].values()):raise RuntimeError('cuDNN tensor check failed')
        result['cuda']['max_allocated_bytes']=torch.cuda.max_memory_allocated()
        result['determinism']={'enabled':torch.are_deterministic_algorithms_enabled(),
            'cudnn_deterministic':torch.backends.cudnn.deterministic,'cudnn_benchmark':torch.backends.cudnn.benchmark,
            'matmul_tf32':torch.backends.cuda.matmul.allow_tf32,'cudnn_tf32':torch.backends.cudnn.allow_tf32,
            'cublas_workspace_config':os.environ.get('CUBLAS_WORKSPACE_CONFIG'),
            'pythonhashseed':os.environ.get('PYTHONHASHSEED'),'same_seed_repeat_exact':outcomes[0]==outcomes[1],
            'scope':'this platform/build and tested operators; not cross-release/device or future-training determinism proof'}
    except Exception as error:result['errors'].append('tensor verification: '+type(error).__name__+': '+str(error))
    check=subprocess.run([sys.executable,'-m','pip','check'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    result['pip_check']={'exit_code':check.returncode,'output':check.stdout.strip()}
    result['gpu_driver']=subprocess.check_output(['nvidia-smi','--query-gpu=name,driver_version,memory.total,compute_cap','--format=csv,noheader'],text=True).strip()
    result['status']='pass' if not result['errors'] else 'fail'
    Path(args.output).write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({key:result[key] for key in ['status','errors','cuda','pip_check'] if key in result}))
    return 0 if result['status']=='pass' else 1
if __name__=='__main__':raise SystemExit(main())
