"""Closed N06 startup/child environment. Pure standard library, no ROS imports."""
import contextlib
import hashlib
import json
import os
from pathlib import Path

WORKSPACE=Path(__file__).resolve().parents[2]
PROTOCOL='astra_n06_fixed_environment_r05'
MODEL_DATABASE='http://127.0.0.1:9'
BASE_KEYS=set('HOME USER LOGNAME LANG PATH PYTHONPATH LD_LIBRARY_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH ROS_ROOT ROS_DISTRO ROS_MASTER_URI GAZEBO_MASTER_URI ROS_IP ROS_HOSTNAME GAZEBO_IP GAZEBO_PLUGIN_PATH GAZEBO_RESOURCE_PATH GAZEBO_MODEL_PATH GAZEBO_MODEL_DATABASE_URI OGRE_RESOURCE_PATH PKG_CONFIG_PATH VIRTUAL_ENV THESIS_WS PYTHONNOUSERSITE PYTHONDONTWRITEBYTECODE ROS_HOME ROS_LOG_DIR OMP_NUM_THREADS MKL_NUM_THREADS'.split())
OPTIONAL_KEYS={'DISPLAY','XDG_RUNTIME_DIR','TZ','LC_ALL','ROS_VERSION','ROS_PYTHON_VERSION','ROS_ETC_DIR'}
FORBIDDEN=set('LD_PRELOAD LD_AUDIT LD_DEBUG LD_PROFILE LD_ORIGIN_PATH PYTHONHOME PYTHONSTARTUP PYTHONUSERBASE PYTHONINSPECT PYTHONWARNINGS PYTHONPYCACHEPREFIX GCONV_PATH LOCPATH LIBGL_DRIVERS_PATH LIBGL_ALWAYS_SOFTWARE MESA_LOADER_DRIVER_OVERRIDE __GLX_VENDOR_LIBRARY_NAME __EGL_VENDOR_LIBRARY_FILENAMES QT_PLUGIN_PATH QML2_IMPORT_PATH BASH_ENV ENV'.split())
BOOTSTRAP_ENV={'HOME':'/home/robot','USER':'robot','LOGNAME':'robot','LANG':'C.UTF-8','PATH':'/usr/bin:/bin'}

def validate_spec(spec):
    if type(spec)is not dict or set(spec)!={'schema','values','forbidden_keys'} or spec['schema']!=PROTOCOL:raise ValueError('closed environment schema required')
    values=spec['values']
    if type(values)is not dict or not BASE_KEYS<=set(values) or set(values)-BASE_KEYS-OPTIONAL_KEYS:raise ValueError('environment key set differs')
    if any(type(v)is not str or '\0' in v for v in values.values()):raise ValueError('environment values must be NUL-free strings')
    if spec['forbidden_keys']!=sorted(FORBIDDEN) or set(values)&FORBIDDEN:raise ValueError('forbidden environment key')
    fixed={'THESIS_WS':str(WORKSPACE),'VIRTUAL_ENV':str(WORKSPACE/'.venv'),'PYTHONNOUSERSITE':'1','PYTHONDONTWRITEBYTECODE':'1',
      'ROS_MASTER_URI':'http://127.0.0.1:14830','GAZEBO_MASTER_URI':'http://127.0.0.1:14831',
      'ROS_IP':'127.0.0.1','ROS_HOSTNAME':'127.0.0.1','GAZEBO_IP':'127.0.0.1','GAZEBO_MODEL_DATABASE_URI':MODEL_DATABASE,
      'OMP_NUM_THREADS':'1','MKL_NUM_THREADS':'1'}
    if any(values.get(k)!=v for k,v in fixed.items()):raise ValueError('fixed execution environment value differs')
    if not values['PATH'].startswith(str(WORKSPACE/'.venv/bin')+':'):raise ValueError('explicit virtualenv PATH required')
    return dict(values)

def make_spec(sourced):
    values={k:v for k,v in sourced.items() if k in BASE_KEYS|OPTIONAL_KEYS and v is not None}
    values.update(ROS_MASTER_URI='http://127.0.0.1:14830',GAZEBO_MASTER_URI='http://127.0.0.1:14831',ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1',GAZEBO_IP='127.0.0.1',
      GAZEBO_MODEL_DATABASE_URI=MODEL_DATABASE,ROS_HOME=str(WORKSPACE/'.astra_runtime/n06/ros_home'),ROS_LOG_DIR=str(WORKSPACE/'.astra_runtime/n06/roslog'),OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
    result={'schema':PROTOCOL,'values':values,'forbidden_keys':sorted(FORBIDDEN)};validate_spec(result);return result

def expected_environment(spec,closure_sha):
    if type(closure_sha)is not str or len(closure_sha)!=64 or any(c not in '0123456789abcdef' for c in closure_sha):raise ValueError('exact runtime closure SHA required')
    result=validate_spec(spec);result['ASTRA_N06_RUNTIME_CLOSURE_SHA256']=closure_sha;return result

def validate_current(data,environ=None):
    expected=expected_environment(data['runtime']['execution_environment'],data['release']['runtime_closure']['sha256'])
    actual=dict(os.environ if environ is None else environ)
    if actual!=expected:
        keys=sorted(set(actual)^set(expected));changed=sorted(k for k in set(actual)&set(expected) if actual[k]!=expected[k])
        raise ValueError('execution environment mismatch keys='+repr(keys)+' changed='+repr(changed))
    return expected

def child_environment(data,root=None,nonce=None):
    values=validate_current(data)
    if root is not None:
        root=Path(root)
        if not root.is_absolute() or root.parent!=WORKSPACE.parent/'runs' or type(nonce)is not str or len(nonce)!=48 or any(c not in '0123456789abcdef' for c in nonce):raise ValueError('canonical run and nonce required')
        values.update(ROS_HOME=str(root/'ros_home'),ROS_LOG_DIR=str(root/'roslog'),ASTRA_N06_CHILD_NONCE=nonce)
    elif nonce is not None:raise ValueError('nonce requires run')
    return values

@contextlib.contextmanager
def assessment_environment(data,root,nonce):
    values=child_environment(data,root,nonce);previous=dict(os.environ)
    os.environ.clear();os.environ.update(values)
    try:yield
    finally:os.environ.clear();os.environ.update(previous)
