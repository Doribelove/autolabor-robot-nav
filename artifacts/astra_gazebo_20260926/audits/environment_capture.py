#!/usr/bin/env python3
"""Record explicit, non-sensitive static environment probes after Astra build."""
import datetime
import hashlib
import importlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys

import yaml

STAGE = Path(__file__).resolve().parents[1]
WS = STAGE / 'workspace'

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def probe(args):
    done = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          text=True, timeout=40, check=False)
    return {'command': args, 'exit_code': done.returncode, 'output': done.stdout}

commands = {
    'os': ['cat', '/etc/os-release'],
    'kernel': ['uname', '-srmo'],
    'cpu': ['lscpu'],
    'memory': ['free', '-b'],
    'disk': ['df', '-B1', str(STAGE)],
    'gcc': ['gcc', '--version'],
    'cmake': ['cmake', '--version'],
    'ros': ['rosversion', 'roslaunch'],
    'catkin': ['rosversion', 'catkin'],
    'gazebo': ['pkg-config', '--modversion', 'gazebo'],
    'teb': ['rosversion', 'teb_local_planner'],
    'dwa': ['rosversion', 'dwa_local_planner'],
    'teb_package': ['rospack', 'find', 'teb_local_planner'],
    'm2_package': ['rospack', 'find', 'm2_gazebo'],
    'gpu': ['nvidia-smi', '--query-gpu=name,driver_version,memory.total,memory.used', '--format=csv,noheader'],
    'opengl': ['glxinfo', '-B'],
    'display': ['xrandr', '--current'],
    'ffmpeg': ['ffmpeg', '-hide_banner', '-version'],
    'rl_lock': [sys.executable, '-B', str(WS/'scripts/verify_rl_stack.py')],
    'pip_check': [sys.executable, '-B', '-m', 'pip', 'check'],
    'camera_plugin_dependencies': ['ldd', '/opt/ros/noetic/lib/libgazebo_ros_camera.so'],
}
probes = {name: probe(args) for name, args in commands.items()}
libraries = {}
for name in ['m2_ackermann_plugin', 'v2_trajectory_actor_plugin',
             'v3_reactive_actor_plugin', 'v3_episode_recorder_plugin', 'teb_local_planner']:
    path = WS / 'devel/lib' / ('lib' + name + '.so')
    libraries[name] = {'path': str(path), 'sha256': sha(path), 'ldd': probe(['ldd', str(path)])}

modules = {}
for name in ['torch', 'numpy', 'gymnasium', 'stable_baselines3', 'scipy', 'PIL',
             'rospy', 'tf2_ros', 'v3_scope_planner', 'v5_e2e_scope_rl', 'm2_gazebo']:
    module = importlib.import_module(name)
    modules[name] = {'origin': module.__file__, 'version': str(getattr(module, '__version__', 'ROS/source'))}
import torch
assert all(p['exit_code'] == 0 for p in probes.values()), probes
assert 'not found' not in probes['camera_plugin_dependencies']['output']
assert all('not found' not in p['ldd']['output'] and p['ldd']['exit_code'] == 0 for p in libraries.values())
assert probes['teb']['output'].strip() == '0.8.4'
assert probes['teb_package']['output'].strip() == str(WS/'src/navigation_arena/forks/navigation/local_planner/teb')
assert probes['m2_package']['output'].strip() == str(WS/'src/simulation/m2_gazebo')
assert not torch.cuda.is_available()
for path in sys.path:
    assert '/home/robot/arena_ws' not in path and '/home/robot/catkin_ws' not in path
assert str(WS/'devel') in os.environ['CMAKE_PREFIX_PATH'].split(':')
assert '/home/robot/robot_ws/devel' not in os.environ['CMAKE_PREFIX_PATH']

manifest = {
    'schema_version': '1.0', 'stage': 'astra_gazebo_20260926',
    'captured_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
    'status': 'clean_build_static_environment_verified',
    'workspace': str(WS), 'python': platform.python_version(),
    'python_executable': sys.executable, 'torch_cuda_available': torch.cuda.is_available(),
    'source_snapshot_manifest_sha256': json.loads((STAGE/'bootstrap/snapshot_report.json').read_text())['manifest_sha256'],
    'source_order': ['/opt/ros/noetic/setup.bash', '/usr/share/gazebo/setup.sh', str(WS/'devel/setup.bash'), str(WS/'.venv/bin/activate')],
    'build_packages': ['autolabor_canbus_driver','m2_gazebo','nav_world_model','teb_local_planner',
                       'teb_mode_manager','teb_rl_tuner','thesis_experiment','v3_scope_planner','v5_e2e_scope_rl'],
    'build_exit_code': 0,
    'build_command': 'ASTRA_BOOTSTRAP_VENV=/home/robot/robot_ws_base_rl/.venv ./scripts/astra_build.sh',
    'build_log': 'logs/build_20260926_01.log',
    'static_test_log': 'logs/build_20260926_02_static_tests.log',
    'static_tests': {'command': 'devel/lib/m2_gazebo/test_ackermann_kinematics',
                     'exit_code': 0, 'passed': 18, 'failed': 0,
                     'suites': ['AckermannKinematics', 'ActuatorDynamics', 'TrajectoryActor', 'V3ReactiveActor']},
    'audit_failures_preserved': [
        {'log': 'logs/build_20260926_03_environment_capture.log',
         'cause': 'gzserver --version prints valid 11.15.1 but returns 255; capture now uses pkg-config --modversion gazebo'},
        {'log': 'audits/environment_smoke_static_entry.log',
         'cause': 'First static-only launch used an empty required actor_checkpoint_path; repeated with nonempty placeholder. Camera ldd lacked standard Gazebo plugin directory; fixed by sourcing the installed Gazebo setup before fresh devel.'}],
    'python_modules': modules, 'python_path': sys.path,
    'sanitized_overlay': {name: os.environ.get(name, '') for name in
        ['CMAKE_PREFIX_PATH','ROS_PACKAGE_PATH','PYTHONPATH','LD_LIBRARY_PATH','GAZEBO_PLUGIN_PATH',
         'ROS_MASTER_URI','GAZEBO_MASTER_URI','ROS_HOME','ROS_LOG_DIR','THESIS_WS']},
    'probes': probes, 'built_libraries': libraries,
    'bindings': {str(path.relative_to(STAGE)): sha(path) for path in
        [WS/'scripts/astra_env.sh', WS/'scripts/astra_build.sh', WS/'requirements/thesis-base.txt',
         WS/'requirements/thesis-rl-lock.txt', WS/'requirements/thesis-rl-lock.yaml',
         STAGE/'logs/build_20260926_01.log', STAGE/'logs/build_20260926_02_static_tests.log']},
    'scope': {'gazebo_started_by_environment_audit': False, 'training_started': False,
              'hardware_accessed': False, 'stable_workspace_modified': False,
              'runtime_simulation_verification': 'pending_separate_smoke',
              'hardware_validated': 'pending_no_evidence'},
    'limitations': [
        'CUDA-capable display GPU is present; frozen Python stack remains CPU-only.',
        'M2 Gazebo plugin applies kinematic SetWorldPose with actuator dynamics, not calibrated tire/friction physics.',
        'Xvfb is absent; NVIDIA DISPLAY :0 rendering is available, actual Gazebo screenshots still require runtime capture.',
        'Core package clean build only; full 76-package workspace is not claimed built.',
    ],
}
(STAGE/'environment_manifest.yaml').write_text(yaml.safe_dump(manifest, sort_keys=False), encoding='utf-8')
print(json.dumps({'status': manifest['status'], 'output': str(STAGE/'environment_manifest.yaml'),
                  'sha256': sha(STAGE/'environment_manifest.yaml')}, sort_keys=True))
