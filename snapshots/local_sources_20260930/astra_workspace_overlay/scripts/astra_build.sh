#!/usr/bin/env bash
# Rebuild without loading a historical workspace. Optional cache is read-only.
set -eo pipefail
ASTRA_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
cd "$ASTRA_WS"
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES
unset PKG_CONFIG_PATH PYTHONPATH LD_LIBRARY_PATH
unset GAZEBO_PLUGIN_PATH GAZEBO_MODEL_PATH GAZEBO_RESOURCE_PATH
unset VIRTUAL_ENV ROS_HOSTNAME ROS_IP
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
if [[ ! -f .venv/pyvenv.cfg ]]; then
  if [[ -n "${ASTRA_BOOTSTRAP_VENV:-}" ]]; then
    /usr/bin/python3 -m venv --without-pip --system-site-packages .venv
    ASTRA_CACHE_VENV="$(realpath "$ASTRA_BOOTSTRAP_VENV")"
    export ASTRA_CACHE_VENV ASTRA_WS
    /usr/bin/python3 -B - <<'PY'
import os
from pathlib import Path
import shutil

source = Path(os.environ['ASTRA_CACHE_VENV'])
target = Path(os.environ['ASTRA_WS']) / '.venv'
if source.resolve() == target.resolve():
    raise SystemExit('cache must differ from new environment')
shutil.copytree(source / 'lib/python3.8/site-packages',
                target / 'lib/python3.8/site-packages', dirs_exist_ok=True)
for entry in (source / 'bin').iterdir():
    if entry.is_symlink() or entry.name.startswith(('python', 'activate')):
        continue
    raw = entry.read_bytes()
    if not raw.startswith(b'#!') or b'python' not in raw.split(b'\n', 1)[0]:
        continue
    # Long interpreter paths exceed Linux shebang limits. Use a shell trampoline.
    body = raw.split(b'\n', 1)[1]
    interpreter = str(target / 'bin/python')
    header = "#!/bin/sh\n'''exec' \"{}\" \"$0\" \"$@\"\n' '''\n".format(interpreter)
    destination = target / 'bin' / entry.name
    destination.write_bytes(header.encode() + body)
    destination.chmod(entry.stat().st_mode & 0o777)
PY
  else
    /usr/bin/python3 -m virtualenv --system-site-packages .venv
    .venv/bin/python -m pip install -r requirements/thesis-base.txt
    .venv/bin/python -m pip install --extra-index-url https://download.pytorch.org/whl/cpu -r requirements/thesis-rl-lock.txt
  fi
fi
.venv/bin/python -B scripts/verify_rl_stack.py
.venv/bin/python -B -m pip check
source /opt/ros/noetic/setup.bash
catkin_make -j4 -l4 \
  -DCATKIN_WHITELIST_PACKAGES='autolabor_canbus_driver;m2_gazebo;nav_world_model;teb_local_planner;teb_mode_manager;teb_rl_tuner;thesis_experiment;v3_scope_planner;v5_e2e_scope_rl' \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_BUILD_TYPE=Release
source "$ASTRA_WS/scripts/astra_env.sh"
python -B scripts/verify_rl_stack.py
python -B -m pip check
python -B - <<'PY'
import os
import rospy, tf2_ros, torch
import autolabor_canbus_driver.msg
import nav_world_model.msg
import v3_scope_planner.msg
import v5_e2e_scope_rl.benchmark_runtime
from pathlib import Path
assert str(Path(torch.__file__).resolve()).startswith(os.environ['THESIS_WS'] + '/.venv/')
for path in __import__('sys').path:
    assert '/home/robot/catkin_ws' not in path and '/home/robot/arena_ws' not in path, path
print('Astra Python/ROS imports and workspace isolation: pass')
PY
test "$(rospack find m2_gazebo)" = "$ASTRA_WS/src/simulation/m2_gazebo"
test "$(rospack find teb_local_planner)" = "$ASTRA_WS/src/navigation_arena/forks/navigation/local_planner/teb"
test "$(rosversion teb_local_planner)" = '0.8.4'
printf 'Astra build and static environment verification passed: %s\n' "$ASTRA_WS"
