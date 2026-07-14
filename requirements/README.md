# Thesis Python environment

Activate the thesis environment from a fresh terminal with:

```bash
source /home/robot/robot_ws_base_rl/scripts/activate_thesis_env.sh
```

The activation script clears inherited ROS/catkin/Python overlay paths before
loading ROS Noetic, this workspace, and `.venv`. It also enables
`PYTHONNOUSERSITE=1`, `PYTHONDONTWRITEBYTECODE=1`, and
`PIP_REQUIRE_VIRTUALENV=true`.

`thesis-base.txt` freezes the small T01 configuration/test toolchain. ROS Python
packages such as `rospy`, `catkin_pkg`, and message modules remain supplied by
Ubuntu/ROS, not pip.

The T09 CPU-only RL stack is frozen in `thesis-rl-lock.txt` and audited with
installed distribution RECORD hashes in `thesis-rl-lock.yaml`. Verify it after
activation with `python scripts/verify_rl_stack.py`. The external catkin/arena
workspaces and the repository-bundled legacy SB3 are not the T09 runtime.
