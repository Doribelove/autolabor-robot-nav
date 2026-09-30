# N06 r05 restricted startup environment, zero execution authority

Seven new sources comprise the four `*_r05` manager modules and `navigation_n06_environment_r05.py`, `navigation_n06_bootstrap_r05.py`, `navigation_n06_entry_r05.sh`. The physical/controller/scene/evidence code and all r01–r04 sources remain frozen. The r04 late-assessment interruption repair remains.

The required future entry starts with a clean environment before any shell setup or Python manager import:

```bash
/usr/bin/env -i HOME=/home/robot USER=robot LOGNAME=robot LANG=C.UTF-8 PATH=/usr/bin:/bin /bin/bash --noprofile --norc /home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/workspace/scripts/astra_gazebo/navigation_n06_entry_r05.sh --verify --release /ABS/FUTURE_RELEASE.json --release-sha256 EXACT_SHA
```

Only a later exact release may replace `--verify` with `--execute`. `--inspect-bootstrap` additionally inspects the verified entry without importing/spawning the manager. This task creates no real release, registration or execution. Temporary synthetic documents are tests only.

The shell does not source setup. It executes `env -i /usr/bin/python3 -I -S` with five fixed startup fields. The standard-library bootstrap refuses any extra startup key, nonisolated Python or site loading, verifies release/closure bytes and all four entry-source bindings, validates the closed environment schema, then `execve`s explicit W/.venv/bin/python with that exact dictionary. This established environment precedes ROS imports. The dynamic `ASTRA_N06_RUNTIME_CLOSURE_SHA256` value derives from the actual bound closure and avoids a hash cycle.

The runtime closure must include `execution_environment` with schema `astra_n06_fixed_environment_r05`, `values`, and the exact `forbidden_keys` list. Values include PATH/PYTHONPATH/LD_LIBRARY_PATH, ROS/Gazebo paths, locale, HOME and declared DISPLAY/XDG fields. Optional is not inherited: absent and empty are distinct. Unknown keys and loader/Python/render injection fields are rejected. Manager preflight checks the complete key/value dictionary before lock/ledger/allocation. Every worker/roslaunch spawn checks it again and constructs a fresh dictionary. Run ROS_HOME/ROS_LOG_DIR and the 48-hex ownership nonce are the only run-specific fields. The single-threaded assessment context installs the same exact run dictionary and restores the parent even after exceptions. ROS launch node namespaces/env_args and the Gazebo shell's transformations are separately included in runtime closure evidence.

`GAZEBO_MODEL_DATABASE_URI=http://127.0.0.1:9` is common to every method. Eight world/URDF inputs contain no nonempty external URI. The real gazebo_ros wrapper, with only its final gzserver resolution replaced by a non-Gazebo print shim, was observed for all eight units: it repeats system paths but preserves this nonempty localhost URI. The old empty value restores http://models.gazebosim.org; that actual-shell negative is retained. The shim occupies a sole temporary PATH prefix; the saved observation and explicit prefix removal distinguish measured environment from normalized real-exec environment. No Gazebo, ROS node, network model fetch or seed use occurs in this probe.

The helper returns complete environment equality, not a claim of cryptographic proof against a malicious same-account launcher or a compromised OS/env executable. The prescribed outer clean invocation is part of the exact reviewed launch workflow; clearing variables late inside an already-loaded manager is not an equivalent supported entry. Existing system loader files, package hooks and runtime libraries still require disk binding and later actual mapped-ELF verification. Disk closure and independent r05 review remain pending, and no live qualification is asserted.
