#!/usr/bin/env bash
stage_root=/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926
cd "$stage_root" || exit 90
source workspace/scripts/astra_env.sh || exit 91
release_sha=e9879ee6ca8ca050b9ee036db87c4728f5f1aa38895b69c2ca2d429bbcf6c628
workspace/.venv/bin/python workspace/scripts/astra_gazebo/run_navigation_n05.py \
  --execute --release contracts/navigation_n05_release.json --release-sha256 "$release_sha" \
  > logs/navigation_n05_execute_outer.log 2>&1
result=$?
printf '%s\n' "$result" > queue/navigation_n05_outer_exit_code.txt
exit "$result"
