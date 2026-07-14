#!/usr/bin/env bash
set -euo pipefail

WORKSPACE=/home/robot/robot_ws_base_rl
ROOT="$WORKSPACE/artifacts/t12/closed_loop"
SCENES="$WORKSPACE/experiments/manifests/t12/closed_loop_scenes.yaml"
cd "$WORKSPACE"
source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"
mkdir -p "$ROOT/runs" "$ROOT/logs" "$ROOT/failed_attempts"

wait_for_shutdown() {
  local deadline=$((SECONDS + 30))
  while (( SECONDS < deadline )); do
    if ! pgrep -f 't11_formal_run.py|/gazebo_ros/gzserver|/move_base/move_base' >/dev/null 2>&1 \
       && ! ss -ltn 2>/dev/null | grep -q ':11311 '; then
      return 0
    fi
    sleep 1
  done
  return 1
}

valid_run() {
  "$WORKSPACE/.venv/bin/python" - "$1" "${2:-}" <<'PY'
import csv
from pathlib import Path
import sys, yaml
p=Path(sys.argv[1])/"t11_run_report.yaml"
if not p.is_file(): raise SystemExit(1)
r=yaml.safe_load(p.read_text())
valid = r.get("task")=="T12" and r.get("passed") is True and r.get("episode_count")==10
if valid and sys.argv[2] == "t12_safety":
    rows=list(csv.DictReader((p.parent/"episodes.csv").open()))
    valid=all(row["termination_reason"] != "interface_fault" for row in rows)
raise SystemExit(0 if valid else 1)
PY
}

run_one() {
  local method="$1" mode="$2" seed="$3" eval_seed="$4"
  local source="$WORKSPACE/artifacts/t11/runs/t11_semantic_eta_fullsafety_seed${seed}"
  local checkpoint
  checkpoint=$("$WORKSPACE/.venv/bin/python" -c \
    "import yaml; print(yaml.safe_load(open('$source/model_selection.yaml'))['selected_checkpoint'])")
  local run_id="t12_${method}_seed${seed}" output="$ROOT/runs/t12_${method}_seed${seed}"
  if valid_run "$output" "$method"; then
    echo "T12 skip validated run: $run_id"
    return 0
  fi
  local attempt stamp
  for attempt in 1 2 3; do
    wait_for_shutdown
    if [[ -e "$output" ]]; then
      stamp=$(date +%Y%m%dT%H%M%S)
      mv "$output" "$ROOT/failed_attempts/${run_id}_${stamp}_attempt${attempt}"
    fi
    echo "T12 running $run_id attempt $attempt/3"
    if roslaunch thesis_experiment t11_formal_run.launch \
      gui:=false gazebo_seed:="$eval_seed" task:=T12 \
      algorithm:=RL-TEB-Semantic-Eta training_seed:="$seed" phase:=ablation \
      safety_mode:="$mode" checkpoint:="$checkpoint" run_id:="$run_id" \
      output_dir:="$output" scene_manifest:="$SCENES" \
      evaluation_seed_override:="$eval_seed" \
      >"$ROOT/logs/${run_id}_attempt${attempt}.log" 2>&1
    then
      if valid_run "$output" "$method"; then return 0; fi
    fi
    wait_for_shutdown || true
  done
  echo "T12 run failed after retries: $run_id" >&2
  return 1
}

for seed in 101 102; do
  eval_seed=$((300 + seed))
  run_one old_full_safety FullSafety "$seed" "$eval_seed"
  run_one t12_safety T12Safety "$seed" "$eval_seed"
  run_one projection_only ProjectionOnly "$seed" "$eval_seed"
done

wait_for_shutdown
rosrun thesis_experiment evaluate_t12_closed_loop.py
