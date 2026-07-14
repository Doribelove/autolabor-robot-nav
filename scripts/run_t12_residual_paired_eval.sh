#!/usr/bin/env bash
set -euo pipefail

W=/home/robot/robot_ws_base_rl
ROOT="$W/artifacts/t12/residual_paired_eval"
SCENES="$W/experiments/manifests/t12/residual_paired_eval_scenes.yaml"
RESIDUAL="$W/config/thesis_experiments/t12_residual_training.yaml"
cd "$W"
source /opt/ros/noetic/setup.bash
source "$W/devel/setup.bash"
mkdir -p "$ROOT/runs" "$ROOT/logs" "$ROOT/failed_attempts"

wait_shutdown() {
  local end=$((SECONDS + 30))
  while (( SECONDS < end )); do
    if ! pgrep -f 't11_formal_run.py|/gazebo_ros/gzserver|/move_base/move_base' >/dev/null 2>&1 \
       && ! ss -ltn 2>/dev/null | grep -q ':11311 '; then
      return 0
    fi
    sleep 1
  done
  return 1
}

valid_run() {
  "$W/.venv/bin/python" - "$1" "$2" <<'PY'
from pathlib import Path
import sys,yaml
run=Path(sys.argv[1]); expected=sys.argv[2]
report=run/'t11_run_report.yaml'
if not report.is_file(): raise SystemExit(1)
state=yaml.safe_load(report.read_text())
ok=(state.get('task')=='T12' and state.get('passed') is True and
    state.get('episode_count')==4 and state.get('evaluation_policy')==expected)
raise SystemExit(0 if ok else 1)
PY
}

run_one() {
  local method="$1" policy="$2" safety="$3" seed="$4" eval_seed="$5"
  local id="t12e_${method}_seed${seed}"
  local out="$ROOT/runs/$id"
  if valid_run "$out" "$policy"; then echo "skip $id"; return 0; fi
  local checkpoint=""
  if [[ "$policy" == "checkpoint" ]]; then
    checkpoint=$("$W/.venv/bin/python" -c \
      "import yaml; print(yaml.safe_load(open('$W/artifacts/t12/residual_training/runs/t12_residual_training_seed${seed}/model_selection.yaml'))['selected_checkpoint'])")
  fi
  local attempt stamp
  for attempt in 1 2 3; do
    wait_shutdown
    if [[ -e "$out" ]]; then
      stamp=$(date +%Y%m%dT%H%M%S)
      mv "$out" "$ROOT/failed_attempts/${id}_${stamp}_a${attempt}"
    fi
    echo "running $id attempt $attempt/3"
    if roslaunch thesis_experiment t11_formal_run.launch gui:=false task:=T12 \
      gazebo_seed:="$eval_seed" algorithm:=RL-TEB-Semantic-Eta training_seed:="$seed" \
      phase:=ablation safety_mode:="$safety" semantic_mode:=residual_training \
      residual_config:="$RESIDUAL" evaluation_policy:="$policy" \
      initialize_residual_anchor:=true checkpoint:="$checkpoint" \
      run_id:="$id" output_dir:="$out" scene_manifest:="$SCENES" \
      evaluation_seed_override:="$eval_seed" >"$ROOT/logs/${id}_a${attempt}.log" 2>&1
    then
      if valid_run "$out" "$policy"; then return 0; fi
    fi
    wait_shutdown || true
  done
  echo "paired evaluation failed: $id" >&2
  return 1
}

for seed in 101 102; do
  eval_seed=$((600 + seed))
  run_one selected_sac checkpoint T12Safety "$seed" "$eval_seed"
  run_one zero_residual zero_residual T12Safety "$seed" "$eval_seed"
  run_one teb_tuned teb_tuned ProjectionOnly "$seed" "$eval_seed"
done

wait_shutdown
rosrun thesis_experiment evaluate_t12_residual_pair.py
