#!/usr/bin/env bash
set -euo pipefail

WORKSPACE=/home/robot/robot_ws_base_rl
cd "$WORKSPACE"
source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"

mkdir -p artifacts/t11/batch_logs artifacts/t11/runs artifacts/t11/failed_attempts

report_is_valid() {
  local run_id="$1"
  "$WORKSPACE/.venv/bin/python" - "$run_id" <<'PY'
import pathlib
import sys
import yaml

run_id = sys.argv[1]
path = pathlib.Path("artifacts/t11/runs") / run_id / "t11_run_report.yaml"
if not path.is_file():
    raise SystemExit(1)
report = yaml.safe_load(path.read_text(encoding="utf-8"))
valid = (
    report.get("run_id") == run_id
    and report.get("status") == "passed"
    and report.get("passed") is True
    and report.get("run_validation", {}).get("valid") is True
)
raise SystemExit(0 if valid else 1)
PY
}

preserve_incomplete_run() {
  local run_id="$1"
  local attempt="$2"
  local output="$WORKSPACE/artifacts/t11/runs/$run_id"
  if [[ -e "$output" ]]; then
    local stamp
    stamp=$(date +%Y%m%dT%H%M%S)
    mv "$output" \
      "$WORKSPACE/artifacts/t11/failed_attempts/${run_id}_${stamp}_attempt${attempt}"
  fi
}

wait_for_t11_shutdown() {
  local deadline=$((SECONDS + 30))
  while (( SECONDS < deadline )); do
    if ! pgrep -f 't11_formal_run.py|/gazebo_ros/gzserver|gzserver .*obstacle_test.world|/move_base/move_base' \
         >/dev/null 2>&1 \
       && ! ss -ltn 2>/dev/null | grep -q ':11311 '; then
      return 0
    fi
    sleep 1
  done
  echo "T11 ROS/Gazebo processes did not shut down within 30 seconds" >&2
  return 1
}

run_training() {
  local algorithm="$1"
  local short_name="$2"
  local seed="$3"
  local run_id="t11_${short_name}_fullsafety_seed${seed}"
  if report_is_valid "$run_id"; then
    echo "T11 skip validated run: $run_id"
    return 0
  fi
  local attempt
  for attempt in 1 2 3; do
    wait_for_t11_shutdown
    preserve_incomplete_run "$run_id" "$attempt"
    if roslaunch thesis_experiment t11_formal_run.launch \
      gui:=false gazebo_seed:="$seed" algorithm:="$algorithm" \
      training_seed:="$seed" phase:=train safety_mode:=FullSafety \
      run_id:="$run_id" \
      output_dir:="$WORKSPACE/artifacts/t11/runs/$run_id" \
      >"$WORKSPACE/artifacts/t11/batch_logs/${run_id}_attempt${attempt}.log" 2>&1
    then
      if report_is_valid "$run_id"; then
        return 0
      fi
    fi
    echo "T11 invalid training run, retry $attempt/3: $run_id" >&2
    wait_for_t11_shutdown
  done
  return 1
}

run_ablation() {
  local seed="$1"
  local mode="$2"
  local suffix="$3"
  local source_run="t11_semantic_eta_fullsafety_seed${seed}"
  local checkpoint
  checkpoint=$(
    "$WORKSPACE/.venv/bin/python" -c \
      "import yaml; print(yaml.safe_load(open('$WORKSPACE/artifacts/t11/runs/$source_run/model_selection.yaml'))['selected_checkpoint'])"
  )
  local run_id="t11_semantic_eta_${suffix}_seed${seed}"
  if report_is_valid "$run_id"; then
    echo "T11 skip validated run: $run_id"
    return 0
  fi
  local attempt
  for attempt in 1 2 3; do
    wait_for_t11_shutdown
    preserve_incomplete_run "$run_id" "$attempt"
    if roslaunch thesis_experiment t11_formal_run.launch \
      gui:=false gazebo_seed:="$seed" algorithm:=RL-TEB-Semantic-Eta \
      training_seed:="$seed" phase:=ablation safety_mode:="$mode" \
      checkpoint:="$checkpoint" run_id:="$run_id" \
      output_dir:="$WORKSPACE/artifacts/t11/runs/$run_id" \
      >"$WORKSPACE/artifacts/t11/batch_logs/${run_id}_attempt${attempt}.log" 2>&1
    then
      if report_is_valid "$run_id"; then
        return 0
      fi
    fi
    echo "T11 invalid ablation run, retry $attempt/3: $run_id" >&2
    wait_for_t11_shutdown
  done
  return 1
}

for seed in 101 102 103 104 105; do
  run_training RL-TEB-Semantic-Eta semantic_eta "$seed"
  run_training RL-TEB-Direct-Theta direct_theta "$seed"
  run_ablation "$seed" ProjectionOnly projection_only
  run_ablation "$seed" NoSafety no_safety
  run_ablation "$seed" NoFallback no_fallback
done
