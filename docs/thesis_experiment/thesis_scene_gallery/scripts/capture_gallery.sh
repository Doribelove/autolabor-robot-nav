#!/usr/bin/env bash
set -euo pipefail

# Capture presentation-only, paused Gazebo worlds.  This script does not start
# move_base, TEB, DWA, a learned policy, an evaluator, or an experiment runner.

gallery_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_root="$(cd "${gallery_root}/../../.." && pwd)"
world_root="${gallery_root}/source_worlds"
image_root="${gallery_root}/images"
raw_root="${image_root}/raw"
task_tmp="$(mktemp -d)"

mkdir -p "${raw_root}"

if pgrep -x rosmaster >/dev/null || pgrep -x gzserver >/dev/null || pgrep -x gzclient >/dev/null; then
  echo "Refusing capture: an existing ROS/Gazebo process is active." >&2
  exit 2
fi

if [[ -z "${DISPLAY:-}" ]]; then
  echo "Refusing capture: DISPLAY is unset." >&2
  exit 2
fi

source /opt/ros/noetic/setup.bash
source "${workspace_root}/devel/setup.bash"

export ROS_MASTER_URI=http://127.0.0.1:11611
export GAZEBO_MASTER_URI=http://127.0.0.1:11645

worlds=(
  01_m2_model_overview
  02_static_avoidance
  03_single_crossing
  04_partially_occluded_crossing
  05_head_on_yield
  06_sequential_interaction
  07_width_change_corridor
)

spawn_x=(0.0 -2.3 -2.4 -2.4 -2.3 -3.0 -3.2)
spawn_y=(0.0 0.0 0.0 0.0 0.0 0.0 0.0)
seeds=(990001 990002 990003 990004 990005 990006 990007)

launch_pid=""

stop_launch() {
  if [[ -z "${launch_pid}" ]]; then
    return
  fi
  if kill -0 "${launch_pid}" 2>/dev/null; then
    kill -INT -- "-${launch_pid}" 2>/dev/null || true
    for _ in $(seq 1 60); do
      if ! kill -0 "${launch_pid}" 2>/dev/null; then
        break
      fi
      sleep 0.25
    done
  fi
  if kill -0 "${launch_pid}" 2>/dev/null; then
    kill -TERM -- "-${launch_pid}" 2>/dev/null || true
    sleep 1
  fi
  if kill -0 "${launch_pid}" 2>/dev/null; then
    kill -KILL -- "-${launch_pid}" 2>/dev/null || true
  fi
  wait "${launch_pid}" 2>/dev/null || true
  launch_pid=""
}

cleanup() {
  stop_launch
  rm -rf "${task_tmp}"
}
trap cleanup EXIT INT TERM

for index in "${!worlds[@]}"; do
  scene="${worlds[$index]}"
  log_path="${task_tmp}/${scene}.log"

  setsid roslaunch m2_gazebo spawn_m2.launch \
    world:="${world_root}/${scene}.sdf" \
    gui:=true paused:=true \
    x:="${spawn_x[$index]}" y:="${spawn_y[$index]}" \
    seed:="${seeds[$index]}" server_required:=true \
    >"${log_path}" 2>&1 &
  launch_pid=$!

  ready=false
  for _ in $(seq 1 80); do
    if rg -q "Successfully spawned entity" "${log_path}" && \
       rg '"Gazebo":' <<<"$(xwininfo -root -tree 2>/dev/null)" >/dev/null; then
      ready=true
      break
    fi
    sleep 0.25
  done
  if [[ "${ready}" != true ]]; then
    echo "Gazebo did not become capture-ready for ${scene}." >&2
    sed -n '1,200p' "${log_path}" >&2
    exit 3
  fi

  sleep 2
  window_tree="$(xwininfo -root -tree)"
  window_id="$(awk '/"Gazebo":/{print $1; exit}' <<<"${window_tree}")"
  python3 "${gallery_root}/scripts/raise_x11_window.py" "${window_id}"
  sleep 1

  window_info="$(xwininfo -id "${window_id}")"
  window_x="$(awk -F: '/Absolute upper-left X/{gsub(/ /,"",$2); print $2}' <<<"${window_info}")"
  window_y="$(awk -F: '/Absolute upper-left Y/{gsub(/ /,"",$2); print $2}' <<<"${window_info}")"
  window_w="$(awk -F: '/Width:/{gsub(/ /,"",$2); print $2}' <<<"${window_info}")"
  window_h="$(awk -F: '/Height:/{gsub(/ /,"",$2); print $2}' <<<"${window_info}")"

  # Gazebo 11's rendering viewport is stable relative to the top-level window
  # on the frozen Ubuntu 20.04 desktop used by this project.
  viewport_x=$((window_x + 279))
  viewport_y=$((window_y + 99))
  viewport_w=$((window_w - 308))
  viewport_h=$((window_h - 177))

  ffmpeg -y -hide_banner -loglevel error \
    -f x11grab -draw_mouse 0 \
    -video_size "${viewport_w}x${viewport_h}" \
    -i ":0.0+${viewport_x},${viewport_y}" \
    -frames:v 1 "${raw_root}/${scene}.png"

  stop_launch
  echo "captured ${scene}"
done

# Clean paper-ready exports.  Scene plates use a wide crop that retains the
# complete route, corridor boundaries, and actor paths while removing unused
# top-down background.  The model view keeps the full angled viewport.
ffmpeg -y -hide_banner -loglevel error \
  -i "${raw_root}/01_m2_model_overview.png" \
  -vf "scale=2400:-2:flags=lanczos" \
  "${image_root}/01_m2_model_overview.png"

for scene in "${worlds[@]:1}"; do
  ffmpeg -y -hide_banner -loglevel error \
    -i "${raw_root}/${scene}.png" \
    -vf "crop=iw:1700:0:275,scale=2400:-2:flags=lanczos" \
    "${image_root}/${scene}.png"
done

bash "${gallery_root}/scripts/build_overview.sh"

echo "gallery capture complete"
