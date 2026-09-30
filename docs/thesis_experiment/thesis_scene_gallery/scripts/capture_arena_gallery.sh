#!/usr/bin/env bash
set -euo pipefail

# Capture static Arena-Rosnav-3D environment illustrations from temporary,
# sanitised world copies. No Arena task generator, planner, evaluator, or
# navigation robot is started, and no upstream source world is modified.

gallery_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_root="$(cd "${gallery_root}/../../.." && pwd)"
arena_root="${workspace_root}/src/navigation_arena/arena-rosnav-3D"
arena_world_root="${arena_root}/simulator_setup/worlds"
gazebo_resource_root="/usr/share/gazebo-11"
image_root="${gallery_root}/arena_images"
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

if [[ ! -d "${gazebo_resource_root}" ]]; then
  echo "Gazebo Classic resource root is unavailable: ${gazebo_resource_root}" >&2
  exit 2
fi

export GAZEBO_MASTER_URI=http://127.0.0.1:11745
export GAZEBO_MODEL_DATABASE_URI=""
unset GAZEBO_PLUGIN_PATH GAZEBO_RESOURCE_PATH

scenes=(aws_house bookstore hospital factory small_warehouse)
labels=(01_residential 02_bookstore 03_hospital 04_factory 05_small_warehouse)
cameras=(
  "-4.4965503 -0.4874 1.508684 0 0 0"
  "-5.73 3.76 8.10 0 1.1316 -0.4398"
  "-27 -41 39 0 0.68 0.60"
  "-15 -24 30 0 0.68 0.68"
  "0 0 30 0 1.570796 0"
)
seeds=(991001 991002 991003 991004 991005)

for requested_scene in "$@"; do
  known_scene=false
  for scene_name in "${scenes[@]}"; do
    if [[ "${requested_scene}" == "${scene_name}" ]]; then
      known_scene=true
      break
    fi
  done
  if [[ "${known_scene}" != true ]]; then
    echo "Unknown Arena scene: ${requested_scene}" >&2
    echo "Choose from: ${scenes[*]}" >&2
    exit 2
  fi
done

server_pid=""
client_pid=""

scene_requested() {
  local requested
  if (( $# == 1 )); then
    return 0
  fi
  for requested in "${@:2}"; do
    [[ "${requested}" == "$1" ]] && return 0
  done
  return 1
}

find_largest_gazebo_window() {
  local id info width height area
  local best_area=0
  local best_id=""

  while read -r id; do
    [[ -n "${id}" ]] || continue
    info="$(xwininfo -id "${id}" 2>/dev/null || true)"
    width="$(awk -F: '/Width:/{gsub(/ /,"",$2); print $2}' <<<"${info}")"
    height="$(awk -F: '/Height:/{gsub(/ /,"",$2); print $2}' <<<"${info}")"
    [[ "${width}" =~ ^[0-9]+$ && "${height}" =~ ^[0-9]+$ ]] || continue
    area=$((width * height))
    if (( area > best_area )); then
      best_area=${area}
      best_id=${id}
    fi
  done < <(xwininfo -root -tree 2>/dev/null | awk '/"Gazebo":/{print $1}' || true)

  printf '%s\n' "${best_id}"
}

stop_scene() {
  for pid in "${client_pid}" "${server_pid}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -INT -- "-${pid}" 2>/dev/null || true
    fi
  done
  for _ in $(seq 1 60); do
    server_alive=false
    client_alive=false
    [[ -n "${server_pid}" ]] && kill -0 "${server_pid}" 2>/dev/null && server_alive=true
    [[ -n "${client_pid}" ]] && kill -0 "${client_pid}" 2>/dev/null && client_alive=true
    if [[ "${server_alive}" == false && "${client_alive}" == false ]]; then
      break
    fi
    sleep 0.25
  done
  for pid in "${client_pid}" "${server_pid}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -TERM -- "-${pid}" 2>/dev/null || true
      sleep 0.5
    fi
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -KILL -- "-${pid}" 2>/dev/null || true
    fi
    [[ -n "${pid}" ]] && wait "${pid}" 2>/dev/null || true
  done
  server_pid=""
  client_pid=""
}

cleanup() {
  stop_scene
  rm -rf "${task_tmp}"
}
trap cleanup EXIT INT TERM

for index in "${!scenes[@]}"; do
  scene="${scenes[$index]}"
  scene_requested "${scene}" "$@" || continue
  label="${labels[$index]}"
  source_world="${arena_world_root}/${scene}/worlds/${scene}.world"
  source_hash_before="$(sha256sum "${source_world}" | awk '{print $1}')"
  model_root="${arena_world_root}/${scene}/models"
  snapshot_world="${task_tmp}/${scene}.world"
  server_log="${task_tmp}/${scene}_server.log"
  client_log="${task_tmp}/${scene}_client.log"

  sanitize_args=(
    --source "${source_world}"
    --output "${snapshot_world}"
    --camera-pose "${cameras[$index]}"
  )
  if [[ "${scene}" == "bookstore" ]]; then
    sanitize_args+=(--drop-model turtlebot3_waffle_pi)
  fi
  if [[ "${scene}" == "factory" ]]; then
    sanitize_args+=(
      --drop-model deletion_wall
      --drop-model deletion_wall_clone
      --drop-model logical_camera_ariac_1
      --drop-model logical_camera_ariac_2
      --drop-model logical_camera_ariac_3
      --drop-model logical_camera_ariac_4
      --drop-model logical_camera_ariac_5
    )
  fi

  python3 "${gallery_root}/scripts/sanitize_arena_world.py" "${sanitize_args[@]}" \
    >"${task_tmp}/${scene}_sanitize.log"

  export GAZEBO_MODEL_PATH="${model_root}"
  # Some AWS ROS1 world assets use file://models/... mesh URIs rather
  # than model:// URIs, so the directory containing models must also be
  # an explicit Gazebo resource root.
  export GAZEBO_RESOURCE_PATH="${arena_world_root}/${scene}:${gazebo_resource_root}"
  setsid gzserver -u --seed "${seeds[$index]}" --verbose "${snapshot_world}" \
    >"${server_log}" 2>&1 &
  server_pid=$!

  server_ready=false
  for _ in $(seq 1 180); do
    if rg -q "Loading world file" "${server_log}" && kill -0 "${server_pid}" 2>/dev/null; then
      server_ready=true
      break
    fi
    sleep 0.25
  done
  if [[ "${server_ready}" != true ]]; then
    echo "Arena world did not load for ${scene}." >&2
    sed -n '1,240p' "${server_log}" >&2
    exit 3
  fi

  setsid gzclient --verbose >"${client_log}" 2>&1 &
  client_pid=$!

  client_ready=false
  window_id=""
  for _ in $(seq 1 180); do
    window_id="$(find_largest_gazebo_window)"
    if [[ -n "${window_id}" ]] && kill -0 "${client_pid}" 2>/dev/null; then
      window_info="$(xwininfo -id "${window_id}" 2>/dev/null || true)"
      window_w="$(awk -F: '/Width:/{gsub(/ /,"",$2); print $2}' <<<"${window_info}")"
      window_h="$(awk -F: '/Height:/{gsub(/ /,"",$2); print $2}' <<<"${window_info}")"
      if [[ "${window_w}" =~ ^[0-9]+$ && "${window_h}" =~ ^[0-9]+$ ]] && \
         (( window_w >= 1200 && window_h >= 700 )); then
        client_ready=true
        break
      fi
    fi
    sleep 0.25
  done
  if [[ "${client_ready}" != true ]]; then
    echo "Gazebo client did not become capture-ready for ${scene}." >&2
    sed -n '1,240p' "${client_log}" >&2
    exit 4
  fi

  # Large Arena meshes continue streaming after the client window appears.
  sleep 16
  window_id="$(find_largest_gazebo_window)"
  if [[ -z "${window_id}" ]]; then
    echo "Gazebo capture window disappeared for ${scene}." >&2
    exit 5
  fi
  python3 "${gallery_root}/scripts/raise_x11_window.py" "${window_id}"
  sleep 2

  window_info="$(xwininfo -id "${window_id}")"
  window_x="$(awk -F: '/Absolute upper-left X/{gsub(/ /,"",$2); print $2}' <<<"${window_info}")"
  window_y="$(awk -F: '/Absolute upper-left Y/{gsub(/ /,"",$2); print $2}' <<<"${window_info}")"
  window_w="$(awk -F: '/Width:/{gsub(/ /,"",$2); print $2}' <<<"${window_info}")"
  window_h="$(awk -F: '/Height:/{gsub(/ /,"",$2); print $2}' <<<"${window_info}")"

  viewport_x=$((window_x + 279))
  viewport_y=$((window_y + 99))
  viewport_w=$((window_w - 308))
  viewport_h=$((window_h - 177))

  ffmpeg -y -hide_banner -loglevel error \
    -f x11grab -draw_mouse 0 \
    -video_size "${viewport_w}x${viewport_h}" \
    -i ":0.0+${viewport_x},${viewport_y}" \
    -frames:v 1 "${raw_root}/${label}.png"

  ffmpeg -y -hide_banner -loglevel error \
    -i "${raw_root}/${label}.png" \
    -vf "scale=2400:-2:flags=lanczos" \
    "${image_root}/${label}.png"

  stop_scene
  source_hash_after="$(sha256sum "${source_world}" | awk '{print $1}')"
  if [[ "${source_hash_after}" != "${source_hash_before}" ]]; then
    echo "Source world changed during capture: ${source_world}" >&2
    exit 6
  fi
  echo "captured ${label}"
done

bash "${gallery_root}/scripts/build_arena_overview.sh"
echo "Arena gallery capture complete"
