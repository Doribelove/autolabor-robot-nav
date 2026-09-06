#!/usr/bin/env bash
set -euo pipefail
lab=/map/robot_j6m_optimized_20260905/bpu_perception_20260906
[[ "$(realpath "$(dirname "$0")/..")" == "$lab" ]] || exit 2
exec python3 "$lab/tools/service_manager.py" "${1:-status}"
