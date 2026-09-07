#!/usr/bin/env bash
set -euo pipefail
lab="$(realpath "$(dirname "$0")/..")"
[[ "$(basename "$lab")" == bpu_perception_20260906 ]] || exit 2
exec python3 "$lab/tools/service_manager.py" "${1:-status}"
