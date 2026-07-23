#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_DIR="${FOD_YOLO_ENV:-/home/robot/python_env/fod_yolo}"
PACKAGE_ROOT="${WORKSPACE_ROOT}/src/application/autolabor_fod_vision"

if [[ ! -x "${ENV_DIR}/bin/python3" ]]; then
  echo "Missing ${ENV_DIR}; run scripts/setup_fod_yolo_env.sh first." >&2
  exit 2
fi

export PYTHONPATH="${PACKAGE_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}"
exec "${ENV_DIR}/bin/python3" \
  "${PACKAGE_ROOT}/scripts/yolo_smoke_test.py" "$@"
