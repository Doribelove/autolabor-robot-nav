#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PRIVATE_SETUP="${PRIVATE_SETUP:-${WORKSPACE_ROOT}/.deps/setup.bash}"
ENV_DIR="${FOD_YOLO_ENV:-${WORKSPACE_ROOT}/.venv/fod_yolo}"
PACKAGE_ROOT="${WORKSPACE_ROOT}/src/application/autolabor_fod_vision"

if [[ ! -x "${ENV_DIR}/bin/python3" ]]; then
  echo "Missing ${ENV_DIR}; run scripts/setup_fod_yolo_env.sh first." >&2
  exit 2
fi

if [[ -f "$PRIVATE_SETUP" ]]; then
  source "$PRIVATE_SETUP"
else
  source /opt/ros/noetic/setup.bash
  if [[ -f "${WORKSPACE_ROOT}/devel/setup.bash" ]]; then
    source "${WORKSPACE_ROOT}/devel/setup.bash"
  fi
fi

export PYTHONPATH="${PACKAGE_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}"
exec "${ENV_DIR}/bin/python3" \
  "${PACKAGE_ROOT}/scripts/yolo_smoke_test.py" "$@"
