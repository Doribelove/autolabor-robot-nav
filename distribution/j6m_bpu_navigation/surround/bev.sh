#!/usr/bin/env bash
set -euo pipefail
bev_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
bev_root="$(cd -- "$bev_dir/../../.." && pwd)"
export PYTHONNOUSERSITE=1
export PYTHONPATH="$bev_root"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
cd -- "$bev_root"
bev_action="${1:-check}"
if [[ $# -gt 0 ]]; then shift; fi
if [[ "$bev_action" == "setup" || "$bev_action" == "download" ]]; then
  exec "${SURROUND_BEV_BOOTSTRAP_PYTHON:-python3}" -s "$bev_dir/environment.py" "$bev_action" "$@"
fi
bev_python="${SURROUND_BEV_PYTHON:-$bev_root/runtime/surround_env/bin/python}"
if [[ ! -x "$bev_python" ]]; then
  echo "BEV environment missing. Run: $0 setup" >&2
  exit 2
fi
if [[ "$bev_action" == "check" ]]; then
  exec "$bev_python" -s "$bev_dir/environment.py" check "$@"
fi
exec "$bev_python" -s -m distribution.j6m_bpu_navigation.surround.bev_cli "$bev_action" "$@"
