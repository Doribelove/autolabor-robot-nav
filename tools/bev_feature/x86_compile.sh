#!/usr/bin/env bash
# Recompile the pinned six-camera reference; this does not train a trash model.
set -euo pipefail
bev_bundle=$(cd -- "$(dirname -- "$0")/../.." && pwd)
if [[ $(uname -s) != Linux || $(uname -m) != x86_64 ]]; then
  echo '此入口用于原生 x86 Linux/WSL2。ARM 仿真诊断请用 x86_probe.py。' >&2
  exit 2
fi
cd -- "$bev_bundle"
bev_output="${1:-runtime/native_reference_$(date +%Y%m%d_%H%M%S)}"
mkdir -p validation
if [[ -x .venv/bin/python && ! -f /.dockerenv ]]; then
  bev_python="$bev_bundle/.venv/bin/python"
else
  bev_python=python3
fi
"$bev_python" tools/bev_feature/oe391_environment.py runtime --stage compiler \
  --output validation/native_x86_environment.json
"$bev_python" -u tools/bev_feature/compile_reference.py --output "$bev_output" --jobs 2 \
  2>&1 | tee validation/compile_console.log
echo "HBM 和报告位于 $bev_output；完成后还需 J6M 实测。"
