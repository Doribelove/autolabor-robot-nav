#!/usr/bin/env bash
# Minimal native x86 CPU compiler environment; no ROS, Torch, or GPU required.
set -euo pipefail
bev_bundle=$(cd -- "$(dirname -- "$0")/../.." && pwd)
if [[ $(uname -s) != Linux || $(uname -m) != x86_64 ]]; then
  echo '请在 Intel/AMD 电脑的 Linux 或 WSL2 Ubuntu 中执行。' >&2
  exit 2
fi
if ! command -v python3.10 >/dev/null; then
  echo '缺 Python 3.10；Ubuntu 22.04 可安装 python3.10-venv、python3-pip 和 libgomp1。' >&2
  exit 2
fi
cd -- "$bev_bundle"
sha256sum -c SHA256SUMS
python3.10 -m venv .venv
.venv/bin/python -m pip install --only-binary=:all: 'numpy==1.26.4' 'netron>=6.0.2' \
  wheels/hbdk4_compiler-4.11.11-cp310-cp310-manylinux_2_28_x86_64.whl \
  wheels/hbdk4_march-4.11.11-cp310-abi3-manylinux_2_28_x86_64.whl
mkdir -p validation
.venv/bin/python tools/bev_feature/oe391_environment.py runtime --stage compiler \
  --output validation/native_x86_environment.json
echo 'CPU 编译环境检查通过。下一步运行：bash tools/bev_feature/x86_compile.sh'
