#!/usr/bin/env bash
# Invoke the supplied official launcher on the intended development architecture.
set -euo pipefail
oe_project=$(cd -- "$(dirname -- "$0")/../.." && pwd)
oe_sdk="${1:-$oe_project/runtime/oe391_20261101/horizon_j6_open_explorer_v3.9.1-py310_20260821}"
oe_mode="${2:-cpu}"
if [[ "$oe_mode" != cpu && "$oe_mode" != gpu ]]; then
  echo 'Usage: bash tools/bev_feature/oe391.sh [SDK_DIRECTORY] [cpu|gpu]' >&2
  exit 2
fi
if [[ $(uname -m) != x86_64 ]]; then
  echo '当前主机是 ARM64；请将 SDK 和项目放到 x86_64 Linux 后执行此命令。未启动容器。' >&2
  exit 2
fi
test -f "$oe_sdk/run_docker.sh"
cd -- "$oe_sdk"
exec bash ./run_docker.sh "$oe_project" "$oe_mode"
