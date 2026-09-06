#!/usr/bin/env bash
set -euo pipefail
lab=/map/robot_j6m_optimized_20260905/bpu_perception_20260906
reference=/map/robot_j6m_optimized_20260905/bpu_lab_20260905
sysroot=/map/autolabor_runtime/rootfs
[[ "$(uname -m)" == aarch64 && "$(realpath "$(dirname "$0")/..")" == "$lab" ]] || exit 2
mkdir -p "$lab/bin" "$lab/build/tmp" "$lab/build/compiler_lib"
export TMPDIR="$lab/build/tmp"
for dependency in libisl.so.22 libmpc.so.3 libopcodes-2.34-system.so libbfd-2.34-system.so libctf.so.0; do
  cp -nL "$sysroot/usr/lib/aarch64-linux-gnu/$dependency" "$lab/build/compiler_lib/$dependency"
done
export LD_LIBRARY_PATH="$lab/build/compiler_lib"
"$sysroot/usr/bin/aarch64-linux-gnu-g++-9" --sysroot="$sysroot" -B"$sysroot/usr/bin/" \
  -std=c++14 -O3 -fPIC -shared -Wall -Wextra -Werror \
  -I"$reference/vendor/ucp_include" "$lab/tools/centerpoint_resident.cpp" \
  -L"$reference/vendor/runtime_4_9_2/lib" -ldnn -lhbucp \
  -o "$lab/bin/libcenterpoint_resident.so.new"
mv "$lab/bin/libcenterpoint_resident.so.new" "$lab/bin/libcenterpoint_resident.so"
sha256sum "$lab/bin/libcenterpoint_resident.so"
