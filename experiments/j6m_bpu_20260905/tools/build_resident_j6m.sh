#!/usr/bin/env bash
# Native J6M build. Reads the existing Ubuntu compiler/sysroot; writes only lab.
set -euo pipefail
lab=/map/robot_j6m_optimized_20260905/bpu_lab_20260905
sysroot=/map/autolabor_runtime/rootfs
[[ "$(uname -m)" == aarch64 && "$(realpath "$(dirname "$0")/..")" == "$lab" ]] || exit 2
mkdir -p "$lab/bin" "$lab/build/tmp" "$lab/build/compiler_lib"
export TMPDIR="$lab/build/tmp"
# Debian's host userland lacks this GCC-9 dependency. Use a private copy only;
# do not install packages or put the Ubuntu libc ahead of the host loader.
for dependency in libisl.so.22 libmpc.so.3 libopcodes-2.34-system.so libbfd-2.34-system.so libctf.so.0; do
  cp -nL "$sysroot/usr/lib/aarch64-linux-gnu/$dependency" "$lab/build/compiler_lib/$dependency"
done
export LD_LIBRARY_PATH="$lab/build/compiler_lib"
compiler="$sysroot/usr/bin/aarch64-linux-gnu-g++-9"
"$compiler" --sysroot="$sysroot" -B"$sysroot/usr/bin/" \
  -std=c++14 -O3 -fPIC -shared -Wall -Wextra -Werror \
  -I"$lab/vendor/ucp_include" \
  "$lab/tools/fcos_resident.cpp" -L"$lab/vendor/runtime_4_9_2/lib" \
  -ldnn -lhbucp -Wl,-rpath,'$ORIGIN/../vendor/runtime_4_9_2/lib' \
  -o "$lab/bin/libfcos_resident.so.new"
mv "$lab/bin/libfcos_resident.so.new" "$lab/bin/libfcos_resident.so"
sha256sum "$lab/tools/fcos_resident.cpp" "$lab/bin/libfcos_resident.so"
