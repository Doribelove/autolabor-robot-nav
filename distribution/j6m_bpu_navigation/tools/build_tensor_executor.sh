#!/usr/bin/env bash
# Private native build; existing vendor SDK/sysroot are read-only dependencies.
set -euo pipefail
[[ $# == 3 ]] || { echo 'Usage: build_tensor_executor.sh OUTPUT_DIR REFERENCE_LAB SYSROOT' >&2; exit 2; }
output=$(realpath "$1")
reference=$(realpath "$2")
sysroot=$(realpath "$3")
case "$output" in /map/robot_j6m_navigation_20260907/bpu_production_20261010/*) ;; *) exit 2;; esac
[[ $(uname -m) == aarch64 && "$reference" == /map/robot_j6m_navigation_20260907/bpu_lab_20260905 &&
   "$sysroot" == /map/autolabor_runtime/rootfs ]] || exit 2
source_file=$(realpath "$(dirname "$0")/../native/tensor_executor.cpp")
[[ ! -e "$output/libj6m_tensor.so" && ! -L "$output/libj6m_tensor.so" ]] || {
  echo 'Preserve previous builds; choose a fresh output directory.' >&2; exit 2;
}
printf '%s\n' '6f5cc1a3b963dbaa3b14294c8b2c6bc55a3f2322acf0d64ea0a2b3403fbb3774  hobot/dnn/hb_dnn.h' \
  '511bb3116a297afabfe34a3fb550a01e4cae9a03aef0c354035f4f4d81c5c90b  hobot/hb_ucp.h' \
  '5c204037b38523bdb4cc5d4409bad44ccca907bc7ba1c8693d357f2d21d10771  hobot/hb_ucp_sys.h' |
  (cd "$reference/vendor/ucp_include" && sha256sum --check)
printf '%s\n' '9b82ef81f0e47b3738fa2902c07dadf9483f8aa3108486f75e210f795a431089  libdnn.so' \
  '85c037af59aa2e69acfc585ce0165cf5e797ef313e6d11a80501e673162b6f42  libhbucp.so' \
  '7fa89a85345ee0b8caa0f5c55ce700aa80844b96e0bbe3f9813b11c7d080866d  libhbrt4.so' |
  (cd "$reference/vendor/runtime_4_9_2/lib" && sha256sum --check)
mkdir "$output/tmp" "$output/compiler_lib"
export TMPDIR="$output/tmp"
for name in libisl.so.22 libmpc.so.3 libopcodes-2.34-system.so libbfd-2.34-system.so libctf.so.0; do
  cp -L "$sysroot/usr/lib/aarch64-linux-gnu/$name" "$output/compiler_lib/$name"
done
export LD_LIBRARY_PATH="$output/compiler_lib"
"$sysroot/usr/bin/aarch64-linux-gnu-g++-9" --sysroot="$sysroot" -B"$sysroot/usr/bin/" \
  -std=c++14 -O3 -fPIC -shared -Wall -Wextra -Werror \
  -I"$reference/vendor/ucp_include" "$source_file" \
  -L"$reference/vendor/runtime_4_9_2/lib" -ldnn -lhbucp \
  -Wl,-rpath,"$reference/vendor/runtime_4_9_2/lib" -o "$output/libj6m_tensor.so"
sha256sum "$source_file" "$output/libj6m_tensor.so"
