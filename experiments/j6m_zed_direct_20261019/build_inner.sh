#!/usr/bin/env bash
set -euo pipefail
prefix=/tmp/install
cmake -S /tmp/source/hidapi -B /tmp/build/hidapi \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$prefix" -DCMAKE_INSTALL_LIBDIR=lib \
  -DHIDAPI_WITH_HIDRAW=OFF -DHIDAPI_WITH_LIBUSB=ON -DHIDAPI_WITH_TESTS=OFF \
  -DHIDAPI_BUILD_HIDTEST=OFF -DBUILD_SHARED_LIBS=ON
cmake --build /tmp/build/hidapi --target install -- -j2
export PKG_CONFIG_PATH="$prefix/lib/pkgconfig"
cmake -S /tmp/source/zed-open-capture -B /tmp/build/open-capture \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$prefix" \
  -DCMAKE_PREFIX_PATH="$prefix" '-DCMAKE_INSTALL_RPATH=$ORIGIN' \
  -DHIDAPI_INCLUDE_DIR="$prefix/include/hidapi" \
  -DHIDAPI_LIBUSB_LIBRARY="$prefix/lib/libhidapi-libusb.so" \
  -DBUILD_VIDEO=ON -DBUILD_SENSORS=ON -DBUILD_EXAMPLES=OFF
cmake --build /tmp/build/open-capture --target install -- -j2
cmake -S /tmp/tools -B /tmp/build/tools \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$prefix" -DCAPTURE_PREFIX="$prefix"
cmake --build /tmp/build/tools --target install -- -j2
"$prefix/bin/zed_capture_probe" --version
"$prefix/bin/zed_opencv_check"
