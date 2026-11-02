// Local ZED left RGB only. Depth is disabled as requested by the user.
#include <sl/Camera.hpp>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

namespace {
thread_local std::string error;
struct Preview { sl::Camera camera; sl::Mat image; sl::RuntimeParameters runtime; };
}
extern "C" {
const char* zp_error() { return error.c_str(); }
void* zp_open(unsigned serial, const char* settings) {
  try {
    auto preview = std::unique_ptr<Preview>(new Preview);
    sl::InitParameters init;
    init.input.setFromSerialNumber(serial);
    init.camera_resolution = sl::RESOLUTION::HD720;
    init.camera_fps = 15;
    init.depth_mode = sl::DEPTH_MODE::NONE;
    init.coordinate_units = sl::UNIT::METER;
    init.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE;
    init.sdk_verbose = 0;
    init.optional_settings_path = settings;
    const auto code = preview->camera.open(init);
    if (code != sl::ERROR_CODE::SUCCESS) {
      error = sl::toString(code).c_str(); return nullptr;
    }
    const auto info = preview->camera.getCameraInformation();
    if (info.serial_number != serial || info.camera_configuration.resolution.width != 1280 ||
        info.camera_configuration.resolution.height != 720) {
      error = "Unexpected ZED serial or resolution"; return nullptr;
    }
    preview->runtime.enable_depth = false;
    return preview.release();
  } catch (const std::exception& ex) { error = ex.what(); return nullptr; }
}
int zp_read(void* handle, unsigned char* output, size_t capacity, uint64_t* stamp) {
  if (!handle || !output || !stamp || capacity < 1280*720*4) return -1;
  auto& preview = *static_cast<Preview*>(handle);
  const auto code = preview.camera.grab(preview.runtime);
  if (code != sl::ERROR_CODE::SUCCESS) { error = sl::toString(code).c_str(); return -1; }
  const auto image_code = preview.camera.retrieveImage(preview.image, sl::VIEW::LEFT, sl::MEM::CPU);
  if (image_code != sl::ERROR_CODE::SUCCESS) { error = sl::toString(image_code).c_str(); return -1; }
  for (unsigned row = 0; row < 720; ++row)
    std::memcpy(output+row*1280*4,
        preview.image.getPtr<sl::uchar1>(sl::MEM::CPU)+row*preview.image.getStepBytes(sl::MEM::CPU), 1280*4);
  *stamp = preview.camera.getTimestamp(sl::TIME_REFERENCE::IMAGE).getNanoseconds();
  return 0;
}
int zp_intrinsics(void* handle, double* values) {
  if (!handle || !values) return -1;
  const auto k = static_cast<Preview*>(handle)->camera.getCameraInformation().camera_configuration.calibration_parameters.left_cam;
  values[0] = k.fx; values[1] = k.fy; values[2] = k.cx; values[3] = k.cy;
  return 0;
}
int zp_depth_enabled(void* handle) {
  if (!handle) return -1;
  return static_cast<Preview*>(handle)->camera.getInitParameters().depth_mode != sl::DEPTH_MODE::NONE;
}
void zp_close(void* handle) {
  if (handle) { auto* preview = static_cast<Preview*>(handle); preview->camera.close(); delete preview; }
}
}
