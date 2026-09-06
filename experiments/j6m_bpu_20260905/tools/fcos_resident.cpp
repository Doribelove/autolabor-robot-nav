// Private J6M FCOS inference adapter. No ROS/device control or disk I/O per frame.
// Built natively on J6M against the installed private DNN/UCP runtime.
#include "hobot/dnn/hb_dnn.h"
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

namespace {
using Clock = std::chrono::steady_clock;
constexpr size_t kY = 896 * 896, kPayload = kY * 3 / 2;
constexpr size_t kOutput = (112 * 112 + 56 * 56 + 28 * 28 + 14 * 14 + 7 * 7) * 85;
constexpr int kSizes[] = {112, 56, 28, 14, 7};
void check(int code, const char* operation) {
  if (code != 0) throw std::runtime_error(std::string(operation) + ": " + std::to_string(code));
}
void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
double elapsed(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}
struct Session {
  hbDNNPackedHandle_t packed = nullptr;
  hbDNNHandle_t model = nullptr;
  hbUCPTaskHandle_t task = nullptr;
  std::array<hbDNNTensor, 2> input{};
  std::array<hbDNNTensor, 15> output{};
  bool failed = false;
  ~Session() {
    if (task) hbUCPReleaseTask(task);
    for (auto& t : output) if (t.sysMem.virAddr) hbUCPFree(&t.sysMem);
    for (auto& t : input) if (t.sysMem.virAddr) hbUCPFree(&t.sysMem);
    if (packed) hbDNNRelease(packed);
  }
  void load(const char* path) {
    const char* version = hbDNNGetVersion();
    require(version && std::string(version).find("3.14.5") != std::string::npos,
            "Unvalidated DNN ABI; expected private 3.14.5 runtime");
    check(hbDNNInitializeFromFiles(&packed, &path, 1), "load model");
    const char** names = nullptr;
    int count = 0;
    check(hbDNNGetModelNameList(&names, &count, packed), "model names");
    require(count == 1 && std::string(names[0]) == "fcos_efficientnetb3_mscoco", "Unexpected model");
    check(hbDNNGetModelHandle(&model, packed, names[0]), "model handle");
    check(hbDNNGetInputCount(&count, model), "input count");
    require(count == 2, "Expected two NV12 planes");
    check(hbDNNGetOutputCount(&count, model), "output count");
    require(count == 15, "Expected fifteen FCOS outputs");
    for (int i = 0; i < 2; ++i) {
      auto& t = input[i];
      auto& p = t.properties;
      check(hbDNNGetInputTensorProperties(&p, model, i), "input properties");
      const int dims[] = {1, i ? 448 : 896, i ? 448 : 896, i ? 2 : 1};
      require(p.validShape.numDimensions == 4 && p.tensorType == HB_DNN_TENSOR_TYPE_U8 &&
              p.quantiType == NONE, "Unexpected NV12 input properties");
      const int64_t strides[] = {static_cast<int64_t>(i ? kY / 2 : kY), 896, i ? 2 : 1, 1};
      for (int d = 0; d < 4; ++d) {
        require(p.validShape.dimensionSize[d] == dims[d], "Input shape mismatch");
        require(p.stride[d] == -1 || p.stride[d] == strides[d], "Input stride mismatch");
        p.stride[d] = strides[d];
      }
      check(hbUCPMallocCached(&t.sysMem, i ? kY / 2 : kY, 0), "allocate input");
    }
    for (int i = 0; i < 15; ++i) {
      auto& t = output[i];
      auto& p = t.properties;
      check(hbDNNGetOutputTensorProperties(&p, model, i), "output properties");
      const int channels = i < 5 ? 80 : (i < 10 ? 4 : 1), size = kSizes[i % 5];
      std::fprintf(stderr, "FCOS tensor %d: ndim=%d shape=%d,%d,%d,%d type=%d quant=%d axis=%d bytes=%lld scales=%d zeros=%d stride=%lld,%lld,%lld,%lld\n",
          i, p.validShape.numDimensions, p.validShape.dimensionSize[0], p.validShape.dimensionSize[1],
          p.validShape.dimensionSize[2], p.validShape.dimensionSize[3], p.tensorType, p.quantiType,
          p.quantizeAxis, static_cast<long long>(p.alignedByteSize), p.scale.scaleLen, p.scale.zeroPointLen,
          static_cast<long long>(p.stride[0]), static_cast<long long>(p.stride[1]),
          static_cast<long long>(p.stride[2]), static_cast<long long>(p.stride[3]));
      require(p.validShape.numDimensions == 4 && p.validShape.dimensionSize[0] == 1 &&
              p.validShape.dimensionSize[1] == channels && p.validShape.dimensionSize[2] == size &&
              p.validShape.dimensionSize[3] == size && p.tensorType == HB_DNN_TENSOR_TYPE_S32 &&
              p.quantiType == SCALE, "FCOS output ABI/shape/type mismatch");
      require(p.quantizeAxis == 1 || (p.quantizeAxis == 0 && p.scale.scaleLen == 1 &&
              p.scale.zeroPointLen <= 1), "Unsupported quantization axis");
      require(p.alignedByteSize > 0 && p.alignedByteSize <= 16 * 1024 * 1024 &&
              p.stride[3] == 4 && p.stride[2] >= size * 4 &&
              p.stride[1] >= size * p.stride[2] && p.stride[0] >= channels * p.stride[1] &&
              p.alignedByteSize >= p.stride[0] && p.stride[1] % 4 == 0 && p.stride[2] % 4 == 0,
              "Unsafe padded output strides");
      require(p.scale.scaleData && (p.scale.scaleLen == 1 || p.scale.scaleLen == channels),
              "Unsupported quantization scale");
      require(p.scale.zeroPointLen == 0 || (p.scale.zeroPointData &&
              (p.scale.zeroPointLen == 1 || p.scale.zeroPointLen == channels)), "Unsupported zero points");
      for (int c = 0; c < p.scale.scaleLen; ++c)
        require(std::isfinite(p.scale.scaleData[c]) && p.scale.scaleData[c] > 0, "Invalid quantization scale");
      check(hbUCPMallocCached(&t.sysMem, p.alignedByteSize, 0), "allocate output");
    }
  }
  void infer(const unsigned char* data, size_t bytes, float* result, size_t count, double* timing) {
    require(!failed, "Session failed; recreate explicitly");
    require(data && bytes == kPayload && result && count == kOutput && timing, "Invalid inference buffers");
    auto start = Clock::now();
    for (int i = 0; i < 2; ++i) {
      std::memcpy(input[i].sysMem.virAddr, data + (i ? kY : 0), i ? kY / 2 : kY);
      check(hbUCPMemFlush(&input[i].sysMem, HB_SYS_MEM_CACHE_CLEAN), "clean input cache");
    }
    timing[0] = elapsed(start);
    start = Clock::now();
    check(hbDNNInferV2(&task, output.data(), input.data(), model), "create inference task");
    hbUCPSchedParam schedule{};
    HB_UCP_INITIALIZE_SCHED_PARAM(&schedule);
    schedule.backend = HB_UCP_BPU_CORE_0;
    check(hbUCPSubmitTask(task, &schedule), "submit inference");
    check(hbUCPWaitTaskDone(task, 8000), "wait inference (8s deadline)");
    timing[1] = elapsed(start);
    check(hbUCPReleaseTask(task), "release inference task");
    task = nullptr;
    start = Clock::now();
    for (auto& t : output) {
      check(hbUCPMemFlush(&t.sysMem, HB_SYS_MEM_CACHE_INVALIDATE), "invalidate output cache");
      const auto& p = t.properties;
      const int channels = p.validShape.dimensionSize[1], size = p.validShape.dimensionSize[2];
      for (int c = 0; c < channels; ++c) {
        const float scale = p.scale.scaleData[p.scale.scaleLen == 1 ? 0 : c];
        const int32_t zero = p.scale.zeroPointLen ? p.scale.zeroPointData[p.scale.zeroPointLen == 1 ? 0 : c] : 0;
        for (int y = 0; y < size; ++y) {
          const auto* row = reinterpret_cast<const int32_t*>(
              static_cast<const char*>(t.sysMem.virAddr) + c * p.stride[1] + y * p.stride[2]);
          for (int x = 0; x < size; ++x)
            *result++ = static_cast<float>(static_cast<int64_t>(row[x]) - zero) * scale;
        }
      }
    }
    timing[2] = elapsed(start);
  }
};
void error_text(char* buffer, size_t size, const std::exception& error) {
  if (buffer && size) std::snprintf(buffer, size, "%s", error.what());
}
}  // namespace

extern "C" {
void* fcos_create(const char* path, char* error, size_t size) {
  try {
    auto session = std::make_unique<Session>();
    session->load(path);
    return session.release();
  } catch (const std::exception& e) { error_text(error, size, e); return nullptr; }
}
int fcos_infer(void* context, const unsigned char* data, size_t bytes, float* result,
               size_t count, double* timing, char* error, size_t size) {
  if (!context) return -1;
  auto* session = static_cast<Session*>(context);
  try { session->infer(data, bytes, result, count, timing); return 0; }
  catch (const std::exception& e) { session->failed = true; error_text(error, size, e); return -1; }
}
void fcos_destroy(void* context) { delete static_cast<Session*>(context); }
}
