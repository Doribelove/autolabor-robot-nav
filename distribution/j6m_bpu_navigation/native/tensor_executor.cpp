// J6M private DNN / UCP adapter. No ROS, sensors or vehicle commands.
// Vendor headers and libraries remain outside the distributable source tree.
// New SDK builds supply the matching runtime version alongside their headers.
// Existing build commands continue targeting the previously validated runtime.
#ifndef J6M_DNN_EXPECTED_VERSION
#define J6M_DNN_EXPECTED_VERSION "3.14.5_(4.9.2 HBRT)"
#endif
#include "hobot/dnn/hb_dnn.h"
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr int64_t kMaxBytes = 256 * 1024 * 1024;
using Clock = std::chrono::steady_clock;
void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}
void check(int code, const char* operation) {
  if (code) throw std::runtime_error(std::string(operation) + ": " + std::to_string(code));
}
size_t item_size(int type) {
  switch (type) {
    case HB_DNN_TENSOR_TYPE_U8: case HB_DNN_TENSOR_TYPE_S8: return 1;
    case HB_DNN_TENSOR_TYPE_S16: return 2;
    case HB_DNN_TENSOR_TYPE_F32: case HB_DNN_TENSOR_TYPE_S32: return 4;
    default: throw std::runtime_error("Unsupported tensor dtype; inspect compiler interface");
  }
}
size_t elements(const hbDNNTensorProperties& p) {
  require(p.validShape.numDimensions > 0 &&
          p.validShape.numDimensions <= HB_DNN_TENSOR_MAX_DIMENSIONS, "Invalid rank");
  size_t count = 1;
  for (int d = 0; d < p.validShape.numDimensions; ++d) {
    const int n = p.validShape.dimensionSize[d];
    require(n > 0 && n <= 1048576 && count <= kMaxBytes / static_cast<size_t>(n),
            "Invalid or oversized shape");
    count *= n;
  }
  require(count <= kMaxBytes / item_size(p.tensorType), "Oversized tensor");
  return count;
}
void validate(hbDNNTensorProperties& p, bool input) {
  elements(p);
  const int rank = p.validShape.numDimensions;
  const size_t bytes = item_size(p.tensorType);
  int64_t inner = bytes;
  int64_t last_offset = 0;
  for (int d = rank - 1; d >= 0; --d) {
    // -1 denotes a caller-specified DDR input stride in this private API.
    if (input && p.stride[d] == -1) p.stride[d] = inner;
    require(p.stride[d] >= inner && p.stride[d] <= kMaxBytes &&
            p.stride[d] % bytes == 0, "Overlapping or unsupported tensor strides");
    const int64_t n = p.validShape.dimensionSize[d];
    require(p.stride[d] <= kMaxBytes / n, "Stride overflow");
    inner = p.stride[d] * n;
    last_offset += (n - 1) * p.stride[d];
    require(last_offset <= kMaxBytes - static_cast<int64_t>(bytes), "Tensor extent overflow");
  }
  if (input && p.alignedByteSize <= 0) p.alignedByteSize = last_offset + bytes;
  require(p.alignedByteSize >= last_offset + static_cast<int64_t>(bytes) &&
          p.alignedByteSize <= kMaxBytes, "Invalid aligned allocation size");
  if (input && p.quantiType == NONE) {
    require(p.tensorType == HB_DNN_TENSOR_TYPE_F32 || p.tensorType == HB_DNN_TENSOR_TYPE_U8,
            "Expected unquantized F32 featuremap or U8 image input");
  } else if (p.quantiType == SCALE) {
    // BEV sampling coordinates are prequantized S16, in the model's own scale.
    // The caller passes exact typed bytes; never silently reinterpret floats.
    if (input) require(p.tensorType == HB_DNN_TENSOR_TYPE_S16,
                       "Expected S16 quantized BEV sampling input");
    require(p.tensorType != HB_DNN_TENSOR_TYPE_F32 && p.quantizeAxis >= 0 &&
            p.quantizeAxis < rank, "Invalid quantization type or axis");
    const int n = p.validShape.dimensionSize[p.quantizeAxis];
    require(p.scale.scaleData && (p.scale.scaleLen == 1 || p.scale.scaleLen == n),
            "Invalid quantization scales");
    require(p.scale.zeroPointLen == 0 || (p.scale.zeroPointData &&
            (p.scale.zeroPointLen == 1 || p.scale.zeroPointLen == n)), "Invalid zero points");
    for (int i = 0; i < p.scale.scaleLen; ++i)
      require(std::isfinite(p.scale.scaleData[i]) && p.scale.scaleData[i] > 0,
              "Nonpositive or nonfinite scale");
  } else {
    require(p.quantiType == NONE && p.tensorType == HB_DNN_TENSOR_TYPE_F32,
            "Expected floating or scale-quantized output");
  }
}
size_t offset_at(const hbDNNTensorProperties& p, size_t index, int* quant_index = nullptr) {
  size_t offset = 0;
  for (int d = p.validShape.numDimensions - 1; d >= 0; --d) {
    const size_t coordinate = index % p.validShape.dimensionSize[d];
    index /= p.validShape.dimensionSize[d];
    offset += coordinate * p.stride[d];
    if (quant_index && d == p.quantizeAxis) *quant_index = coordinate;
  }
  return offset;
}
std::string quote(const std::string& value) {
  std::ostringstream out; out << '"';
  for (unsigned char c : value) {
    if (c == '"' || c == '\\') out << '\\' << c;
    else if (c < 32) { char hex[7]; std::snprintf(hex, sizeof(hex), "\\u%04x", c); out << hex; }
    else out << c;
  }
  return out.str() + '"';
}
struct Session {
  hbDNNPackedHandle_t packed = nullptr;
  hbDNNHandle_t model = nullptr;
  hbUCPTaskHandle_t task = nullptr;
  std::vector<hbDNNTensor> inputs, outputs;
  std::vector<bool> input_ready;
  std::string metadata;
  bool failed = false, output_ready = false, pending = false;
  ~Session() {
    // Never free buffers underneath an unfinished device task. A failed worker
    // exits; the driver then owns process cleanup rather than buffer reuse.
    if (task && pending && hbUCPWaitTaskDone(task, 8000) != 0) return;
    if (task) hbUCPReleaseTask(task);
    for (auto& t : outputs) if (t.sysMem.virAddr) hbUCPFree(&t.sysMem);
    for (auto& t : inputs) if (t.sysMem.virAddr) hbUCPFree(&t.sysMem);
    if (packed) hbDNNRelease(packed);
  }
  void load(const char* path, const char* expected_name) {
    const char* version = hbDNNGetVersion();
    require(version && std::string(version) == J6M_DNN_EXPECTED_VERSION,
            "DNN runtime differs from build target: " J6M_DNN_EXPECTED_VERSION);
    check(hbDNNInitializeFromFiles(&packed, &path, 1), "load HBM");
    const char** names = nullptr; int count = 0;
    check(hbDNNGetModelNameList(&names, &count, packed), "model names");
    require(count == 1 && names && names[0], "Expected exactly one model");
    require(expected_name && std::string(expected_name) == names[0], "Model name mismatch");
    check(hbDNNGetModelHandle(&model, packed, names[0]), "model handle");
    std::ostringstream info;
    info << "{\"dnn_version\":" << quote(version) << ",\"model_name\":" << quote(names[0]);
    int64_t total_bytes = 0;
    for (int side = 0; side < 2; ++side) {
      auto& tensors = side ? outputs : inputs;
      check(side ? hbDNNGetOutputCount(&count, model) : hbDNNGetInputCount(&count, model), "tensor count");
      require(count > 0 && count <= 32, "Invalid tensor count");
      tensors.resize(count);
      info << (side ? ",\"outputs\":[" : ",\"inputs\":[");
      for (int i = 0; i < count; ++i) {
        auto& t = tensors[i]; auto& p = t.properties; const char* name = nullptr;
        check(side ? hbDNNGetOutputTensorProperties(&p, model, i) :
                     hbDNNGetInputTensorProperties(&p, model, i), "tensor properties");
        check(side ? hbDNNGetOutputName(&name, model, i) : hbDNNGetInputName(&name, model, i), "tensor name");
        require(name, "Missing tensor name"); validate(p, !side);
        total_bytes += p.alignedByteSize;
        require(total_bytes <= 2 * kMaxBytes, "Model buffers exceed budget");
        check(hbUCPMallocCached(&t.sysMem, p.alignedByteSize, 0), "allocate cached tensor");
        if (i) info << ',';
        info << "{\"name\":" << quote(name) << ",\"shape\":[";
        for (int d = 0; d < p.validShape.numDimensions; ++d) {
          if (d) info << ',';
          info << p.validShape.dimensionSize[d];
        }
        info << "],\"strides\":[";
        for (int d = 0; d < p.validShape.numDimensions; ++d) {
          if (d) info << ',';
          info << p.stride[d];
        }
        info << "],\"type\":" << p.tensorType << ",\"quant\":" << p.quantiType
             << ",\"axis\":" << p.quantizeAxis << ",\"bytes\":" << p.alignedByteSize
             << ",\"scale_len\":" << p.scale.scaleLen << ",\"zero_len\":" << p.scale.zeroPointLen;
        info << ",\"scales\":[";
        if (p.quantiType == SCALE) for (int j = 0; j < p.scale.scaleLen; ++j) {
          if (j) info << ',';
          info << p.scale.scaleData[j];
        }
        info << "],\"zero_points\":[";
        if (p.quantiType == SCALE) for (int j = 0; j < p.scale.zeroPointLen; ++j) {
          if (j) info << ',';
          info << p.scale.zeroPointData[j];
        }
        info << "]}";
      }
      info << ']';
    }
    info << '}'; metadata = info.str(); input_ready.resize(inputs.size(), false);
  }
  void set_input(int index, const void* data, size_t bytes) {
    require(!failed && index >= 0 && static_cast<size_t>(index) < inputs.size(), "Invalid input/session");
    auto& t = inputs[index]; const auto& p = t.properties;
    const size_t item = item_size(p.tensorType), count = elements(p);
    require(data && bytes == count * item, "Dense input byte count mismatch");
    if (p.tensorType == HB_DNN_TENSOR_TYPE_F32) {
      for (size_t i = 0; i < count; ++i) {
        float value; std::memcpy(&value, static_cast<const char*>(data) + i * item, item);
        require(std::isfinite(value), "Nonfinite input tensor");
      }
    }
    std::memset(t.sysMem.virAddr, 0, p.alignedByteSize);
    bool dense = true;
    size_t expected_stride = item;
    for (int d = p.validShape.numDimensions - 1; d >= 0; --d) {
      if (p.validShape.dimensionSize[d] > 1 &&
          p.stride[d] != static_cast<int64_t>(expected_stride)) dense = false;
      expected_stride *= p.validShape.dimensionSize[d];
    }
    const size_t width = p.validShape.dimensionSize[p.validShape.numDimensions - 1];
    if (dense) std::memcpy(t.sysMem.virAddr, data, bytes);
    else for (size_t i = 0; i < count; i += width) {
      char* target = static_cast<char*>(t.sysMem.virAddr) + offset_at(p, i);
      const char* source = static_cast<const char*>(data) + i * item;
      if (p.stride[p.validShape.numDimensions - 1] == static_cast<int64_t>(item))
        std::memcpy(target, source, width * item);
      else for (size_t x = 0; x < width; ++x)
        std::memcpy(target + x * p.stride[p.validShape.numDimensions - 1], source + x * item, item);
    }
    check(hbUCPMemFlush(&t.sysMem, HB_SYS_MEM_CACHE_CLEAN), "clean input cache");
    input_ready[index] = true; output_ready = false;
  }
  void infer(int timeout, double* elapsed) {
    require(!failed && !task && elapsed && timeout > 0 && timeout <= 8000, "Invalid inference/session");
    for (bool ready : input_ready) require(ready, "Set every input before inference");
    output_ready = false;
    for (size_t i = 0; i < input_ready.size(); ++i) input_ready[i] = false;
    auto started = Clock::now();
    check(hbDNNInferV2(&task, outputs.data(), inputs.data(), model), "create inference task");
    hbUCPSchedParam schedule{}; HB_UCP_INITIALIZE_SCHED_PARAM(&schedule);
    schedule.backend = HB_UCP_BPU_CORE_0;
    // A submission error can leave task state uncertain; protect buffers until
    // a successful device wait, including the error path.
    pending = true;
    check(hbUCPSubmitTask(task, &schedule), "submit to BPU core 0");
    check(hbUCPWaitTaskDone(task, timeout), "bounded device wait"); pending = false;
    check(hbUCPReleaseTask(task), "release completed task"); task = nullptr;
    *elapsed = std::chrono::duration<double, std::milli>(Clock::now() - started).count();
    for (auto& t : outputs) check(hbUCPMemFlush(&t.sysMem, HB_SYS_MEM_CACHE_INVALIDATE), "invalidate output cache");
    output_ready = true;
  }
  void output(int index, float* values, size_t count) {
    require(!failed && output_ready && index >= 0 && static_cast<size_t>(index) < outputs.size(), "Invalid output/session");
    const auto& t = outputs[index]; const auto& p = t.properties;
    require(values && count == elements(p), "Output element count mismatch");
    const int rank = p.validShape.numDimensions;
    const size_t width = p.validShape.dimensionSize[rank - 1];
    for (size_t row = 0; row < count; row += width) {
      int axis_index = 0;
      const char* start = static_cast<const char*>(t.sysMem.virAddr) + offset_at(p, row, &axis_index);
      for (size_t x = 0; x < width; ++x) {
        const char* source = start + x * p.stride[rank - 1];
        if (p.quantizeAxis == rank - 1) axis_index = x;
        double value = 0;
        switch (p.tensorType) {
          case HB_DNN_TENSOR_TYPE_F32: { float v; std::memcpy(&v, source, 4); value = v; break; }
          case HB_DNN_TENSOR_TYPE_S32: { int32_t v; std::memcpy(&v, source, 4); value = v; break; }
          case HB_DNN_TENSOR_TYPE_S16: { int16_t v; std::memcpy(&v, source, 2); value = v; break; }
          case HB_DNN_TENSOR_TYPE_S8: { int8_t v; std::memcpy(&v, source, 1); value = v; break; }
          case HB_DNN_TENSOR_TYPE_U8: { uint8_t v; std::memcpy(&v, source, 1); value = v; break; }
          default: throw std::runtime_error("Unsupported output type");
        }
        if (p.quantiType == SCALE) {
          const int zero = p.scale.zeroPointLen ? p.scale.zeroPointData[p.scale.zeroPointLen == 1 ? 0 : axis_index] : 0;
          value = static_cast<float>(value - zero) *
                  p.scale.scaleData[p.scale.scaleLen == 1 ? 0 : axis_index];
        }
        require(std::isfinite(value) && std::isfinite(static_cast<float>(value)), "Nonfinite output tensor");
        values[row + x] = static_cast<float>(value);
      }
    }
  }
};
void error_text(char* out, size_t size, const std::exception& error) {
  if (out && size) std::snprintf(out, size, "%s", error.what());
}
template<class Work> int guard(void* context, char* error, size_t size, Work work) {
  if (!context) return -1;
  auto* session = static_cast<Session*>(context);
  try { work(*session); return 0; }
  catch (const std::exception& e) { session->failed = true; error_text(error, size, e); return -1; }
}
}  // namespace

extern "C" {
void* j6m_tensor_create(const char* path, const char* model, char* error, size_t size) {
  try {
    require(path && model, "Missing model path/name");
    auto session = std::make_unique<Session>(); session->load(path, model); return session.release();
  } catch (const std::exception& e) { error_text(error, size, e); return nullptr; }
}
int j6m_tensor_metadata(void* context, char* out, size_t bytes, char* error, size_t size) {
  return guard(context, error, size, [&](Session& s) {
    require(out && bytes > s.metadata.size(), "Metadata buffer too small");
    std::memcpy(out, s.metadata.c_str(), s.metadata.size() + 1);
  });
}
int j6m_tensor_input(void* context, int index, const void* data, size_t bytes, char* error, size_t size) {
  return guard(context, error, size, [&](Session& s) { s.set_input(index, data, bytes); });
}
int j6m_tensor_infer(void* context, int timeout, double* ms, char* error, size_t size) {
  return guard(context, error, size, [&](Session& s) { s.infer(timeout, ms); });
}
int j6m_tensor_output(void* context, int index, float* values, size_t count, char* error, size_t size) {
  return guard(context, error, size, [&](Session& s) { s.output(index, values, count); });
}
void j6m_tensor_destroy(void* context) { delete static_cast<Session*>(context); }
}
