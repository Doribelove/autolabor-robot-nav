// Real resident adapter validation against the official six-view HBM.
// Reuse its allocation/stride/cache/submit/dequantization implementation.
// Does not interpret traffic classes as trash.
#include "tensor_executor.cpp"
#include <algorithm>
#include <fstream>
#include <iostream>

int main(int argc, char** argv) {
  try {
    require(argc == 3, "Usage: native_reference MODEL.hbm RESULT.json");
    Session session;
    session.load(argv[1], "bev_vtv2_resnet50_nuscenes");
    require(session.inputs.size() == 14 && session.outputs.size() == 6,
            "Official six-view ABI differs; inspect model properties");
    std::vector<std::vector<uint8_t>> inputs;
    for (size_t i = 0; i < session.inputs.size(); ++i) {
      const auto& p = session.inputs[i].properties;
      if (i < 12) require(p.tensorType == HB_DNN_TENSOR_TYPE_U8, "Expected NV12 plane");
      else require(p.tensorType == HB_DNN_TENSOR_TYPE_S16 && p.quantiType == SCALE,
                   "Expected quantized sampling grid");
      inputs.emplace_back(elements(p)*item_size(p.tensorType), i < 12 ? 128 : 0);
    }
    std::vector<double> times;
    std::vector<std::vector<float>> first;
    double delta = 0;
    for (int n = 0; n < 12; ++n) {
      for (size_t i = 0; i < inputs.size(); ++i)
        session.set_input(i, inputs[i].data(), inputs[i].size());
      double ms = 0;
      session.infer(2000, &ms);
      if (n >= 2) times.push_back(ms);
      for (size_t i = 0; i < session.outputs.size(); ++i) {
        std::vector<float> values(elements(session.outputs[i].properties));
        session.output(i, values.data(), values.size());
        if (!n) first.push_back(values);
        else for (size_t j = 0; j < values.size(); ++j)
          delta = std::max(delta, std::abs(double(values[j])-first[i][j]));
      }
    }
    require(delta == 0, "Repeated neutral-input outputs differ");
    double sum = 0;
    for (double ms : times) sum += ms;
    std::ofstream report(argv[2]);
    require(bool(report), "Cannot write report");
    report << "{\"stage\":\"official_six_view_resident_adapter_only\",\"metadata\":"
           << session.metadata << ",\"measured_frames\":" << times.size()
           << ",\"vendor_call_ms_mean\":" << sum/times.size()
           << ",\"repeat_max_difference\":" << delta << "}\n";
    std::cout << "Resident NV12 + S16 input / six scaled S32 outputs passed\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
