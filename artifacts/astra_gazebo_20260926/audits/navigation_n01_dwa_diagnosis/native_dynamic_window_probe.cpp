#include <base_local_planner/simple_trajectory_generator.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>

// Offline probe: directly execute the installed library's generator; no ROS init,
// master, node, service, costmap, Gazebo or command publisher is constructed.
class Probe : public base_local_planner::SimpleTrajectoryGenerator {
 public:
  const std::vector<Eigen::Vector3f>& samples() const { return sample_params_; }
};

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  std::cout << std::setprecision(17);
  std::string label;
  double vx, vy, wz, acc_y;
  while (std::cin >> label >> vx >> vy >> wz >> acc_y) {
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz) ||
        !std::isfinite(acc_y) || acc_y < 0) return 3;
    base_local_planner::LocalPlannerLimits limits(
        1.20, 0.05, 1.20, 0.0, 0.0, 0.0, 0.7805, 0.0,
        0.60, acc_y, 0.75, 0.60, 0.25, 0.20);
    Probe gen;
    gen.setParameters(3.0, 0.05, 0.025, true, 0.05);
    Eigen::Vector3f pos(0, 0, 0), vel(vx, vy, wz), goal(9, 0, 0);
    gen.initialise(pos, vel, goal, &limits, Eigen::Vector3f(18, 1, 36));
    std::map<float, bool> ys;
    for (const auto& sample : gen.samples()) {
      if (ys.count(sample[1]) && ys[sample[1]]) continue;
      base_local_planner::Trajectory trajectory;
      const bool valid = gen.generateTrajectory(pos, vel, sample, trajectory);
      if (valid && trajectory.yv_ != sample[1]) return 4;
      ys[sample[1]] = valid || ys[sample[1]];
    }
    // Bounds here are descriptive; samples and accepted trajectories above come
    // from the installed native implementation, not this expression.
    const float a = limits.getAccLimits()[1];
    const float lower = std::max(0.0, double(vel[1]) - a * 0.05);
    const float upper = std::min(0.0, double(vel[1]) + a * 0.05);
    std::cout << "{\"case\":\"" << label << "\",\"input_vy\":" << vy
              << ",\"float32_vy\":" << vel[1] << ",\"acc_lim_y\":" << acc_y
              << ",\"dynamic_lower\":" << lower << ",\"dynamic_upper\":" << upper
              << ",\"sample_count\":" << gen.samples().size() << ",\"y_samples\":[";
    bool first = true;
    for (const auto& value : ys) {
      if (!first) std::cout << ',';
      first = false;
      std::cout << "{\"vy\":" << value.first
                << ",\"has_generator_accepted_trajectory\":"
                << (value.second ? "true" : "false") << '}';
    }
    std::cout << "]}\n";
  }
  std::ifstream maps("/proc/self/maps");
  std::ofstream out(argv[1], std::ios::out | std::ios::trunc);
  out << maps.rdbuf();
  return (!out || !maps) ? 5 : 0;
}
