#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace m2_gazebo {

struct TrajectoryWaypoint {
  double time_s{0.0};
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
};

inline double wrappedAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline bool validTrajectory(const std::vector<TrajectoryWaypoint>& points) {
  if (points.size() < 2U || points.front().time_s < 0.0) return false;
  for (std::size_t index = 0; index < points.size(); ++index) {
    const auto& point = points[index];
    if (!std::isfinite(point.time_s) || !std::isfinite(point.x_m) ||
        !std::isfinite(point.y_m) || !std::isfinite(point.yaw_rad)) {
      return false;
    }
    if (index > 0U && point.time_s <= points[index - 1U].time_s) return false;
  }
  return true;
}

inline TrajectoryWaypoint interpolateTrajectory(
    const std::vector<TrajectoryWaypoint>& points, double time_s, bool loop) {
  if (!validTrajectory(points)) return TrajectoryWaypoint{};
  const double first = points.front().time_s;
  const double last = points.back().time_s;
  double query = time_s;
  if (loop && last > first && query > last) {
    query = first + std::fmod(query - first, last - first);
  }
  if (query <= first) return points.front();
  if (query >= last) return points.back();
  const auto upper = std::upper_bound(
      points.begin(), points.end(), query,
      [](double value, const TrajectoryWaypoint& point) {
        return value < point.time_s;
      });
  const auto& after = *upper;
  const auto& before = *(upper - 1);
  const double ratio = (query - before.time_s) / (after.time_s - before.time_s);
  TrajectoryWaypoint result;
  result.time_s = query;
  result.x_m = before.x_m + ratio * (after.x_m - before.x_m);
  result.y_m = before.y_m + ratio * (after.y_m - before.y_m);
  result.yaw_rad = wrappedAngle(
      before.yaw_rad + ratio * wrappedAngle(after.yaw_rad - before.yaw_rad));
  return result;
}

}  // namespace m2_gazebo
