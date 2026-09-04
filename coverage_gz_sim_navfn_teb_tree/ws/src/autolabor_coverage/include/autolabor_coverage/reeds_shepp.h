#ifndef AUTOLABOR_COVERAGE_REEDS_SHEPP_H
#define AUTOLABOR_COVERAGE_REEDS_SHEPP_H

#include <array>
#include <limits>
#include <vector>

namespace autolabor_coverage
{

enum class ReedsSheppSegmentType
{
  NOP = 0,
  LEFT = 1,
  STRAIGHT = 2,
  RIGHT = 3
};

struct ReedsSheppPath
{
  std::array<ReedsSheppSegmentType, 5> types{{
      ReedsSheppSegmentType::NOP, ReedsSheppSegmentType::NOP,
      ReedsSheppSegmentType::NOP, ReedsSheppSegmentType::NOP,
      ReedsSheppSegmentType::NOP}};
  std::array<double, 5> lengths{{0.0, 0.0, 0.0, 0.0, 0.0}};
  double normalized_length = std::numeric_limits<double>::infinity();

  bool valid() const;
};

// Returns the shortest obstacle-free Reeds-Shepp curve. Segment lengths are
// normalized by turning_radius; a negative length denotes reverse travel.
ReedsSheppPath shortestReedsSheppPath(
    double start_x, double start_y, double start_yaw,
    double goal_x, double goal_y, double goal_yaw,
    double turning_radius);

// Returns every valid member of the 48 Reeds-Shepp families. The paths are
// geometric candidates only; callers remain responsible for collision checks
// and for ranking them with the active coverage-transit time cost.
std::vector<ReedsSheppPath> allReedsSheppPaths(
    double start_x, double start_y, double start_yaw,
    double goal_x, double goal_y, double goal_yaw,
    double turning_radius);

}  // namespace autolabor_coverage

#endif
