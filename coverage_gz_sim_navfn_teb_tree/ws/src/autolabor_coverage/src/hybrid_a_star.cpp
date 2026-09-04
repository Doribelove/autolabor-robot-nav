#include "autolabor_coverage/hybrid_a_star.h"
#include "autolabor_coverage/reeds_shepp.h"

#include <base_local_planner/costmap_model.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/footprint.h>
#include <ros/time.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_map>
#include <utility>

namespace autolabor_coverage
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kEpsilon = 1.0e-9;

double normalizeYaw(double yaw)
{
  return std::atan2(std::sin(yaw), std::cos(yaw));
}

double positiveYaw(double yaw)
{
  yaw = std::fmod(yaw, kTwoPi);
  return yaw < 0.0 ? yaw + kTwoPi : yaw;
}

double yawError(double first, double second)
{
  return std::abs(normalizeYaw(second - first));
}

bool quaternionYaw(const geometry_msgs::Quaternion& quaternion, double& yaw)
{
  const double norm = quaternion.x * quaternion.x +
                      quaternion.y * quaternion.y +
                      quaternion.z * quaternion.z +
                      quaternion.w * quaternion.w;
  if (!std::isfinite(norm) || norm < kEpsilon)
    return false;
  const double inverse = 1.0 / std::sqrt(norm);
  const double x = quaternion.x * inverse;
  const double y = quaternion.y * inverse;
  const double z = quaternion.z * inverse;
  const double w = quaternion.w * inverse;
  yaw = std::atan2(2.0 * (w * z + x * y),
                   1.0 - 2.0 * (y * y + z * z));
  return std::isfinite(yaw);
}

geometry_msgs::Quaternion yawQuaternion(double yaw)
{
  geometry_msgs::Quaternion quaternion;
  quaternion.z = std::sin(0.5 * yaw);
  quaternion.w = std::cos(0.5 * yaw);
  return quaternion;
}

struct SearchNode
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double g = 0.0;
  double f = 0.0;
  int parent = -1;
  int gear = 0;
  int steering_index = 0;
  double curvature = 0.0;
  double reverse_distance = 0.0;
  unsigned int direction_changes = 0;
  std::uint64_t key = 0;
};

struct QueueEntry
{
  double score = 0.0;
  std::size_t node_index = 0;
};

struct QueueGreater
{
  bool operator()(const QueueEntry& first, const QueueEntry& second) const
  {
    if (std::abs(first.score - second.score) > kEpsilon)
      return first.score > second.score;
    return first.node_index > second.node_index;
  }
};

bool finitePositive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

bool validConfiguration(const HybridAStarConfig& config,
                        const HybridAStarProfile& profile,
                        std::string& reason)
{
  if (!finitePositive(config.minimum_turning_radius) ||
      !finitePositive(config.motion_step) ||
      !finitePositive(config.collision_check_step) ||
      !finitePositive(config.state_resolution) ||
      config.heading_bins < 16 || config.heading_bins > 360 ||
      config.steering_samples < 3 || config.steering_samples > 11 ||
      config.steering_samples % 2 == 0 || config.max_expansions < 100 ||
      config.analytic_expansion_interval < 1 ||
      !finitePositive(config.planning_timeout) ||
      !finitePositive(config.analytic_improvement_timeout) ||
      !finitePositive(config.analytic_gearchange_improvement_timeout) ||
      !finitePositive(config.analytic_connector_improvement_timeout) ||
      !std::isfinite(config.heuristic_weight) ||
      config.heuristic_weight < 1.0 || config.heuristic_weight > 3.0 ||
      !std::isfinite(config.steering_penalty) ||
      config.steering_penalty < 0.0 ||
      !std::isfinite(config.steering_change_penalty) ||
      config.steering_change_penalty < 0.0 ||
      !std::isfinite(config.obstacle_cost_scale) ||
      config.obstacle_cost_scale < 0.0)
  {
    reason = "Hybrid A* static configuration is invalid";
    return false;
  }
  if (!finitePositive(profile.max_forward_speed) ||
      !finitePositive(profile.max_angular_speed) ||
      !finitePositive(profile.linear_acceleration) ||
      !finitePositive(profile.angular_acceleration) ||
      !finitePositive(profile.goal_position_tolerance) ||
      !finitePositive(profile.goal_yaw_tolerance) ||
      !std::isfinite(profile.direction_change_penalty) ||
      profile.direction_change_penalty < 0.0 ||
      (profile.allow_reverse && !finitePositive(profile.max_reverse_speed)))
  {
    reason = "Hybrid A* transit profile is invalid";
    return false;
  }
  return true;
}

void integrate(double x, double y, double yaw, double signed_distance,
               double curvature, double& output_x, double& output_y,
               double& output_yaw)
{
  if (std::abs(curvature) < kEpsilon)
  {
    output_x = x + signed_distance * std::cos(yaw);
    output_y = y + signed_distance * std::sin(yaw);
    output_yaw = yaw;
    return;
  }
  output_yaw = normalizeYaw(yaw + signed_distance * curvature);
  output_x = x + (std::sin(output_yaw) - std::sin(yaw)) / curvature;
  output_y = y + (-std::cos(output_yaw) + std::cos(yaw)) / curvature;
}

class ObstacleHeuristic
{
public:
  ObstacleHeuristic(const costmap_2d::Costmap2D* costmap,
                    double goal_x, double goal_y, bool enabled)
    : costmap_(costmap)
  {
    if (!enabled || !costmap_)
      return;
    width_ = costmap_->getSizeInCellsX();
    height_ = costmap_->getSizeInCellsY();
    unsigned int goal_x_index = 0;
    unsigned int goal_y_index = 0;
    if (width_ == 0 || height_ == 0 ||
        !costmap_->worldToMap(goal_x, goal_y,
                             goal_x_index, goal_y_index))
      return;

    distances_.assign(static_cast<std::size_t>(width_) * height_,
                      std::numeric_limits<double>::infinity());
    using Entry = std::pair<double, std::size_t>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
    const std::size_t goal_index = index(goal_x_index, goal_y_index);
    distances_[goal_index] = 0.0;
    queue.push({0.0, goal_index});
    const double resolution = costmap_->getResolution();
    const std::array<int, 8> delta_x{{-1, 0, 1, -1, 1, -1, 0, 1}};
    const std::array<int, 8> delta_y{{-1, -1, -1, 0, 0, 1, 1, 1}};
    while (!queue.empty())
    {
      const Entry current = queue.top();
      queue.pop();
      if (current.first > distances_[current.second] + kEpsilon)
        continue;
      const unsigned int x = static_cast<unsigned int>(current.second % width_);
      const unsigned int y = static_cast<unsigned int>(current.second / width_);
      for (std::size_t neighbor = 0; neighbor < delta_x.size(); ++neighbor)
      {
        const int next_x = static_cast<int>(x) + delta_x[neighbor];
        const int next_y = static_cast<int>(y) + delta_y[neighbor];
        if (next_x < 0 || next_y < 0 ||
            next_x >= static_cast<int>(width_) ||
            next_y >= static_cast<int>(height_))
          continue;
        const unsigned char cost = costmap_->getCost(
            static_cast<unsigned int>(next_x),
            static_cast<unsigned int>(next_y));
        if (cost == costmap_2d::LETHAL_OBSTACLE ||
            cost == costmap_2d::NO_INFORMATION)
          continue;
        // Deliberately allow diagonal corner cutting and ignore footprint and
        // inflation penalties. This distance may underestimate the true
        // Ackermann detour, so it remains a safe lower bound.
        const double step = resolution *
            ((delta_x[neighbor] != 0 && delta_y[neighbor] != 0)
                 ? std::sqrt(2.0)
                 : 1.0);
        const std::size_t next = index(
            static_cast<unsigned int>(next_x),
            static_cast<unsigned int>(next_y));
        const double candidate = current.first + step;
        if (candidate + kEpsilon < distances_[next])
        {
          distances_[next] = candidate;
          queue.push({candidate, next});
        }
      }
    }
    ready_ = true;
  }

  bool ready() const
  {
    return ready_;
  }

  double distance(double x, double y) const
  {
    if (!ready_)
      return std::numeric_limits<double>::infinity();
    unsigned int map_x = 0;
    unsigned int map_y = 0;
    if (!costmap_->worldToMap(x, y, map_x, map_y))
      return std::numeric_limits<double>::infinity();
    return distances_[index(map_x, map_y)];
  }

private:
  std::size_t index(unsigned int x, unsigned int y) const
  {
    return static_cast<std::size_t>(y) * width_ + x;
  }

  const costmap_2d::Costmap2D* costmap_ = nullptr;
  unsigned int width_ = 0;
  unsigned int height_ = 0;
  bool ready_ = false;
  std::vector<double> distances_;
};

double heuristic(double x, double y, double yaw,
                 double goal_x, double goal_y, double goal_yaw,
                 const HybridAStarConfig& config,
                 const HybridAStarProfile& profile,
                 const ObstacleHeuristic* obstacle_heuristic,
                 bool use_nonholonomic = true)
{
  const double exact_distance = std::hypot(goal_x - x, goal_y - y);
  const double exact_orientation = yawError(yaw, goal_yaw);
  // A goal-region request is complete at any collision-free pose inside the
  // caller's position/yaw basin. Heuristics aimed at the mathematical centre
  // made the queue prefer a needlessly long exact-pose loop even though the
  // goal predicate accepted the surrounding region. Subtract the admissible
  // terminal displacement/orientation so every term guides toward the region.
  const double distance = profile.accept_goal_region
      ? std::max(0.0, exact_distance - profile.goal_position_tolerance)
      : exact_distance;
  const double orientation = profile.accept_goal_region
      ? std::max(0.0,
                 exact_orientation - profile.goal_yaw_tolerance)
      : exact_orientation;
  const double maximum_linear = std::max(
      profile.max_forward_speed,
      profile.allow_reverse ? profile.max_reverse_speed : 0.0);
  const double distance_time = distance / std::max(maximum_linear, kEpsilon);
  const double orientation_time = orientation /
      std::max(profile.max_angular_speed, kEpsilon);
  const double curvature_time = config.minimum_turning_radius * orientation /
      std::max(maximum_linear, kEpsilon);
  double result = std::max(distance_time,
                           std::max(orientation_time, curvature_time));
  if (use_nonholonomic && config.use_nonholonomic_heuristic &&
      profile.allow_reverse)
  {
    const ReedsSheppPath path = shortestReedsSheppPath(
        x, y, yaw, goal_x, goal_y, goal_yaw,
        config.minimum_turning_radius);
    if (path.valid())
    {
      // The shortest geometric Reeds-Shepp length divided by the fastest
      // allowed linear speed is a lower bound even when the selected curve
      // contains slower reverse segments. Keep all non-negative steering,
      // obstacle and direction-change penalties out of the heuristic.
      double path_length =
          path.normalized_length * config.minimum_turning_radius;
      if (profile.accept_goal_region)
      {
        // Position and yaw allowances may overlap. Subtracting both is
        // conservative and removes the exact-centre bias from queue ordering.
        path_length = std::max(
            0.0, path_length - profile.goal_position_tolerance -
                config.minimum_turning_radius *
                    profile.goal_yaw_tolerance);
      }
      result = std::max(
          result, path_length / std::max(maximum_linear, kEpsilon));
    }
  }
  if (obstacle_heuristic && obstacle_heuristic->ready())
  {
    const double obstacle_distance = obstacle_heuristic->distance(x, y);
    if (std::isfinite(obstacle_distance))
    {
      const double distance_to_region = profile.accept_goal_region
          ? std::max(0.0, obstacle_distance -
                              profile.goal_position_tolerance)
          : obstacle_distance;
      result = std::max(
          result, distance_to_region /
                      std::max(maximum_linear, kEpsilon));
    }
  }
  return result;
}

}  // namespace

bool HybridAStarPlanner::makePlan(
    costmap_2d::Costmap2D* costmap,
    const std::vector<geometry_msgs::Point>& footprint,
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    const HybridAStarConfig& config,
    const HybridAStarProfile& profile,
    std::vector<geometry_msgs::PoseStamped>& plan,
    HybridAStarStatistics& statistics,
    std::string& reason) const
{
  plan.clear();
  statistics = HybridAStarStatistics();
  if (!costmap || footprint.size() < 3)
  {
    reason = "Hybrid A* requires a costmap and polygon footprint";
    return false;
  }
  if (start.header.frame_id.empty() || start.header.frame_id != goal.header.frame_id)
  {
    reason = "Hybrid A* start and goal must use the same non-empty frame";
    return false;
  }
  if (!validConfiguration(config, profile, reason))
    return false;

  double start_yaw = 0.0;
  double goal_yaw = 0.0;
  if (!quaternionYaw(start.pose.orientation, start_yaw) ||
      !quaternionYaw(goal.pose.orientation, goal_yaw) ||
      !std::isfinite(start.pose.position.x) ||
      !std::isfinite(start.pose.position.y) ||
      !std::isfinite(goal.pose.position.x) ||
      !std::isfinite(goal.pose.position.y))
  {
    reason = "Hybrid A* received a non-finite pose";
    return false;
  }

  // The deadline covers distance-field construction and the search itself.
  const ros::WallTime deadline = ros::WallTime::now() +
                                 ros::WallDuration(config.planning_timeout);

  double inscribed_radius = 0.0;
  double circumscribed_radius = 0.0;
  costmap_2d::calculateMinAndMaxDistances(
      footprint, inscribed_radius, circumscribed_radius);
  base_local_planner::CostmapModel world_model(*costmap);

  auto poseCost = [&](double x, double y, double yaw, double* normalized_cost) {
    unsigned int map_x = 0;
    unsigned int map_y = 0;
    if (!costmap->worldToMap(x, y, map_x, map_y))
      return false;
    const unsigned char center_cost = costmap->getCost(map_x, map_y);
    if (center_cost == costmap_2d::NO_INFORMATION ||
        center_cost == costmap_2d::LETHAL_OBSTACLE)
      return false;
    const double footprint_cost = world_model.footprintCost(
        x, y, yaw, footprint, inscribed_radius, circumscribed_radius);
    if (footprint_cost < 0.0)
      return false;
    if (normalized_cost)
    {
      const double combined = std::max<double>(center_cost, footprint_cost);
      *normalized_cost = std::min(1.0, combined / 252.0);
    }
    return true;
  };

  if (!poseCost(start.pose.position.x, start.pose.position.y, start_yaw, nullptr))
  {
    reason = "Hybrid A* start footprint is in occupied or unknown space";
    return false;
  }
  if (!poseCost(goal.pose.position.x, goal.pose.position.y, goal_yaw, nullptr))
  {
    reason = "Hybrid A* goal footprint is in occupied or unknown space";
    return false;
  }

  struct AnalyticCandidate
  {
    bool valid = false;
    double cost = std::numeric_limits<double>::infinity();
    double reverse_distance = 0.0;
    unsigned int direction_changes = 0;
    std::vector<geometry_msgs::PoseStamped> poses;
  };

  auto preferAnalyticCandidate = [](double candidate_cost,
                                    unsigned int candidate_changes,
                                    const AnalyticCandidate& current) {
    if (!current.valid)
      return true;
    if (candidate_changes < current.direction_changes)
      return candidate_cost <= 1.05 * current.cost + kEpsilon;
    if (candidate_changes > current.direction_changes)
      return 1.05 * candidate_cost < current.cost - kEpsilon;
    return candidate_cost + kEpsilon < current.cost;
  };

  auto findExactAnalyticCandidate = [&](double source_x, double source_y,
                                        double source_yaw, int initial_gear,
                                        double initial_curvature,
                                        double target_x, double target_y,
                                        double target_yaw) {
    AnalyticCandidate best;
    const std::vector<ReedsSheppPath> paths = allReedsSheppPaths(
        source_x, source_y, source_yaw,
        target_x, target_y, target_yaw,
        config.minimum_turning_radius);
    for (const ReedsSheppPath& path : paths)
    {
      double x = source_x;
      double y = source_y;
      double yaw = source_yaw;
      double total_cost = 0.0;
      double reverse_distance = 0.0;
      unsigned int direction_changes = 0;
      int previous_gear = initial_gear;
      double previous_curvature = initial_curvature;
      bool collision_free = true;
      std::vector<geometry_msgs::PoseStamped> poses;
      poses.reserve(static_cast<std::size_t>(std::ceil(
          path.normalized_length * config.minimum_turning_radius /
          config.motion_step)) + 2u);
      geometry_msgs::PoseStamped source;
      source.header.frame_id = start.header.frame_id;
      source.pose.position.x = source_x;
      source.pose.position.y = source_y;
      source.pose.orientation = yawQuaternion(source_yaw);
      poses.push_back(source);

      for (std::size_t segment = 0;
           segment < path.lengths.size() && collision_free; ++segment)
      {
        const double normalized_length = path.lengths[segment];
        if (std::abs(normalized_length) <= kEpsilon ||
            path.types[segment] == ReedsSheppSegmentType::NOP)
          continue;
        const int gear = normalized_length > 0.0 ? 1 : -1;
        if (gear < 0 && !profile.allow_reverse)
        {
          collision_free = false;
          break;
        }
        double curvature = 0.0;
        if (path.types[segment] == ReedsSheppSegmentType::LEFT)
          curvature = 1.0 / config.minimum_turning_radius;
        else if (path.types[segment] == ReedsSheppSegmentType::RIGHT)
          curvature = -1.0 / config.minimum_turning_radius;

        const bool changed_direction = previous_gear != 0 &&
                                       previous_gear != gear;
        if (changed_direction)
        {
          total_cost += profile.direction_change_penalty;
          ++direction_changes;
        }
        total_cost += config.steering_change_penalty *
                      std::abs(curvature - previous_curvature) *
                      config.minimum_turning_radius;

        double remaining = std::abs(normalized_length) *
                           config.minimum_turning_radius;
        while (remaining > kEpsilon && collision_free)
        {
          const double distance = std::min(config.motion_step, remaining);
          const double signed_distance = gear * distance;
          const int checks = std::max(1, static_cast<int>(std::ceil(
              distance / config.collision_check_step)));
          double next_x = x;
          double next_y = y;
          double next_yaw = yaw;
          double maximum_obstacle_cost = 0.0;
          for (int check = 1; check <= checks; ++check)
          {
            const double partial = signed_distance *
                static_cast<double>(check) / static_cast<double>(checks);
            integrate(x, y, yaw, partial, curvature,
                      next_x, next_y, next_yaw);
            double obstacle_cost = 0.0;
            if (!poseCost(next_x, next_y, next_yaw, &obstacle_cost))
            {
              collision_free = false;
              break;
            }
            maximum_obstacle_cost = std::max(maximum_obstacle_cost,
                                             obstacle_cost);
            geometry_msgs::PoseStamped sampled;
            sampled.header.frame_id = start.header.frame_id;
            sampled.pose.position.x = next_x;
            sampled.pose.position.y = next_y;
            sampled.pose.orientation = yawQuaternion(next_yaw);
            poses.push_back(std::move(sampled));
          }
          if (!collision_free)
            break;

          double effective_speed = gear > 0
              ? profile.max_forward_speed : profile.max_reverse_speed;
          if (std::abs(curvature) > kEpsilon)
          {
            effective_speed = std::min(
                effective_speed,
                profile.max_angular_speed / std::abs(curvature));
          }
          if (!finitePositive(effective_speed))
          {
            collision_free = false;
            break;
          }
          const double primitive_fraction = distance / config.motion_step;
          total_cost += distance / effective_speed;
          total_cost += primitive_fraction * config.steering_penalty *
                        std::abs(curvature) *
                        config.minimum_turning_radius;
          total_cost += primitive_fraction * config.obstacle_cost_scale *
                        maximum_obstacle_cost;
          if (gear < 0)
            reverse_distance += distance;

          x = next_x;
          y = next_y;
          yaw = next_yaw;
          remaining -= distance;
        }
        previous_gear = gear;
        previous_curvature = curvature;
      }
      if (!collision_free ||
          std::hypot(x - target_x, y - target_y) > 1e-5 ||
          yawError(yaw, target_yaw) > 1e-5)
        continue;
      if (!preferAnalyticCandidate(
              total_cost, direction_changes, best))
        continue;
      geometry_msgs::PoseStamped target = goal;
      target.pose.position.x = target_x;
      target.pose.position.y = target_y;
      target.pose.orientation = yawQuaternion(target_yaw);
      if (poses.empty() ||
          std::hypot(poses.back().pose.position.x - target_x,
                     poses.back().pose.position.y - target_y) > kEpsilon ||
          yawError(yaw, target_yaw) > kEpsilon)
      {
        poses.push_back(target);
      }
      else
      {
        poses.back() = target;
      }
      best.valid = true;
      best.cost = total_cost;
      best.reverse_distance = reverse_distance;
      best.direction_changes = direction_changes;
      best.poses = std::move(poses);
    }
    return best;
  };

  AnalyticCandidate direct;
  if (config.use_analytic_expansion)
  {
    direct = findExactAnalyticCandidate(
        start.pose.position.x, start.pose.position.y, start_yaw, 0, 0.0,
        goal.pose.position.x, goal.pose.position.y, goal_yaw);
  }
  auto useAnalyticCandidate = [&](const AnalyticCandidate& candidate,
                                  const std::string& selected_reason) {
    plan = candidate.poses;
    const ros::Time stamp = ros::Time::now();
    for (geometry_msgs::PoseStamped& pose : plan)
      pose.header.stamp = stamp;
    statistics.estimated_time = candidate.cost;
    statistics.reverse_distance = candidate.reverse_distance;
    statistics.direction_changes = candidate.direction_changes;
    statistics.used_analytic_expansion = true;
    reason = selected_reason;
    return plan.size() >= 2;
  };

  double direct_lower_bound = std::numeric_limits<double>::infinity();
  const ReedsSheppPath geometric_shortest = shortestReedsSheppPath(
      start.pose.position.x, start.pose.position.y, start_yaw,
      goal.pose.position.x, goal.pose.position.y, goal_yaw,
      config.minimum_turning_radius);
  if (geometric_shortest.valid())
  {
    const double maximum_linear = std::max(
        profile.max_forward_speed,
        profile.allow_reverse ? profile.max_reverse_speed : 0.0);
    direct_lower_bound = geometric_shortest.normalized_length *
        config.minimum_turning_radius /
        std::max(maximum_linear, kEpsilon);
  }
  // A very expensive analytic detour usually means a nearby obstacle has
  // invalidated the useful canonical curves. Do not let such a curve replace
  // the obstacle-aware lattice; retain only a bounded incumbent and give the
  // lattice a short window to improve easy/open transitions.
  const bool has_direct_incumbent = direct.valid &&
      std::isfinite(direct_lower_bound) &&
      direct.cost <= 2.0 * direct_lower_bound + kEpsilon;
  AnalyticCandidate analytic_incumbent;
  int analytic_parent = -1;
  double analytic_total_cost = std::numeric_limits<double>::infinity();
  double analytic_total_reverse_distance = 0.0;
  unsigned int analytic_total_direction_changes = 0;
  if (has_direct_incumbent && !profile.accept_goal_region)
  {
    analytic_incumbent = direct;
    analytic_total_cost = direct.cost;
    analytic_total_reverse_distance = direct.reverse_distance;
    analytic_total_direction_changes = direct.direction_changes;
  }
  ros::WallTime search_deadline = deadline;
  if (has_direct_incumbent && !profile.accept_goal_region)
  {
    const double delta_x = goal.pose.position.x - start.pose.position.x;
    const double delta_y = goal.pose.position.y - start.pose.position.y;
    const double longitudinal_offset =
        std::cos(start_yaw) * delta_x + std::sin(start_yaw) * delta_y;
    const double transition_distance = std::hypot(delta_x, delta_y);
    const double reversal_asymmetry = std::abs(
        kPi - yawError(start_yaw, goal_yaw));
    const bool search_for_smoother_forward_arc =
        direct.direction_changes > 0u &&
        transition_distance > config.minimum_turning_radius &&
        std::abs(longitudinal_offset) >
            profile.goal_position_tolerance &&
        reversal_asymmetry > 0.5 * profile.goal_yaw_tolerance;
    const double improvement_timeout = search_for_smoother_forward_arc
        ? config.analytic_gearchange_improvement_timeout
        : config.analytic_improvement_timeout;
    const ros::WallTime improvement_deadline = ros::WallTime::now() +
        ros::WallDuration(improvement_timeout);
    if (improvement_deadline < search_deadline)
      search_deadline = improvement_deadline;
  }

  const ObstacleHeuristic obstacle_heuristic(
      costmap, goal.pose.position.x, goal.pose.position.y,
      config.use_obstacle_heuristic);
  if (ros::WallTime::now() > search_deadline)
  {
    if (has_direct_incumbent && !profile.accept_goal_region)
    {
      return useAnalyticCandidate(
          direct, "Hybrid A* selected the collision-free analytic "
                  "Reeds-Shepp incumbent after its improvement window");
    }
    reason = "Hybrid A* planning timeout while building obstacle heuristic";
    return false;
  }
  if (obstacle_heuristic.ready() &&
      !std::isfinite(obstacle_heuristic.distance(
          start.pose.position.x, start.pose.position.y)))
  {
    reason = "Hybrid A* found no known-free 2-D connection";
    return false;
  }

  const std::size_t state_width = static_cast<std::size_t>(std::ceil(
      costmap->getSizeInMetersX() / config.state_resolution));
  const std::size_t state_height = static_cast<std::size_t>(std::ceil(
      costmap->getSizeInMetersY() / config.state_resolution));
  if (state_width == 0 || state_height == 0)
  {
    reason = "Hybrid A* costmap has no searchable extent";
    return false;
  }

  auto stateKey = [&](double x, double y, double yaw, int gear,
                      int steering_index, std::uint64_t& key) {
    const double relative_x = (x - costmap->getOriginX()) /
                              config.state_resolution;
    const double relative_y = (y - costmap->getOriginY()) /
                              config.state_resolution;
    if (!std::isfinite(relative_x) || !std::isfinite(relative_y) ||
        relative_x < 0.0 || relative_y < 0.0)
      return false;
    const std::size_t x_index = static_cast<std::size_t>(relative_x);
    const std::size_t y_index = static_cast<std::size_t>(relative_y);
    if (x_index >= state_width || y_index >= state_height)
      return false;
    const int yaw_index = static_cast<int>(std::llround(
        positiveYaw(yaw) * config.heading_bins / kTwoPi)) %
        config.heading_bins;
    const std::uint64_t gear_index = gear < 0 ? 1u : 0u;
    key = static_cast<std::uint64_t>(y_index * state_width + x_index);
    key = key * static_cast<std::uint64_t>(config.heading_bins) +
          static_cast<std::uint64_t>(yaw_index);
    key = key * 2u + gear_index;
    key = key * static_cast<std::uint64_t>(config.steering_samples) +
          static_cast<std::uint64_t>(steering_index);
    return true;
  };

  const int center_steering = config.steering_samples / 2;
  std::vector<double> curvatures;
  curvatures.reserve(config.steering_samples);
  for (int index = 0; index < config.steering_samples; ++index)
  {
    const double normalized = static_cast<double>(index - center_steering) /
                              static_cast<double>(center_steering);
    curvatures.push_back(normalized / config.minimum_turning_radius);
  }

  std::vector<SearchNode> nodes;
  nodes.reserve(std::min(config.max_expansions * 2, 250000));
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueGreater> frontier;
  std::unordered_map<std::uint64_t, double> best_cost;
  best_cost.reserve(std::min(config.max_expansions * 2, 250000));

  SearchNode root;
  root.x = start.pose.position.x;
  root.y = start.pose.position.y;
  root.yaw = start_yaw;
  root.steering_index = center_steering;
  root.f = config.heuristic_weight * heuristic(
      root.x, root.y, root.yaw, goal.pose.position.x, goal.pose.position.y,
      goal_yaw, config, profile, &obstacle_heuristic,
      !analytic_incumbent.valid);
  if (!stateKey(root.x, root.y, root.yaw, root.gear,
                root.steering_index, root.key))
  {
    reason = "Hybrid A* start is outside the search lattice";
    return false;
  }
  nodes.push_back(root);
  best_cost[root.key] = 0.0;
  frontier.push({root.f, 0});

  auto appendDenseSearchChain = [&](
      const std::vector<int>& chain,
      std::vector<geometry_msgs::PoseStamped>& output) {
    if (chain.empty())
      return;
    geometry_msgs::PoseStamped root_pose;
    root_pose.header.frame_id = start.header.frame_id;
    root_pose.pose.position.x = nodes[chain.front()].x;
    root_pose.pose.position.y = nodes[chain.front()].y;
    root_pose.pose.orientation = yawQuaternion(nodes[chain.front()].yaw);
    output.push_back(std::move(root_pose));
    const int checks = std::max(1, static_cast<int>(std::ceil(
        config.motion_step / config.collision_check_step)));
    for (std::size_t chain_index = 1;
         chain_index < chain.size(); ++chain_index)
    {
      const SearchNode& parent = nodes[chain[chain_index - 1]];
      const SearchNode& child = nodes[chain[chain_index]];
      const double signed_step = child.gear * config.motion_step;
      for (int check = 1; check <= checks; ++check)
      {
        const double partial = signed_step *
            static_cast<double>(check) / static_cast<double>(checks);
        double x = parent.x;
        double y = parent.y;
        double yaw = parent.yaw;
        integrate(parent.x, parent.y, parent.yaw, partial, child.curvature,
                  x, y, yaw);
        geometry_msgs::PoseStamped sampled;
        sampled.header.frame_id = start.header.frame_id;
        sampled.pose.position.x = x;
        sampled.pose.position.y = y;
        sampled.pose.orientation = yawQuaternion(yaw);
        output.push_back(std::move(sampled));
      }
      // Preserve the exact lattice endpoint instead of accumulating floating
      // point integration error across a long reconstructed chain.
      output.back().pose.position.x = child.x;
      output.back().pose.position.y = child.y;
      output.back().pose.orientation = yawQuaternion(child.yaw);
    }
  };

  auto useSearchAnalyticCandidate = [&](const std::string& selected_reason) {
    plan.clear();
    if (analytic_parent >= 0)
    {
      std::vector<int> chain;
      for (int index = analytic_parent; index >= 0;
           index = nodes[index].parent)
        chain.push_back(index);
      std::reverse(chain.begin(), chain.end());
      plan.reserve(
          chain.size() * static_cast<std::size_t>(std::ceil(
              config.motion_step / config.collision_check_step)) +
          analytic_incumbent.poses.size());
      appendDenseSearchChain(chain, plan);
      if (!analytic_incumbent.poses.empty())
      {
        plan.insert(plan.end(), analytic_incumbent.poses.begin() + 1,
                    analytic_incumbent.poses.end());
      }
    }
    else
    {
      plan = analytic_incumbent.poses;
    }
    const ros::Time stamp = ros::Time::now();
    for (geometry_msgs::PoseStamped& pose : plan)
      pose.header.stamp = stamp;
    statistics.estimated_time = analytic_total_cost;
    statistics.reverse_distance = analytic_total_reverse_distance;
    statistics.direction_changes = analytic_total_direction_changes;
    statistics.used_analytic_expansion = true;
    reason = selected_reason;
    return plan.size() >= 2;
  };

  int goal_node = -1;
  AnalyticCandidate lattice_terminal;
  double lattice_total_cost = std::numeric_limits<double>::infinity();
  double lattice_total_reverse_distance = 0.0;
  unsigned int lattice_total_direction_changes = 0;
  while (!frontier.empty() &&
         statistics.expansions < static_cast<std::size_t>(config.max_expansions))
  {
    if (ros::WallTime::now() > search_deadline)
    {
      reason = "Hybrid A* planning timeout";
      break;
    }
    const QueueEntry entry = frontier.top();
    frontier.pop();
    if (entry.node_index >= nodes.size())
      continue;
    const SearchNode current = nodes[entry.node_index];
    const auto best = best_cost.find(current.key);
    if (best == best_cost.end() || current.g > best->second + kEpsilon)
      continue;
    ++statistics.expansions;

    const double position_error = std::hypot(
        goal.pose.position.x - current.x,
        goal.pose.position.y - current.y);
    if (position_error <= profile.goal_position_tolerance &&
        yawError(current.yaw, goal_yaw) <= profile.goal_yaw_tolerance)
    {
      if (profile.accept_goal_region)
      {
        // Entry alignment is a pose-acquisition problem, not an exact-point
        // parking problem.  The caller has explicitly requested the same
        // position/yaw tolerance that guards the following sweep.  Stop at
        // the first cost-optimal lattice state inside that basin instead of
        // appending a Reeds-Shepp loop merely to hit the mathematical center.
        goal_node = static_cast<int>(entry.node_index);
        lattice_total_cost = current.g;
        lattice_total_reverse_distance = current.reverse_distance;
        lattice_total_direction_changes = current.direction_changes;
        reason = "Hybrid A* acquired the requested sweep-entry goal region";
        break;
      }
      // Goal tolerances are only a trigger for attempting an exact terminal
      // connection.  Appending the action goal directly would create a final
      // edge that is collision-checked but can violate the Ackermann turning
      // radius.  The Reeds-Shepp connector below is sampled and footprint-
      // checked with the same kinematic bound as every other analytic edge.
      AnalyticCandidate terminal = findExactAnalyticCandidate(
          current.x, current.y, current.yaw,
          current.gear, current.curvature,
          goal.pose.position.x, goal.pose.position.y, goal_yaw);
      if (terminal.valid)
      {
        goal_node = static_cast<int>(entry.node_index);
        lattice_total_cost = current.g + terminal.cost;
        lattice_total_reverse_distance = current.reverse_distance +
            terminal.reverse_distance;
        lattice_total_direction_changes = current.direction_changes +
            terminal.direction_changes;
        lattice_terminal = std::move(terminal);
        reason = "Hybrid A* found an exact kinematically feasible coverage "
                 "transition";
        break;
      }
    }

    if (config.use_analytic_expansion && !profile.accept_goal_region &&
        statistics.expansions % static_cast<std::size_t>(
            config.analytic_expansion_interval) == 0u)
    {
      const AnalyticCandidate connector = findExactAnalyticCandidate(
          current.x, current.y, current.yaw,
          current.gear, current.curvature,
          goal.pose.position.x, goal.pose.position.y, goal_yaw);
      const double connector_total = current.g + connector.cost;
      const bool had_analytic_incumbent = analytic_incumbent.valid;
      if (connector.valid &&
          connector_total + kEpsilon < analytic_total_cost)
      {
        analytic_incumbent = connector;
        analytic_parent = static_cast<int>(entry.node_index);
        analytic_total_cost = connector_total;
        analytic_total_reverse_distance = current.reverse_distance +
            connector.reverse_distance;
        analytic_total_direction_changes = current.direction_changes +
            connector.direction_changes;
        if (!had_analytic_incumbent)
        {
          const ros::WallTime improvement_deadline = ros::WallTime::now() +
              ros::WallDuration(
                  config.analytic_connector_improvement_timeout);
          if (improvement_deadline < search_deadline)
            search_deadline = improvement_deadline;
        }
      }
    }

    const int minimum_gear = profile.allow_reverse ? -1 : 1;
    for (int gear = minimum_gear; gear <= 1; gear += 2)
    {
      const double configured_speed = gear > 0
                                          ? profile.max_forward_speed
                                          : profile.max_reverse_speed;
      if (!finitePositive(configured_speed))
        continue;
      for (int steering_index = 0;
           steering_index < config.steering_samples; ++steering_index)
      {
        const double curvature = curvatures[steering_index];
        const double signed_step = gear * config.motion_step;
        bool collision_free = true;
        double maximum_obstacle_cost = 0.0;
        const int checks = std::max(1, static_cast<int>(std::ceil(
            config.motion_step / config.collision_check_step)));
        double next_x = current.x;
        double next_y = current.y;
        double next_yaw = current.yaw;
        for (int check = 1; check <= checks; ++check)
        {
          const double partial = signed_step *
              static_cast<double>(check) / static_cast<double>(checks);
          integrate(current.x, current.y, current.yaw, partial, curvature,
                    next_x, next_y, next_yaw);
          double obstacle_cost = 0.0;
          if (!poseCost(next_x, next_y, next_yaw, &obstacle_cost))
          {
            collision_free = false;
            break;
          }
          maximum_obstacle_cost = std::max(maximum_obstacle_cost,
                                           obstacle_cost);
        }
        if (!collision_free)
          continue;

        std::uint64_t key = 0;
        if (!stateKey(next_x, next_y, next_yaw, gear,
                      steering_index, key))
          continue;
        double effective_speed = configured_speed;
        if (std::abs(curvature) > kEpsilon)
        {
          effective_speed = std::min(
              effective_speed,
              profile.max_angular_speed / std::abs(curvature));
        }
        if (!finitePositive(effective_speed))
          continue;
        double primitive_cost = config.motion_step / effective_speed;
        primitive_cost += config.steering_penalty *
                          std::abs(curvature) *
                          config.minimum_turning_radius;
        primitive_cost += config.steering_change_penalty *
                          std::abs(curvature - current.curvature) *
                          config.minimum_turning_radius;
        primitive_cost += config.obstacle_cost_scale * maximum_obstacle_cost;
        const bool changed_direction = current.gear != 0 &&
                                       current.gear != gear;
        if (changed_direction)
          primitive_cost += profile.direction_change_penalty;
        const double candidate_g = current.g + primitive_cost;
        const unsigned int candidate_direction_changes =
            current.direction_changes + (changed_direction ? 1u : 0u);
        const auto old = best_cost.find(key);
        if (old != best_cost.end() && candidate_g >= old->second - kEpsilon)
          continue;

        SearchNode successor;
        successor.x = next_x;
        successor.y = next_y;
        successor.yaw = next_yaw;
        successor.g = candidate_g;
        const double remaining_cost = heuristic(
            next_x, next_y, next_yaw,
            goal.pose.position.x, goal.pose.position.y, goal_yaw,
            config, profile, &obstacle_heuristic,
            !analytic_incumbent.valid);
        // Grid Dijkstra is excellent for queue ordering but can overestimate
        // a continuous path by a fraction of a cell. Use only the continuous
        // obstacle-free lower bound for incumbent pruning.
        const double pruning_lower_bound = analytic_incumbent.valid
            ? heuristic(next_x, next_y, next_yaw,
                        goal.pose.position.x, goal.pose.position.y, goal_yaw,
                        config, profile, nullptr, false)
            : remaining_cost;
        if (analytic_incumbent.valid &&
            candidate_direction_changes >=
                analytic_total_direction_changes &&
            candidate_g + pruning_lower_bound >=
                1.05 * analytic_total_cost - kEpsilon)
          continue;
        successor.f = candidate_g +
            config.heuristic_weight * remaining_cost;
        successor.parent = static_cast<int>(entry.node_index);
        successor.gear = gear;
        successor.steering_index = steering_index;
        successor.curvature = curvature;
        successor.reverse_distance = current.reverse_distance +
                                     (gear < 0 ? config.motion_step : 0.0);
        successor.direction_changes = candidate_direction_changes;
        successor.key = key;
        const std::size_t successor_index = nodes.size();
        nodes.push_back(successor);
        best_cost[key] = candidate_g;
        frontier.push({successor.f, successor_index});
      }
    }
  }

  if (goal_node < 0)
  {
    if (analytic_incumbent.valid)
    {
      return useSearchAnalyticCandidate(
          "Hybrid A* selected the collision-free analytic Reeds-Shepp "
          "incumbent after its improvement window");
    }
    if (reason.empty())
      reason = frontier.empty()
                   ? "Hybrid A* exhausted the reachable kinematic lattice"
                   : "Hybrid A* expansion limit reached";
    return false;
  }

  const bool lattice_is_preferred = analytic_incumbent.valid &&
      (lattice_total_direction_changes <
           analytic_total_direction_changes
           ? lattice_total_cost <=
                 1.05 * analytic_total_cost + kEpsilon
           : (lattice_total_direction_changes >
                  analytic_total_direction_changes
                  ? 1.05 * lattice_total_cost <
                        analytic_total_cost - kEpsilon
                  : lattice_total_cost <=
                        analytic_total_cost + kEpsilon));
  if (analytic_incumbent.valid && !lattice_is_preferred)
  {
    return useSearchAnalyticCandidate(
        "Hybrid A* selected the lower-time collision-free analytic "
        "Reeds-Shepp incumbent");
  }

  std::vector<int> chain;
  for (int index = goal_node; index >= 0; index = nodes[index].parent)
    chain.push_back(index);
  std::reverse(chain.begin(), chain.end());
  const ros::Time stamp = ros::Time::now();
  plan.reserve(
      chain.size() * static_cast<std::size_t>(std::ceil(
          config.motion_step / config.collision_check_step)) +
      lattice_terminal.poses.size());
  appendDenseSearchChain(chain, plan);
  if (lattice_terminal.poses.size() >= 2)
  {
    plan.insert(plan.end(), lattice_terminal.poses.begin() + 1,
                lattice_terminal.poses.end());
  }
  for (geometry_msgs::PoseStamped& pose : plan)
    pose.header.stamp = stamp;
  statistics.estimated_time = lattice_total_cost;
  statistics.reverse_distance = lattice_total_reverse_distance;
  statistics.direction_changes = lattice_total_direction_changes;
  statistics.used_analytic_expansion = !lattice_terminal.poses.empty();
  return plan.size() >= 2;
}

}  // namespace autolabor_coverage
