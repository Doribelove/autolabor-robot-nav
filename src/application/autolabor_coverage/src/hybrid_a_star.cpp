#include "autolabor_coverage/hybrid_a_star.h"

#include <base_local_planner/costmap_model.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/footprint.h>
#include <ros/time.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
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
      !finitePositive(config.planning_timeout) ||
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

double heuristic(double x, double y, double yaw,
                 double goal_x, double goal_y, double goal_yaw,
                 const HybridAStarConfig& config,
                 const HybridAStarProfile& profile)
{
  const double distance = std::hypot(goal_x - x, goal_y - y);
  const double orientation = yawError(yaw, goal_yaw);
  const double maximum_linear = std::max(
      profile.max_forward_speed,
      profile.allow_reverse ? profile.max_reverse_speed : 0.0);
  const double distance_time = distance / std::max(maximum_linear, kEpsilon);
  const double orientation_time = orientation /
      std::max(profile.max_angular_speed, kEpsilon);
  const double curvature_time = config.minimum_turning_radius * orientation /
      std::max(maximum_linear, kEpsilon);
  return std::max(distance_time, std::max(orientation_time, curvature_time));
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
      goal_yaw, config, profile);
  if (!stateKey(root.x, root.y, root.yaw, root.gear,
                root.steering_index, root.key))
  {
    reason = "Hybrid A* start is outside the search lattice";
    return false;
  }
  nodes.push_back(root);
  best_cost[root.key] = 0.0;
  frontier.push({root.f, 0});

  const ros::WallTime deadline = ros::WallTime::now() +
                                 ros::WallDuration(config.planning_timeout);
  int goal_node = -1;
  while (!frontier.empty() &&
         statistics.expansions < static_cast<std::size_t>(config.max_expansions))
  {
    if (ros::WallTime::now() > deadline)
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
      goal_node = static_cast<int>(entry.node_index);
      reason = "Hybrid A* found a kinematically feasible coverage transition";
      break;
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
        const auto old = best_cost.find(key);
        if (old != best_cost.end() && candidate_g >= old->second - kEpsilon)
          continue;

        SearchNode successor;
        successor.x = next_x;
        successor.y = next_y;
        successor.yaw = next_yaw;
        successor.g = candidate_g;
        successor.f = candidate_g + config.heuristic_weight * heuristic(
            next_x, next_y, next_yaw,
            goal.pose.position.x, goal.pose.position.y, goal_yaw,
            config, profile);
        successor.parent = static_cast<int>(entry.node_index);
        successor.gear = gear;
        successor.steering_index = steering_index;
        successor.curvature = curvature;
        successor.reverse_distance = current.reverse_distance +
                                     (gear < 0 ? config.motion_step : 0.0);
        successor.direction_changes = current.direction_changes +
                                      (changed_direction ? 1u : 0u);
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
    if (reason.empty())
      reason = frontier.empty()
                   ? "Hybrid A* exhausted the reachable kinematic lattice"
                   : "Hybrid A* expansion limit reached";
    return false;
  }

  std::vector<int> chain;
  for (int index = goal_node; index >= 0; index = nodes[index].parent)
    chain.push_back(index);
  std::reverse(chain.begin(), chain.end());
  const ros::Time stamp = ros::Time::now();
  plan.reserve(chain.size() + 1);
  for (int index : chain)
  {
    const SearchNode& node = nodes[index];
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = start.header.frame_id;
    pose.header.stamp = stamp;
    pose.pose.position.x = node.x;
    pose.pose.position.y = node.y;
    pose.pose.orientation = yawQuaternion(node.yaw);
    plan.push_back(std::move(pose));
  }
  geometry_msgs::PoseStamped exact_goal = goal;
  exact_goal.header.stamp = stamp;
  if (plan.size() < 2 ||
      std::hypot(plan.back().pose.position.x - exact_goal.pose.position.x,
                 plan.back().pose.position.y - exact_goal.pose.position.y) >
          kEpsilon ||
      yawError(nodes[goal_node].yaw, goal_yaw) > kEpsilon)
  {
    plan.push_back(std::move(exact_goal));
  }
  statistics.estimated_time = nodes[goal_node].g;
  statistics.reverse_distance = nodes[goal_node].reverse_distance;
  statistics.direction_changes = nodes[goal_node].direction_changes;
  return plan.size() >= 2;
}

}  // namespace autolabor_coverage
