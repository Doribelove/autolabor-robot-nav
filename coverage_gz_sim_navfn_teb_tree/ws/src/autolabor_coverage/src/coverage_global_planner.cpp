#include "autolabor_coverage/coverage_global_planner.h"

#include <base_local_planner/costmap_model.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/footprint.h>
#include <pluginlib/class_list_macros.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

namespace autolabor_coverage
{
namespace
{

costmap_2d::Costmap2D snapshotCostmap(
    costmap_2d::Costmap2DROS* costmap_ros)
{
  costmap_2d::Costmap2D* live = costmap_ros->getCostmap();
  std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*live->getMutex());
  return costmap_2d::Costmap2D(*live);
}

bool normalizedYaw(const geometry_msgs::Quaternion& quaternion, double& yaw)
{
  const double squared_norm = quaternion.x * quaternion.x +
                              quaternion.y * quaternion.y +
                              quaternion.z * quaternion.z +
                              quaternion.w * quaternion.w;
  if (!std::isfinite(squared_norm) || squared_norm < 1.0e-12)
    return false;
  const double inverse_norm = 1.0 / std::sqrt(squared_norm);
  const double x = quaternion.x * inverse_norm;
  const double y = quaternion.y * inverse_norm;
  const double z = quaternion.z * inverse_norm;
  const double w = quaternion.w * inverse_norm;
  yaw = std::atan2(2.0 * (w * z + x * y),
                   1.0 - 2.0 * (y * y + z * z));
  return std::isfinite(yaw);
}

double yawError(double first, double second)
{
  return std::abs(std::atan2(std::sin(second - first),
                            std::cos(second - first)));
}

HybridAStarProfile hybridProfile(
    const autolabor_coverage::TransitProfile& message)
{
  HybridAStarProfile profile;
  profile.allow_reverse = message.allow_reverse;
  profile.max_forward_speed = message.max_forward_speed_mps;
  profile.max_reverse_speed = message.max_reverse_speed_mps;
  profile.max_angular_speed = message.max_angular_speed_rps;
  profile.linear_acceleration = message.linear_accel_mps2;
  profile.angular_acceleration = message.angular_accel_rps2;
  profile.direction_change_penalty = message.direction_change_penalty_sec;
  profile.goal_position_tolerance = message.goal_position_tolerance_m;
  profile.goal_yaw_tolerance = message.goal_yaw_tolerance_rad;
  return profile;
}

bool validHybridProfile(const HybridAStarProfile& profile)
{
  const auto positive = [](double value) {
    return std::isfinite(value) && value > 0.0;
  };
  return positive(profile.max_forward_speed) &&
         positive(profile.max_angular_speed) &&
         positive(profile.linear_acceleration) &&
         positive(profile.angular_acceleration) &&
         positive(profile.goal_position_tolerance) &&
         positive(profile.goal_yaw_tolerance) &&
         std::isfinite(profile.direction_change_penalty) &&
         profile.direction_change_penalty >= 0.0 &&
         (!profile.allow_reverse || positive(profile.max_reverse_speed));
}

bool validReplanPeriod(double value)
{
  return std::isfinite(value) && value >= 1.0 && value <= 10.0;
}

bool selectRollingGoal(
    const std::vector<geometry_msgs::PoseStamped>& topology,
    double horizon,
    geometry_msgs::PoseStamped& goal,
    bool& reaches_final_goal,
    std::string& reason)
{
  if (topology.size() < 2 || !std::isfinite(horizon) || horizon <= 0.0)
  {
    reason = "rolling topology or horizon is invalid";
    return false;
  }
  double accumulated = 0.0;
  for (std::size_t index = 1; index < topology.size(); ++index)
  {
    const auto& previous = topology[index - 1].pose.position;
    const auto& next = topology[index].pose.position;
    const double dx = next.x - previous.x;
    const double dy = next.y - previous.y;
    const double edge_length = std::hypot(dx, dy);
    if (!std::isfinite(edge_length))
    {
      reason = "rolling topology contains a non-finite edge";
      return false;
    }
    if (edge_length > 1.0e-9 && accumulated + edge_length >= horizon)
    {
      const double fraction = std::max(
          0.0, std::min(1.0, (horizon - accumulated) / edge_length));
      goal = topology[index];
      goal.pose.position.x = previous.x + fraction * dx;
      goal.pose.position.y = previous.y + fraction * dy;
      goal.pose.position.z = previous.z +
          fraction * (next.z - previous.z);
      const double yaw = std::atan2(dy, dx);
      goal.pose.orientation.x = 0.0;
      goal.pose.orientation.y = 0.0;
      goal.pose.orientation.z = std::sin(0.5 * yaw);
      goal.pose.orientation.w = std::cos(0.5 * yaw);
      reaches_final_goal = index + 1 == topology.size() &&
          fraction >= 1.0 - 1.0e-9;
      reason.clear();
      return true;
    }
    accumulated += edge_length;
  }
  goal = topology.back();
  reaches_final_goal = true;
  reason.clear();
  return true;
}

}  // namespace

CoverageGlobalPlanner::CoverageGlobalPlanner(std::string name,
                                             costmap_2d::Costmap2DROS* costmap_ros)
{
  initialize(std::move(name), costmap_ros);
}

void CoverageGlobalPlanner::initialize(std::string name,
                                       costmap_2d::Costmap2DROS* costmap_ros)
{
  if (initialized_)
    return;
  if (!costmap_ros)
    throw std::invalid_argument("CoverageGlobalPlanner requires a costmap");
  costmap_ros_ = costmap_ros;
  private_nh_ = ros::NodeHandle("~/" + name);
  private_nh_.param("goal_match_tolerance", goal_match_tolerance_, 0.35);
  private_nh_.param("goal_yaw_match_tolerance", goal_yaw_match_tolerance_, 0.20);
  private_nh_.param("path_timeout", path_timeout_, 1.0);
  private_nh_.param("hybrid_cache_max_deviation",
                    hybrid_cache_max_deviation_, 0.60);
  private_nh_.param("hybrid_cache_collision_check_horizon",
                    hybrid_cache_collision_check_horizon_, 3.00);
  private_nh_.param("hybrid_online_kinematic_horizon",
                    hybrid_online_kinematic_horizon_, 12.0);
  private_nh_.param("hybrid_replan_every_cycle",
                    hybrid_replan_every_cycle_, false);
  private_nh_.param("hybrid_minimum_turning_radius",
                    hybrid_config_.minimum_turning_radius, 1.35);
  private_nh_.param("hybrid_motion_step", hybrid_config_.motion_step, 0.30);
  private_nh_.param("hybrid_collision_check_step",
                    hybrid_config_.collision_check_step, 0.10);
  private_nh_.param("hybrid_state_resolution",
                    hybrid_config_.state_resolution, 0.15);
  private_nh_.param("hybrid_heading_bins", hybrid_config_.heading_bins, 72);
  private_nh_.param("hybrid_steering_samples",
                    hybrid_config_.steering_samples, 5);
  private_nh_.param("hybrid_max_expansions",
                    hybrid_config_.max_expansions, 80000);
  private_nh_.param("hybrid_planning_timeout",
                    hybrid_config_.planning_timeout, 1.50);
  private_nh_.param("hybrid_heuristic_weight",
                    hybrid_config_.heuristic_weight, 1.05);
  private_nh_.param("hybrid_steering_penalty",
                    hybrid_config_.steering_penalty, 0.04);
  private_nh_.param("hybrid_steering_change_penalty",
                    hybrid_config_.steering_change_penalty, 0.10);
  private_nh_.param("hybrid_obstacle_cost_scale",
                    hybrid_config_.obstacle_cost_scale, 0.25);
  private_nh_.param("hybrid_use_obstacle_heuristic",
                    hybrid_config_.use_obstacle_heuristic, true);
  private_nh_.param("hybrid_use_nonholonomic_heuristic",
                    hybrid_config_.use_nonholonomic_heuristic, true);
  private_nh_.param("hybrid_use_analytic_expansion",
                    hybrid_config_.use_analytic_expansion, true);
  private_nh_.param("hybrid_analytic_improvement_timeout",
                    hybrid_config_.analytic_improvement_timeout, 0.20);
  private_nh_.param("hybrid_analytic_gearchange_improvement_timeout",
                    hybrid_config_.analytic_gearchange_improvement_timeout,
                    0.65);
  private_nh_.param("hybrid_analytic_connector_improvement_timeout",
                    hybrid_config_.analytic_connector_improvement_timeout,
                    0.50);
  private_nh_.param("hybrid_analytic_expansion_interval",
                    hybrid_config_.analytic_expansion_interval, 200);
  const auto positive = [](double value) {
    return std::isfinite(value) && value > 0.0;
  };
  if (!positive(goal_match_tolerance_) ||
      !positive(goal_yaw_match_tolerance_) || !positive(path_timeout_) ||
      !positive(hybrid_cache_max_deviation_) ||
      !positive(hybrid_cache_collision_check_horizon_) ||
      !positive(hybrid_online_kinematic_horizon_))
  {
    throw std::invalid_argument(
        "CoverageGlobalPlanner cache and hand-off parameters must be positive");
  }
  fallback_.initialize(name + "_navfn", costmap_ros);
  path_subscriber_ = private_nh_.subscribe(
      "/coverage/enforced_path", 1, &CoverageGlobalPlanner::pathCallback, this);
  // A topic is retained for freshness refreshes, while the service provides a
  // synchronous mode hand-off before coverage_manager submits a move_base
  // goal.  This prevents a transit goal from racing the previous sweep path
  // (and prevents a sweep goal from briefly falling back to Navfn).
  set_path_service_ = private_nh_.advertiseService(
      "set_enforced_path", &CoverageGlobalPlanner::setPathCallback, this);
  precompute_service_ = private_nh_.advertiseService(
      "precompute_transitions", &CoverageGlobalPlanner::precomputeCallback,
      this);
  hybrid_path_safe_publisher_ = private_nh_.advertise<std_msgs::Bool>(
      "hybrid_path_safe", 1, true);
  publishHybridPathSafety(false);
  initialized_ = true;
}

void CoverageGlobalPlanner::publishHybridPathSafety(bool safe)
{
  std_msgs::Bool message;
  message.data = safe;
  hybrid_path_safe_publisher_.publish(message);
}

void CoverageGlobalPlanner::pathCallback(
    const autolabor_coverage::EnforcedPath::ConstPtr& message)
{
  std::string reason;
  if (!updateEnforcedPath(*message, reason))
    ROS_ERROR_THROTTLE(1.0, "coverage rejected enforced-path refresh: %s",
                       reason.c_str());
}

bool CoverageGlobalPlanner::setPathCallback(
    autolabor_coverage::SetEnforcedPath::Request& request,
    autolabor_coverage::SetEnforcedPath::Response& response)
{
  if (!validateEnforcedPath(request.enforced_path, response.message))
  {
    response.success = false;
    return true;
  }
  const std::uint8_t mode = request.enforced_path.planner_mode;
  const bool ordinary =
      mode == autolabor_coverage::EnforcedPath::MODE_ORDINARY_NAVFN;
  const bool entry =
      mode == autolabor_coverage::EnforcedPath::MODE_COVERAGE_NAVFN;
  const bool hybrid =
      mode == autolabor_coverage::EnforcedPath::MODE_HYBRID_TRANSIT;
  const bool sweep =
      mode == autolabor_coverage::EnforcedPath::MODE_ENFORCED_SWEEP;
  if ((!request.coverage_active && !ordinary) ||
      (request.coverage_active && ordinary))
  {
    response.success = false;
    response.message = "planner mode does not match coverage ownership";
    return true;
  }
  if (request.enforced_path.active != sweep)
  {
    response.success = false;
    response.message = "only ENFORCED_SWEEP may arm the enforced-path flag";
    return true;
  }
  const HybridAStarProfile requested_profile = hybridProfile(
      request.transit_profile);
  if (hybrid && (!validHybridProfile(requested_profile) ||
                 !validReplanPeriod(
                     request.transit_profile.replan_period_sec)))
  {
    response.success = false;
    response.message =
        "coverage Hybrid A* transit profile or replan period is invalid";
    return true;
  }
  {
    // Mission ownership and planner mode are one transaction.  move_base can
    // therefore never observe a newly armed sweep with stale ownership (or a
    // new transit with the previous sweep still armed).
    std::lock_guard<std::mutex> lock(mutex_);
    coverage_active_ = request.coverage_active;
    enforced_path_ = request.enforced_path;
    if (hybrid)
    {
      transit_profile_ = requested_profile;
      hybrid_replan_period_ = request.transit_profile.replan_period_sec;
    }
    enforced_path_received_ = ros::WallTime::now();
    hybrid_last_search_ = hybrid ? ros::WallTime::now() : ros::WallTime();
    hybrid_last_search_failed_ = false;
  }
  // A latched true from the previous connector must never authorize a newly
  // armed path before move_base has checked it against the latest costmap.
  publishHybridPathSafety(false);
  response.success = true;
  const char* mode_name = ordinary ? "ORDINARY_NAVFN" :
                          entry ? "COVERAGE_ENTRY_NAVFN" :
                          hybrid ? "KINEMATIC_HYBRID_ASTAR_TRANSIT" :
                                   "ENFORCED_SWEEP";
  response.message = std::string("coverage planner mode armed: ") + mode_name;
  ROS_INFO("coverage planner hand-off: plan=%s segment=%u mode=%s",
           request.enforced_path.plan_id.c_str(),
           request.enforced_path.segment_index + 1,
           mode_name);
  return true;
}

bool CoverageGlobalPlanner::precomputeCallback(
    autolabor_coverage::PrecomputeTransitions::Request& request,
    autolabor_coverage::PrecomputeTransitions::Response& response)
{
  const HybridAStarProfile base_profile = hybridProfile(request.transit_profile);
  if (request.plan_id.empty() || request.transitions.empty() ||
      !validHybridProfile(base_profile) ||
      !validReplanPeriod(request.transit_profile.replan_period_sec) ||
      !std::isfinite(request.total_timeout_sec) ||
      request.total_timeout_sec <= 0.0)
  {
    response.success = false;
    response.message = "Hybrid transition precompute request is invalid";
    return true;
  }
  const std::string frame = costmap_ros_->getGlobalFrameID();
  // This service can run while the layered costmap update thread is writing.
  // Freeze one internally consistent generation for the complete batch.
  costmap_2d::Costmap2D snapshot = snapshotCostmap(costmap_ros_);
  const auto footprint = costmap_ros_->getRobotFootprint();
  const ros::WallTime deadline = ros::WallTime::now() +
      ros::WallDuration(request.total_timeout_sec);
  std::size_t successes = 0;
  response.results.reserve(request.transitions.size());
  for (const auto& transition : request.transitions)
  {
    HybridAStarProfile profile = base_profile;
    profile.accept_goal_region = transition.accept_goal_region;
    autolabor_coverage::HybridTransitionResult result;
    result.candidate_index = transition.candidate_index;
    result.transition_index = transition.transition_index;
    result.path.header.frame_id = frame;
    result.path.header.stamp = ros::Time::now();
    result.planned_goal = transition.goal;
    result.reaches_final_goal = true;
    const double remaining = (deadline - ros::WallTime::now()).toSec();
    if (remaining <= 0.0)
    {
      result.outcome =
          autolabor_coverage::HybridTransitionResult::OUTCOME_TIMEOUT;
      result.reason = "Hybrid transition precompute batch deadline reached";
      response.results.push_back(std::move(result));
      continue;
    }
    if (transition.start.header.frame_id != frame ||
        transition.goal.header.frame_id != frame ||
        (transition.rolling &&
         (!std::isfinite(transition.rolling_horizon_m) ||
          transition.rolling_horizon_m <= 0.0)))
    {
      result.outcome =
          autolabor_coverage::HybridTransitionResult::OUTCOME_INVALID;
      result.reason = "Hybrid transition endpoints use the wrong frame";
      response.results.push_back(std::move(result));
      continue;
    }

    geometry_msgs::PoseStamped planning_goal = transition.goal;
    std::string reason;
    if (transition.rolling)
    {
      std::vector<geometry_msgs::PoseStamped> topology;
      bool reaches_final_goal = false;
      if (!fallback_.makePlan(transition.start, transition.goal, topology) ||
          !selectRollingGoal(
              topology, transition.rolling_horizon_m, planning_goal,
              reaches_final_goal, reason))
      {
        result.outcome =
            autolabor_coverage::HybridTransitionResult::OUTCOME_NO_PATH;
        result.reason = reason.empty()
            ? "rolling Navfn topology has no known-free path" : reason;
        response.results.push_back(std::move(result));
        continue;
      }
      result.reaches_final_goal = reaches_final_goal;
      if (result.reaches_final_goal)
        planning_goal = transition.goal;
      result.planned_goal = planning_goal;
    }

    HybridAStarConfig bounded_config = hybrid_config_;
    bounded_config.planning_timeout = std::min(
        bounded_config.planning_timeout, remaining);
    HybridAStarStatistics statistics;
    std::vector<geometry_msgs::PoseStamped> path;
    if (hybrid_planner_.makePlan(
            &snapshot, footprint, transition.start, planning_goal,
            bounded_config, profile, path, statistics, reason))
    {
      result.outcome =
          autolabor_coverage::HybridTransitionResult::OUTCOME_SUCCESS;
      result.path.poses = std::move(path);
      if (profile.accept_goal_region && !result.path.poses.empty())
        result.planned_goal = result.path.poses.back();
      result.estimated_time_sec = statistics.estimated_time;
      result.reverse_distance_m = statistics.reverse_distance;
      result.direction_changes = statistics.direction_changes;
      ++successes;
    }
    else
    {
      const bool resource_limit =
          reason.find("timeout") != std::string::npos ||
          reason.find("expansion limit") != std::string::npos;
      const bool invalid = reason.find("invalid") != std::string::npos ||
          reason.find("outside") != std::string::npos ||
          reason.find("occupied") != std::string::npos ||
          reason.find("unknown") != std::string::npos ||
          reason.find("footprint") != std::string::npos;
      result.outcome = resource_limit
          ? autolabor_coverage::HybridTransitionResult::OUTCOME_TIMEOUT
          : (invalid
                 ? autolabor_coverage::HybridTransitionResult::OUTCOME_INVALID
                 : autolabor_coverage::HybridTransitionResult::OUTCOME_NO_PATH);
    }
    result.expansions = statistics.expansions;
    result.reason = reason;
    response.results.push_back(std::move(result));
  }
  response.success = true;
  response.message = "precomputed " + std::to_string(successes) + " of " +
      std::to_string(request.transitions.size()) + " Hybrid transitions";
  return true;
}

bool CoverageGlobalPlanner::validateEnforcedPath(
    const autolabor_coverage::EnforcedPath& message, std::string& reason) const
{
  const std::string global_frame = costmap_ros_->getGlobalFrameID();
  if (message.header.frame_id != global_frame)
  {
    reason = "enforced-path header is not in the global costmap frame";
    return false;
  }
  if (message.planner_mode >
      autolabor_coverage::EnforcedPath::MODE_ENFORCED_SWEEP)
  {
    reason = "unknown coverage planner mode";
    return false;
  }
  const bool hybrid = message.planner_mode ==
      autolabor_coverage::EnforcedPath::MODE_HYBRID_TRANSIT;
  const bool sweep = message.planner_mode ==
      autolabor_coverage::EnforcedPath::MODE_ENFORCED_SWEEP;
  const bool path_required = hybrid || sweep;
  const bool path_supplied = !message.path.poses.empty();
  if ((path_required || path_supplied) &&
      (message.path.header.frame_id != global_frame ||
       message.path.poses.size() < 2))
  {
    reason = "planner path is empty or uses the wrong frame";
    return false;
  }
  if (message.active != sweep)
  {
    reason = "enforced-path flag does not match the explicit planner mode";
    return false;
  }
  if ((hybrid && message.expected_gear !=
                     autolabor_coverage::EnforcedPath::GEAR_FORWARD &&
                 message.expected_gear !=
                     autolabor_coverage::EnforcedPath::GEAR_REVERSE) ||
      (!hybrid && message.expected_gear !=
                      autolabor_coverage::EnforcedPath::GEAR_AUTO))
  {
    reason = "planner mode and expected fixed gear are inconsistent";
    return false;
  }
  return true;
}

bool CoverageGlobalPlanner::updateEnforcedPath(
    const autolabor_coverage::EnforcedPath& message, std::string& reason)
{
  if (!validateEnforcedPath(message, reason))
    return false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // The topic is only a freshness channel.  Cross-transport scheduling must
    // not allow a delayed refresh from the previous segment to undo the mode
    // selected by the synchronous service transaction.
    if (message.plan_id != enforced_path_.plan_id ||
        message.segment_index != enforced_path_.segment_index ||
        message.path_generation != enforced_path_.path_generation ||
        message.planner_mode != enforced_path_.planner_mode ||
        message.expected_gear != enforced_path_.expected_gear ||
        message.active != enforced_path_.active)
    {
      reason = "refresh does not match the synchronously armed coverage segment";
      return false;
    }
    // A refresh only renews the lease.  In Hybrid mode the plugin may have
    // replaced the manager-supplied connector or rolling chunk after a
    // persistent live blockage; a delayed topic copy must not roll it back.
    enforced_path_.header.stamp = message.header.stamp;
    enforced_path_.path.header.stamp = message.path.header.stamp;
    enforced_path_received_ = ros::WallTime::now();
  }
  reason = "coverage planner-mode lease refreshed";
  return true;
}

bool CoverageGlobalPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_)
    return false;
  autolabor_coverage::EnforcedPath path;
  HybridAStarProfile transit_profile;
  double hybrid_replan_period = 1.0;
  bool active = false;
  double age = std::numeric_limits<double>::infinity();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active = coverage_active_;
    path = enforced_path_;
    transit_profile = transit_profile_;
    hybrid_replan_period = hybrid_replan_period_;
    if (!enforced_path_received_.isZero())
      age = (ros::WallTime::now() - enforced_path_received_).toSec();
  }
  if (!active)
  {
    if (path.planner_mode !=
        autolabor_coverage::EnforcedPath::MODE_ORDINARY_NAVFN)
    {
      ROS_ERROR_THROTTLE(
          1.0, "ordinary navigation ownership has a non-Navfn planner mode");
      return false;
    }
    ROS_DEBUG_THROTTLE(2.0, "ordinary navigation is using Navfn");
    return fallback_.makePlan(start, goal, plan);
  }
  if (age > path_timeout_)
  {
    if (path.planner_mode ==
        autolabor_coverage::EnforcedPath::MODE_HYBRID_TRANSIT)
      publishHybridPathSafety(false);
    ROS_ERROR_THROTTLE(
        1.0, "coverage planner hand-off is stale; refusing any planner fallback");
    return false;
  }
  switch (path.planner_mode)
  {
    case autolabor_coverage::EnforcedPath::MODE_COVERAGE_NAVFN:
      ROS_DEBUG_THROTTLE(
          2.0, "coverage entry/inter-region navigation is using Navfn + TEB");
      return fallback_.makePlan(start, goal, plan);
    case autolabor_coverage::EnforcedPath::MODE_HYBRID_TRANSIT:
      return makeHybridPlan(start, goal, path, transit_profile,
                            hybrid_replan_period, plan);
    case autolabor_coverage::EnforcedPath::MODE_ENFORCED_SWEEP:
      return makeEnforcedPlan(start, goal, path, plan);
    default:
      ROS_ERROR_THROTTLE(
          1.0, "coverage ownership has an invalid or ordinary planner mode");
      return false;
  }
}

bool CoverageGlobalPlanner::makeHybridPlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    const autolabor_coverage::EnforcedPath& message,
    const HybridAStarProfile& transit_profile,
    double replan_period,
    std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!validReplanPeriod(replan_period))
  {
    ROS_ERROR_THROTTLE(1.0, "coverage Hybrid A* replan period is invalid");
    return false;
  }
  const ros::WallTime now = ros::WallTime::now();
  std::vector<geometry_msgs::PoseStamped> cached_plan;
  const bool cache_valid = !hybrid_replan_every_cycle_ &&
      message.path.poses.size() >= 2 &&
      makeEnforcedPlan(start, goal, message, cached_plan,
                       hybrid_cache_max_deviation_,
                       hybrid_cache_collision_check_horizon_);

  // In the production event-triggered mode, the manager-supplied connector
  // is the single path authority.  This plugin validates it at move_base's
  // global-planner rate and publishes a fail-closed permit for the TEB command
  // mux.  It must not silently create a different path that only TEB can see
  // while the mux is still validating the old EnforcedPath generation.
  if (!hybrid_replan_every_cycle_)
  {
    publishHybridPathSafety(cache_valid);
    if (cache_valid)
    {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        hybrid_last_search_failed_ = false;
      }
      plan = std::move(cached_plan);
      ROS_DEBUG_THROTTLE(
          2.0, "coverage Hybrid A* reused its validated transition cache");
      return true;
    }
    ROS_WARN_THROTTLE(
        1.0,
        "Hybrid transition cache is unsafe or deviated; holding the "
        "TEB command mux at zero until the coverage manager replans");
    return false;
  }

  // The unconditional-search mode is retained only for isolated comparison.
  // It cannot authorize the production TEB command mux because its newly
  // generated path is not the manager's EnforcedPath.
  publishHybridPathSafety(false);

  bool failed_search_cooling_down = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double elapsed = hybrid_last_search_.isZero()
        ? std::numeric_limits<double>::infinity()
        : (now - hybrid_last_search_).toSec();
    failed_search_cooling_down =
        hybrid_last_search_failed_ && elapsed < replan_period;
  }
  // The manager-supplied Hybrid connector or rolling chunk is the active
  // kinematic reference.  Replacing a still-safe path periodically can flip
  // between forward/reverse or different homotopy classes while TEB tracks it.
  // Replan only when the immediate commitment horizon is blocked or the
  // vehicle has materially departed from the connector.  TEB already checks
  // its complete local horizon at 10 Hz; rejecting the whole remaining cache
  // here would make a distant live obstacle stop the vehicle before TEB has a
  // chance to leave the reference path.  The Qt period is therefore a retry
  // cooldown after an unsuccessful recovery search, not a timer that
  // invalidates a healthy path.
  if (failed_search_cooling_down)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "Hybrid transition search failed; online search will retry "
        "after the %.1fs configured period",
        replan_period);
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (enforced_path_.plan_id != message.plan_id ||
        enforced_path_.segment_index != message.segment_index ||
        enforced_path_.planner_mode != message.planner_mode)
      return false;
    hybrid_last_search_ = now;
  }

  // Keep a blocked/deviation recovery search shorter than its retry period.
  // At the 1 s setting this bounds the planner thread to 0.8 s and
  // proportionally cuts the expansion budget; longer settings retain the
  // normal 1.5 s / 80000 ceiling.
  HybridAStarConfig online_config = hybrid_config_;
  online_config.planning_timeout = std::min(
      hybrid_config_.planning_timeout, std::max(0.20, 0.80 * replan_period));
  if (hybrid_config_.planning_timeout > 0.0)
  {
    const double fraction = std::min(
        1.0, online_config.planning_timeout / hybrid_config_.planning_timeout);
    online_config.max_expansions = std::min(
        hybrid_config_.max_expansions,
        std::max(
            10000, static_cast<int>(std::ceil(
                hybrid_config_.max_expansions * fraction))));
  }

  costmap_2d::Costmap2D snapshot = snapshotCostmap(costmap_ros_);
  HybridAStarStatistics statistics;
  std::string reason;
  std::vector<geometry_msgs::PoseStamped> replanned;
  geometry_msgs::PoseStamped kinematic_goal = goal;
  std::vector<geometry_msgs::PoseStamped> topology_plan;
  std::size_t topology_join_index = 0;
  bool rolling_horizon = false;
  if (hybrid_replan_every_cycle_)
  {
    const double direct_distance = std::hypot(
        goal.pose.position.x - start.pose.position.x,
        goal.pose.position.y - start.pose.position.y);
    if (direct_distance > hybrid_online_kinematic_horizon_)
    {
      if (!fallback_.makePlan(start, goal, topology_plan) ||
          topology_plan.size() < 2)
      {
        reason = "online topology guide has no known-free path";
      }
      else
      {
        double distance = 0.0;
        topology_join_index = topology_plan.size() - 1;
        for (std::size_t index = 1; index < topology_plan.size(); ++index)
        {
          distance += std::hypot(
              topology_plan[index].pose.position.x -
                  topology_plan[index - 1].pose.position.x,
              topology_plan[index].pose.position.y -
                  topology_plan[index - 1].pose.position.y);
          if (distance >= hybrid_online_kinematic_horizon_)
          {
            topology_join_index = index;
            break;
          }
        }
        if (topology_join_index + 1 < topology_plan.size())
        {
          rolling_horizon = true;
          kinematic_goal = topology_plan[topology_join_index];
          const geometry_msgs::PoseStamped& next =
              topology_plan[topology_join_index + 1];
          const double yaw = std::atan2(
              next.pose.position.y - kinematic_goal.pose.position.y,
              next.pose.position.x - kinematic_goal.pose.position.x);
          kinematic_goal.pose.orientation.x = 0.0;
          kinematic_goal.pose.orientation.y = 0.0;
          kinematic_goal.pose.orientation.z = std::sin(0.5 * yaw);
          kinematic_goal.pose.orientation.w = std::cos(0.5 * yaw);
        }
      }
    }
  }
  const bool topology_failed = hybrid_replan_every_cycle_ &&
      !reason.empty();
  if (topology_failed || !hybrid_planner_.makePlan(
          &snapshot, costmap_ros_->getRobotFootprint(), start, kinematic_goal,
          online_config, transit_profile, replanned, statistics, reason))
  {
    bool still_current = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      still_current = enforced_path_.plan_id == message.plan_id &&
          enforced_path_.segment_index == message.segment_index &&
          enforced_path_.planner_mode == message.planner_mode;
      if (still_current)
        hybrid_last_search_failed_ = true;
    }
    if (!still_current)
      return false;
    ROS_WARN_THROTTLE(
        1.0, "coverage Hybrid A* blocked-path replan failed: %s",
        reason.c_str());
    return false;
  }
  const std::size_t kinematic_pose_count = replanned.size();
  if (rolling_horizon)
  {
    // Keep the true action goal as the plan endpoint so move_base cannot
    // report success at the rolling waypoint.  TEB only consumes its bounded
    // lookahead, which remains inside the collision-checked Hybrid prefix;
    // the static Navfn suffix supplies topology and is regenerated next tick.
    replanned.insert(
        replanned.end(),
        topology_plan.begin() + static_cast<std::ptrdiff_t>(
            topology_join_index + 1),
        topology_plan.end());
  }
  bool still_current = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    still_current = enforced_path_.plan_id == message.plan_id &&
        enforced_path_.segment_index == message.segment_index &&
        enforced_path_.planner_mode == message.planner_mode;
    if (still_current)
    {
      enforced_path_.path.header.frame_id = start.header.frame_id;
      enforced_path_.path.header.stamp = ros::Time::now();
      enforced_path_.path.poses = replanned;
      hybrid_last_search_failed_ = false;
    }
  }
  if (!still_current)
    return false;
  plan = std::move(replanned);
  ROS_INFO(
      "coverage Hybrid A* %s replan (%.1fs retry cooldown): "
      "poses=%zu kinematic_prefix=%zu rolling=%s expansions=%zu "
      "cost=%.2fs reverse=%.2fm switches=%u analytic=%s",
      hybrid_replan_every_cycle_ ? "1 Hz online" : "blocked/deviation",
      replan_period,
      plan.size(), kinematic_pose_count, rolling_horizon ? "yes" : "no",
      statistics.expansions, statistics.estimated_time,
      statistics.reverse_distance, statistics.direction_changes,
      statistics.used_analytic_expansion ? "yes" : "no");
  return true;
}

bool CoverageGlobalPlanner::makeEnforcedPlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    const autolabor_coverage::EnforcedPath& message,
    std::vector<geometry_msgs::PoseStamped>& plan,
    double maximum_start_deviation,
    double maximum_lethal_check_distance)
{
  const auto& poses = message.path.poses;
  if (poses.size() < 2 || message.path.header.frame_id.empty() ||
      start.header.frame_id != message.path.header.frame_id ||
      goal.header.frame_id != message.path.header.frame_id)
    return false;
  const geometry_msgs::Point& expected = poses.back().pose.position;
  const double goal_error = std::hypot(goal.pose.position.x - expected.x,
                                       goal.pose.position.y - expected.y);
  if (!std::isfinite(goal_error) || goal_error > goal_match_tolerance_)
  {
    ROS_ERROR_THROTTLE(1.0, "coverage goal does not match enforced path endpoint");
    return false;
  }
  double expected_yaw = 0.0;
  double goal_yaw = 0.0;
  if (!normalizedYaw(poses.back().pose.orientation, expected_yaw) ||
      !normalizedYaw(goal.pose.orientation, goal_yaw) ||
      yawError(expected_yaw, goal_yaw) > goal_yaw_match_tolerance_)
  {
    ROS_ERROR_THROTTLE(1.0,
                       "coverage goal yaw does not match enforced path endpoint");
    return false;
  }
  const auto& footprint = costmap_ros_->getRobotFootprint();
  if (footprint.size() < 3)
  {
    ROS_ERROR_THROTTLE(1.0, "coverage costmap footprint is unavailable");
    return false;
  }
  double inscribed_radius = 0.0;
  double circumscribed_radius = 0.0;
  costmap_2d::calculateMinAndMaxDistances(
      footprint, inscribed_radius, circumscribed_radius);
  // Validate every pose against one costmap generation.  Reading the live
  // layered map pose-by-pose could otherwise combine cells from two updates
  // and spuriously accept or reject a cached connector.
  costmap_2d::Costmap2D snapshot = snapshotCostmap(costmap_ros_);
  base_local_planner::CostmapModel world_model(snapshot);
  std::size_t nearest = 0;
  double nearest_squared = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < poses.size(); ++index)
  {
    const auto& point = poses[index].pose.position;
    const double dx = start.pose.position.x - point.x;
    const double dy = start.pose.position.y - point.y;
    const double squared = dx * dx + dy * dy;
    if (squared < nearest_squared)
    {
      nearest_squared = squared;
      nearest = index;
    }
  }
  if (!std::isfinite(nearest_squared) ||
      std::sqrt(nearest_squared) > maximum_start_deviation)
  {
    ROS_WARN_THROTTLE(
        1.0, "vehicle deviated %.2fm from the cached Hybrid transition",
        std::sqrt(nearest_squared));
    return false;
  }
  double start_yaw = 0.0;
  if (!normalizedYaw(start.pose.orientation, start_yaw))
    return false;
  unsigned int start_mx = 0;
  unsigned int start_my = 0;
  if (!snapshot.worldToMap(start.pose.position.x, start.pose.position.y,
                           start_mx, start_my) ||
      snapshot.getCost(start_mx, start_my) == costmap_2d::NO_INFORMATION ||
      world_model.footprintCost(
          start.pose.position.x, start.pose.position.y, start_yaw, footprint,
          inscribed_radius, circumscribed_radius) < 0.0)
  {
    ROS_WARN_THROTTLE(
        1.0, "coverage vehicle footprint is not in currently known free space");
    return false;
  }
  plan.clear();
  geometry_msgs::PoseStamped normalized_start = start;
  normalized_start.header.frame_id = message.path.header.frame_id;
  plan.push_back(normalized_start);
  const ros::Time stamp = ros::Time::now();
  double distance_from_start = 0.0;
  double previous_x = start.pose.position.x;
  double previous_y = start.pose.position.y;
  for (std::size_t index = nearest; index < poses.size(); ++index)
  {
    geometry_msgs::PoseStamped pose = poses[index];
    pose.header.stamp = stamp;
    if (!std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y))
      return false;
    double pose_yaw = 0.0;
    if (!normalizedYaw(pose.pose.orientation, pose_yaw))
      return false;
    distance_from_start += std::hypot(
        pose.pose.position.x - previous_x,
        pose.pose.position.y - previous_y);
    previous_x = pose.pose.position.x;
    previous_y = pose.pose.position.y;
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!snapshot.worldToMap(pose.pose.position.x,
                             pose.pose.position.y, mx, my))
      return false;
    const unsigned char cost = snapshot.getCost(mx, my);
    if (cost == costmap_2d::NO_INFORMATION)
      return false;
    const double footprint_cost = world_model.footprintCost(
        pose.pose.position.x, pose.pose.position.y, pose_yaw, footprint,
        inscribed_radius, circumscribed_radius);
    // Unknown space and map-boundary crossings remain forbidden over the
    // entire connector.  A live lethal obstacle only invalidates the stable
    // cache once it enters the near commitment horizon; until then the local
    // controller is allowed to form a collision-free homotopy around it.
    if (footprint_cost == -2.0 || footprint_cost == -3.0)
      return false;
    const bool check_lethal =
        !std::isfinite(maximum_lethal_check_distance) ||
        distance_from_start <= maximum_lethal_check_distance;
    if (check_lethal &&
        (cost == costmap_2d::LETHAL_OBSTACLE || footprint_cost == -1.0))
    {
      if (std::isfinite(maximum_lethal_check_distance))
      {
        ROS_WARN_THROTTLE(
            1.0,
            "coverage path is blocked inside the %.2fm commitment horizon",
            maximum_lethal_check_distance);
      }
      else
      {
        ROS_WARN_THROTTLE(1.0, "coverage enforced path is blocked");
      }
      return false;
    }
    // The live start pose already replaces the nearest cache anchor.  Sending
    // that anchor again can make TEB backtrack to a cusp that the previous
    // zero-velocity action has just accepted.  Validate it above, but continue
    // toward the next signed-motion sample whenever one exists.
    if (index == nearest && nearest + 1 < poses.size())
      continue;
    plan.push_back(std::move(pose));
  }
  return plan.size() >= 2;
}

}  // namespace autolabor_coverage

PLUGINLIB_EXPORT_CLASS(autolabor_coverage::CoverageGlobalPlanner,
                       nav_core::BaseGlobalPlanner)
