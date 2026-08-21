#include "autolabor_coverage/coverage_global_planner.h"

#include <costmap_2d/cost_values.h>
#include <pluginlib/class_list_macros.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace autolabor_coverage
{

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
  private_nh_.param("path_timeout", path_timeout_, 1.0);
  fallback_.initialize(name + "_navfn", costmap_ros);
  active_subscriber_ = private_nh_.subscribe(
      "/coverage/active", 1, &CoverageGlobalPlanner::activeCallback, this);
  path_subscriber_ = private_nh_.subscribe(
      "/coverage/enforced_path", 1, &CoverageGlobalPlanner::pathCallback, this);
  initialized_ = true;
}

void CoverageGlobalPlanner::activeCallback(const std_msgs::Bool::ConstPtr& message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  coverage_active_ = message->data;
}

void CoverageGlobalPlanner::pathCallback(
    const autolabor_coverage::EnforcedPath::ConstPtr& message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  enforced_path_ = *message;
  enforced_path_received_ = ros::WallTime::now();
}

bool CoverageGlobalPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_)
    return false;
  autolabor_coverage::EnforcedPath path;
  bool active = false;
  double age = std::numeric_limits<double>::infinity();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active = coverage_active_;
    path = enforced_path_;
    if (!enforced_path_received_.isZero())
      age = (ros::WallTime::now() - enforced_path_received_).toSec();
  }
  if (!path.active)
    return fallback_.makePlan(start, goal, plan);
  if (age > path_timeout_)
  {
    if (active)
    {
      ROS_ERROR_THROTTLE(1.0, "coverage enforced path is stale; refusing fallback shortcut");
      return false;
    }
    return fallback_.makePlan(start, goal, plan);
  }
  return makeEnforcedPlan(start, goal, plan);
}

bool CoverageGlobalPlanner::makeEnforcedPlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan)
{
  autolabor_coverage::EnforcedPath message;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    message = enforced_path_;
  }
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
  plan.clear();
  geometry_msgs::PoseStamped normalized_start = start;
  normalized_start.header.frame_id = message.path.header.frame_id;
  plan.push_back(normalized_start);
  const ros::Time stamp = ros::Time::now();
  for (std::size_t index = nearest; index < poses.size(); ++index)
  {
    geometry_msgs::PoseStamped pose = poses[index];
    pose.header.stamp = stamp;
    if (!std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y))
      return false;
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap_ros_->getCostmap()->worldToMap(pose.pose.position.x,
                                                 pose.pose.position.y, mx, my))
      return false;
    const unsigned char cost = costmap_ros_->getCostmap()->getCost(mx, my);
    if (cost == costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::NO_INFORMATION)
      return false;
    plan.push_back(std::move(pose));
  }
  return plan.size() >= 2;
}

}  // namespace autolabor_coverage

PLUGINLIB_EXPORT_CLASS(autolabor_coverage::CoverageGlobalPlanner,
                       nav_core::BaseGlobalPlanner)
