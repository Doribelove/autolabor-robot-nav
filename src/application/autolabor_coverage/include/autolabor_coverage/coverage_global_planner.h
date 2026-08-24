#ifndef AUTOLABOR_COVERAGE_COVERAGE_GLOBAL_PLANNER_H
#define AUTOLABOR_COVERAGE_COVERAGE_GLOBAL_PLANNER_H

#include <autolabor_coverage/EnforcedPath.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <navfn/navfn_ros.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <mutex>
#include <string>
#include <vector>

namespace autolabor_coverage
{

class CoverageGlobalPlanner : public nav_core::BaseGlobalPlanner
{
public:
  CoverageGlobalPlanner() = default;
  CoverageGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override;

private:
  void activeCallback(const std_msgs::Bool::ConstPtr& message);
  void pathCallback(const autolabor_coverage::EnforcedPath::ConstPtr& message);
  bool makeEnforcedPlan(const geometry_msgs::PoseStamped& start,
                        const geometry_msgs::PoseStamped& goal,
                        std::vector<geometry_msgs::PoseStamped>& plan);

  bool initialized_ = false;
  bool coverage_active_ = false;
  double goal_match_tolerance_ = 0.35;
  double goal_yaw_match_tolerance_ = 0.20;
  double path_timeout_ = 1.0;
  ros::NodeHandle private_nh_;
  ros::Subscriber active_subscriber_;
  ros::Subscriber path_subscriber_;
  navfn::NavfnROS fallback_;
  costmap_2d::Costmap2DROS* costmap_ros_ = nullptr;
  autolabor_coverage::EnforcedPath enforced_path_;
  ros::WallTime enforced_path_received_;
  std::mutex mutex_;
};

}  // namespace autolabor_coverage

#endif
