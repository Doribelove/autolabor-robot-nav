#ifndef AUTOLABOR_COVERAGE_COVERAGE_GLOBAL_PLANNER_H
#define AUTOLABOR_COVERAGE_COVERAGE_GLOBAL_PLANNER_H

#include <autolabor_coverage/EnforcedPath.h>
#include <autolabor_coverage/PrecomputeTransitions.h>
#include <autolabor_coverage/SetEnforcedPath.h>
#include <autolabor_coverage/TransitProfile.h>
#include <autolabor_coverage/hybrid_a_star.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <navfn/navfn_ros.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <limits>
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
  void pathCallback(const autolabor_coverage::EnforcedPath::ConstPtr& message);
  bool setPathCallback(autolabor_coverage::SetEnforcedPath::Request& request,
                       autolabor_coverage::SetEnforcedPath::Response& response);
  bool precomputeCallback(
      autolabor_coverage::PrecomputeTransitions::Request& request,
      autolabor_coverage::PrecomputeTransitions::Response& response);
  bool validateEnforcedPath(const autolabor_coverage::EnforcedPath& message,
                            std::string& reason) const;
  bool updateEnforcedPath(const autolabor_coverage::EnforcedPath& message,
                          std::string& reason);
  bool makeEnforcedPlan(const geometry_msgs::PoseStamped& start,
                        const geometry_msgs::PoseStamped& goal,
                        const autolabor_coverage::EnforcedPath& message,
                        std::vector<geometry_msgs::PoseStamped>& plan,
                        double maximum_start_deviation =
                            std::numeric_limits<double>::infinity(),
                        double maximum_lethal_check_distance =
                            std::numeric_limits<double>::infinity());
  bool makeHybridPlan(const geometry_msgs::PoseStamped& start,
                      const geometry_msgs::PoseStamped& goal,
                      const autolabor_coverage::EnforcedPath& message,
                      const HybridAStarProfile& transit_profile,
                      double replan_period,
                      std::vector<geometry_msgs::PoseStamped>& plan);
  void publishHybridPathSafety(bool safe);

  bool initialized_ = false;
  bool coverage_active_ = false;
  double goal_match_tolerance_ = 0.35;
  double goal_yaw_match_tolerance_ = 0.20;
  double path_timeout_ = 1.0;
  double hybrid_cache_max_deviation_ = 0.60;
  double hybrid_cache_collision_check_horizon_ = 3.00;
  double hybrid_online_kinematic_horizon_ = 12.0;
  double hybrid_replan_period_ = 1.0;
  bool hybrid_replan_every_cycle_ = false;
  ros::NodeHandle private_nh_;
  ros::Subscriber path_subscriber_;
  ros::ServiceServer set_path_service_;
  ros::ServiceServer precompute_service_;
  ros::Publisher hybrid_path_safe_publisher_;
  navfn::NavfnROS fallback_;
  HybridAStarPlanner hybrid_planner_;
  HybridAStarConfig hybrid_config_;
  HybridAStarProfile transit_profile_;
  costmap_2d::Costmap2DROS* costmap_ros_ = nullptr;
  autolabor_coverage::EnforcedPath enforced_path_;
  ros::WallTime enforced_path_received_;
  ros::WallTime hybrid_last_search_;
  bool hybrid_last_search_failed_ = false;
  std::mutex mutex_;
};

}  // namespace autolabor_coverage

#endif
