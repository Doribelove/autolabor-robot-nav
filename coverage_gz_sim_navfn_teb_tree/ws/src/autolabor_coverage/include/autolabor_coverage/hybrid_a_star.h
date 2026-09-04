#ifndef AUTOLABOR_COVERAGE_HYBRID_A_STAR_H
#define AUTOLABOR_COVERAGE_HYBRID_A_STAR_H

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

#include <cstddef>
#include <string>
#include <vector>

namespace autolabor_coverage
{

struct HybridAStarConfig
{
  double minimum_turning_radius = 1.35;
  double motion_step = 0.30;
  double collision_check_step = 0.10;
  double state_resolution = 0.15;
  int heading_bins = 72;
  int steering_samples = 5;
  int max_expansions = 80000;
  double planning_timeout = 1.50;
  double heuristic_weight = 1.05;
  double steering_penalty = 0.04;
  double steering_change_penalty = 0.10;
  double obstacle_cost_scale = 0.25;
  bool use_nonholonomic_heuristic = true;
  bool use_obstacle_heuristic = true;
  bool use_analytic_expansion = true;
  double analytic_improvement_timeout = 0.20;
  double analytic_gearchange_improvement_timeout = 0.65;
  double analytic_connector_improvement_timeout = 0.50;
  int analytic_expansion_interval = 200;
};

struct HybridAStarProfile
{
  bool allow_reverse = true;
  double max_forward_speed = 0.80;
  double max_reverse_speed = 0.30;
  double max_angular_speed = 0.60;
  double linear_acceleration = 1.00;
  double angular_acceleration = 0.50;
  double direction_change_penalty = 0.50;
  double goal_position_tolerance = 0.30;
  double goal_yaw_tolerance = 0.40;
  bool accept_goal_region = false;
};

struct HybridAStarStatistics
{
  std::size_t expansions = 0;
  double estimated_time = 0.0;
  double reverse_distance = 0.0;
  unsigned int direction_changes = 0;
  bool used_analytic_expansion = false;
};

class HybridAStarPlanner
{
public:
  bool makePlan(costmap_2d::Costmap2D* costmap,
                const std::vector<geometry_msgs::Point>& footprint,
                const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                const HybridAStarConfig& config,
                const HybridAStarProfile& profile,
                std::vector<geometry_msgs::PoseStamped>& plan,
                HybridAStarStatistics& statistics,
                std::string& reason) const;
};

}  // namespace autolabor_coverage

#endif
