#include <autolabor_coverage/hybrid_a_star.h>

#include <costmap_2d/cost_values.h>
#include <gtest/gtest.h>
#include <ros/time.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace
{

constexpr double kPi = 3.14159265358979323846;

geometry_msgs::PoseStamped pose(double x, double y, double yaw)
{
  geometry_msgs::PoseStamped result;
  result.header.frame_id = "map";
  result.pose.position.x = x;
  result.pose.position.y = y;
  result.pose.orientation.z = std::sin(0.5 * yaw);
  result.pose.orientation.w = std::cos(0.5 * yaw);
  return result;
}

double yaw(const geometry_msgs::PoseStamped& value)
{
  return std::atan2(
      2.0 * value.pose.orientation.w * value.pose.orientation.z,
      1.0 - 2.0 * value.pose.orientation.z * value.pose.orientation.z);
}

std::vector<geometry_msgs::Point> footprint()
{
  std::vector<geometry_msgs::Point> result(4);
  result[0].x = 0.30;
  result[0].y = 0.20;
  result[1].x = 0.30;
  result[1].y = -0.20;
  result[2].x = -0.30;
  result[2].y = -0.20;
  result[3].x = -0.30;
  result[3].y = 0.20;
  return result;
}

autolabor_coverage::HybridAStarConfig testConfig()
{
  autolabor_coverage::HybridAStarConfig config;
  config.minimum_turning_radius = 1.35;
  config.motion_step = 0.25;
  config.collision_check_step = 0.05;
  config.state_resolution = 0.125;
  config.heading_bins = 72;
  config.steering_samples = 5;
  config.max_expansions = 60000;
  config.planning_timeout = 2.0;
  return config;
}

autolabor_coverage::HybridAStarProfile testProfile()
{
  autolabor_coverage::HybridAStarProfile profile;
  profile.goal_position_tolerance = 0.22;
  profile.goal_yaw_tolerance = 0.20;
  return profile;
}

}  // namespace

TEST(HybridAStarPlanner, ProducesCurvatureBoundedForwardTurn)
{
  costmap_2d::Costmap2D costmap(
      160, 160, 0.10, -8.0, -8.0, costmap_2d::FREE_SPACE);
  autolabor_coverage::HybridAStarPlanner planner;
  autolabor_coverage::HybridAStarConfig config = testConfig();
  autolabor_coverage::HybridAStarProfile profile = testProfile();
  profile.allow_reverse = false;
  std::vector<geometry_msgs::PoseStamped> plan;
  autolabor_coverage::HybridAStarStatistics statistics;
  std::string reason;

  ASSERT_TRUE(planner.makePlan(
      &costmap, footprint(), pose(-2.0, -2.0, 0.0),
      pose(0.0, 0.0, 0.5 * kPi), config, profile,
      plan, statistics, reason)) << reason;
  ASSERT_GE(plan.size(), 3u);
  EXPECT_GT(statistics.expansions, 0u);
  EXPECT_DOUBLE_EQ(0.0, statistics.reverse_distance);
  // The final exact action goal is appended after the accepted lattice state;
  // check the actual motion primitives, not that tolerance-only final marker.
  for (std::size_t index = 1; index + 1 < plan.size(); ++index)
  {
    const double distance = std::hypot(
        plan[index].pose.position.x - plan[index - 1].pose.position.x,
        plan[index].pose.position.y - plan[index - 1].pose.position.y);
    const double heading = std::abs(std::atan2(
        std::sin(yaw(plan[index]) - yaw(plan[index - 1])),
        std::cos(yaw(plan[index]) - yaw(plan[index - 1]))));
    EXPECT_LE(heading, distance / config.minimum_turning_radius + 1.0e-3);
  }
}

TEST(HybridAStarPlanner, UsesReverseInsideNarrowCorridor)
{
  costmap_2d::Costmap2D costmap(
      100, 60, 0.10, -5.0, -3.0, costmap_2d::LETHAL_OBSTACLE);
  for (unsigned int y = 25; y <= 35; ++y)
  {
    for (unsigned int x = 20; x <= 65; ++x)
      costmap.setCost(x, y, costmap_2d::FREE_SPACE);
  }
  autolabor_coverage::HybridAStarPlanner planner;
  autolabor_coverage::HybridAStarConfig config = testConfig();
  config.max_expansions = 10000;
  config.planning_timeout = 0.5;
  autolabor_coverage::HybridAStarProfile profile = testProfile();
  profile.allow_reverse = true;
  std::vector<geometry_msgs::PoseStamped> plan;
  autolabor_coverage::HybridAStarStatistics statistics;
  std::string reason;

  ASSERT_TRUE(planner.makePlan(
      &costmap, footprint(), pose(0.0, 0.0, 0.0),
      pose(-1.50, 0.0, 0.0), config, profile,
      plan, statistics, reason)) << reason;
  EXPECT_GT(statistics.reverse_distance, 1.0);
  EXPECT_EQ(0u, statistics.direction_changes);

  profile.allow_reverse = false;
  plan.clear();
  EXPECT_FALSE(planner.makePlan(
      &costmap, footprint(), pose(0.0, 0.0, 0.0),
      pose(-1.50, 0.0, 0.0), config, profile,
      plan, statistics, reason));
}

TEST(HybridAStarPlanner, CorrectsCloseEntryHeadingWithoutInPlaceRotation)
{
  costmap_2d::Costmap2D costmap(
      160, 160, 0.10, -8.0, -8.0, costmap_2d::FREE_SPACE);
  autolabor_coverage::HybridAStarPlanner planner;
  autolabor_coverage::HybridAStarConfig config = testConfig();
  config.max_expansions = 100000;
  config.planning_timeout = 2.0;
  autolabor_coverage::HybridAStarProfile profile = testProfile();
  profile.allow_reverse = true;
  std::vector<geometry_msgs::PoseStamped> plan;
  autolabor_coverage::HybridAStarStatistics statistics;
  std::string reason;

  ASSERT_TRUE(planner.makePlan(
      &costmap, footprint(), pose(0.0, 0.0, 0.0),
      pose(0.0, 0.0, 1.0), config, profile,
      plan, statistics, reason)) << reason;
  ASSERT_GE(plan.size(), 3u);
  double maximum_excursion = 0.0;
  for (const auto& value : plan)
  {
    maximum_excursion = std::max(
        maximum_excursion,
        std::hypot(value.pose.position.x, value.pose.position.y));
  }
  EXPECT_GT(maximum_excursion, profile.goal_position_tolerance);
}

TEST(HybridAStarPlanner, RejectsUnknownGoalFootprint)
{
  costmap_2d::Costmap2D costmap(
      100, 100, 0.10, -5.0, -5.0, costmap_2d::FREE_SPACE);
  unsigned int goal_x = 0;
  unsigned int goal_y = 0;
  ASSERT_TRUE(costmap.worldToMap(2.0, 0.0, goal_x, goal_y));
  costmap.setCost(goal_x, goal_y, costmap_2d::NO_INFORMATION);
  autolabor_coverage::HybridAStarPlanner planner;
  std::vector<geometry_msgs::PoseStamped> plan;
  autolabor_coverage::HybridAStarStatistics statistics;
  std::string reason;

  EXPECT_FALSE(planner.makePlan(
      &costmap, footprint(), pose(0.0, 0.0, 0.0),
      pose(2.0, 0.0, 0.0), testConfig(), testProfile(),
      plan, statistics, reason));
  EXPECT_NE(std::string::npos, reason.find("goal footprint"));
}

int main(int argc, char** argv)
{
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
