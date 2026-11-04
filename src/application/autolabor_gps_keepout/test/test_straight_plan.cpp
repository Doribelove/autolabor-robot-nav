#include <autolabor_gps_keepout/straight_plan.h>
#include <costmap_2d/cost_values.h>
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

using autolabor_gps_keepout::makeFreeSpaceStraightPlan;

class StraightPlanTest : public testing::Test {
 protected:
  costmap_2d::Costmap2D map{400, 400, .1, -20., -20., costmap_2d::FREE_SPACE};
  geometry_msgs::PoseStamped start, goal;
  std::vector<geometry_msgs::PoseStamped> plan;
  // Existing 1.6 x 0.8 m footprint, with the current 0.1 m padding.
  const double radius = std::hypot(.9, .5);
  void SetUp() override {
    start.header.frame_id = goal.header.frame_id = "camera_init";
    start.pose.orientation.w = goal.pose.orientation.w = 1.;
    goal.pose.position.x = 14.4;
    goal.pose.position.y = 4.2;
  }
  void put(double x, double y, unsigned char cost) {
    unsigned int mx, my;
    ASSERT_TRUE(map.worldToMap(x, y, mx, my));
    map.setCost(mx, my, cost);
  }
};

TEST_F(StraightPlanTest, FreeDiagonalIsStraightAndRetainsExactEndpoints) {
  start.pose.orientation.z = .2;
  start.pose.orientation.w = std::sqrt(.96);
  ASSERT_TRUE(makeFreeSpaceStraightPlan(map, radius, start, goal, plan));
  ASSERT_GT(plan.size(), 2u);
  EXPECT_EQ(plan.front(), start);
  EXPECT_EQ(plan.back(), goal);
  for (size_t i = 0; i < plan.size(); ++i) {
    const auto& p = plan[i].pose.position;
    EXPECT_NEAR(p.x * 4.2 - p.y * 14.4, 0., 1e-10);
    if (i) EXPECT_LE(std::hypot(p.x-plan[i-1].pose.position.x,
                               p.y-plan[i-1].pose.position.y), .05+1e-9);
  }
}

TEST_F(StraightPlanTest, ObstacleUnknownAndInflationAllUseOriginalPlanner) {
  for (const auto cost : {costmap_2d::LETHAL_OBSTACLE, costmap_2d::NO_INFORMATION,
                          costmap_2d::INSCRIBED_INFLATED_OBSTACLE,
                          static_cast<unsigned char>(1)}) {
    put(7.2, 2.1, cost);
    EXPECT_FALSE(makeFreeSpaceStraightPlan(map, radius, start, goal, plan));
    EXPECT_TRUE(plan.empty());
  }
}

TEST_F(StraightPlanTest, SideObstacleInsideVehicleEnvelopePreventsShortcut) {
  goal.pose.position.y = 0.;
  put(7., .85, costmap_2d::LETHAL_OBSTACLE);
  EXPECT_FALSE(makeFreeSpaceStraightPlan(map, radius, start, goal, plan));
}

TEST_F(StraightPlanTest, OutsideCorridorObstacleDoesNotBendFreeReference) {
  goal.pose.position.y = 0.;
  put(7., 2., costmap_2d::LETHAL_OBSTACLE);
  EXPECT_TRUE(makeFreeSpaceStraightPlan(map, radius, start, goal, plan));
}

TEST_F(StraightPlanTest, MapEdgeAndUnknownStartDoNotGetCleared) {
  goal.pose.position.x = 19.5;
  EXPECT_FALSE(makeFreeSpaceStraightPlan(map, radius, start, goal, plan));
  goal.pose.position.x = 14.4;
  put(0., 0., costmap_2d::NO_INFORMATION);
  EXPECT_FALSE(makeFreeSpaceStraightPlan(map, radius, start, goal, plan));
  unsigned int x, y;
  ASSERT_TRUE(map.worldToMap(0., 0., x, y));
  EXPECT_EQ(map.getCost(x, y), costmap_2d::NO_INFORMATION);
}

TEST_F(StraightPlanTest, ReverseAndCoincidentGoalsRemainFinite) {
  goal.pose.position.x = -14.4;
  ASSERT_TRUE(makeFreeSpaceStraightPlan(map, radius, start, goal, plan));
  EXPECT_EQ(plan.back(), goal);
  goal = start;
  ASSERT_TRUE(makeFreeSpaceStraightPlan(map, radius, start, goal, plan));
  EXPECT_EQ(plan.back(), goal);
}

TEST_F(StraightPlanTest, InvalidCoordinatesOrFrameCannotCreateShortcut) {
  goal.header.frame_id = "another_frame";
  EXPECT_FALSE(makeFreeSpaceStraightPlan(map, radius, start, goal, plan));
  goal.header.frame_id = start.header.frame_id;
  goal.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(makeFreeSpaceStraightPlan(map, radius, start, goal, plan));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
