#include <cmath>
#include <vector>

#include <costmap_2d/cost_values.h>
#include <geometry_msgs/Point.h>
#include <gtest/gtest.h>

#include <robot_bringup/ackermann_recovery_geometry.h>
#include <robot_bringup/ackermann_recovery_interrupt_gate.h>

namespace
{

using robot_bringup::recovery_detail::ArcPose;
using robot_bringup::recovery_detail::footprintInteriorIsSafe;
using robot_bringup::recovery_detail::integrateArcStep;
using robot_bringup::recovery_detail::PendingInterruptGate;

geometry_msgs::Point point(double x, double y)
{
  geometry_msgs::Point result;
  result.x = x;
  result.y = y;
  return result;
}

std::vector<geometry_msgs::Point> centeredRectangle()
{
  return {point(-0.4, -0.3), point(0.4, -0.3),
          point(0.4, 0.3), point(-0.4, 0.3)};
}

void setWorldCost(costmap_2d::Costmap2D& costmap, double x, double y,
                  unsigned char cost)
{
  unsigned int mx = 0;
  unsigned int my = 0;
  ASSERT_TRUE(costmap.worldToMap(x, y, mx, my));
  costmap.setCost(mx, my, cost);
}

TEST(AckermannRecoveryGeometry, IntegratesForwardAndReverseArc)
{
  const double curvature = 0.5;
  const ArcPose start{0.0, 0.0, 0.0};

  const ArcPose forward = integrateArcStep(start, 1.0, curvature);
  EXPECT_NEAR(forward.x, std::sin(curvature) / curvature, 1e-12);
  EXPECT_NEAR(forward.y, (1.0 - std::cos(curvature)) / curvature, 1e-12);
  EXPECT_NEAR(forward.yaw, curvature, 1e-12);

  const ArcPose reverse = integrateArcStep(start, -1.0, curvature);
  EXPECT_NEAR(reverse.x, -std::sin(curvature) / curvature, 1e-12);
  EXPECT_NEAR(reverse.y, (1.0 - std::cos(curvature)) / curvature, 1e-12);
  EXPECT_NEAR(reverse.yaw, -curvature, 1e-12);
}

TEST(AckermannRecoveryGeometry, IntegratesStraightStep)
{
  const double half_pi = std::acos(-1.0) / 2.0;
  const ArcPose start{1.0, 2.0, half_pi};
  const ArcPose result = integrateArcStep(start, 0.7, 0.0);
  EXPECT_NEAR(result.x, 1.0, 1e-12);
  EXPECT_NEAR(result.y, 2.7, 1e-12);
  EXPECT_NEAR(result.yaw, half_pi, 1e-12);
}

TEST(AckermannRecoveryGeometry, AcceptsFreeFootprintAndReportsMaximumCost)
{
  costmap_2d::Costmap2D costmap(20, 20, 0.1, -1.0, -1.0,
                                costmap_2d::FREE_SPACE);
  setWorldCost(costmap, 0.2, 0.0, 100);

  unsigned char observed = 0;
  EXPECT_TRUE(footprintInteriorIsSafe(costmap, centeredRectangle(), 252, &observed));
  EXPECT_EQ(observed, 100);
}

TEST(AckermannRecoveryGeometry, RejectsLethalCellInsideFootprint)
{
  costmap_2d::Costmap2D costmap(20, 20, 0.1, -1.0, -1.0,
                                costmap_2d::FREE_SPACE);
  // This cell is inside the polygon but is neither its center nor its outline.
  setWorldCost(costmap, 0.2, 0.0, costmap_2d::LETHAL_OBSTACLE);
  EXPECT_FALSE(footprintInteriorIsSafe(costmap, centeredRectangle(), 252));
}

TEST(AckermannRecoveryGeometry, RejectsUnknownAndInscribedInteriorCells)
{
  costmap_2d::Costmap2D costmap(20, 20, 0.1, -1.0, -1.0,
                                costmap_2d::FREE_SPACE);
  setWorldCost(costmap, -0.2, 0.0, costmap_2d::NO_INFORMATION);
  EXPECT_FALSE(footprintInteriorIsSafe(costmap, centeredRectangle(), 252));

  setWorldCost(costmap, -0.2, 0.0, costmap_2d::FREE_SPACE);
  setWorldCost(costmap, 0.2, 0.0, costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_FALSE(footprintInteriorIsSafe(costmap, centeredRectangle(), 252));
}

TEST(AckermannRecoveryGeometry, RejectsFootprintOutsideMap)
{
  costmap_2d::Costmap2D costmap(20, 20, 0.1, -1.0, -1.0,
                                costmap_2d::FREE_SPACE);
  std::vector<geometry_msgs::Point> footprint = centeredRectangle();
  for (geometry_msgs::Point& vertex : footprint)
  {
    vertex.x += 1.0;
  }
  EXPECT_FALSE(footprintInteriorIsSafe(costmap, footprint, 252));
}

TEST(AckermannRecoveryInterruptGate, FreshInterruptRejectsExactlyOneRun)
{
  PendingInterruptGate gate;
  gate.record(10.0);

  EXPECT_TRUE(gate.consumeIfFresh(10.2, 0.5));
  EXPECT_FALSE(gate.consumeIfFresh(10.3, 0.5));
}

TEST(AckermannRecoveryInterruptGate, OldInterruptDoesNotLatch)
{
  PendingInterruptGate gate;
  gate.record(10.0);

  EXPECT_FALSE(gate.consumeIfFresh(11.0, 0.5));
  EXPECT_FALSE(gate.consumeIfFresh(11.1, 0.5));
}

TEST(AckermannRecoveryInterruptGate, MostRecentInterruptDefinesFreshness)
{
  PendingInterruptGate gate;
  gate.record(10.0);
  gate.record(11.0);

  EXPECT_TRUE(gate.consumeIfFresh(11.2, 0.5));
}

TEST(AckermannRecoveryInterruptGate, ClockRegressionFailsSafeOnce)
{
  PendingInterruptGate gate;
  gate.record(10.0);

  EXPECT_TRUE(gate.consumeIfFresh(9.9, 0.5));
  EXPECT_FALSE(gate.consumeIfFresh(9.9, 0.5));
}

}  // namespace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
