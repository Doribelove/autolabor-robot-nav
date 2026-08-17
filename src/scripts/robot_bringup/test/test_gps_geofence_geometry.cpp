#include <gtest/gtest.h>

#include <robot_bringup/gps_geofence_layer.h>

#include <vector>

namespace
{

std::vector<robot_bringup::FencePoint> square()
{
  return {{0.0, 0.0}, {4.0, 0.0}, {4.0, 4.0}, {0.0, 4.0}};
}

TEST(GpsGeofenceGeometry, FillsPolygonInteriorAndBoundary)
{
  const auto polygon = square();
  EXPECT_TRUE(robot_bringup::pointInPolygon({2.0, 2.0}, polygon));
  EXPECT_TRUE(robot_bringup::pointInPolygon({4.0, 2.0}, polygon));
  EXPECT_FALSE(robot_bringup::pointInPolygon({5.0, 2.0}, polygon));
}

TEST(GpsGeofenceGeometry, AppliesHardMarginOutsidePolygon)
{
  const auto polygon = square();
  EXPECT_TRUE(robot_bringup::pointInKeepout({4.8, 2.0}, polygon, 1.0));
  EXPECT_FALSE(robot_bringup::pointInKeepout({5.2, 2.0}, polygon, 1.0));
  EXPECT_TRUE(robot_bringup::pointInKeepout({4.6, 4.6}, polygon, 1.0));
}

TEST(GpsGeofenceGeometry, HandlesConcaveManualPolygon)
{
  const std::vector<robot_bringup::FencePoint> polygon = {
      {0.0, 0.0}, {4.0, 0.0}, {4.0, 4.0}, {2.0, 2.0}, {0.0, 4.0}};
  EXPECT_TRUE(robot_bringup::pointInPolygon({1.0, 1.0}, polygon));
  EXPECT_FALSE(robot_bringup::pointInPolygon({2.0, 3.0}, polygon));
}

}  // namespace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
