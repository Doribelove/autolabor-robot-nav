#include <gtest/gtest.h>

#include <robot_bringup/unknown_space_guard_layer.h>

#include <cmath>

namespace
{

nav_msgs::OccupancyGrid makeMap(double yaw = 0.0)
{
  nav_msgs::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.width = 3;
  map.info.height = 2;
  map.info.resolution = 1.0;
  map.info.origin.position.x = 10.0;
  map.info.origin.position.y = 20.0;
  map.info.origin.orientation.z = std::sin(0.5 * yaw);
  map.info.origin.orientation.w = std::cos(0.5 * yaw);
  map.data.assign(6, 0);
  map.data[1] = -1;
  map.data[2] = 100;
  return map;
}

}  // namespace

TEST(UnknownSpaceGuard, DistinguishesUnknownFromKnownOccupiedCells)
{
  const nav_msgs::OccupancyGrid map = makeMap();
  EXPECT_FALSE(robot_bringup::occupancyGridCellIsUnknown(map, 10.5, 20.5));
  EXPECT_TRUE(robot_bringup::occupancyGridCellIsUnknown(map, 11.5, 20.5));
  // Occupied cells are known; the ordinary StaticLayer remains responsible
  // for assigning their lethal cost.
  EXPECT_FALSE(robot_bringup::occupancyGridCellIsUnknown(map, 12.5, 20.5));
}

TEST(UnknownSpaceGuard, TreatsOutsideAndMalformedMapsAsUnknown)
{
  nav_msgs::OccupancyGrid map = makeMap();
  EXPECT_TRUE(robot_bringup::occupancyGridCellIsUnknown(map, 9.99, 20.5));
  EXPECT_TRUE(robot_bringup::occupancyGridCellIsUnknown(map, 13.01, 20.5));
  map.data.pop_back();
  EXPECT_TRUE(robot_bringup::occupancyGridCellIsUnknown(map, 10.5, 20.5));
}

TEST(UnknownSpaceGuard, AppliesRotatedMapOrigin)
{
  const nav_msgs::OccupancyGrid map = makeMap(0.5 * std::acos(-1.0));
  // Cell-local centre (1.5, 0.5) rotates to map point (9.5, 21.5).
  EXPECT_TRUE(robot_bringup::occupancyGridCellIsUnknown(map, 9.5, 21.5));
  EXPECT_FALSE(robot_bringup::occupancyGridCellIsUnknown(map, 9.5, 20.5));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
