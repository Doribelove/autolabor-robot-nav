#include <gtest/gtest.h>

#include <robot_bringup/moving_self_crop.h>

#include <cmath>

namespace crop = robot_bringup::moving_self_crop;

namespace
{
crop::Quaternion quaternionFromRpy(const double roll, const double pitch,
                                   const double yaw)
{
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  return crop::Quaternion{sr * cp * cy - cr * sp * sy,
                          cr * sp * cy + sr * cp * sy,
                          cr * cp * sy - sr * sp * cy,
                          cr * cp * cy + sr * sp * sy};
}

void mapPointFromBase(const crop::Quaternion& orientation,
                      const double body_x, const double body_y,
                      const double body_z, const double body_to_base_x,
                      const double body_to_base_y,
                      const double body_to_base_z, const double local_x,
                      const double local_y, const double local_z,
                      double* map_x, double* map_y, double* map_z)
{
  double rotated_x = 0.0;
  double rotated_y = 0.0;
  double rotated_z = 0.0;
  crop::rotateVector(orientation, body_to_base_x + local_x,
                     body_to_base_y + local_y,
                     body_to_base_z + local_z, &rotated_x, &rotated_y,
                     &rotated_z);
  *map_x = body_x + rotated_x;
  *map_y = body_y + rotated_y;
  *map_z = body_z + rotated_z;
}
}  // namespace

TEST(MovingSelfCrop, UsesFullPoseAndBodyToBaseOffset)
{
  const crop::CropBox bounds{-0.75, 0.75, -0.50, 0.50};
  const crop::Quaternion orientation = quaternionFromRpy(0.15, -0.10, 1.2);
  double map_x = 0.0;
  double map_y = 0.0;
  double map_z = 0.0;
  mapPointFromBase(orientation, 4.0, -3.0, 0.2, -0.211, -0.02329,
                   -0.95588, 0.70, -0.45, 0.55, &map_x, &map_y, &map_z);
  EXPECT_TRUE(crop::mapPointInsideBaseCrop(
      map_x, map_y, map_z, 4.0, -3.0, 0.2, orientation, -0.211,
      -0.02329, -0.95588, bounds));

  mapPointFromBase(orientation, 4.0, -3.0, 0.2, -0.211, -0.02329,
                   -0.95588, 0.76, -0.45, 0.55, &map_x, &map_y, &map_z);
  EXPECT_FALSE(crop::mapPointInsideBaseCrop(
      map_x, map_y, map_z, 4.0, -3.0, 0.2, orientation, -0.211,
      -0.02329, -0.95588, bounds));
}

TEST(MovingSelfCrop, IncludesCropBoundary)
{
  const crop::CropBox bounds{-0.75, 0.75, -0.50, 0.50};
  const crop::Quaternion identity;
  EXPECT_TRUE(crop::mapPointInsideBaseCrop(
      -0.75, 0.50, 0.0, 0.0, 0.0, 0.0, identity, 0.0, 0.0, 0.0,
      bounds));
}

TEST(MovingSelfCrop, RejectsInvalidQuaternion)
{
  const crop::CropBox bounds{-0.75, 0.75, -0.50, 0.50};
  const crop::Quaternion invalid{0.0, 0.0, 0.0, 0.0};
  crop::Pose2d pose;
  EXPECT_FALSE(crop::basePoseInMap(0.0, 0.0, 0.0, invalid, -0.211,
                                   -0.02329, -0.95588, &pose));
  EXPECT_FALSE(crop::mapPointInsideBaseCrop(
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, invalid, -0.211, -0.02329,
      -0.95588, bounds));
}

TEST(MovingSelfCrop, RotatedSweepIntersectsNegativeGridCells)
{
  const crop::CropBox footprint{-0.62, 0.62, -0.45, 0.45};
  const crop::Pose2d pose{-0.05, -0.05, 0.5 * M_PI};
  EXPECT_TRUE(crop::cropIntersectsGridCell(footprint, pose, -1, -1, 0.10));
  EXPECT_TRUE(crop::cropIntersectsGridCell(footprint, pose, 3, -1, 0.10));
  EXPECT_FALSE(crop::cropIntersectsGridCell(footprint, pose, 7, -1, 0.10));
}

TEST(MovingSelfCrop, InterpolatesTranslationAndShortestYawWithoutGaps)
{
  const crop::Pose2d start{0.0, 0.0, 170.0 * M_PI / 180.0};
  const crop::Pose2d finish{0.21, 0.0, -170.0 * M_PI / 180.0};
  const std::size_t steps =
      crop::interpolationSteps(start, finish, 0.05, 2.0 * M_PI / 180.0);
  EXPECT_EQ(10U, steps);
  const crop::Pose2d midpoint = crop::interpolatePose(start, finish, 0.5);
  EXPECT_NEAR(0.105, midpoint.x, 1e-12);
  EXPECT_NEAR(M_PI, std::fabs(midpoint.yaw), 1e-12);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
