#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace robot_bringup
{
namespace moving_self_crop
{

struct Quaternion
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

struct Pose2d
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct CropBox
{
  double min_x = -0.75;
  double max_x = 0.75;
  double min_y = -0.50;
  double max_y = 0.50;

  bool valid() const
  {
    return std::isfinite(min_x) && std::isfinite(max_x) &&
           std::isfinite(min_y) && std::isfinite(max_y) &&
           min_x < max_x && min_y < max_y;
  }
};

inline bool normalizeQuaternion(const Quaternion& input, Quaternion* output)
{
  if (output == nullptr || !std::isfinite(input.x) || !std::isfinite(input.y) ||
      !std::isfinite(input.z) || !std::isfinite(input.w))
    return false;
  const double norm_squared = input.x * input.x + input.y * input.y +
                              input.z * input.z + input.w * input.w;
  if (!(norm_squared > 1e-12) || !std::isfinite(norm_squared))
    return false;
  const double inverse_norm = 1.0 / std::sqrt(norm_squared);
  output->x = input.x * inverse_norm;
  output->y = input.y * inverse_norm;
  output->z = input.z * inverse_norm;
  output->w = input.w * inverse_norm;
  return true;
}

inline void rotateVector(const Quaternion& quaternion, const double x,
                         const double y, const double z, double* output_x,
                         double* output_y, double* output_z)
{
  const double tx = 2.0 * (quaternion.y * z - quaternion.z * y);
  const double ty = 2.0 * (quaternion.z * x - quaternion.x * z);
  const double tz = 2.0 * (quaternion.x * y - quaternion.y * x);
  *output_x = x + quaternion.w * tx + quaternion.y * tz - quaternion.z * ty;
  *output_y = y + quaternion.w * ty + quaternion.z * tx - quaternion.x * tz;
  *output_z = z + quaternion.w * tz + quaternion.x * ty - quaternion.y * tx;
}

inline double yawFromQuaternion(const Quaternion& quaternion)
{
  return std::atan2(
      2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
      1.0 - 2.0 * (quaternion.y * quaternion.y +
                   quaternion.z * quaternion.z));
}

inline bool basePoseInMap(const double body_x, const double body_y,
                          const double body_z, const Quaternion& input_orientation,
                          const double body_to_base_x,
                          const double body_to_base_y,
                          const double body_to_base_z, Pose2d* base_pose)
{
  if (base_pose == nullptr || !std::isfinite(body_x) || !std::isfinite(body_y) ||
      !std::isfinite(body_z) || !std::isfinite(body_to_base_x) ||
      !std::isfinite(body_to_base_y) || !std::isfinite(body_to_base_z))
    return false;
  Quaternion orientation;
  if (!normalizeQuaternion(input_orientation, &orientation))
    return false;
  double offset_x = 0.0;
  double offset_y = 0.0;
  double offset_z = 0.0;
  rotateVector(orientation, body_to_base_x, body_to_base_y, body_to_base_z,
               &offset_x, &offset_y, &offset_z);
  base_pose->x = body_x + offset_x;
  base_pose->y = body_y + offset_y;
  base_pose->yaw = yawFromQuaternion(orientation);
  return std::isfinite(base_pose->x) && std::isfinite(base_pose->y) &&
         std::isfinite(base_pose->yaw);
}

inline bool mapPointInsideBaseCrop(
    const double point_x, const double point_y, const double point_z,
    const double body_x, const double body_y, const double body_z,
    const Quaternion& input_orientation, const double body_to_base_x,
    const double body_to_base_y, const double body_to_base_z,
    const CropBox& crop)
{
  if (!crop.valid() || !std::isfinite(point_x) || !std::isfinite(point_y) ||
      !std::isfinite(point_z) || !std::isfinite(body_x) ||
      !std::isfinite(body_y) || !std::isfinite(body_z) ||
      !std::isfinite(body_to_base_x) || !std::isfinite(body_to_base_y) ||
      !std::isfinite(body_to_base_z))
    return false;
  Quaternion orientation;
  if (!normalizeQuaternion(input_orientation, &orientation))
    return false;
  const Quaternion inverse{-orientation.x, -orientation.y, -orientation.z,
                           orientation.w};
  double point_body_x = 0.0;
  double point_body_y = 0.0;
  double point_body_z = 0.0;
  rotateVector(inverse, point_x - body_x, point_y - body_y, point_z - body_z,
               &point_body_x, &point_body_y, &point_body_z);
  const double point_base_x = point_body_x - body_to_base_x;
  const double point_base_y = point_body_y - body_to_base_y;
  return crop.min_x <= point_base_x && point_base_x <= crop.max_x &&
         crop.min_y <= point_base_y && point_base_y <= crop.max_y;
}

inline double normalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline std::size_t interpolationSteps(const Pose2d& start, const Pose2d& finish,
                                      const double maximum_linear_step,
                                      const double maximum_angular_step)
{
  if (!(maximum_linear_step > 0.0) || !(maximum_angular_step > 0.0))
    return 0;
  const double distance = std::hypot(finish.x - start.x, finish.y - start.y);
  const double yaw_distance =
      std::fabs(normalizeAngle(finish.yaw - start.yaw));
  return std::max<std::size_t>(
      1U, std::max<std::size_t>(
              static_cast<std::size_t>(std::ceil(distance / maximum_linear_step)),
              static_cast<std::size_t>(
                  std::ceil(yaw_distance / maximum_angular_step))));
}

inline Pose2d interpolatePose(const Pose2d& start, const Pose2d& finish,
                              const double fraction)
{
  const double bounded_fraction = std::max(0.0, std::min(1.0, fraction));
  const double yaw_delta = normalizeAngle(finish.yaw - start.yaw);
  return Pose2d{start.x + bounded_fraction * (finish.x - start.x),
                start.y + bounded_fraction * (finish.y - start.y),
                normalizeAngle(start.yaw + bounded_fraction * yaw_delta)};
}

inline bool cropIntersectsGridCell(const CropBox& crop, const Pose2d& base_pose,
                                   const int cell_x, const int cell_y,
                                   const double resolution)
{
  if (!crop.valid() || !(resolution > 0.0) || !std::isfinite(base_pose.x) ||
      !std::isfinite(base_pose.y) || !std::isfinite(base_pose.yaw))
    return false;

  const double local_center_x = 0.5 * (crop.min_x + crop.max_x);
  const double local_center_y = 0.5 * (crop.min_y + crop.max_y);
  const double half_x = 0.5 * (crop.max_x - crop.min_x);
  const double half_y = 0.5 * (crop.max_y - crop.min_y);
  const double cosine = std::cos(base_pose.yaw);
  const double sine = std::sin(base_pose.yaw);
  const double axis_x_x = cosine;
  const double axis_x_y = sine;
  const double axis_y_x = -sine;
  const double axis_y_y = cosine;
  const double crop_center_x = base_pose.x + cosine * local_center_x -
                               sine * local_center_y;
  const double crop_center_y = base_pose.y + sine * local_center_x +
                               cosine * local_center_y;
  const double cell_center_x = (static_cast<double>(cell_x) + 0.5) * resolution;
  const double cell_center_y = (static_cast<double>(cell_y) + 0.5) * resolution;
  const double cell_half = 0.5 * resolution;
  const double delta_x = crop_center_x - cell_center_x;
  const double delta_y = crop_center_y - cell_center_y;

  const auto separated = [&](const double axis_x, const double axis_y) {
    const double center_distance =
        std::fabs(delta_x * axis_x + delta_y * axis_y);
    const double crop_radius =
        half_x * std::fabs(axis_x_x * axis_x + axis_x_y * axis_y) +
        half_y * std::fabs(axis_y_x * axis_x + axis_y_y * axis_y);
    const double cell_radius =
        cell_half * (std::fabs(axis_x) + std::fabs(axis_y));
    return center_distance > crop_radius + cell_radius + 1e-12;
  };

  return !separated(1.0, 0.0) && !separated(0.0, 1.0) &&
         !separated(axis_x_x, axis_x_y) &&
         !separated(axis_y_x, axis_y_y);
}

}  // namespace moving_self_crop
}  // namespace robot_bringup
