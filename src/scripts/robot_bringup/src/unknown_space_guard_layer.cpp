#include <robot_bringup/unknown_space_guard_layer.h>

#include <costmap_2d/cost_values.h>
#include <pluginlib/class_list_macros.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace robot_bringup
{
namespace
{

constexpr double kQuaternionTolerance = 1.0e-3;

std::string normalizedFrame(std::string frame)
{
  while (!frame.empty() && frame.front() == '/')
  {
    frame.erase(frame.begin());
  }
  return frame;
}

bool validPlanarMap(const nav_msgs::OccupancyGrid& map)
{
  if (map.info.width == 0U || map.info.height == 0U ||
      !std::isfinite(map.info.resolution) || map.info.resolution <= 0.0F ||
      !std::isfinite(map.info.origin.position.x) ||
      !std::isfinite(map.info.origin.position.y))
  {
    return false;
  }
  if (map.info.width >
      std::numeric_limits<std::size_t>::max() / map.info.height)
  {
    return false;
  }
  if (map.data.size() !=
      static_cast<std::size_t>(map.info.width) * map.info.height)
  {
    return false;
  }
  const auto& quaternion = map.info.origin.orientation;
  if (!std::isfinite(quaternion.x) || !std::isfinite(quaternion.y) ||
      !std::isfinite(quaternion.z) || !std::isfinite(quaternion.w))
  {
    return false;
  }
  const double norm = std::sqrt(
      quaternion.x * quaternion.x + quaternion.y * quaternion.y +
      quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  return std::abs(norm - 1.0) <= kQuaternionTolerance &&
         std::abs(quaternion.x) <= kQuaternionTolerance &&
         std::abs(quaternion.y) <= kQuaternionTolerance;
}

}  // namespace

bool occupancyGridCellIsUnknown(const nav_msgs::OccupancyGrid& map,
                                double world_x, double world_y)
{
  if (!validPlanarMap(map) || !std::isfinite(world_x) ||
      !std::isfinite(world_y))
  {
    return true;
  }

  const auto& origin = map.info.origin;
  const double yaw = std::atan2(
      2.0 * (origin.orientation.w * origin.orientation.z),
      1.0 - 2.0 * origin.orientation.z * origin.orientation.z);
  const double dx = world_x - origin.position.x;
  const double dy = world_y - origin.position.y;
  const double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
  const double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
  const int64_t cell_x = static_cast<int64_t>(
      std::floor(local_x / map.info.resolution));
  const int64_t cell_y = static_cast<int64_t>(
      std::floor(local_y / map.info.resolution));
  if (cell_x < 0 || cell_y < 0 ||
      cell_x >= static_cast<int64_t>(map.info.width) ||
      cell_y >= static_cast<int64_t>(map.info.height))
  {
    return true;
  }
  const std::size_t index = static_cast<std::size_t>(cell_y) * map.info.width +
                            static_cast<std::size_t>(cell_x);
  return map.data[index] < 0;
}

UnknownSpaceGuardLayer::UnknownSpaceGuardLayer()
  : map_topic_("/map")
  , map_received_(false)
  , full_update_required_(true)
{
}

void UnknownSpaceGuardLayer::onInitialize()
{
  ros::NodeHandle private_nh("~/" + name_);
  private_nh.param("enabled", enabled_, true);
  private_nh.param("map_topic", map_topic_, std::string("/map"));

  current_ = !enabled_;
  if (enabled_)
  {
    ros::NodeHandle node;
    map_subscriber_ = node.subscribe(
        map_topic_, 1, &UnknownSpaceGuardLayer::mapCallback, this);
  }
  ROS_INFO_STREAM(name_ << ": unknown-space guard enabled="
                        << (enabled_ ? "true" : "false")
                        << ", source=" << map_topic_);
}

bool UnknownSpaceGuardLayer::mapMatchesCostmapFrame() const
{
  return map_received_ &&
         normalizedFrame(map_.header.frame_id) ==
             normalizedFrame(layered_costmap_->getGlobalFrameID());
}

void UnknownSpaceGuardLayer::mapCallback(
    const nav_msgs::OccupancyGrid::ConstPtr& message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!validPlanarMap(*message))
  {
    map_received_ = false;
    full_update_required_ = true;
    current_ = false;
    ROS_ERROR_THROTTLE(2.0,
                       "%s: rejected malformed occupancy grid; costmap remains fail-closed",
                       name_.c_str());
    return;
  }
  map_ = *message;
  map_received_ = true;
  full_update_required_ = true;
  current_ = mapMatchesCostmapFrame();
  if (!current_)
  {
    ROS_ERROR_STREAM(name_ << ": map frame " << map_.header.frame_id
                           << " does not match costmap frame "
                           << layered_costmap_->getGlobalFrameID()
                           << "; costmap remains fail-closed");
  }
}

void UnknownSpaceGuardLayer::updateBounds(
    double, double, double, double* min_x, double* min_y,
    double* max_x, double* max_y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_)
  {
    return;
  }

  // A rolling costmap can expose a new strip outside the static map at every
  // origin shift.  Cover its complete window on every update.  A fixed
  // global costmap only needs a full pass when a new map/reset arrives; later
  // obstacle-layer bounds still flow through updateCosts for re-masking.
  if (!layered_costmap_->isRolling() && !full_update_required_ &&
      mapMatchesCostmapFrame())
  {
    return;
  }
  const costmap_2d::Costmap2D* master = layered_costmap_->getCostmap();
  if (master->getSizeInCellsX() == 0U || master->getSizeInCellsY() == 0U)
  {
    return;
  }
  const double resolution = master->getResolution();
  const double origin_x = master->getOriginX();
  const double origin_y = master->getOriginY();
  *min_x = std::min(*min_x, origin_x);
  *min_y = std::min(*min_y, origin_y);
  *max_x = std::max(
      *max_x, origin_x + master->getSizeInCellsX() * resolution);
  *max_y = std::max(
      *max_y, origin_y + master->getSizeInCellsY() * resolution);
}

void UnknownSpaceGuardLayer::updateCosts(
    costmap_2d::Costmap2D& master_grid,
    int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_)
  {
    return;
  }
  const bool usable_map = mapMatchesCostmapFrame();
  const double master_resolution = master_grid.getResolution();
  const double master_origin_x = master_grid.getOriginX();
  const double master_origin_y = master_grid.getOriginY();
  double map_cos = 1.0;
  double map_sin = 0.0;
  double inverse_map_resolution = 0.0;
  if (usable_map)
  {
    const auto& orientation = map_.info.origin.orientation;
    const double map_yaw = std::atan2(
        2.0 * orientation.w * orientation.z,
        1.0 - 2.0 * orientation.z * orientation.z);
    map_cos = std::cos(map_yaw);
    map_sin = std::sin(map_yaw);
    inverse_map_resolution = 1.0 / map_.info.resolution;
  }
  for (int my = min_j; my < max_j; ++my)
  {
    const double world_y = master_origin_y +
                           (static_cast<double>(my) + 0.5) * master_resolution;
    for (int mx = min_i; mx < max_i; ++mx)
    {
      bool unknown = !usable_map;
      if (usable_map)
      {
        const double world_x = master_origin_x +
                               (static_cast<double>(mx) + 0.5) *
                                   master_resolution;
        const double dx = world_x - map_.info.origin.position.x;
        const double dy = world_y - map_.info.origin.position.y;
        const int64_t map_x = static_cast<int64_t>(std::floor(
            (map_cos * dx + map_sin * dy) * inverse_map_resolution));
        const int64_t map_y = static_cast<int64_t>(std::floor(
            (-map_sin * dx + map_cos * dy) * inverse_map_resolution));
        unknown = (
            map_x < 0 || map_y < 0 ||
            map_x >= static_cast<int64_t>(map_.info.width) ||
            map_y >= static_cast<int64_t>(map_.info.height));
        if (!unknown)
        {
          const std::size_t index =
              static_cast<std::size_t>(map_y) * map_.info.width +
              static_cast<std::size_t>(map_x);
          unknown = map_.data[index] < 0;
        }
      }
      if (unknown)
      {
        // NO_INFORMATION is 255, the highest costmap byte value.  This layer
        // runs last so live ray clearing cannot turn static unknown/outside-map
        // cells into free space.
        master_grid.setCost(
            static_cast<unsigned int>(mx), static_cast<unsigned int>(my),
            costmap_2d::NO_INFORMATION);
      }
    }
  }
  full_update_required_ = false;
  current_ = usable_map;
}

void UnknownSpaceGuardLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  full_update_required_ = true;
  current_ = !enabled_ || mapMatchesCostmapFrame();
}

}  // namespace robot_bringup

PLUGINLIB_EXPORT_CLASS(robot_bringup::UnknownSpaceGuardLayer,
                       costmap_2d::Layer)
