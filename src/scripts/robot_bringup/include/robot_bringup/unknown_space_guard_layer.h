#ifndef ROBOT_BRINGUP_UNKNOWN_SPACE_GUARD_LAYER_H_
#define ROBOT_BRINGUP_UNKNOWN_SPACE_GUARD_LAYER_H_

#include <costmap_2d/layer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <mutex>
#include <string>

namespace robot_bringup
{

// Invalid maps and coordinates outside the source map are deliberately
// classified as unknown.  The costmap plugin therefore fails closed while a
// map is absent, malformed, or does not cover the rolling local window.
bool occupancyGridCellIsUnknown(const nav_msgs::OccupancyGrid& map,
                                double world_x, double world_y);

class UnknownSpaceGuardLayer : public costmap_2d::Layer
{
public:
  UnknownSpaceGuardLayer();

  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y,
                    double* max_x, double* max_y) override;
  void updateCosts(costmap_2d::Costmap2D& master_grid,
                   int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;

protected:
  void onInitialize() override;

private:
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& message);
  bool mapMatchesCostmapFrame() const;

  ros::Subscriber map_subscriber_;
  nav_msgs::OccupancyGrid map_;
  std::string map_topic_;
  mutable std::mutex mutex_;
  bool map_received_;
  bool full_update_required_;
};

}  // namespace robot_bringup

#endif  // ROBOT_BRINGUP_UNKNOWN_SPACE_GUARD_LAYER_H_
