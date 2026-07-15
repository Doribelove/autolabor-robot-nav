#ifndef ROBOT_BRINGUP_ACKERMANN_RECOVERY_GEOMETRY_H
#define ROBOT_BRINGUP_ACKERMANN_RECOVERY_GEOMETRY_H

#include <vector>

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>

namespace robot_bringup
{
namespace recovery_detail
{

struct ArcPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/** Integrate one signed-distance step at constant Ackermann curvature. */
ArcPose integrateArcStep(const ArcPose& pose, double signed_distance,
                         double curvature);

/**
 * Check every costmap cell covered by a convex, world-frame footprint.
 *
 * The caller must hold costmap.getMutex(). Unknown, inscribed-inflated and
 * lethal cells are always rejected, as are polygons that leave the map.
 */
bool footprintInteriorIsSafe(costmap_2d::Costmap2D& costmap,
                             const std::vector<geometry_msgs::Point>& oriented_footprint,
                             unsigned char maximum_allowed_cost,
                             unsigned char* observed_maximum_cost = nullptr);

}  // namespace recovery_detail
}  // namespace robot_bringup

#endif  // ROBOT_BRINGUP_ACKERMANN_RECOVERY_GEOMETRY_H
