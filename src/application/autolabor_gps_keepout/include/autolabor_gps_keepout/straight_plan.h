#pragma once

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>

namespace autolabor_gps_keepout {
// Only shortcut a wholly FREE_SPACE swept footprint. Otherwise the caller
// retains Navfn's existing obstacle, unknown-space and goal-tolerance behavior.
bool makeFreeSpaceStraightPlan(costmap_2d::Costmap2D& map, double radius,
                              const geometry_msgs::PoseStamped& start,
                              const geometry_msgs::PoseStamped& goal,
                              std::vector<geometry_msgs::PoseStamped>& plan);
}
