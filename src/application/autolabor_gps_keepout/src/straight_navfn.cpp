#include <autolabor_gps_keepout/straight_plan.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/footprint.h>
#include <navfn/navfn_ros.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <cmath>

namespace autolabor_gps_keepout {

bool makeFreeSpaceStraightPlan(costmap_2d::Costmap2D& map, double radius,
                              const geometry_msgs::PoseStamped& start,
                              const geometry_msgs::PoseStamped& goal,
                              std::vector<geometry_msgs::PoseStamped>& plan) {
  plan.clear();
  const auto& a = start.pose.position;
  const auto& b = goal.pose.position;
  if (start.header.frame_id != goal.header.frame_id ||
      !std::isfinite(a.x) || !std::isfinite(a.y) ||
      !std::isfinite(b.x) || !std::isfinite(b.y) ||
      !std::isfinite(radius) || radius <= 0.0) return false;

  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*map.getMutex());
  const double resolution = map.getResolution();
  if (resolution <= 0.0) return false;
  // Check every cell touching the swept circumscribed footprint, including
  // cell corners. No cost is cleared or reduced, even for the start cell.
  const double clearance = radius + resolution / std::sqrt(2.0);
  unsigned int min_x, min_y, max_x, max_y;
  if (!map.worldToMap(std::min(a.x, b.x) - clearance,
                      std::min(a.y, b.y) - clearance, min_x, min_y) ||
      !map.worldToMap(std::max(a.x, b.x) + clearance,
                      std::max(a.y, b.y) + clearance, max_x, max_y)) return false;
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double length_squared = dx * dx + dy * dy;
  for (unsigned int y = min_y; y <= max_y; ++y) {
    for (unsigned int x = min_x; x <= max_x; ++x) {
      if (map.getCost(x, y) == costmap_2d::FREE_SPACE) continue;
      double wx, wy;
      map.mapToWorld(x, y, wx, wy);
      const double t = length_squared > 1e-12
          ? std::max(0.0, std::min(1.0, ((wx-a.x)*dx+(wy-a.y)*dy)/length_squared))
          : 0.0;
      if (std::hypot(wx-a.x-t*dx, wy-a.y-t*dy) <= clearance) return false;
    }
  }

  const double length = std::sqrt(length_squared);
  const int steps = std::max(1, static_cast<int>(std::ceil(length / (resolution * 0.5))));
  const double yaw = std::atan2(dy, dx);
  plan.reserve(steps + 1);
  for (int i = 0; i <= steps; ++i) {
    geometry_msgs::PoseStamped pose = start;
    const double t = static_cast<double>(i) / steps;
    pose.pose.position.x = a.x + t * dx;
    pose.pose.position.y = a.y + t * dy;
    pose.pose.orientation.x = pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(yaw * 0.5);
    pose.pose.orientation.w = std::cos(yaw * 0.5);
    plan.push_back(pose);
  }
  plan.front() = start;
  plan.back() = goal;
  return true;
}

// Retain the NavfnROS plugin basename, topics and original fallback planner.
// This changes only the open-space global reference, never cmd_vel ownership.
class NavfnROS : public navfn::NavfnROS {
 public:
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override {
    navfn::NavfnROS::initialize(name, costmap_ros);
    costmap_ros_ = costmap_ros;
  }

  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override {
    if (costmap_ros_ && start.header.frame_id == costmap_ros_->getGlobalFrameID() &&
        goal.header.frame_id == costmap_ros_->getGlobalFrameID()) {
      double inscribed = 0.0, circumscribed = 0.0;
      costmap_2d::calculateMinAndMaxDistances(costmap_ros_->getRobotFootprint(),
                                           inscribed, circumscribed);
      if (makeFreeSpaceStraightPlan(*costmap_ros_->getCostmap(), circumscribed,
                                   start, goal, plan)) {
        publishPlan(plan, 0.0, 1.0, 0.0, 1.0);
        return true;
      }
    }
    return navfn::NavfnROS::makePlan(start, goal, plan);
  }

 private:
  costmap_2d::Costmap2DROS* costmap_ros_ = nullptr;
};
}

PLUGINLIB_EXPORT_CLASS(autolabor_gps_keepout::NavfnROS, nav_core::BaseGlobalPlanner)
