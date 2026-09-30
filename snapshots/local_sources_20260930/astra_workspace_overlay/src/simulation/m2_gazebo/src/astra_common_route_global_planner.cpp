// New nav_core bridge only. No grid search, path smoothing, or actor inputs.
// Root integration must register this separate plugin and a shared static map.
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <pluginlib/class_list_macros.h>

namespace astra_common_route {
class FixedRouteGlobalPlanner : public nav_core::BaseGlobalPlanner {
 public:
  FixedRouteGlobalPlanner() = default;
  void initialize(std::string name, costmap_2d::Costmap2DROS *costmap) override {
    if (initialized_) return;
    if (!costmap || costmap->getGlobalFrameID() != "map")
      throw std::runtime_error("Common route bridge requires global costmap frame map");
    ros::NodeHandle nh("~/" + name);
    std::string service;
    nh.param<std::string>("route_service", service, "/astra/common_route/get_plan");
    service_ = nh.serviceClient<nav_msgs::GetPlan>(service, false);
    provided_ = nh.advertise<nav_msgs::Path>("provided_path", 10, true);
    initialized_ = true;
  }
  bool makePlan(const geometry_msgs::PoseStamped &start,
                const geometry_msgs::PoseStamped &goal,
                std::vector<geometry_msgs::PoseStamped> &plan) override {
    plan.clear();
    if (!initialized_ || start.header.frame_id != "map" || goal.header.frame_id != "map") return false;
    nav_msgs::GetPlan call; call.request.start = start; call.request.goal = goal; call.request.tolerance = 0.;
    if (!service_.call(call) || call.response.plan.header.frame_id != "map" || call.response.plan.poses.size() < 2) return false;
    for (const auto &pose : call.response.plan.poses) {
      const auto &p = pose.pose.position; const auto &q = pose.pose.orientation;
      const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
      if (pose.header.frame_id != "map" || !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
          !std::isfinite(norm) || std::abs(norm-1.) > 1e-6 || std::abs(q.x)+std::abs(q.y)>1e-9) return false;
    }
    const auto &end = call.response.plan.poses.back().pose.position;
    if (std::hypot(end.x-goal.pose.position.x,end.y-goal.pose.position.y)>1e-6) return false;
    // No start insertion or method-dependent pruning. move_base passes this
    // same vector into the selected local planner's setPlan implementation.
    plan = call.response.plan.poses; provided_.publish(call.response.plan); return true;
  }
 private:
  bool initialized_ = false;
  ros::ServiceClient service_;
  ros::Publisher provided_;
};
}
PLUGINLIB_EXPORT_CLASS(astra_common_route::FixedRouteGlobalPlanner, nav_core::BaseGlobalPlanner)
