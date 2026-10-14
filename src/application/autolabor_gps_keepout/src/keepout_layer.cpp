#include <autolabor_gps_keepout/KeepoutPolygons.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/cost_values.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <cmath>
#include <mutex>

namespace autolabor_gps_keepout
{
class KeepoutLayer : public costmap_2d::Layer
{
public:
  void onInitialize() override
  {
    ros::NodeHandle nh;
    current_ = true;
    enabled_ = true;
    subscriber_ = nh.subscribe("/gps/keepout/polygons", 1, &KeepoutLayer::receive, this);
  }

  void updateBounds(double, double, double, double* min_x, double* min_y,
                    double* max_x, double* max_y) override
  {
    std::lock_guard<std::mutex> guard(mutex_);
    if (polygons_.empty() && !keep_in_enabled_ && !dirty_) return;
    auto* map = layered_costmap_->getCostmap();
    *min_x = std::min(*min_x, map->getOriginX());
    *min_y = std::min(*min_y, map->getOriginY());
    *max_x = std::max(*max_x, map->getOriginX()+map->getSizeInMetersX());
    *max_y = std::max(*max_y, map->getOriginY()+map->getSizeInMetersY());
  }

  void updateCosts(costmap_2d::Costmap2D& map, int min_i, int min_j, int max_i, int max_j) override
  {
    std::lock_guard<std::mutex> guard(mutex_);
    // The master resets this window before replaying all layers. Repaint only
    // this layer's current polygons, so deleting a polygon leaves no ghosts.
    if (!polygons_.empty() || keep_in_enabled_)
      for (int j = min_j; j < max_j; ++j)
      {
        double x, y; map.mapToWorld(min_i, j, x, y);
        // Intersect each row once; an airport has thousands of vertices.
        // Per-cell ray casting made a 400 x 400 window unnecessarily costly.
        const auto forbidden = crossings(polygons_, y);
        const auto allowed = crossings(keep_in_polygons_, y);
        std::size_t out = 0, in = 0;
        for (int i = min_i; i < max_i; ++i)
        {
          map.mapToWorld(i, j, x, y);
          while (out < forbidden.size() && forbidden[out] <= x) ++out;
          while (in < allowed.size() && allowed[in] <= x) ++in;
          const bool occupied = (out % 2 != 0) || (keep_in_enabled_ && in % 2 == 0);
          if (occupied) map.setCost(i, j, costmap_2d::LETHAL_OBSTACLE);
        }
      }
    dirty_ = false;
    current_ = true;
  }

  void reset() override { std::lock_guard<std::mutex> guard(mutex_); dirty_ = true; }

private:
  static std::vector<double> crossings(const std::vector<geometry_msgs::Polygon>& contours, double y)
  {
    std::vector<double> result;
    for (const auto& ring : contours)
      for (std::size_t a=0, b=ring.points.size()-1; a<ring.points.size(); b=a++)
      {
        const auto& p=ring.points[a]; const auto& q=ring.points[b];
        if ((p.y > y) != (q.y > y))
          result.push_back((q.x-p.x)*(y-p.y)/(q.y-p.y)+p.x);
      }
    std::sort(result.begin(), result.end());
    return result;
  }

  void receive(const KeepoutPolygons::ConstPtr& msg)
  {
    if (msg->header.frame_id != layered_costmap_->getGlobalFrameID())
    {
      ROS_WARN_THROTTLE(5, "Ignoring GPS keepout polygons in a different coordinate frame");
      return;
    }
    for (const auto* contours : {&msg->polygons, &msg->keep_in_polygons})
      for (const auto& ring : *contours)
      {
        if (ring.points.size() < 3) return;
        for (const auto& p : ring.points)
          if (!std::isfinite(p.x) || !std::isfinite(p.y)) return;
      }
    std::lock_guard<std::mutex> guard(mutex_);
    polygons_ = msg->polygons;
    keep_in_polygons_ = msg->keep_in_polygons;
    keep_in_enabled_ = msg->keep_in_enabled;
    dirty_ = true;
  }
  ros::Subscriber subscriber_;
  std::mutex mutex_;
  std::vector<geometry_msgs::Polygon> polygons_, keep_in_polygons_;
  bool keep_in_enabled_ = false;
  bool dirty_ = false;
};
}  // namespace autolabor_gps_keepout
PLUGINLIB_EXPORT_CLASS(autolabor_gps_keepout::KeepoutLayer, costmap_2d::Layer)
