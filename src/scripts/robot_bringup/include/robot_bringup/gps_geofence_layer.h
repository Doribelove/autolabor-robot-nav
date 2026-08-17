#ifndef ROBOT_BRINGUP_GPS_GEOFENCE_LAYER_H_
#define ROBOT_BRINGUP_GPS_GEOFENCE_LAYER_H_

#include <costmap_2d/layer.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include <limits>
#include <mutex>
#include <string>
#include <vector>

namespace robot_bringup
{

struct FencePoint
{
  double x = 0.0;
  double y = 0.0;
};

struct GpsFencePoint
{
  double latitude = 0.0;
  double longitude = 0.0;
};

double pointToSegmentDistance(const FencePoint& point, const FencePoint& start, const FencePoint& end);
bool pointInPolygon(const FencePoint& point, const std::vector<FencePoint>& polygon);
bool pointInKeepout(const FencePoint& point, const std::vector<FencePoint>& polygon, double margin_m);

class GpsGeofenceLayer : public costmap_2d::Layer
{
public:
  GpsGeofenceLayer();

  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;
  void updateCosts(costmap_2d::Costmap2D& master_grid,
                   int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;

protected:
  void onInitialize() override;

private:
  struct Region
  {
    std::string name;
    std::vector<GpsFencePoint> gps_vertices;
    std::vector<FencePoint> vertices;
    double min_x = 0.0;
    double min_y = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;
  };

  void loadRegions(const ros::NodeHandle& private_nh);
  void reloadCallback(const std_msgs::Empty::ConstPtr& message);
  void rememberProjectedBounds();
  bool refreshProjection();
  bool cellIsForbidden(double world_x, double world_y) const;

  std::vector<Region> regions_;
  double hard_margin_m_;
  double projected_origin_lat_;
  double projected_origin_lon_;
  bool projection_ready_;
  bool waiting_for_origin_logged_;
  ros::Subscriber reload_subscriber_;
  std::mutex mutex_;
  bool dirty_bounds_pending_;
  double dirty_min_x_;
  double dirty_min_y_;
  double dirty_max_x_;
  double dirty_max_y_;
};

}  // namespace robot_bringup

#endif  // ROBOT_BRINGUP_GPS_GEOFENCE_LAYER_H_
