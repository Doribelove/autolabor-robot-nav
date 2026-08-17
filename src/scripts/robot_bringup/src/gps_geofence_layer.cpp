#include <robot_bringup/gps_geofence_layer.h>

#include <costmap_2d/cost_values.h>
#include <pluginlib/class_list_macros.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot_bringup
{
namespace
{

constexpr double kEarthRadiusM = 6378137.0;
constexpr double kGeometryEpsilon = 1e-9;

double xmlNumber(const XmlRpc::XmlRpcValue& value, const std::string& label)
{
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
  {
    return static_cast<double>(value);
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    return static_cast<int>(value);
  }
  throw std::runtime_error(label + " must be numeric");
}

double signedArea(const std::vector<FencePoint>& polygon)
{
  double twice_area = 0.0;
  for (std::size_t i = 0; i < polygon.size(); ++i)
  {
    const FencePoint& current = polygon[i];
    const FencePoint& next = polygon[(i + 1) % polygon.size()];
    twice_area += current.x * next.y - next.x * current.y;
  }
  return 0.5 * twice_area;
}

}  // namespace

double pointToSegmentDistance(const FencePoint& point, const FencePoint& start, const FencePoint& end)
{
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double length_squared = dx * dx + dy * dy;
  if (length_squared <= kGeometryEpsilon)
  {
    return std::hypot(point.x - start.x, point.y - start.y);
  }

  const double projection = ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared;
  const double clamped = std::max(0.0, std::min(1.0, projection));
  const double nearest_x = start.x + clamped * dx;
  const double nearest_y = start.y + clamped * dy;
  return std::hypot(point.x - nearest_x, point.y - nearest_y);
}

bool pointInPolygon(const FencePoint& point, const std::vector<FencePoint>& polygon)
{
  if (polygon.size() < 3)
  {
    return false;
  }

  bool inside = false;
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++)
  {
    if (pointToSegmentDistance(point, polygon[j], polygon[i]) <= kGeometryEpsilon)
    {
      return true;
    }
    const bool crosses = ((polygon[i].y > point.y) != (polygon[j].y > point.y));
    if (crosses)
    {
      const double x_intersection =
          (polygon[j].x - polygon[i].x) * (point.y - polygon[i].y) /
              (polygon[j].y - polygon[i].y) +
          polygon[i].x;
      if (point.x < x_intersection)
      {
        inside = !inside;
      }
    }
  }
  return inside;
}

bool pointInKeepout(const FencePoint& point, const std::vector<FencePoint>& polygon, double margin_m)
{
  if (pointInPolygon(point, polygon))
  {
    return true;
  }
  if (margin_m <= 0.0)
  {
    return false;
  }
  for (std::size_t i = 0; i < polygon.size(); ++i)
  {
    if (pointToSegmentDistance(point, polygon[i], polygon[(i + 1) % polygon.size()]) <= margin_m)
    {
      return true;
    }
  }
  return false;
}

GpsGeofenceLayer::GpsGeofenceLayer()
  : hard_margin_m_(1.0)
  , projected_origin_lat_(std::numeric_limits<double>::quiet_NaN())
  , projected_origin_lon_(std::numeric_limits<double>::quiet_NaN())
  , projection_ready_(false)
  , waiting_for_origin_logged_(false)
  , dirty_bounds_pending_(false)
  , dirty_min_x_(std::numeric_limits<double>::infinity())
  , dirty_min_y_(std::numeric_limits<double>::infinity())
  , dirty_max_x_(-std::numeric_limits<double>::infinity())
  , dirty_max_y_(-std::numeric_limits<double>::infinity())
{
}

void GpsGeofenceLayer::onInitialize()
{
  ros::NodeHandle private_nh("~/" + name_);
  private_nh.param("enabled", enabled_, false);
  private_nh.param("hard_margin_m", hard_margin_m_, 1.0);
  if (!std::isfinite(hard_margin_m_) || hard_margin_m_ < 0.0)
  {
    throw std::runtime_error(name_ + ": hard_margin_m must be finite and non-negative");
  }

  loadRegions(private_nh);
  ros::NodeHandle node;
  reload_subscriber_ =
      node.subscribe("/gps/geofence/reload", 2, &GpsGeofenceLayer::reloadCallback, this);
  current_ = true;
  ROS_INFO_STREAM(name_ << ": loaded " << regions_.size()
                        << " enabled GPS keepout region(s), hard margin="
                        << hard_margin_m_ << " m, enabled=" << (enabled_ ? "true" : "false"));
}

void GpsGeofenceLayer::loadRegions(const ros::NodeHandle& private_nh)
{
  regions_.clear();
  XmlRpc::XmlRpcValue values;
  if (!private_nh.getParam("regions", values))
  {
    return;
  }
  if (values.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    throw std::runtime_error(name_ + ": regions must be a YAML list");
  }

  for (int region_index = 0; region_index < values.size(); ++region_index)
  {
    XmlRpc::XmlRpcValue& value = values[region_index];
    if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      throw std::runtime_error(name_ + ": each region must be a YAML mapping");
    }
    const bool region_enabled = !value.hasMember("enabled") || static_cast<bool>(value["enabled"]);
    if (!region_enabled)
    {
      continue;
    }
    if (!value.hasMember("name") || value["name"].getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      throw std::runtime_error(name_ + ": each enabled region requires a string name");
    }
    if (!value.hasMember("vertices") || value["vertices"].getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      throw std::runtime_error(name_ + ": enabled region requires a vertices list");
    }

    Region region;
    region.name = static_cast<std::string>(value["name"]);
    XmlRpc::XmlRpcValue& vertices = value["vertices"];
    if (vertices.size() < 3)
    {
      throw std::runtime_error(name_ + ": region " + region.name + " requires at least three vertices");
    }
    for (int vertex_index = 0; vertex_index < vertices.size(); ++vertex_index)
    {
      XmlRpc::XmlRpcValue& vertex = vertices[vertex_index];
      if (vertex.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
          !vertex.hasMember("latitude") || !vertex.hasMember("longitude"))
      {
        throw std::runtime_error(name_ + ": every vertex requires latitude and longitude");
      }
      GpsFencePoint gps;
      gps.latitude = xmlNumber(vertex["latitude"], "latitude");
      gps.longitude = xmlNumber(vertex["longitude"], "longitude");
      if (!std::isfinite(gps.latitude) || !std::isfinite(gps.longitude) ||
          gps.latitude < -90.0 || gps.latitude > 90.0 ||
          gps.longitude < -180.0 || gps.longitude > 180.0)
      {
        throw std::runtime_error(name_ + ": region " + region.name + " has an invalid GPS vertex");
      }
      region.gps_vertices.push_back(gps);
    }
    regions_.push_back(region);
  }
}

void GpsGeofenceLayer::rememberProjectedBounds()
{
  if (!projection_ready_)
  {
    return;
  }
  for (const Region& region : regions_)
  {
    dirty_min_x_ = std::min(dirty_min_x_, region.min_x - hard_margin_m_);
    dirty_min_y_ = std::min(dirty_min_y_, region.min_y - hard_margin_m_);
    dirty_max_x_ = std::max(dirty_max_x_, region.max_x + hard_margin_m_);
    dirty_max_y_ = std::max(dirty_max_y_, region.max_y + hard_margin_m_);
    dirty_bounds_pending_ = true;
  }
}

void GpsGeofenceLayer::reloadCallback(const std_msgs::Empty::ConstPtr&)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ros::NodeHandle private_nh("~/" + name_);
  const std::vector<Region> previous_regions = regions_;
  const double previous_margin = hard_margin_m_;
  rememberProjectedBounds();
  try
  {
    private_nh.param("hard_margin_m", hard_margin_m_, 1.0);
    if (!std::isfinite(hard_margin_m_) || hard_margin_m_ < 0.0)
    {
      throw std::runtime_error(name_ + ": hard_margin_m must be finite and non-negative");
    }
    loadRegions(private_nh);
  }
  catch (const std::exception& error)
  {
    regions_ = previous_regions;
    hard_margin_m_ = previous_margin;
    ROS_ERROR_STREAM(name_ << ": rejected live GPS geofence reload: " << error.what());
    return;
  }

  projection_ready_ = false;
  waiting_for_origin_logged_ = false;
  current_ = regions_.empty();
  ROS_INFO_STREAM(name_ << ": live reload accepted, " << regions_.size()
                        << " enabled GPS keepout region(s), hard margin="
                        << hard_margin_m_ << " m");
}

bool GpsGeofenceLayer::refreshProjection()
{
  if (regions_.empty())
  {
    projection_ready_ = true;
    return true;
  }

  double origin_lat = 0.0;
  double origin_lon = 0.0;
  if (!ros::param::get("/gps/origin_lat", origin_lat) ||
      !ros::param::get("/gps/origin_lon", origin_lon) ||
      !std::isfinite(origin_lat) || !std::isfinite(origin_lon))
  {
    if (!waiting_for_origin_logged_)
    {
      ROS_WARN_STREAM(name_ << ": waiting for /gps/origin_lat and /gps/origin_lon before projecting fences");
      waiting_for_origin_logged_ = true;
    }
    projection_ready_ = false;
    current_ = false;
    return false;
  }

  if (projection_ready_ && std::abs(origin_lat - projected_origin_lat_) <= 1e-12 &&
      std::abs(origin_lon - projected_origin_lon_) <= 1e-12)
  {
    return true;
  }

  const double reference_latitude = origin_lat * M_PI / 180.0;
  for (Region& region : regions_)
  {
    region.vertices.clear();
    region.min_x = std::numeric_limits<double>::infinity();
    region.min_y = std::numeric_limits<double>::infinity();
    region.max_x = -std::numeric_limits<double>::infinity();
    region.max_y = -std::numeric_limits<double>::infinity();
    for (const GpsFencePoint& gps : region.gps_vertices)
    {
      FencePoint point;
      point.x = kEarthRadiusM * ((gps.longitude - origin_lon) * M_PI / 180.0) *
                std::cos(reference_latitude);
      point.y = kEarthRadiusM * ((gps.latitude - origin_lat) * M_PI / 180.0);
      region.vertices.push_back(point);
      region.min_x = std::min(region.min_x, point.x);
      region.min_y = std::min(region.min_y, point.y);
      region.max_x = std::max(region.max_x, point.x);
      region.max_y = std::max(region.max_y, point.y);
    }
    if (std::abs(signedArea(region.vertices)) < 0.01)
    {
      ROS_FATAL_STREAM(name_ << ": region " << region.name << " is degenerate after GPS projection");
      throw std::runtime_error(name_ + ": degenerate projected geofence " + region.name);
    }
  }

  projected_origin_lat_ = origin_lat;
  projected_origin_lon_ = origin_lon;
  projection_ready_ = true;
  waiting_for_origin_logged_ = false;
  current_ = true;
  ROS_INFO_STREAM(name_ << ": projected GPS keepout regions using origin lat="
                        << origin_lat << " lon=" << origin_lon);
  return true;
}

void GpsGeofenceLayer::updateBounds(double, double, double,
                                    double* min_x, double* min_y, double* max_x, double* max_y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_)
  {
    return;
  }
  if (dirty_bounds_pending_)
  {
    *min_x = std::min(*min_x, dirty_min_x_);
    *min_y = std::min(*min_y, dirty_min_y_);
    *max_x = std::max(*max_x, dirty_max_x_);
    *max_y = std::max(*max_y, dirty_max_y_);
  }
  if (!refreshProjection())
  {
    return;
  }
  for (const Region& region : regions_)
  {
    *min_x = std::min(*min_x, region.min_x - hard_margin_m_);
    *min_y = std::min(*min_y, region.min_y - hard_margin_m_);
    *max_x = std::max(*max_x, region.max_x + hard_margin_m_);
    *max_y = std::max(*max_y, region.max_y + hard_margin_m_);
  }
}

bool GpsGeofenceLayer::cellIsForbidden(double world_x, double world_y) const
{
  const FencePoint point{world_x, world_y};
  for (const Region& region : regions_)
  {
    if (world_x < region.min_x - hard_margin_m_ || world_x > region.max_x + hard_margin_m_ ||
        world_y < region.min_y - hard_margin_m_ || world_y > region.max_y + hard_margin_m_)
    {
      continue;
    }
    if (pointInKeepout(point, region.vertices, hard_margin_m_))
    {
      return true;
    }
  }
  return false;
}

void GpsGeofenceLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
                                   int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_)
  {
    return;
  }
  if (projection_ready_ && !regions_.empty())
  {
    for (int my = min_j; my < max_j; ++my)
    {
      for (int mx = min_i; mx < max_i; ++mx)
      {
        double world_x = 0.0;
        double world_y = 0.0;
        master_grid.mapToWorld(static_cast<unsigned int>(mx), static_cast<unsigned int>(my), world_x, world_y);
        if (cellIsForbidden(world_x, world_y))
        {
          master_grid.setCost(static_cast<unsigned int>(mx), static_cast<unsigned int>(my),
                              costmap_2d::LETHAL_OBSTACLE);
        }
      }
    }
  }
  dirty_bounds_pending_ = false;
  dirty_min_x_ = std::numeric_limits<double>::infinity();
  dirty_min_y_ = std::numeric_limits<double>::infinity();
  dirty_max_x_ = -std::numeric_limits<double>::infinity();
  dirty_max_y_ = -std::numeric_limits<double>::infinity();
  current_ = true;
}

void GpsGeofenceLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  projection_ready_ = false;
  current_ = regions_.empty();
}

}  // namespace robot_bringup

PLUGINLIB_EXPORT_CLASS(robot_bringup::GpsGeofenceLayer, costmap_2d::Layer)
