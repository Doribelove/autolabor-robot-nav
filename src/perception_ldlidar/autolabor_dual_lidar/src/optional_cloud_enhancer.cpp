#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>

namespace
{

bool findFloat32Field(const sensor_msgs::PointCloud2& cloud, const std::string& name, uint32_t* offset)
{
  for (const auto& field : cloud.fields)
  {
    if (field.name == name && field.datatype == sensor_msgs::PointField::FLOAT32 && field.count >= 1)
    {
      *offset = field.offset;
      return true;
    }
  }
  return false;
}

std::string normalizedFrame(std::string frame)
{
  while (!frame.empty() && frame.front() == '/')
  {
    frame.erase(frame.begin());
  }
  return frame;
}

void writeFloat32(std::vector<uint8_t>* data, const size_t offset, const float value)
{
  std::memcpy(data->data() + offset, &value, sizeof(value));
}

class OptionalCloudEnhancer
{
public:
  OptionalCloudEnhancer() : private_nh_("~"), transform_listener_(transform_buffer_)
  {
    private_nh_.param<std::string>("mid_cloud_topic", mid_cloud_topic_, "/cloud_registered_body");
    private_nh_.param<std::string>("lidar_scan_topic", lidar_scan_topic_, "/dual_lidar/scan");
    private_nh_.param<std::string>("output_topic", output_topic_, "/cloud_registered_body_enhanced");
    private_nh_.param("lidar_z", lidar_z_, 0.0);
    private_nh_.param("scan_timeout", scan_timeout_, 0.35);
    const int queue_size = private_nh_.param("queue_size", 5);

    if (mid_cloud_topic_ == output_topic_)
    {
      throw std::runtime_error("mid_cloud_topic and output_topic must be different");
    }
    if (scan_timeout_ <= 0.0)
    {
      throw std::runtime_error("scan_timeout must be positive");
    }

    cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, queue_size);
    active_publisher_ = nh_.advertise<std_msgs::Bool>("/dual_lidar/enhancement_active", 1, true);
    scan_subscriber_ = nh_.subscribe(lidar_scan_topic_, queue_size, &OptionalCloudEnhancer::scanCallback, this);
    cloud_subscriber_ = nh_.subscribe(mid_cloud_topic_, queue_size, &OptionalCloudEnhancer::cloudCallback, this);

    publishActive(false);
    ROS_INFO_STREAM("optional_cloud_enhancer: " << mid_cloud_topic_ << " + " << lidar_scan_topic_ << " -> "
                                                << output_topic_ << ", lidar_z=" << lidar_z_
                                                << " m, timeout=" << scan_timeout_ << " s");
  }

private:
  void scanCallback(const sensor_msgs::LaserScanConstPtr& scan)
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    latest_scan_ = scan;
    latest_scan_wall_time_ = ros::WallTime::now();
  }

  void publishActive(const bool active)
  {
    std_msgs::Bool status;
    status.data = active;
    active_publisher_.publish(status);
    if (!have_previous_active_state_ || active != previous_active_state_)
    {
      if (active)
      {
        ROS_INFO("optional_cloud_enhancer: dual LD19 scan is live; enhanced points are enabled");
      }
      else
      {
        ROS_WARN("optional_cloud_enhancer: dual LD19 scan is absent or stale; publishing MID360 cloud unchanged");
      }
      previous_active_state_ = active;
      have_previous_active_state_ = true;
    }
  }

  sensor_msgs::LaserScanConstPtr freshScan() const
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    if (!latest_scan_)
    {
      return sensor_msgs::LaserScanConstPtr();
    }
    if ((ros::WallTime::now() - latest_scan_wall_time_).toSec() > scan_timeout_)
    {
      return sensor_msgs::LaserScanConstPtr();
    }
    return latest_scan_;
  }

  void publishPassthrough(const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
    cloud_publisher_.publish(*cloud);
    publishActive(false);
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
    const auto scan = freshScan();
    if (!scan)
    {
      publishPassthrough(cloud);
      return;
    }

    const bool frames_match = normalizedFrame(scan->header.frame_id) == normalizedFrame(cloud->header.frame_id);
    geometry_msgs::TransformStamped scan_to_cloud;
    if (!frames_match)
    {
      try
      {
        // The relationship is static on this vehicle. Latest avoids rejecting a
        // valid scan solely because the two sensor timestamps differ slightly.
        scan_to_cloud = transform_buffer_.lookupTransform(cloud->header.frame_id, scan->header.frame_id,
                                                          ros::Time(0), ros::Duration(0.05));
      }
      catch (const tf2::TransformException& error)
      {
        ROS_WARN_THROTTLE(
            5.0,
            "optional_cloud_enhancer: cannot transform %s -> %s (%s); publishing MID360 cloud unchanged",
            scan->header.frame_id.c_str(), cloud->header.frame_id.c_str(), error.what());
        publishPassthrough(cloud);
        return;
      }
    }

    uint32_t x_offset = 0;
    uint32_t y_offset = 0;
    uint32_t z_offset = 0;
    uint32_t intensity_offset = 0;
    const bool has_xyz = findFloat32Field(*cloud, "x", &x_offset) && findFloat32Field(*cloud, "y", &y_offset) &&
                         findFloat32Field(*cloud, "z", &z_offset);
    const bool has_intensity = findFloat32Field(*cloud, "intensity", &intensity_offset);
    const bool xyz_layout_valid = x_offset + sizeof(float) <= cloud->point_step &&
                                  y_offset + sizeof(float) <= cloud->point_step &&
                                  z_offset + sizeof(float) <= cloud->point_step;
    const bool intensity_layout_valid = !has_intensity || intensity_offset + sizeof(float) <= cloud->point_step;
    if (!has_xyz || !xyz_layout_valid || !intensity_layout_valid || cloud->point_step == 0 || cloud->is_bigendian)
    {
      ROS_WARN_THROTTLE(5.0,
                        "optional_cloud_enhancer: MID360 cloud must use little-endian float32 x/y/z fields; "
                        "publishing it unchanged");
      publishPassthrough(cloud);
      return;
    }

    sensor_msgs::PointCloud2 output = *cloud;
    output.height = 1;
    output.width = 0;
    output.row_step = 0;
    output.data.clear();

    const size_t source_point_count = static_cast<size_t>(cloud->height) * cloud->width;
    output.data.reserve(cloud->data.size() + scan->ranges.size() * cloud->point_step);
    for (uint32_t row = 0; row < cloud->height; ++row)
    {
      const size_t row_offset = static_cast<size_t>(row) * cloud->row_step;
      for (uint32_t col = 0; col < cloud->width; ++col)
      {
        const size_t point_offset = row_offset + static_cast<size_t>(col) * cloud->point_step;
        if (point_offset + cloud->point_step > cloud->data.size())
        {
          ROS_WARN_THROTTLE(5.0, "optional_cloud_enhancer: malformed MID360 PointCloud2 layout");
          publishPassthrough(cloud);
          return;
        }
        output.data.insert(output.data.end(), cloud->data.begin() + point_offset,
                           cloud->data.begin() + point_offset + cloud->point_step);
      }
    }

    size_t added_points = 0;
    for (size_t index = 0; index < scan->ranges.size(); ++index)
    {
      const float range = scan->ranges[index];
      if (!std::isfinite(range) || range <= 0.0f || range < scan->range_min || range > scan->range_max)
      {
        continue;
      }

      const float angle = scan->angle_min + static_cast<float>(index) * scan->angle_increment;
      float x = range * std::cos(angle);
      float y = range * std::sin(angle);
      float z = static_cast<float>(lidar_z_);
      if (!frames_match)
      {
        const auto& rotation = scan_to_cloud.transform.rotation;
        const auto& translation = scan_to_cloud.transform.translation;
        const tf2::Quaternion quaternion(rotation.x, rotation.y, rotation.z, rotation.w);
        const tf2::Vector3 transformed =
            tf2::Matrix3x3(quaternion) * tf2::Vector3(x, y, z) +
            tf2::Vector3(translation.x, translation.y, translation.z);
        x = static_cast<float>(transformed.x());
        y = static_cast<float>(transformed.y());
        z = static_cast<float>(transformed.z());
      }
      const size_t point_offset = output.data.size();
      output.data.resize(point_offset + cloud->point_step, 0);
      writeFloat32(&output.data, point_offset + x_offset, x);
      writeFloat32(&output.data, point_offset + y_offset, y);
      writeFloat32(&output.data, point_offset + z_offset, z);
      if (has_intensity)
      {
        const float intensity = index < scan->intensities.size() ? scan->intensities[index] : 0.0f;
        writeFloat32(&output.data, point_offset + intensity_offset, intensity);
      }
      ++added_points;
    }

    output.width = static_cast<uint32_t>(source_point_count + added_points);
    output.row_step = output.point_step * output.width;
    cloud_publisher_.publish(output);
    publishActive(added_points > 0);
    ROS_DEBUG_THROTTLE(2.0, "optional_cloud_enhancer: appended %zu LD19 points", added_points);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber cloud_subscriber_;
  ros::Subscriber scan_subscriber_;
  ros::Publisher cloud_publisher_;
  ros::Publisher active_publisher_;
  tf2_ros::Buffer transform_buffer_;
  tf2_ros::TransformListener transform_listener_;

  std::string mid_cloud_topic_;
  std::string lidar_scan_topic_;
  std::string output_topic_;
  double lidar_z_ = 0.0;
  double scan_timeout_ = 0.35;

  mutable std::mutex scan_mutex_;
  sensor_msgs::LaserScanConstPtr latest_scan_;
  ros::WallTime latest_scan_wall_time_;
  bool have_previous_active_state_ = false;
  bool previous_active_state_ = false;
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "optional_cloud_enhancer");
  try
  {
    OptionalCloudEnhancer enhancer;
    ros::spin();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL("optional_cloud_enhancer configuration error: %s", error.what());
    return 2;
  }
  return 0;
}
