#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include <livox_ros_driver2/CustomMsg.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace
{

struct OutputPoint
{
  float x;
  float y;
  float z;
  float intensity;
};

std::array<double, 9> rotationFromRpy(const double roll, const double pitch, const double yaw)
{
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  // Rz(yaw) * Ry(pitch) * Rx(roll): sensor coordinates -> target coordinates.
  return {{cy * cp,
           cy * sp * sr - sy * cr,
           cy * sp * cr + sy * sr,
           sy * cp,
           sy * sp * sr + cy * cr,
           sy * sp * cr - cy * sr,
           -sp,
           cp * sr,
           cp * cr}};
}

bool isUsableLivoxTag(const uint8_t tag)
{
  // Match the validity rule used by FAST-LIO's Livox preprocessor. Bits 4-5
  // encode the spatial confidence/noise class; 0x00 and 0x10 are usable.
  const uint8_t spatial_status = tag & 0x30;
  return spatial_status == 0x00 || spatial_status == 0x10;
}

class LivoxCustomToPointCloud
{
public:
  LivoxCustomToPointCloud() : private_nh_("~")
  {
    private_nh_.param<std::string>("input_topic", input_topic_, "/livox/lidar");
    private_nh_.param<std::string>("output_topic", output_topic_, "/mid360/points_avoidance");
    private_nh_.param<std::string>("target_frame", target_frame_, "base_link");

    private_nh_.param("sensor_x", sensor_x_, 0.20);
    private_nh_.param("sensor_y", sensor_y_, 0.0);
    private_nh_.param("sensor_z", sensor_z_, 0.9);
    private_nh_.param("sensor_roll", sensor_roll_, 0.0);
    private_nh_.param("sensor_pitch", sensor_pitch_, 0.0);
    private_nh_.param("sensor_yaw", sensor_yaw_, 0.0);
    private_nh_.param("min_range", min_range_, 0.5);
    private_nh_.param("max_range", max_range_, 100.0);
    private_nh_.param("filter_invalid_tags", filter_invalid_tags_, true);
    private_nh_.param("line_count", line_count_, 4);
    private_nh_.param("point_decimation", point_decimation_, 1);
    private_nh_.param("crop_enabled", crop_enabled_, false);
    private_nh_.param("crop_min_x", crop_min_x_, -0.75);
    private_nh_.param("crop_max_x", crop_max_x_, 0.75);
    private_nh_.param("crop_min_y", crop_min_y_, -0.50);
    private_nh_.param("crop_max_y", crop_max_y_, 0.50);
    const int queue_size = private_nh_.param("queue_size", 5);

    if (min_range_ < 0.0 || max_range_ <= min_range_)
    {
      throw std::runtime_error("livox_custom_to_pointcloud: invalid min_range/max_range");
    }
    if (line_count_ < 0)
    {
      throw std::runtime_error("livox_custom_to_pointcloud: line_count must be non-negative");
    }
    if (point_decimation_ < 1)
    {
      throw std::runtime_error("livox_custom_to_pointcloud: point_decimation must be positive");
    }
    if (crop_enabled_ && (crop_max_x_ <= crop_min_x_ || crop_max_y_ <= crop_min_y_))
    {
      throw std::runtime_error("livox_custom_to_pointcloud: invalid crop rectangle");
    }

    rotation_ = rotationFromRpy(sensor_roll_, sensor_pitch_, sensor_yaw_);
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, queue_size);
    subscriber_ = nh_.subscribe(input_topic_, queue_size, &LivoxCustomToPointCloud::cloudCallback, this,
                                ros::TransportHints().tcpNoDelay());

    ROS_INFO_STREAM("livox_custom_to_pointcloud: " << input_topic_ << " -> " << output_topic_
                                                    << " in " << target_frame_
                                                    << ", sensor xyz/rpy=" << sensor_x_ << ","
                                                    << sensor_y_ << "," << sensor_z_ << "/"
                                                    << sensor_roll_ << "," << sensor_pitch_ << ","
                                                    << sensor_yaw_
                                                    << ", exclusion crop="
                                                    << (crop_enabled_ ? "enabled" : "disabled")
                                                    << " x=[" << crop_min_x_ << "," << crop_max_x_
                                                    << "] y=[" << crop_min_y_ << "," << crop_max_y_
                                                    << "]");
  }

private:
  void cloudCallback(const livox_ros_driver2::CustomMsgConstPtr& msg)
  {
    const std::size_t declared_count = static_cast<std::size_t>(msg->point_num);
    const std::size_t input_count = std::min(declared_count, msg->points.size());
    if (declared_count != msg->points.size())
    {
      ROS_WARN_THROTTLE(5.0,
                        "livox_custom_to_pointcloud: point_num (%u) differs from points.size() (%zu)",
                        msg->point_num, msg->points.size());
    }

    const double min_range_sq = min_range_ * min_range_;
    const double max_range_sq = max_range_ * max_range_;
    std::vector<OutputPoint> points;
    points.reserve(input_count);

    for (std::size_t index = 0; index < input_count; ++index)
    {
      const auto& point = msg->points[index];
      if (index % static_cast<std::size_t>(point_decimation_) != 0)
      {
        continue;
      }
      if (line_count_ > 0 && point.line >= static_cast<uint8_t>(line_count_))
      {
        continue;
      }
      if (filter_invalid_tags_ && !isUsableLivoxTag(point.tag))
      {
        continue;
      }
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
      {
        continue;
      }

      const double range_sq = static_cast<double>(point.x) * point.x +
                              static_cast<double>(point.y) * point.y +
                              static_cast<double>(point.z) * point.z;
      if (range_sq < min_range_sq || range_sq > max_range_sq)
      {
        continue;
      }

      OutputPoint output;
      output.x = static_cast<float>(rotation_[0] * point.x + rotation_[1] * point.y +
                                    rotation_[2] * point.z + sensor_x_);
      output.y = static_cast<float>(rotation_[3] * point.x + rotation_[4] * point.y +
                                    rotation_[5] * point.z + sensor_y_);
      output.z = static_cast<float>(rotation_[6] * point.x + rotation_[7] * point.y +
                                    rotation_[8] * point.z + sensor_z_);
      output.intensity = static_cast<float>(point.reflectivity);
      // Remove the chassis/self-reflection rectangle after applying the
      // calibrated sensor pose.  The rectangle is therefore expressed in
      // target_frame (base_link), not around the offset lidar.  Boundaries
      // are part of the excluded rectangle.
      if (crop_enabled_ &&
          output.x >= crop_min_x_ && output.x <= crop_max_x_ &&
          output.y >= crop_min_y_ && output.y <= crop_max_y_)
      {
        continue;
      }
      points.push_back(output);
    }

    sensor_msgs::PointCloud2 cloud;
    cloud.header = msg->header;
    cloud.header.frame_id = target_frame_;
    cloud.height = 1;
    cloud.is_bigendian = false;
    cloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1,
                                  sensor_msgs::PointField::FLOAT32, "z", 1,
                                  sensor_msgs::PointField::FLOAT32, "intensity", 1,
                                  sensor_msgs::PointField::FLOAT32);
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> x_iterator(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y_iterator(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z_iterator(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity_iterator(cloud, "intensity");
    for (const auto& point : points)
    {
      *x_iterator = point.x;
      *y_iterator = point.y;
      *z_iterator = point.z;
      *intensity_iterator = point.intensity;
      ++x_iterator;
      ++y_iterator;
      ++z_iterator;
      ++intensity_iterator;
    }

    publisher_.publish(cloud);
    ROS_DEBUG_THROTTLE(2.0, "livox_custom_to_pointcloud: published %zu/%zu valid points",
                       points.size(), input_count);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  double sensor_x_;
  double sensor_y_;
  double sensor_z_;
  double sensor_roll_;
  double sensor_pitch_;
  double sensor_yaw_;
  double min_range_;
  double max_range_;
  bool filter_invalid_tags_;
  int line_count_;
  int point_decimation_;
  bool crop_enabled_;
  double crop_min_x_;
  double crop_max_x_;
  double crop_min_y_;
  double crop_max_y_;
  std::array<double, 9> rotation_;
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "livox_custom_to_pointcloud");
  try
  {
    LivoxCustomToPointCloud converter;
    ros::spin();
  }
  catch (const std::exception& exception)
  {
    ROS_FATAL_STREAM(exception.what());
    return 1;
  }
  return 0;
}
