#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/String.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace
{
struct VoxelKey
{
  int x;
  int y;
  int z;

  bool operator==(const VoxelKey& other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey& key) const
  {
    const std::size_t h1 = std::hash<int>()(key.x);
    const std::size_t h2 = std::hash<int>()(key.y);
    const std::size_t h3 = std::hash<int>()(key.z);
    return h1 ^ (h2 << 1U) ^ (h3 << 7U);
  }
};

struct Voxel
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double intensity = 0.0;
  std::uint32_t point_count = 0;
  std::uint32_t frame_observations = 0;
  std::uint64_t last_cloud_id = 0;
};

struct PcdPoint
{
  float x;
  float y;
  float z;
  float intensity;
  std::uint32_t observations;
};
static_assert(sizeof(PcdPoint) == 20, "PCD point layout must match the header");

struct GridKey
{
  int x;
  int y;

  bool operator==(const GridKey& other) const
  {
    return x == other.x && y == other.y;
  }
};

struct GridKeyHash
{
  std::size_t operator()(const GridKey& key) const
  {
    const std::size_t h1 = std::hash<int>()(key.x);
    const std::size_t h2 = std::hash<int>()(key.y);
    return h1 ^ (h2 << 1U);
  }
};
}  // namespace

class VoxelCloudMapper
{
public:
  VoxelCloudMapper() : private_nh_("~")
  {
    private_nh_.param<std::string>("input_topic", input_topic_, "/cloud_registered");
    private_nh_.param<std::string>("odom_topic", odom_topic_, "/Odometry");
    private_nh_.param<std::string>("output_file", output_file_, std::string());
    private_nh_.param<std::string>("slice_observations_file", slice_observations_file_,
                                   std::string());
    private_nh_.param<double>("voxel_size", voxel_size_, 0.10);
    private_nh_.param<int>("min_frame_observations", min_frame_observations_, 3);
    private_nh_.param<double>("max_point_range", max_point_range_, 20.0);
    private_nh_.param<double>("max_pose_age", max_pose_age_, 0.20);
    private_nh_.param<double>("slice_center_z", slice_center_z_, -0.756);
    private_nh_.param<double>("slice_half_width", slice_half_width_, 0.10);
    private_nh_.param<double>("slice_resolution", slice_resolution_, 0.10);
    private_nh_.param<int>("slice_min_frame_observations",
                           slice_min_frame_observations_, 20);
    private_nh_.param<int>("queue_size", queue_size_, 2);
    if (output_file_.empty())
      throw std::runtime_error("~output_file must not be empty");
    if (!(voxel_size_ > 0.0) || min_frame_observations_ < 1 ||
        !(max_point_range_ > 0.0) || !(max_pose_age_ > 0.0) ||
        !(slice_half_width_ > 0.0) || !(slice_resolution_ > 0.0) ||
        slice_min_frame_observations_ < 1 || queue_size_ < 1)
      throw std::runtime_error("invalid voxel mapper parameters");

    status_publisher_ = nh_.advertise<std_msgs::String>("/static_mapping/pointcloud_status", 1, true);
    odom_subscriber_ = nh_.subscribe(odom_topic_, 20, &VoxelCloudMapper::odomCallback, this);
    subscriber_ = nh_.subscribe(input_topic_, queue_size_, &VoxelCloudMapper::cloudCallback, this);
    publishStatus("RECORDING");
    ROS_INFO_STREAM("voxel_cloud_mapper: " << input_topic_ << " -> " << output_file_
                    << " (voxel " << voxel_size_ << " m, >= "
                    << min_frame_observations_ << " frames, range <= "
                    << max_point_range_ << " m; persistent slice >= "
                    << slice_min_frame_observations_ << " frames)");
  }

  ~VoxelCloudMapper()
  {
    save();
  }

private:
  static std::string normalizedFrame(const std::string& frame)
  {
    return !frame.empty() && frame.front() == '/' ? frame.substr(1) : frame;
  }

  void publishStatus(const std::string& state)
  {
    std_msgs::String message;
    message.data = state;
    status_publisher_.publish(message);
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& odometry)
  {
    odom_frame_id_ = normalizedFrame(odometry->header.frame_id);
    odom_stamp_ = odometry->header.stamp;
    odom_x_ = odometry->pose.pose.position.x;
    odom_y_ = odometry->pose.pose.position.y;
    odom_z_ = odometry->pose.pose.position.z;
    have_odometry_ = std::isfinite(odom_x_) && std::isfinite(odom_y_) &&
                     std::isfinite(odom_z_);
  }

  bool hasField(const sensor_msgs::PointCloud2& cloud, const std::string& field) const
  {
    for (const auto& candidate : cloud.fields)
      if (candidate.name == field)
        return true;
    return false;
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
    if (!hasField(*cloud, "x") || !hasField(*cloud, "y") || !hasField(*cloud, "z"))
    {
      ROS_ERROR_THROTTLE(5.0, "voxel_cloud_mapper: cloud lacks x/y/z fields");
      return;
    }
    const std::string cloud_frame = normalizedFrame(cloud->header.frame_id);
    if (frame_id_.empty())
      frame_id_ = cloud_frame;
    else if (frame_id_ != cloud_frame)
    {
      ++rejected_clouds_;
      ROS_ERROR_THROTTLE(5.0, "voxel_cloud_mapper: input frame changed; cloud rejected");
      return;
    }
    if (!have_odometry_ || odom_frame_id_ != cloud_frame)
    {
      ++rejected_clouds_;
      ROS_WARN_THROTTLE(5.0, "voxel_cloud_mapper: matching odometry is unavailable");
      return;
    }
    const double pose_age = std::fabs((cloud->header.stamp - odom_stamp_).toSec());
    if (pose_age > max_pose_age_)
    {
      ++rejected_clouds_;
      ROS_WARN_THROTTLE(5.0, "voxel_cloud_mapper: odometry age exceeds limit");
      return;
    }

    const bool has_intensity = hasField(*cloud, "intensity");
    sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> intensity;
    if (has_intensity)
      intensity.reset(new sensor_msgs::PointCloud2ConstIterator<float>(*cloud, "intensity"));

    const std::uint64_t cloud_id = clouds_ + 1;
    const double maximum_range_squared = max_point_range_ * max_point_range_;
    std::unordered_set<GridKey, GridKeyHash> slice_cells_seen;
    for (; x != x.end(); ++x, ++y, ++z)
    {
      const float px = *x;
      const float py = *y;
      const float pz = *z;
      const float value = intensity ? **intensity : 0.0f;
      if (intensity)
        ++(*intensity);
      if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz))
        continue;
      const double dx = static_cast<double>(px) - odom_x_;
      const double dy = static_cast<double>(py) - odom_y_;
      const double dz = static_cast<double>(pz) - odom_z_;
      if (dx * dx + dy * dy + dz * dz > maximum_range_squared)
      {
        ++range_rejected_points_;
        continue;
      }
      const VoxelKey key{static_cast<int>(std::floor(px / voxel_size_)),
                         static_cast<int>(std::floor(py / voxel_size_)),
                         static_cast<int>(std::floor(pz / voxel_size_))};
      Voxel& voxel = voxels_[key];
      voxel.x += px;
      voxel.y += py;
      voxel.z += pz;
      voxel.intensity += std::isfinite(value) ? value : 0.0f;
      ++voxel.point_count;
      if (voxel.last_cloud_id != cloud_id)
      {
        voxel.last_cloud_id = cloud_id;
        ++voxel.frame_observations;
      }
      if (slice_center_z_ - slice_half_width_ <= pz &&
          pz <= slice_center_z_ + slice_half_width_)
      {
        slice_cells_seen.insert(
            GridKey{static_cast<int>(std::floor(px / slice_resolution_)),
                    static_cast<int>(std::floor(py / slice_resolution_))});
      }
      ++input_points_;
    }
    for (const GridKey& cell : slice_cells_seen)
      ++slice_frame_observations_[cell];
    ++clouds_;
    ROS_INFO_STREAM_THROTTLE(10.0, "voxel_cloud_mapper: " << clouds_ << " clouds, "
                                                           << voxels_.size() << " voxels");
  }

  bool saveSliceObservations() const
  {
    if (slice_observations_file_.empty())
      return true;
    const std::string temporary = slice_observations_file_ + ".tmp";
    std::ofstream stream(temporary.c_str(), std::ios::trunc);
    if (!stream)
      return false;
    std::size_t accepted = 0;
    for (const auto& entry : slice_frame_observations_)
      if (entry.second >= static_cast<std::uint32_t>(slice_min_frame_observations_))
        ++accepted;
    stream << std::setprecision(12)
           << "schema_version: 1\n"
           << "frame_id: " << frame_id_ << "\n"
           << "resolution_m: " << slice_resolution_ << "\n"
           << "slice_center_z_m: " << slice_center_z_ << "\n"
           << "slice_half_width_m: " << slice_half_width_ << "\n"
           << "min_frame_observations: " << slice_min_frame_observations_ << "\n"
           << "observed_clouds: " << clouds_ << "\n"
           << "candidate_cells: " << slice_frame_observations_.size() << "\n"
           << "accepted_cells: " << accepted << "\n"
           << "cells:\n";
    for (const auto& entry : slice_frame_observations_)
    {
      if (entry.second < static_cast<std::uint32_t>(slice_min_frame_observations_))
        continue;
      stream << "  - [" << entry.first.x << ", " << entry.first.y << ", "
             << entry.second << "]\n";
    }
    stream.close();
    if (!stream || std::rename(temporary.c_str(), slice_observations_file_.c_str()) != 0)
    {
      std::remove(temporary.c_str());
      return false;
    }
    return true;
  }

  void save()
  {
    if (saved_)
      return;
    saved_ = true;
    publishStatus("SAVING");
    std::vector<PcdPoint> points;
    points.reserve(voxels_.size());
    for (const auto& entry : voxels_)
    {
      const Voxel& voxel = entry.second;
      if (voxel.frame_observations <
          static_cast<std::uint32_t>(min_frame_observations_))
        continue;
      const double divisor = static_cast<double>(voxel.point_count);
      points.push_back(PcdPoint{static_cast<float>(voxel.x / divisor),
                               static_cast<float>(voxel.y / divisor),
                               static_cast<float>(voxel.z / divisor),
                               static_cast<float>(voxel.intensity / divisor),
                               voxel.frame_observations});
    }
    if (points.empty())
    {
      publishStatus("FAILED_EMPTY");
      ROS_ERROR("voxel_cloud_mapper: no points collected; map not written");
      return;
    }

    const std::string temporary = output_file_ + ".tmp";
    std::ofstream stream(temporary.c_str(), std::ios::binary | std::ios::trunc);
    if (!stream)
    {
      publishStatus("FAILED_WRITE");
      ROS_ERROR_STREAM("voxel_cloud_mapper: cannot open " << temporary);
      return;
    }
    stream << "# .PCD v0.7 - Point Cloud Data file format\n"
           << "VERSION 0.7\n"
           << "FIELDS x y z intensity observations\n"
           << "SIZE 4 4 4 4 4\n"
           << "TYPE F F F F U\n"
           << "COUNT 1 1 1 1 1\n"
           << "WIDTH " << points.size() << "\n"
           << "HEIGHT 1\n"
           << "VIEWPOINT 0 0 0 1 0 0 0\n"
           << "POINTS " << points.size() << "\n"
           << "DATA binary\n";
    stream.write(reinterpret_cast<const char*>(points.data()),
                 static_cast<std::streamsize>(points.size() * sizeof(PcdPoint)));
    stream.close();
    if (!stream || std::rename(temporary.c_str(), output_file_.c_str()) != 0)
    {
      std::remove(temporary.c_str());
      publishStatus("FAILED_WRITE");
      ROS_ERROR_STREAM("voxel_cloud_mapper: failed to finalize " << output_file_);
      return;
    }
    if (!saveSliceObservations())
    {
      publishStatus("FAILED_WRITE");
      ROS_ERROR_STREAM("voxel_cloud_mapper: failed to save persistent slice "
                       << slice_observations_file_);
      return;
    }
    publishStatus("COMPLETE");
    ROS_INFO_STREAM("voxel_cloud_mapper: saved " << points.size() << " points to "
                    << output_file_ << "; range-rejected " << range_rejected_points_
                    << " points");
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber subscriber_;
  ros::Subscriber odom_subscriber_;
  ros::Publisher status_publisher_;
  std::string input_topic_;
  std::string odom_topic_;
  std::string output_file_;
  std::string slice_observations_file_;
  std::string frame_id_;
  std::string odom_frame_id_;
  double voxel_size_ = 0.10;
  int min_frame_observations_ = 3;
  double max_point_range_ = 20.0;
  double max_pose_age_ = 0.20;
  double slice_center_z_ = -0.756;
  double slice_half_width_ = 0.10;
  double slice_resolution_ = 0.10;
  int slice_min_frame_observations_ = 20;
  int queue_size_ = 2;
  std::unordered_map<VoxelKey, Voxel, VoxelKeyHash> voxels_;
  std::unordered_map<GridKey, std::uint32_t, GridKeyHash> slice_frame_observations_;
  std::uint64_t clouds_ = 0;
  std::uint64_t rejected_clouds_ = 0;
  std::uint64_t input_points_ = 0;
  std::uint64_t range_rejected_points_ = 0;
  ros::Time odom_stamp_;
  double odom_x_ = 0.0;
  double odom_y_ = 0.0;
  double odom_z_ = 0.0;
  bool have_odometry_ = false;
  bool saved_ = false;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "voxel_cloud_mapper");
  try
  {
    VoxelCloudMapper mapper;
    ros::spin();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL_STREAM("voxel_cloud_mapper: " << error.what());
    return 2;
  }
  return 0;
}
