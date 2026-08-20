#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/String.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
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
  std::uint32_t count = 0;
};

struct PcdPoint
{
  float x;
  float y;
  float z;
  float intensity;
};
}  // namespace

class VoxelCloudMapper
{
public:
  VoxelCloudMapper() : private_nh_("~")
  {
    private_nh_.param<std::string>("input_topic", input_topic_, "/cloud_registered");
    private_nh_.param<std::string>("output_file", output_file_, std::string());
    private_nh_.param<double>("voxel_size", voxel_size_, 0.10);
    private_nh_.param<int>("min_observations", min_observations_, 1);
    private_nh_.param<int>("queue_size", queue_size_, 2);
    if (output_file_.empty())
      throw std::runtime_error("~output_file must not be empty");
    if (!(voxel_size_ > 0.0) || min_observations_ < 1 || queue_size_ < 1)
      throw std::runtime_error("invalid voxel mapper parameters");

    status_publisher_ = nh_.advertise<std_msgs::String>("/static_mapping/pointcloud_status", 1, true);
    subscriber_ = nh_.subscribe(input_topic_, queue_size_, &VoxelCloudMapper::cloudCallback, this);
    publishStatus("RECORDING");
    ROS_INFO_STREAM("voxel_cloud_mapper: " << input_topic_ << " -> " << output_file_
                                            << " (voxel " << voxel_size_ << " m)");
  }

  ~VoxelCloudMapper()
  {
    save();
  }

private:
  void publishStatus(const std::string& state)
  {
    std_msgs::String message;
    message.data = state;
    status_publisher_.publish(message);
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
    if (frame_id_.empty())
      frame_id_ = cloud->header.frame_id;
    else if (frame_id_ != cloud->header.frame_id)
    {
      ++rejected_clouds_;
      ROS_ERROR_THROTTLE(5.0, "voxel_cloud_mapper: input frame changed; cloud rejected");
      return;
    }

    const bool has_intensity = hasField(*cloud, "intensity");
    sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> intensity;
    if (has_intensity)
      intensity.reset(new sensor_msgs::PointCloud2ConstIterator<float>(*cloud, "intensity"));

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
      const VoxelKey key{static_cast<int>(std::floor(px / voxel_size_)),
                         static_cast<int>(std::floor(py / voxel_size_)),
                         static_cast<int>(std::floor(pz / voxel_size_))};
      Voxel& voxel = voxels_[key];
      voxel.x += px;
      voxel.y += py;
      voxel.z += pz;
      voxel.intensity += std::isfinite(value) ? value : 0.0f;
      ++voxel.count;
      ++input_points_;
    }
    ++clouds_;
    ROS_INFO_STREAM_THROTTLE(10.0, "voxel_cloud_mapper: " << clouds_ << " clouds, "
                                                           << voxels_.size() << " voxels");
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
      if (voxel.count < static_cast<std::uint32_t>(min_observations_))
        continue;
      const double divisor = static_cast<double>(voxel.count);
      points.push_back(PcdPoint{static_cast<float>(voxel.x / divisor),
                               static_cast<float>(voxel.y / divisor),
                               static_cast<float>(voxel.z / divisor),
                               static_cast<float>(voxel.intensity / divisor)});
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
           << "FIELDS x y z intensity\n"
           << "SIZE 4 4 4 4\n"
           << "TYPE F F F F\n"
           << "COUNT 1 1 1 1\n"
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
    publishStatus("COMPLETE");
    ROS_INFO_STREAM("voxel_cloud_mapper: saved " << points.size() << " points to "
                                                  << output_file_);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber subscriber_;
  ros::Publisher status_publisher_;
  std::string input_topic_;
  std::string output_file_;
  std::string frame_id_;
  double voxel_size_ = 0.10;
  int min_observations_ = 1;
  int queue_size_ = 2;
  std::unordered_map<VoxelKey, Voxel, VoxelKeyHash> voxels_;
  std::uint64_t clouds_ = 0;
  std::uint64_t rejected_clouds_ = 0;
  std::uint64_t input_points_ = 0;
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
