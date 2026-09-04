#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/String.h>

#include <robot_bringup/moving_self_crop.h>

#include <boost/bind/bind.hpp>

#include <algorithm>
#include <array>
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
    private_nh_.param<std::string>("history_topic", history_topic_,
                                   "/static_mapping/history_cloud");
    private_nh_.param<double>("history_publish_period", history_publish_period_, 1.0);
    private_nh_.param<double>("voxel_size", voxel_size_, 0.10);
    private_nh_.param<int>("min_frame_observations", min_frame_observations_, 3);
    private_nh_.param<double>("max_point_range", max_point_range_, 20.0);
    private_nh_.param<double>("slice_center_z", slice_center_z_, -0.4);
    private_nh_.param<double>("slice_half_width", slice_half_width_, 0.20);
    private_nh_.param<double>("slice_resolution", slice_resolution_, 0.10);
    private_nh_.param<int>("slice_min_frame_observations",
                           slice_min_frame_observations_, 20);
    private_nh_.param<bool>("slice_self_crop_enabled", slice_self_crop_enabled_, true);
    private_nh_.param<double>("slice_self_crop_min_x", slice_self_crop_.min_x, -0.75);
    private_nh_.param<double>("slice_self_crop_max_x", slice_self_crop_.max_x, 0.75);
    private_nh_.param<double>("slice_self_crop_min_y", slice_self_crop_.min_y, -0.50);
    private_nh_.param<double>("slice_self_crop_max_y", slice_self_crop_.max_y, 0.50);
    private_nh_.param<double>("slice_sweep_front", slice_sweep_crop_.max_x, 0.62);
    private_nh_.param<double>("slice_sweep_rear", slice_sweep_rear_, 0.62);
    slice_sweep_crop_.min_x = -slice_sweep_rear_;
    private_nh_.param<double>("slice_sweep_half_width", slice_sweep_crop_.max_y, 0.45);
    slice_sweep_crop_.min_y = -slice_sweep_crop_.max_y;
    private_nh_.param<double>("body_to_base_x", body_to_base_x_, -0.211);
    private_nh_.param<double>("body_to_base_y", body_to_base_y_, -0.02329);
    private_nh_.param<double>("body_to_base_z", body_to_base_z_, -0.95588);
    private_nh_.param<double>("slice_sweep_linear_step", slice_sweep_linear_step_, 0.05);
    private_nh_.param<double>("slice_sweep_angular_step",
                              slice_sweep_angular_step_, 0.03490658503988659);
    private_nh_.param<std::string>("odom_child_frame", odom_child_frame_, "body");
    private_nh_.param<int>("queue_size", queue_size_, 20);
    if (output_file_.empty())
      throw std::runtime_error("~output_file must not be empty");
    if (!(voxel_size_ > 0.0) || min_frame_observations_ < 1 ||
        !(max_point_range_ > 0.0) ||
        !(slice_half_width_ > 0.0) || !(slice_resolution_ > 0.0) ||
        slice_min_frame_observations_ < 1 || queue_size_ < 1 ||
        (slice_self_crop_enabled_ &&
         (!slice_self_crop_.valid() || !slice_sweep_crop_.valid() ||
          !(slice_sweep_linear_step_ > 0.0) ||
          !(slice_sweep_angular_step_ > 0.0) ||
          !std::isfinite(body_to_base_x_) || !std::isfinite(body_to_base_y_) ||
          !std::isfinite(body_to_base_z_))) ||
        normalizedFrame(odom_child_frame_).empty() || history_topic_.empty() ||
        !(history_publish_period_ > 0.0) ||
        !std::isfinite(history_publish_period_))
      throw std::runtime_error("invalid voxel mapper parameters");
    odom_child_frame_ = normalizedFrame(odom_child_frame_);

    status_publisher_ = nh_.advertise<std_msgs::String>("/static_mapping/pointcloud_status", 1, true);
    history_publisher_ =
        nh_.advertise<sensor_msgs::PointCloud2>(history_topic_, 1, false);
    odom_subscriber_.subscribe(nh_, odom_topic_, queue_size_);
    cloud_subscriber_.subscribe(nh_, input_topic_, queue_size_);
    synchronizer_.reset(new Synchronizer(SyncPolicy(queue_size_), odom_subscriber_,
                                         cloud_subscriber_));
    synchronizer_->registerCallback(boost::bind(
        &VoxelCloudMapper::cloudCallback, this, boost::placeholders::_1,
        boost::placeholders::_2));
    publishStatus("RECORDING");
    ROS_INFO_STREAM("voxel_cloud_mapper: " << input_topic_ << " -> " << output_file_
                    << " (voxel " << voxel_size_ << " m, >= "
                    << min_frame_observations_ << " frames, range <= "
                    << max_point_range_ << " m; persistent slice >= "
                    << slice_min_frame_observations_ << " frames; moving self-crop "
                    << (slice_self_crop_enabled_ ? "enabled" : "disabled")
                    << "; requested history preview <= "
                    << 1.0 / history_publish_period_ << " Hz on "
                    << history_topic_ << ")");
  }

  ~VoxelCloudMapper()
  {
    save();
  }

private:
  using SyncPolicy = message_filters::sync_policies::ExactTime<
      nav_msgs::Odometry, sensor_msgs::PointCloud2>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

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

  bool hasField(const sensor_msgs::PointCloud2& cloud, const std::string& field) const
  {
    for (const auto& candidate : cloud.fields)
      if (candidate.name == field)
        return true;
    return false;
  }

  std::vector<PcdPoint> persistentPoints() const
  {
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
    return points;
  }

  void publishHistoryCloudIfRequested(const ros::Time& stamp)
  {
    // Building a complete snapshot can be expensive.  Hidden RViz displays
    // unsubscribe, so keep the normal mapping path free of preview work and
    // publish at most one full, replace-in-place history map per period.
    if (history_publisher_.getNumSubscribers() == 0U)
      return;
    const ros::WallTime now = ros::WallTime::now();
    if (!last_history_publish_at_.isZero() &&
        (now - last_history_publish_at_).toSec() < history_publish_period_)
      return;
    last_history_publish_at_ = now;

    const std::vector<PcdPoint> points = persistentPoints();
    sensor_msgs::PointCloud2 message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id_;
    sensor_msgs::PointCloud2Modifier modifier(message);
    modifier.setPointCloud2Fields(
        5, "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::PointField::FLOAT32,
        "observations", 1, sensor_msgs::PointField::UINT32);
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(message, "x");
    sensor_msgs::PointCloud2Iterator<float> y(message, "y");
    sensor_msgs::PointCloud2Iterator<float> z(message, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity(message, "intensity");
    sensor_msgs::PointCloud2Iterator<std::uint32_t> observations(
        message, "observations");
    for (const PcdPoint& point : points)
    {
      *x = point.x;
      *y = point.y;
      *z = point.z;
      *intensity = point.intensity;
      *observations = point.observations;
      ++x;
      ++y;
      ++z;
      ++intensity;
      ++observations;
    }
    message.is_dense = true;
    history_publisher_.publish(message);
  }

  void cloudCallback(const nav_msgs::OdometryConstPtr& odometry,
                     const sensor_msgs::PointCloud2ConstPtr& cloud)
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
    const std::string odom_frame = normalizedFrame(odometry->header.frame_id);
    const std::string odom_child = normalizedFrame(odometry->child_frame_id);
    if (odom_frame != cloud_frame || odom_child != odom_child_frame_)
    {
      ++rejected_clouds_;
      ROS_WARN_THROTTLE(5.0, "voxel_cloud_mapper: synchronized odometry frame mismatch");
      return;
    }
    const double odom_x = odometry->pose.pose.position.x;
    const double odom_y = odometry->pose.pose.position.y;
    const double odom_z = odometry->pose.pose.position.z;
    const auto& orientation_message = odometry->pose.pose.orientation;
    const robot_bringup::moving_self_crop::Quaternion orientation_input{
        orientation_message.x, orientation_message.y, orientation_message.z,
        orientation_message.w};
    robot_bringup::moving_self_crop::Quaternion orientation;
    if (!std::isfinite(odom_x) || !std::isfinite(odom_y) ||
        !std::isfinite(odom_z) ||
        !robot_bringup::moving_self_crop::normalizeQuaternion(
            orientation_input, &orientation))
    {
      ++rejected_clouds_;
      ROS_WARN_THROTTLE(5.0, "voxel_cloud_mapper: synchronized odometry pose is invalid");
      return;
    }
    if (slice_self_crop_enabled_ &&
        !recordSweptCrop(odom_x, odom_y, odom_z, orientation))
    {
      ++rejected_clouds_;
      ROS_WARN_THROTTLE(5.0, "voxel_cloud_mapper: cannot transform synchronized base pose");
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
      const double dx = static_cast<double>(px) - odom_x;
      const double dy = static_cast<double>(py) - odom_y;
      const double dz = static_cast<double>(pz) - odom_z;
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
        if (slice_self_crop_enabled_ &&
            robot_bringup::moving_self_crop::mapPointInsideBaseCrop(
                px, py, pz, odom_x, odom_y, odom_z, orientation,
                body_to_base_x_, body_to_base_y_, body_to_base_z_,
                slice_self_crop_))
        {
          ++slice_self_crop_rejected_points_;
        }
        else
        {
          slice_cells_seen.insert(
              GridKey{static_cast<int>(std::floor(px / slice_resolution_)),
                      static_cast<int>(std::floor(py / slice_resolution_))});
        }
      }
      ++input_points_;
    }
    for (const GridKey& cell : slice_cells_seen)
      ++slice_frame_observations_[cell];
    ++clouds_;
    publishHistoryCloudIfRequested(cloud->header.stamp);
    ROS_INFO_STREAM_THROTTLE(10.0, "voxel_cloud_mapper: " << clouds_ << " clouds, "
                                                           << voxels_.size() << " voxels");
  }

  void rasterizeSweptCrop(
      const robot_bringup::moving_self_crop::Pose2d& base_pose)
  {
    const std::array<double, 2> xs{{slice_sweep_crop_.min_x,
                                    slice_sweep_crop_.max_x}};
    const std::array<double, 2> ys{{slice_sweep_crop_.min_y,
                                    slice_sweep_crop_.max_y}};
    const double cosine = std::cos(base_pose.yaw);
    const double sine = std::sin(base_pose.yaw);
    double minimum_x = std::numeric_limits<double>::infinity();
    double maximum_x = -std::numeric_limits<double>::infinity();
    double minimum_y = std::numeric_limits<double>::infinity();
    double maximum_y = -std::numeric_limits<double>::infinity();
    for (const double local_x : xs)
    {
      for (const double local_y : ys)
      {
        const double world_x = base_pose.x + cosine * local_x - sine * local_y;
        const double world_y = base_pose.y + sine * local_x + cosine * local_y;
        minimum_x = std::min(minimum_x, world_x);
        maximum_x = std::max(maximum_x, world_x);
        minimum_y = std::min(minimum_y, world_y);
        maximum_y = std::max(maximum_y, world_y);
      }
    }
    const int minimum_cell_x =
        static_cast<int>(std::floor(minimum_x / slice_resolution_));
    const int maximum_cell_x =
        static_cast<int>(std::floor(maximum_x / slice_resolution_));
    const int minimum_cell_y =
        static_cast<int>(std::floor(minimum_y / slice_resolution_));
    const int maximum_cell_y =
        static_cast<int>(std::floor(maximum_y / slice_resolution_));
    for (int cell_y = minimum_cell_y; cell_y <= maximum_cell_y; ++cell_y)
    {
      for (int cell_x = minimum_cell_x; cell_x <= maximum_cell_x; ++cell_x)
      {
        if (robot_bringup::moving_self_crop::cropIntersectsGridCell(
                slice_sweep_crop_, base_pose, cell_x, cell_y,
                slice_resolution_))
          slice_swept_cells_.insert(GridKey{cell_x, cell_y});
      }
    }
  }

  bool recordSweptCrop(
      const double body_x, const double body_y, const double body_z,
      const robot_bringup::moving_self_crop::Quaternion& orientation)
  {
    robot_bringup::moving_self_crop::Pose2d current_pose;
    if (!robot_bringup::moving_self_crop::basePoseInMap(
            body_x, body_y, body_z, orientation, body_to_base_x_,
            body_to_base_y_, body_to_base_z_, &current_pose))
      return false;
    if (!have_last_sweep_pose_)
    {
      rasterizeSweptCrop(current_pose);
      ++slice_sweep_pose_samples_;
    }
    else
    {
      const std::size_t steps =
          robot_bringup::moving_self_crop::interpolationSteps(
              last_sweep_pose_, current_pose, slice_sweep_linear_step_,
              slice_sweep_angular_step_);
      if (steps == 0U)
        return false;
      for (std::size_t step = 1; step <= steps; ++step)
      {
        rasterizeSweptCrop(robot_bringup::moving_self_crop::interpolatePose(
            last_sweep_pose_, current_pose,
            static_cast<double>(step) / static_cast<double>(steps)));
        ++slice_sweep_pose_samples_;
      }
    }
    last_sweep_pose_ = current_pose;
    have_last_sweep_pose_ = true;
    return true;
  }

  bool saveSliceObservations() const
  {
    if (slice_observations_file_.empty())
      return true;
    const std::string temporary = slice_observations_file_ + ".tmp";
    std::ofstream stream(temporary.c_str(), std::ios::trunc);
    if (!stream)
      return false;
    std::vector<std::pair<GridKey, std::uint32_t>> accepted_entries;
    std::size_t accepted_before_sweep = 0;
    std::size_t swept_accepted_cells_filtered = 0;
    for (const auto& entry : slice_frame_observations_)
    {
      if (entry.second < static_cast<std::uint32_t>(slice_min_frame_observations_))
        continue;
      ++accepted_before_sweep;
      if (slice_self_crop_enabled_ &&
          slice_swept_cells_.find(entry.first) != slice_swept_cells_.end())
      {
        ++swept_accepted_cells_filtered;
        continue;
      }
      accepted_entries.push_back(entry);
    }
    const auto grid_less = [](const auto& first, const auto& second) {
      if (first.first.x != second.first.x)
        return first.first.x < second.first.x;
      return first.first.y < second.first.y;
    };
    std::sort(accepted_entries.begin(), accepted_entries.end(), grid_less);
    std::vector<GridKey> swept_cells(slice_swept_cells_.begin(),
                                     slice_swept_cells_.end());
    std::sort(swept_cells.begin(), swept_cells.end(),
              [](const GridKey& first, const GridKey& second) {
                if (first.x != second.x)
                  return first.x < second.x;
                return first.y < second.y;
              });
    stream << std::setprecision(12)
           << "schema_version: 2\n"
           << "frame_id: " << frame_id_ << "\n"
           << "resolution_m: " << slice_resolution_ << "\n"
           << "slice_center_z_m: " << slice_center_z_ << "\n"
           << "slice_half_width_m: " << slice_half_width_ << "\n"
           << "min_frame_observations: " << slice_min_frame_observations_ << "\n"
           << "observed_clouds: " << clouds_ << "\n"
           << "rejected_clouds: " << rejected_clouds_ << "\n"
           << "candidate_cells: " << slice_frame_observations_.size() << "\n"
           << "accepted_cells_before_sweep: " << accepted_before_sweep << "\n"
           << "accepted_cells: " << accepted_entries.size() << "\n"
           << "moving_self_crop:\n"
           << "  enabled: " << (slice_self_crop_enabled_ ? "true" : "false") << "\n"
           << "  point_frame_id: base_link\n"
           << "  point_bounds_xy_m: [" << slice_self_crop_.min_x << ", "
           << slice_self_crop_.max_x << ", " << slice_self_crop_.min_y << ", "
           << slice_self_crop_.max_y << "]\n"
           << "  body_to_base_xyz_m: [" << body_to_base_x_ << ", "
           << body_to_base_y_ << ", " << body_to_base_z_ << "]\n"
           << "  exact_time_sync: true\n"
           << "  point_rejected_count: " << slice_self_crop_rejected_points_ << "\n"
           << "  sweep_frame_id: base_link\n"
           << "  sweep_bounds_xy_m: [" << slice_sweep_crop_.min_x << ", "
           << slice_sweep_crop_.max_x << ", " << slice_sweep_crop_.min_y << ", "
           << slice_sweep_crop_.max_y << "]\n"
           << "  sweep_linear_step_m: " << slice_sweep_linear_step_ << "\n"
           << "  sweep_angular_step_rad: " << slice_sweep_angular_step_ << "\n"
           << "  sweep_pose_samples: " << slice_sweep_pose_samples_ << "\n"
           << "  swept_cells: " << swept_cells.size() << "\n"
           << "  swept_accepted_cells_filtered: "
           << swept_accepted_cells_filtered << "\n"
           << "cells:\n";
    for (const auto& entry : accepted_entries)
    {
      stream << "  - [" << entry.first.x << ", " << entry.first.y << ", "
             << entry.second << "]\n";
    }
    stream << "swept_cells:\n";
    for (const GridKey& cell : swept_cells)
      stream << "  - [" << cell.x << ", " << cell.y << "]\n";
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
    const std::vector<PcdPoint> points = persistentPoints();
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
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_subscriber_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_subscriber_;
  std::unique_ptr<Synchronizer> synchronizer_;
  ros::Publisher status_publisher_;
  ros::Publisher history_publisher_;
  std::string input_topic_;
  std::string odom_topic_;
  std::string output_file_;
  std::string slice_observations_file_;
  std::string history_topic_ = "/static_mapping/history_cloud";
  std::string frame_id_;
  std::string odom_child_frame_ = "body";
  double voxel_size_ = 0.10;
  double history_publish_period_ = 1.0;
  int min_frame_observations_ = 3;
  double max_point_range_ = 20.0;
  double slice_center_z_ = -0.4;
  double slice_half_width_ = 0.20;
  double slice_resolution_ = 0.10;
  int slice_min_frame_observations_ = 20;
  bool slice_self_crop_enabled_ = true;
  robot_bringup::moving_self_crop::CropBox slice_self_crop_;
  robot_bringup::moving_self_crop::CropBox slice_sweep_crop_;
  double slice_sweep_rear_ = 0.62;
  double body_to_base_x_ = -0.211;
  double body_to_base_y_ = -0.02329;
  double body_to_base_z_ = -0.95588;
  double slice_sweep_linear_step_ = 0.05;
  double slice_sweep_angular_step_ = 0.03490658503988659;
  int queue_size_ = 20;
  std::unordered_map<VoxelKey, Voxel, VoxelKeyHash> voxels_;
  std::unordered_map<GridKey, std::uint32_t, GridKeyHash> slice_frame_observations_;
  std::unordered_set<GridKey, GridKeyHash> slice_swept_cells_;
  robot_bringup::moving_self_crop::Pose2d last_sweep_pose_;
  std::uint64_t clouds_ = 0;
  std::uint64_t rejected_clouds_ = 0;
  std::uint64_t input_points_ = 0;
  std::uint64_t range_rejected_points_ = 0;
  std::uint64_t slice_self_crop_rejected_points_ = 0;
  std::uint64_t slice_sweep_pose_samples_ = 0;
  ros::WallTime last_history_publish_at_;
  bool have_last_sweep_pose_ = false;
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
