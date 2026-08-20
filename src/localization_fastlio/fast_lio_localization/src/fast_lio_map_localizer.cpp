// SPDX-License-Identifier: GPL-2.0-only
// Architecture adapted for ROS Noetic/PCL from:
// https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Geometry>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
using Matrix = Eigen::Matrix4f;

std::string normalizedFrame(std::string frame)
{
  while (!frame.empty() && frame.front() == '/')
    frame.erase(frame.begin());
  return frame;
}

bool finiteMatrix(const Matrix& matrix)
{
  return matrix.array().isFinite().all();
}

Matrix poseMatrix(const geometry_msgs::Pose& pose)
{
  Eigen::Quaternionf quaternion(static_cast<float>(pose.orientation.w),
                                static_cast<float>(pose.orientation.x),
                                static_cast<float>(pose.orientation.y),
                                static_cast<float>(pose.orientation.z));
  if (!std::isfinite(quaternion.norm()) || quaternion.norm() < 1.0e-6f)
    throw std::runtime_error("pose contains an invalid quaternion");
  quaternion.normalize();
  Matrix matrix = Matrix::Identity();
  matrix.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
  matrix(0, 3) = static_cast<float>(pose.position.x);
  matrix(1, 3) = static_cast<float>(pose.position.y);
  matrix(2, 3) = static_cast<float>(pose.position.z);
  return matrix;
}

geometry_msgs::Pose matrixPose(const Matrix& matrix)
{
  geometry_msgs::Pose pose;
  Eigen::Quaternionf quaternion(matrix.block<3, 3>(0, 0));
  quaternion.normalize();
  pose.position.x = matrix(0, 3);
  pose.position.y = matrix(1, 3);
  pose.position.z = matrix(2, 3);
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

double yawFromQuaternion(const geometry_msgs::Quaternion& quaternion)
{
  const double sin_yaw = 2.0 * (quaternion.w * quaternion.z +
                                quaternion.x * quaternion.y);
  const double cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y +
                                     quaternion.z * quaternion.z);
  return std::atan2(sin_yaw, cos_yaw);
}

Cloud::Ptr downsample(const Cloud::ConstPtr& input, double leaf_size)
{
  Cloud::Ptr output(new Cloud());
  pcl::VoxelGrid<Point> filter;
  filter.setInputCloud(input);
  const float leaf = static_cast<float>(leaf_size);
  filter.setLeafSize(leaf, leaf, leaf);
  filter.filter(*output);
  return output;
}

struct RegistrationFlagReset
{
  explicit RegistrationFlagReset(std::atomic<bool>& flag) : flag_(flag) {}
  ~RegistrationFlagReset() { flag_.store(false); }
  std::atomic<bool>& flag_;
};
}  // namespace

class FastLioMapLocalizer
{
public:
  FastLioMapLocalizer() : private_nh_("~")
  {
    loadParameters();
    validateParameters();
    loadMap();

    status_publisher_ = nh_.advertise<std_msgs::String>(status_topic_, 1, true);
    localization_publisher_ = nh_.advertise<nav_msgs::Odometry>(localization_topic_, 5);
    prior_map_publisher_ =
        nh_.advertise<sensor_msgs::PointCloud2>(prior_map_topic_, 1, true);
    submap_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(submap_topic_, 1);
    aligned_scan_publisher_ =
        nh_.advertise<sensor_msgs::PointCloud2>(aligned_scan_topic_, 1);

    scan_subscriber_ = nh_.subscribe(scan_topic_, 1,
                                     &FastLioMapLocalizer::scanCallback, this);
    odom_subscriber_ = nh_.subscribe(odom_topic_, 20,
                                     &FastLioMapLocalizer::odomCallback, this);
    initial_pose_subscriber_ = nh_.subscribe(
        initial_pose_topic_, 2, &FastLioMapLocalizer::initialPoseCallback, this);

    registration_timer_ = nh_.createTimer(
        ros::Duration(1.0 / registration_frequency_),
        &FastLioMapLocalizer::registrationTimer, this);
    output_timer_ = nh_.createTimer(ros::Duration(1.0 / tf_publish_frequency_),
                                    &FastLioMapLocalizer::outputTimer, this);

    publishPriorMap();
    publishStatus();
    ROS_INFO_STREAM("fast_lio_map_localizer: loaded " << global_map_->size()
                    << " points from " << map_file_ << "; waiting for "
                    << initial_pose_topic_);
  }

private:
  void loadParameters()
  {
    private_nh_.param<std::string>("map_file", map_file_, std::string());
    private_nh_.param<std::string>("scan_topic", scan_topic_, "/cloud_registered");
    private_nh_.param<std::string>("odom_topic", odom_topic_, "/Odometry");
    private_nh_.param<std::string>("initial_pose_topic", initial_pose_topic_,
                                  "/initialpose");
    private_nh_.param<std::string>("status_topic", status_topic_,
                                  "/fast_lio/localization_status");
    private_nh_.param<std::string>("localization_topic", localization_topic_,
                                  "/localization");
    private_nh_.param<std::string>("prior_map_topic", prior_map_topic_,
                                  "/fast_lio_localization/prior_map");
    private_nh_.param<std::string>("submap_topic", submap_topic_,
                                  "/fast_lio_localization/submap");
    private_nh_.param<std::string>("aligned_scan_topic", aligned_scan_topic_,
                                  "/fast_lio_localization/aligned_scan");
    private_nh_.param<std::string>("map_frame", map_frame_, "map");
    private_nh_.param<std::string>("odom_frame", odom_frame_, "camera_init");
    private_nh_.param<std::string>("body_frame", body_frame_, "body");

    private_nh_.param("registration_frequency", registration_frequency_, 0.5);
    private_nh_.param("tf_publish_frequency", tf_publish_frequency_, 20.0);
    private_nh_.param("data_timeout", data_timeout_, 1.0);
    private_nh_.param("localization_timeout", localization_timeout_, 5.0);
    private_nh_.param("map_voxel_size", map_voxel_size_, 0.40);
    private_nh_.param("coarse_scan_voxel_size", coarse_scan_voxel_size_, 0.50);
    private_nh_.param("coarse_map_voxel_size", coarse_map_voxel_size_, 2.00);
    private_nh_.param("coarse_max_correspondence", coarse_max_correspondence_, 5.0);
    private_nh_.param("coarse_iterations", coarse_iterations_, 20);
    private_nh_.param("fine_scan_voxel_size", fine_scan_voxel_size_, 0.10);
    private_nh_.param("fine_map_voxel_size", fine_map_voxel_size_, 0.40);
    private_nh_.param("fine_max_correspondence", fine_max_correspondence_, 1.0);
    private_nh_.param("fine_iterations", fine_iterations_, 30);
    private_nh_.param("submap_radius", submap_radius_, 60.0);
    private_nh_.param("submap_half_height", submap_half_height_, 8.0);
    private_nh_.param("min_source_points", min_source_points_, 100);
    private_nh_.param("min_target_points", min_target_points_, 300);
    private_nh_.param("min_inliers", min_inliers_, 100);
    private_nh_.param("min_overlap", min_overlap_, 0.30);
    private_nh_.param("max_rmse", max_rmse_, 0.35);
    private_nh_.param("good_matches_required", good_matches_required_, 1);
    private_nh_.param("lost_matches_required", lost_matches_required_, 3);
    private_nh_.param("base_to_body_x", base_to_body_x_, 0.20);
    private_nh_.param("base_to_body_y", base_to_body_y_, 0.0);
    private_nh_.param("initial_body_z", initial_body_z_, 0.0);

    map_frame_ = normalizedFrame(map_frame_);
    odom_frame_ = normalizedFrame(odom_frame_);
    body_frame_ = normalizedFrame(body_frame_);
  }

  void validateParameters() const
  {
    if (map_file_.empty())
      throw std::runtime_error("~map_file is required");
    if (map_frame_.empty() || odom_frame_.empty() || body_frame_.empty())
      throw std::runtime_error("frame names must not be empty");
    if (map_frame_ == odom_frame_)
      throw std::runtime_error("map_frame and odom_frame must differ");
    if (!(registration_frequency_ > 0.0) || !(tf_publish_frequency_ > 0.0) ||
        !(data_timeout_ > 0.0) || !(localization_timeout_ > data_timeout_))
      throw std::runtime_error("frequency/timeout parameters are invalid");
    if (!(map_voxel_size_ > 0.0) || !(coarse_scan_voxel_size_ > 0.0) ||
        !(coarse_map_voxel_size_ > 0.0) || !(fine_scan_voxel_size_ > 0.0) ||
        !(fine_map_voxel_size_ > 0.0) || !(coarse_max_correspondence_ > 0.0) ||
        !(fine_max_correspondence_ > 0.0))
      throw std::runtime_error("ICP scale parameters must be positive");
    if (!(submap_radius_ > fine_max_correspondence_) ||
        !(submap_half_height_ > 0.0) || min_source_points_ < 10 ||
        min_target_points_ < 10 || min_inliers_ < 10 ||
        !(min_overlap_ > 0.0 && min_overlap_ <= 1.0) || !(max_rmse_ > 0.0) ||
        coarse_iterations_ < 1 || fine_iterations_ < 1 ||
        good_matches_required_ < 1 || lost_matches_required_ < 1)
      throw std::runtime_error("ICP quality parameters are invalid");
  }

  void loadMap()
  {
    Cloud::Ptr loaded(new Cloud());
    if (pcl::io::loadPCDFile<Point>(map_file_, *loaded) < 0)
      throw std::runtime_error("cannot load prior PCD: " + map_file_);
    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(*loaded, *loaded, valid_indices);
    global_map_ = downsample(loaded, map_voxel_size_);
    if (global_map_->size() < static_cast<std::size_t>(min_target_points_))
      throw std::runtime_error("prior PCD has too few valid points");
  }

  void scanCallback(const sensor_msgs::PointCloud2ConstPtr& message)
  {
    if (normalizedFrame(message->header.frame_id) != odom_frame_)
    {
      ROS_ERROR_THROTTLE(5.0,
                         "fast_lio_map_localizer: scan frame must be camera_init/odom frame");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    latest_scan_ = message;
    latest_scan_received_ = ros::Time::now();
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& message)
  {
    if (normalizedFrame(message->header.frame_id) != odom_frame_ ||
        normalizedFrame(message->child_frame_id) != body_frame_)
    {
      ROS_ERROR_THROTTLE(5.0, "fast_lio_map_localizer: odometry frame mismatch");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    latest_odom_ = message;
    latest_odom_received_ = ros::Time::now();
  }

  void initialPoseCallback(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr& message)
  {
    if (normalizedFrame(message->header.frame_id) != map_frame_)
    {
      ROS_ERROR_STREAM("fast_lio_map_localizer: /initialpose must use frame "
                       << map_frame_);
      return;
    }
    if (!std::isfinite(message->pose.pose.position.x) ||
        !std::isfinite(message->pose.pose.position.y))
    {
      ROS_ERROR("fast_lio_map_localizer: rejecting non-finite initial pose");
      return;
    }

    nav_msgs::OdometryConstPtr odometry;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      odometry = latest_odom_;
    }
    if (!odometry)
    {
      ROS_WARN("fast_lio_map_localizer: wait for FAST-LIO odometry before initial pose");
      return;
    }

    try
    {
      const double yaw = yawFromQuaternion(message->pose.pose.orientation);
      Matrix map_to_body = Matrix::Identity();
      const Eigen::AngleAxisf yaw_rotation(static_cast<float>(yaw),
                                           Eigen::Vector3f::UnitZ());
      map_to_body.block<3, 3>(0, 0) = yaw_rotation.toRotationMatrix();
      map_to_body(0, 3) = static_cast<float>(
          message->pose.pose.position.x + std::cos(yaw) * base_to_body_x_ -
          std::sin(yaw) * base_to_body_y_);
      map_to_body(1, 3) = static_cast<float>(
          message->pose.pose.position.y + std::sin(yaw) * base_to_body_x_ +
          std::cos(yaw) * base_to_body_y_);
      map_to_body(2, 3) = static_cast<float>(initial_body_z_);
      const Matrix odom_to_body = poseMatrix(odometry->pose.pose);
      const Matrix seed = map_to_body * odom_to_body.inverse();
      if (!finiteMatrix(seed))
        throw std::runtime_error("initial transform is non-finite");

      {
        std::lock_guard<std::mutex> lock(mutex_);
        map_to_odom_ = seed;
        initial_pose_received_ = true;
        ever_localized_ = false;
        state_ = "ALIGNING";
        good_matches_ = 0;
        bad_matches_ = 0;
        overlap_ = 0.0;
        rmse_ = std::numeric_limits<double>::infinity();
        inliers_ = 0;
        last_success_ = ros::Time(0);
        ++seed_generation_;
      }
      ROS_WARN_STREAM("fast_lio_map_localizer: accepted approximate base pose x="
                      << message->pose.pose.position.x << " y="
                      << message->pose.pose.position.y << " yaw=" << yaw);
      publishStatus();
    }
    catch (const std::exception& error)
    {
      ROS_ERROR_STREAM("fast_lio_map_localizer: invalid initial pose: " << error.what());
    }
  }

  void registrationTimer(const ros::TimerEvent&)
  {
    if (registration_running_.exchange(true))
    {
      ROS_WARN_THROTTLE(5.0, "fast_lio_map_localizer: previous ICP is still running");
      return;
    }
    RegistrationFlagReset reset(registration_running_);

    sensor_msgs::PointCloud2ConstPtr scan_message;
    nav_msgs::OdometryConstPtr odometry;
    Matrix initial_guess;
    ros::Time scan_received;
    std::uint64_t generation = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!initial_pose_received_ || !latest_scan_ || !latest_odom_)
        return;
      scan_message = latest_scan_;
      odometry = latest_odom_;
      initial_guess = map_to_odom_;
      scan_received = latest_scan_received_;
      generation = seed_generation_;
    }

    const ros::Time now = ros::Time::now();
    if (!now.isZero() && !scan_received.isZero() &&
        (now - scan_received).toSec() > data_timeout_)
    {
      recordFailure(generation, "scan is stale");
      return;
    }

    Cloud::Ptr scan(new Cloud());
    pcl::fromROSMsg(*scan_message, *scan);
    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(*scan, *scan, valid_indices);
    Cloud::Ptr source_fine = downsample(scan, fine_scan_voxel_size_);
    if (source_fine->size() < static_cast<std::size_t>(min_source_points_))
    {
      recordFailure(generation, "current scan has too few points");
      return;
    }

    Matrix odom_to_body;
    try
    {
      odom_to_body = poseMatrix(odometry->pose.pose);
    }
    catch (const std::exception& error)
    {
      recordFailure(generation, error.what());
      return;
    }
    const Matrix predicted_map_to_body = initial_guess * odom_to_body;
    const Eigen::Vector3f center = predicted_map_to_body.block<3, 1>(0, 3);

    pcl::CropBox<Point> crop;
    crop.setInputCloud(global_map_);
    crop.setMin(Eigen::Vector4f(center.x() - submap_radius_,
                               center.y() - submap_radius_,
                               center.z() - submap_half_height_, 1.0f));
    crop.setMax(Eigen::Vector4f(center.x() + submap_radius_,
                               center.y() + submap_radius_,
                               center.z() + submap_half_height_, 1.0f));
    Cloud::Ptr target_fine(new Cloud());
    crop.filter(*target_fine);
    target_fine = downsample(target_fine, fine_map_voxel_size_);
    if (target_fine->size() < static_cast<std::size_t>(min_target_points_))
    {
      recordFailure(generation, "predicted submap has too few points");
      return;
    }

    Cloud::Ptr source_coarse = downsample(source_fine, coarse_scan_voxel_size_);
    Cloud::Ptr target_coarse = downsample(target_fine, coarse_map_voxel_size_);
    pcl::IterativeClosestPoint<Point, Point> coarse_icp;
    configureIcp(coarse_icp, coarse_iterations_, coarse_max_correspondence_);
    coarse_icp.setInputSource(source_coarse);
    coarse_icp.setInputTarget(target_coarse);
    Cloud coarse_aligned;
    coarse_icp.align(coarse_aligned, initial_guess);
    if (!coarse_icp.hasConverged())
    {
      recordFailure(generation, "coarse ICP did not converge");
      return;
    }

    pcl::IterativeClosestPoint<Point, Point> fine_icp;
    configureIcp(fine_icp, fine_iterations_, fine_max_correspondence_);
    fine_icp.setInputSource(source_fine);
    fine_icp.setInputTarget(target_fine);
    Cloud::Ptr aligned(new Cloud());
    fine_icp.align(*aligned, coarse_icp.getFinalTransformation());
    if (!fine_icp.hasConverged() || !finiteMatrix(fine_icp.getFinalTransformation()))
    {
      recordFailure(generation, "fine ICP did not converge");
      return;
    }

    int inliers = 0;
    double overlap = 0.0;
    double rmse = std::numeric_limits<double>::infinity();
    evaluateAlignment(aligned, target_fine, inliers, overlap, rmse);
    const bool accepted = inliers >= min_inliers_ && overlap >= min_overlap_ &&
                          std::isfinite(rmse) && rmse <= max_rmse_;
    if (!accepted)
    {
      std::ostringstream reason;
      reason << "quality rejected: inliers=" << inliers << " overlap=" << overlap
             << " rmse=" << rmse;
      recordFailure(generation, reason.str(), inliers, overlap, rmse);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (generation != seed_generation_)
        return;
      map_to_odom_ = fine_icp.getFinalTransformation();
      inliers_ = inliers;
      overlap_ = overlap;
      rmse_ = rmse;
      ++good_matches_;
      bad_matches_ = 0;
      last_success_ = now;
      if (good_matches_ >= good_matches_required_)
      {
        state_ = "LOCALIZED";
        ever_localized_ = true;
      }
      else
      {
        state_ = "ALIGNING";
      }
    }
    publishCloud(submap_publisher_, target_fine, map_frame_, now);
    publishCloud(aligned_scan_publisher_, aligned, map_frame_, scan_message->header.stamp);
    ROS_INFO_STREAM("fast_lio_map_localizer: accepted ICP overlap=" << overlap
                    << " rmse=" << rmse << " inliers=" << inliers);
  }

  void configureIcp(pcl::IterativeClosestPoint<Point, Point>& icp, int iterations,
                    double correspondence) const
  {
    icp.setMaximumIterations(iterations);
    icp.setMaxCorrespondenceDistance(correspondence);
    icp.setTransformationEpsilon(1.0e-6);
    icp.setEuclideanFitnessEpsilon(1.0e-6);
    icp.setRANSACOutlierRejectionThreshold(correspondence);
  }

  void evaluateAlignment(const Cloud::ConstPtr& aligned,
                         const Cloud::ConstPtr& target, int& inliers,
                         double& overlap, double& rmse) const
  {
    pcl::KdTreeFLANN<Point> tree;
    tree.setInputCloud(target);
    const float maximum_squared =
        static_cast<float>(fine_max_correspondence_ * fine_max_correspondence_);
    double squared_sum = 0.0;
    std::vector<int> indices(1);
    std::vector<float> squared_distances(1);
    inliers = 0;
    for (const Point& point : aligned->points)
    {
      if (!pcl::isFinite(point))
        continue;
      if (tree.nearestKSearch(point, 1, indices, squared_distances) == 1 &&
          squared_distances[0] <= maximum_squared)
      {
        ++inliers;
        squared_sum += squared_distances[0];
      }
    }
    overlap = aligned->empty() ? 0.0
                               : static_cast<double>(inliers) / aligned->size();
    rmse = inliers > 0 ? std::sqrt(squared_sum / inliers)
                       : std::numeric_limits<double>::infinity();
  }

  void recordFailure(std::uint64_t generation, const std::string& reason,
                     int inliers = 0, double overlap = 0.0,
                     double rmse = std::numeric_limits<double>::infinity())
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (generation != seed_generation_)
        return;
      inliers_ = inliers;
      overlap_ = overlap;
      rmse_ = rmse;
      good_matches_ = 0;
      ++bad_matches_;
      if (bad_matches_ >= lost_matches_required_)
        state_ = "LOST";
      else
        state_ = ever_localized_ ? "DEGRADED" : "ALIGNING";
    }
    ROS_WARN_STREAM("fast_lio_map_localizer: " << reason);
  }

  void outputTimer(const ros::TimerEvent&)
  {
    Matrix map_to_odom;
    nav_msgs::OdometryConstPtr odometry;
    bool publish_transform = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const ros::Time now = ros::Time::now();
      if (initial_pose_received_)
      {
        const bool data_stale =
            (!latest_scan_received_.isZero() &&
             (now - latest_scan_received_).toSec() > data_timeout_) ||
            (!latest_odom_received_.isZero() &&
             (now - latest_odom_received_).toSec() > data_timeout_);
        const bool match_expired =
            ever_localized_ && !last_success_.isZero() &&
            (now - last_success_).toSec() > localization_timeout_;
        if (data_stale || match_expired)
          state_ = "LOST";
        map_to_odom = map_to_odom_;
        odometry = latest_odom_;
        publish_transform = true;
      }
    }

    if (publish_transform)
    {
      const ros::Time now = ros::Time::now();
      geometry_msgs::TransformStamped transform;
      transform.header.stamp = now;
      transform.header.frame_id = map_frame_;
      transform.child_frame_id = odom_frame_;
      const geometry_msgs::Pose pose = matrixPose(map_to_odom);
      transform.transform.translation.x = pose.position.x;
      transform.transform.translation.y = pose.position.y;
      transform.transform.translation.z = pose.position.z;
      transform.transform.rotation = pose.orientation;
      transform_broadcaster_.sendTransform(transform);

      if (odometry)
      {
        try
        {
          nav_msgs::Odometry localization;
          localization.header.stamp = odometry->header.stamp;
          localization.header.frame_id = map_frame_;
          localization.child_frame_id = body_frame_;
          localization.pose.pose = matrixPose(map_to_odom * poseMatrix(odometry->pose.pose));
          localization.pose.covariance = odometry->pose.covariance;
          localization.twist = odometry->twist;
          localization_publisher_.publish(localization);
        }
        catch (const std::exception& error)
        {
          ROS_ERROR_THROTTLE(5.0, "fast_lio_map_localizer: invalid odometry pose");
        }
      }
    }
    publishStatus();
  }

  void publishStatus()
  {
    std_msgs::String message;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      std::ostringstream stream;
      stream << std::fixed << std::setprecision(4)
             << "state=" << state_ << ";overlap=" << overlap_ << ";rmse=";
      if (std::isfinite(rmse_))
        stream << rmse_;
      else
        stream << "inf";
      stream << ";inliers=" << inliers_
             << ";initial_pose=" << (initial_pose_received_ ? "applied" : "waiting")
             << ";backend=multiscale_icp";
      message.data = stream.str();
    }
    status_publisher_.publish(message);
  }

  void publishPriorMap()
  {
    publishCloud(prior_map_publisher_, global_map_, map_frame_, ros::Time::now());
  }

  void publishCloud(const ros::Publisher& publisher, const Cloud::ConstPtr& cloud,
                    const std::string& frame, const ros::Time& stamp) const
  {
    sensor_msgs::PointCloud2 message;
    pcl::toROSMsg(*cloud, message);
    message.header.frame_id = frame;
    message.header.stamp = stamp;
    publisher.publish(message);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber scan_subscriber_;
  ros::Subscriber odom_subscriber_;
  ros::Subscriber initial_pose_subscriber_;
  ros::Publisher status_publisher_;
  ros::Publisher localization_publisher_;
  ros::Publisher prior_map_publisher_;
  ros::Publisher submap_publisher_;
  ros::Publisher aligned_scan_publisher_;
  ros::Timer registration_timer_;
  ros::Timer output_timer_;
  tf2_ros::TransformBroadcaster transform_broadcaster_;

  std::mutex mutex_;
  std::atomic<bool> registration_running_{false};
  sensor_msgs::PointCloud2ConstPtr latest_scan_;
  nav_msgs::OdometryConstPtr latest_odom_;
  ros::Time latest_scan_received_;
  ros::Time latest_odom_received_;
  ros::Time last_success_;
  Matrix map_to_odom_ = Matrix::Identity();
  Cloud::Ptr global_map_;
  std::uint64_t seed_generation_ = 0;
  bool initial_pose_received_ = false;
  bool ever_localized_ = false;
  std::string state_ = "WAITING_INITIAL_POSE";
  int good_matches_ = 0;
  int bad_matches_ = 0;
  int inliers_ = 0;
  double overlap_ = 0.0;
  double rmse_ = std::numeric_limits<double>::infinity();

  std::string map_file_;
  std::string scan_topic_;
  std::string odom_topic_;
  std::string initial_pose_topic_;
  std::string status_topic_;
  std::string localization_topic_;
  std::string prior_map_topic_;
  std::string submap_topic_;
  std::string aligned_scan_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string body_frame_;
  double registration_frequency_ = 0.5;
  double tf_publish_frequency_ = 20.0;
  double data_timeout_ = 1.0;
  double localization_timeout_ = 5.0;
  double map_voxel_size_ = 0.40;
  double coarse_scan_voxel_size_ = 0.50;
  double coarse_map_voxel_size_ = 2.00;
  double coarse_max_correspondence_ = 5.0;
  int coarse_iterations_ = 20;
  double fine_scan_voxel_size_ = 0.10;
  double fine_map_voxel_size_ = 0.40;
  double fine_max_correspondence_ = 1.00;
  int fine_iterations_ = 30;
  double submap_radius_ = 60.0;
  double submap_half_height_ = 8.0;
  int min_source_points_ = 100;
  int min_target_points_ = 300;
  int min_inliers_ = 100;
  double min_overlap_ = 0.30;
  double max_rmse_ = 0.35;
  int good_matches_required_ = 1;
  int lost_matches_required_ = 3;
  double base_to_body_x_ = 0.20;
  double base_to_body_y_ = 0.0;
  double initial_body_z_ = 0.0;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fast_lio_map_localizer");
  try
  {
    FastLioMapLocalizer localizer;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL_STREAM("fast_lio_map_localizer: " << error.what());
    return 2;
  }
  return 0;
}
