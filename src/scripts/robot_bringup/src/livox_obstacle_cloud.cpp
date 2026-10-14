#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <livox_ros_driver2/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// GPS navigation needs observations in the current chassis frame, not a LIO
// map. In feature-poor terrain the LIO velocity estimate can diverge and its
// deskewed cloud can leave the obstacle height/range window while still being
// published at 10 Hz. This adapter never uses or integrates a global pose.
class LivoxObstacleCloud
{
public:
  LivoxObstacleCloud() : private_nh_("~")
  {
    const std::string input = private_nh_.param<std::string>("input_topic", "/livox/lidar");
    const std::string output = private_nh_.param<std::string>("output_topic", "/cloud_livox_body");
    const std::string odom = private_nh_.param<std::string>("odom_topic", "/gps/odom");
    private_nh_.param<std::string>("body_frame", body_frame_, "body");
    // Same fixed MID360 lidar-to-IMU extrinsics as FAST_LIO/config/mid360.yaml.
    // Lidar and body axes are aligned; base_link is 0.6 m below body.
    private_nh_.param("lidar_to_body_x", mount_x_, -0.011);
    private_nh_.param("lidar_to_body_y", mount_y_, -0.02329);
    private_nh_.param("lidar_to_body_z", mount_z_, 0.04412);
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(output, 2);
    odom_subscriber_ = nh_.subscribe(odom, 5, &LivoxObstacleCloud::odomCallback, this);
    cloud_subscriber_ = nh_.subscribe(input, 2, &LivoxObstacleCloud::cloudCallback, this);
    ROS_INFO_STREAM("Local obstacle cloud: " << input << " -> " << output
                    << "; independent of FAST-LIO pose/map; measured planar twist from " << odom);
  }

private:
  void odomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    // Twist is expressed in child_frame_id. The configured mount has no yaw
    // offset between base_link and body; translation in z does not affect it.
    if (msg->child_frame_id != "base_link" && msg->child_frame_id != body_frame_)
      return;
    const auto& twist = msg->twist.twist;
    if (!std::isfinite(twist.linear.x) || !std::isfinite(twist.linear.y) ||
        !std::isfinite(twist.angular.z))
      return;
    odom_stamp_ = msg->header.stamp;
    vx_ = twist.linear.x;
    vy_ = twist.linear.y;
    yaw_rate_ = twist.angular.z;
  }

  void cloudCallback(const livox_ros_driver2::CustomMsgConstPtr& msg)
  {
    // Driver point offsets are nanoseconds from header.stamp (scan start).
    // Limit malformed offsets to this 10 Hz sensor's frame, without waiting
    // for localization or relying on an accumulated IMU velocity estimate.
    uint32_t end_offset = 0;
    for (const auto& point : msg->points)
      if (point.offset_time <= 250000000u)
        end_offset = std::max(end_offset, point.offset_time);
    const ros::Time end_stamp = msg->header.stamp + ros::Duration(end_offset * 1e-9);
    const bool compensate = !odom_stamp_.isZero() &&
                            std::abs((end_stamp - odom_stamp_).toSec()) <= 0.5;

    sensor_msgs::PointCloud2 cloud;
    cloud.header = msg->header;
    cloud.header.stamp = end_stamp;
    cloud.header.frame_id = body_frame_;
    cloud.height = 1;
    cloud.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                 "y", 1, sensor_msgs::PointField::FLOAT32,
                                 "z", 1, sensor_msgs::PointField::FLOAT32,
                                 "intensity", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(msg->points.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z"),
        intensity(cloud, "intensity");
    size_t kept = 0;
    for (const auto& point : msg->points)
    {
      // Preserve the return selection used by the existing FAST-LIO reader.
      const uint8_t return_type = point.tag & 0x30;
      if (point.line >= 4 || (return_type != 0x00 && return_type != 0x10) ||
          point.offset_time > 250000000u || !std::isfinite(point.x) ||
          !std::isfinite(point.y) || !std::isfinite(point.z))
        continue;
      // Zero/missing returns must not become points at the mount offset.
      if (point.x * point.x + point.y * point.y + point.z * point.z < 0.04f)
        continue;
      double px = point.x + mount_x_, py = point.y + mount_y_;
      if (compensate)
      {
        // Constant measured chassis twist over ONE frame only, including
        // reverse and turning. Express every return at scan-end time.
        const double dt = (static_cast<double>(point.offset_time) - end_offset) * 1e-9;
        const double angle = yaw_rate_ * dt, c = std::cos(angle), s = std::sin(angle);
        double tx = vx_ * dt, ty = vy_ * dt;
        if (std::abs(yaw_rate_) > 1e-6)
        {
          tx = (vx_ * s - vy_ * (1.0 - c)) / yaw_rate_;
          ty = (vx_ * (1.0 - c) + vy_ * s) / yaw_rate_;
        }
        const double old_x = px;
        px = c * old_x - s * py + tx;
        py = s * old_x + c * py + ty;
      }
      *x = px;
      *y = py;
      *z = point.z + mount_z_;
      *intensity = point.reflectivity;
      ++x; ++y; ++z; ++intensity;
      ++kept;
    }
    modifier.resize(kept);
    publisher_.publish(cloud);
  }

  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber cloud_subscriber_, odom_subscriber_;
  ros::Publisher publisher_;
  ros::Time odom_stamp_;
  std::string body_frame_;
  double mount_x_, mount_y_, mount_z_;
  double vx_ = 0.0, vy_ = 0.0, yaw_rate_ = 0.0;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "livox_obstacle_cloud");
  LivoxObstacleCloud node;
  ros::spin();
  return 0;
}
