#include <cmath>
#include <cstring>
#include <algorithm>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>
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

float readFloat32(const std::vector<uint8_t>& data, const size_t offset)
{
  float value = 0.0f;
  std::memcpy(&value, data.data() + offset, sizeof(value));
  return value;
}

class PointCloudSelfFilter
{
public:
  PointCloudSelfFilter() : private_nh_("~"), tf_listener_(tf_buffer_)
  {
    private_nh_.param<std::string>("input_topic", input_topic_, "/cloud_registered_body");
    private_nh_.param<std::string>("output_topic", output_topic_, "/cloud_filtered_for_scan");
    private_nh_.param("remove_above_z", remove_above_z_, 0.1);
    private_nh_.param("near_radius", near_radius_, 0.4);
    private_nh_.param("near_min_z", near_min_z_, -0.1);
    private_nh_.param("near_max_z", near_max_z_, 0.1);
    private_nh_.param("remove_nan", remove_nan_, true);
    private_nh_.param("max_xy_range", max_xy_range_, 0.0);
    private_nh_.param("remove_ground", remove_ground_, false);
    private_nh_.param<std::string>("footprint_frame", footprint_frame_, "base_link");
    XmlRpc::XmlRpcValue footprint;
    if (private_nh_.getParam("self_footprint", footprint))
    {
      for (int i = 0; i < footprint.size(); ++i)
      {
        const double x = footprint[i][0].getType() == XmlRpc::XmlRpcValue::TypeInt
                             ? static_cast<int>(footprint[i][0]) : static_cast<double>(footprint[i][0]);
        const double y = footprint[i][1].getType() == XmlRpc::XmlRpcValue::TypeInt
                             ? static_cast<int>(footprint[i][1]) : static_cast<double>(footprint[i][1]);
        footprint_.emplace_back(x, y);
      }
    }

    const int queue_size = private_nh_.param("queue_size", 5);

    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, queue_size);
    subscriber_ = nh_.subscribe(input_topic_, queue_size, &PointCloudSelfFilter::cloudCallback, this);

    ROS_INFO_STREAM("pointcloud_self_filter: " << input_topic_ << " -> " << output_topic_
                    << ", remove z > " << remove_above_z_ << " m, remove radius <= "
                    << near_radius_ << " m for " << near_min_z_ << " <= z <= " << near_max_z_ << " m"
                    << ", physical footprint vertices=" << footprint_.size() << " in " << footprint_frame_);
  }

private:
  bool fitGroundPlane(const sensor_msgs::PointCloud2& cloud, const tf2::Transform& to_base,
                      uint32_t xo, uint32_t yo, uint32_t zo,
                      tf2::Vector3* normal, double* offset)
  {
    // Estimate only the local road surface. This is recomputed per frame and
    // never uses global pose, accumulated LIO state, or acceleration as tilt.
    std::vector<tf2::Vector3> candidates;
    for (uint32_t row = 0; row < cloud.height; ++row)
      for (uint32_t col = 0; col < cloud.width; ++col)
      {
        const size_t at = static_cast<size_t>(row) * cloud.row_step + col * cloud.point_step;
        const double x = readFloat32(cloud.data, at + xo), y = readFloat32(cloud.data, at + yo),
                     z = readFloat32(cloud.data, at + zo);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
          continue;
        const auto p = to_base * tf2::Vector3(x, y, z);
        const double r2 = p.x()*p.x() + p.y()*p.y();
        if (r2 >= 1.0 && r2 <= 144.0 && p.z() >= -0.8 && p.z() <= 1.0)
          candidates.push_back(p);
      }
    if (candidates.size() < 60)
      return false;  // No convincing road sample: preserve all observations.
    std::vector<tf2::Vector3> sample;
    const size_t stride = (candidates.size() + 1199) / 1200;
    for (size_t i = 0; i < candidates.size(); i += stride)
      sample.push_back(candidates[i]);

    auto plausible = [](const tf2::Vector3& n, double d) {
      const double height_at_chassis = -d / n.z();
      return n.z() >= 0.965925826 && height_at_chassis >= -0.4 && height_at_chassis <= 0.15;
    };
    std::uniform_int_distribution<size_t> pick(0, sample.size()-1);
    size_t best_count = 0;
    tf2::Vector3 best_normal;
    double best_offset = 0.0;
    for (int iteration = 0; iteration < 120; ++iteration)
    {
      const auto& a = sample[pick(random_)];
      const auto& b = sample[pick(random_)];
      const auto& c = sample[pick(random_)];
      auto n = (b-a).cross(c-a);
      if (n.length2() < 1e-8)
        continue;
      n.normalize();
      if (n.z() < 0.0) n = -n;
      if (n.z() < 0.965925826) continue;  // not a wall (maximum road tilt 15 deg)
      const double d = -n.dot(a);
      if (!plausible(n, d)) continue;  // not a raised tabletop/vehicle roof
      size_t count = 0;
      for (const auto& p : sample)
        if (std::abs(n.dot(p) + d) <= 0.04) ++count;
      if (count > best_count)
      {
        best_count = count;
        best_normal = n;
        best_offset = d;
      }
    }
    if (best_count < 60 || best_count * 2 < sample.size())
      return false;

    // Least-squares refinement of the robust inliers: z = ax + by + c.
    double xx=0, xy=0, x=0, yy=0, y=0, xz=0, yz=0, z=0, count=0;
    double min_x=1e9, max_x=-1e9, min_y=1e9, max_y=-1e9;
    for (const auto& p : sample)
      if (std::abs(best_normal.dot(p) + best_offset) <= 0.04)
      {
        xx += p.x()*p.x(); xy += p.x()*p.y(); x += p.x();
        yy += p.y()*p.y(); y += p.y(); xz += p.x()*p.z(); yz += p.y()*p.z(); z += p.z();
        ++count;
        min_x=std::min(min_x,p.x()); max_x=std::max(max_x,p.x());
        min_y=std::min(min_y,p.y()); max_y=std::max(max_y,p.y());
      }
    // A small planar object is insufficient evidence to redefine the road.
    if (max_x-min_x < 3.0 || max_y-min_y < 3.0)
      return false;
    const tf2::Matrix3x3 system(xx,xy,x, xy,yy,y, x,y,count);
    if (std::abs(system.determinant()) < 1e-6)
      return false;
    const auto solution = system.inverse() * tf2::Vector3(xz,yz,z);
    tf2::Vector3 n(-solution.x(),-solution.y(),1.0);
    const double scale = n.length(); n /= scale;
    const double d = -solution.z()/scale;
    if (!plausible(n,d)) return false;
    *normal=n; *offset=d;
    ROS_INFO_THROTTLE(15.0, "Local ground: normal=(%.4f,%.4f,%.4f) base_height=%.3f support=%zu/%zu; remove only +/-0.06 m surface band",
                      n.x(),n.y(),n.z(),-d/n.z(),best_count,sample.size());
    return true;
  }

  bool insideFootprint(double x, double y) const
  {
    bool inside = false;
    for (size_t i = 0, j = footprint_.size() - 1; i < footprint_.size(); j = i++)
    {
      const double xi = footprint_[i].first, yi = footprint_[i].second;
      const double xj = footprint_[j].first, yj = footprint_[j].second;
      // Include the physical edge despite float32 point-coordinate rounding.
      const double cross = (x - xi) * (yj - yi) - (y - yi) * (xj - xi);
      const double dot = (x - xi) * (x - xj) + (y - yi) * (y - yj);
      if (std::abs(cross) <= 1e-6 && dot <= 1e-6)
        return true;
      if ((yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi)
        inside = !inside;
    }
    return inside;
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    uint32_t x_offset = 0;
    uint32_t y_offset = 0;
    uint32_t z_offset = 0;
    if (!findFloat32Field(*msg, "x", &x_offset) || !findFloat32Field(*msg, "y", &y_offset) ||
        !findFloat32Field(*msg, "z", &z_offset))
    {
      ROS_WARN_THROTTLE(5.0, "pointcloud_self_filter: input cloud has no float32 x/y/z fields");
      return;
    }

    bool filter_footprint = footprint_.size() >= 3;
    bool have_mount_tf = true;
    tf2::Transform to_footprint;
    to_footprint.setIdentity();
    if ((filter_footprint || remove_ground_) && msg->header.frame_id != footprint_frame_)
    {
      try
      {
        // These are fixed sensor-to-chassis extrinsics. Use the latest mount
        // transform; navigation's existing projection handles timestamped TF.
        const auto transform = tf_buffer_.lookupTransform(footprint_frame_, msg->header.frame_id,
                                                         ros::Time(0)).transform;
        to_footprint.setOrigin(tf2::Vector3(transform.translation.x, transform.translation.y,
                                          transform.translation.z));
        to_footprint.setRotation(tf2::Quaternion(transform.rotation.x, transform.rotation.y,
                                               transform.rotation.z, transform.rotation.w));
      }
      catch (const tf2::TransformException& error)
      {
        // Keep the existing height/radius filtering while mount TF initializes.
        filter_footprint = false;
        have_mount_tf = false;
        ROS_WARN_THROTTLE(5.0, "pointcloud_self_filter: physical footprint TF unavailable: %s", error.what());
      }
    }

    sensor_msgs::PointCloud2 output = *msg;
    output.height = 1;
    output.width = 0;
    output.row_step = 0;
    output.is_dense = false;
    output.data.clear();
    output.data.reserve(msg->data.size());

    const double near_radius_sq = near_radius_ * near_radius_;
    tf2::Vector3 ground_normal;
    double ground_offset = 0.0;
    const bool filter_ground = remove_ground_ && have_mount_tf &&
        fitGroundPlane(*msg, to_footprint, x_offset, y_offset, z_offset, &ground_normal, &ground_offset);
    size_t kept = 0;

    for (uint32_t row = 0; row < msg->height; ++row)
    {
      const size_t row_offset = static_cast<size_t>(row) * msg->row_step;
      for (uint32_t col = 0; col < msg->width; ++col)
      {
        const size_t point_offset = row_offset + static_cast<size_t>(col) * msg->point_step;
        const float x = readFloat32(msg->data, point_offset + x_offset);
        const float y = readFloat32(msg->data, point_offset + y_offset);
        const float z = readFloat32(msg->data, point_offset + z_offset);

        if (remove_nan_ && (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)))
        {
          continue;
        }

        const bool above_self = z > remove_above_z_;
        const bool beyond_range = max_xy_range_ > 0.0 &&
            (static_cast<double>(x) * x + static_cast<double>(y) * y) > max_xy_range_ * max_xy_range_;
        const bool near_self_level = z >= near_min_z_ && z <= near_max_z_ &&
                                     (static_cast<double>(x) * x + static_cast<double>(y) * y) <= near_radius_sq;
        bool body_return = false;
        bool ground_return = false;
        if (filter_footprint || filter_ground)
        {
          const tf2::Vector3 point = to_footprint * tf2::Vector3(x, y, z);
          body_return = filter_footprint && insideFootprint(point.x(), point.y());
          ground_return = filter_ground && std::abs(ground_normal.dot(point) + ground_offset) <= 0.06;
        }
        if (above_self || beyond_range || near_self_level || body_return || ground_return)
        {
          continue;
        }

        output.data.insert(output.data.end(), msg->data.begin() + point_offset,
                           msg->data.begin() + point_offset + msg->point_step);
        ++kept;
      }
    }

    output.width = static_cast<uint32_t>(kept);
    output.row_step = output.point_step * output.width;
    publisher_.publish(output);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string input_topic_;
  std::string output_topic_;
  std::string footprint_frame_;
  std::vector<std::pair<double, double>> footprint_;
  double remove_above_z_;
  double near_radius_;
  double near_min_z_;
  double near_max_z_;
  double max_xy_range_;
  bool remove_nan_;
  bool remove_ground_;
  std::mt19937 random_{731};
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_self_filter");
  PointCloudSelfFilter filter;
  ros::spin();
  return 0;
}
