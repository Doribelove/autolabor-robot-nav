#include <cmath>
#include <cstring>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

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
  PointCloudSelfFilter() : private_nh_("~")
  {
    private_nh_.param<std::string>("input_topic", input_topic_, "/cloud_registered_body");
    private_nh_.param<std::string>("output_topic", output_topic_, "/cloud_filtered_for_scan");
    private_nh_.param("remove_above_z", remove_above_z_, 0.1);
    private_nh_.param("near_radius", near_radius_, 0.4);
    private_nh_.param("near_min_z", near_min_z_, -0.1);
    private_nh_.param("near_max_z", near_max_z_, 0.1);
    private_nh_.param("remove_nan", remove_nan_, true);

    const int queue_size = private_nh_.param("queue_size", 5);

    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, queue_size);
    subscriber_ = nh_.subscribe(input_topic_, queue_size, &PointCloudSelfFilter::cloudCallback, this);

    ROS_INFO_STREAM("pointcloud_self_filter: " << input_topic_ << " -> " << output_topic_
                    << ", remove z > " << remove_above_z_ << " m, remove radius <= "
                    << near_radius_ << " m for " << near_min_z_ << " <= z <= " << near_max_z_ << " m");
  }

private:
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

    sensor_msgs::PointCloud2 output = *msg;
    output.height = 1;
    output.width = 0;
    output.row_step = 0;
    output.is_dense = false;
    output.data.clear();
    output.data.reserve(msg->data.size());

    const double near_radius_sq = near_radius_ * near_radius_;
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
        const bool near_self_level = z >= near_min_z_ && z <= near_max_z_ &&
                                     (static_cast<double>(x) * x + static_cast<double>(y) * y) <= near_radius_sq;
        if (above_self || near_self_level)
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

  std::string input_topic_;
  std::string output_topic_;
  double remove_above_z_;
  double near_radius_;
  double near_min_z_;
  double near_max_z_;
  bool remove_nan_;
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_self_filter");
  PointCloudSelfFilter filter;
  ros::spin();
  return 0;
}
