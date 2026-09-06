// Isolated regression fixture for the installed ROS1 CameraPublisher library.
// No device access or vehicle topics. Run only on the dedicated loopback master.
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cstdlib>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
  const char* master = std::getenv("ROS_MASTER_URI");
  if (!master || std::string(master) != "http://127.0.0.1:11489")
  {
    std::cerr << "Refusing non-fixture ROS master\n";
    return 2;
  }
  ros::M_string remaps;
  for (const std::string mode : {"legacy", "candidate"})
  {
    const std::string root = "/alias_fixture_" + mode;
    remaps[root + "/rgb/image_rect_color"] = root + "/fod/image_raw";
    remaps[root + "/depth/depth_registered"] = root + "/fod/depth_registered";
    if (mode == "legacy")
      remaps[root + "/rgb/camera_info"] = root + "/fod/camera_info";
  }
  ros::init(remaps, "candidate_camera_alias_fixture");
  bool all_ok = true;
  for (const std::string mode : {"legacy", "candidate"})
  {
    const std::string root = "/alias_fixture_" + mode;
    ros::NodeHandle nh(root);
    image_transport::ImageTransport it(nh);
    auto rgb = it.advertiseCamera("rgb/image_rect_color", 10);
    auto gray = it.advertiseCamera("rgb/image_rect_gray", 10);
    auto depth = it.advertiseCamera("depth/depth_registered", 10);
    int count = 0;
    bool unchanged = true;
    auto subscriber = nh.subscribe<sensor_msgs::CameraInfo>(
        "fod/camera_info", 100,
        [&count, &unchanged](const sensor_msgs::CameraInfo::ConstPtr& msg) {
          ++count;
          unchanged = unchanged && msg->width == 1 && msg->height == 1 &&
                      msg->K[0] == 262.5 && msg->K[4] == 262.5 &&
                      msg->header.frame_id == "fixture_optical_frame";
        });
    for (int i = 0; i < 100; ++i)
    {
      ros::spinOnce();
      ros::WallDuration(0.01).sleep();
    }
    const bool gray_demand = gray.getNumSubscribers() > 0;
    const bool endpoints_ok = rgb.getInfoTopic() == root + "/fod/camera_info" &&
                              depth.getInfoTopic() == root + "/fod/camera_info";
    sensor_msgs::Image image;
    image.width = image.height = 1;
    image.encoding = "mono8";
    image.step = 1;
    image.data = {42};
    sensor_msgs::CameraInfo calibration;
    calibration.width = calibration.height = 1;
    calibration.K[0] = calibration.K[4] = 262.5;
    calibration.K[8] = 1.0;
    calibration.header.frame_id = image.header.frame_id = "fixture_optical_frame";
    for (int i = 0; i < 5; ++i)
    {
      calibration.header.stamp = image.header.stamp = ros::Time::now();
      rgb.publish(image, calibration);
      depth.publish(image, calibration);
      // Mirrors the wrapper's demand-driven gray retrieve/publish condition.
      if (gray_demand)
        gray.publish(image, calibration);
      for (int j = 0; j < 10; ++j)
      {
        ros::spinOnce();
        ros::WallDuration(0.01).sleep();
      }
    }
    const int expected = mode == "legacy" ? 15 : 10;
    const bool ok = endpoints_ok && unchanged && count == expected &&
                    gray_demand == (mode == "legacy");
    std::cout << mode << " camera_info=" << count
              << " expected=" << expected << " gray_demand=" << gray_demand
              << " rgb_depth_info_endpoints_unchanged=" << endpoints_ok
              << " calibration_unchanged=" << unchanged << " pass=" << ok << '\n';
    all_ok = all_ok && ok;
  }
  return all_ok ? 0 : 1;
}
