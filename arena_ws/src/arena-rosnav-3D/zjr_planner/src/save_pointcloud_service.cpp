// save_pointcloud_service.cpp
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "zjr_planner/SaveScan.h" // 生成后的 srv 头 (包名按实际修改)
#include <fstream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <string>
#include <sstream>
#include <filesystem>
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;
using json = nlohmann::json;

struct Point2D {
  double x;
  double y;
};

static std::string make_save_dir_path() {
  const char* home = std::getenv("HOME");
  std::string base;
  if (home) base = std::string(home);
  else base = ".";
  base += "/catkin_arena/src/zjr_planner/scripts/data_json";
  return base;
}

static std::string create_file_for_index(int index) {
  std::string dir = make_save_dir_path();
  try {
    fs::create_directories(dir);
  } catch (std::exception &e) {
    ROS_WARN("Failed to create directories: %s", e.what());
  }
  std::ostringstream ss;
  ss << dir << "/data_start_" << index << ".json";
  std::string file_path = ss.str();

  if (!fs::exists(file_path)) {
    // create empty JSON {}
    std::ofstream ofs(file_path);
    if (ofs) {
      ofs << "{}";
      ofs.close();
    } else {
      ROS_WARN("Unable to create file %s", file_path.c_str());
    }
  }
  return file_path;
}

static std::vector<Point2D> ranges_to_pointcloud(const sensor_msgs::LaserScan::ConstPtr &scan) {
  std::vector<Point2D> points;
  double angle = scan->angle_min;
  for (double r : scan->ranges) {
    if (!std::isfinite(r) || r <= scan->range_min || r >= scan->range_max) {
      angle += scan->angle_increment;
      continue;
    }
    double x = std::round((r * std::cos(angle)) * 100.0) / 100.0;
    double y = std::round((r * std::sin(angle)) * 100.0) / 100.0;
    points.push_back({x, y});
    angle += scan->angle_increment;
  }
  return points;
}

static std::vector<std::vector<int>> build_grid_map_from_points(
    const std::vector<Point2D> &points,
    double grid_resolution,
    int width,
    int height)
{
  std::vector<std::vector<int>> grid_map(height, std::vector<int>(width, 0));
  double half_w = static_cast<double>(width) / 2.0;
  double half_h = static_cast<double>(height) / 2.0;

  for (const auto &p : points) {
    int grid_x = static_cast<int>(std::floor(p.x / grid_resolution + half_w));
    int grid_y = static_cast<int>(std::floor(p.y / grid_resolution + half_h));
    if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height) {
      grid_map[grid_y][grid_x] = 100;
    }
  }
  return grid_map;
}

static bool save_pointcloud_grid_to_file(const std::string &file_path, int index, const std::vector<std::vector<int>> &grid_map, std::string &out_message) {
  std::string key_name = "pointcloud" + std::to_string(index);

  json data;
  // read existing file if possible
  try {
    std::ifstream ifs(file_path);
    if (ifs && ifs.peek() != std::ifstream::traits_type::eof()) {
      ifs >> data;
      ifs.close();
    } else {
      data = json::object();
    }
  } catch (std::exception &e) {
    ROS_WARN("Error reading JSON file: %s", e.what());
    data = json::object();
  }

  // convert grid_map to JSON array
  json grid_json = json::array();
  for (const auto &row : grid_map) {
    json jrow = json::array();
    for (int v : row) jrow.push_back(v);
    grid_json.push_back(jrow);
  }

  data[key_name] = json::object();
  data[key_name]["grid_map"] = grid_json;

  // write back (compact)
  try {
    std::ofstream ofs(file_path);
    ofs << data.dump() ;
    ofs.close();
  } catch (std::exception &e) {
    out_message = std::string("Failed to write JSON: ") + e.what();
    return false;
  }
  out_message = file_path;
  return true;
}

bool handle_save_request(zjr_planner::SaveScan::Request &req, zjr_planner::SaveScan::Response &res) {
  ros::NodeHandle nh;
  // use topic from request (or default "scan" if empty)
  std::string scan_topic = req.scan_topic.empty() ? "scan" : req.scan_topic;
  double timeout = (req.timeout <= 0.0) ? 10.0 : static_cast<double>(req.timeout);

  // wait for message (blocking) - same behavior as rospy.wait_for_message
  ROS_INFO("Waiting for one LaserScan message on topic '%s' (timeout %.2f s)...", scan_topic.c_str(), timeout);
  sensor_msgs::LaserScan::ConstPtr scan_msg;
  try {
    scan_msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>(scan_topic, ros::Duration(timeout));
  } catch (std::exception &e) {
    ROS_ERROR("Exception when waiting for LaserScan: %s", e.what());
  }

  if (!scan_msg) {
    res.success = false;
    res.message = std::string("No LaserScan received on ") + scan_topic + " within timeout";
    ROS_WARN("%s", res.message.c_str());
    return true;
  }

  // transform ranges -> points
  auto points = ranges_to_pointcloud(scan_msg);

  // build grid map
  int width = req.width <= 0 ? 100 : req.width;
  int height = req.height <= 0 ? 100 : req.height;
  double resolution = req.resolution <= 0.0 ? 0.1 : static_cast<double>(req.resolution);

  auto grid_map = build_grid_map_from_points(points, resolution, width, height);

  // create file path & save
  std::string file_path = create_file_for_index(req.index);
  std::string save_msg;
  bool ok = save_pointcloud_grid_to_file(file_path, req.index, grid_map, save_msg);
  res.success = ok;
  res.message = save_msg;
  if (ok) ROS_INFO("Saved pointcloud to %s", save_msg.c_str());
  else ROS_ERROR("Failed to save pointcloud: %s", save_msg.c_str());
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pointcloud_generate_once"); // 与 Python 节点名一致
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("save_pointcloud", handle_save_request);
  ROS_INFO("Service 'save_pointcloud' ready. Call to save one LaserScan -> grid JSON.");
  ros::spin();
  return 0;
}

