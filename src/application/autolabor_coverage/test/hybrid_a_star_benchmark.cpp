#include <autolabor_coverage/hybrid_a_star.h>

#include <costmap_2d/cost_values.h>
#include <ros/time.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{

constexpr double kPi = 3.14159265358979323846;
bool g_baseline_mode = false;
int g_max_expansions = 80000;
std::string g_real_map_path;
double g_real_map_origin_x = -9.1;
double g_real_map_origin_y = -52.6;
double g_real_map_resolution = 0.10;
double g_swath_spacing = 0.85;

struct CaseDefinition
{
  std::string name;
  std::string group;
  geometry_msgs::PoseStamped start;
  geometry_msgs::PoseStamped goal;
  bool allow_reverse = true;
  double forward_speed = 0.80;
  double timeout_sec = 0.80;
  bool expected_success = true;
  enum class MapKind
  {
    OPEN,
    INSIDE_BLOCK,
    WALL_GAP,
    REVERSE_CORRIDOR,
    DISCONNECTED,
    UNKNOWN_BARRIER,
    REAL_MAP
  } map_kind = MapKind::OPEN;
};

struct RunResult
{
  bool success = false;
  bool timed_out = false;
  double wall_ms = 0.0;
  std::size_t expansions = 0;
  double cost = std::numeric_limits<double>::quiet_NaN();
  double reverse_distance = std::numeric_limits<double>::quiet_NaN();
  unsigned int direction_changes = 0;
  double path_length = std::numeric_limits<double>::quiet_NaN();
  std::string reason;
  std::vector<geometry_msgs::PoseStamped> plan;
};

geometry_msgs::PoseStamped pose(double x, double y, double yaw)
{
  geometry_msgs::PoseStamped result;
  result.header.frame_id = "map";
  result.pose.position.x = x;
  result.pose.position.y = y;
  result.pose.orientation.z = std::sin(0.5 * yaw);
  result.pose.orientation.w = std::cos(0.5 * yaw);
  return result;
}

std::vector<geometry_msgs::Point> productionFootprint()
{
  std::vector<geometry_msgs::Point> result(4);
  result[0].x = 0.62;
  result[0].y = 0.45;
  result[1].x = 0.62;
  result[1].y = -0.45;
  result[2].x = -0.62;
  result[2].y = -0.45;
  result[3].x = -0.62;
  result[3].y = 0.45;
  return result;
}

void setRectangle(costmap_2d::Costmap2D& map,
                  double minimum_x, double minimum_y,
                  double maximum_x, double maximum_y,
                  unsigned char cost)
{
  for (unsigned int y = 0; y < map.getSizeInCellsY(); ++y)
  {
    for (unsigned int x = 0; x < map.getSizeInCellsX(); ++x)
    {
      double world_x = 0.0;
      double world_y = 0.0;
      map.mapToWorld(x, y, world_x, world_y);
      if (world_x >= minimum_x && world_x <= maximum_x &&
          world_y >= minimum_y && world_y <= maximum_y)
      {
        map.setCost(x, y, cost);
      }
    }
  }
}

std::unique_ptr<costmap_2d::Costmap2D> makeMap(const CaseDefinition& test_case)
{
  if (test_case.map_kind == CaseDefinition::MapKind::REAL_MAP)
  {
    std::ifstream input(g_real_map_path.c_str(), std::ios::in | std::ios::binary);
    if (!input)
      throw std::runtime_error("cannot open real-map PGM: " +
                               g_real_map_path);
    auto token = [&]() {
      std::string value;
      while (input >> value)
      {
        if (!value.empty() && value[0] == '#')
        {
          std::string ignored;
          std::getline(input, ignored);
          continue;
        }
        return value;
      }
      throw std::runtime_error("truncated real-map PGM header");
    };
    if (token() != "P5")
      throw std::runtime_error("real-map benchmark requires binary P5 PGM");
    const unsigned int width = static_cast<unsigned int>(std::stoul(token()));
    const unsigned int height = static_cast<unsigned int>(std::stoul(token()));
    const unsigned int maximum = static_cast<unsigned int>(std::stoul(token()));
    if (width == 0 || height == 0 || maximum != 255)
      throw std::runtime_error("unsupported real-map PGM geometry");
    input.get();
    std::vector<unsigned char> pixels(
        static_cast<std::size_t>(width) * height);
    input.read(reinterpret_cast<char*>(pixels.data()),
               static_cast<std::streamsize>(pixels.size()));
    if (input.gcount() != static_cast<std::streamsize>(pixels.size()))
      throw std::runtime_error("truncated real-map PGM pixels");
    std::unique_ptr<costmap_2d::Costmap2D> map(
        new costmap_2d::Costmap2D(width, height, g_real_map_resolution,
                                  g_real_map_origin_x,
                                  g_real_map_origin_y,
                                  costmap_2d::NO_INFORMATION));
    for (unsigned int map_y = 0; map_y < height; ++map_y)
    {
      const unsigned int image_y = height - 1u - map_y;
      for (unsigned int map_x = 0; map_x < width; ++map_x)
      {
        const unsigned char pixel = pixels[
            static_cast<std::size_t>(image_y) * width + map_x];
        const unsigned char cost = pixel >= 250u
            ? costmap_2d::FREE_SPACE
            : (pixel <= 100u ? costmap_2d::LETHAL_OBSTACLE
                             : costmap_2d::NO_INFORMATION);
        map->setCost(map_x, map_y, cost);
      }
    }
    return map;
  }

  const bool corridor =
      test_case.map_kind == CaseDefinition::MapKind::REVERSE_CORRIDOR;
  std::unique_ptr<costmap_2d::Costmap2D> map(new costmap_2d::Costmap2D(
      200, 200, 0.10, -10.0, -10.0,
      corridor ? costmap_2d::LETHAL_OBSTACLE : costmap_2d::FREE_SPACE));

  switch (test_case.map_kind)
  {
    case CaseDefinition::MapKind::OPEN:
      break;
    case CaseDefinition::MapKind::INSIDE_BLOCK:
      setRectangle(*map, 0.35, 0.55, 2.20, 1.15,
                   costmap_2d::LETHAL_OBSTACLE);
      break;
    case CaseDefinition::MapKind::WALL_GAP:
      setRectangle(*map, -0.15, -9.9, 0.15, 2.30,
                   costmap_2d::LETHAL_OBSTACLE);
      break;
    case CaseDefinition::MapKind::REVERSE_CORRIDOR:
      setRectangle(*map, -4.0, -0.75, 4.0, 0.75,
                   costmap_2d::FREE_SPACE);
      break;
    case CaseDefinition::MapKind::DISCONNECTED:
      setRectangle(*map, -0.20, -10.0, 0.20, 10.0,
                   costmap_2d::LETHAL_OBSTACLE);
      break;
    case CaseDefinition::MapKind::UNKNOWN_BARRIER:
      setRectangle(*map, -0.20, -10.0, 0.20, 10.0,
                   costmap_2d::NO_INFORMATION);
      break;
    case CaseDefinition::MapKind::REAL_MAP:
      break;
  }
  return map;
}

autolabor_coverage::HybridAStarConfig productionConfig(double timeout)
{
  autolabor_coverage::HybridAStarConfig config;
  config.minimum_turning_radius = 1.35;
  config.motion_step = 0.30;
  config.collision_check_step = 0.10;
  config.state_resolution = 0.15;
  config.heading_bins = 72;
  config.steering_samples = 5;
  config.max_expansions = g_max_expansions;
  config.planning_timeout = timeout;
  config.heuristic_weight = 1.05;
  config.steering_penalty = 0.04;
  config.steering_change_penalty = 0.10;
  config.obstacle_cost_scale = 0.25;
  if (g_baseline_mode)
  {
    config.use_nonholonomic_heuristic = false;
    config.use_obstacle_heuristic = false;
    config.use_analytic_expansion = false;
  }
  return config;
}

autolabor_coverage::HybridAStarProfile productionProfile(
    const CaseDefinition& test_case)
{
  autolabor_coverage::HybridAStarProfile profile;
  profile.allow_reverse = test_case.allow_reverse;
  profile.max_forward_speed = test_case.forward_speed;
  profile.max_reverse_speed = 0.30;
  profile.max_angular_speed = 0.60;
  profile.linear_acceleration = 2.00;
  profile.angular_acceleration = 0.50;
  profile.direction_change_penalty = 1.00;
  profile.goal_position_tolerance = 0.30;
  profile.goal_yaw_tolerance = 0.40;
  return profile;
}

double pathLength(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  double result = 0.0;
  for (std::size_t index = 1; index < plan.size(); ++index)
  {
    result += std::hypot(
        plan[index].pose.position.x - plan[index - 1].pose.position.x,
        plan[index].pose.position.y - plan[index - 1].pose.position.y);
  }
  return result;
}

RunResult runOnce(const CaseDefinition& test_case,
                  costmap_2d::Costmap2D& map,
                  double timeout)
{
  autolabor_coverage::HybridAStarPlanner planner;
  std::vector<geometry_msgs::PoseStamped> plan;
  autolabor_coverage::HybridAStarStatistics statistics;
  std::string reason;
  const auto started = std::chrono::steady_clock::now();
  const bool success = planner.makePlan(
      &map, productionFootprint(), test_case.start, test_case.goal,
      productionConfig(timeout), productionProfile(test_case), plan,
      statistics, reason);
  const auto stopped = std::chrono::steady_clock::now();

  RunResult result;
  result.success = success;
  result.timed_out = reason.find("timeout") != std::string::npos;
  result.wall_ms = std::chrono::duration<double, std::milli>(
      stopped - started).count();
  result.expansions = statistics.expansions;
  result.cost = statistics.estimated_time;
  result.reverse_distance = statistics.reverse_distance;
  result.direction_changes = statistics.direction_changes;
  result.path_length = pathLength(plan);
  result.reason = reason;
  result.plan = std::move(plan);
  return result;
}

void writePathCsv(const std::string& directory,
                  const CaseDefinition& test_case,
                  const RunResult& result)
{
  if (directory.empty() || result.plan.empty())
    return;
  std::ofstream output((directory + "/" + test_case.name + ".csv").c_str(),
                       std::ios::out | std::ios::trunc);
  if (!output)
    throw std::runtime_error("cannot write path artifact directory: " +
                             directory);
  output << "index,x,y,yaw\n";
  output << std::setprecision(12);
  for (std::size_t index = 0; index < result.plan.size(); ++index)
  {
    const geometry_msgs::PoseStamped& pose = result.plan[index];
    const double yaw = 2.0 * std::atan2(
        pose.pose.orientation.z, pose.pose.orientation.w);
    output << index << ',' << pose.pose.position.x << ','
           << pose.pose.position.y << ',' << yaw << '\n';
  }
}

std::vector<CaseDefinition> makeOpenCases(bool quick)
{
  const double spacing = g_swath_spacing;
  const std::vector<int> multiples = quick
      ? std::vector<int>{1, 2, 4}
      : std::vector<int>{1, 2, 3, 4};
  const std::vector<double> offsets = quick
      ? std::vector<double>{0.0}
      : std::vector<double>{-0.5 * spacing, 0.0, 0.5 * spacing};
  const std::vector<double> yaw_offsets = quick
      ? std::vector<double>{0.0}
      : std::vector<double>{-15.0 * kPi / 180.0, 0.0,
                            15.0 * kPi / 180.0};
  const std::vector<int> mirrors = quick
      ? std::vector<int>{1}
      : std::vector<int>{-1, 1};

  std::vector<CaseDefinition> cases;
  for (int multiple : multiples)
  {
    for (double offset : offsets)
    {
      for (double yaw_offset : yaw_offsets)
      {
        for (int mirror : mirrors)
        {
          CaseDefinition value;
          std::ostringstream name;
          name << "open_s" << multiple
               << "_x" << std::llround(offset * 100.0)
               << "_yaw" << std::llround(yaw_offset * 180.0 / kPi)
               << (mirror < 0 ? "_right" : "_left");
          value.name = name.str();
          value.group = "open";
          value.start = pose(0.0, 0.0, 0.0);
          value.goal = pose(offset, mirror * multiple * spacing,
                            mirror * (kPi + yaw_offset));
          value.allow_reverse = true;
          value.timeout_sec = 0.80;
          cases.push_back(value);
        }
      }
    }
  }

  if (!quick)
  {
    for (double speed : {0.80, 1.20})
    {
      for (bool allow_reverse : {false, true})
      {
        CaseDefinition value;
        std::ostringstream name;
        name << "profile_s2_v" << std::llround(speed * 10.0)
             << (allow_reverse ? "_reverse" : "_forward");
        value.name = name.str();
        value.group = "open_profile";
        value.start = pose(0.0, 0.0, 0.0);
        value.goal = pose(0.0, 2.0 * spacing, kPi);
        value.allow_reverse = allow_reverse;
        value.forward_speed = speed;
        value.timeout_sec = 0.80;
        cases.push_back(value);
      }
    }
  }
  return cases;
}

std::vector<CaseDefinition> makeObstacleCases()
{
  std::vector<CaseDefinition> cases;

  CaseDefinition inside;
  inside.name = "obstacle_inside_turn";
  inside.group = "obstacle";
  inside.start = pose(0.0, 0.0, 0.0);
  inside.goal = pose(0.0, 1.70, kPi);
  inside.map_kind = CaseDefinition::MapKind::INSIDE_BLOCK;
  inside.timeout_sec = 1.50;
  cases.push_back(inside);

  CaseDefinition wall;
  wall.name = "obstacle_wall_gap";
  wall.group = "obstacle";
  wall.start = pose(-3.0, -1.0, 0.0);
  wall.goal = pose(3.0, 1.0, kPi);
  wall.map_kind = CaseDefinition::MapKind::WALL_GAP;
  wall.timeout_sec = 1.50;
  cases.push_back(wall);

  CaseDefinition corridor;
  corridor.name = "obstacle_reverse_corridor";
  corridor.group = "obstacle";
  corridor.start = pose(0.0, 0.0, 0.0);
  corridor.goal = pose(-1.50, 0.0, 0.0);
  corridor.map_kind = CaseDefinition::MapKind::REVERSE_CORRIDOR;
  corridor.timeout_sec = 1.50;
  cases.push_back(corridor);

  CaseDefinition disconnected;
  disconnected.name = "blocked_lethal_wall";
  disconnected.group = "blocked";
  disconnected.start = pose(-2.0, 0.0, 0.0);
  disconnected.goal = pose(2.0, 0.0, 0.0);
  disconnected.map_kind = CaseDefinition::MapKind::DISCONNECTED;
  disconnected.expected_success = false;
  disconnected.timeout_sec = 1.50;
  cases.push_back(disconnected);

  CaseDefinition unknown = disconnected;
  unknown.name = "blocked_unknown_wall";
  unknown.map_kind = CaseDefinition::MapKind::UNKNOWN_BARRIER;
  cases.push_back(unknown);
  return cases;
}

std::vector<CaseDefinition> makeRealMapCases()
{
  if (g_real_map_path.empty())
    return {};
  std::vector<CaseDefinition> cases;
  auto append = [&](const std::string& name,
                    double first_x, double first_y, double first_yaw,
                    double second_x, double second_y, double second_yaw) {
    CaseDefinition value;
    value.name = name;
    value.group = "real_map";
    value.start = pose(first_x, first_y, first_yaw);
    value.goal = pose(second_x, second_y, second_yaw);
    value.map_kind = CaseDefinition::MapKind::REAL_MAP;
    value.timeout_sec = 1.50;
    cases.push_back(value);
  };
  // Adjacent 0.85 m swath-end transitions sampled inside saved C区/A区.
  append("real_C_north_left", 56.0, 6.0, 0.5 * kPi,
         56.85, 6.0, -0.5 * kPi);
  append("real_C_south_right", 60.0, 0.0, -0.5 * kPi,
         60.85, 0.0, 0.5 * kPi);
  append("real_A_north_left", 15.5, -36.0, 0.5 * kPi,
         16.35, -36.0, -0.5 * kPi);
  append("real_A_south_right", 16.0, -42.0, -0.5 * kPi,
         16.85, -42.0, 0.5 * kPi);
  // Exact first cross-region connector captured by the isolated A->C
  // Gazebo comparison run.  It is intentionally long enough to expose
  // whether a nominal 1 Hz full-distance Hybrid replan can meet its budget.
  append("real_A_to_C_entry", 17.2250206759, -35.2757006389, 1.536,
         55.9282096866, -0.3589923978, 0.0);
  return cases;
}

double percentile(std::vector<double> values, double quantile)
{
  if (values.empty())
    return std::numeric_limits<double>::quiet_NaN();
  std::sort(values.begin(), values.end());
  const std::size_t index = static_cast<std::size_t>(std::ceil(
      quantile * static_cast<double>(values.size()))) - 1u;
  return values[std::min(index, values.size() - 1u)];
}

std::string csvEscape(const std::string& value)
{
  std::string escaped = "\"";
  for (char character : value)
  {
    if (character == '\"')
      escaped += '\"';
    escaped += character;
  }
  escaped += "\"";
  return escaped;
}

int parsePositive(const char* value, const std::string& label)
{
  const int parsed = std::atoi(value);
  if (parsed <= 0)
    throw std::runtime_error(label + " must be positive");
  return parsed;
}

double parseFinite(const char* value, const std::string& label)
{
  const double parsed = std::strtod(value, nullptr);
  if (!std::isfinite(parsed))
    throw std::runtime_error(label + " must be finite");
  return parsed;
}

}  // namespace

int main(int argc, char** argv)
{
  ros::Time::init();
  bool quick = false;
  bool oracle = false;
  int warmups = 3;
  int repetitions = 20;
  std::string output_path;
  std::string case_filter;
  std::string path_directory;
  for (int index = 1; index < argc; ++index)
  {
    const std::string argument(argv[index]);
    if (argument == "--quick")
      quick = true;
    else if (argument == "--oracle")
      oracle = true;
    else if (argument == "--baseline")
      g_baseline_mode = true;
    else if (argument == "--warmups" && index + 1 < argc)
      warmups = parsePositive(argv[++index], "warmups");
    else if (argument == "--repetitions" && index + 1 < argc)
      repetitions = parsePositive(argv[++index], "repetitions");
    else if (argument == "--output" && index + 1 < argc)
      output_path = argv[++index];
    else if (argument == "--case" && index + 1 < argc)
      case_filter = argv[++index];
    else if (argument == "--path-dir" && index + 1 < argc)
      path_directory = argv[++index];
    else if (argument == "--real-map" && index + 1 < argc)
      g_real_map_path = argv[++index];
    else if (argument == "--real-map-origin-x" && index + 1 < argc)
      g_real_map_origin_x = parseFinite(argv[++index], "real-map origin x");
    else if (argument == "--real-map-origin-y" && index + 1 < argc)
      g_real_map_origin_y = parseFinite(argv[++index], "real-map origin y");
    else if (argument == "--real-map-resolution" && index + 1 < argc)
      g_real_map_resolution = parseFinite(argv[++index],
                                          "real-map resolution");
    else if (argument == "--swath-spacing" && index + 1 < argc)
    {
      g_swath_spacing = parseFinite(argv[++index], "swath spacing");
      if (g_swath_spacing <= 0.0)
        throw std::runtime_error("swath spacing must be positive");
    }
    else
      throw std::runtime_error("unknown or incomplete argument: " + argument);
  }

  std::vector<CaseDefinition> cases = makeOpenCases(quick);
  const std::vector<CaseDefinition> obstacle_cases = makeObstacleCases();
  cases.insert(cases.end(), obstacle_cases.begin(), obstacle_cases.end());
  const std::vector<CaseDefinition> real_map_cases = makeRealMapCases();
  cases.insert(cases.end(), real_map_cases.begin(), real_map_cases.end());
  if (!case_filter.empty())
  {
    cases.erase(std::remove_if(
        cases.begin(), cases.end(), [&](const CaseDefinition& test_case) {
          return test_case.name.find(case_filter) == std::string::npos;
        }), cases.end());
    if (cases.empty())
      throw std::runtime_error("case filter matched no benchmark cases");
  }
  if (oracle)
    g_max_expansions = 1000000;

  std::ofstream output_file;
  std::ostream* output = &std::cout;
  if (!output_path.empty())
  {
    output_file.open(output_path.c_str(), std::ios::out | std::ios::trunc);
    if (!output_file)
      throw std::runtime_error("cannot open output path: " + output_path);
    output = &output_file;
  }
  *output << "case,group,expected_success,budget_ms,runs,successes,timeouts,"
             "median_ms,p95_ms,max_ms,median_expansions,reference_cost,"
             "reverse_distance,direction_changes,path_length,reason\n";

  bool expectation_failure = false;
  for (const CaseDefinition& test_case : cases)
  {
    std::unique_ptr<costmap_2d::Costmap2D> map = makeMap(test_case);
    const double timeout = oracle ? 30.0 : test_case.timeout_sec;
    for (int iteration = 0; iteration < warmups; ++iteration)
      runOnce(test_case, *map, timeout);

    std::vector<RunResult> results;
    results.reserve(repetitions);
    for (int iteration = 0; iteration < repetitions; ++iteration)
      results.push_back(runOnce(test_case, *map, timeout));

    std::vector<double> times;
    std::vector<double> expansions;
    int successes = 0;
    int timeouts = 0;
    const RunResult* reference = nullptr;
    for (const RunResult& result : results)
    {
      times.push_back(result.wall_ms);
      expansions.push_back(static_cast<double>(result.expansions));
      successes += result.success ? 1 : 0;
      timeouts += result.timed_out ? 1 : 0;
      if (!reference || (result.success && !reference->success))
        reference = &result;
    }
    if (!reference)
      throw std::runtime_error("benchmark produced no run result");
    writePathCsv(path_directory, test_case, *reference);

    const bool observed_expected = test_case.expected_success
        ? successes == repetitions
        : successes == 0;
    if (oracle && !observed_expected)
      expectation_failure = true;

    *output << csvEscape(test_case.name) << ','
            << csvEscape(test_case.group) << ','
            << (test_case.expected_success ? 1 : 0) << ','
            << std::fixed << std::setprecision(3) << timeout * 1000.0 << ','
            << repetitions << ',' << successes << ',' << timeouts << ','
            << percentile(times, 0.50) << ','
            << percentile(times, 0.95) << ','
            << *std::max_element(times.begin(), times.end()) << ','
            << percentile(expansions, 0.50) << ','
            << reference->cost << ',' << reference->reverse_distance << ','
            << reference->direction_changes << ',' << reference->path_length
            << ',' << csvEscape(reference->reason) << '\n';
  }
  output->flush();
  return expectation_failure ? 2 : 0;
}
