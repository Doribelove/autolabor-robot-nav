#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <base_local_planner/costmap_model.h>
#include <actionlib_msgs/GoalID.h>
#include <boost/thread/locks.hpp>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/footprint.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/recovery_behavior.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <robot_bringup/ackermann_recovery_geometry.h>
#include <robot_bringup/ackermann_recovery_interrupt_gate.h>

namespace robot_bringup
{

namespace recovery_detail
{

ArcPose integrateArcStep(const ArcPose& pose, double signed_distance,
                         double curvature)
{
  ArcPose result = pose;
  if (std::abs(curvature) < 1e-9)
  {
    result.x += signed_distance * std::cos(pose.yaw);
    result.y += signed_distance * std::sin(pose.yaw);
    return result;
  }

  result.yaw = pose.yaw + signed_distance * curvature;
  result.x += (std::sin(result.yaw) - std::sin(pose.yaw)) / curvature;
  result.y += (-std::cos(result.yaw) + std::cos(pose.yaw)) / curvature;
  return result;
}

bool footprintInteriorIsSafe(costmap_2d::Costmap2D& costmap,
                             const std::vector<geometry_msgs::Point>& oriented_footprint,
                             unsigned char maximum_allowed_cost,
                             unsigned char* observed_maximum_cost)
{
  if (oriented_footprint.size() < 3)
  {
    return false;
  }

  std::vector<costmap_2d::MapLocation> polygon;
  polygon.reserve(oriented_footprint.size());
  for (const geometry_msgs::Point& point : oriented_footprint)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y))
    {
      return false;
    }
    costmap_2d::MapLocation location;
    if (!costmap.worldToMap(point.x, point.y, location.x, location.y))
    {
      return false;
    }
    polygon.push_back(location);
  }

  std::vector<costmap_2d::MapLocation> cells;
  costmap.convexFillCells(polygon, cells);
  if (cells.empty())
  {
    return false;
  }

  unsigned char maximum_seen = costmap_2d::FREE_SPACE;
  for (const costmap_2d::MapLocation& cell : cells)
  {
    const unsigned char cost = costmap.getCost(cell.x, cell.y);
    if (cost == costmap_2d::NO_INFORMATION ||
        cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
        cost > maximum_allowed_cost)
    {
      return false;
    }
    maximum_seen = std::max(maximum_seen, cost);
  }

  if (observed_maximum_cost != nullptr)
  {
    *observed_maximum_cost = maximum_seen;
  }
  return true;
}

}  // namespace recovery_detail

namespace
{

struct TrajectoryEvaluation
{
  bool safe{false};
  double maximum_cost{std::numeric_limits<double>::infinity()};
};

// The reverse and forward behaviors are separate plugin instances. Remember
// the last steering side so equally safe consecutive arcs do not simply retrace
// one another and return the vehicle to its original pose.
std::atomic<int> last_selected_turn_sign{0};

double clampPositive(double value, double fallback, const std::string& parameter)
{
  if (std::isfinite(value) && value > 0.0)
  {
    return value;
  }
  ROS_ERROR("AckermannArcRecovery: parameter %s must be finite and > 0; using %.3f",
            parameter.c_str(), fallback);
  return fallback;
}

}  // namespace

class AckermannArcRecovery : public nav_core::RecoveryBehavior
{
public:
  AckermannArcRecovery() = default;

  ~AckermannArcRecovery() override
  {
    publishStop();
  }

  void initialize(std::string name, tf2_ros::Buffer*,
                  costmap_2d::Costmap2DROS*,
                  costmap_2d::Costmap2DROS* local_costmap) override
  {
    if (initialized_)
    {
      ROS_WARN("AckermannArcRecovery %s was initialized more than once", name_.c_str());
      return;
    }

    name_ = std::move(name);
    local_costmap_ = local_costmap;
    if (local_costmap_ == nullptr || local_costmap_->getCostmap() == nullptr)
    {
      ROS_ERROR("AckermannArcRecovery %s has no local costmap", name_.c_str());
      return;
    }

    ros::NodeHandle private_nh("~/" + name_);
    private_nh.param("direction", direction_, -1);
    private_nh.param("linear_speed", linear_speed_, 0.30);
    private_nh.param("acceleration_limit", acceleration_limit_, 0.60);
    private_nh.param("min_turning_radius", min_turning_radius_, 1.30);
    private_nh.param("max_angular_speed", max_angular_speed_, 0.24);
    private_nh.param("max_distance", max_distance_, 0.55);
    private_nh.param("max_duration", max_duration_, 4.0);
    private_nh.param("frequency", frequency_, 20.0);
    private_nh.param("command_hold_timeout", command_hold_timeout_, 0.20);
    private_nh.param("pre_run_interrupt_timeout", pre_run_interrupt_timeout_, 0.50);
    private_nh.param("sim_granularity", sim_granularity_, 0.05);
    private_nh.param("safety_lookahead", safety_lookahead_, 0.30);
    private_nh.param("max_pose_jump", max_pose_jump_, 0.50);
    private_nh.param("max_footprint_cost", max_footprint_cost_, 252.0);
    private_nh.param<std::string>("preferred_turn", preferred_turn_, "auto");
    private_nh.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "cmd_vel");
    private_nh.param<std::string>("cancel_topic", cancel_topic_, "/move_base/cancel");
    private_nh.param<std::string>("action_goal_topic", action_goal_topic_, "/move_base/goal");

    if (direction_ != -1 && direction_ != 1)
    {
      ROS_ERROR("AckermannArcRecovery %s: direction must be -1 or 1; using -1",
                name_.c_str());
      direction_ = -1;
    }
    linear_speed_ = clampPositive(linear_speed_, 0.30, name_ + "/linear_speed");
    acceleration_limit_ =
        clampPositive(acceleration_limit_, 0.60, name_ + "/acceleration_limit");
    min_turning_radius_ =
        clampPositive(min_turning_radius_, 1.30, name_ + "/min_turning_radius");
    max_angular_speed_ =
        clampPositive(max_angular_speed_, 0.24, name_ + "/max_angular_speed");
    max_distance_ = clampPositive(max_distance_, 0.55, name_ + "/max_distance");
    max_duration_ = clampPositive(max_duration_, 4.0, name_ + "/max_duration");
    frequency_ = clampPositive(frequency_, 20.0, name_ + "/frequency");
    command_hold_timeout_ =
        clampPositive(command_hold_timeout_, 0.20, name_ + "/command_hold_timeout");
    pre_run_interrupt_timeout_ = clampPositive(
        pre_run_interrupt_timeout_, 0.50, name_ + "/pre_run_interrupt_timeout");
    sim_granularity_ =
        clampPositive(sim_granularity_, 0.05, name_ + "/sim_granularity");
    safety_lookahead_ =
        clampPositive(safety_lookahead_, 0.30, name_ + "/safety_lookahead");
    max_pose_jump_ = clampPositive(max_pose_jump_, 0.50, name_ + "/max_pose_jump");

    std::transform(preferred_turn_.begin(), preferred_turn_.end(), preferred_turn_.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (preferred_turn_ != "auto" && preferred_turn_ != "left" &&
        preferred_turn_ != "right")
    {
      ROS_WARN("AckermannArcRecovery %s: preferred_turn must be auto, left, or right; using auto",
               name_.c_str());
      preferred_turn_ = "auto";
    }
    if (!std::isfinite(max_footprint_cost_) || max_footprint_cost_ < 0.0 ||
        max_footprint_cost_ > 252.0)
    {
      ROS_WARN("AckermannArcRecovery %s: max_footprint_cost must be in [0, 252]; using 252",
               name_.c_str());
      max_footprint_cost_ = 252.0;
    }

    footprint_ = local_costmap_->getRobotFootprint();
    if (footprint_.size() < 3)
    {
      ROS_ERROR("AckermannArcRecovery %s requires a polygon robot footprint", name_.c_str());
      return;
    }
    costmap_2d::calculateMinAndMaxDistances(footprint_, inscribed_radius_,
                                           circumscribed_radius_);
    world_model_.reset(new base_local_planner::CostmapModel(
        *local_costmap_->getCostmap()));

    ros::NodeHandle root_nh;
    cmd_vel_pub_ = root_nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    cancel_sub_ = root_nh.subscribe(cancel_topic_, 10,
                                    &AckermannArcRecovery::cancelCallback, this);
    action_goal_sub_ = root_nh.subscribe(action_goal_topic_, 10,
                                         &AckermannArcRecovery::actionGoalCallback, this);
    initialized_ = true;

    ROS_INFO("AckermannArcRecovery %s: direction=%d speed=%.2f m/s radius>=%.2f m "
             "distance<=%.2f m duration<=%.2f s cmd=%s",
             name_.c_str(), direction_, linear_speed_, min_turning_radius_,
             max_distance_, max_duration_, cmd_vel_pub_.getTopic().c_str());
  }

  void runBehavior() override
  {
    // A stop is published on every exit path, including exceptions and early
    // returns. New cancel/goal messages can also publish stop immediately from
    // the ROS callback thread while this synchronous behavior is running.
    struct StopGuard
    {
      explicit StopGuard(AckermannArcRecovery* owner) : owner_(owner)
      {
        owner_->beginRun();
      }
      ~StopGuard()
      {
        owner_->endRun();
        owner_->publishStop();
      }
      AckermannArcRecovery* owner_;
    } stop_guard(this);
    (void)stop_guard;

    if (!initialized_)
    {
      ROS_ERROR("AckermannArcRecovery cannot run before successful initialization");
      return;
    }
    publishStop();

    if (abort_requested_.load())
    {
      ROS_WARN("AckermannArcRecovery %s refused to move: cancel/preempt arrived "
               "immediately before recovery started",
               name_.c_str());
      return;
    }

    if (!local_costmap_->isCurrent())
    {
      ROS_WARN("AckermannArcRecovery %s refused to move: local costmap is stale",
               name_.c_str());
      return;
    }
    if (cmd_vel_pub_.getNumSubscribers() == 0)
    {
      ROS_WARN("AckermannArcRecovery %s refused to move: %s has no subscriber",
               name_.c_str(), cmd_vel_pub_.getTopic().c_str());
      return;
    }

    geometry_msgs::PoseStamped start_pose;
    if (!local_costmap_->getRobotPose(start_pose))
    {
      ROS_WARN("AckermannArcRecovery %s refused to move: robot pose unavailable",
               name_.c_str());
      return;
    }

    // Curvature is bounded both by the configured hardware turning radius and
    // by the angular-velocity cap. angular.z remains standard Twist yaw rate:
    // omega = signed_linear_velocity * curvature.
    const double curvature_magnitude =
        std::min(1.0 / min_turning_radius_, max_angular_speed_ / linear_speed_);
    const TrajectoryEvaluation left =
        evaluateTrajectory(start_pose, curvature_magnitude, max_distance_);
    const TrajectoryEvaluation right =
        evaluateTrajectory(start_pose, -curvature_magnitude, max_distance_);

    const double curvature = selectCurvature(left, right, curvature_magnitude);
    if (curvature == 0.0)
    {
      ROS_WARN("AckermannArcRecovery %s refused to move: neither complete arc is "
               "collision-free in the local costmap",
               name_.c_str());
      return;
    }
    last_selected_turn_sign.store(curvature > 0.0 ? 1 : -1);

    const char* motion = direction_ > 0 ? "forward" : "reverse";
    const char* turn = curvature > 0.0 ? "left" : "right";
    ROS_WARN("AckermannArcRecovery %s executing %s-%s arc", name_.c_str(), motion, turn);

    geometry_msgs::PoseStamped previous_pose = start_pose;
    double measured_distance = 0.0;
    double commanded_distance = 0.0;
    double speed_magnitude = 0.0;
    double held_speed_magnitude = 0.0;
    const ros::WallTime start_wall = ros::WallTime::now();
    ros::WallTime command_accounted_at = start_wall;
    ros::WallTime speed_updated_at = start_wall;
    ros::WallRate rate(frequency_);

    const auto accountHeldCommand = [&](const ros::WallTime& now) {
      const double held_duration = (now - command_accounted_at).toSec();
      if (std::isfinite(held_duration) && held_duration > 0.0)
      {
        commanded_distance += held_speed_magnitude * held_duration;
      }
      command_accounted_at = now;
    };

    while (ros::ok())
    {
      if (abort_requested_.load())
      {
        ROS_WARN("AckermannArcRecovery %s stopped by navigation cancel/preempt",
                 name_.c_str());
        break;
      }
      const ros::WallTime loop_wall = ros::WallTime::now();
      accountHeldCommand(loop_wall);
      const double elapsed = (loop_wall - start_wall).toSec();
      if (elapsed >= max_duration_)
      {
        ROS_WARN("AckermannArcRecovery %s stopped at its %.2f s timeout",
                 name_.c_str(), max_duration_);
        break;
      }
      if (!local_costmap_->isCurrent())
      {
        ROS_WARN("AckermannArcRecovery %s stopped: local costmap became stale",
                 name_.c_str());
        break;
      }

      geometry_msgs::PoseStamped current_pose;
      if (!local_costmap_->getRobotPose(current_pose))
      {
        ROS_WARN("AckermannArcRecovery %s stopped: robot pose became unavailable",
                 name_.c_str());
        break;
      }
      const double pose_step = std::hypot(current_pose.pose.position.x - previous_pose.pose.position.x,
                                          current_pose.pose.position.y - previous_pose.pose.position.y);
      if (!std::isfinite(pose_step) || pose_step > max_pose_jump_)
      {
        ROS_WARN("AckermannArcRecovery %s stopped: pose jumped %.3f m",
                 name_.c_str(), pose_step);
        break;
      }
      measured_distance += pose_step;
      previous_pose = current_pose;

      double bounded_distance = std::max(measured_distance, commanded_distance);
      double remaining = max_distance_ - bounded_distance;
      if (remaining <= sim_granularity_)
      {
        ROS_INFO("AckermannArcRecovery %s completed %.3f m bounded arc",
                 name_.c_str(), bounded_distance);
        break;
      }

      const double preview_distance = std::min(remaining, safety_lookahead_);
      if (!evaluateTrajectory(current_pose, curvature, preview_distance).safe)
      {
        ROS_WARN("AckermannArcRecovery %s stopped: obstacle entered the %.2f m "
                 "safety lookahead",
                 name_.c_str(), preview_distance);
        break;
      }

      // Account for the previous command while the pose and costmap checks ran.
      // This is deliberately wall-time based: a delayed control thread must not
      // underestimate how long a nonzero command was held by the chassis.
      const ros::WallTime command_wall = ros::WallTime::now();
      accountHeldCommand(command_wall);
      if ((command_wall - start_wall).toSec() >= max_duration_)
      {
        break;
      }
      bounded_distance = std::max(measured_distance, commanded_distance);
      remaining = max_distance_ - bounded_distance;
      if (remaining <= sim_granularity_)
      {
        break;
      }

      const double acceleration_dt = (command_wall - speed_updated_at).toSec();
      speed_updated_at = command_wall;
      if (!std::isfinite(acceleration_dt) || acceleration_dt < 0.0)
      {
        ROS_WARN("AckermannArcRecovery %s stopped: invalid wall-clock interval",
                 name_.c_str());
        break;
      }
      speed_magnitude = std::min(
          linear_speed_, speed_magnitude + acceleration_limit_ * acceleration_dt);
      const double command_linear = static_cast<double>(direction_) * speed_magnitude;

      // Even if this thread stalls after publishing, the command cannot cover
      // the remaining distance before the M2 command watchdog expires.
      const double bounded_speed =
          std::min(std::abs(command_linear), remaining / command_hold_timeout_);
      geometry_msgs::Twist command;
      command.linear.x = std::copysign(bounded_speed, command_linear);
      command.angular.z = command.linear.x * curvature;
      if (!publishCommandIfActive(command))
      {
        break;
      }
      held_speed_magnitude = bounded_speed;
      rate.sleep();
    }
  }

private:
  void beginRun()
  {
    std::lock_guard<std::mutex> lock(run_state_mutex_);
    // requestAbort() uses the same mutex. Therefore an interrupt racing this
    // transition is either consumed here or observes run_active_ and sets the
    // live abort flag; it cannot fall between the two states and be lost.
    abort_requested_.store(pending_interrupt_.consumeIfFresh(
        ros::WallTime::now().toSec(), pre_run_interrupt_timeout_));
    run_active_ = true;
  }

  void endRun()
  {
    std::lock_guard<std::mutex> lock(run_state_mutex_);
    run_active_ = false;
  }

  void requestAbort()
  {
    {
      std::lock_guard<std::mutex> lock(run_state_mutex_);
      if (!run_active_)
      {
        pending_interrupt_.record(ros::WallTime::now().toSec());
        return;
      }
      abort_requested_.store(true);
    }
    publishStop();
  }

  bool publishCommandIfActive(const geometry_msgs::Twist& command)
  {
    // Serialize the final abort check with nonzero publication. If this lock
    // is acquired first, requestAbort() publishes zero afterwards; if the
    // abort wins first, no later nonzero command can pass this guard.
    std::lock_guard<std::mutex> lock(run_state_mutex_);
    if (!run_active_ || abort_requested_.load())
    {
      return false;
    }
    cmd_vel_pub_.publish(command);
    return true;
  }

  void cancelCallback(const actionlib_msgs::GoalID::ConstPtr&)
  {
    requestAbort();
  }

  void actionGoalCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr&)
  {
    requestAbort();
  }

  TrajectoryEvaluation evaluateTrajectory(const geometry_msgs::PoseStamped& start,
                                          double curvature,
                                          double distance) const
  {
    TrajectoryEvaluation result;
    result.maximum_cost = 0.0;
    if (distance < 0.0 || !std::isfinite(distance) || !std::isfinite(curvature))
    {
      return result;
    }

    costmap_2d::Costmap2D* costmap = local_costmap_->getCostmap();
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());

    double x = start.pose.position.x;
    double y = start.pose.position.y;
    double yaw = tf2::getYaw(start.pose.orientation);
    double simulated = 0.0;

    while (true)
    {
      const double footprint_cost = world_model_->footprintCost(
          x, y, yaw, footprint_, inscribed_radius_, circumscribed_radius_);
      if (!std::isfinite(footprint_cost) || footprint_cost < 0.0 ||
          footprint_cost > max_footprint_cost_)
      {
        return result;
      }

      std::vector<geometry_msgs::Point> oriented_footprint;
      costmap_2d::transformFootprint(x, y, yaw, footprint_, oriented_footprint);
      unsigned char interior_maximum_cost = costmap_2d::FREE_SPACE;
      if (!recovery_detail::footprintInteriorIsSafe(
              *costmap, oriented_footprint,
              static_cast<unsigned char>(std::floor(max_footprint_cost_)),
              &interior_maximum_cost))
      {
        return result;
      }
      result.maximum_cost = std::max(
          result.maximum_cost,
          std::max(footprint_cost, static_cast<double>(interior_maximum_cost)));

      if (simulated >= distance)
      {
        result.safe = true;
        return result;
      }

      const double step = std::min(sim_granularity_, distance - simulated);
      const double signed_step = static_cast<double>(direction_) * step;
      const recovery_detail::ArcPose next = recovery_detail::integrateArcStep(
          recovery_detail::ArcPose{x, y, yaw}, signed_step, curvature);
      x = next.x;
      y = next.y;
      yaw = next.yaw;
      simulated += step;
    }
  }

  double selectCurvature(const TrajectoryEvaluation& left,
                         const TrajectoryEvaluation& right,
                         double magnitude) const
  {
    if (preferred_turn_ == "left")
    {
      if (left.safe)
        return magnitude;
      return right.safe ? -magnitude : 0.0;
    }
    if (preferred_turn_ == "right")
    {
      if (right.safe)
        return -magnitude;
      return left.safe ? magnitude : 0.0;
    }
    if (left.safe && right.safe)
    {
      // Lower maximum inflation cost means the full footprint has more
      // clearance. On an exact tie, use the opposite steering side from the
      // previous recovery arc so reverse+forward cannot just retrace one path.
      if (left.maximum_cost < right.maximum_cost)
        return magnitude;
      if (right.maximum_cost < left.maximum_cost)
        return -magnitude;
      return last_selected_turn_sign.load() > 0 ? -magnitude : magnitude;
    }
    if (left.safe)
      return magnitude;
    if (right.safe)
      return -magnitude;
    return 0.0;
  }

  void publishStop()
  {
    if (!cmd_vel_pub_)
      return;
    geometry_msgs::Twist stop;
    cmd_vel_pub_.publish(stop);
  }

  bool initialized_{false};
  std::string name_;
  costmap_2d::Costmap2DROS* local_costmap_{nullptr};
  std::unique_ptr<base_local_planner::CostmapModel> world_model_;
  std::vector<geometry_msgs::Point> footprint_;
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber cancel_sub_;
  ros::Subscriber action_goal_sub_;
  std::mutex run_state_mutex_;
  bool run_active_{false};
  std::atomic<bool> abort_requested_{false};
  recovery_detail::PendingInterruptGate pending_interrupt_;

  int direction_{-1};
  double linear_speed_{0.30};
  double acceleration_limit_{0.60};
  double min_turning_radius_{1.30};
  double max_angular_speed_{0.24};
  double max_distance_{0.55};
  double max_duration_{4.0};
  double frequency_{20.0};
  double command_hold_timeout_{0.20};
  double pre_run_interrupt_timeout_{0.50};
  double sim_granularity_{0.05};
  double safety_lookahead_{0.30};
  double max_pose_jump_{0.50};
  double max_footprint_cost_{252.0};
  double inscribed_radius_{0.0};
  double circumscribed_radius_{0.0};
  std::string preferred_turn_{"auto"};
  std::string cmd_vel_topic_{"cmd_vel"};
  std::string cancel_topic_{"/move_base/cancel"};
  std::string action_goal_topic_{"/move_base/goal"};
};

}  // namespace robot_bringup

PLUGINLIB_EXPORT_CLASS(robot_bringup::AckermannArcRecovery, nav_core::RecoveryBehavior)
