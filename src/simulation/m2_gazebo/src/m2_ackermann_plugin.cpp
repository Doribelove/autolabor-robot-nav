#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <autolabor_canbus_driver/ChassisParameterServer.h>
#include <m2_gazebo/ackermann_kinematics.h>
#include <m2_gazebo/actuator_dynamics.h>

namespace gazebo {

class M2AckermannPlugin : public ModelPlugin {
 public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("m2_ackermann_plugin requires gazebo_ros_api_plugin");
      return;
    }
    model_ = model;
    world_ = model_->GetWorld();
    initial_pose_ = model_->WorldPose();

    readSdf(sdf);
    if (!validConfiguration()) {
      ROS_FATAL_STREAM("m2_ackermann_plugin rejected invalid V2 dynamics configuration");
      return;
    }
    front_left_steer_ = requireJoint("front_left_steer_joint");
    front_right_steer_ = requireJoint("front_right_steer_joint");
    front_left_wheel_ = requireJoint("front_left_wheel_joint");
    front_right_wheel_ = requireJoint("front_right_wheel_joint");
    rear_left_wheel_ = requireJoint("rear_left_wheel_joint");
    rear_right_wheel_ = requireJoint("rear_right_wheel_joint");
    if (!front_left_steer_ || !front_right_steer_ || !front_left_wheel_ ||
        !front_right_wheel_ || !rear_left_wheel_ || !rear_right_wheel_) {
      ROS_FATAL_STREAM("m2_ackermann_plugin: required M2 joints are missing");
      return;
    }

    ros::NodeHandle root;
    ros::NodeHandle driver("m2_driver");
    cmd_sub_ = root.subscribe(cmd_vel_topic_, 1, &M2AckermannPlugin::cmdVel, this);
    ackermann_sub_ = root.subscribe(ackerman_topic_, 1, &M2AckermannPlugin::ackermanVel, this);
    brake_sub_ = driver.subscribe("brake_set", 1, &M2AckermannPlugin::brake, this);
    emergency_sub_ = driver.subscribe("emergency_stop", 1, &M2AckermannPlugin::emergency, this);
    reset_sub_ = driver.subscribe("reset_odom", 1, &M2AckermannPlugin::resetOdom, this);
    bias_sub_ = driver.subscribe("steer_center_bias", 1, &M2AckermannPlugin::steerBias, this);

    odom_pub_ = root.advertise<nav_msgs::Odometry>(odom_topic_, 10);
    joint_pub_ = root.advertise<sensor_msgs::JointState>("/joint_states", 10);
    wheel_angle_pub_ = driver.advertise<std_msgs::Float64>("wheel_angle", 10);
    left_wheel_pub_ = driver.advertise<std_msgs::Float64>("left_wheel_vel", 10);
    right_wheel_pub_ = driver.advertise<std_msgs::Float64>("right_wheel_vel", 10);
    timeout_pub_ = driver.advertise<std_msgs::Bool>("control_timeout", 1, true);
    applied_speed_pub_ = driver.advertise<std_msgs::Float64>("applied_speed", 10);
    command_latency_pub_ = driver.advertise<std_msgs::Float64>("command_activation_latency", 10);
    parameter_service_ = driver.advertiseService(
        "chassis_parameter", &M2AckermannPlugin::chassisParameters, this);

    last_update_ = world_->SimTime();
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&M2AckermannPlugin::update, this));
    spinner_.reset(new ros::AsyncSpinner(1));
    spinner_->start();
    ROS_INFO_STREAM("M2 simulation-only Ackermann interface loaded; cmd_angle_instead_rotvel=false; "
                    << "v2_actuator_dynamics=" << (enable_actuator_dynamics_ ? "true" : "false"));
  }

 private:
  template <typename T>
  T sdfValue(const sdf::ElementPtr& sdf, const std::string& name,
             const T& fallback) {
    return sdf->HasElement(name) ? sdf->Get<T>(name) : fallback;
  }

  void readSdf(const sdf::ElementPtr& sdf) {
    wheelbase_ = sdfValue<double>(sdf, "wheelbase", 0.65);
    track_ = sdfValue<double>(sdf, "track", 0.60);
    wheel_radius_ = sdfValue<double>(sdf, "wheelRadius", 0.15);
    robot_width_ = sdfValue<double>(sdf, "robotWidth", 0.70);
    max_speed_ = sdfValue<double>(sdf, "maxSpeed", 2.778);
    max_steer_ = sdfValue<double>(sdf, "maxSteer", 0.4964);
    timeout_ = sdfValue<double>(sdf, "commandTimeout", 0.5);
    update_rate_ = sdfValue<double>(sdf, "updateRate", 50.0);
    enable_actuator_dynamics_ = sdfValue<bool>(sdf, "enableActuatorDynamics", false);
    dynamics_limits_.speed_time_constant_s =
        sdfValue<double>(sdf, "speedTimeConstant", 0.22);
    dynamics_limits_.steering_time_constant_s =
        sdfValue<double>(sdf, "steeringTimeConstant", 0.18);
    dynamics_limits_.max_acceleration_mps2 =
        sdfValue<double>(sdf, "maxAcceleration", 1.20);
    dynamics_limits_.max_deceleration_mps2 =
        sdfValue<double>(sdf, "maxDeceleration", 1.60);
    dynamics_limits_.max_brake_deceleration_mps2 =
        sdfValue<double>(sdf, "maxBrakeDeceleration", 2.40);
    dynamics_limits_.max_emergency_deceleration_mps2 =
        sdfValue<double>(sdf, "maxEmergencyDeceleration", 3.00);
    dynamics_limits_.max_steering_rate_radps =
        sdfValue<double>(sdf, "maxSteeringRate", 0.80);
    command_delay_s_ = sdfValue<double>(sdf, "commandDelay", 0.0);
    command_jitter_s_ = sdfValue<double>(sdf, "commandJitter", 0.0);
    dynamics_seed_ = sdfValue<unsigned int>(sdf, "dynamicsSeed", 42U);
    publish_tf_ = sdfValue<bool>(sdf, "publishTf", true);
    cmd_vel_topic_ = sdfValue<std::string>(sdf, "cmdVelTopic", "/cmd_vel");
    ackerman_topic_ = sdfValue<std::string>(sdf, "ackermanVelTopic", "/ackerman_vel");
    odom_topic_ = sdfValue<std::string>(sdf, "odomTopic", "/odom");
    odom_frame_ = sdfValue<std::string>(sdf, "odomFrame", "odom");
    base_frame_ = sdfValue<std::string>(sdf, "baseFrame", "base_link");
  }

  bool validConfiguration() const {
    const bool common = wheelbase_ > 0.0 && track_ > 0.0 && wheel_radius_ > 0.0 &&
                        max_speed_ > 0.0 && max_steer_ > 0.0 && timeout_ > 0.0 &&
                        update_rate_ > 0.0;
    if (!common) return false;
    if (!enable_actuator_dynamics_) return true;
    return m2_gazebo::validActuatorLimits(dynamics_limits_) &&
           command_delay_s_ >= 0.0 && command_jitter_s_ >= 0.0 &&
           command_jitter_s_ <= command_delay_s_;
  }

  physics::JointPtr requireJoint(const std::string& name) {
    auto joint = model_->GetJoint(name);
    if (!joint) ROS_ERROR_STREAM("m2_ackermann_plugin missing joint " << name);
    return joint;
  }

  void cmdVel(const geometry_msgs::Twist::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    const double speed = m2_gazebo::clamp(msg->linear.x, -max_speed_, max_speed_);
    enqueueCommand(speed, m2_gazebo::twistToSteering(
        speed, msg->angular.z, wheelbase_, max_steer_));
  }

  void ackermanVel(const geometry_msgs::Twist::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    enqueueCommand(
        m2_gazebo::clamp(msg->linear.x, -max_speed_, max_speed_),
        m2_gazebo::clamp(msg->angular.z, -max_steer_, max_steer_));
  }

  void enqueueCommand(double speed, double steering) {
    const double receipt = world_->SimTime().Double();
    command_received_ = true;
    last_command_time_ = receipt;
    if (!enable_actuator_dynamics_) {
      command_speed_ = speed;
      command_steer_ = steering;
      return;
    }
    const double jitter = m2_gazebo::deterministicSignedJitter(
        command_sequence_++, dynamics_seed_, command_jitter_s_);
    const double candidate_activation = receipt + command_delay_s_ + jitter;
    last_scheduled_activation_ = std::max(last_scheduled_activation_, candidate_activation);
    pending_commands_.push_back(
        PendingCommand{speed, steering, receipt, last_scheduled_activation_});
    if (pending_commands_.size() > 512U) {
      pending_commands_.pop_front();
      ROS_ERROR_THROTTLE(1.0, "m2_ackermann_plugin command delay queue overflow");
    }
  }

  void brake(const std_msgs::Bool::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    brake_active_ = msg->data;
  }

  void emergency(const std_msgs::Bool::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    emergency_active_ = msg->data;
  }

  void steerBias(const std_msgs::Float64::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    steer_bias_ = m2_gazebo::clamp(msg->data, -max_steer_, max_steer_);
  }

  void resetOdom(const std_msgs::Empty::ConstPtr&) {
    std::lock_guard<std::mutex> lock(mutex_);
    x_ = y_ = yaw_ = 0.0;
    command_speed_ = command_steer_ = 0.0;
    actuator_state_ = m2_gazebo::ActuatorState{};
    pending_commands_.clear();
    command_received_ = false;
    command_sequence_ = 0U;
    last_command_time_ = 0.0;
    last_scheduled_activation_ = 0.0;
    initial_pose_ = model_->WorldPose();
  }

  void activatePendingCommands(double now_seconds) {
    while (!pending_commands_.empty() &&
           pending_commands_.front().activation_time_s <= now_seconds) {
      const PendingCommand command = pending_commands_.front();
      pending_commands_.pop_front();
      command_speed_ = command.speed_mps;
      command_steer_ = command.steering_rad;
      std_msgs::Float64 latency;
      latency.data = std::max(0.0, now_seconds - command.receipt_time_s);
      command_latency_pub_.publish(latency);
    }
  }

  bool chassisParameters(
      autolabor_canbus_driver::ChassisParameterServer::Request&,
      autolabor_canbus_driver::ChassisParameterServer::Response& response) {
    response.success = true;
    response.parameters.max_speed = max_speed_;
    response.parameters.max_steer = max_steer_;
    response.parameters.robot_width = robot_width_;
    // The real driver uses robot_length in the bicycle equation; return the
    // candidate wheelbase here instead of the external body length.
    response.parameters.robot_length = wheelbase_;
    response.parameters.wheel_radius = wheel_radius_;
    response.message = "Simulation candidates only; calibrated=false";
    return true;
  }

  ros::Time rosTime(double seconds) const {
    ros::Time result;
    result.fromSec(seconds);
    return result;
  }

  void update() {
    const common::Time now = world_->SimTime();
    const double dt = (now - last_update_).Double();
    if (dt <= 0.0 || (update_rate_ > 0.0 && dt < 1.0 / update_rate_)) return;
    last_update_ = now;

    double velocity = 0.0;
    double steering = 0.0;
    bool timed_out = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (enable_actuator_dynamics_) activatePendingCommands(now.Double());
      timed_out = !command_received_ || now.Double() - last_command_time_ > timeout_;
      if (enable_actuator_dynamics_) {
        if (timed_out) pending_commands_.clear();
        const double target_speed = timed_out ? 0.0 : command_speed_;
        const double target_steering = timed_out ? 0.0 :
            m2_gazebo::clamp(command_steer_ + steer_bias_, -max_steer_, max_steer_);
        actuator_state_ = m2_gazebo::advanceActuators(
            actuator_state_, target_speed, target_steering,
            brake_active_ || timed_out, emergency_active_, dt, dynamics_limits_);
        actuator_state_.steering_rad = m2_gazebo::clamp(
            actuator_state_.steering_rad, -max_steer_, max_steer_);
        velocity = actuator_state_.speed_mps;
        steering = actuator_state_.steering_rad;
      } else if (!timed_out && !brake_active_ && !emergency_active_) {
        velocity = command_speed_;
        steering = m2_gazebo::clamp(
            command_steer_ + steer_bias_, -max_steer_, max_steer_);
      }
    }

    const auto geometry = m2_gazebo::steeringGeometry(
        steering, velocity, wheelbase_, track_, max_steer_);
    double left_velocity = 0.0;
    double right_velocity = 0.0;
    m2_gazebo::rearWheelLinearVelocities(
        velocity, geometry.yaw_rate, track_, &left_velocity, &right_velocity);

    x_ += velocity * std::cos(yaw_) * dt;
    y_ += velocity * std::sin(yaw_) * dt;
    yaw_ += geometry.yaw_rate * dt;
    const ignition::math::Pose3d relative(x_, y_, 0.0, 0.0, 0.0, yaw_);
    model_->SetWorldPose(initial_pose_ * relative);
    model_->SetLinearVel(ignition::math::Vector3d::Zero);
    model_->SetAngularVel(ignition::math::Vector3d::Zero);

    front_left_steer_->SetPosition(0, geometry.left, true);
    front_right_steer_->SetPosition(0, geometry.right, true);
    rear_left_wheel_->SetVelocity(0, left_velocity / wheel_radius_);
    rear_right_wheel_->SetVelocity(0, right_velocity / wheel_radius_);
    front_left_wheel_->SetVelocity(0, left_velocity / wheel_radius_);
    front_right_wheel_->SetVelocity(0, right_velocity / wheel_radius_);

    publish(now.Double(), velocity, geometry, left_velocity, right_velocity, timed_out);
  }

  void publish(double seconds, double velocity,
               const m2_gazebo::SteeringGeometry& steering,
               double left_velocity, double right_velocity, bool timed_out) {
    const ros::Time stamp = rosTime(seconds);
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw_);

    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.x = quaternion.x();
    odom.pose.pose.orientation.y = quaternion.y();
    odom.pose.pose.orientation.z = quaternion.z();
    odom.pose.pose.orientation.w = quaternion.w();
    odom.twist.twist.linear.x = velocity;
    odom.twist.twist.angular.z = steering.yaw_rate;
    odom_pub_.publish(odom);

    if (publish_tf_) {
      geometry_msgs::TransformStamped transform;
      transform.header = odom.header;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = x_;
      transform.transform.translation.y = y_;
      transform.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_.sendTransform(transform);
    }

    sensor_msgs::JointState joints;
    joints.header.stamp = stamp;
    joints.name = {"front_left_steer_joint", "front_right_steer_joint",
                   "front_left_wheel_joint", "front_right_wheel_joint",
                   "rear_left_wheel_joint", "rear_right_wheel_joint"};
    joints.position = {steering.left, steering.right,
                       front_left_wheel_->Position(0), front_right_wheel_->Position(0),
                       rear_left_wheel_->Position(0), rear_right_wheel_->Position(0)};
    joints.velocity = {0.0, 0.0, left_velocity / wheel_radius_,
                       right_velocity / wheel_radius_, left_velocity / wheel_radius_,
                       right_velocity / wheel_radius_};
    joint_pub_.publish(joints);

    std_msgs::Float64 value;
    value.data = steering.center;
    wheel_angle_pub_.publish(value);
    value.data = left_velocity;
    left_wheel_pub_.publish(value);
    value.data = right_velocity;
    right_wheel_pub_.publish(value);
    value.data = velocity;
    applied_speed_pub_.publish(value);
    std_msgs::Bool timeout_message;
    timeout_message.data = timed_out;
    timeout_pub_.publish(timeout_message);
  }

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::JointPtr front_left_steer_, front_right_steer_;
  physics::JointPtr front_left_wheel_, front_right_wheel_;
  physics::JointPtr rear_left_wheel_, rear_right_wheel_;
  event::ConnectionPtr update_connection_;
  ignition::math::Pose3d initial_pose_;
  common::Time last_update_;

  std::unique_ptr<ros::AsyncSpinner> spinner_;
  ros::Subscriber cmd_sub_, ackermann_sub_, brake_sub_, emergency_sub_, reset_sub_, bias_sub_;
  ros::Publisher odom_pub_, joint_pub_, wheel_angle_pub_, left_wheel_pub_, right_wheel_pub_, timeout_pub_;
  ros::Publisher applied_speed_pub_, command_latency_pub_;
  ros::ServiceServer parameter_service_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::mutex mutex_;
  struct PendingCommand {
    double speed_mps;
    double steering_rad;
    double receipt_time_s;
    double activation_time_s;
  };
  double command_speed_{0.0}, command_steer_{0.0}, steer_bias_{0.0};
  double last_command_time_{0.0};
  double last_scheduled_activation_{0.0};
  bool command_received_{false}, brake_active_{false}, emergency_active_{false};
  bool enable_actuator_dynamics_{false};
  std::uint64_t command_sequence_{0U};
  unsigned int dynamics_seed_{42U};
  std::deque<PendingCommand> pending_commands_;
  m2_gazebo::ActuatorLimits dynamics_limits_;
  m2_gazebo::ActuatorState actuator_state_;
  double x_{0.0}, y_{0.0}, yaw_{0.0};
  double wheelbase_{0.65}, track_{0.60}, wheel_radius_{0.15}, robot_width_{0.70};
  double max_speed_{2.778}, max_steer_{0.4964}, timeout_{0.5}, update_rate_{50.0};
  double command_delay_s_{0.0}, command_jitter_s_{0.0};
  bool publish_tf_{true};
  std::string cmd_vel_topic_{"/cmd_vel"}, ackerman_topic_{"/ackerman_vel"};
  std::string odom_topic_{"/odom"}, odom_frame_{"odom"}, base_frame_{"base_link"};
};

GZ_REGISTER_MODEL_PLUGIN(M2AckermannPlugin)

}  // namespace gazebo
