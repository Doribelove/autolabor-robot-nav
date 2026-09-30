#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
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
#include <std_msgs/String.h>
#include <tf2_ros/transform_broadcaster.h>
#include <autolabor_canbus_driver/ChassisParameterServer.h>
#include <m2_gazebo/actuator_dynamics.h>
#include <m2_gazebo/astra_physical_control.h>

namespace gazebo {
// Simulation-only effort backend. After spawn, only joint efforts influence
// physical state: no pose, joint-position, or body/joint velocity writes.
class AstraPhysicalAckermannPlugin : public ModelPlugin {
 public:
  ~AstraPhysicalAckermannPlugin() override {
    begin_.reset();end_.reset();if (spinner_) spinner_->stop();
  }
  void Load(physics::ModelPtr model,sdf::ElementPtr sdf) override {
    if (!ros::isInitialized()) {gzerr<<"Astra physical backend requires gazebo_ros_api_plugin\n";return;}
    model_=model;world_=model_->GetWorld();
    if (world_->Physics()->GetType()!="ode") {gzerr<<"Astra physical backend requires frozen ODE environment\n";return;}
    wheelbase_=read(sdf,"wheelbase",.65);track_=read(sdf,"track",.60);radius_=read(sdf,"wheelRadius",.15);
    max_speed_=read(sdf,"maxSpeed",2.778);max_steer_=read(sdf,"maxSteer",.4964);
    timeout_=read(sdf,"commandTimeout",.5);publish_hz_=read(sdf,"publishRate",50.);
    delay_=read(sdf,"commandDelay",0.);jitter_=read(sdf,"commandJitter",0.);
    seed_=sdf->HasElement("dynamicsSeed")?sdf->Get<unsigned int>("dynamicsSeed"):42U;
    limits_.speed_time_constant_s=read(sdf,"speedTimeConstant",.22);
    limits_.steering_time_constant_s=read(sdf,"steeringTimeConstant",.18);
    limits_.max_acceleration_mps2=read(sdf,"maxAcceleration",.60);
    limits_.max_deceleration_mps2=read(sdf,"maxDeceleration",.90);
    limits_.max_brake_deceleration_mps2=read(sdf,"maxBrakeDeceleration",.90);
    limits_.max_emergency_deceleration_mps2=read(sdf,"maxEmergencyDeceleration",.90);
    limits_.max_steering_rate_radps=read(sdf,"maxSteeringRate",.75);
    drive_torque_=read(sdf,"wheelDriveTorqueLimit",8.);
    brake_torque_=read(sdf,"wheelBrakeTorqueLimit",12.);
    steer_torque_=read(sdf,"steeringTorqueLimit",20.);
    steer_kp_=read(sdf,"steeringKp",60.);steer_kd_=read(sdf,"steeringKd",4.);
    wheel_kp_=read(sdf,"wheelKp",4.);wheel_ki_=read(sdf,"wheelKi",2.);
    integral_limit_=read(sdf,"wheelIntegralTorqueLimit",1.5);
    wheel_damping_=read(sdf,"wheelJointDamping",.05);
    wheel_coulomb_=read(sdf,"wheelJointFriction",.01);
    expected_mass_=read(sdf,"expectedMass",79.6);
    publish_tf_=sdf->HasElement("publishTf")?sdf->Get<bool>("publishTf"):true;
    if (!valid()) {gzerr<<"Astra physical backend invalid finite controller configuration\n";return;}
    base_=model_->GetLink("base_link");
    const std::array<std::string,6> names={{"front_left_steer_joint","front_right_steer_joint",
      "front_left_wheel_joint","front_right_wheel_joint","rear_left_wheel_joint","rear_right_wheel_joint"}};
    for (size_t i=0;i<6;++i) {joints_[i]=model_->GetJoint(names[i]);if (!joints_[i]) {gzerr<<"Astra physical joint missing "<<names[i]<<"\n";return;}}
    if (!base_) {gzerr<<"Astra physical base_link missing\n";return;}
    for (auto link:model_->GetLinks()) if (link->GetInertial()) mass_+=link->GetInertial()->Mass();
    if (!std::isfinite(mass_) || std::abs(mass_-expected_mass_)>.001) {gzerr<<"Astra physical mass mismatch actual="<<mass_<<" expected="<<expected_mass_<<"\n";return;}
    // Quarter of chassis translation plus the original 3kg cylindrical wheel
    // spin inertia; controller feedforward only, actual physics inertia is URDF.
    wheel_effective_inertia_=mass_*radius_*radius_/4.+3.*radius_*radius_/2.;
    for (auto& pi:wheel_pi_) {pi.kp=wheel_kp_;pi.ki=wheel_ki_;pi.integral_limit=integral_limit_;}
    odom_origin_=base_->WorldPose();last_control_=last_publish_=world_->SimTime().Double();
    ros::NodeHandle root,driver("m2_driver");
    cmd_sub_=root.subscribe("/cmd_vel",1,&AstraPhysicalAckermannPlugin::cmd,this);
    ack_sub_=root.subscribe("/ackerman_vel",1,&AstraPhysicalAckermannPlugin::ack,this);
    brake_sub_=driver.subscribe("brake_set",1,&AstraPhysicalAckermannPlugin::brake,this);
    emergency_sub_=driver.subscribe("emergency_stop",1,&AstraPhysicalAckermannPlugin::emergency,this);
    reset_sub_=driver.subscribe("reset_odom",1,&AstraPhysicalAckermannPlugin::reset,this);
    bias_sub_=driver.subscribe("steer_center_bias",1,&AstraPhysicalAckermannPlugin::bias,this);
    odom_pub_=root.advertise<nav_msgs::Odometry>("/odom",10);
    joint_pub_=root.advertise<sensor_msgs::JointState>("/joint_states",10);
    target_pub_=driver.advertise<sensor_msgs::JointState>("actuator_target",10);
    effort_pub_=driver.advertise<sensor_msgs::JointState>("actuator_commanded_effort",10);
    angle_pub_=driver.advertise<std_msgs::Float64>("wheel_angle",10);
    speed_pub_=driver.advertise<std_msgs::Float64>("applied_speed",10);
    left_pub_=driver.advertise<std_msgs::Float64>("left_wheel_vel",10);
    right_pub_=driver.advertise<std_msgs::Float64>("right_wheel_vel",10);
    timeout_pub_=driver.advertise<std_msgs::Bool>("control_timeout",1,true);
    latency_pub_=driver.advertise<std_msgs::Float64>("command_activation_latency",10);
    fault_pub_=driver.advertise<std_msgs::Bool>("physical_backend_fault",1,true);
    metadata_pub_=driver.advertise<std_msgs::String>("physical_metadata",1,true);
    parameter_service_=driver.advertiseService("chassis_parameter",&AstraPhysicalAckermannPlugin::parameters,this);
    publishMetadata();
    begin_=event::Events::ConnectWorldUpdateBegin(std::bind(&AstraPhysicalAckermannPlugin::control,this));
    end_=event::Events::ConnectWorldUpdateEnd(std::bind(&AstraPhysicalAckermannPlugin::publish,this));
    spinner_.reset(new ros::AsyncSpinner(1));spinner_->start();
    gzmsg<<"AstraPhysicalAckermannPlugin loaded mass="<<mass_<<" effort_only=true calibrated=false\n";
  }
 private:
  double read(sdf::ElementPtr sdf,const std::string& key,double fallback) {
    return sdf->HasElement(key)?sdf->Get<double>(key):fallback;
  }
  bool valid() const {
    for (double x:{wheelbase_,track_,radius_,max_speed_,max_steer_,timeout_,publish_hz_,
         drive_torque_,brake_torque_,steer_torque_,steer_kp_,steer_kd_,wheel_kp_,integral_limit_,expected_mass_,
         limits_.speed_time_constant_s,limits_.steering_time_constant_s,
         limits_.max_acceleration_mps2,limits_.max_deceleration_mps2,
         limits_.max_brake_deceleration_mps2,limits_.max_emergency_deceleration_mps2,
         limits_.max_steering_rate_radps})
      if (!std::isfinite(x)||x<=0.) return false;
    return m2_gazebo::validActuatorLimits(limits_) && std::isfinite(delay_) && delay_>=0. &&
      std::isfinite(jitter_) && jitter_>=0. && jitter_<=delay_ && std::isfinite(wheel_ki_) && wheel_ki_>=0. &&
      std::isfinite(wheel_damping_) && wheel_damping_>=0. && std::isfinite(wheel_coulomb_) && wheel_coulomb_>=0.;
  }
  void cmd(const geometry_msgs::Twist::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!std::isfinite(msg->linear.x)||!std::isfinite(msg->angular.z)) {fault_=true;pending_.clear();return;}
    const double vx=m2_gazebo::clamp(msg->linear.x,-max_speed_,max_speed_);
    enqueue(vx,m2_gazebo::twistToSteering(vx,msg->angular.z,wheelbase_,max_steer_));
  }
  void ack(const geometry_msgs::Twist::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!std::isfinite(msg->linear.x)||!std::isfinite(msg->angular.z)) {fault_=true;pending_.clear();return;}
    enqueue(m2_gazebo::clamp(msg->linear.x,-max_speed_,max_speed_),m2_gazebo::clamp(msg->angular.z,-max_steer_,max_steer_));
  }
  void enqueue(double speed,double steering) {
    if (fault_) return;
    const double now=world_->SimTime().Double();
    if (!std::isfinite(now)|| (received_&&now<last_receipt_)) {fault_=true;pending_.clear();return;}
    received_=true;last_receipt_=now;
    const double activate=now+delay_+m2_gazebo::deterministicSignedJitter(sequence_++,seed_,jitter_);
    last_activation_=std::max(last_activation_,activate);
    pending_.push_back({speed,steering,now,last_activation_});
    if (pending_.size()>512U) {fault_=true;pending_.clear();ROS_ERROR("Astra physical command queue overflow; episode invalid");}
  }
  void brake(const std_msgs::Bool::ConstPtr& m) {std::lock_guard<std::mutex> l(mutex_);brake_=m->data;}
  void emergency(const std_msgs::Bool::ConstPtr& m) {std::lock_guard<std::mutex> l(mutex_);emergency_=m->data;}
  void reset(const std_msgs::Empty::ConstPtr&) {std::lock_guard<std::mutex> l(mutex_);reset_requested_=true;}
  void bias(const std_msgs::Float64::ConstPtr& m) {std::lock_guard<std::mutex> l(mutex_);if (!std::isfinite(m->data)) fault_=true;else bias_=m2_gazebo::clamp(m->data,-max_steer_,max_steer_);}
  void control() {
    const double now=world_->SimTime().Double();const double dt=now-last_control_;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!std::isfinite(now)||dt<0.) {
      fault_=true;pending_.clear();
      if (std::isfinite(now)) {last_control_=now;last_publish_=now;}
      if (!fault_logged_) {gzerr<<"AstraPhysicalAckermannPlugin physical_backend_fault reason=clock_regression_or_nonfinite requires_new_episode=true\n";fault_logged_=true;}
    }
    if (dt<=0.||!std::isfinite(dt)) return;
    last_control_=now;
    // Physics-step loss is an integration failure, not a reason to integrate
    // a large hidden interval into the target or to reset the physical model.
    if (dt>std::max(.01,world_->Physics()->GetMaxStepSize()*1.5)) {fault_=true;pending_.clear();}
    while (!pending_.empty()&&pending_.front().activation<=now) {
      const auto p=pending_.front();pending_.pop_front();command_speed_=p.speed;command_steer_=p.steering;
      std_msgs::Float64 latency;latency.data=now-p.receipt;latency_pub_.publish(latency);
    }
    timed_out_=!received_||now-last_receipt_>timeout_;
    if (timed_out_||fault_) pending_.clear();
    if (reset_requested_) {
      // Reset only the reported odometry frame. Physical body/joints remain
      // untouched; request while moving invalidates this episode.
      if (base_->WorldLinearVel().Length()>.05) fault_=true;
      else {odom_origin_=base_->WorldPose();received_=false;pending_.clear();command_speed_=command_steer_=0.;}
      reset_requested_=false;
    }
    const auto previous=reference_;
    const bool stopping=timed_out_||brake_||emergency_||fault_;
    const double target_speed=stopping?0.:command_speed_;
    const double target_steer=(timed_out_||fault_)?0.:m2_gazebo::clamp(command_steer_+bias_,-max_steer_,max_steer_);
    reference_=m2_gazebo::advanceActuators(reference_,target_speed,target_steer,brake_||timed_out_||fault_,emergency_,dt,limits_);
    geometry_=m2_gazebo::steeringGeometry(reference_.steering_rad,reference_.speed_mps,wheelbase_,track_,max_steer_);
    const auto wheel_targets=m2_gazebo::physicalWheelSpeeds(reference_.speed_mps,geometry_,wheelbase_,track_,radius_);
    target_rates_={{(reference_.speed_mps-previous.speed_mps)/dt,(reference_.steering_rad-previous.steering_rad)/dt,
                   (geometry_.left-previous_left_)/dt,(geometry_.right-previous_right_)/dt}};
    previous_left_=geometry_.left;previous_right_=geometry_.right;
    for (size_t i=0;i<6;++i) if (!std::isfinite(joints_[i]->Position(0))||!std::isfinite(joints_[i]->GetVelocity(0))) fault_=true;
    for (size_t i=0;i<2;++i) {
      const double target=i==0?geometry_.left:geometry_.right;
      efforts_[i]=m2_gazebo::boundedSteeringTorque(target,target_rates_[i+2],joints_[i]->Position(0),joints_[i]->GetVelocity(0),steer_kp_,steer_kd_,steer_torque_);
      joints_[i]->SetForce(0,efforts_[i]);
    }
    for (size_t i=0;i<4;++i) {
      const double rate=(wheel_targets[i]-previous_wheel_targets_[i])/dt;
      const double sign=(wheel_targets[i]>1e-6)-(wheel_targets[i]<-1e-6);
      const double feedforward=wheel_effective_inertia_*rate+wheel_damping_*wheel_targets[i]+wheel_coulomb_*sign;
      efforts_[i+2]=wheel_pi_[i].update(wheel_targets[i],joints_[i+2]->GetVelocity(0),feedforward,dt,stopping?brake_torque_:drive_torque_);
      // On a fault, finite velocity feedback applies braking effort without
      // commanding any body velocity or introducing state discontinuities.
      if (fault_) {
        const double actual_rate=joints_[i+2]->GetVelocity(0);
        efforts_[i+2]=std::isfinite(actual_rate)?m2_gazebo::clamp(-wheel_kp_*actual_rate,-brake_torque_,brake_torque_):0.;
      }
      joints_[i+2]->SetForce(0,efforts_[i+2]);
    }
    previous_wheel_targets_=wheel_targets;
    if (fault_&&!fault_logged_) {gzerr<<"AstraPhysicalAckermannPlugin physical_backend_fault requires_new_episode=true sim_time="<<now<<"\n";fault_logged_=true;}
  }
  ros::Time stamp(double now) {ros::Time t;t.fromSec(now);return t;}
  void publish() {
    const double now=world_->SimTime().Double();if (now-last_publish_+1e-12<1./publish_hz_) return;last_publish_=now;
    std::lock_guard<std::mutex> lock(mutex_);
    const auto pose=base_->WorldPose();const auto relative=odom_origin_.Inverse()*pose;
    const auto body_v=pose.Rot().RotateVectorReverse(base_->WorldLinearVel());
    const auto body_w=pose.Rot().RotateVectorReverse(base_->WorldAngularVel());
    nav_msgs::Odometry odom;odom.header.stamp=stamp(now);odom.header.frame_id="odom";odom.child_frame_id="base_link";
    odom.pose.pose.position.x=relative.Pos().X();odom.pose.pose.position.y=relative.Pos().Y();odom.pose.pose.position.z=relative.Pos().Z();
    odom.pose.pose.orientation.x=relative.Rot().X();odom.pose.pose.orientation.y=relative.Rot().Y();odom.pose.pose.orientation.z=relative.Rot().Z();odom.pose.pose.orientation.w=relative.Rot().W();
    odom.twist.twist.linear.x=body_v.X();odom.twist.twist.linear.y=body_v.Y();odom.twist.twist.linear.z=body_v.Z();
    odom.twist.twist.angular.x=body_w.X();odom.twist.twist.angular.y=body_w.Y();odom.twist.twist.angular.z=body_w.Z();odom_pub_.publish(odom);
    if (publish_tf_) {geometry_msgs::TransformStamped tf;tf.header=odom.header;tf.child_frame_id="base_link";tf.transform.translation.x=relative.Pos().X();tf.transform.translation.y=relative.Pos().Y();tf.transform.translation.z=relative.Pos().Z();tf.transform.rotation=odom.pose.pose.orientation;tf_pub_.sendTransform(tf);}
    sensor_msgs::JointState joint;joint.header.stamp=stamp(now);
    for (auto j:joints_) {joint.name.push_back(j->GetName());joint.position.push_back(j->Position(0));joint.velocity.push_back(j->GetVelocity(0));}
    // effort intentionally absent: commanded efforts are not measured reaction.
    joint_pub_.publish(joint);
    sensor_msgs::JointState effort;effort.header=joint.header;effort.header.frame_id="commanded_torque_not_reaction";
    effort.name=joint.name;effort.effort.assign(efforts_.begin(),efforts_.end());effort_pub_.publish(effort);
    sensor_msgs::JointState target;target.header.stamp=stamp(now);target.header.frame_id="actuator_reference_units_named";
    target.name={"speed","center_steering","front_left_steering","front_right_steering"};
    target.position={reference_.speed_mps,reference_.steering_rad,geometry_.left,geometry_.right};
    target.velocity.assign(target_rates_.begin(),target_rates_.end());target_pub_.publish(target);
    std_msgs::Float64 value;value.data=m2_gazebo::actualCenterSteering(joint.position[0],joint.position[1],wheelbase_,track_);angle_pub_.publish(value);
    value.data=body_v.X();speed_pub_.publish(value);value.data=joint.velocity[4]*radius_;left_pub_.publish(value);value.data=joint.velocity[5]*radius_;right_pub_.publish(value);
    std_msgs::Bool flag;flag.data=timed_out_;timeout_pub_.publish(flag);flag.data=fault_;fault_pub_.publish(flag);
  }
  void publishMetadata() {
    std::ostringstream o;o<<std::setprecision(17)<<"{\"backend\":\"astra_physical_ackermann_v1\",\"effort_only\":true,\"calibrated\":false,\"actual_mass_kg\":"<<mass_
      <<",\"wheelbase_m\":"<<wheelbase_<<",\"track_m\":"<<track_<<",\"wheel_radius_m\":"<<radius_
      <<",\"wheel_drive_torque_limit_nm\":"<<drive_torque_<<",\"wheel_brake_torque_limit_nm\":"<<brake_torque_<<",\"steering_torque_limit_nm\":"<<steer_torque_
      <<",\"wheel_kp\":"<<wheel_kp_<<",\"wheel_ki\":"<<wheel_ki_<<",\"steering_kp\":"<<steer_kp_<<",\"steering_kd\":"<<steer_kd_
      <<",\"control_rate\":\"every_physics_step\",\"publish_hz\":"<<publish_hz_<<",\"source_stamp\":\"WorldUpdateEnd\",\"odom_twist\":\"actual_base_link_body_frame_including_vy\",\"wheel_angle\":\"actual_two_joint_inverse_Ackermann_curvature_mean\",\"actuator_target_position_units\":[\"m_s\",\"rad\",\"rad\",\"rad\"],\"drive_topology\":\"four_independent_effort_driven_wheels_simulation_assumption\",\"joint_effort_fields\":\"omitted_not_measured_reaction\"}";
    std_msgs::String m;m.data=o.str();metadata_pub_.publish(m);
  }
  bool parameters(autolabor_canbus_driver::ChassisParameterServer::Request&,autolabor_canbus_driver::ChassisParameterServer::Response& r) {
    r.success=true;r.parameters.max_speed=max_speed_;r.parameters.max_steer=max_steer_;r.parameters.robot_width=.70;r.parameters.robot_length=wheelbase_;r.parameters.wheel_radius=radius_;r.message="M12 physical simulation candidate; calibrated=false; base_link at axle midpoint";return true;
  }
  struct Pending {double speed,steering,receipt,activation;};
  physics::ModelPtr model_;physics::WorldPtr world_;physics::LinkPtr base_;std::array<physics::JointPtr,6> joints_;
  event::ConnectionPtr begin_,end_;std::unique_ptr<ros::AsyncSpinner> spinner_;std::mutex mutex_;
  ros::Subscriber cmd_sub_,ack_sub_,brake_sub_,emergency_sub_,reset_sub_,bias_sub_;
  ros::Publisher odom_pub_,joint_pub_,target_pub_,effort_pub_,angle_pub_,speed_pub_,left_pub_,right_pub_,timeout_pub_,latency_pub_,fault_pub_,metadata_pub_;
  ros::ServiceServer parameter_service_;tf2_ros::TransformBroadcaster tf_pub_;
  ignition::math::Pose3d odom_origin_;m2_gazebo::ActuatorLimits limits_;m2_gazebo::ActuatorState reference_;m2_gazebo::SteeringGeometry geometry_;
  std::array<m2_gazebo::BoundedPI,4> wheel_pi_;std::array<double,4> previous_wheel_targets_{{0.,0.,0.,0.}},target_rates_{{0.,0.,0.,0.}};
  std::array<double,6> efforts_{{0.,0.,0.,0.,0.,0.}};std::deque<Pending> pending_;
  double wheelbase_{.65},track_{.60},radius_{.15},max_speed_{2.778},max_steer_{.4964},timeout_{.5},publish_hz_{50.};
  double drive_torque_{8.},brake_torque_{12.},steer_torque_{20.},steer_kp_{60.},steer_kd_{4.},wheel_kp_{4.},wheel_ki_{2.},integral_limit_{1.5},wheel_damping_{.05},wheel_coulomb_{.01};
  double expected_mass_{79.6},mass_{0.},wheel_effective_inertia_{0.},delay_{0.},jitter_{0.},last_control_{0.},last_publish_{0.},last_receipt_{0.},last_activation_{0.};
  double command_speed_{0.},command_steer_{0.},bias_{0.},previous_left_{0.},previous_right_{0.};
  unsigned int seed_{42U};std::uint64_t sequence_{0U};bool publish_tf_{true},received_{false},brake_{false},emergency_{false},timed_out_{true},fault_{false},fault_logged_{false},reset_requested_{false};
};
GZ_REGISTER_MODEL_PLUGIN(AstraPhysicalAckermannPlugin)
}  // namespace gazebo
