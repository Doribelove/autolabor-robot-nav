#include "autolabor_canbus_driver/m2_driver.h"
#include "autolabor_canbus_driver/Autocan.h"
#include "autolabor_canbus_driver/m2_twist_steering.h"
#include "utilities/big_endian_transform.h"

#include <algorithm>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>


namespace autolabor_driver{

    M2Driver::M2Driver() {
        autolabor_canbus_driver::CanBusMessage reqMsg;
        // 查询底盘参数
        reqMsg.node_type = Autocan::Vcu::Type;
        reqMsg.node_seq = Autocan::Vcu::NodeId;
        reqMsg.msg_type = Autocan::Vcu::MaxSpeed;
        param_req_queue_.push(reqMsg);
        reqMsg.msg_type = Autocan::Vcu::MaxSteer;
        param_req_queue_.push(reqMsg);
        reqMsg.msg_type = Autocan::Vcu::RobotWidth;
        reqMsg.msg_type = Autocan::Vcu::RobotWidth;
        param_req_queue_.push(reqMsg);
        reqMsg.msg_type = Autocan::Vcu::RobotLength;
        param_req_queue_.push(reqMsg);
        reqMsg.msg_type = Autocan::Vcu::WheelRadius;
        param_req_queue_.push(reqMsg);

        // 查询车辆状态
        reqMsg.msg_type = Autocan::Vcu::BatteryPercent;
        telemetry_info_req_queue_.push(reqMsg);
        reqMsg.msg_type = Autocan::Vcu::RemainSec;
        telemetry_info_req_queue_.push(reqMsg);
        reqMsg.msg_type = Autocan::Vcu::RemainCapacity;
        telemetry_info_req_queue_.push(reqMsg);
        reqMsg.msg_type = Autocan::Vcu::BatteryVoltage;
        telemetry_info_req_queue_.push(reqMsg);
        reqMsg.msg_type = Autocan::Vcu::BatteryCurrent;
        telemetry_info_req_queue_.push(reqMsg);
        reqMsg.msg_type = Autocan::Vcu::HardEmergency;
        safety_info_req_queue_.push(reqMsg);
        reqMsg.msg_type = Autocan::Vcu::SoftEmergency;
        safety_info_req_queue_.push(reqMsg);
        reqMsg.msg_type = Autocan::Vcu::GamepadEmergency;
        safety_info_req_queue_.push(reqMsg);
    }

    void M2Driver::ask_robot_param(const ros::TimerEvent &){
        if(!ChassisParameterHelper::areSet(chassis_parameter_))
        {
            autolabor_canbus_driver::CanBusService srv;
            autolabor_canbus_driver::CanBusMessage nextReq = param_req_queue_.front();
            srv.request.requests.push_back(nextReq);
            canbus_client_.call(srv);
            // 如果需要，将请求放回队尾
            param_req_queue_.pop();
            param_req_queue_.push(nextReq);
        }
        else
        {
            // 重置里程计
            send_to_canbus(Autocan::Vcu::Type, Autocan::Vcu::NodeId, Autocan::Vcu::ResetOdom);
            // 关闭查询定时器
            ask_parameters_timer_.stop();
        }
    }

    void M2Driver::ask_robot_info(const ros::TimerEvent &){
        if (safety_info_req_queue_.empty() || telemetry_info_req_queue_.empty()) {
            return;
        }
        autolabor_canbus_driver::CanBusService srv;
        // The VCU reply path was measured at roughly four queries per second.
        // Use three safety slots (hard/soft/gamepad emergency) and one rotating
        // telemetry slot per cycle.  At the default 1 Hz safety snapshot rate,
        // each emergency field is refreshed every second while battery fields
        // rotate every five seconds.  Every tick still contains exactly one
        // request, so no serial burst can overrun the VCU.
        const std::size_t safety_slots = safety_info_req_queue_.size();
        const bool safety_slot = info_poll_phase_ < safety_slots;
        std::queue<autolabor_canbus_driver::CanBusMessage>& selected_queue =
            safety_slot ? safety_info_req_queue_ : telemetry_info_req_queue_;
        const autolabor_canbus_driver::CanBusMessage next_req =
            selected_queue.front();
        srv.request.requests.push_back(next_req);
        selected_queue.pop();
        selected_queue.push(next_req);
        info_poll_phase_ = (info_poll_phase_ + 1) % (safety_slots + 1);
        if (!canbus_client_.call(srv)) {
            ROS_WARN_THROTTLE(
                1.0,
                "M2 chassis-status query failed; coverage freshness gates "
                "will remain fail-closed");
        }
    }

    void M2Driver::handle_canbus_msg(const autolabor_canbus_driver::CanBusMessage::ConstPtr &msg) {
        if(msg->node_type == Autocan::Vcu::Type)
        {
            switch (msg->msg_type) {
                case Autocan::Vcu::MaxSpeed:
                    chassis_parameter_.max_speed = autolabor::build_from_little_endian<float>(msg->payload.data());
                    break;
                case Autocan::Vcu::MaxSteer:
                    chassis_parameter_.max_steer = autolabor::build_from_little_endian<float>(msg->payload.data());
                    break;
                case Autocan::Vcu::RobotWidth:
                    chassis_parameter_.robot_width = autolabor::build_from_little_endian<float>(msg->payload.data());
                    break;
                case Autocan::Vcu::RobotLength:
                    chassis_parameter_.robot_length = autolabor::build_from_little_endian<float>(msg->payload.data());
                    break;
                case Autocan::Vcu::WheelRadius:
                    chassis_parameter_.wheel_radius = autolabor::build_from_little_endian<float>(msg->payload.data());
                    break;
                case Autocan::Vcu::BatteryPercent:
                    chassis_status_info_.battery_percent = msg->payload[0];
                    chassis_info_pub_.publish(chassis_status_info_);
                    break;
                case Autocan::Vcu::RemainSec:
                    chassis_status_info_.remain_sec = autolabor::build_from_little_endian<uint32_t>(msg->payload.data());
                    chassis_info_pub_.publish(chassis_status_info_);
                    break;
                case Autocan::Vcu::RemainCapacity:
                    chassis_status_info_.remain_capacity = autolabor::build_from_little_endian<uint32_t>(msg->payload.data());
                    chassis_info_pub_.publish(chassis_status_info_);
                    break;
                case Autocan::Vcu::BatteryVoltage:
                    chassis_status_info_.battery_voltage = autolabor::build_from_little_endian<uint16_t>(msg->payload.data());
                    chassis_info_pub_.publish(chassis_status_info_);
                    break;
                case Autocan::Vcu::BatteryCurrent:
                    chassis_status_info_.battery_current = autolabor::build_from_little_endian<uint32_t>(msg->payload.data());
                    chassis_info_pub_.publish(chassis_status_info_);
                    break;
                case Autocan::Vcu::HardEmergency:
                    chassis_status_info_.hard_emergency = msg->payload[0] != 0;
                    chassis_info_pub_.publish(chassis_status_info_);
                    break;
                case Autocan::Vcu::SoftEmergency:
                    chassis_status_info_.soft_emergency = msg->payload[0] != 0;
                    chassis_info_pub_.publish(chassis_status_info_);
                    break;
                case Autocan::Vcu::GamepadEmergency:
                    chassis_status_info_.gamepad_emergency = msg->payload[0] != 0;
                    chassis_info_pub_.publish(chassis_status_info_);
                    break;
                case Autocan::Common::State:
                    chassis_status_info_.robot_emergency = msg->payload[0] != Autocan::NodeState::Running;
                    chassis_info_pub_.publish(chassis_status_info_);
                    break;
                case Autocan::Vcu::CurrentVelocity:
                    cur_vel_ = autolabor::build_from_little_endian<float>(msg->payload.data());
                    cur_vel_time_ = ros::Time::now();
                    break;
                case Autocan::Vcu::OdomXy:
                    cur_odom_x_ = autolabor::build_from_little_endian<float>(msg->payload.data());
                    cur_odom_y_ = autolabor::build_from_little_endian<float>(msg->payload.data() + 4);
                    break;
                case Autocan::Vcu::OdomTheta:
                    cur_odom_yaw_ = autolabor::build_from_little_endian<float>(msg->payload.data());
                    break;
                case Autocan::Vcu::ControllerMonitor:
                    if (msg->payload.size() < 3) {
                        ROS_ERROR("Malformed VCU ControllerMonitor (0x23): "
                                  "expected at least 3 payload bytes, got %zu",
                                  msg->payload.size());
                        break;
                    }
                    // TCU
                    chassis_control_info_.tcu_state = (msg->payload[0] & Autocan::Vcu::VCU_MONITOR_STATUS_BIT) ? 1 : 0;
                    chassis_control_info_.tcu_timeout = (msg->payload[0] & Autocan::Vcu::VCU_MONITOR_DATA_TIMEOUT_BIT) ? 1 : 0;
                    chassis_control_info_.tcu_stuck = (msg->payload[0] & Autocan::Vcu::VCU_MONITOR_CURRENT_OVERLIMIT_BIT) ? 1 : 0;
                    // 左ECU
                    chassis_control_info_.lecu_state = (msg->payload[1] & Autocan::Vcu::VCU_MONITOR_STATUS_BIT) ? 1 : 0;
                    chassis_control_info_.lecu_timeout = (msg->payload[1] & Autocan::Vcu::VCU_MONITOR_DATA_TIMEOUT_BIT) ? 1 : 0;
                    chassis_control_info_.lecu_stuck = (msg->payload[1] & Autocan::Vcu::VCU_MONITOR_CURRENT_OVERLIMIT_BIT) ? 1 : 0;
                    chassis_control_info_.lecu_brake = (msg->payload[1] & Autocan::Vcu::VCU_MONITOR_BRAKE_BIT) ? 1 : 0;
                    // 右ECU
                    chassis_control_info_.recu_state = (msg->payload[2] & Autocan::Vcu::VCU_MONITOR_STATUS_BIT) ? 1 : 0;
                    chassis_control_info_.recu_timeout = (msg->payload[2] & Autocan::Vcu::VCU_MONITOR_DATA_TIMEOUT_BIT) ? 1 : 0;
                    chassis_control_info_.recu_stuck = (msg->payload[2] & Autocan::Vcu::VCU_MONITOR_CURRENT_OVERLIMIT_BIT) ? 1 : 0;
                    chassis_control_info_.recu_brake = (msg->payload[2] & Autocan::Vcu::VCU_MONITOR_BRAKE_BIT) ? 1 : 0;
                    // 发布到话题中
                    chassis_monitor_pub_.publish(chassis_control_info_);
                    // Keep the legacy *_stuck message fields for ROS API
                    // compatibility, but label protocol bit2 accurately in
                    // the diagnostic and retain all three raw status bytes.
                    // The VCU can repeat the same fault frame at tens of Hz.
                    // Preserve both the warning and the published status, but
                    // keep a persistent hardware state from flooding logs.
                    ROS_WARN_THROTTLE(5.0,
                             "VCU ControllerMonitor (0x23) raw=[0x%02X 0x%02X 0x%02X]: "
                             "TCU{emergency=%u timeout=%u current_overlimit=%u brake=%u} "
                             "LECU{emergency=%u timeout=%u current_overlimit=%u brake=%u} "
                             "RECU{emergency=%u timeout=%u current_overlimit=%u brake=%u}; "
                             "legacy *_stuck fields carry current_overlimit",
                             static_cast<unsigned int>(msg->payload[0]),
                             static_cast<unsigned int>(msg->payload[1]),
                             static_cast<unsigned int>(msg->payload[2]),
                             static_cast<unsigned int>(chassis_control_info_.tcu_state),
                             static_cast<unsigned int>(chassis_control_info_.tcu_timeout),
                             static_cast<unsigned int>(chassis_control_info_.tcu_stuck),
                             static_cast<unsigned int>((msg->payload[0] & Autocan::Vcu::VCU_MONITOR_BRAKE_BIT) != 0),
                             static_cast<unsigned int>(chassis_control_info_.lecu_state),
                             static_cast<unsigned int>(chassis_control_info_.lecu_timeout),
                             static_cast<unsigned int>(chassis_control_info_.lecu_stuck),
                             static_cast<unsigned int>(chassis_control_info_.lecu_brake),
                             static_cast<unsigned int>(chassis_control_info_.recu_state),
                             static_cast<unsigned int>(chassis_control_info_.recu_timeout),
                             static_cast<unsigned int>(chassis_control_info_.recu_stuck),
                             static_cast<unsigned int>(chassis_control_info_.recu_brake));
                    break;
                case Autocan::Vcu::ControlTimeout:
                    if(is_pub_control_timeout_)
                    {
                        std_msgs::Bool timeout_msg;
                        timeout_msg.data = true;
                        control_timeout_pub_.publish(timeout_msg);
                    }
                    ROS_ERROR_THROTTLE(1.0,
                                       "VCU ControlTimeout (0x24): no motion "
                                       "control command received for 200 ms "
                                       "(topic publication %s)",
                                       is_pub_control_timeout_ ? "enabled" : "disabled");
                    break;
            }
        }
        else if(msg->node_type == Autocan::Ecu::Type)
        {
            if(msg->node_seq == Autocan::Ecu::Node::Left)
            {
                if(msg->msg_type == Autocan::Ecu::CurrentSpeed)
                {
                    float wheelRadius = 0.16;
                    if(ChassisParameterHelper::areSet(chassis_parameter_))wheelRadius = chassis_parameter_.wheel_radius;
                    float speed = autolabor::build_from_little_endian<float>(msg->payload.data());
                    cur_left_vel_ = speed * wheelRadius;
                    cur_left_time_ = ros::Time::now();
                    std_msgs::Float64 vel;
                    vel.data = cur_left_vel_;
                    left_wheel_vel_pub_.publish(vel);
                }
            }
            else if (msg->node_seq == Autocan::Ecu::Node::Right)
            {
                if(msg->msg_type == Autocan::Ecu::CurrentSpeed)
                {
                    float wheelRadius = 0.16;
                    if(ChassisParameterHelper::areSet(chassis_parameter_))wheelRadius = chassis_parameter_.wheel_radius;
                    float speed = autolabor::build_from_little_endian<float>(msg->payload.data());
                    cur_right_vel_ = speed * wheelRadius;
                    cur_right_time_ = ros::Time::now();
                    std_msgs::Float64 vel;
                    vel.data = cur_right_vel_;
                    right_wheel_vel_pub_.publish(vel);
                }
            }
        }
        else if(msg->node_type == Autocan::Tcu::Type)
        {
            if(msg->msg_type == Autocan::Tcu::CurrentPosition)
            {
                cur_steer_ = autolabor::build_from_little_endian<float>(msg->payload.data());
                cur_steer_time_ = ros::Time::now();
                // 发布前轮转角
                send_wheel_angle(cur_steer_);
            }
        }
    }

    void M2Driver::handle_ackerman_msg(const geometry_msgs::Twist::ConstPtr &msg)
    {
        if(!ChassisParameterHelper::areSet(chassis_parameter_)) {
            ROS_ERROR_THROTTLE(
                1.0,
                "Rejecting Ackermann command: invalid chassis parameters "
                "max_speed=%g max_steer=%g wheelbase=%g width=%g radius=%g; "
                "sending zero MotionCtrl",
                static_cast<double>(chassis_parameter_.max_speed),
                static_cast<double>(chassis_parameter_.max_steer),
                static_cast<double>(chassis_parameter_.robot_length),
                static_cast<double>(chassis_parameter_.robot_width),
                static_cast<double>(chassis_parameter_.wheel_radius));
            driver_car(0.0f, 0.0f);
            return;
        }
        if(!m2_twist_command_is_valid(msg->linear.x, msg->angular.z)) {
            ROS_ERROR_THROTTLE(
                1.0,
                "Rejecting non-finite Ackermann command v=%g steer=%g; "
                "sending zero MotionCtrl",
                msg->linear.x,
                msg->angular.z);
            driver_car(0.0f, 0.0f);
            return;
        }
        float target_vel = clamp(static_cast<float>(msg->linear.x), -chassis_parameter_.max_speed, chassis_parameter_.max_speed);
        float target_angular = static_cast<float>(msg->angular.z);

        target_angular = clamp(target_angular, -chassis_parameter_.max_steer, chassis_parameter_.max_steer);
        // 转化为相对速度
        double relative_vel = target_vel / chassis_parameter_.max_speed;

        // 发送底盘控制指令
        driver_car(relative_vel, target_angular);
    }

    void M2Driver::handle_twist_msg(const geometry_msgs::Twist::ConstPtr &msg) {
        if(!ChassisParameterHelper::areSet(chassis_parameter_)) {
            ROS_ERROR_THROTTLE(
                1.0,
                "Rejecting Twist command: invalid chassis parameters "
                "max_speed=%g max_steer=%g wheelbase=%g width=%g radius=%g; "
                "sending zero MotionCtrl",
                static_cast<double>(chassis_parameter_.max_speed),
                static_cast<double>(chassis_parameter_.max_steer),
                static_cast<double>(chassis_parameter_.robot_length),
                static_cast<double>(chassis_parameter_.robot_width),
                static_cast<double>(chassis_parameter_.wheel_radius));
            driver_car(0.0f, 0.0f);
            return;
        }
        if(!m2_twist_command_is_valid(msg->linear.x, msg->angular.z)) {
            ROS_ERROR_THROTTLE(
                1.0,
                "Rejecting non-finite Twist command v=%g omega=%g; "
                "sending zero MotionCtrl",
                msg->linear.x,
                msg->angular.z);
            driver_car(0.0f, 0.0f);
            return;
        }
        const float requested_vel = static_cast<float>(msg->linear.x);
        const float target_vel = clamp(requested_vel, -chassis_parameter_.max_speed, chassis_parameter_.max_speed);
        const float target_angular = static_cast<float>(
            angular_velocity_after_linear_limit(
                requested_vel, target_vel, msg->angular.z));

        // Preserve both curvature when the chassis speed limit clips v, and
        // the sign of linear velocity.  A reverse command with the same
        // angular.z needs the opposite steering angle to produce the requested
        // base-frame yaw rate.
        const float steer_rad = static_cast<float>(twist_to_front_steering(
            target_vel,
            target_angular,
            chassis_parameter_.robot_length,
            chassis_parameter_.max_steer));
        // 转化为相对速度
        double relative_vel = target_vel / chassis_parameter_.max_speed;

        // 发送底盘控制指令
        driver_car(relative_vel, steer_rad);
    }

    void M2Driver::driver_car(float rel_vel, float steer_rad) {
        const M2SafeMotionControl safe_command = sanitize_m2_motion_control(
            rel_vel,
            steer_rad,
            chassis_parameter_.max_steer);
        if (!safe_command.input_valid) {
            ROS_ERROR_THROTTLE(
                1.0,
                "Invalid CAN-bound MotionCtrl rel_vel=%g steer=%g max_steer=%g; "
                "encoding fail-closed zero command",
                static_cast<double>(rel_vel),
                static_cast<double>(steer_rad),
                static_cast<double>(chassis_parameter_.max_steer));
        } else if (safe_command.was_limited) {
            ROS_WARN_THROTTLE(
                1.0,
                "Limiting CAN-bound MotionCtrl rel_vel=%g->%g steer=%g->%g",
                static_cast<double>(rel_vel),
                safe_command.relative_velocity,
                static_cast<double>(steer_rad),
                safe_command.steering_angle);
        }

        uint8_t combined_bytes[8];
        autolabor::pack_into_little_endian(
            static_cast<float>(safe_command.relative_velocity),
            combined_bytes);
        autolabor::pack_into_little_endian(
            static_cast<float>(safe_command.steering_angle),
            combined_bytes + 4);
        send_to_canbus(Autocan::Vcu::Type, Autocan::Vcu::NodeId, Autocan::Vcu::MotionCtrl, combined_bytes);
    }

    void M2Driver::handle_steer_center_set(const std_msgs::Float64::ConstPtr& msg) {
        uint8_t payload[8];
        autolabor::pack_into_little_endian((float)msg->data, payload);
        send_to_canbus(Autocan::Vcu::Type, Autocan::Vcu::NodeId, Autocan::Vcu::SteerCenterSet, payload);
    }

    void M2Driver::handle_brake_set(const std_msgs::Bool::ConstPtr& msg) {
        uint8_t payload[8] = {0};  // 保证数组足够大以存储转换后的数据
        payload[0] = msg->data ? 1 : 0;
        send_to_canbus(Autocan::Vcu::Type, Autocan::Vcu::NodeId, Autocan::Vcu::MotorBrake, payload);
    }

    void M2Driver::handle_reset_odom(const std_msgs::Empty::ConstPtr& msg) {
        send_to_canbus(Autocan::Vcu::Type, Autocan::Vcu::NodeId, Autocan::Vcu::ResetOdom);
    }

    void M2Driver::handle_emergency_brake(const std_msgs::Bool::ConstPtr& msg){
        uint8_t payload[8] = {0};
        payload[0] = msg->data ? Autocan::NodeState::Stop : Autocan::NodeState::Running;
        send_to_canbus(Autocan::Common::AllType, Autocan::Common::AllNode, Autocan::Common::Emergency, payload);
    }

    void M2Driver::send_to_canbus(const uint8_t node_type, const uint8_t node, const uint8_t msg_type) {
        autolabor_canbus_driver::CanBusService srv;
        autolabor_canbus_driver::CanBusMessage message;
        message.node_type = node_type;
        message.node_seq = node;
        message.msg_type = msg_type;
        srv.request.requests.push_back(message);
        canbus_client_.call(srv);
    }

    void M2Driver::send_to_canbus(const uint8_t node_type, const uint8_t node, const uint8_t msg_type, const uint8_t *payload) {
        autolabor_canbus_driver::CanBusService srv;
        autolabor_canbus_driver::CanBusMessage message;
        message.node_type = node_type;
        message.node_seq = node;
        message.msg_type = msg_type;
        message.payload.assign(payload, payload + 8);  // 假设payload长度总是8字节
        srv.request.requests.push_back(message);
        canbus_client_.call(srv);
    }

    void M2Driver::send_odom(const ros::TimerEvent& event) {
        ros::Time now = ros::Time::now();
        const auto is_fresh = [&](const ros::Time& stamp) {
            if (stamp.isZero()) return false;
            const double age = (now - stamp).toSec();
            return age >= 0.0 && age < sync_timeout_;
        };
        if (is_fresh(cur_vel_time_) && is_fresh(cur_left_time_) &&
            is_fresh(cur_right_time_) && is_fresh(cur_steer_time_)) {
            // Twist depends on velocity and steering feedback.  Stamp odometry
            // with the oldest contributing measurement so repeated timer
            // publications never make stale chassis data look fresh.
            ros::Time measurement_time = cur_vel_time_;
            measurement_time = std::min(measurement_time, cur_left_time_);
            measurement_time = std::min(measurement_time, cur_right_time_);
            measurement_time = std::min(measurement_time, cur_steer_time_);
            // 读取车长度
            double robotLength = 0.65;
            if(ChassisParameterHelper::areSet(chassis_parameter_))robotLength = chassis_parameter_.robot_length;
            // 计算角速度
            double angular = cur_vel_ * std::tan(cur_steer_) / robotLength;

            // 构建四元数
            tf2::Quaternion q;
            q.setRPY(0, 0, cur_odom_yaw_);

            if (publish_tf_) {
                geometry_msgs::TransformStamped transform_stamped;
                transform_stamped.header.stamp = measurement_time;
                transform_stamped.header.frame_id = odom_frame_;
                transform_stamped.child_frame_id = base_frame_;
                transform_stamped.transform.translation.x = cur_odom_x_;
                transform_stamped.transform.translation.y = cur_odom_y_;
                transform_stamped.transform.translation.z = 0.0;
                transform_stamped.transform.rotation.x = q.x();
                transform_stamped.transform.rotation.y = q.y();
                transform_stamped.transform.rotation.z = q.z();
                transform_stamped.transform.rotation.w = q.w();
                tf_broadcast_.sendTransform(transform_stamped);
            }

            nav_msgs::Odometry odom_msg;
            odom_msg.header.frame_id = odom_frame_;
            if(is_odom_child_baselink_)
            {
                odom_msg.child_frame_id = base_frame_;
            }
            odom_msg.header.stamp = measurement_time;
            odom_msg.pose.pose.position.x = cur_odom_x_;
            odom_msg.pose.pose.position.y = cur_odom_y_;
            odom_msg.pose.pose.position.z = 0;
            odom_msg.pose.pose.orientation.x = q.getX();
            odom_msg.pose.pose.orientation.y = q.getY();
            odom_msg.pose.pose.orientation.z = q.getZ();
            odom_msg.pose.pose.orientation.w = q.getW();
            odom_msg.twist.twist.linear.x = cur_vel_;
            odom_msg.twist.twist.linear.y = 0;
            odom_msg.twist.twist.angular.z = angular;
            odom_pub_.publish(odom_msg);
        }
    }

    void M2Driver::send_wheel_angle(double wheel_angle)
    {
        std_msgs::Float64 wheel_angle_msg;
        wheel_angle_msg.data = wheel_angle;
        wheel_angle_pub_.publish(wheel_angle_msg);
    }


    bool M2Driver::handle_get_chassis_parameters(autolabor_canbus_driver::ChassisParameterServer::Request &req,
                                                 autolabor_canbus_driver::ChassisParameterServer::Response &res)
    {
        if (ChassisParameterHelper::areSet(chassis_parameter_)) {
            res.success = true;
            res.parameters = chassis_parameter_;
            res.message = "Parameters are set.";
        } else {
            res.success = false;
            res.message = "Parameters not fully set.";
        }
        return true;
    }

    void M2Driver::run() {
        ros::NodeHandle nodeHandle;
        ros::NodeHandle privateNodeHandle("~");
        // 初始化机器人参数
        chassis_parameter_ = ChassisParameterHelper::createDefaultChassisParameters();
        // 读取参数
        privateNodeHandle.param<std::string>("odom_frame", odom_frame_, std::string("odom"));
        privateNodeHandle.param<std::string>("base_frame", base_frame_, std::string("base_link"));
        privateNodeHandle.param<double>("poller_rate_hz", poller_rate_hz_, 1.0);
        privateNodeHandle.param<double>(
            "status_query_rate_limit_hz", status_query_rate_limit_hz_, 4.0);
        privateNodeHandle.param<int>("pub_odom_hz", pub_odom_hz_, 10);
        privateNodeHandle.param<bool>("publish_tf", publish_tf_, false);
        privateNodeHandle.param<bool>("is_odom_child_baselink", is_odom_child_baselink_, false);
        privateNodeHandle.param<bool>("is_pub_control_timeout", is_pub_control_timeout_, true);
        sync_timeout_ = 1;  // 超时时间设置为1s
        // can消息访问
        canbus_client_ = nodeHandle.serviceClient<autolabor_canbus_driver::CanBusService>("canbus_server");
        canbus_msg_subscriber_ = nodeHandle.subscribe("/canbus_msg", 100, &M2Driver::handle_canbus_msg, this);
        // 接收控制指令
        twist_subscriber_ = nodeHandle.subscribe("/cmd_vel", 10, &M2Driver::handle_twist_msg, this);
        ackerman_subscriber_ = nodeHandle.subscribe("/ackerman_vel", 10, &M2Driver::handle_ackerman_msg, this);
        steer_center_subscriber_ = privateNodeHandle.subscribe("steer_center_bias",10,&M2Driver::handle_steer_center_set,this);
        brake_subscriber_ = privateNodeHandle.subscribe("brake_set",10,&M2Driver::handle_brake_set,this);
        reset_odom_subscriber_ = privateNodeHandle.subscribe("reset_odom",10,&M2Driver::handle_reset_odom,this);
        emergency_subscriber_ = privateNodeHandle.subscribe("emergency_stop",10,&M2Driver::handle_emergency_brake, this);
        // 发布底盘数据
        odom_pub_ = nodeHandle.advertise<nav_msgs::Odometry>("/odom", 10);
        chassis_info_pub_ = privateNodeHandle.advertise<autolabor_canbus_driver::ChassisStatusInfo>("chassis_info", 10);
        chassis_monitor_pub_ = privateNodeHandle.advertise<autolabor_canbus_driver::ChassisMonitorInfo>("chassis_monitor", 10);
        wheel_angle_pub_ = privateNodeHandle.advertise<std_msgs::Float64>("wheel_angle", 10);
        left_wheel_vel_pub_ = privateNodeHandle.advertise<std_msgs::Float64>("left_wheel_vel", 10);
        right_wheel_vel_pub_ = privateNodeHandle.advertise<std_msgs::Float64>("right_wheel_vel", 10);
        control_timeout_pub_ = privateNodeHandle.advertise<std_msgs::Bool>("control_timeout", 10);
        // 底盘参数查询服务器
        chassis_parameter_server_ = privateNodeHandle.advertiseService("chassis_parameter", &M2Driver::handle_get_chassis_parameters, this);
        // 查询定时器
        ask_parameters_timer_ = nodeHandle.createTimer(ros::Duration(1.0 / 20.0), &M2Driver::ask_robot_param, this);
        const std::size_t status_query_slots =
            safety_info_req_queue_.size() + 1;
        const double status_query_rate_hz = m2_status_query_rate_hz(
            poller_rate_hz_, status_query_slots, status_query_rate_limit_hz_);
        if (status_query_rate_hz <= 0.0) {
            ROS_FATAL("M2 status polling rates must be finite and positive and "
                      "the status request queues must not be empty");
            return;
        }
        ROS_INFO("M2 chassis status: %.3f requested safety snapshots/s, "
                 "%.3f paced queries/s (limit %.3f), %zu telemetry fields",
                 poller_rate_hz_, status_query_rate_hz,
                 status_query_rate_limit_hz_, telemetry_info_req_queue_.size());
        ask_info_timer_ = nodeHandle.createTimer(
            ros::Duration(1.0 / status_query_rate_hz),
            &M2Driver::ask_robot_info, this);
        // 里程计发送定时器
        send_odom_timer_ = nodeHandle.createTimer(ros::Duration(1.0 / pub_odom_hz_), &M2Driver::send_odom, this);
        // 开启线程循环
        ros::spin();
    }
}

int main(int argc,char **argv){
    ros::init(argc,argv,"m2_driver");
    autolabor_driver::M2Driver m2Driver;
    m2Driver.run();
    return 0;
}
