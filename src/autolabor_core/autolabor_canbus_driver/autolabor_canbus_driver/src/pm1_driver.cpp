#include "autolabor_canbus_driver/pm1_driver.h"
#include "utilities/big_endian_transform.h"

#include <cmath>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

namespace autolabor_driver {


    Pm1Driver::Pm1Driver() : first_send_odom_flag_(true) {

    }

    void Pm1Driver::ask_encoder(const ros::TimerEvent &) {
        autolabor_canbus_driver::CanBusService srv;

        autolabor_canbus_driver::CanBusMessage ask_ecu_encoder;
        ask_ecu_encoder.node_type = CANBUS_NODETYPE_ECU;
        ask_ecu_encoder.node_seq = CANBUS_NODESEQ_BROADCAST;
        ask_ecu_encoder.msg_type = CANBUS_MESSAGETYPE_ECU_CURRENTENCODER;
        srv.request.requests.push_back(ask_ecu_encoder);

        autolabor_canbus_driver::CanBusMessage ask_tcu_encoder;
        ask_tcu_encoder.node_type = CANBUS_NODETYPE_TCU;
        ask_tcu_encoder.node_seq = CANBUS_NODESEQ_BROADCAST;
        ask_tcu_encoder.msg_type = CANBUS_MESSAGETYPE_TCU_CURRENTANGLE;
        srv.request.requests.push_back(ask_tcu_encoder);

        canbus_client_.call(srv);
    }

    void Pm1Driver::send_odom(const ros::TimerEvent& event) {
        ros::Time now = ros::Time::now();
            if (first_send_odom_flag_) {
                ecu_left_last_encoder_ = ecu_left_current_encoder_;
                ecu_right_last_encoder_ = ecu_right_current_encoder_;
                last_send_odom_time_ = now;
                accumulation_x_ = 0.0;
                accumulation_y_ = 0.0;
                accumulation_yaw_ = 0.0;
                first_send_odom_flag_ = false;
            } else {
                double delta_time = (now - last_send_odom_time_).toSec();

                int delta_left = calculate_delta(ecu_left_last_encoder_, ecu_left_current_encoder_);
                int delta_right = calculate_delta(ecu_right_last_encoder_, ecu_right_current_encoder_);

                last_send_odom_time_ = now;
                ecu_left_last_encoder_ = ecu_left_current_encoder_;
                ecu_right_last_encoder_ = ecu_right_current_encoder_;

                double delta_dis = RAD_OF((delta_left + delta_right) / 2.0, wheel_k_) * user_config.radius;
                double delta_theta = RAD_OF((delta_right - delta_left) / wheel_spacing_, wheel_k_) * user_config.radius;

                double deltaX, deltaY;
                if (delta_theta == 0) {
                    deltaX = delta_dis;
                    deltaY = 0.0;
                } else {
                    deltaX = delta_dis * (sin(delta_theta) / delta_theta);
                    deltaY = delta_dis * ((1 - cos(delta_theta)) / delta_theta);
                }

                accumulation_x_ += (cos(accumulation_yaw_) * deltaX - sin(accumulation_yaw_) * deltaY);
                accumulation_y_ += (sin(accumulation_yaw_) * deltaX + cos(accumulation_yaw_) * deltaY);
                accumulation_yaw_ += delta_theta;

                tf2::Quaternion q;
                q.setRPY(0, 0, accumulation_yaw_);

                if (publish_tf_) {
                    geometry_msgs::TransformStamped transform_stamped;
                    transform_stamped.header.stamp = now;
                    transform_stamped.header.frame_id = odom_frame_;
                    transform_stamped.child_frame_id = base_frame_; 
                    transform_stamped.transform.translation.x = accumulation_x_;
                    transform_stamped.transform.translation.y = accumulation_y_;
                    transform_stamped.transform.translation.z = 0.0;
                    transform_stamped.transform.rotation.x = q.x();
                    transform_stamped.transform.rotation.y = q.y();
                    transform_stamped.transform.rotation.z = q.z();
                    transform_stamped.transform.rotation.w = q.w();
                    br_.sendTransform(transform_stamped);
                }

                nav_msgs::Odometry odom_msg;
                odom_msg.header.frame_id = odom_frame_;
                odom_msg.child_frame_id = base_frame_;
                odom_msg.header.stamp = now;
                odom_msg.pose.pose.position.x = accumulation_x_;
                odom_msg.pose.pose.position.y = accumulation_y_;
                odom_msg.pose.pose.position.z = 0;
                odom_msg.pose.pose.orientation.x = q.getX();
                odom_msg.pose.pose.orientation.y = q.getY();
                odom_msg.pose.pose.orientation.z = q.getZ();
                odom_msg.pose.pose.orientation.w = q.getW();
                odom_msg.twist.twist.linear.x = delta_dis / delta_time;
                odom_msg.twist.twist.linear.y = 0;
                odom_msg.twist.twist.angular.z = delta_theta / delta_time;

                odom_pub_.publish(odom_msg);
            }
    }

    void Pm1Driver::send_wheel_angle(double wheel_angle) {
        std_msgs::Float64 wheel_angle_msg;
        wheel_angle_msg.data = wheel_angle;
        wheel_angle_pub_.publish(wheel_angle_msg);
    }


    void Pm1Driver::driver_car(double left, double right, double angle) {
        autolabor_canbus_driver::CanBusService srv;

        autolabor_canbus_driver::CanBusMessage ecu_left_target_speed;
        ecu_left_target_speed.node_type = CANBUS_NODETYPE_ECU;
        ecu_left_target_speed.node_seq = static_cast<unsigned char>(ecu_left_id_);
        ecu_left_target_speed.msg_type = CANBUS_MESSAGETYPE_ECU_TARGETSPEED;
        ecu_left_target_speed.payload = autolabor::pack(static_cast<int32_t>(PULSES_OF(left, wheel_k_)));
        srv.request.requests.push_back(ecu_left_target_speed);

        autolabor_canbus_driver::CanBusMessage ecu_right_target_speed;
        ecu_right_target_speed.node_type = CANBUS_NODETYPE_ECU;
        ecu_right_target_speed.node_seq = static_cast<unsigned char>(ecu_right_id_);
        ecu_right_target_speed.msg_type = CANBUS_MESSAGETYPE_ECU_TARGETSPEED;
        ecu_right_target_speed.payload = autolabor::pack(static_cast<int32_t>(PULSES_OF(right, wheel_k_)));
        srv.request.requests.push_back(ecu_right_target_speed);

        autolabor_canbus_driver::CanBusMessage tcu_target_angle;
        tcu_target_angle.node_type = CANBUS_NODETYPE_TCU;
        tcu_target_angle.node_seq = static_cast<unsigned char>(tcu_id_);
        tcu_target_angle.msg_type = CANBUS_MESSAGETYPE_TCU_TARGETANGLE;
        tcu_target_angle.payload = autolabor::pack(static_cast<int16_t>(PULSES_OF(angle, rudder_k_)));
        srv.request.requests.push_back(tcu_target_angle);

        canbus_client_.call(srv);
    }

    void Pm1Driver::handle_canbus_msg(const autolabor_canbus_driver::CanBusMessage::ConstPtr &msg) {
        if (msg->payload.empty()) {
            ROS_DEBUG_STREAM("RECEIVE_CANBUS_MESSAGE: " << " NODE_TYPE:" << (int) msg->node_type << ","
                                                        << " NODE_SEQ:" << (int) msg->node_seq << ","
                                                        << " MSG_TYPE:" << (int) msg->msg_type);
        } else {
            ROS_DEBUG_STREAM("RECEIVE_CANBUS_MESSAGE: " << " NODE_TYPE:" << (int) msg->node_type << ","
                                                        << " NODE_SEQ:" << (int) msg->node_seq << ","
                                                        << " MSG_TYPE:" << (int) msg->msg_type << ","
                                                        << " MSG_PAYLOAD:" << vector_to_string(msg->payload));
        }

        if ((msg->node_type == CANBUS_NODETYPE_ECU) && (msg->msg_type == CANBUS_MESSAGETYPE_ECU_CURRENTENCODER) && (!msg->payload.empty())) {
            if (msg->node_seq == ecu_left_id_) {
                ecu_left_time_ = ros::Time::now();
                ecu_left_current_encoder_ = autolabor::build<uint32_t>(msg->payload.data());
            } else if (msg->node_seq == ecu_right_id_) {
                ecu_right_time_ = ros::Time::now();
                ecu_right_current_encoder_ = autolabor::build<uint32_t>(msg->payload.data());
            }
        } else if ((msg->node_type == CANBUS_NODETYPE_TCU) && (msg->msg_type == CANBUS_MESSAGETYPE_TCU_CURRENTANGLE) && (!msg->payload.empty())) {
            int16_t wheel_spin_encoder = autolabor::build<int16_t>(msg->payload.data());
            float wheel_angle = RAD_OF(wheel_spin_encoder, rudder_k_);
            current_physical_.rudder = wheel_angle;
            send_wheel_angle(wheel_angle);
        }
    }

    void Pm1Driver::handle_twist_msg(const geometry_msgs::Twist::ConstPtr &msg) {
    // 更新上次接收到twist消息的时间
    last_twist_time_ = ros::Time::now();

    // 缓存接收到的twist消息
    twist_cache_ = *msg;

    // 创建目标速度结构体，并将接收到的速度信息转换为目标速度
    struct velocity target_velocity{static_cast<float>(twist_cache_.linear.x), static_cast<float>(twist_cache_.angular.z)};

    // 将目标速度转换为物理参数
    auto target_physical = velocity_to_physical(&target_velocity, &user_config);

    // 对物理参数进行合法化处理，确保不超过最大速度
    legalize_physical(&target_physical, max_speed_rad_);

    // 如果物理参数非法或超时未接收到新消息，将目标物理参数设置为停止状态
    if (std::isnan(target_physical.rudder) || (ros::Time::now() - last_twist_time_).sec >= twist_timeout_) {
        target_physical = {0, current_physical_.rudder};
    }

    // 优化物理参数
    struct physical optimize_physical_ = optimize(&target_physical, &current_physical_, optimize_limit_, smooth_coefficient_depend_);

    // 将优化后的物理参数转换为轮子参数
    struct wheels optimize_wheels = physical_to_wheels(&optimize_physical_, &user_config);

    // 驱动汽车
    driver_car(optimize_wheels.left, optimize_wheels.right, target_physical.rudder);

    // 更新当前物理参数的速度
    current_physical_.speed = optimize_physical_.speed;
}

    bool Pm1Driver::reset_odom(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        first_send_odom_flag_ = true;
    }

    void Pm1Driver::run() {
        ros::NodeHandle node;
        ros::NodeHandle private_node("~");

        private_node.param<std::string>("odom_frame", odom_frame_, std::string("odom"));
        private_node.param<std::string>("base_frame", base_frame_, std::string("base_link"));

        private_node.param<int>("ecu_left_id", ecu_left_id_, 0);
        private_node.param<int>("ecu_right_id", ecu_right_id_, 1);
        private_node.param<int>("tcu_id", tcu_id_, 0);
        private_node.param<int>("rate", rate_,20);

        private_node.param<float>("reduction_ratio", reduction_ratio_, 20.0);
        private_node.param<float>("encoder_resolution", encoder_resolution_, 1600.0);
        private_node.param<float>("wheel_diameter", wheel_diameter_, 0.20);
        private_node.param<float>("wheel_spacing", wheel_spacing_, 0.4137);
        private_node.param<float>("shaft_spacing", shaft_spacing_, 0.317);
        private_node.param<float>("max_speed", max_speed_, 2.0);
        max_speed_rad_ = max_speed_ / (wheel_diameter_ / 2);

        private_node.param<double>("twist_timeout", twist_timeout_, 1.0);

        private_node.param<float>("optimize_limit", optimize_limit_, static_cast<const float>(M_PI / 4));
        private_node.param<float>("smooth_coefficient", smooth_coefficient_, 1.0);
        private_node.param<bool>("publish_tf", publish_tf_, true);
        private_node.param<int>("pub_odom_hz", pub_odom_hz_, 10);
        smooth_coefficient_depend_ = smooth_coefficient_ * (max_speed_rad_ / rate_);

        user_config.width = wheel_spacing_;
        user_config.length = shaft_spacing_;
        user_config.radius = wheel_diameter_ / 2;

        sync_timeout_ = 10;

        wheel_k_ = static_cast<float>(2 * M_PI / (reduction_ratio_ * encoder_resolution_));
        rudder_k_ = static_cast<float>(2 * M_PI / 16384);

        canbus_msg_subscriber_ = node.subscribe("/canbus_msg", 100, &Pm1Driver::handle_canbus_msg, this);
        twist_subscriber_ = node.subscribe("/cmd_vel", 10, &Pm1Driver::handle_twist_msg, this);
        canbus_client_ = node.serviceClient<autolabor_canbus_driver::CanBusService>("canbus_server");

        odom_pub_ = node.advertise<nav_msgs::Odometry>("/odom", 10);
        wheel_angle_pub_ = node.advertise<std_msgs::Float64>("/wheel_angle", 10);

        reset_odom_service_ = node.advertiseService("reset_odom", &Pm1Driver::reset_odom, this);

        ros::Timer ask_encoder_timer = node.createTimer(ros::Duration(1.0 / rate_), &Pm1Driver::ask_encoder, this);
        // 里程计发送定时器
        send_odom_timer_ = node.createTimer(ros::Duration(1.0 / pub_odom_hz_), &Pm1Driver::send_odom, this);

        ros::spin();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pm1_driver");
    autolabor_driver::Pm1Driver driver;
    driver.run();
    return 0;
}