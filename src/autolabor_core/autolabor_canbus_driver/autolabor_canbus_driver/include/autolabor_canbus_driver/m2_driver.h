#ifndef AUTOLABOR_CANBUS_DRIVER_M2_DRIVER_H
#define AUTOLABOR_CANBUS_DRIVER_M2_DRIVER_H

#include <algorithm>
#include <climits>
#include <cmath>
#include <limits>
#include <queue>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>

#include <tf2_ros/transform_broadcaster.h>

#include "autolabor_canbus_driver/CanBusMessage.h"
#include "autolabor_canbus_driver/CanBusService.h"
#include "autolabor_canbus_driver/ChassisStatusInfo.h"
#include "autolabor_canbus_driver/ChassisMonitorInfo.h"
#include "autolabor_canbus_driver/ChassisParameter.h"
#include "autolabor_canbus_driver/ChassisParameterServer.h"
#include "autolabor_canbus_driver/m2_status_polling.h"
#include "autolabor_canbus_driver/m2_twist_steering.h"

namespace autolabor_driver {

    class M2Driver {
    public:
        M2Driver();
        ~M2Driver() = default;
        void run();

    private:
        // 查询指令
        void ask_robot_param(const ros::TimerEvent &);
        void ask_robot_info(const ros::TimerEvent &);
        // 服务
        bool handle_get_chassis_parameters(autolabor_canbus_driver::ChassisParameterServer::Request &req,
                                                     autolabor_canbus_driver::ChassisParameterServer::Response &res);
        // 控制指令
        void handle_twist_msg(const geometry_msgs::Twist::ConstPtr &msg);
        void handle_ackerman_msg(const geometry_msgs::Twist::ConstPtr &msg);
        void handle_steer_center_set(const std_msgs::Float64::ConstPtr& msg);
        void handle_brake_set(const std_msgs::Bool::ConstPtr& msg);
        void handle_reset_odom(const std_msgs::Empty::ConstPtr& msg);
        void handle_emergency_brake(const std_msgs::Bool::ConstPtr& msg);
        // CAN协议
        void handle_canbus_msg(const autolabor_canbus_driver::CanBusMessage::ConstPtr &msg);
        void send_to_canbus(const uint8_t node_type, const uint8_t node, const uint8_t msg_type);
        void send_to_canbus(const uint8_t node_type, const uint8_t node, const uint8_t msg_type, const uint8_t *payload);
        // 发送Odom消息
        void send_odom(const ros::TimerEvent& event);
        // 发送模拟IMU消息
        void publish_imu_data(const ros::TimerEvent& event);
        // 发送前轮转角
        void send_wheel_angle(double wheel_angle);
        // 发送控制车辆指令
        void driver_car(float rel_vel,  float steer_rad);

        template <typename T>
        inline T clamp(T value, T min, T max) {
            return std::max(min, std::min(value, max));
        }

    private:
        std::queue<autolabor_canbus_driver::CanBusMessage> param_req_queue_;
        std::queue<autolabor_canbus_driver::CanBusMessage> safety_info_req_queue_;
        std::queue<autolabor_canbus_driver::CanBusMessage> telemetry_info_req_queue_;
        M2SafetyQueryRetryGate safety_query_retry_;
        M2QueryIntervalSchedule status_query_schedule_;
        std::size_t completed_safety_fields_ = 0;
        autolabor_canbus_driver::ChassisStatusInfo chassis_status_info_;
        autolabor_canbus_driver::ChassisMonitorInfo chassis_control_info_;
        autolabor_canbus_driver::ChassisParameter chassis_parameter_;

        float cur_odom_x_ = 0,cur_odom_y_ = 0,cur_odom_yaw_ = 0;
        float cur_vel_ = 0,cur_left_vel_ = 0,cur_right_vel_ = 0,cur_steer_ = 0;
        ros::Time cur_vel_time_, cur_left_time_, cur_right_time_, cur_steer_time_;
        double poller_rate_hz_;
        double status_query_rate_limit_hz_;
        double status_query_jitter_fraction_;
        int status_query_max_attempts_;
        int pub_odom_hz_;
        bool is_odom_child_baselink_;
        bool is_pub_control_timeout_;
        // 里程计计算
        double sync_timeout_;
        // tf变换
        bool publish_tf_;
        tf2_ros::TransformBroadcaster tf_broadcast_;
        // 坐标系
        std::string odom_frame_, base_frame_;
        // Can协议
        ros::Subscriber canbus_msg_subscriber_;
        ros::ServiceClient canbus_client_;
        // 服务器
        ros::ServiceServer chassis_parameter_server_;
        // 订阅指令
        ros::Subscriber twist_subscriber_,ackerman_subscriber_;
        ros::Subscriber brake_subscriber_,steer_center_subscriber_,reset_odom_subscriber_,emergency_subscriber_;
        // 发布话题
        ros::Publisher chassis_info_pub_;
        ros::Publisher chassis_monitor_pub_;
        ros::Publisher odom_pub_;
        ros::Publisher left_wheel_vel_pub_;
        ros::Publisher right_wheel_vel_pub_;
        ros::Publisher wheel_angle_pub_;
        ros::Publisher control_timeout_pub_;
        // 查询定时器
        ros::Timer ask_parameters_timer_;
        ros::Timer ask_info_timer_;
        ros::Timer send_odom_timer_;

    };


    class ChassisParameterHelper {
    public:
        // 创建默认参数实例
        static autolabor_canbus_driver::ChassisParameter createDefaultChassisParameters() {
            autolabor_canbus_driver::ChassisParameter params;
            params.max_speed = std::numeric_limits<float>::infinity();
            params.max_steer = std::numeric_limits<float>::infinity();
            params.robot_width = std::numeric_limits<float>::infinity();
            params.robot_length = std::numeric_limits<float>::infinity();
            params.wheel_radius = std::numeric_limits<float>::infinity();
            return params;
        }

        // 所有参数必须为有限值；控制相关的速度、转角和轴距还必须为正值。
        static bool areSet(const autolabor_canbus_driver::ChassisParameter& params) {
            return m2_chassis_parameters_are_valid(
                params.max_speed,
                params.max_steer,
                params.robot_width,
                params.robot_length,
                params.wheel_radius);
        }
    };
}



#endif //PROJECT_M2_DRIVER_H
