#ifndef AUTOLABOR_CANBUS_DRIVER_PM1_DRIVER_H
#define AUTOLABOR_CANBUS_DRIVER_PM1_DRIVER_H

#include <climits>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>

#include <tf2_ros/transform_broadcaster.h>

#include "autolabor_canbus_driver/CanBusMessage.h"
#include "autolabor_canbus_driver/CanBusService.h"

extern "C" {
#include "control_model/model.h"
#include "control_model/motor_map.h"
#include "control_model/optimization.h"
}


const int CANBUS_NODETYPE_ECU = 0x11;
const int CANBUS_MESSAGETYPE_ECU_TARGETSPEED = 0x01;
const int CANBUS_MESSAGETYPE_ECU_CURRENTENCODER = 0x06;

const int CANBUS_NODETYPE_TCU = 0x12;
const int CANBUS_MESSAGETYPE_TCU_TARGETANGLE = 0x01;
const int CANBUS_MESSAGETYPE_TCU_CURRENTANGLE = 0x03;

const int CANBUS_NODESEQ_BROADCAST = 0x0F;

namespace autolabor_driver {

    struct Point {
        double x;
        double y;
        double yaw;
    };


    class Pm1Driver {
    public:
        Pm1Driver();

        ~Pm1Driver() = default;

        void run();

    private:
        void ask_encoder(const ros::TimerEvent &);

        void handle_canbus_msg(const autolabor_canbus_driver::CanBusMessage::ConstPtr &msg);

        void handle_twist_msg(const geometry_msgs::Twist::ConstPtr &msg);

        void send_odom(const ros::TimerEvent& event);

        void send_wheel_angle(double wheel_angle);

        bool reset_odom(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

        void driver_car(double left, double right, double angle);

        inline int calculate_delta(long last, long current) {
            long delta = current > last
                         ?(current - last) < INT32_MAX ? (current - last) : (current - last - UINT32_MAX)
                         :(last - current) < INT32_MAX ? (current - last) : (current - last + UINT32_MAX);
            return static_cast<int>(delta);
        }

        inline std::string vector_to_string(std::vector<uint8_t> v) {
            std::stringstream ss;
            for (int i = 0; i < v.size(); i++)
                ss << std::hex << std::setw(2) << std::setfill('0') << (int) v[i] << " ";
            return ss.str();
        }

    private:
        ros::Time last_twist_time_;
        geometry_msgs::Twist twist_cache_;
        double twist_timeout_;

        std::string odom_frame_, base_frame_;
        int ecu_left_id_, ecu_right_id_, tcu_id_;
        int rate_;
        double sync_timeout_;

        float reduction_ratio_, encoder_resolution_, wheel_diameter_, wheel_spacing_, shaft_spacing_, max_speed_;
        float optimize_limit_, smooth_coefficient_;

        bool publish_tf_;

        float max_speed_rad_, smooth_coefficient_depend_;

        struct physical current_physical_{};

        float wheel_k_, rudder_k_;

        struct chassis_config_t user_config;

        ros::Time ecu_left_time_, ecu_right_time_;
        long ecu_left_last_encoder_, ecu_right_last_encoder_;
        long ecu_left_current_encoder_, ecu_right_current_encoder_;

        bool first_send_odom_flag_;
        ros::Time last_send_odom_time_;
        double accumulation_x_, accumulation_y_, accumulation_yaw_;

        tf2_ros::TransformBroadcaster br_;

        ros::Subscriber twist_subscriber_;
        ros::Subscriber canbus_msg_subscriber_;

        ros::ServiceClient canbus_client_;

        ros::ServiceServer reset_odom_service_;

        ros::Publisher odom_pub_;
        ros::Publisher wheel_angle_pub_;

        ros::Timer send_odom_timer_;
        int pub_odom_hz_;
    };

}


#endif //AUTOLABOR_CANBUS_DRIVER_PM1_DRIVER_H
