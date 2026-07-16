//
// Created by QP on 2023/11/29.
//

#ifndef AUTOLABOR_CANBUS_DRIVER_AUTOCAN_H
#define AUTOLABOR_CANBUS_DRIVER_AUTOCAN_H

#include <cstdint>

namespace autolabor_driver {

    class Autocan {
    public:
        static const int HeadLength = 5;
        static const int DataLength = 8;
        static const int CrcLength = 1;

        static const int CrcStartIdx = 1;
        static const int CrcReqEndIdx = 5;
        static const int CrcDataEndIdx = 13;

        static const uint8_t StartFmx = 0xfe;

        static const uint8_t EveryType = 0x3f;
        static const uint8_t EveryIndex = 0x0f;

        static const uint8_t State = 0x80;
        static const uint8_t Emerge = 0xff;

        struct Vcu {
        public:
            static const uint8_t Type = 0x10;
            static const uint8_t NodeId = 0x00;
            // 消息类型
            // 控制消息
            static const uint8_t MotionCtrl = 0x01;         // 运动控制
            static const uint8_t ResetOdom = 0x02;          // 重置里程计
            static const uint8_t MotorBrake = 0x03;         // 电机刹车
            static const uint8_t SteerCenterSet = 0x04;     // 舵机中心设置
            // 查询消息
            static const uint8_t BatteryPercent = 0x11;     // 电量百分比
            static const uint8_t RemainSec = 0x12;          // 剩余时间
            static const uint8_t RemainCapacity = 0x13;     // 剩余容量
            static const uint8_t BatteryVoltage = 0x14;     // 电池电压
            static const uint8_t BatteryCurrent = 0x15;     // 电池电流
            static const uint8_t HardEmergency = 0x17;      // 硬件急停
            static const uint8_t SoftEmergency = 0x18;      // 软件急停
            static const uint8_t GamepadEmergency = 0x19;   // 手柄急停
            static const uint8_t MaxSpeed = 0x1a;           // 最大速度
            static const uint8_t MaxSteer = 0x1b;           // 最大转角
            static const uint8_t RobotWidth = 0x1c;         // 机器人宽度
            static const uint8_t RobotLength = 0x1d;        // 机器人长度
            static const uint8_t WheelRadius = 0x1e;        // 轮半径
            // 反馈消息
            static const uint8_t CurrentVelocity = 0x20;    // 当前速度
            static const uint8_t OdomXy = 0x21;             // 里程计xy
            static const uint8_t OdomTheta = 0x22;          // 里程计Yaw
            static const uint8_t ControllerMonitor = 0x23;  // 控制器监控，前三个字节：[0]:TCU状态；[1]:左轮ECU状态 [2]:右轮ECU状态
                                                            // 每个字节0位为急停，1位为数据超时，2位为电流超限，3位为刹车状态。该消息在控制发送时，如果发现有控制器异常，会向上发送
            static const uint8_t ControlTimeout = 0x24;     // 控制超时


            // 定义位掩码为公共静态常量
            static constexpr uint8_t VCU_MONITOR_STATUS_BIT = 0x01;  // 第0位，表示状态
            static constexpr uint8_t VCU_MONITOR_DATA_TIMEOUT_BIT = 0x02;  // 第1位，表示数据超时
            static constexpr uint8_t VCU_MONITOR_CURRENT_OVERLIMIT_BIT = 0x04;  // 第2位，表示电流超限
            // 兼容现有 ChassisMonitorInfo 的 *_stuck 字段名；协议含义是电流超限。
            static constexpr uint8_t VCU_MONITOR_STUCK_BIT = VCU_MONITOR_CURRENT_OVERLIMIT_BIT;
            static constexpr uint8_t VCU_MONITOR_BRAKE_BIT = 0x08;  // 第3位，表示刹车
        };

        struct Ecu {
        public:
            static const uint8_t Type = 0x11;
            // 消息类型
            static const uint8_t CurrentSpeed = 0x11;       // 当前速度，单位rad/s

            class Node {
            public:
                static const uint8_t Left = 1;
                static const uint8_t Right = 0;
            };
        };

        struct Tcu {
        public:
            static const uint8_t Type = 0x12;
            // 消息类型
            static const uint8_t CurrentPosition = 0x11;    // 前轮转角，单位rad
        };

        struct Common {
        public:
            static const uint8_t AllType = 0x3F;
            static const uint8_t AllNode = 0x0F;
            static const uint8_t State = 0x80;
            static const uint8_t Emergency = 0xFF;
        };

        enum NodeState : uint8_t {
            Stop = 0xFF,
            Running = 0x10,
        };

        enum RtResult {
            RtOk = 0,
            RtError = -1,
            RtTimeout = -2,
        };
    };
}

#endif //AUTOLABOR_CANBUS_DRIVER_AUTOCAN_H
