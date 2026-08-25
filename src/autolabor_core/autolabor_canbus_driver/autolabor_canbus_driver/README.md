# Autolabor M2 Chassis Driver 使用手册

## 1. 介绍

Autolabor ROS驱动模块包括**CANBus驱动**和**Autolabor CAN系列(PM1/M2) 底盘驱动**，主要用于实现与Autolabor_CANbus模块的通信，并通过线速度和角速度来控制Autolabor CAN系列底盘的行驶。该模块提供了全面的底盘控制策略和状态反馈机制，使得底盘的操作既直观又高效。

### 特性

- **CANBus数据获取和控制**: 直接从CANBus网络获取数据并通过CAN指令控制车辆。能够获取动力轮和转向轮编码器的原始数值，并独立控制动力轮的转速及转向轮的绝对转向角度。
- **高级速度控制**: 通过线速度和角速度直接控制移动底盘，无需单独控制后轮转向。这一特性简化了驱动操作并增强了操作的直观性。
- **实时位置信息**: 提供实时的机器人底盘位置信息，便于进行闭环控制，确保导航和定位的精确性。
- **电子差速控制**: 采用双动力轮电子差速控制，保证机器人在行驶过程中始终满足阿克曼原理，从而提高行驶稳定性和精度。
- **运动优化**: 根据后轮转向动态优化车辆的运动速度，确保在维持行驶精度的前提下，提升车辆行驶的流畅性。
- **驻车和急停控制**: 驱动包括了高级的驻车和急停特性，能够在紧急情况下快速响应，立即停车，保障操作安全。
- **电量反馈和状态监控**: 驱动能够实时监控底盘的电量状态，并反馈至ROS系统，同时持续监控底盘各部件的状态，及时报告任何异常情况，确保系统的可靠运行。

## 2. 编译与安装

### 2.1 引入ROS环境变量
在启动ROS相关程序前，确保ROS环境变量已经正确配置。在终端执行以下命令以导入环境变量：
```bash
source /opt/ros/noetic/setup.bash
```

### 2.2 添加串口权限或配置udev规则
为确保程序可以正常访问底盘的串口，需要给予相应的权限。可以通过以下命令临时赋予权限：
```bash
sudo chmod 666 /dev/ttyUSB0
```
或者设置udev规则，为串口设定固定的设备名称，详情参见[Autolabor官方文档](http://www.autolabor.com.cn/usedoc/navigationKit2/common/q_a/doc1#1)。

### 2.3 修改launch文件
在启动底盘前，需要在launch文件中指定正确的串口地址。编辑`m2keyboard.launch`文件，确保`port_name`的值反映了当前设备的串口地址：
```xml
<param name="port_name" value="/dev/ttyUSB0" />
```

### 2.4 启动launch
编译并加载ROS工作环境后，使用以下命令启动底盘：
```bash
roslaunch autolabor_canbus_driver m2keyboard.launch
```
确保在图形界面的终端运行以避免X11Server相关的错误。


## 3. 节点使用详细信息

### 3.1 启动launch
完成配置后，通过`roslaunch`命令启动底盘控制程序。这将激活包括底盘驱动、键盘控制以及其他必要节点。确保在图形用户界面的终端中运行此命令，以避免遇到X11Server相关的错误。

### 3.2 控制运动

#### a. 键盘控制
运行键盘控制节点后，使用键盘上的指令控制底盘前进、后退、左转和右转。观察RViz中的`base_link`是否根据指令正确移动。此外，监测各轮的运动是否平滑且响应及时。

#### b. cmd_vel vs ackerman_vel
- **`cmd_vel`（标准速度控制）**:
  使用`geometry_msgs/Twist`消息类型发送线速度和角速度指令，控制底盘的整体移动方向和速度。

- **`ackerman_vel`（阿克曼转向速度控制）**:
  除了标准的线速度和角速度外，额外控制前轮的转角，适用于需要精确转向控制的场景。

### 3.3 使用rqt_gui工具发送指令和服务

#### a. 发送控制指令
通过`rqt_gui`发送以下指令：
- **紧急停车** (`/m2_driver/emergency_stop`): 立即停止底盘运动。
- **刹车** (`/m2_driver/brake_set`): 控制底盘的刹车系统。
- **里程计清零** (`/m2_driver/reset_odom`): 重置底盘的里程计数据。
- **转向中心对齐** (`/m2_driver/steer_center_bias`): 调整转向轮的中心位置。

#### b. 调用服务节点
- **查询底盘参数** (`/m2_driver/chassis_parameter`): 获取或设置底盘的配置参数，如轮间距、轮直径等。

#### c. 绘图监听
使用rqt的绘图功能来实时监控和分析以下数据：
- **前轮转角** (`/m2_driver/wheel_angle`): 显示实时转向角度。
- **左右轮速度** (`/m2_driver/left_wheel_vel`, `/m2_driver/right_wheel_vel`): 监控各动力轮的速度。

#### d. 话题查看
关注以下话题以获取底盘状态和电量信息：
- **车辆状态** (`/m2_driver/chassis_info`): 提供底盘的详细状态，如电量、温度等。
- **电量信息** (`/m2_driver/chassis_monitor`): 监测底盘的电池电量和健康状态。

### 3.4 参数说明
在底盘驱动的配置中，可以通过设置以下参数来定制驱动的行为和性能：

- **`odom_frame`**: 用于发布里程计信息的坐标系名称，默认为`"odom"`。这是底盘位置信息的参考坐标系。
- **`base_frame`**: 底盘的基础坐标系名称，默认为`"base_link"`。这是底盘上各个组件和传感器的相对参考点。
- **`poller_rate_hz`**: 设置三项急停安全状态的目标轮询频率，默认为 1 Hz。驱动每个周期依次查询硬急停、软急停、手柄急停，再轮询一项电池遥测；每次只发送一个 CAN 查询，并由 `status_query_rate_limit_hz`（默认 4 Hz）限制为 VCU 可持续处理的速率。五项电池遥测在默认配置下各约每 5 秒更新一次。
- **`publish_tf`**: 是否发布TF变换数据，布尔值，用于控制是否在ROS网络中发布从`odom`到`base_link`的转换，默认关闭。

### 3.5 话题订阅与发布
#### 订阅话题
底盘驱动节点订阅以下话题，以接收外部控制命令和系统指令：

- **`/cmd_vel`** (`geometry_msgs/Twist`): 接收标凈的速度指令，控制底盘的线速度和角速度。
- **`/ackerman_vel`** (`geometry_msgs/Twist`): 接收针对阿克曼转向系统的速度和转向角度指令。
- **`/m2_driver/steer_center_bias`** (`std_msgs/Float64`): 接收用于调整转向中心的偏置值，用于微调转向准确性。
- **`/m2_driver/brake_set`** (`std_msgs/Bool`): 接收制动指令，用于控制底盘的停止和启动。
- **`/m2_driver/reset_odom`** (`std_msgs/Empty`): 接收里程计重置指令，用于归零里程计数据。
- **`/m2_driver/emergency_stop`** (`std_msgs/Bool`): 接收紧急停车指令，用于立即停止所有底盘运动。

#### 发布话题
底盘驱动节点发布以下话题，提供底盘的状态信息和运动反馈：

- **`/odom`** (`nav_msgs/Odometry`): 发布底盘的里程计信息，包括位置和速度。
- **`/m2_driver/chassis_info`** (`autolabor_canbus_driver/ChassisStatusInfo`): 发布底盘的状态信息，如电量、温度等。
- **`/m2_driver/chassis_monitor`** (`autolabor_canbus_driver/ChassisMonitorInfo`): 发布底盘的监控信息，提供实时的健康状况和性能数据。
- **`/m2_driver/wheel_angle`** (`std_msgs/Float64`): 发布当前前轮的转角，单位为弧度。
- **`/m2_driver/left_wheel_vel`** (`std_msgs/Float64`): 发布左轮的速度，单位为米/秒。
- **`/m2_driver/right_wheel_vel`** (`std_msgs/Float64`): 发布右轮的速度，单位为米/秒。

### 3.6 服务节点
底盘驱动提供以下ROS服务以支持参数查询和配置：

- **`/m2_driver/chassis_parameter`** (`autolabor_canbus_driver/ChassisParameterServer`): 这个服务端点用于查询或设置底盘的参数。如果参数已经设置，则返回当前参数；如果未设置，则返回失败。这使得用户可以动态地调整底盘配置，以适应不同的工作条件和需求。
