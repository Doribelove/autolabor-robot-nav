# Autolabor M2 GPS Qt 模式新手交接手册

> 适用工作区：`/home/slam/robot_ws`
>
> 适用系统：Ubuntu + ROS Noetic
>
> 适用车辆：Autolabor M2（阿克曼转向）
>
> 最后核对日期：2026-07-28

这份文档写给第一次接触 ROS、Qt 和本项目的人。正常情况下，只需要完成一次
串口权限设置、雷达网络设置和编译，以后每次使用只运行一条启动命令。

本文重点是“GPS 导航 + Qt 操作台”模式。不要在没有培训、没有空旷场地、没有
物理急停的情况下让车辆运动。

---

## 1. 先记住最重要的三件事

1. **物理急停永远是第一安全手段。** 软件取消、关闭 Qt、按 `Ctrl+C` 都不能
   替代物理急停。操作员必须站在安全位置并全程手持物理急停。
2. **第一次测试只使用 `0.3 m/s`。** 不要因为界面显示正常就直接使用
   `2.7 m/s`。
3. **必须看到准确的就绪文字后才能发目标：**

   ```text
   Robot bringup is running in gps mode.
   ```

   Qt 窗口打开不代表导航已经就绪。Qt 可以在导航故障时以诊断模式继续打开。

---

## 2. 这套系统在做什么

车辆使用：

- CAN 总线控制 M2 底盘；
- Livox Mid-360 激光雷达感知障碍物；
- FAST_LIO 只进行点云配准，为激光扫描提供数据；
- 双天线 GNSS 提供车辆位置和车头方向；
- move_base + TEB 计算行驶路线和速度；
- Qt 界面显示状态、地图、相机，并用于发送 GPS 目标；
- GPS/FOD 安全仲裁器保证只有一个模块可以向底盘发送速度。

普通 GPS 经纬度目标的完整数据链是：

```text
Qt 手工目标或 RabbitMQ 目标
  -> /gps/goal_fix
  -> GPS 远距离目标管理器
  -> 每次生成最远约 15 m 的滚动子目标
  -> move_base + TEB
  -> /cmd_vel_navigation
  -> GPS 目标速度安全继电器
  -> /cmd_vel_gps
  -> GPS/FOD 安全仲裁器
  -> /cmd_vel
  -> M2 底盘驱动
```

Qt 本身不直接发布 `/cmd_vel`，不能绕开安全控制链。

---

## 3. 硬件和接口

默认接口如下：

| 用途 | 默认设备 | 说明 |
|---|---|---|
| M2 CAN 转串口 | `/dev/ttyUSB0` | 启动时会先做 CAN 通信检查 |
| 双天线 GNSS | `/dev/ttyUSB1` | 默认波特率 `115200` |
| Livox Mid-360 | `eth2` 以太网 | 序列号 `47MDMBB0030112`，雷达 IP `192.168.1.112` |
| Hikrobot 相机 | USB/网口，按现场安装 | 使用前关闭会独占相机的 MVS 桌面客户端 |
| 物理急停 | 车辆硬件 | 每次运动试验都必须可立即触发 |

USB 设备编号可能在重新插拔后变化。若 CAN 和 GPS 对调，可在启动时临时指定：

```bash
CAN_PORT=/dev/ttyUSB1 GPS_PORT=/dev/ttyUSB0 \
./scripts/operator_all_in_one.sh 0.3 cruise
```

只有确认设备实际对应关系后才能这样改，不能靠猜测。

### 3.1 当前新雷达的固定信息

2026-07-28 更换后的雷达信息如下：

```text
型号：Livox Mid-360
序列号：47MDMBB0030112
雷达 IP：192.168.1.112
雷达 MAC：58:b8:58:79:18:dd
工控机网口：eth2
工控机 IP：192.168.1.50/24
```

虽然它是更新版本的硬件，但设备发现包自报类型为 `9`，即标准 Mid-360。不要把
启动文件改成 Mid-360S；Mid-360S 的设备类型是 `35`，配置错型号时可能仍能
`ping` 通，但不会发布点云。

特别注意：Mid-360 的 RJ45 网口**不能接 PoE 设备或 PoE 交换机口**；接线时还要
核对供电电压、极性，并避免两台雷达的激光束正对。Livox 官方说明这些错误可能造成
不可逆损坏，见 [Mid-360 官方安全说明](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/mid360.html)。

本机 NetworkManager 配置 `有线连接 2` 已经设成固定地址，并且没有网关和 DNS，
因此不会抢走 Wi-Fi 的上网默认路由。每次开机后只需检查：

```bash
ip -brief address show eth2
ping -I eth2 -c 2 -W 1 192.168.1.112
```

正常应看到：

```text
eth2  UP  192.168.1.50/24
2 packets transmitted, 2 received
```

中文系统的 `ping` 统计文字可能不同，只要“已接收 2 个包、0% 丢包”即可。

只有当前机的配置丢失时，才执行下面的一次性恢复命令：

```bash
nmcli connection modify '有线连接 2' \
  ipv4.method manual \
  ipv4.addresses 192.168.1.50/24 \
  ipv4.gateway '' \
  ipv4.dns '' \
  ipv4.never-default yes \
  ipv6.method disabled \
  connection.autoconnect yes

nmcli connection up '有线连接 2' ifname eth2
```

如果另一台电脑的连接名称不是 `有线连接 2`，先用下面命令查名称，不能照抄猜测：

```bash
nmcli -f NAME,DEVICE,TYPE connection show
```

---

## 4. 第一次使用：设置串口权限

打开终端。终端是一个可以输入命令的窗口；复制下面代码块里的内容即可，不要复制
代码块外的说明文字。

先检查当前用户：

```bash
whoami
```

本机正常应显示：

```text
slam
```

把 `slam` 加入串口设备组：

```bash
sudo usermod -aG dialout slam
```

执行后必须**注销当前桌面账户并重新登录**，只关闭终端不够。重新登录后检查：

```bash
id -nG
```

输出中必须包含：

```text
dialout
```

如果只是临时测试，也可以在每次设备重新连接后执行：

```bash
sudo chmod 666 /dev/ttyUSB0 /dev/ttyUSB1
```

永久方案应优先使用 `dialout` 用户组。

### 2026-07-28 本机现状

两个设备都存在，但 `slam` 当前不在 `dialout` 组，因此直接读写权限不足。
`bringup.sh` 会尝试非交互式 `sudo chmod`，但如果系统要求输入密码，它会停止启动。
正式交接前应完成上面的永久权限设置。

---

## 5. 第一次使用或源码改变后：编译

编译不需要启动车辆，也不需要连接 GNSS，但依赖环境必须存在。

推荐命令：

```bash
cd /home/slam/robot_ws
source /opt/ros/noetic/setup.bash
BUILD_JOBS=2 ./scripts/build_workspace.sh
source .deps/setup.bash
```

说明：

- `cd` 表示进入项目目录；
- `source` 表示加载 ROS、本工作区和本地依赖环境；
- `BUILD_JOBS=2` 限制同时编译数量，减少工控机负载；
- 构建脚本会清空可能遗留的包白名单，确保不是只编译一个小包。

编译成功时，命令会正常返回终端提示符，末尾会出现类似：

```text
[100%] Built target ...
```

只要出现 `error:`、`make ... failed` 或命令返回非零，就不能认为编译成功。

### 当前已经验证的编译结果

2026-07-28 对当前磁盘上的 `pre-safety-runtime` 工作树执行了：

```bash
BUILD_JOBS=2 ./scripts/build_workspace.sh
```

结果：

- CMake 遍历全部 73 个 catkin 包；
- Livox-SDK2 已更新到 `v1.3.1`，ROS 驱动已更新到 `1.2.6`；
- 新的 `livox_ros_driver2_node` 已重新编译和链接；
- GPS、FOD、Hikrobot 相机、M2 驱动、FAST_LIO、TEB/MPC 均完成；
- `autolabor_operator_gui_node` 成功生成；
- 最终退出码为 `0`，即**当前项目编译通过**。

构建中有 Gazebo Classic 弃用、旧 VTK 可选工具缺失和包名规范警告，但它们没有
导致本次构建失败。

这次还连接新雷达验证了点云、IMU、FAST-LIO 和 `/scan`，但没有重新运行全部
自动化测试，也没有启动 CAN 或驱动真车。

注意：当前工作树有大量尚未提交的修改和未跟踪文件。这个结果对应“当前磁盘内容”，
不代表远程仓库 `main` 分支的干净版本。不要为了清理状态执行
`git reset --hard`、`git clean` 或删除不认识的文件。

---

## 6. 每次开机后的启动前检查

按以下顺序检查：

### 6.1 场地和车辆

- 车辆四周有足够净空；
- 人员、车辆和障碍物不在预计路线内；
- 物理急停工作正常，并由操作员手持；
- 第一次运行或修改代码后，先将车辆置于不会意外运动的安全状态；
- 不允许无人看守运行。

### 6.2 设备

```bash
ls -l /dev/ttyUSB0 /dev/ttyUSB1
```

两个设备都必须存在。若显示 `No such file or directory`，检查 USB 连接和供电。

检查读写权限：

```bash
test -r /dev/ttyUSB0 && test -w /dev/ttyUSB0 && echo "CAN 权限正常"
test -r /dev/ttyUSB1 && test -w /dev/ttyUSB1 && echo "GPS 权限正常"
```

两个“权限正常”都应显示。

### 6.3 雷达网络

```bash
ip -brief address show eth2
ping -I eth2 -c 2 -W 1 192.168.1.112
```

必须同时满足：

- `eth2` 是 `UP`；
- 本机地址是 `192.168.1.50/24`；
- `192.168.1.112` 有回复；
- Wi-Fi 或其他正常上网接口的默认路由没有被雷达网口替换。

查看默认路由：

```bash
ip route | grep '^default'
```

雷达专用的 `eth2` 不应出现为默认路由。

### 6.4 相机

关闭 Hikrobot MVS 桌面客户端。它可能独占相机，导致 ROS 相机节点无法打开设备。

### 6.5 GNSS 环境

- 尽量在室外开阔区域启动；
- 两根天线视野无遮挡；
- 天线、电缆和固定基线配置正确；
- 正常导航要求双天线解算达到
  `SOL_COMPUTED + NARROW_INT`。

`NARROW_FLOAT` 表示仍是浮点模糊度解，本项目默认会拒绝这种航向质量，不能通过
降低门槛来进行正常高速试验。

---

## 7. 推荐启动方法：一条命令

打开一个终端，输入：

```bash
cd /home/slam/robot_ws
./scripts/operator_all_in_one.sh 0.3 cruise
```

通常不需要手工运行 `roscore`，脚本会复用已有 ROS master，或在没有 master 时
自动启动一个。

参数解释：

- 第一个参数 `0.3`：请求的最大前进速度，单位为 m/s；
- 第二个参数 `cruise`：开阔路面配置；
- `obstacle`：密集固定障碍环境配置。

新手和首次实车验证固定使用：

```bash
./scripts/operator_all_in_one.sh 0.3 cruise
```

不要直接复制历史文档中的 `2.7 cruise` 进行第一次测试。

---

## 8. 一键启动时系统会依次做什么

### 8.1 基础环境

脚本检查 ROS、工作区和 YOLO Python 环境，创建运行日志目录，并确保有一个共享
ROS master。

日志目录类似：

```text
/home/slam/robot_ws/log/operator_all_in_one_20260728_153000/
```

里面主要有：

```text
bringup.log
vision.log
rabbitmq.log
gui.log
fallback_roscore.log
```

### 8.2 导航主链

脚本以 GPS 模式启动 `bringup.sh`，并强制：

```text
NAV_START_RVIZ=false
```

这样不会出现第二个独立 RViz；地图显示在 Qt 窗口内部。

导航启动流程为：

1. 停止上一次残留的主要导航节点；
2. 检查 CAN 串口是否存在且可写；
3. 做一次 CAN 通信预检；
4. 启动 CAN 驱动和 M2 底盘驱动；
5. 读取底盘报告的最大速度，并对规划速度再次限幅；
6. 启动 Livox 雷达并等待激光和 IMU；
7. 启动 FAST_LIO 点云配准；
8. 生成 `/scan` 激光扫描；
9. 等待底盘 `/odom`；
10. 启动双天线 GPS 定位；
11. 等待 `/gps/fix`、`/gps/heading`、`/gps/odom`；
12. 检查 `camera_init -> base_link` 坐标变换；
13. 启动 GPS 远距离滚动目标管理器；
14. 启动 GPS/FOD 安全仲裁器和待机视觉控制器；
15. 启动 move_base、TEB 和代价地图；
16. 检查 GPS 目标、服务和每一级速度话题连接是否唯一且正确。

### 8.3 Qt、视觉和 RabbitMQ

在导航初始化的同时，一键脚本还会启动：

- Hikrobot 相机；
- YOLO11 检测器；
- 路面 ROI 图像质量控制器；
- RabbitMQ GPS 目标桥；
- Qt 操作台和内嵌 RViz；
- GPS 静态误差监视器。

因此 Qt 可能很快打开，但车辆端仍在继续初始化。灰色按钮是正常的安全门控，不要
尝试绕过。

---

## 9. 如何判断启动成功

### 9.1 终端判据

必须看到：

```text
Robot bringup is running in gps mode.
```

这是唯一完整 readiness gate 成功标志。

随后通常还会显示：

```text
FOD mode command: .../scripts/fod_mode.sh start
FOD/GPS status:  .../scripts/fod_mode.sh status
```

### 9.2 Qt 判据

Qt 中至少确认：

- ROS master：在线；
- CAN：在线；
- GPS 原点：已设置；
- `/gps/odom`：持续更新；
- 双天线航向：持续更新；
- 激光：在线；
- move_base：在线；
- 控制模式：`GPS_ACTIVE`；
- GPS 目标按钮不再是灰色。

GPS 航向显示约定：

```text
0° = 北
90° = 东
180° = 南
270° = 西
角度从北向顺时针增加
```

### 9.3 GPS 按钮自动检查的条件

Qt 只有同时满足以下条件才允许发送 GPS 目标：

- ROS master 和 Qt ROS 接口已连接；
- GPS 原点有效；
- `/gps/odom` 消息不超过 2 秒；
- `/move_base/status` 消息不超过 2 秒；
- `/gps/goal_fix` 至少有一个订阅者；
- GPS/FOD 模式状态新鲜且为 `GPS_ACTIVE`。

如果按钮是灰色，先看按钮旁边的原因文字，不要直接调用 ROS 话题绕过界面。

---

## 10. 第一次低速试车

建议流程：

1. 车辆置于空旷、封闭区域；
2. 操作员手持物理急停；
3. 使用 `0.3 cruise` 启动；
4. 等待完整就绪文字；
5. 在 Qt 中确认 `GPS_ACTIVE`；
6. 如需留存数据，先点击“开始 mode1 录包”；
7. 点击“发送车头正前方 8 m GPS 目标”；
8. 观察车辆初始转向方向、速度、地图路径和周边安全；
9. 发现方向错误、异常加速、连续左右摆动或人员进入路线，立即按物理急停；
10. 试验结束后停止录包并按正常退出流程关闭系统。

“车头正前方 8 m”会根据当前 `/gps/odom` 位置和朝向换算成 WGS84 经纬度，再发布
到 `/gps/goal_fix`。

---

## 11. 在 Qt 中发送目标

### 11.1 手工输入经纬度

1. 找到综合页或 GPS 页的纬度、经度输入框；
2. 输入目标 WGS84 纬度和经度；
3. 再次核对小数点和数字；
4. 点击发送 GPS 目标；
5. 在事件日志中确认发布成功。

有效范围：

```text
纬度：-90 到 90
经度：-180 到 180
```

默认最大目标距离是 1000 m。过远目标会被拒绝，以防经纬度输入错误。

“填入当前位置”只把当前位置写入输入框，不会立即发车。

### 11.2 8 m 测试目标

点击：

```text
发送车头正前方 8 m GPS 目标
```

这是首次低速测试的推荐入口。

### 11.3 RabbitMQ 目标

RabbitMQ 消息到达后只会缓存，不会自动让车辆行驶。

操作步骤：

1. 打开 Qt 的“远程”页；
2. 确认连接状态和缓存目标；
3. 核对纬度、经度、设备和时间；
4. 点击发送缓存目标；
5. 不需要该目标时点击清空缓存。

### 11.4 内嵌 RViz 的 `2D Nav Goal`

RViz 目标是 `camera_init` 坐标系中的局部地图目标，不是 WGS84 经纬度。经纬度目标
必须使用 Qt 的 GPS 输入框或 RabbitMQ 目标。

新的 RViz/action 目标可能接管并终止当前 GPS 滚动路线。新手测试不要同时混用
RViz 局部目标和 GPS 经纬度目标。

---

## 12. 取消目标、紧急情况和恢复

### 12.1 普通取消

车辆状态正常、只是想取消当前路线时，点击：

```text
取消当前导航目标
```

Qt 会向 `/move_base/cancel` 发布空 GoalID，取消当前目标。

### 12.2 紧急情况

发生以下任一情况，立即使用物理急停：

- 车辆朝错误方向运动；
- 车辆异常加速；
- 转向快速左右摆动；
- 软件按钮无响应；
- 人员、车辆或障碍物进入安全区；
- 定位或地图明显跳变；
- 不确定车辆接下来会做什么。

安全优先顺序：

1. **按下物理急停；**
2. 确认车辆真正停止；
3. 在安全情况下点击取消目标；
4. 在启动终端按 `Ctrl+C`；
5. 保存日志和 rosbag；
6. 排查原因后才能重新启动。

不要把“关闭窗口”当作处理失控车辆的第一动作。

### 12.3 急停后的重新启动

1. 保持车辆静止；
2. 停止旧的一键启动命令；
3. 记录故障时间和现象；
4. 检查日志；
5. 确认旧节点已退出；
6. 在安全人员同意后重新运行一键命令；
7. 再次从 `0.3 m/s` 开始。

---

## 13. GPS/FOD 视觉模式说明

一键入口默认启动相机和 YOLO，但视觉行驶不会自动开始。

Qt 中的“立即单独启动”只有在以下条件成立时才可用：

- 当前模式为 `GPS_ACTIVE`；
- 相机消息不超过 1.5 秒；
- YOLO 检测消息不超过 1.5 秒；
- 当前至少有一个可见检测目标。

按钮只调用：

```text
/fod_navigation_mode/set_fod_enabled
```

安全仲裁器会先：

1. 屏蔽 GPS 速度；
2. 暂停并保留最终 GPS 路线；
3. 取消当前滚动子目标；
4. 确认车辆已经停止；
5. 最后才把控制权交给视觉模块。

视觉完成后会自动恢复 GPS。视觉 `ABORT` 或故障时会保持停车，需要操作员明确退出
视觉模式后才尝试恢复。

禁止在集成运行时直接调用：

```text
/fod_visual_servo/set_enabled
```

新手在没有现场指导时不要启动视觉行驶。

---

## 14. 正常退出

推荐方法：

1. 确认没有活动目标，车辆已经停止；
2. 如在录包，先停止录包；
3. 关闭 Qt 窗口，或者在启动终端按一次 `Ctrl+C`；
4. 等待终端完成清理并返回命令提示符；
5. 再关闭车辆电源或拔设备。

一键脚本会先停止导航主链，使安全仲裁器发布最终零速度，再停止：

- 相机和 YOLO；
- RabbitMQ；
- Qt；
- 日志跟随进程；
- 由它自己创建的 ROS master。

不要使用 `Ctrl+Z`，它只会暂停进程，可能留下仍在运行的子进程。

---

## 15. 不需要视觉或 RabbitMQ 时

不启动 RabbitMQ：

```bash
OPERATOR_START_RABBITMQ=false \
./scripts/operator_all_in_one.sh 0.3 cruise
```

不自动启动相机和 YOLO，Qt 视觉页面仍保留：

```bash
OPERATOR_START_VISION=false \
./scripts/operator_all_in_one.sh 0.3 cruise
```

相机已经由其他程序启动，只启动 YOLO 和图像质量侧：

```bash
OPERATOR_START_CAMERA=false \
./scripts/operator_all_in_one.sh 0.3 cruise
```

关闭图像质量自动控制：

```bash
OPERATOR_IMAGE_QUALITY_CONTROL=false \
./scripts/operator_all_in_one.sh 0.3 cruise
```

---

## 16. 分终端启动方法

只有排查问题或开发时才建议分终端启动。

### 终端 1：GPS 导航

```bash
cd /home/slam/robot_ws
NAV_START_RVIZ=false ./scripts/bringup.sh gps 0.3 cruise
```

### 终端 2：Qt

```bash
cd /home/slam/robot_ws
./scripts/operator_gui.sh
```

### 终端 3：相机和 YOLO，可选

```bash
cd /home/slam/robot_ws
source .deps/setup.bash
roslaunch autolabor_fod_vision hikrobot_fod_detection.launch \
  start_camera:=true \
  enable_image_quality_controller:=true \
  image_quality_exposure_max_us:=12000
```

### 终端 4：RabbitMQ，可选

```bash
cd /home/slam/robot_ws
source .deps/setup.bash
./scripts/rabbitmq_gps_goal_bridge.py
```

`.deps/setup.bash` 已经加载 ROS 和本工作区。执行它之后不要再执行
`source devel/setup.bash`，否则会暂时隐藏本地依赖目录中的
`pointcloud_to_laserscan`。

分终端运行时，停止系统需要分别终止对应命令。新手优先使用一键入口，避免遗漏进程。

---

## 17. 常见问题排查

### 17.1 提示串口不存在

典型信息：

```text
Device does not exist: /dev/ttyUSB0
```

处理：

```bash
ls -l /dev/ttyUSB*
```

检查设备供电、USB 线和编号。不要随意把一个未知串口当成 CAN 或 GPS。

### 17.2 提示串口不可写

典型信息：

```text
Device is not writable
```

处理：

```bash
id -nG
ls -l /dev/ttyUSB0 /dev/ttyUSB1
```

确认用户在 `dialout` 组，并重新登录。临时可用 `sudo chmod 666`。

### 17.3 Qt 打开，但 GPS 按钮是灰色

查看按钮附近的原因，常见情况：

- `ROS 未连接`：ROS master 或 Qt ROS 接口未连接；
- `GPS 原点未设置`：还没有收到有效定位；
- `/gps/odom 未就绪`：双天线航向质量不足或消息超时；
- `move_base 未就绪`：导航节点或代价地图未完成；
- `/gps/goal_fix 无订阅者`：滚动目标管理器未启动；
- `控制模式状态超时`：GPS/FOD 仲裁器状态丢失；
- `GPS 正在休眠`：当前处于视觉模式，不能替换 GPS 目标。

不要为了让按钮亮起而直接绕过这些条件。

### 17.4 一直没有 `/gps/odom`

重点查看 `bringup.log` 中最新的 `UNIHEADINGA` 状态。

正常要求：

```text
SOL_COMPUTED + NARROW_INT
```

如果是：

```text
NARROW_FLOAT
```

检查：

- 两根 GNSS 天线是否无遮挡；
- 天线间距和方向是否正确；
- 天线线缆是否接反或松动；
- 接收机固定基线设置；
- 是否有足够卫星和等待时间。

不要为正常行驶降低默认航向质量门槛。

### 17.5 Qt 没有打开

检查：

```bash
echo "$DISPLAY"
```

本机桌面会话通常应有类似 `:1` 的值。然后查看：

```bash
ls -dt log/operator_all_in_one_* | head -n 1
```

进入最新目录并查看：

```bash
tail -n 100 log/operator_all_in_one_*/gui.log
```

远程或无 OpenGL 环境可以暂时禁用内嵌 RViz做诊断：

```bash
./scripts/operator_gui.sh enable_rviz:=false
```

### 17.6 相机或 YOLO 离线

先关闭 Hikrobot MVS 桌面客户端，然后查看最新的：

```text
vision.log
```

检查 YOLO Python：

```bash
test -x /home/slam/robot_ws/.venv/fod_yolo/bin/python3 \
  && echo "YOLO 环境存在"
```

视觉模块故障不应自动放行视觉运动。只做 GPS 导航时，可临时使用
`OPERATOR_START_VISION=false`。

### 17.7 RabbitMQ 离线

RabbitMQ 离线不会阻止本地 GPS 和 Qt 启动。查看 `rabbitmq.log`，或临时使用：

```bash
OPERATOR_START_RABBITMQ=false \
./scripts/operator_all_in_one.sh 0.3 cruise
```

### 17.8 终端显示降级操作台

典型信息：

```text
continuing in degraded-console mode
GPS navigation is offline
```

含义是导航主链没有通过完整 readiness gate。Qt 仅用于看状态和日志，不能认为车辆
可以行驶。优先检查 `bringup.log` 中第一处失败。

### 17.9 雷达在线，但启动卡在激光或 `/scan`

先确认网络：

```bash
ip -brief address show eth2
ping -I eth2 -c 2 -W 1 192.168.1.112
```

如果没有回复，依次检查雷达供电、网线、`eth2` 是否为 `UP`、本机是否为
`192.168.1.50/24`。不要先改 ROS 参数。

如果能 `ping` 通，保持车辆断开运动控制或可靠急停，只单独测试雷达。终端 1：

```bash
cd /home/slam/robot_ws
source .deps/setup.bash
roslaunch robot_bringup livox_mid360.launch
```

正常日志必须出现这些关键信息：

```text
Livox Ros Driver2 Version: 1.2.6
dev_type:9
sn:47MDMBB0030112
successfully enable Livox Lidar imu
```

终端 2：

```bash
cd /home/slam/robot_ws
source .deps/setup.bash
rostopic hz /livox/lidar /livox/imu
```

正常频率约为：

```text
/livox/lidar  10 Hz
/livox/imu   200 Hz
```

还可以检查一帧点数：

```bash
rostopic echo -n 1 /livox/lidar/point_num
```

现场验收时读到约 `19968`。数值会随驱动分帧方式变化，但不能长期没有输出。

若日志显示配置类型 `35` 或使用 `msg_MID360s.launch`，说明误用了 Mid-360S
配置；本机这台序列号必须使用标准 `msg_MID360.launch` 和
`config/MID360_config.json`。

如果 `/livox/lidar` 和 `/livox/imu` 正常，但 `/scan` 不存在，检查本地 ROS 依赖：

```bash
source /home/slam/robot_ws/.deps/setup.bash
rospack find pointcloud_to_laserscan
```

正常应返回 `.deps/sysroot/opt/ros/noetic/share/pointcloud_to_laserscan`。不要在
`source .deps/setup.bash` 后再次 `source devel/setup.bash`。排查结束后，在两个
测试终端分别按一次 `Ctrl+C`。

### 17.10 怀疑有旧节点残留

先保持车辆安全停止，然后退出所有相关终端。检查：

```bash
rosnode list
```

不要一边运行旧的 `visual_recovery.launch`，一边启动集成 GPS/FOD 模式。
不要同时启动第二个独立 `move_base`、底盘驱动或 `/cmd_vel` 发布者。

---

## 18. 日志和录包

### 一键启动日志

```text
/home/slam/robot_ws/log/operator_all_in_one_时间/
```

发生问题时至少保存：

```text
bringup.log
vision.log
rabbitmq.log
gui.log
```

### mode1 rosbag

在 Qt 中点击“开始 mode1 录包”，测试完成后点击停止。出现异常时记录：

- 发生的准确时间；
- 当时发送的目标；
- 车辆实际运动；
- 是否按了急停；
- Qt 显示的 GPS/FOD 状态；
- 对应日志目录；
- rosbag 文件名。

不要把大型 `.bag` 文件提交到 Git 仓库。

---

## 19. 给交接人员的每日最短操作清单

### 启动

```text
[ ] 场地净空
[ ] 物理急停在手且已测试
[ ] /dev/ttyUSB0 和 /dev/ttyUSB1 存在并可读写
[ ] eth2 是 192.168.1.50/24，雷达 192.168.1.112 能 ping 通
[ ] 相机 MVS 客户端已关闭
[ ] 进入 /home/slam/robot_ws
[ ] 运行 ./scripts/operator_all_in_one.sh 0.3 cruise
[ ] 等到 Robot bringup is running in gps mode.
[ ] Qt 显示 GPS_ACTIVE，GPS/odom/move_base 正常
[ ] 开始 mode1 录包
[ ] 首先发送车头前方 8 m 测试目标
```

### 结束

```text
[ ] 取消活动目标
[ ] 确认车辆完全停止
[ ] 停止 mode1 录包
[ ] 关闭 Qt 或在启动终端按 Ctrl+C
[ ] 等待所有进程完成清理
[ ] 记录异常、日志目录和 rosbag 文件名
```

---

## 20. 交接到另一台机器前必须确认

截至 2026-07-28，下面三个关键文件仍是 Git 未跟踪状态：

```text
GPS_QT_BEGINNER_HANDOFF.md
scripts/build_workspace.sh
scripts/operator_all_in_one.sh
```

它们在当前机器人电脑上可以使用，但从远程仓库重新克隆时不会自动出现。正式把项目
交给另一台电脑或另一位开发人员前，必须由项目维护者审核并把需要的文件提交、推送
到远程仓库。

当前仓库还有大量其他未提交修改和多个有改动的子模块，不能让新手直接执行
“全部提交”，也不能用 `git reset --hard` 或 `git clean` 清理。正确顺序是：

1. 由熟悉当前开发内容的人审核 `git status` 和每一项差异；
2. 先提交并推送需要保留的子模块提交；
3. 再提交父仓库中的 gitlink 和源文件；
4. 确认一键入口和本手册已经进入远程仓库；
5. 在另一目录做一次 `git clone --recurse-submodules` 验证；
6. 在新克隆中重新编译并检查一键脚本是否存在。

克隆命令：

```bash
git clone --recurse-submodules \
  git@github.com:Doribelove/autolabor-robot-nav.git
```

已有克隆补齐子模块：

```bash
git submodule update --init --recursive
```

构建产物 `build/`、`devel/`、`install/`、`log/` 和所有 rosbag 不能提交。

---

## 21. 相关文件

| 文件 | 用途 |
|---|---|
| `scripts/operator_all_in_one.sh` | 推荐的一键 GPS/视觉/RabbitMQ/Qt 入口 |
| `scripts/operator_gui.sh` | 单独启动 Qt |
| `scripts/bringup.sh` | 导航和硬件主启动脚本 |
| `scripts/build_workspace.sh` | 当前本机完整构建包装脚本 |
| `scripts/rabbitmq_gps_goal_bridge.py` | RabbitMQ GPS 目标桥 |
| `scripts/fod_mode.sh` | GPS/FOD 模式状态和调试命令 |
| `scripts/record_rosbag.sh` | mode1 录包 |
| `src/scripts/robot_bringup/launch/livox_mid360.launch` | 本机雷达启动入口 |
| `src/perception_livox/livox_ros_driver2/config/MID360_config.json` | 雷达和工控机 IP/端口配置 |
| `src/application/autolabor_operator_gui/README.md` | Qt 功能说明 |
| `CURRENT_GPS_DEV_HANDOFF.md` | 研发细节和历史变更 |
| `README.md` | 项目总体说明 |

---

## 22. 当前仍需完成的实车验证

当前代码已通过编译和既有自动化检查记录，但“编译成功”不等于“实车安全验收完成”。
新 Mid-360 的静止感知链已经实测通过：
`/livox/lidar -> FAST_LIO -> /cloud_registered_body -> /scan` 均稳定约 `10 Hz`，
IMU 约 `200 Hz`。这不代表车辆运动已经重新验收。
仍需由熟悉车辆的人员进行有人值守的低速测试，重点确认：

- 双天线航向与车辆真实方向一致；
- 8 m 目标的初始转向正确；
- `cruise` 配置下没有异常左右摆动；
- 目标附近能够可靠停车；
- 物理急停、软件取消和 `Ctrl+C` 清理均符合预期；
- GPS/FOD 切换遵循
  `GPS_ACTIVE -> ENTERING_FOD -> FOD_ACTIVE -> ... -> GPS_ACTIVE`；
- FOD 完成后 GPS 从车辆新位置恢复原最终路线。

在这些实车项目没有完成前，不要进行无人值守或高速运行。
