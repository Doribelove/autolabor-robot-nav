# `slam` 工作空间部署与验证记录

更新时间：2026-07-27

工作空间：`/home/slam/robot_ws`

代码基线：`pre-safety-runtime`，`9dde205`

## 1. 当前结论

当前完整版本已经在本机 `slam` 用户下完成环境适配、Release 全量构建、自动
测试和无硬件运行冒烟测试。

- 共发现 74 个 ROS 包，其中 1 个带 `CATKIN_IGNORE`，实际构建 73 个包。
- `./scripts/build_workspace.sh` 全量构建退出码为 0。
- `catkin_test_results --all build/test_results`：
  `298 tests, 0 errors, 0 failures, 0 skipped`。
- ROS 生产视觉链已使用正式 `best.pt`、CUDA 和样图成功运行并发布检测消息。
- Qt 操作台已在 `slam` 的 X11 会话中实际启动，内嵌 RViz，窗口和 ROS 节点
  均保持在线。
- GPS 一体化脚本、8 组 launch 配置、GPS/FOD 安全仲裁和无设备失效保护均已
  验证。
- 海康驱动可以加载本地 MVS SDK；当前没有相机时会保持在线并周期重试。
- RabbitMQ 桥接已兼容本机的 `pika 0.11` 和新版 `pika`，Broker 离线时持续
  重连且 ROS 状态/操作服务仍可用。

本轮没有进行真实车辆运动测试。验证时机器上没有 `/dev/ttyUSB0`、
`/dev/ttyUSB1`，也没有检测到海康相机，因此 CAN、GNSS 和相机的最终验收需要
硬件接入后由现场操作员完成。

## 2. 项目作用与核心控制链

这是一个 ROS Noetic/catkin 无人车工作空间，主要包含：

- Autolabor CAN/M2 底盘驱动；
- Livox MID360 和 FAST_LIO 点云配准；
- 双天线 GNSS 定位、航向质量门禁和经纬度目标转换；
- 15 m 滚动 GPS 子目标、`move_base`、TEB 和局部代价地图；
- 海康工业相机、YOLO11 FOD 检测、视觉回收伺服；
- GPS 与视觉回收的独占速度仲裁；
- Qt5 操作与诊断台，内嵌 RViz；
- RabbitMQ GPS 目标接收桥。

GPS 模式的主要数据流为：

```text
CAN/M2 ───────────────> /odom、底盘状态
Livox -> FAST_LIO ────> 点云配准 -> /scan -> move_base costmap
双天线 GPS ───────────> /gps/fix、/gps/heading、/gps/odom
/gps/goal_fix
  -> 15 m 滚动目标管理器
  -> /move_base/goal
  -> move_base + TEB
  -> /cmd_vel_navigation
  -> GPS 安全继电器
  -> /cmd_vel_gps
```

启用默认 FOD 待机集成后，唯一底盘命令链为：

```text
GPS/TEB -> /cmd_vel_gps ┐
                        ├-> /fod_navigation_mode -> /cmd_vel -> m2_driver
视觉伺服 -> /cmd_vel_fod ┘
```

`/fod_navigation_mode` 是 `/cmd_vel` 的唯一发布者。相机和 YOLO 可以常驻，
但视觉控制器默认待机，不会与 GPS 同时控制底盘。

## 3. 本机环境

已验证的平台和关键版本：

| 项目 | 版本 |
| --- | --- |
| 设备 | NVIDIA Jetson AGX Orin 64 GB |
| 系统 | Ubuntu 20.04，aarch64 |
| 内核 | `5.10.192-tegra` |
| JetPack / L4T | 5.1.3 / 35.5.0 |
| ROS | Noetic |
| GCC / CMake | 9.4.0 / 3.16.3 |
| CUDA | 11.4 |
| PyTorch | `2.0.0+nv23.05`，CUDA 可用 |
| torchvision | 0.15.1 |
| Ultralytics | 8.3.0 |
| OpenCV / NumPy / SciPy | 4.10.0 / 1.24.4 / 1.10.1 |

项目自带的可迁移环境：

- `.deps/sysroot`：用户态 ROS/系统依赖；
- `.deps/livox-sdk2`：Livox SDK2；
- `.deps/mvs`：海康 MVS ARM64 SDK；
- `.venv/fod_yolo`：Jetson CUDA YOLO Python 环境；
- `.rosdep`：本工作空间使用的 rosdep 数据。

所有终端优先这样加载环境：

```bash
cd /home/slam/robot_ws
source .deps/setup.bash
```

该脚本会加载 ROS、`devel` 和项目私有依赖。不要把 MVS SDK 的库目录全局
加入 `LD_LIBRARY_PATH`：MVS 附带的旧 `libusb` 会影响 FAST_LIO/PCL。相机
可执行文件已经写入独立 RUNPATH，`bringup.sh` 也会为 FAST_LIO 优先选择系统
`libusb`。

## 4. 从当前源码重新构建

YOLO 环境检查/补齐：

```bash
cd /home/slam/robot_ws
./scripts/setup_fod_yolo_env.sh
```

全量 Release 构建：

```bash
cd /home/slam/robot_ws
./scripts/build_workspace.sh
```

默认使用 6 个并行任务，可按内存情况调整：

```bash
BUILD_JOBS=4 ./scripts/build_workspace.sh
```

全量测试和结果汇总：

```bash
cd /home/slam/robot_ws
source .deps/setup.bash
catkin_make run_tests -j6 -l6
catkin_test_results --all build/test_results
```

预期最终摘要：

```text
Summary: 298 tests, 0 errors, 0 failures, 0 skipped
```

CMake 可能提示旧版 VTK 的可选命令或 PCL 的可选 pcap/png 功能缺失，也会提示
历史包名 `gazebo_ros_2Dmap_plugin` 不符合小写命名约定；这些提示没有造成目标
缺失或链接失败。

## 5. 硬件接入前必须完成

当前 `slam` 不在 `dialout` 组。需要在本机终端执行一次：

```bash
sudo usermod -aG dialout slam
```

然后注销 `slam` 并重新登录，再确认：

```bash
id -nG
```

输出应包含 `dialout`。这是唯一尚需管理员权限完成的环境动作；自动安装过程中
没有获得 `slam` 的 sudo 密码，因此没有绕过权限修改。

连接硬件后先确认稳定设备名：

```bash
ls -l /dev/serial/by-id/
ls -l /dev/ttyUSB*
lsusb
```

默认约定是：

- CAN/M2：`/dev/ttyUSB0`
- 双天线 GPS：`/dev/ttyUSB1`
- GPS 波特率：115200
- 海康相机序列号：`DA7535899`

若 USB 编号与默认值不同，用 `CAN_PORT`、`GPS_PORT` 覆盖；不要在未确认映射时
直接启动。

## 6. GPS 模式启动

### 6.1 终端模式

开阔巡航：

```bash
cd /home/slam/robot_ws
./scripts/bringup.sh gps 2.0 cruise
```

静态障碍较密集的场景：

```bash
./scripts/bringup.sh gps 1.0 obstacle
```

未给速度时默认为 1.5 m/s，未给配置时默认为 `cruise`。启动脚本会依次：

1. 校验参数、设备权限和 CAN；
2. 启动 M2 底盘并读取底盘最大速度，规划速度不会超过底盘限制；
3. 启动 Livox、FAST_LIO 点云配准和 `/scan`；
4. 启动 GPS，严格等待 `SOL_COMPUTED + NARROW_INT` 双天线航向；
5. 启动 15 m 滚动目标管理器；
6. 启动视觉控制器待机和 GPS/FOD 速度仲裁器；
7. 启动 `move_base + TEB`；
8. 检查目标、服务和所有 `/cmd_vel*` 路由；
9. 打印 `Robot bringup is running in gps mode.`。

只有出现最后一行后，导航链才算完整就绪。

手机罗盘方向可转换成初始 ROS yaw，例如：

```bash
GPS_COMPASS_HEADING="东北45度" ./scripts/bringup.sh --print-gps-yaw
GPS_COMPASS_HEADING="东北45度" ./scripts/bringup.sh gps 2.0 cruise
```

“东北 45 度”转换结果为约 `0.7853981634 rad`。双天线高质量航向建立后会取代
初始值。

### 6.2 RabbitMQ 目标桥

另开终端：

```bash
cd /home/slam/robot_ws
source .deps/setup.bash
./scripts/rabbitmq_gps_goal_bridge.py
```

桥接器只缓存新目标；必须在终端输入 `1` 或在 Qt 中点击确认后，才会发布
`/gps/goal_fix`。Broker 离线不会让 Qt 或 ROS 接口退出。

## 7. Qt 启动方式

推荐的一体化入口：

```bash
cd /home/slam/robot_ws
./scripts/operator_all_in_one.sh 2.0 cruise
```

它会启动：

- GPS 导航，关闭独立 RViz；
- 海康相机、图像质量控制和生产 YOLO；
- RabbitMQ GPS 目标桥；
- 内嵌 RViz 的 Qt 操作台。

导航或某个可选侧车启动失败时，Qt 会保留在降级诊断模式，运动按钮由实时
就绪门禁禁用。关闭 Qt 窗口会清理该脚本启动的进程。

分开启动时：

```bash
# 终端 1
cd /home/slam/robot_ws
NAV_START_RVIZ=false ./scripts/bringup.sh gps 2.0 cruise

# 终端 2
cd /home/slam/robot_ws
./scripts/operator_gui.sh
```

Qt 本身不发布 `/cmd_vel`，它发布 GPS/地图目标、取消 GoalID、诊断复位请求，
并调用经过安全门禁的模式服务。

## 8. 相机和 YOLO

启动前关闭会独占相机的 MVS 桌面客户端。

```bash
cd /home/slam/robot_ws
source .deps/setup.bash

roslaunch autolabor_fod_vision hikrobot_fod_detection.launch \
  start_camera:=true \
  enable_image_quality_controller:=true \
  image_quality_exposure_max_us:=12000
```

查看结果：

```bash
rqt_image_view /fod/debug/image
rostopic hz /fod_camera/image_raw
rostopic hz /fod/detections
rosservice call /fod_camera/driver/get_imaging_controls
```

生产权重：

```text
src/yolo/fod_yolo11n_img640_e300_orig/weights/best.pt
```

SHA256：

```text
7bf99d4c61343e8cdb37289f2eece6cf18342b508f9b7f80723592edce398500
```

模型类别为 `Metal, Soft, Plastic, Wire, Tool, w`。launch 会校验散列和完整类别，
错误权重不会静默进入生产模式。

## 9. GPS 与 FOD 回收切换

GPS bringup 已自动启动视觉控制器为待机状态，不要再并行单独启动
`visual_recovery.launch`。

进入 FOD 回收：

```bash
cd /home/slam/robot_ws
./scripts/fod_mode.sh start
```

查看状态：

```bash
./scripts/fod_mode.sh status
./scripts/fod_mode.sh watch
```

主动退出并恢复保留的 GPS 路线：

```bash
./scripts/fod_mode.sh stop
```

`start` 的安全切换顺序是：

1. 先屏蔽 GPS 速度输出；
2. 暂停长距离管理器并保留最终经纬度；
3. 取消当前滚动子目标；
4. 连续确认车辆已经停车；
5. 放行视觉伺服；
6. 视觉到达 `COMPLETE` 后，从新位置生成 GPS 子目标并自动恢复。

视觉 `ABORT`、里程计/底盘反馈过期、CAN 安全故障或命令发布冲突都会保持
零速，且不会自动恢复 GPS。排障后由操作员明确执行 `fod_mode.sh stop`。

首次受控实车回收建议缩短盲走距离：

```bash
FOD_RECOVERY_BLIND_DISTANCE_M=0.20 \
  ./scripts/operator_all_in_one.sh 0.3 cruise
```

只有在封闭净空区域、FOD 柔软无害、检测稳定且操作员手持物理急停时才允许
运动验证。

## 10. 本轮验证证据

| 验证项 | 结果 |
| --- | --- |
| 归档完整性 | 12,358 个条目，Git/submodule/fsck 正常 |
| rosdep | 所有系统依赖满足 |
| 全量构建 | 73 包，Release，退出码 0 |
| 自动测试 | 298 项，0 错误、0 失败、0 跳过 |
| 关键动态库 | Qt、MVS、M2、FAST_LIO、Livox 均无 `not found` |
| 生产 YOLO 单次脚本 | CUDA 成功，正式权重成功推理 |
| ROS 视觉链 | 1280×720 样图，16 个检测，调试图像和诊断 OK |
| 模型门禁 | SHA256 和六个生产类别校验通过 |
| Qt/RViz | X11 真实窗口 1680×1000，节点持续在线 |
| GPS launch | 8 组完整 launch 参数展开通过 |
| GPS 无设备预检 | 缺 `/dev/ttyUSB0` 时退出码 3，未启动运动节点 |
| Qt 一体化降级 | GPS 预检失败后 Qt 保持在线，运动门禁禁用 |
| 海康驱动 | MVS SDK 成功加载；无相机时在线重试，服务返回未连接 |
| RabbitMQ | `pika 0.11` 离线重连、状态话题和服务通过 |

GPS/FOD 重点测试覆盖：

- 双天线航向质量、过期和跳变策略；
- 正反向速度、天线偏置和静止噪声；
- 15 m 滚动目标、暂停、取消、回收后重新规划；
- 近终点零速锁存和 stale odom 失效停车；
- GPS/FOD 完成自动恢复、视觉 ABORT 保持停车；
- 视觉闭环完成、偏心目标转向、里程计/车轮反馈过期；
- CAN 急停、原始 CAN 故障和转向中心偏差锁存；
- Qt 只经安全仲裁进入视觉运动模式。

## 11. 现场最终验收边界

软件、构建、模型、ROS 图、Qt 和无硬件失效路径已经验证。接入真车后仍需
现场确认：

1. `/dev/serial/by-id` 对应 CAN 和 GPS 的实际映射；
2. CAN/M2 状态、物理急停和最大速度服务；
3. Livox 网络、点云、IMU、TF 和 `/scan` 频率；
4. GPS `UNIHEADINGA` 能稳定达到 `SOL_COMPUTED + NARROW_INT`；
5. 相机序列号、20 Hz 图像、曝光/增益和外参方向；
6. 先零速和架空轮测试，再进行低速封闭场地测试；
7. 最后才验证 GPS→FOD→GPS 的真实运动切换。

旧错误版本已可恢复地保存在：

```text
/home/slam/robot_ws_wrong_version_backup_20260727
```

最新完整归档仍保留在工作空间根目录，未删除。
