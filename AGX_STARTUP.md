# Jetson AGX Orin 本机启动

本次源码位于 `/home/slam/robot_ws_agx`；来自 Doribelove 的
`Autolabor_AGX_Orin_64GB_20260927_214253`，基线分支
`gnss_0913_lianxinag`、提交 `eb2353ad`。原 `/home/slam/robot_ws`
保留其原有内容。所有 ROS 节点与 CUDA 推理均在本机运行。

## 无设备打开 Qt

在 Jetson 图形桌面终端执行：

```bash
cd /home/slam/robot_ws_agx
export ROS_MASTER_URI=http://127.0.0.1:11311
unset ROS_IP ROS_HOSTNAME
./scripts/operator_gui.sh
```

Qt 内嵌 RViz。此入口不启动底盘、GNSS、雷达或相机；未接设备时状态离线、
缺少 `camera_init` 坐标系是预期结果。关闭窗口或 Ctrl+C 结束。

## 室内查看雷达与导航（无需 GNSS 信号）

2026-09-28 已按用户要求增加独立室内模式。先关闭上一套操作台，在桌面终端执行：

```bash
cd /home/slam/robot_ws_agx
export ROS_MASTER_URI=http://127.0.0.1:11311
unset ROS_IP ROS_HOSTNAME
CAN_PORT=/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B400CG51-if00-port0 \
OPERATOR_NAVIGATION_MODE=fast_lio_indoor OPERATOR_CAMERA_DRIVER=zed \
  ./scripts/operator_all_in_one.sh 0.3
```

该模式启动 CAN、MID360、FAST-LIO 定位、扫描投影、move_base/TEB、ZED/YOLO 和 Qt。
定位来自 `/Odometry` 及 FAST-LIO TF；不启动 GNSS 读取和经纬度目标转换，
不会因室内没有卫星信号而等待 `/gps/fix` 或 `/gps/odom`。Qt 导航页显示实时点云、
激光扫描、雷达定位轨迹和避障地图；GPS 数值仍显示无数据。已接入原有导航/视觉
模式管理器和待命视觉回收控制器，室内切换不依赖 GNSS 路线服务。
已有 GPS 按钮不用于室内模式。默认规划速度上限按上述命令为 0.3 m/s。

成功标志为 `Robot bringup is running in fast_lio_indoor mode.`。
关闭 Qt 会停止本次启动的整套进程。未设置 `OPERATOR_NAVIGATION_MODE` 时仍走
原双天线 `gps` 流程；原有 `fast_lio`、`fast_lio_gps` 入口也保持原行为。

室内视觉回收：在“视觉”页观察地面目标，点击“启动相机控制回收”并按原提示
确认。切入时自动启动清扫装置；控制器稳定锁定最近目标后以标称 0.6 m/s 靠近，
目标进入中下回收区再消失时，按里程计连续前进 0.80 m、停车。按钮在相机/检测/模式服务尚未就绪或没有检测框时
不可用。当前仍保留运动置信度 0.30 和单目标锁定逻辑，低分预览框不代表可回收目标。
“取消相机回收 / 恢复导航”终止视觉任务。回收自动开启的清扫器会自动关闭；
若之前已手动开启，则仍由“清扫”页单独停止。
室内切换会取消当前局部导航目标，完成后回到导航待命，需要重新发送局部目标；
室外 GNSS 模式保留原路线，完成回收后停车，点击“取消相机回收 / 恢复导航”后
再恢复路线。只启动整套程序不会自动使能行驶或启动清扫器。

ZED 检测范围已按清扫装置遮挡设置为顶部忽略 2/8、底部忽略 3/8：
1280×720 图像只将第 180～449 行送入 YOLO。完整预览中忽略区压暗、有效区描边；
检测框、锚点、相机内参保持原图坐标。配置在 `config/fod_zed_ground_roi.yaml`，
检测和视觉回收共同读取，回收区默认是有效识别带下半部分、中央 60% 宽度。
预览黄框显示区域，最近目标进入后变绿；随后丢失即执行一次 0.80 m 回收，不因框
闪回、其他 FOD 或已进入直行阶段后的相机断流而重置距离。2026-09-29 按用户要求
由 0.50 m 改为 0.80 m；已通过 27 场景 × 3 轮控制链仿真，尚未完成实车回收验收。

同日后续按用户要求删除了识别 `0.35 s` 断流停车、控制更新 `0.25 s` 到期停车，
以及穿越时 `4°` 航向和 `8 cm` 横偏中止条件。断流期间沿用最后检测/控制指令，
航向和横移只记录到状态；下一次正常启动加载。修改和验证详见
[FOD_CONTROL_LIMITS_REMOVAL_REPORT.md](FOD_CONTROL_LIMITS_REMOVAL_REPORT.md)。

本次实测日志：`log/operator_indoor_20260928_160122/`。雷达定位、点云和扫描约
10 Hz，ZED 彩色/深度及 YOLO 约 15 Hz，move_base 的全局/局部避障地图均有实际数据。
本次仅启动与查看，未下发目标或测试车辆运动。

## 室外双天线 GNSS + ZED 2 完整启动

2026-09-28 已实测本机 ZED 2（序列号 `23748636`），使用以下命令：

```bash
cd /home/slam/robot_ws_agx
export ROS_MASTER_URI=http://127.0.0.1:11311
unset ROS_IP ROS_HOSTNAME
CAN_PORT=/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B400CG51-if00-port0 \
GPS_PORT=/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTENXGO4-if00-port0 \
GPS_BAUD_RATE=115200 GPS_HEADING_SOURCE=dual_antenna \
OPERATOR_CAMERA_DRIVER=zed ./scripts/operator_all_in_one.sh 1.6 cruise
```

2026-09-29 按用户“效率优先”的新要求，完整入口和 `bringup.sh gps` 的默认
前进、倒车规划上限均改为 **1.6 m/s**；省略速度参数同样生效。`cruise` 纵向
规划加速度为 **3.0 m/s²**，转向角加速度为 **1.2 rad/s²**；`obstacle` 分别为
2.0 m/s² 和 1.0 rad/s²。两个场景均允许倒车初始化，将前进偏好权重降至 5，
转向加速度惩罚权重降至 60，以加快起步、转弯起步和前后换向。
实际速度仍不超过底盘上报的上限；显式较低的倒车限速仍会保留。
车体和障碍物碰撞检查继续使用 1.6×0.8 m 轮廓，最小规划转弯半径仍为 1.35 m。
如果显式传入不超过 0.5 m/s 的 GPS 速度，仍加载原 `navigation_low_speed.yaml`，
其中纵向和转向加速度均为 0.8，便于继续使用原低速配置。

完整入口启用既有终点减速器：距最终目标约 5.32 m 开始降低 1.6 m/s 指令，
2 m 时上限约 0.6 m/s，1.1 m 时约 0.3 m/s，0.8 m 时约 0.2 m/s，
近处最低接近上限 0.20 m/s；
更低的规划指令不会被抬高，0.2 m 到达停车和原取消逻辑保留。
距离指 `base_link` 到最终目标，减速时同步缩放角速度以保持转弯曲率。
长路线中间目标不触发终点减速；视觉回收按 2026-09-29 的要求使用独立的
0.6 m/s 标称速度，转向及穿越末端仍按控制器实际需要降速。
`GPS_GOAL_SPEED_CAP_ENABLED`、`GPS_GOAL_APPROACH_TIME`（默认 3.0 s）和
`GPS_GOAL_COMFORTABLE_DECEL`（默认 0.25 m/s²）、`GPS_GOAL_MIN_APPROACH_SPEED`
（默认 0.20 m/s，沿用底盘起步实测值）可显式覆盖。
以上是软件控制参数，实际停车距离和起步响应仍需在现场使用中复核。

2026-09-29 增加三项连续性修正，使用同一条启动命令即可加载：

- 双天线固定解短暂中断时，以最后有效航向为基准，积分底盘 `/odom` 实测角速度，
  继续更新 GNSS 位姿与 TF。`GPS_HEADING_CONTINUATION_TIMEOUT=3.0` 表示从最后
  一帧有效固定航向算起最多 3 秒，不是额外增加 3 秒；设为 0 可关闭续接。
  底盘反馈必须连续且新鲜（默认 0.5 秒）。启动仍需要有效双天线航向，FLOAT
  不会被当作固定解，也不会刷新固定航向时间戳。长期失去航向仍按原失效流程停车。
- TEB 修正了恢复时混用全局、局部路径索引的问题。轨迹不可行后，局部规划范围
  正确缩为一半，连续失败超过 9 次缩为四分之一，同时更新范围内的路径约束点。
  已包含最终目标的路径不缩短；1.6×0.8 m 车体与原碰撞余量继续参与检查。
- `GPS_GOAL_NEAR_TIMEOUT=15.0` 现在表示近终点连续 15 秒没有实际机动进展，
  不再限制整个接近过程必须在 15 秒内结束。实测位移累计达到 5 cm，或实测转向
  达到约 2.9°，且里程计速度表明正在运动时，重新计时；倒车、绕弯均可计入。
  `GPS_GOAL_NEAR_MAX_REGRESSION=0.5` 现仅作为距离回退日志阈值，不再单独锁停。
  定位失效期间仍输出零速，恢复定位后重新给予机动时间；到达、取消与陈旧命令
  处理继续生效。此超时检测停滞，运动中的反复绕行仍由原规划器恢复逻辑处理。

上述修正已完成 ARM 构建、140 项相关单元/配置测试和隔离 ROS 消息链验证，
未做本次实车行驶验证。详细记录见 [AGX_MIGRATION_REPORT.md](AGX_MIGRATION_REPORT.md)。

2026-09-29 相机回收与 GPS 目标联动：

1. 正常发送 GPS 经纬度目标；不开启相机回收则全程按原 GPS 导航。
2. 点击“启动相机控制回收”。界面显示等待状态、到最终目标的距离和有效 FOD 状态。
3. **距最终目标小于 5 米就启动清扫装置，与是否检测到 FOD 无关**。没有有效
   FOD 时，继续按 GPS 目标导航并清扫；最新实际检测帧有有效 FOD 才
   暂停 GPS、切入 0.6 m/s 视觉伺服。等于或超过 5 米时继续 GPS，暂不自动开启清扫。
4. 切入后只锁定最近一个目标。目标进入中下回收区再消失时，连续计量前进
   0.80 米并停车；途中框闪回不重置距离，末端按剩余距离减速。
5. 视觉完成后停车并保留 GPS 路线，点击“取消相机回收 / 恢复导航”后再恢复，
   每次点击执行一次回收。若始终未识别到
   FOD，GPS 到达目标时结束本次回收并关闭自动启动的清扫装置。等待期间取消
   会撤销请求并关闭本次自动启动的清扫装置，GPS 导航继续；视觉期间退出则
   按原流程停车并恢复 GPS。

有效 FOD 指最新实际帧中原六类、置信度 ≥0.30 且有有效深度的检测，使用既有地面 ROI。
检测消息暂停不再按经过时间清除已有目标状态，后续实际空帧仍清除；初次视觉
使能仍等待新帧与稳定锁定。低分预览框
不会触发视觉接管，但不影响小于 5 米时启动清扫。仅本次回收自动开启的清扫装置
会在到达/完成/退出/中止时关闭；手动开启的保持运行。清扫通信失败会显示原因，
保持 GPS 导航，取消后重新开启可重试。

该入口启动当前工作区的 GNSS 导航、Qt、GPU YOLO 和 RabbitMQ，同时调用当前
工作区的 `scripts/zed2_camera.sh`。相机进程单独加载
`/home/slam/robot_ws/.deps/setup.bash`，复用本机已有的 ZED ARM 驱动及 SDK 4.2.5；
导航、Qt、FOD 模型和自定义消息仍来自 `robot_ws_agx`。
因此需要保留旧工作区的 ZED 源码、`devel`、`.deps` 以及 `/usr/local/zed`。
`OPERATOR_ZED_LAUNCHER` 可指定其他已安装的兼容启动脚本；
`OPERATOR_ZED_SERIAL_NUMBER` 默认 `0`，自动选择连接的相机，也可指定实际序列号。

ZED 彩色图和深度分别发布到 `/fod_camera/image_raw`、
`/fod_camera/depth_registered`，YOLO 结果与画面为 `/fod/detections`、
`/fod/debug/image`。初次启动为 640×360；9 月 28 日识别优化后使用当前工作区
`zed2_camera.launch` 覆盖相机降采样配置，输出 1280×720，实测约 13.6 Hz，
GPU 推理平均 44 ms（含录制观测时负载）。
ZED 的定位 TF 关闭，保留原 GNSS 定位职责。当前 FOD 检测仍使用彩色图，
没有移植旧工作区的深度融合消息；当前工作区原有像素视觉回收控制器已适配
ZED 的 1280×720、光学坐标系和实际节点名，深度不参与该控制器的距离判断。

ZED 使用自身自动曝光；该模式不启动海康驱动或海康图像质量控制器。
2026-09-29 已在 Qt“视觉”页接入“ZED 原生参数”：读取/应用联合自动曝光与
增益、手动 0～100 档位、白平衡、锐度、伽马及深度过滤参数。完整入口自动选择
对应相机页签；手动修改在本次相机会话生效，重启后加载启动配置。
关闭 Qt 或 Ctrl+C 会连同本次启动的 ZED 一起结束。

当前 ZED 配置使用 1280×720、采集 60 FPS、输出上限 30 FPS，关闭当前导航
不用的 ZED 后台定位计算，保留按需 ULTRA 注册深度和原生标定。ZED + 机场
识别配置默认使用已在本 Orin 生成的 TensorRT FP16 缓存；录像中单帧中位耗时
从 28.99 ms 降为 14.21 ms。当前相机未接入，新的采集帧率尚未实机测量。
完整参数、Qt 用法、重建引擎与切回原后端的命令见 [ZED_PERFORMANCE.md](ZED_PERFORMANCE.md)。

只有相机接入时，原导航入口会报告缺少 CAN/GNSS 等设备，Qt 保持诊断状态。
连接好底盘、双天线 GNSS 和雷达后重新运行；串口设置仍使用下节的
`CAN_PORT`、`GPS_PORT`、`GPS_BAUD_RATE`，相机不占用 `/dev/ttyUSB*`。
默认不设置 `OPERATOR_CAMERA_DRIVER` 时仍启动原海康相机流程。

当前 `OPERATOR_FOD_PROFILE` 默认 `airport_recall`，提高出框召回并稳定预览。
用户要求地面小石头不作为清扫对象；此排除能力仍缺原六类训练集和现场石头负样本，
**当前调试模型未完成石头排除验收**。具体参数、实测和数据缺口见
[识别优化记录](FOD_RECOGNITION_TUNING.md)。

如需对比原检测参数（相机仍保持高清输出）：

```bash
OPERATOR_CAMERA_DRIVER=zed OPERATOR_FOD_PROFILE=original \
  ./scripts/operator_all_in_one.sh 1.6 cruise
```

## 接入原硬件后启动完整系统

先关闭独立 Qt，然后在同一桌面终端执行：

```bash
cd /home/slam/robot_ws_agx
export ROS_MASTER_URI=http://127.0.0.1:11311
unset ROS_IP ROS_HOSTNAME
CAN_PORT=/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B400CG51-if00-port0 \
GPS_PORT=/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTENXGO4-if00-port0 \
GPS_BAUD_RATE=115200 \
  ./scripts/operator_all_in_one.sh 1.6 cruise
```

2026-09-28 下午已根据串口实际数据确认：`B400CG51` 是底盘 CAN
（本次 `/dev/ttyUSB1`，可收到 `/canbus_msg`），`FTENXGO4` 是 GNSS
（本次 `/dev/ttyUSB0`，115200 波特率下收到 GGA）。这次编号与入口的通用默认值
相反，使用上述固定设备标识覆盖即可，不改变原入口默认行为。更换转接器后应重新确认。

本次实际雷达为 MID360 `47MDMBB0030112`，地址 `192.168.1.112`；已把当前
`MID360_config.json` 的雷达地址从迁移包中的 `.125` 改为实机 `.112`。
主机接收地址仍为 `192.168.1.50/24`。本次链路是 `eth0`
（MAC `50:54:7b:e3:c9:10`），沿用已有 NetworkManager 连接 `mid360-eth0`，
无网关，不抢占默认路由。

另一个已保存连接 `autolabor-agx-mid360` 绑定板载网卡 MAC
`4c:bb:47:4c:29:6a`，该网卡本次叫 `eth2`，未连接网线；已移除该连接中过时的
`eth0` 名称限制，保留 MAC 匹配。不要把网口名称当作永久硬件身份，也不要在两个
网口同时启用相同的 `192.168.1.50`。海康相机序列号仍为 `DA7535899`；
当前使用 ZED 时请采用上节带 `OPERATOR_CAMERA_DRIVER=zed` 的完整命令。

只开 GNSS 导航：

```bash
./scripts/bringup.sh gps 1.6 cruise
```

GNSS 双天线固定解要求、天线偏移、坐标系与速度路由保留；航向续接和
TEB 恢复行为采用上节 2026-09-29 的修正。
GNSS 模式仍使用 FAST_LIO 输出避障点云，关闭其定位里程计和定位 TF。
本次日志中的 `Robot bringup is running in gps mode.` 才是导航入口完成标志。
RabbitMQ 仍使用原服务配置，收到目标后保持原来的人工确认方式。

2026-09-28 当前 GNSS 确认为 UM982。串口最初仅输出 GGA，本次已请求
`UNIHEADINGA COM1 0.1` 并收到 OK，随后 GGA 与航向均约 10 Hz；没有保存接收机
配置到闪存，也没有更改接收机定位模式。用户确认设备处于室内，主副天线均未获得
卫星定位、航向为 `INSUFFICIENT_OBS/NONE`，因此本次室内运行使用上节雷达模式。
原双天线入口仍需真实有效 GNSS 数据；窗口出现不代表 GNSS 导航已就绪。

## 清扫装置快速验证

清扫控制器实测为 `192.168.0.197:50003`。本机独立网口配置已保存为
`autolabor-agx-sweeper`，绑定 MAC `6C:1F:F7:C4:82:83`（本次 `eth1`），
地址 `192.168.0.50/24`，自动连接、不设默认网关。

```bash
cd /home/slam/robot_ws_agx
./scripts/sweep_test.py
```

输入 `1` 启动、`0` 停止、`s` 查询、`q` 停止并退出，均需回车。
也可单次执行 `./scripts/sweep_test.py status`、`on` 或 `off`。
已实际读到“已关闭”，未代替操作者执行启停。更多说明见
[清扫快速验证](src/SweepDeviceControl/QUICK_TEST.md)。
完整 Qt 的“清扫”页已提供同样的启动、停止和状态回读按钮。手动开启的清扫器
不会随 Qt 退出自动关闭，结束任务前请点击停止；相机回收自动开启的清扫器由回收流程关闭。

## GPU 环境与离线检查

Python：`/home/slam/python_env/fod_yolo/bin/python3`，保留 system-site-packages。
NVIDIA torch `2.0.0+nv23.05`、CUDA torchvision `0.15.1`、ultralytics `8.3.0`。
推理节点禁用用户级 Python 包目录，避免被 `~/.local` 的 CPU torch 干扰。

```bash
cd /home/slam/robot_ws_agx
./scripts/fod_yolo_smoke.sh \
  --source src/yolo/fod_yolo11n_img640_e300_orig/train_batch0.jpg --device 0
```

默认加载原 FOD `best.pt`。此检查只验证离线图像与 GPU 推理，不代表相机或实车验收。
环境安装/修复入口为 `scripts/setup_fod_yolo_env.sh`，会保留已有环境并约束 CUDA
torch 版本。`FOD_YOLO_ENV`、`OPERATOR_DETECTOR_PYTHON` 等原有覆盖方式保留。

实际构建、依赖安装位置、日志及未完成项见 [迁移报告](AGX_MIGRATION_REPORT.md)。

当前车体外轮廓已改为 **1.60 × 0.80 m**，以 MID360 正下方的 `base_link` 为中心。
Qt 导航视图的黄色矩形表示实际车体，绿色箭头朝前；导航、TEB 和车身点云过滤
共同读取一份轮廓配置。尺寸、原有 padding 及验证范围见
[车体尺寸记录](VEHICLE_GEOMETRY.md)。

## 本机重新编译

当前缺失系统依赖安装于用户目录，正常启动入口会由 `devel/setup.bash` 自动加载。
`/home/slam/agx_migration_20260927/sysroot` 包含实际运行库，请保留该目录。
修改源码后的构建命令：

```bash
cd /home/slam/robot_ws_agx
source /home/slam/agx_migration_20260927/staged_build_env.bash
catkin_make -j6 -l8 -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DCATKIN_WHITELIST_PACKAGES='' -DCATKIN_BLACKLIST_PACKAGES='' \
  -DMVS_ROOT=/home/slam/agx_migration_20260927/sysroot/opt/MVS \
  -DLIVOX_LIDAR_SDK_LIBRARY=/home/slam/agx_migration_20260927/sysroot/usr/local/lib/liblivox_lidar_sdk_static.a
```

系统目录安装仍需本机 sudo 认证，具体脚本：
`sudo bash /home/slam/agx_migration_20260927/install_system.sh`。
该步骤完成前，不应将用户目录依赖状态称为 apt/rosdep 系统级安装完成。
