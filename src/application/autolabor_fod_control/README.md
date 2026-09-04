# Autolabor FOD Visual Recovery Control

本包在现有 `/fod/detections` 基础上提供一套真车视觉伺服回环。它使用 ZED
注册深度在多个候选中选择最近 FOD。GPS 联动入口只在最近 FOD 严格小于 `5 m`
时切换控制；目标在 `5 m` 外或连续 `1 s` 没有有效深度识别时继续原 GPS 导航。
进入视觉控制后使用像素位置闭环转向；由于当前还没有
经过验证的相机安装外参，不会把轴向深度或 bbox 大小伪造成地面坐标。目标在
闭环转向校正下移动到画面下方并按连续帧确认消失后，程序立即以该
时刻的底盘 `/odom` 位姿开始计量，先停车等待前轮回中，再继续直行，直到自
首次消失位置起的净前进距离达到 `0.50 m`。

状态机为：

```text
DISABLED → PRECHECK → ACQUIRE → APPROACH → EDGE_ARMED
                                      ↓（过早丢失）
                                  REACQUIRE → ABORT

EDGE_ARMED → LOSS_CONFIRM → STEER_SETTLE → BLIND_ADVANCE → FINAL_STOP → COMPLETE
任何控制冲突、数据断流、急停或里程异常 ─────────────────────────────────→ ABORT
```

特别注意：检测话题断流不会被当成 FOD 进入盲区。只有目标已经在画面底部中央
稳定出现，随后 `/fod/detections` 仍按频率更新但连续给出空候选，才允许进入
盲走。

## 双机版本更换模型

双机运行只维护一份活动模型契约：NVIDIA 的
`config/dual_host.env` 中的 `NVIDIA_FOD_BACKEND`、后端专用模型 SHA256 和
`NVIDIA_FOD_REQUIRED_CLASS_NAMES`。模型文件只需存在于 NVIDIA；活动 SHA256 和
逗号分隔的精确类别名同时提供给 NVIDIA 检测器和 J6M
视觉伺服。托管冷启动会原子同步配置到 J6M，运行时健康检查也会核对两端参数，
避免检测器已经换模型而控制器仍使用旧白名单。

首次升级到支持共享契约的版本时，需要停止主链并执行一次
`scripts/deploy_j6m.sh`，使 `autolabor_fod_control` 进入 J6M 的版本化 release。
以后换模型不需要重新编译：更新活动后端契约并完成对应清单/权重校验，再通过
统一入口完整冷重启。模型和类别不会
在运行中热切换；`FOD_MOTION_ENABLED` 仍是独立运动门，换模型不会自动开启运动。

当前外部 LocateAnything 后端不提供校准的逐框置信度且延迟为数秒，因此 NVIDIA
适配器发布 `confidence=0`、不填充逐框运动深度，并报告
`/fod_detector/motion_eligible=false`。即使 J6M 的功能门原先为 true，这些识别框
也不能通过既有进入门；需要视觉行驶时必须切回已验收的 YOLO，或先完成新模型的
独立标注校准、实时性和真车安全验收。

## 安全边界

- YOLO 不是避障器。第一次及调参试验必须在封闭净空区域进行，人员离开车辆
  前方，操作员全程手持物理急停。
- 运动候选必须有同步的 ZED 注册深度，并通过距离、有效像素数、有效比例和
  MAD 离散度门限。锁定目标仍可见但其深度质量失效时立即进入 `ABORT`。
- 单独运行本页后半部分的 `visual_recovery.launch` 时，只能启动纯 CAN/M2
  底盘；不要同时启动 move_base、GPS 导航、速度限制器、键盘遥控、rqt topic
  publisher 或任何 `rostopic pub /cmd_vel`。
- GPS 联动模式使用 `gps_visual_recovery_standby.launch`：GPS 和视觉控制器只
  分别发布到 `/cmd_vel_gps`、`/cmd_vel_fod`，`/fod_navigation_mode` 是
  `/cmd_vel` 的唯一发布者。不得另起一个独立 `visual_recovery.launch`。
- 独立模式要求视觉控制器自己是 `/cmd_vel` 的唯一发布者；联动模式要求它自己
  是 `/cmd_vel_fod` 的唯一发布者，并且该话题只接到模式仲裁器。两种模式都要求
  `/ackerman_vel` 没有任何发布者，运行中发现控制冲突会失效停车。
- `/m2_driver/steer_center_bias`、`/m2_driver/reset_odom`、
  `/m2_driver/brake_set` 和 `/m2_driver/emergency_stop` 都会绕过 `/cmd_vel`。
  默认模式下 `PRECHECK` 和运行期图审计要求这些话题没有发布者。转向中心标定
  必须在进入本模式前用 `rostopic pub -1` 完成，并等待该一次性发布进程退出；
  不得在视觉回收过程中修改转向中心、复位里程计或另行控制刹车。这三项会使
  运动几何或里程判断失效，因此即使启用外部急停覆盖也仍会被检查；覆盖模式仅
  不再检查属于 VCU 安全链的 `/m2_driver/emergency_stop`。
- ROS Master 只能审计 topic/service 图，不能枚举已连接后绕过 topic、直接调用
  `/canbus_server` 的客户端。因此必须使用可信且无其他底盘/CAN 控制进程的 ROS
  图；“唯一 `/cmd_vel` 发布者”不代表能隔离恶意或错误的直连 CAN 客户端。本视觉
  控制器自身不再连接该服务，周期性 VCU 状态查询只由 NVIDIA 的 M2 驱动负责。
- `allow_motion:=true` 只是第一重授权。节点启动后只发零速，还必须由操作员
  显式调用 `/fod_visual_servo/set_enabled`。
- 默认接近和盲走速度均为 `0.20 m/s`。真车测试中，`0.0525 m/s` 只移动
  `0.0015 m`，提高到 `0.12 m/s` 后三秒仍只累计 `0.0106 m`，两者都只能
  转动前轮而不能让底盘持续起步；`0.20 m/s` 已由同一底盘的直线标定验证
  可以起步，同时也是控制器不可突破的绝对硬上限。
  软件前轮转角不超过 `12°`。
- 默认盲走目标不可配置到 `0.50 m` 以上，绝对里程上限为 `0.55 m`。
- 盲走期间前轮反馈必须持续保持在回中门限内，同时还受 `4°` 航向变化和
  `0.08 m` 横向偏移上限保护。
- `COMPLETE` 前还要求实测速度连续低于 `0.01 m/s`，且确认窗内 odom 路程漂移
  不超过 `0.005 m`。首次真车测试必须选择平整地面，不能在坡面依赖软件零速
  命令阻止车辆溜行。

## 编译

```bash
cd ~/robot_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg autolabor_canbus_driver autolabor_fod_msgs \
  autolabor_fod_vision autolabor_fod_control -j2
source devel/setup.bash
```

## GPS 导航联动（推荐运行方式）

`./scripts/bringup.sh gps` 默认同时启动安全仲裁器和处于 `DISABLED` 的视觉控制
器，但不自动占用相机。正常速度链为：

```text
move_base/TEB
  -> /cmd_vel_navigation
  -> GPS 安全继电器
  -> /cmd_vel_gps
  -> /fod_navigation_mode
  -> /cmd_vel
  -> m2_driver
```

相机与 YOLO 可在第二个终端常驻：

```bash
cd ~/robot_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch autolabor_fod_vision zed_fod_detection.launch \
  start_camera:=true \
  enable_image_quality_controller:=false
```

确认 `/fod/detections` 正常、车辆前方净空且操作员拿好急停后，只需执行：

```bash
cd ~/robot_ws
./scripts/fod_mode.sh start
```

该命令按以下顺序执行，不会让两个控制器同时控制底盘：

```text
GPS_ACTIVE
  -> 检查最近有效深度 FOD
      ├─ 距离 >= 5m：保持 GPS_ACTIVE，不暂停路线
      ├─ 连续 1s 无有效深度 FOD：保持 GPS_ACTIVE，不暂停路线
      └─ 距离 < 5m：开始控制权交接
  -> 屏蔽 GPS 速度
  -> 暂停长距离管理器并取消当前 move_base 子目标
  -> 保留最终 GPS 经纬度目标
  -> 用新鲜 /odom 连续确认停车 0.5s
  -> 使能视觉控制器
  -> FOD_ACTIVE
```

视觉控制器让目标沿图像中线向下移动；目标在底部中央消失后，车辆回正并继续
直行 `0.50 m`，使 FOD 沿车体中心通过滚轴。到达 `COMPLETE` 后，仲裁器先保持
停车、将视觉控制器复位为
`DISABLED`，再从车辆回收后的当前位置重新计算一个滚动 GPS 子目标，自动恢复
GPS。它不会恢复已经取消的旧 15m 子目标，也不会扩大 40m 全局滚动地图。
回收期间新发布到 `/gps/goal_fix` 的目标会被明确拒绝，不会悄悄覆盖已保留的
最终目标；如果确实需要换目标，请等状态回到 `GPS_ACTIVE` 后重新发送。

查看状态：

```bash
./scripts/fod_mode.sh status
# 或持续查看
./scripts/fod_mode.sh watch
```

主动退出回收并恢复 GPS：

```bash
./scripts/fod_mode.sh stop
```

`ABORT` 不会自动恢复 GPS：模式仲裁器会保持 `/cmd_vel` 为零，避免检测断流、
控制冲突或里程异常后车辆突然继续高速导航。排除原因后执行 `fod_mode.sh stop`
明确恢复 GPS；需要再次回收时再执行 `start`。如果相机/YOLO 没启动，视觉
入口连续 `1 s` 收不到有效深度 FOD，会直接保持 `GPS_ACTIVE`，不会暂停路线。

联动模式下不要直接调用 `/fod_visual_servo/set_enabled`；该服务由模式仲裁器
管理。正常操作只使用 `/fod_navigation_mode/set_fod_enabled`，也就是
`fod_mode.sh`。

首次验证建议把盲走距离缩短到 `0.20m`，然后重新启动 GPS：

```bash
FOD_RECOVERY_BLIND_DISTANCE_M=0.20 ./scripts/bringup.sh gps 0.3 cruise
```

默认保留全部 CAN/VCU 安全检查。只有在有人全程手持外部急停且现场确实需要时
才可显式覆盖：

```bash
FOD_RECOVERY_EXTERNAL_ESTOP_OVERRIDE=true \
FOD_RECOVERY_BLIND_DISTANCE_M=0.20 \
./scripts/bringup.sh gps 0.3 cruise
```

如需旧的纯导航拓扑做诊断，可使用
`FOD_RECOVERY_STANDBY_ENABLED=false ./scripts/bringup.sh gps`；此时
`fod_mode.sh` 不可用。

## 从开机零状态启动

以下流程只用于“不启动 GPS/move_base”的视觉控制器独立测试。GPS 联动真车运行
请使用上一节。

### 终端一：相机和 YOLO

保持已经验证过的命令不变：

```bash
cd ~/robot_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch autolabor_fod_vision zed_fod_detection.launch \
  start_camera:=true \
  enable_image_quality_controller:=false
```

先确认下面三个话题接近 `15 Hz`，并确认检测消息的
`depth_synchronized: true`：

```bash
rostopic hz /fod_camera/image_raw
rostopic hz /fod_camera/depth_registered
rostopic hz /fod/detections
rostopic echo -n 1 /fod/detections
```

### 终端二：只启动底盘

不要启动完整导航或键盘控制。如果 `/canbus_driver` 和 `/m2_driver` 已经存在，
不要重复执行本步。

必须使用下面的 `robot_bringup can.launch`（其内部是 `drive_only.launch`）。不要
改用旧的 `autolabor_canbus_driver/m2driver.launch`；旧 launch 没有提供本控制器
要求的 odom 子坐标系和控制超时配置。

```bash
cd ~/robot_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch robot_bringup can.launch \
  port_name:=/dev/ttyUSB0 \
  publish_tf:=true
```

确认急停已经释放，并确认没有其他运动发布者：

```bash
rostopic info /cmd_vel
rostopic info /ackerman_vel
rostopic info /m2_driver/steer_center_bias
rostopic info /m2_driver/reset_odom
rostopic info /m2_driver/brake_set
rostopic info /m2_driver/emergency_stop
```

此时 `/cmd_vel` 的 `Publishers` 应为空；其余五个旁路/控制话题也都应没有
发布者。如果之前使用过下面这种一次性转向中心标定命令，必须先等命令退出，
再启动或使能视觉回收：

```bash
rostopic pub -1 /m2_driver/steer_center_bias std_msgs/Float64 "data: -0.4"
```

### 终端三：启动视觉回收模式

```bash
cd ~/robot_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch autolabor_fod_control visual_recovery.launch \
  allow_motion:=true
```

现场 CAN/VCU 状态回复不可靠、并且确有人员全程手持可立即停车的遥控器时，可
显式使用外部急停覆盖。它会跳过原始 CAN 回包观察、底盘聚合急停、VCU 控制超时、
CAN 图/服务以及软件急停话题检查；不会跳过相机检测、odom、轮角、唯一
`/cmd_vel` 发布者、命令租约、速度/转角、进度、距离和绝对运行超时：

```bash
roslaunch autolabor_fod_control visual_recovery.launch \
  allow_motion:=true \
  external_estop_override:=true \
  blind_distance_m:=0.20
```

该参数默认是 `false`，只对本次重新启动的节点生效。已进入 `ABORT` 的旧进程
必须先停止并按上面命令重启；不能仅修改参数后继续使能。

该命令启动后车辆仍然不会运动，只会持续向 `/cmd_vel` 发零速。可先查看状态：

```bash
rostopic echo /fod_visual_servo/state
rostopic echo /fod_visual_servo/status
```

### 终端四：明确使能

将一个或多个无害、柔软的试验 FOD 放在画面可见范围内，保证最近目标的初始
底部锚点不在画面最下沿，并清空车辆前方至少 `7 m`。操作员拿好物理急停后执行：

```bash
cd ~/robot_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

rosservice call /fod_visual_servo/set_enabled "data: true"
```

服务先进入 `PRECHECK`。检查通过后状态变为 `ACQUIRE`。若有多个合格 FOD，
先按 `depth_m` 选择最近者；正在确认的目标允许 `0.10 m` 深度迟滞，避免相近
目标因测距噪声来回切换。最近目标连续稳定 6 帧后才开始低速前进并通过像素
误差自动转向，不要求目标预先位于正中央。完成锁定后只按空间关联继续跟随同一
目标，不会因为另一个 FOD 后来更近而中途换目标。
多类别模型偶尔会把同一物体同时发布成两个类别。控制器只在所有候选框两两高度
重叠（默认 IoU 不低于 `0.80`）、锚点距离不超过画面对角线的 `0.02`，且注册深度
相差不超过 `0.15 m` 时，将它们视为同一物体并继续原几何锁定。仅仅相邻、框重叠
不足或深度不同的目标仍保持歧义并立即停车，不能用该规则在两个真实物体间跳转。
控制器默认接受置信度不低于 `0.30` 的目标；现场批准模型对当前小尺寸 Metal
目标的稳定输出约为 `0.38–0.43`，此前 `0.45` 的二次门槛会错误地把这些检测
过滤成 `no eligible target`。连续 6 帧确认仍用于抑制单帧误检。
运行时可在 Qt“视觉”页将这个目标锁定阈值调整为 `0.25–0.95`。
只有控制器处于 `DISABLED`、`COMPLETE` 或 `ABORT` 停车状态时才会接受修改；
正在预检或行驶时保留原值。这是本次进程的临时值，冷重启后恢复
`visual_servo.yaml` 的 `0.30`。检测器本身低于 `0.25` 的结果不会发布，
因此控制器阈值不能用于绕过检测器下限。
默认捕获范围是相机中心左右各 `0.65 × 半幅宽`，即画面中央约 65% 的宽度；
进入闭环后保留 `0.70` 的瞬时跟踪边界。更靠近画面极端边缘的目标仍保持停车，
状态详情会直接报告横向误差超出捕获范围。目标到达画面底部中央并经连续帧确认
消失后，程序以
“第一帧目标消失时的插值 odom 位姿”为起点计量 `0.50 m`，而不是从停车回中
结束后才开始计量；最后还会保持零速并用新的 odom/轮角样本确认车已完全停稳。
运行中允许最多 20 个连续空检测帧（20Hz 下约 `1.0s`，以 `0.20m/s` 计算最多
前进约 `0.20m`）沿用最后一次目标闭环命令；随后进入 `REACQUIRE` 停车等待，
累计 60 帧（约 `3.0s`）仍未恢复才锁存 `ABORT`。漏检后的同一目标允许在图像
对角线 `0.18` 范围内重新关联，避免车辆已经前进后把它误判成新目标。
完成时：

```text
/fod_visual_servo/state      COMPLETE
/fod_visual_servo/completed  true
```

随时停车并退出模式：

```bash
rosservice call /fod_visual_servo/set_enabled "data: false"
```

`ABORT` 和 `COMPLETE` 都是锁存状态，不会自动重新启动。需要再次试验时，先
调用一次 `data: false` 清除本次会话，排除原因后再调用 `data: true`。

## 状态与故障定位

结构化状态是 JSON 字符串：

```bash
rostopic echo -n 1 /fod_visual_servo/status
```

其中包含当前状态、停止原因、目标类别/置信度、当前锁定阈值、raw CAN 查询所有者/超时、
目标深度/MAD/采样数、像素误差、目标纵向比例、
接近路程、盲走路程、实际下发的速度/曲率，以及检测、CameraInfo、odom、
前轮转角和底盘状态的新鲜度。相同信息也会写入 `/diagnostics`。

常见停止原因：

- `another publisher`：导航、键盘、手工 topic publisher 等仍在发布速度；
- `M2 bypass-control publisher`：转向中心、里程复位、刹车或软件急停话题仍有
  发布者；先停止对应工具，并等待一次性 `rostopic pub -1` 进程退出；
- `target was lost before reaching the bottom gate`：目标过早丢失，程序不会盲走；
- `receipt timeout` / `source stamp age`：相机、推理或底盘反馈断流/陈旧；
- `raw CAN ... unsafe`：物理、软件、手柄急停或整车运行状态不允许运动；
- `raw CAN ... response is stale`：M2 驱动的底层状态查询没有按时收到回复。M2 是
  唯一周期查询所有者，以平均 `3.0 Hz`、周期 ±20% 去同步抖动发送请求；硬急停、
  软急停、手柄急停和整车运行状态各自最多重试 4 次，再轮换一项遥测。视觉控制器
  只观察回包，不再额外调用 `/canbus_server`。`8.00 s` 新鲜度门覆盖一次全字段均漏回的
  有界公平轮询，但持续缺失仍会中止；已收到的不安全回复仍会立即锁存并中止，不会等待
  `8.00 s`。若现场使用独立遥控急停并由
  人员全程值守，可按上面的 `external_estop_override:=true` 明确跳过该安全链；
- `front steering did not settle`：目标消失后前轮未能在 3 秒内回中；
- `blind ... deviation/watchdog`：0.5 m 阶段航向、横偏、里程或进度异常。

## 现场标定参数

默认横向基准使用运行时 ZED CameraInfo 的主点 `cx`，不是硬编码为图像宽度
的一半。实际回收机构中心线与相机光轴可能不重合，可小幅调整：

```bash
roslaunch autolabor_fod_control visual_recovery.launch \
  allow_motion:=true \
  target_u_offset_px:=20
```

正值表示希望 FOD 位于画面更靠右的位置。必须用柔软标记、小步调整并记录结果，
不能根据单次试验大幅修改。

“消失后 0.5 m”本质上是相机盲区边界到实际回收口的安装几何量。程序已按用户
要求默认设为 `0.50 m`，但正式回收前仍应先用无害标记确认该距离确实能让回收
机构越过目标。可以在早期安全试验中临时调小，程序拒绝调大到 0.50 m 以上：

```bash
roslaunch autolabor_fod_control visual_recovery.launch \
  allow_motion:=true \
  blind_distance_m:=0.20
```

其余门限集中在 `config/visual_servo.yaml`。不要先提高速度、转角、盲走距离或
放宽航向/横偏保护；应先从调小速度和盲走距离开始验证转向符号、相机中心线与
回收机构几何关系。
