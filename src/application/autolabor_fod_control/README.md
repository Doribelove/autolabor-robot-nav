# Autolabor FOD Visual Recovery Control

本包在现有 `/fod/detections` 基础上提供一套真车视觉伺服回环。它只使用像素
位置接近 FOD；由于当前还没有经过验证的相机安装外参，不会用 bbox 大小伪造
米制距离。ZED 注册深度用于从多个 FOD 中选择并锁定最近一个。目标在闭环
转向校正下接近，锁定目标进入中下回收区后记住该条件。随后第一次检测不到该目标时，
从这帧的 `/odom` 插值位姿累计净前进距离，连续直行 `0.80 m` 后停车。
本次只回收锁定的一个目标，不再中途停车回正，也不再使用旧的 `0.15 m` 滑行限制。

状态机为：

```text
DISABLED → PRECHECK → ACQUIRE → APPROACH
                                  ↕（未进入回收区的较长漏检）TRACK_WAIT
                                  ↓（已进入回收区，锁定目标消失）
                              BLIND_ADVANCE → FINAL_STOP → COMPLETE
控制冲突、底盘反馈断流、急停或真实里程异常 ───────────────────→ ABORT
```

GPS 联动仍由模式管理器执行最终 GPS 目标小于 10 米的接管条件。锁定最近一个
FOD 后，回收区默认覆盖地面识别带的下半部分、以 CameraInfo 主点为中心的 60%
图像宽度。当前 1280×720 ZED 配置约为 x=236～1004、y=315～449；上方 2/8、
下方 3/8 的识别遮罩保持原配置。Qt 相机预览显示黄色回收区，选中目标进入后变绿。
进入条件锁存，边缘抖动不会撤销。随后新鲜检测帧中不再有该目标，立即完成一段
0.80 米直行；框再次出现或其他物体仍在画面中都不重置距离、不切换目标。
锁定后的真实低分框、暂时无有效深度的同一 ID 框仍用于跟踪。

未进回收区时，0.30 秒内的漏检沿用目标跟随；较长漏检进入可恢复的 `TRACK_WAIT`，
同一目标回来后继续，不以普通漏检锁存 `ABORT`。进入 `BLIND_ADVANCE` 后即使相机
断流或出现无效帧，也继续按里程走完回收；首次消失之前，相机断流仍不能冒充有效
目标消失。未曾锁定目标时执行既有 GPS 导航：启用回收且离目标不足 10 米会开启清扫，
没有有效 FOD 则继续朝 GPS 目标行驶。

2026-09-29 按用户要求，完全删除识别消息 `0.35 s` 断流中止、控制更新
`0.25 s` 到期停车，以及穿越期间 `4°` 航向/`0.08 m` 横偏中止条件。
YOLO 暂停发帧或检测节点断开时沿用最后观测，不把断流伪造为空检测帧；
控制更新延迟期间持续发布最后指令，直到控制器明确更新/停车或原任务总时限到期。
相机 CameraInfo、里程计、底盘反馈的独立检查继续按原配置执行。

## 安全边界

- YOLO 不是避障器。第一次及调参试验必须在封闭净空区域进行，人员离开车辆
  前方，操作员全程手持物理急停。
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
  图；“唯一 `/cmd_vel` 发布者”不代表能隔离恶意或错误的直连 CAN 客户端。
- `allow_motion:=true` 只是第一重授权。节点启动后只发零速，还必须由操作员
  显式调用 `/fod_visual_servo/set_enabled`。
- 2026-09-29 按用户要求，接近和盲走标称速度、控制器硬上限改为 `0.60 m/s`。
  转向时仍按原曲率逻辑调速；消失后的直行按剩余距离逐步降速以适应底盘惯性、
  反馈与指令延迟，使用 `0.80 m` 停车目标和主动前进阶段的 `0.90 m` 路程上限。
  发出零速后的制动滑行单独计入实际距离及 `stop_overshoot_m`，不再仅因滑过
  `0.90 m` 把停车确认变成视觉中止；停车超时、倒滑及停稳反馈检查继续生效。
  真车历史测试中，`0.0525 m/s` 只移动
  `0.0015 m`，提高到 `0.12 m/s` 后三秒仍只累计 `0.0106 m`，两者都只能
  转动前轮而不能让底盘持续起步；`0.20 m/s` 已由同一底盘的直线标定验证
  可以起步，因此仍作为接近/末端降速的最低起步指令。本次 `0.60 m/s` 尚未实车路测。
  软件前轮转角不超过 `12°`。
- 默认盲走目标为 `0.80 m`，主动前进阶段路程上限为 `0.90 m`。
- 消失后下发直行指令，前轮在行驶中回正；航向变化和横向偏移只记录到状态，
  没有对应中止阈值。最终确认要求轮角反馈有效新鲜，但车已停稳时不再要求
  前轮精确回到 2° 内，避免把转角残差误报为“未停稳”。
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

roslaunch autolabor_fod_vision hikrobot_fod_detection.launch \
  start_camera:=true \
  enable_image_quality_controller:=true \
  image_quality_exposure_max_us:=12000
```

确认 `/fod/detections` 正常、车辆前方净空且操作员拿好急停后，只需执行：

```bash
cd ~/robot_ws
./scripts/fod_mode.sh start
```

GPS 模式下该命令与 Qt“启动相机控制回收”相同：先进入等待，保持正常 GPS
导航。底盘距**最终 GPS 目标**小于 `10.0 m` 即后台联动液压杆升出 3 秒和启动清扫，不依赖相机
或检测；最新实际检测帧在地面有效区域有属于原六类、置信度不低于 `0.30` 的 FOD
时才切换为视觉伺服。没有检测时按原 GPS 目标导航并清扫。
距离来自同一 `camera_init` 坐标系下的 `/gps/odom` 与
`/gps/long_range/status.final_x/final_y`，不使用滚动子目标或 bbox 估算距离。
低分预览框不触发视觉接管；检测断流不再按经过时间清除已有目标状态，后续真实
空帧仍会清除。视觉控制器初次使能仍等待新帧与稳定锁定，不凭断流伪造目标。
等于或超过 10 米时暂不自动开启清扫，继续 GPS 导航。

切换过程沿用原有控制链：

```text
GPS_ACTIVE
  -> 操作员开启相机回收，recovery_armed=true，继续 GPS 导航
  -> 最终目标距离 <10m：液压杆升出 3 秒，同时启动清扫，GPS 保持行驶
  -> 无有效 FOD：按 GPS 目标回收，到达后停止清扫并回收液压杆 3 秒
  -> 范围内出现有效 FOD：执行以下视觉接管
  -> 屏蔽 GPS 速度
  -> 暂停长距离管理器并取消当前 move_base 子目标
  -> 保留最终 GPS 经纬度目标
  -> 用新鲜 /odom 连续确认停车 0.5s
  -> 再确认清扫装置已开启；同一轮不重复升出液压杆
  -> 使能视觉控制器
  -> FOD_ACTIVE
```

未开启相机回收时完全按原 GPS 流程导航。等待期间可以更换最终 GPS 目标；
取消已开始的清扫会停止清扫并回收液压杆 3 秒，不会暂停 GPS 或取消子目标。视觉接管停车
期间若目标消失，恢复 GPS 并保持清扫。已进入视觉模式后仍按原单目标跟踪、
遮挡确认、穿越流程执行，不会因目标正常进入盲区而退回 GPS。

ZED 模式使用注册深度及原生内参，按距相机的直线距离选择捕获范围内最近的
有效 FOD，再按独立检测编号锁定。其他目标仍显示，但不参与本次回收，也不会
阻止已锁定目标进入盲区。深度未知的框不参与首次选择；已锁定目标短时深度
空洞时可以继续按同一编号的像素观测跟踪。显示保留框不会送入控制。

视觉控制器到达 `COMPLETE` 后，仲裁器保持 `FOD_COMPLETE_STOP` 和零速度，
消耗本次回收请求，GPS 保持暂停；剩余目标不会触发下一次回收。点击 Qt 的
“取消相机回收 / 恢复导航”（或 `fod_mode.sh stop`）才从当前位置恢复 GPS，
需要回收下一个时再点击启动。海康原单像素目标选择保持兼容。
自动回收一旦开始即联动控制清扫电机和液压杆，包括电机原本已手动开启的情况。
GPS 到达、视觉完成、退出或故障中止时，停止清扫并回收液压杆 3 秒。
开始使用 `sweep_test.py recovery-start`：同一 TCP 连接先发升出 3 秒指令，收到 ACK
后立即启动清扫，两项动作重叠；结束使用 `recovery-stop`：先发停止清扫指令，紧接着
发回收 3 秒指令，再确认清扫状态。液压动作计时结束才释放设备锁、完成该次操作。
同一轮 GPS 接近与视觉接管共用一次升出；完成、取消、退出的收尾只发一次回收脉冲。
液压杆只有指令 ACK，没有到位反馈；通信失败报告未确认，不自动重发液压脉冲。
初始联动失败时对已下发的动作执行一次收尾，GPS 原导航保持运行。
自动控制默认地址 `192.168.0.197:50003`，与 Qt 手动清扫共用脚本和设备互斥锁。
范围内清扫启动失败会报告原因并保持 GPS 导航，不反复尝试切换；取消再开启
可重试。未开启回收的 GPS 导航不依赖清扫装置。
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

`ABORT` 不会自动恢复 GPS：模式仲裁器会保持 `/cmd_vel` 为零，避免控制冲突
或里程异常后车辆突然继续高速导航。排除原因后执行 `fod_mode.sh stop`
明确恢复 GPS；需要再次回收时再执行 `start`。GPS 等待回收阶段若相机/YOLO
没有有效检测，继续 GPS；已经切入后的传感器故障仍由原视觉控制器处理。
`retain_gps_route=false` 的室内模式不使用 10 米条件，按按钮直接切换：确认停车后
升出液压杆 3 秒并启动清扫，动作计时结束后使能视觉回收。

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

roslaunch autolabor_fod_vision hikrobot_fod_detection.launch \
  start_camera:=true \
  enable_image_quality_controller:=true \
  image_quality_exposure_max_us:=12000
```

先确认下面两个话题接近 `20 Hz`：

```bash
rostopic hz /fod_camera/image_raw
rostopic hz /fod/detections
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
显式使用外部急停覆盖。它会跳过原始 CAN 查询、底盘聚合急停、VCU 控制超时、
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

将一个无害、柔软的试验 FOD 放在地面识别带内，并清空车辆前方至少 `7 m`。
操作员拿好物理急停后执行：

```bash
cd ~/robot_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

rosservice call /fod_visual_servo/set_enabled "data: true"
```

服务先进入 `PRECHECK`。检查通过后状态变为 `ACQUIRE`，唯一目标连续稳定
6 帧后才开始低速前进并通过像素误差自动转向，不要求目标预先位于正中央。
控制器默认接受置信度不低于 `0.30` 的目标；现场批准模型对当前小尺寸 Metal
目标的稳定输出约为 `0.38–0.43`，此前 `0.45` 的二次门槛会错误地把这些检测
过滤成 `no eligible target`。连续 6 帧确认仍用于抑制单帧误检。
默认捕获范围是相机中心左右各 `0.65 × 半幅宽`，即画面中央约 65% 的宽度；
进入闭环后保留 `0.70` 的瞬时跟踪边界。更靠近画面极端边缘的目标仍保持停车，
状态详情会直接报告横向误差超出捕获范围。锁定目标进入回收区后再消失，程序以
“第一帧目标消失时的插值 odom 位姿”为起点计量 `0.80 m`，而不是从停车回中
结束后才开始计量；最后保持零速，用新 odom 样本的速度与位移确认车已完全停稳。
这段距离只执行一次，不因框再出现、低分抖动或其他 FOD 停车或换目标。ZED 模式按
已锁定 track_id 关联，最近目标只在本次开始时选择一次。里程传输延迟使用已有
odom 新鲜度时限等待，源时间插值精度仍独立检查；偶发丢帧按时间间隔判断允许位移，
不把正常的较大帧间位移当作瞬移。末端减速计入里程反馈的源时间延迟，提前降速；
停车触发仍要求实测净前进达到目标，不使用预测位姿提前完成任务。
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

其中包含当前状态、停止原因、目标类别/置信度、像素误差、目标纵向比例、
接近路程、盲走路程、实际下发的速度/曲率，以及检测、CameraInfo、odom、
前轮转角和底盘状态的新鲜度。相同信息也会写入 `/diagnostics`。
`recovery_region_px` 给出实际回收区，`recovery_region_entered` 表示已进入，
`vision_required` 表示当前阶段是否仍需要视觉帧。
`blind_heading_change_deg`、`blind_lateral_m` 记录穿越航向变化和横移，仅供观察。

常见停止原因：

- `another publisher`：导航、键盘、手工 topic publisher 等仍在发布速度；
- `M2 bypass-control publisher`：转向中心、里程复位、刹车或软件急停话题仍有
  发布者；先停止对应工具，并等待一次性 `rostopic pub -1` 进程退出；
- `receipt timeout` / `source stamp age`：接近时 CameraInfo，或任意运动阶段底盘反馈断流/陈旧；YOLO 检测消息不再执行这两项年龄检查；
- `raw CAN ... unsafe`：物理、软件、手柄急停或整车运行状态不允许运动；
- `raw CAN ... response is stale`：底层状态查询没有按时收到回复。默认模式每
  `0.20s` 只查询一项并轮询四项状态，避免串口桥把四条查询背靠背发送导致 VCU
  漏回包；`2.50s` 超时覆盖两个以上完整轮询周期。若现场使用独立遥控急停并由
  人员全程值守，可按上面的 `external_estop_override:=true` 明确跳过该安全链；
- `blind ... distance/progress`：0.8 m 阶段路程超限、里程异常或长时间没有前进；航向变化和横偏不再单独触发中止。

## 现场标定参数

默认横向基准使用 CameraInfo 的主点 `cx`（当前标定约 `620.04 px`），不是简单
假定 `1280/2=640 px`。实际回收机构中心线与相机光轴可能不重合，可小幅调整：

```bash
roslaunch autolabor_fod_control visual_recovery.launch \
  allow_motion:=true \
  target_u_offset_px:=20
```

正值表示希望 FOD 位于画面更靠右的位置。必须用柔软标记、小步调整并记录结果，
不能根据单次试验大幅修改。

“进入回收区后消失，再走 0.8 m”使用相机盲区边界到实际回收口的安装几何量。
程序已按用户要求默认设为 `0.80 m`，可用柔软标记确认这一距离与实际装置的关系。
需要临时缩短时可以使用：

```bash
roslaunch autolabor_fod_control visual_recovery.launch \
  allow_motion:=true \
  blind_distance_m:=0.20
```

其余参数集中在 `config/visual_servo.yaml`。回收区对应 `recovery_region_top_fraction`
和 `recovery_region_width_fraction`，分别以地面识别带高度和完整图像宽度归一化。

## 完整任务仿真

`test/full_recovery.test` 在独立 ROS master 上运行真实视觉控制器和模式管理器，
用模拟底盘、ZED 消息、GPS 路线及本机回环 TCP 清扫装置执行整个任务。
它不连接真实设备。覆盖 10 米接管、最近目标选择、漏检/深度抖动、框闪回、
回收区边界、相机失联、里程延迟/丢帧、转向执行延迟、制动、停稳与关闭清扫，
并验证真实急停、里程断流/跳变、底盘卡住仍应停车。

```bash
cd /home/slam/robot_ws_agx
source devel/setup.bash
ROS_MASTER_URI=http://127.0.0.1:11391 rostest autolabor_fod_control full_recovery.test \
  seed:=1 results_file:=/tmp/fod_full_recovery_results.json
```

`seed` 改变模拟执行延迟；`case_filter` 可指定逗号分隔的场景名称。每个场景保存
状态、距离、选中 ID、清扫开关和轨迹。仿真不验证真实 YOLO 准确率或清扫吸入口效果。
# GPS 目标丢失接续（2026-10-13）

GPS + ZED 集成模式启用 `gps_fallback_enabled`。相机回收仍由原按钮开启；距原最终目标
小于 10 米即升杆 3 秒并启动清扫，不要求此时可见 FOD。6 米内最近 FOD 的同一 track ID
连续 6 个源时间戳、世界平面位置相对中位数偏差不超过 0.25 米后，记录 WGS84 点。
使用检测框中心对应的 ZED registered depth 光轴 Z、CameraInfo.P、图像源时间戳的 TF，
以及现有 GPS 原点/yaw_offset 转换；不是按当前车位加估计距离，也不使用框底部像素代替
实际深度采样中心。GPS 记忆与视觉获取指定同一 track ID。

目标可见时仍执行视觉靠近。首个新的检测帧中该目标缺席后，固定最后稳定 FOD 点和
初始接近方向，向 `/gps/goal_fix` 只发一次前延 0.8 米终点。该距离从 FOD 点计算，
不是从丢框时车位计算；GPS 到达仍使用已有 0.3 米容差。后续空帧、框返回、旁观目标
均不影响已发送终点。回收阶段速度上限 0.6 m/s，转速同比缩放以保持曲率。

`ENTERING_GPS_RECOVERY` 切换时先禁用视觉输出，GPS 原路线恢复期间仍选 STOP；确认
新的 GPS 路线身份后进入 `GPS_FOD_RECOVERY`。GPS 管理器每次发送（包括恢复最终段）
都生成新 GoalID，旧取消/完成消息不会结束新任务。只把本次路线 COMPLETE 记为行程完成，
ABORTED/CANCELED 不再显示回收成功；随后停清扫、收杆 3 秒。地图/禁入区和原避障继续生效。

状态 `/fod_navigation_mode/status` 含 `fod_gps_point`、`fod_gps_reason`、
`gps_fallback_route_id`；Qt 显示记录坐标和接续阶段。室内独立视觉模式保持原流程。
`test/gps_loss_recovery.test` 使用隔离 ROS master、真实 GPS 管理器/限速器/视觉控制器/
模式管理器、真实 actionlib 状态历史以及模拟车辆/传感器/规划器/清扫 TCP；不接实车设备。

## 2026-10-14 模糊 GPS 点回收修复

GPS 点只负责接近；近距离以真实单个 FOD 的注册深度和相机到车体 TF 做 pure pursuit 转向。短时漏检允许 1 秒重接，模式管理器不再用另一个 0.3 秒计时抢先切走。真正失去目标或进入遮挡区后仍接续该 FOD 的 GPS 穿越终点，保留 0.8 米偏移。修复了视觉进入 GPS_HANDOFF 同一控制周期又被判断为未知状态的问题。

同一目标的近距离连续稳定深度可以修正初始记录点，但必须位于首次确认点 0.5 米以内，不能通过逐帧漂移无限拖动目标。已知其他目标不参与本次回收。

清扫仍在距导航目标小于 10 米时联动升杆 3 秒及电机；去掉 TCP 连接后固定 0.5 秒静默等待，已确认开启后进入视觉模式不再重复连接查询。电机与升杆并行，完成后停机和收杆 3 秒。

新增真实 pt3 的 0.383 秒漏检重放，以及 GPS 横向误差正负 1 米的 ROS 整链路仿真。仿真通过不表示实物已入仓。
