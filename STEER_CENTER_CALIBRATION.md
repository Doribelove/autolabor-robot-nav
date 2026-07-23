# M2 直线航向与转向中心标定

该工具让车辆以低速发送一段 `angular.z=0` 的直线 `/cmd_vel`，使用双天线
GNSS 的 `/gps/odom` 独立测量航向变化，并换算成大致的等效前轮中心误差。

工具只生成测量记录和下一次试验建议，不会发布
`/m2_driver/steer_center_bias`。

## 为什么不能直接使用航向变化角

车辆行驶后的航向变化不等于转向轮偏差。恒定小曲率近似为：

```text
曲率 k ≈ 航向变化 / 行驶距离
等效前轮偏角 delta = atan(轴距 * k)
```

例如轴距 `0.65m`、行驶 `5m`、航向向右变化 `3.08°`，对应的等效前轮
误差约为 `-0.40°`，而不是 `-3.08°`。

程序会从 `/m2_driver/chassis_parameter` 动态读取驱动实际使用的
`parameters.robot_length` 作为轴距，并用稳态区间内全部样本拟合曲率。

还要区分两种现象：

- 航向随距离持续变化，即 `d(yaw)/ds != 0`：更像转向中心偏差；
- 航向基本不变，但运动轨迹相对车头斜着走：更像蟹行、后桥/轮胎问题、横坡，
  或双天线安装方向误差。

程序会额外计算近似蟹行角：

```text
蟹行角 ≈ 起终点弦方向 - (起始航向 + 结束航向) / 2
```

它扣除了正常圆弧轨迹中约一半航向变化的影响。蟹行角超过质量门限时仍记录
测量值，但不会给出转向中心偏置建议。

## 安全要求

- 只在平整、空旷、硬质路面测试；默认需要前方至少 `7m` 净空、左右各 `1m`。
- 物理急停/遥控人员必须就位。
- 不要运行完整的 `./scripts/bringup.sh gps`，也不要同时运行键盘遥控。
- 测试期间本工具必须是 `/cmd_vel` 唯一发布者。
- 默认安全模式保持 `0.20m/s`，且硬限制为 `0.30m/s`。只有显式启用外部遥控
  急停覆盖时，程序才允许最高 `0.50m/s`。
- 建议相反场地方向各测试至少 3 次；若结果随方向反转，优先检查横坡、风、
  GNSS 多路径、胎压和载荷。

程序在运动前还会检查：

- `/cmd_vel` 没有其他发布者，且 `/m2_driver` 已订阅；
- `/ackerman_vel` 没有发布者；
- `/m2_driver/steer_center_bias` 没有持续发布者；
- `/gps/odom` 新鲜、有限、frame 正确，且唯一发布者是 `/gps_localization`；
- GPS 节点确实使用双天线航向、GNSS 位置和底盘轮速（拒绝轮式里程计位置替代）；
- 底盘状态、轮角和控制超时话题唯一来自 `/m2_driver`，控制超时上报已启用；
- `/odom` 唯一来自 `/m2_driver`，其时间戳和轮速保持新鲜；
- 主动向底层 CAN 查询硬件急停、软件急停、手柄急停和整车运行状态，逐项收到
  新鲜原始响应并确认安全；
- 零速基线期间前轮已回中且反馈稳定；
- 双天线静止航向稳定；
- VCU 没有报告控制命令超时。

四项原始 CAN 状态采用单项轮转查询，避免串口桥在同一批次无间隔发送四个请求
时丢失 VCU 响应。默认每 `0.2s` 查询下一项；任一项连续 `2.5s` 没有有效响应，
或任一响应显示急停未释放/整车不在运行态，都会立即发送零速并中止。

如果现场已经确认遥控器物理急停可靠、人员全程持有遥控器，并且底层串口只因
状态查询响应丢失而误报超时，可以显式传入
`_use_raw_can_emergency_check:=false`。该覆盖只关闭四项原始 CAN 主动轮询；
`/m2_driver/chassis_info` 聚合急停状态、VCU 控制超时、命令租约、GPS、轮角、
速度和控制发布者检查仍然生效。程序会在终端警告并在 JSON 中记录此覆盖。

若现场决定完全由外部遥控器承担急停，可改用
`_external_estop_override:=true`。它会跳过原始 CAN、底盘聚合急停状态以及 VCU
控制超时门禁，同时自动跳过上述原始 CAN 轮询。GPS、底盘轮速、轮角、唯一
控制发布者、速度/偏航/横移上限、`0.30s` 命令租约和绝对超时仍然生效；这些
也是生成有效标定结果所必需的。该选项仅限遥控器急停人员全程在场时使用。

外部急停模式还可显式传入 `_use_progress_watchdog:=false`，关闭起步后的有效
进展门禁。完全未起步检查仍保留；但车辆一旦被判定起步，随后停滞时软件会继续
发出速度命令，直到达到距离条件或绝对运行超时，因此遥控器操作员必须持续观察。

运行中任何话题冲突、GPS 超时/跳变、底盘急停、控制超时、横向偏移过大、
航向突变、轮角异常、连续超速或绝对超时都会触发零速。正速度还有 `0.30s`
主循环安全租约：即使程序卡在 ROS master 或文件 I/O，独立定时器也会在租约
到期时自行切为零；起步后 GPS 位移和底盘轮速积分若都连续 `3s` 没有至少
`0.02m` 的有效进展也会中止。这样可兼容底盘低速起步期间 GPS 的静止位置
保持，但轮速同样停滞时仍会停车。到达目标后必须确认连续静止 `0.5s`，否则
结果不标记为完成。

双天线航向瞬时变化率默认门限为 `20deg/s`。默认安全模式单次超限即中止；
外部遥控急停覆盖模式会丢弃前两个连续超限样本，第三个连续尖峰才中止，以免
GNSS 单点抖动误停。累计航向变化和横向偏移门禁不受此容错影响。

正常退出、异常和 Ctrl-C 都会以 20Hz 连续发送零速至少 1 秒；这些软件保护
不能替代物理急停。按默认实测速度保护上限估算，无进展看门狗触发前仍可能
额外移动约 `1.1m`，之后还需要制动距离；务必保留文首要求的完整净空。

## 编译

```bash
cd /home/robot/robot_ws
source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps robot_diagnostics -j2
source devel/setup.bash
```

## 启动最小标定栈

不要启动 `move_base`。以下命令分别放在三个终端中。

终端一，启动底盘：

```bash
cd /home/robot/robot_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch robot_bringup can.launch port_name:=/dev/ttyUSB0 publish_tf:=false
```

终端二，启动双天线 GPS 定位：

```bash
cd /home/robot/robot_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch robot_bringup gps_localization.launch \
  port:=/dev/ttyUSB1 baud_rate:=115200 broadcast_tf:=true \
  heading_source:=dual_antenna use_wheel_twist:=true wheel_odom_topic:=/odom
```

`roslaunch` 会在需要时自动启动 ROS master。若串口无权限，先按现场权限策略
处理 `/dev/ttyUSB0` 和 `/dev/ttyUSB1`，不要改变设备号后盲目启动。

终端三，先确认没有其他控制发布者：

```bash
rostopic info /cmd_vel
rostopic info /ackerman_vel
```

然后运行默认 `0.20m/s、5m` 标定：

```bash
cd /home/robot/robot_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun robot_diagnostics steer_center_calibration.py \
  _allow_motion:=true \
  _speed_mps:=0.20 \
  _distance_m:=5.0 \
  _current_bias_deg:=0.0
```

现场明确使用遥控器外部急停覆盖时，命令为：

```bash
rosrun robot_diagnostics steer_center_calibration.py \
  _allow_motion:=true \
  _external_estop_override:=true \
  _use_progress_watchdog:=false \
  _speed_mps:=0.50 \
  _distance_m:=5.0 \
  _current_bias_deg:=-0.4
```

程序完成自动预检后仍会要求在终端中准确输入 `START`，随后先采集静止基线，
再开始运动。默认模式还会逐项取得原始 CAN 急停状态；显式关闭该检查时则跳过。

操作员可以在提示处从容完成现场确认，不需要抢在 CAN 状态超时前输入。
收到 `START` 后，程序仍保持零速，重新取得 GPS、底盘和 ROS 控制图，并要求
短暂的无控制超时稳定窗口；默认模式还会丢弃旧急停缓存并重新取得四项原始
CAN 状态。复检自动通过后才进入静止基线，不会再要求第二次输入。

如果当前已经向底盘设置过 `-0.4`，必须如实传入：

```bash
rosrun robot_diagnostics steer_center_calibration.py \
  _allow_motion:=true _current_bias_deg:=-0.4
```

否则“下一次绝对偏置候选值”会计算错误。

## 结果与符号

每次结果保存在忽略提交的目录：

```text
/home/robot/robot_ws/test_results/steer_center_calibration_*.csv
/home/robot/robot_ws/test_results/steer_center_calibration_*_summary.json
```

CSV 保存每个 GPS odom 样本、原始 GPS 航向、轮角、命令、累计距离和横向偏移。
运动距离以 `/odom` 中的底盘轮速积分为主，并用 GNSS 弦长和累计路径
交叉检查；CSV 同时记录原始航向/轮角相对该 odom 样本的接收时间差。
摘要重点字段：

- `net_heading_change_deg_left_positive`：ROS 航向变化，左正右负；
- `gps_heading_change_deg_right_positive`：原始 GNSS 航向变化，右正左负；
- `lateral_displacement_m_left_positive`：相对初始车头的横向偏移，左正右负；
- `estimated_crab_angle_deg_left_positive`：扣除圆弧效应后的蟹行角近似；
- `fitted_heading_drift_deg_per_m_left_positive`：每米偏航；
- `fit_heading_rmse_deg`：恒曲率航向拟合残差；
- `equivalent_front_steer_error_deg_left_positive`：换算后的等效前轮误差；
- `position_equivalent_front_steer_deg_left_positive`：由终点位置独立反算的交叉检查；
- `suggested_bias_increment_deg`：建议偏置增量；
- `suggested_next_absolute_bias_deg`：结合 `_current_bias_deg` 的下一次候选值。

最后两个建议字段只会在试验完整结束并通过全部质量门限时出现。默认
`_recommendation_gain:=0.5`，因此候选值采用半步修正；等效误差仍会按完整测量值
报告。中止、蟹行角超过 `0.5°`、等效误差超过 `1°`、拟合残差/不确定度过大、
轮速距离与 GNSS 弦长不一致、位置与航向估计不一致，或单次候选增量超过
`0.5°` 时，程序会明确抑制建议。中止数据只能用于排查，不能用于调偏置。

`/gps/heading` 是“北为 0°、顺时针为正”的原始 GNSS 航向；计算使用的
`/gps/odom` 则是“东为 0、逆时针/左转为正”的 ROS ENU yaw，两者符号相反。

当前驱动源码只把 `steer_center_bias` 数值原样写入 CAN，没有声明单位、符号或
掉电持久化。工具的建议采用现场已验证约定：“负值向左修正、正值向右修正”，
并把数值视为度。正式修改前请先用至少 3 次结果取中位数；不要让程序自动写入。
