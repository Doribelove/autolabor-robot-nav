# P02 独立 raw / bag 审查

独立测量/协议检查 24/24 通过；冻结评估器精确复演为 119/120 项通过，唯一失败 `actual_center_slew_with_declared_sampling_tolerance`。P02 总 pass 仍为 false，不重跑、不改数据或门。

本轮测量与命令执行证据可支持后续独立新身份的物理控制候选评价；这不是 P02 整体物理资格通过，也不是导航性能或实车资格。所有审查只读，未启动 ROS/Gazebo。

| 项目 | 独立复算 |
|---|---|
| 正式区间 | 1.108–37.108 s，36 s |
| 命令 | 640 条；639 个相邻间隔均为 50000000 ns；missed_slots 最大 0 |
| loss 区间 | 33.108–37.108 s；source event、bag导航输入及relay均无发送 |
| 新 physical_status | 正式 1800 包，间隔精确 20000000 ns；与odom/target源stamp列表完全相同 |
| 正式 clock | 18001 点，覆盖含首尾，间隔精确 2000000 ns |
| 原 50 Hz 实际中心转向峰值 | 0.886363714902 rad/s，原门0.85；超限区间 3 个 |
| 附加 500 Hz 峰值 | 0.944291354618 rad/s；仅补充诊断，不替代50Hz门 |

原20ms峰值源区间为 19.222→19.242 s，属于 zero_speed_yaw 回正阶段。四元数Z twist独立算法的50Hz最大速率差仅 0.000003725 rad/s，不能用Euler投影解释失败。实际中心角最大绝对值 0.487349 rad。

target加速度 [-0.900000, 0.600000] m/s²、target slew [-0.750000, 0.750000] rad/s；实际加速度 [-4.229160, 1.164693] m/s²。实际速度/加速度来自物理观测，参考边界不能当实际保证；实际加速度/jerk没有事前资格门。

| topic | raw / bag | 共享包完全相同 | 正式窗口缺包 |
|---|---:|---:|---:|
| `/astra/world_models` | 1858 / 1858 | 1858 / 1858 | 0 |
| `/astra/world_links` | 1858 / 1858 | 1858 / 1858 | 0 |
| `/astra/physics_models` | 18572 / 18572 | 18572 / 18572 | 0 |
| `/astra/physics_links` | 18572 / 18572 | 18572 / 18572 | 0 |
| `/odom` | 1857 / 1857 | 1857 / 1857 | 0 |
| `/joint_states` | 1857 / 1857 | 1857 / 1857 | 0 |
| `/m2_driver/actuator_target` | 1857 / 1857 | 1857 / 1857 | 0 |
| `/m2_driver/actuator_commanded_effort` | 1857 / 1857 | 1857 / 1857 | 0 |
| `/astra/physics_contacts` | 18569 / 18572 | 18569 / 18569 | 0 |
| `/m2_driver/physical_status` | 1857 / 1857 | 1857 / 1857 | 0 |
| `/astra_fixture/command_source_status` | 3 / 3 | 2 / 2 | 0 |
| `/astra_fixture/command_event` | 670 / 666 | 666 / 666 | 0 |

录包节点在 unpause 前完成全部26个topic的TCPROS incoming连接；warmup源区间 0.000→1.000 s，随后Trigger、running与正式start顺序成立。source状态完整为bootstrap/running/complete，标记与Trigger身份一致，终态零命令恰一条并在正式结束之后。P01启动期bag缺包没有在P02正式区间复现。contacts的raw尾部缺包只在正式结束之后，保留在JSON而不称全文件完全相等。

旧Bool仍有 6 个大于0.06s的collector接收间隙，最大 0.140 s；独立bag Bool周期仍约20ms，且新带header状态源包完整。这支持旧Bool接收时钟受collector调度影响的解释，不能再将该间隙当driver源停发；P02隔离命令源本轮没有P01的同窗主循环延迟。Bool无header仍不具备逐条精确源时间证明。

四轮地面接触pair累计 64712 次，非轮地面 0、非地面机器人接触 0；ground support保留而未误作障碍碰撞。全物理门与负面原样保留。

`recompute_raw.py` 进行独立四元数、动力学、接触和共享payload复算；`audit_protocol.py` 重演冻结评估器，并独立检查整数命令、录包完整覆盖与source状态。CSV全部自动来自raw。初稿r0沿用的P01主循环因果措辞已修正并保留，数值结果未变。

```bash
/usr/bin/python3 /home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/audits/m12_p02_raw_independent/recompute_raw.py
/usr/bin/python3 /home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/audits/m12_p02_raw_independent/audit_protocol.py
/usr/bin/python3 /home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/audits/m12_p02_raw_independent/write_report.py
```
