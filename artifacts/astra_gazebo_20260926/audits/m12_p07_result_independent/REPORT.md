# P07 独立结果审计

六回合按冻结顺序完成，6 consumed / 0 retired，均为 attempt1。653 项发布资源及六回合各 658 项输入绑定的活动文件、归档字节全部匹配，原 122 门评估精确回放。每回合另通过 28 项独立源协议与 15 项完整暂停水位审计。

| Seed | Kd4 原生20ms峰(rad/s) | Kd6 原生20ms峰(rad/s) | Kd4/Kd6通过门数 |
|---|---:|---:|---:|
| 260926023 | 0.843004388129 | 0.820135095046 | 122/122 |
| 260926024 | 0.896392361264 | 0.834794571628 | 121/122 |
| 260926025 | 0.896392361264 | 0.834794571628 | 121/122 |

Kd6 三回合均通过122/122，仅获得本冻结仿真执行资格。Kd4 seed024/025均保留唯一 `actual_center_slew_with_declared_sampling_tolerance` 失败，未改变门槛或结果。

正式36秒内，50Hz数据精确20ms、500Hz数据精确2ms，所有10条完整源流与已闭合bag的有序源stamp和payload一致。每回合640个正式命令加1个terminal zero与driver源时间接收逐一因果对应，实际接收延迟0–2ms；32–36秒失联段符合原门，尾包达到实际暂停服务水位，未按formal end截断原raw。

| 回合 | 2ms描述性峰(rad/s) | 实际暂停水位(ns) |
|---|---:|---:|
| m12_physics_p07_s023_kd4 | 0.915175967481 | 37144000000 |
| m12_physics_p07_s023_kd6 | 0.855810774468 | 37146000000 |
| m12_physics_p07_s024_kd6 | 0.845846660741 | 37136000000 |
| m12_physics_p07_s024_kd4 | 0.963452965207 | 37138000000 |
| m12_physics_p07_s025_kd4 | 0.963452965207 | 37138000000 |
| m12_physics_p07_s025_kd6 | 0.845846660741 | 37138000000 |

20ms资格不能外推为每physicsstep的连续速率约束：Kd6 seed023的2ms峰0.855810774468仍高于0.85，已保留。非地接触和非车轮地面接触计数均0，只表示这些完整源消息中未见相应接触。

种子024/025每个arm的归一化源时间转向、实际/target速度、50Hz和500Hz模型轨迹逐行完全相同。它们是固定几何、无噪声/无抖动设置下的重复执行资格单位，不是独立几何或一般性能统计样本。比较详见 `deterministic_pair_trace_comparison.json`。

PHY05按结果前冻结的三control峰中位规则选择seed024，峰并列按seed，不受treatment影响。所有六回合峰和全部移动相位稳态误差点均呈现。最终r03为170×220mm PDF/SVG及原生600dpi PNG，中文图注/LaTex/源数据CSV/全部SHA/实际view_image质量检查齐全。原PHY01–04均保持not_complete。

结果审计SHA：`8ce74b527fd15e69ebd9587c10233bce715949f21eae8ace363587dfe59f3380`。
PHY05 PDF：`/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/visuals/plots/physical_qualification/PHY05/r03/PHY05.pdf`，SHA `75a72d862c10e35609f040bd1eae952b732c1ad40e567492db6e295c688ced24`。

审计初次只读路径猜测和finalizer的manifest后缀错误记录于tool_errors.log，并保留原脚本/报告/log；仅修正审计代码，所有冻结运行证据不变。图r01缺字与r02标记压门线及修订产物完整保留。未启动ROS/Gazebo、未改变运行进程、未改全局figure manifest/plan。
