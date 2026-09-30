# Q01 独立启动失败诊断

Q01（seed 260926004，attempt 1）为不可复跑的基础设施启动失败，未到达执行资格测试。原回执的 `completed/pass=false` 原样保留；roslaunch 返回 0，而 required probe 实际退出 3。

所有 model/link/contact/odom/joint 与计划指令计数均为 0，metadata 已收到。readiness 在暂停期间等待 `/cmd_vel` 连接，但 relay 收到首条输入才发布输出；Q01 的首条零输入来自超时后的 finally。原始日志第 41 行延后 advertise，与代码、30 秒异常和精确 AST 懒 relay fixture 一致。这里“计划指令 0”不代表 cleanup 未发布零命令。

先前离线评估覆盖了运动和数据故障，遗漏懒 relay 的启动依赖；本次明确记录这一审查缺口。Q02 仅预热零命令，动作与门限不变。所有源 SHA、原始计数、日志行号和可执行反事实 fixture 结果见同名 JSON。未改 Q01、未启动 ROS。
