# Actuation Q01 离线组件审查

状态：**离线组件与 runner 审查通过，仅适用于 fresh actuation_q01**。未启动 ROS/Gazebo，未消耗种子；合成 fixture 的 260926004 不代表任何 live 身份授权或消费。

探针仅发布 `/cmd_vel_navigation`，观察 relay 后 `/cmd_vel`。通过有效 `/robot_description` 校验轴距0.65、中心角0.4964、内部转向slew0.75、加速0.60和减速0.90；左右前关节角度和速度上限从有效 URDF 独立读取。真实前转向 link 与 base 的相对旋转来自同源时间 world_links，不冒用 joint_states 发布值。

36秒时序：静止2、加速4、正转3、反转3、正饱和3、负饱和3、零速非零yaw3、再加速4、制动3、再加速4、command loss4。空地100×100m、spawn全0、ODE 2ms、WorldUpdateEnd 50Hz；完整 SDF 在合同中冻结，探针核对结构一致性。

七项合成验收/故障注入通过：理想数据、非地接触、前轮超限、掉 truth 包、重复源stamp、丢命令阶段出现额外命令、非有限数据。附加检查确认错误加速参数、错误seed、转动地面均拒绝。首次读取尚不存在 physics_q01.yaml 的 discovery 错误保存在 JSON。

模型位姿20ms和100ms差分与odom内部加速度/jerk分开报告。实际中心slew额外0.10rad/s和前关节速度额外0.20rad/s是已冻结的采样诊断容差；内部限值仍用1e-6数值容差。正常地面接触单列，非地面机器人接触必须为0。无事后物理jerk门限，不宣称摩擦、碰撞动力学或实车资格。

运行接口：`~output`, `~contract_path`, `~world_path`, `~seed`, `~run_id`。必须使用root新身份、暂停启动、现有relay和已资格确认的physics evidence插件；探针 readiness 后 unpause，成功/失败均保存 `probe_raw.jsonl`、`probe_raw.json`、`probe_report.json`，用 monotonic wallcap，拒绝覆盖旧输出。root负责 launch/CMake安装、bag录制、receipt和cleanup。

所有有效参数、文件SHA、实际xacro命令及测试数值见同名 JSON。这份审查不替代 root 的独立整合审查。

Runner 最终复核：SHA `90aacb0037a319b2b14e63be1f1499465dfb2c64f10acbf35c6897e819e910c1`，20项启动/依赖/记账/收尾检查通过。root 已补充实际 `actuation_contract.yaml` 读取副本的 path+SHA 与归档；先前待修项关闭。physics_evidence_q01 的既有 receipt/probe 均 pass。`roslib.packages.find_node` 已确认可执行源码节点，未安装 devel 代理不阻断 roslaunch。不得将这些离线通过表述为 live 资格已通过。
