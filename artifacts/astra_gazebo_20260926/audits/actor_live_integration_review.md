# Gate-relative actor 与 live 派生输入

组件结果：`actor_relative_gate_and_live_input_components_pass_runtime_qualification_pending`。未启动ROS，旧V3 plugin/header、A02几何与manifest/world均未改。

新的共同环境版本为`astra_common_environment_v3_routegate_deterministic_actors_z0_evidence`。新增`astra_reactive_actor_plugin.cpp`调用`astra_actor_gate.h`，route SE2逆投影的x首次达到-4.02时锁定gate source time，actor等待各自gate_delay_s后移动。移除了原absolute start_delay与world-x激活组合，并在读取旧字段时显式拒绝。跨delay边界的一个physics step只积分激活后的部分，重复时间不积分；回退/非法时间或坐标使该实例永久停用，需要新episode。Load与等待阶段将actor固定于start，内部initial_speed保持原反应模型设定。

反应公式、三种policy、ego距离/closing-speed采样保持继承的确定性规则。active段直接调用旧`advanceV3ReactiveActor`，没有添加噪声、response_seed作用或新策略。C++组件通过route gate、delay、无preactivation进度、边界裁剪、SE2触发时序、clock fault及4500次旧规则逐字段精确等价；5个Python派生/SE2/输入保留测试通过。

参数映射先于派生输入写入`actor_live_parameter_mapping.json`。真实变化来自离线actor_parameters命名流：policy、速度、响应距离、响应裕量、delay；Gazebo episode --seed驱动实际scan RNG，0.01m噪声由现有传感器配置提供。response_seed虽传入但旧规则jitter乘0；聚合actor_response、独立sensor_noise与actuator命名流属于未用future字段，明确不作为执行阻断。command_jitter=0时没有随机actuator延迟。不得把此环境称为随机反应actor。

可供root新诊断runner绑定的输入表：`scenes/development_a02_live_v1/diagnostic_blocks.csv`，12输入/原6几何，gate delays覆盖0/.2/.4/.6/.8/1.0s。它继承尚未执行A02的参数与episode seeds，增加环境与variant身份，不是新增独立几何或新数据样本。25个父CSV/world/manifest散列验证保持不变；旧输入不覆盖。camera、source-stamped physics plugin与spawnz0继承V2修复。

Actor插件本身支持任意有限SE2 route投影，纯helper已测试世界start/end/conflict与初始姿态的复合变换；完整派生script仍拒绝非identity SE2，因planner/map/evaluator live桥接尚未完成。当前yaw0输入可对接root现有launch。

## 下一轮live检查

- root编译并绑定最终source/library SHA，确认每个actor加载新plugin。
- 通过source-stamped actor pose与日志中的model/gate_time/activation_time验证等待阶段零位移、delay后的动作。gate触发在WorldUpdateBegin，physics后pose由WorldUpdateEnd记录；应保留这个相位关系。
- actor阈值-4.02与控制器warmup阈值-4.03、记录的policy-start pose是分别测量的事件，不能宣称完全相同时间戳；每方法均使用相同环境协议。
- 支撑、转向包络、source-stamped contact/图像/rosbag及独立geometry evaluator资格属于root其他并行任务；此报告不替代其live门。

## 后续完整SE2 node桥接建议

`pct_teb_dwa_benchmark_episode.py`的`_scan`约194行与`robot_world_pose`约222行直接使用spawn_world_x+odom.x；应在新派生node内将odom pose从spawn frame转world，再逆route transform提供route-frame policy观察，不改冻结R2。`send_goal`约391行目前只发goal_world_x-spawn_world_x；需把world目标转换到move_base使用的odom frame，并设置正确yaw。所有候选轨迹、scan点、actor观测、路径绘图、global/local map/evaluator必须明确各自frame；body Twist速度/yaw-rate两个标量保持不变。与世界transform同时移动static/actor几何、spawn与camera覆盖区；每个变换只生成参数化变体，不能算独立parent几何。下一阶段在非零yaw/translation上分别做native TF/scan/path/goal实证。
