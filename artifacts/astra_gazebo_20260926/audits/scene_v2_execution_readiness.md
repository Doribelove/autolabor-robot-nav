# V2 场景离线资格审查

结果：`offline_v2_geometry_qualification_pass_live_integration_pending`。未启动 ROS，没有访问或生成 final outcomes。

- 新身份：development_a02，六类各一布局、每布局两个随机种子，共6个新几何/12个回合输入；完整保留development_a01的18布局/36输入和原生成器。与prior合并的几何、近重复、分割审计为零问题；90个新增分配种子，所有prior RNG登记原样保留。
- 6/6布局存在向前Ackermann静态路径见证，包括遮挡屏绕行。采用车身与四轮投影凸包、0.40rad转向上限、0.75rad/s转向速率、0.18s执行滞后和0.4m/s见证速度；物理间距0.33m另加0.03m离散步裕量。路径仅证明存在，不证明任何候选集覆盖、动态安全或22秒内完成。
- 10/10 actor完整0.72m方形扫掠路径距静态障碍≥0.05m，零穿屏；10/10在不响应的名义速度下出现≤0.33m保守冲突代理。真实响应时机仍需live资格验证。
- 遮挡布局的屏障到actor完整扫掠间距0.24165m；真实720束平面lidar离散角下，gate位置目标可见射线1/30=3.33%，揭露位置60/60=100%。此证明是同一actor起始姿态的两个静态传感器位置，不是动态突现效果证明；未模拟0.01m噪声及roll/pitch。
- 12/12 world/manifest/source散列、actor参数/姿态、camera姿态一致。相机z19/yawπ/2，地面视域约22.01×12.38m；spawn z0是新共同环境输入，需要真实轮地支撑验证。
- 新world接入libastra_evidence_plugin.so（world直下、50Hz），声明world Models/Links source stamps、每physics step完整contact、metadata接口。力/力矩未填，SetWorldPose模型速度应从pose差分算，不能把内部body twist为零当作静止。
- 6个组件测试通过，包括真实A01穿屏失败检出、整个actor扫掠、新版多射线遮挡、prior保留、碰撞allocator和renderer一致性。environment_audit完成独立只读renderer/lidar审查。

## 保留的离线输入生成失败

第一次SHA映射遇prior登记种子260970676，fail closed且输出尚未创建。新allocator以hash候选起点进行确定性first-free线性分配，保留每个占用身份，并记录跳过的slot；按冻结family/index/draw顺序，重复key映射稳定。第二次采样在过长actor起始距下未找到合格遮挡，保留诊断后调整未冻结的沿路径起距1.6–2.25m和静态揭露位置。屏障尺寸、screen到conflict范围、扫掠间距和可见比例判据均未放宽。第三次生成成功；未发生实验回合或实验seed重试。

## Live或新版本仍未覆盖

1. 新physics contact/source-stamp fixture、精确时戳图像与bag关联，以及z0下轮胎/车身支撑。
2. actor gate delay目前只是manifest参数，继承plugin仍absolute start_delay5.0/world_x -4.02触发；不能声称已经gate-relative执行。
3. actor response_seed虽传入plugin，但冻结header以0乘jitter；sensor/actuator命名流也未接live。真实参数采样多样性不等于这些随机流已生效。
4. SE2 route/world helper只有纯函数测试；当前全部yaw0，planner/map/evaluator/actor激活的live变换桥接未实现。
5. 真实ego/actor时间轨迹下遮挡揭露与全部方法行为；静态见证忽略动态actor。
6. 本资格集仅每族一个已实施topology，不替代每族24布局×3seed development；其他topology/stress/OOD仍待完成。

机器证据：scene_v2_execution_readiness.json、scene_witnesses_v2/、scene_v2_render_independent_review.json、scene_v2_generation_attempts.json，以及scenes/development_a02/generator_review.json。
