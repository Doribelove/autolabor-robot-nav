# Astra 历史证据独立审计（2026-09-26）

这是新阶段的只读离线诊断，历史源码、release、报告、种子和结果均未修改，也未启动 ROS、Gazebo 或训练。任务书、根 AGENTS、最新 PCT 交接、论文 claims/canonical/汇总 CSV、benchmark progress 全文至 P14、decision_log、两轮 assessment/review/preregistration 和 final_transfer 已核对。旧 progress 顶部 P03 与旧 V6 “尚无 Gazebo formal”属于未同步的历史叙述；当前 R1/R2 完成且阴性的证据优先。

## 实际执行与统计复核

执行命令（退出码 0，约 65.87 秒）：

```bash
python3 artifacts/astra_gazebo_20260926/audits/historical/recompute_historical.py --workspace /home/robot/robot_ws_base_rl --output /home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/audits/historical
```

脚本独立实现统计和二维几何，没有 import 历史 assessor，没有重写旧结果。stdout/stderr 均保存。`audit.json` 保存脚本哈希、Python/Numpy/PyYAML 版本及结果；`raw_replay.csv` 保存每一身份核查。核查内容：

- R1 640/640、R2 800/800 的 report 与 receipt SHA 均匹配 CSV 和 receipt；每身份 attempt=1，唯一性成立。
- 从原始 report 重判 contact→负保守净空→执行完整性→goal→timeout 优先级，逐行复算成功实际时间、失败 22 s，全部匹配。
- 四组 200,000 次配对 block bootstrap 使用预注册随机流，以独立分批实现复算；点估计和 97.5% 区间与原 assessment 在 1e-12 内一致。
- 没有合并 R1/R2，没有复跑或重新释放任何旧正式身份。

| 研究 | 方法 | 有效到达 | 失败计入均时 s |
|---|---|---:|---:|
| R1 | PCT | 153/160 | 15.0634375 |
| R1 | 解析 | 146/160 | 16.1231375 |
| R1 | TEB | 89/160 | 14.9225375 |
| R1 | DWA | 80/160 | 16.0881375 |
| R2 | PCT | 180/200 | 14.8412500 |
| R2 | 解析 | 150/200 | 16.3987600 |
| R2 | TEB | 119/200 | 14.4470200 |
| R2 | DWA | 100/200 | 16.0925100 |

R2 相对 TEB 的 -2.72879805%（97.5% CI [-9.80825518%, 3.59142453%]）和相对 DWA 的 7.77541850%（[2.55210385%, 12.61477161%]）保留阴性结论。R1 独立结果也阴性。历史 completion 非劣门只检验点差（`assess_formal_r1.py:147–158`），新任务要求区间下界，不能复制旧点门冒充新协议。

## 两次保守净空失败：物理含义与方向预测失配

| seed / 任务 | 旧保守圆净空 m | 独立 chassis–actor 最小投影间距 m | 原 contact | 解释 |
|---|---:|---:|---:|---|
| 198136 / sequential | -0.217291252 | +0.040508017 | 0 | 有裕量净空违例，二维 chassis 未相交 |
| 198146 / single crossing | -0.269571175 | -0.001360627 | 0 | 328 帧中 1 帧 chassis 投影相交约 1.36 mm；contact 需新阶段验证 |

原报告结果位置：

- `formal/round_2/execution/episodes/198136_core_sequential_interaction_pct_scope_rl/episode_report.yaml:19250`。
- `formal/round_2/execution/episodes/198146_core_single_crossing_pct_scope_rl/episode_report.yaml:12793`。

上面路径以 `artifacts/gazebo_pct_teb_dwa_benchmark/` 为前缀。两者原始 `goal_reached=true`、mask violation=0，失败由负保守净空触发，不能删去或当有效成功。

独立几何使用 URDF chassis 1.04×0.70 m、各帧 model_states 的 yaw、world actor 0.72×0.72 m 正方形；分离时算精确边—点欧氏最短距离，相交时用 SAT 最小平移深度。原 evaluator 用机器人圆半径 0.79 m + actor 0.36 m（combined=1.15），它不是物理长方形间距。旧圆指标逐帧复算误差最大 8.9e-16 m。机器人 body 的圆外接半径约 0.627 m，actor square 的外接半径约 0.509 m，actor 0.36 是内切半径；不能把一方 circle 半径的语义单独等同完整 collision geometry。两失败处 geometry CSV 同时保留旧圆/独立圆/物理投影值。

这里不能宣称完整 3D collision ground truth：旧日志没有 z、roll/pitch、全部关节姿态；chassis 相交是名义高度重叠下的二维警报，chassis 分离不排除转向轮/其他部件接触。原 contact sensor 只引用 chassis，`m2_gazebo.xacro:28–38`，须验证固定关节合并后的 collision 名称及消息覆盖。模型位姿采样也没有共同原子时间戳。结论是**contact=0 的检测含义未独立验证**，不是据此重写历史 contact 计数。

两次违例的机器人都先获准前进，然后在冲突通道附近停止，actor 随后穿过机器人保护圆：

- seed198146 在 t=10.202 s、ego x=-1.925 m 时 action3 获准，估计安全方向 [0.706,0.708]；真实 actor 仅沿 +y 运动。t=10.452 已全拒绝非停动作，实际制动至约 x=-0.833、y=0.310。actor 穿过时机器人几乎静止。
- seed198136 在 t=10.222 s、ego x=-2.398 m 时 action3 获准，方向 [0.701,0.713]；随后停止于 x=-1.933、y=0.115，actor1 固定 x=-1。

**强假设 H1：可见表面中心受 ego 运动/遮挡变化影响，拟合出伪 x 速度，方向性安全预测把 actor 预测到远离冲突的方向。** 源码 `benchmark_runtime.py:1087–1115` 用 dynamic surface median 加沿激光视线的 0.36 m 偏移估计中心，再做 8 帧直线回归；仅 residual≤0.25、span≥0.35 即可 valid，velocity≥0.05 就更新方向。`1149–1158` 把该方向乘最大速度1.35并平移所有动态点；这不是包含真实速度方向的已校准可达集。

`runtime_adapter.py:217–234` 无条件令 action0 的 mask 为 True，即使它自身 backup_clearance<0。因此“certified_stop”是动作名称，不证明当前停车必定安全。新实现应显式区分“停车动作可调用”与“停车包络当前通过几何校验”，并在尚可避免时保持可制动安全集。不能仅调小裕量以提高完成率。

可证伪测试：新 development 多角度/接近速度的同一真实 actor 轨迹，记录逐簇 association、方向置信区间、真实事后方向误差、候选 backup 点、执行轨迹与 stopping endpoint。改变中心估计/方向不确定集这一因素，要求真实轨迹包含率与零违例改善，同时报告是否新增停车。历史日志只能作归因，不能成为新 validation。

## 18 次 timeout 不是一种原因

`r2_pct_failures.csv` 保留全部 20 个 PCT 失败；18 timeout 为 7 static + 11 sequential。

1. 7 个 static timeout：seeds198007/049/061/067/169/193/199。全部最终只剩 stop；|y|=1.557–1.611 m，高于候选侧向中心1.35 m，最佳非停 backup 为0.127–0.192 m，低于0.20 m门限。若只调切换惩罚，未必能解除已落入不可行恢复状态的死锁。H2 是执行跟踪外偏和恢复候选支持不足，应检查实际转向动态与 preview 一致性，先改可行恢复机制而非放宽硬门。
2. 11 个 sequential timeout 中 7 个停在 x=-0.54…-0.34 m、actors 已到两端，最后 mask 仍只剩 stop；一个全局方向被施于另一相反运动 actor 的扫描点，是“离场后幽灵冲突”的具体假设。H3 为 per-cluster association/occupancy，不与 H1 混成一个不可消融的大改动。
3. 其余4个 sequential：seed198028 最终 x=4.7469、y=.0041，距0.25m终点球边界约3mm；198100/118/148 仍有0.85–1.32m侧偏。应区分耗尽时间与终点恢复失败，禁止简单加长22s旧合同。

分层重复计数：head-on 平均 stop24.9697、switch5.6061；single 平均stop15.5294、switch14；sequential 平均stop38.6667、switch23.2424、recovery9。这里 stop 是每0.25s决策选择stop的次数，**不是互相独立的停车事件数**；论文图表须用准确标签，新阶段另统计停车段数量/时长。

## LaserScan TF 完整性失败

21 个 R2 analytic integrity failures 全部 `scan_timestamp_transform_success_rate<0.999`，总24个 scan transform拒绝；最新 TF 成功率均1、scan/odom/TF age 均在0.20s以内。`r2_tf_failures.csv` 逐身份保留。

真实日志证据：

- seed198009 head_on `roslaunch.log:14`：请求10.948s，TF最新10.940s，future extrapolation 8ms。
- seed198018 width_change `roslaunch.log:15–16`：20ms和4ms future extrapolation。

这不等于传感器长期失联。回调 `pct_teb_dwa_benchmark_episode.py:171–203` 在 scan 回调中同步lookup(0.03s)，失败便永久加计；`648–649` 在warmup后复位计数，`684–690` 任一失败足以不满足99.9%。建议 H4 用时间戳感知的有界待处理队列，TF实际可用后消耗同一scan，在固定最大age内等待，超时丢弃并停机；增加消息到达/TF就绪时刻记录，验证排序竞争与时钟重置，而非放宽99.9%门或接受latest TF代替精确时刻。

## 随机化调用链

- `run_development_matrix.py:125–126` 确实把 seed项经`seed*7 %7`消去。actor0恒0.84 m/s，actor1恒0.915 m/s。
- `run_frozen_matrix.py:127–129` 使用同一 renderer，故影响 formal，不仅 development。
- `v3_reactive_actor.h:112–117` 又把seed response jitter乘0；world几何、actor政策/起终点/触发参数固定。
- 对800个 R2 scene manifest去掉seed/split后做 canonical fingerprint：每任务族恰好1个值，具体速度见`r2_scene_fingerprints.csv`和`audit.json`。
- 仍存在真实随机来源：`spawn_m2.launch:34` 把seed传给Gazebo，`m2_gazebo.xacro:18`激光高斯噪声stddev=.01。ROS异步时序也会导致轨迹变异。不能宣称“全部仿真无随机性”；可准确说是**每族单一几何/actor参数，仅仿真噪声/调度等随机重复**。
- 新阶段应独立派生layout/actor/sensor/dynamics随机流，存每个实际采样字段、去种子指纹、几何距离和跨分割近重复审计；仅修正一个取模表达式不满足多布局要求。

## 运动学、物理与评价执行链

- wheelbase全链0.65 m。共享YAML `38–50`标steer .40、rate .75；实际候选runtime默认.48/.80 (`benchmark_runtime.py:434–439`)，同YAML方法段`106–108`已承认.48/.80和执行.4964，故是合同层级/实际动态不一致，需要实测而非把不同数字本身当笔误。
- 仿真steer最大.4964 (`m2.urdf.xacro:31`)，launch生效rate .75、加速.60、减速.90 (`m2_pct_teb_dwa_benchmark_episode.launch:18–24`)。终点controller又以.40限制曲率 (`benchmark_runtime.py:753`)。候选与实际须统一明确的可执行包络并记录实际饱和/跟踪差异。
- TEB：turn radius1.20、wheelbase.65、max yaw .7805，`cmd_angle_instead_rotvel=false`；DWA非全向但按Twist搜索。实际两者经 `/cmd_vel_navigation` relay→`/cmd_vel`→`twistToSteering` (`m2_ackermann_plugin.cpp:144–148`，`ackermann_kinematics.h:23–29`)；v≈0时steer=0，因此DWA没有真实原地旋转能力，但其预测弧线可能被Ackermann限幅改变。
- **物理边界：** `m2_ackermann_plugin.cpp:280–286` 将内部速度/航向积分后每次`SetWorldPose`并清空模型线/角速度；odom/TF在`309–325`发布同一内部状态。它是Gazebo内执行器滤波运动学驱动。Gazebo world/physics/传感器是真实进程，但摩擦/接触不能自然决定内部运动，不能据此宣称摩擦压力测试或实车制动已验证。新版本须单列该模型边界，若要物理驱动则独立版本和校准。
- 新阶段评价需用真实sim model/link pose，记录与policy odom/TF之间的误差，原始命令、最终仲裁命令与实际关节/速度分开。历史报告只记录部分二维pose/速度，不能事后补造命令或物理参数。
- `pct_teb_dwa_benchmark_episode.py:523–527` 把异步odom速度差除固定.05计算accel/jerk，部分日志max acceleration=1.08恰为.90×.06/.05；这是采样时间解释风险，不能直接说执行器违反.90。新记录应绑定odom时间差和去重。

## 建议的新阶段优先顺序

先用新身份四方法smoke建立环境、消息、真实执行/评价闭环；同时固定评价几何、contact检测故障注入和物理模型声明。之后分别验证 H1 方向/中心不确定性、H2 静态跟踪恢复、H3 多簇关联/反向actor预测、H4 scan时序。动态切换滞回与停车释放独立消融，始终每周期安全重验证。待这些瓶颈稳定后再投入大规模重训和分组多布局validation；不能把旧正式日志做成新验证集。

独立审计未声称已完成新算法、安全验证、Gazebo新实验或任务书整体验收。所有强假设都仍须新的development与独立validation证伪。
