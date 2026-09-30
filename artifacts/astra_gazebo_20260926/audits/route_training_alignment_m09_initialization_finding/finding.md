# Fresh actor 残差门初始化死区

**已确认执行探索支持为零，不能按当前 M09 草案直接开始训练。** 这只是新的补充 finding，原 M09 草案及冻结 M08 源码未改。`probe.py` 调用实际 `SensorActor` 和 `choose_actions`，CPU 单线程、64 个固定 synthetic schema 输入、无 optimizer/backward/GPU/ROS/仿真。没有 warmload 或新 checkpoint。

`asymmetric_ppo.py:113–114` 把 residual head 最后一层权重和偏置置零，因此任何合法输入的八个 residual 均为零。`:189–193` 要求 sampled latent 与 analytic 不同且 residual 差≥0.10，才能作为最终 proposal。因此初始状态下所有 latent 都映射为 analytic；不只是这次样本偶然未探索。

| 输入族（各 16 个） | sampled latent≠analytic | 经过 0.10 gate 后不同 | 同一 RNG、gate=None 后不同 |
|---|---:|---:|---:|
| 8 个有效动作，梯度 prior | 13 | 0 | 13 |
| 4 个有效动作 | 8 | 0 | 8 |
| 8 个有效动作，prior 相同 | 12 | 0 | 12 |
| 只有 1 个有效动作 | 0 | 0 | 0 |

枚举全部八种 latent 的实际 gate 映射后，所有 64 行最终 analytic 概率为 1（浮点和容差 2e−7）。三个多动作输入的 latent 熵仍为 1.7343、1.0616、2.0794 nats，说明 latent 熵高不能证明物理探索发生。gate=0 在零初始化时也让全部 sampled latent 通过，但这只是边界对照，不是采用建议或新运行阈值。

此时只改成“正确记录 latent log-prob”能避免标签错误，却不能产生执行反事实。奖励在这个 gate 分区内不依赖 sampled latent；理想条件下其 score-function 梯度对动作区分的期望为零。有限样本噪声、熵项或其他辅助损失仍可能让参数漂移跨门，因此不宣称它在数学上永远不可训练；它没有可依赖的初始执行探索支持。若候选 hard mask 本身没有移动支持，即使绕开残差门也无法修复，须先完成静态 selector/支持诊断。

建议后继设计单独预注册一个**明确的训练探索分支**，而非改标签或悄悄放宽部署门。一个可直接实现、待 root 取舍的有限定义如下：

- actor 概率为 hard-mask 内的 πθ(z|o)，原 0.10 门为 Gθ(o,z)。训练固定 ε=0.20：以 ε 走 exploration 分支、proposal=z；以 1−ε 走原 gate 分支、proposal=Gθ(o,z)。所有 proposal 仍须真实 hard mask 与每 20 Hz backup/总 deadline 检查，ε 不 bypass verifier。
- 训练 proposal 分布显式为 `qθ(a|o)=ε·πθ(a|o)+(1−ε)·Σ[z:Gθ(o,z)=a]πθ(z|o)`。它提供每个有效动作至少 ε·πθ(a|o) 的概率。ε=0.20 是**待审设计值**，本 finding 没有修改运行配置或授予训练预算。
- 记录 `branch`、原 `latent_z`/`log_pi_z`、`G(z)`、最终 `proposal_a`/`log_q_a`、hard mask/门映射、实际 executed command/屏蔽原因。若采用 proposal-space PPO，ratio 必须使用 old/new **q**，不是把 a 当作 latent 填进 π；保留真实 z。重新求 q 时用同一已冻结 observation 与该策略版本的真实 gate 分区，边界不光滑须有专门测试。
- 过期/后验拒绝 proposal 的数据保持失败账本，不能冒充有效 actor 执行；训练 episode 的奖励/时间归属与实执行 receipt 对齐。all-false mask 不调用 categorical、不强塞 stop-valid。
- 部署和 validation 仍采用确定性、原 0.10 gate、ε=0；checkpoint metadata 分别声明 training behavior 与 deployment policy。这个训练/部署差异必须用成对 admission 数据测量；没有已证明收益，不从这次 synthetic fixture 选择 ε。

在实现前必须新增门控分布的质量守恒/掩码/初始支持/边界/准确概率比测试，以及 fresh 无 warmstart actor 的 execution-support admission（实际候选与 Gazebo 数据，记录每动作被执行的支持，不能拿 latent counts 代替）。训练特权仍仅进 critic，不使用 oracle 挑探索动作。现有五训练 seed 草案、N04 与 final 分割均不变；本 finding 没有训练或仿真授权。
