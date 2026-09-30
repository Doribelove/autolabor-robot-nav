# P02 独立测量与命令链审查

结论：离线接口审查 26/26 项通过。未运行 ROS/Gazebo，W 与合同已冻结。

命令源原 28 项加独立 12 项合计 40 项通过。新增测试调用完整 runtime 并替换 rospy/time/transport：验证 paused bootstrap、所有 publisher 先 advertise、一次 Trigger 的 run_id/seed/SHA、loss 区间零发布、迟发跳 slot 不补发、时钟回退/停滞、提前 ROS shutdown，以及 complete/fault 常驻。100 组随机迟发轨迹未重复发布同 slot。

评估器旧176快照的有效 toy baseline 通过，同时保留13个 false-pass 反例。修补后的bcbe执行完整67项通过，涵盖源包、目标/实际分离、接触、实际slew失败、命令错误/边界/重复slot、完整状态序列、录包连接及fault。最终68dd仅增强warmup的整数/非负/前后顺序约束；机器AST证实其余代码一致，8项focused通过，其中合理start=1.1s、warmup/running=1.0s的最终完整assessor基线通过。没有把旧快照失败归到修补版。

独立用最终probe验证 exact runner offline launch 的 URDF/world，26个录包topic与参数完全对应，source和probe共用新身份、seed与合同SHA。命令collector没有Publisher；外部shutdown会失败，source最后running状态不能误作complete。原P01动作、全部数字门和物理控制参数保持相同。

限制：这些是离线软件资格，不是物理通过结果。P01实际转向峰值失败保留；P02仍须接受同一0.85rad/s门。command_event是Python发布前采样的时刻，Twist自身无header，不能等同driver接收原子时刻；实际relay观察和时限门继续单独判断。提前ROS shutdown时source返回0但不发补零，由probe强制完整状态和终态异常守住失败记账。

最终 probe SHA：`68dd16ef4bf5a8ac85bafc8996e52c48f3c145c6a9dc88f490aa3e2d2fbfbe00`。

最终 command source SHA：`82e80264c8eb62cc94b93a74f402047433efda0d7970e8d194b6c3ffeda4489e`。

合同 SHA：`0015576bb4d8a087e2fdbeb8919d7050b9561719050292db31d394e7404dc3c2`；runner SHA：`217bf464f5efd526540acc6b31dccce75a7b8ef884adca174e5664f3a2077800`。

完整来源与检查见 `m12_p02_independent_integration_review.json`；67项报告、8项focused、40项命令源证据分别保存在同目录，包含输入SHA与实际命令。所有失败历史保留。
