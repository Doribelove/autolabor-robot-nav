# 2026-09-08 验收交接

2026-09-07 收工状态；保存分支：`feature/navigation-optimization-20260907`。

## 已保存成果

- 候选 Qt/RViz 业务回调隔离、ROS 重连保留原生窗口，ZED 跟踪设置修复。
- BPU 运行路径迁入本候选独立根，保留当前伴随感知实现。
- 首入口只读 Navfn 预览、Qt 路径显示配置和批次失败后预览保留修复。
- 单元回归、隔离 ROS 场景验证工具、现场实验记录和重复入场问题复盘。

当前 J6M 候选 release：`20260907_214928`。管理器 SHA256：
`35dfd9f452b8062e32b420ae36b255ec24fe3f06c70dc6f662d892d20fba0606`。
首入口修复的 206 项针对性测试通过；15 项隔离 ROS 集成检查通过，其中包括旧版故障
复现、修复后连续三次预览保留、绕墙、取消、换图和新批次替换。仿真未发送动作目标。
Qt 更早的独立验证见 [Qt 修复记录](../QT_CRASH_FIX_20260907.md)，不要把该结果代替
此次首入口路径在真实 Qt 中的验收。

## 当前现场状态

用户已主动断开底盘供电，雷达因此无电。原版和候选完整服务均 inactive，仿真和等待
监控的进程已结束。主运动门和 FOD 门均 false，候选运动授权标记不存在。
没有可以沿用的新会话初始位姿；不得从旧日志复用初值。
保留停止状态，待操作员恢复供电并开始验收。

## 明日首先验收首入口预览

1. 恢复现场供电和连接，使用本候选统一入口加载原静态地图：

   ```bash
   ./scripts/optimized.sh start --map-set \
     /home/slam/robot_j6m_ws_navigation_20260907/global_maps/map_sets/map_20260829_221335_gimp_cleaned </dev/null
   ```

2. 在新 Qt 按车辆真实位置设置初值；确认定位 LOCALIZED、Qt 地图 READY，Navfn 预览
   服务已初始化。节点存活不代替这几项就绪检查。
3. 选“研究院大楼前大厅”区域创建覆盖批次。保持现有主运动门 false 时，执行预检
   应报告 `main motion gate is disabled`；**深蓝色首入口路径仍应保留**，从当前位姿
   连到第一条清扫线入口。失败状态是当前关闭运动门的预期结果。
4. 等待多次界面刷新，确认路径没有被清掉；取消任务后应清空，新批次应替换为自己的
   入口路径。若无路，应记录真正的规划失败原因，不能显示旧任务路线。
5. 记录 release、地图摘要、批次 ID、定位状态、入口路径首尾及完整时间线。

以上验证不需要启动车辆移动。真实 TEB 跟踪、首线终端调姿、Hybrid 转场 CPU 和
整车行驶仍待现场运行试验验收；不能从本次静态预览或仿真结果推断这些项目已通过。
此前 7 条 VCU ControlTimeout 日志根因未明，需要再次同步记录命令间隔和 CAN 反馈。

## 证据恢复与仿真复现

源代码与报告直接保存在分支。由于根目录 `validation/` 是忽略项，关键证据另外归档为
`docs/validation/20260907/checkpoint_evidence.tar.gz`，SHA256 清单为同目录
`checkpoint_evidence_manifest.json`。归档只含本候选的选定测试与实验数据，不含模型、
地图资产、构建目录、私有配置或运动授权标记。

本机原始证据保留在原位置。全新检出若需要恢复报告中的证据链接和旧版对照脚本，在
工作区根执行：

```bash
tar -xzf docs/validation/20260907/checkpoint_evidence.tar.gz -C .
bash src/application/autolabor_coverage/test/run_first_entry_simulation.sh
```

详细结果：[仿真报告](../FIRST_ENTRY_SIMULATION_20260907.md)、
[现场实验记录](../COVERAGE_CPU_EXPERIMENT_20260907.md)、
[重复入场问题复盘](ENTRY_ROUTE_RECURRING_FAILURE_AUDIT_20260907.md)。
