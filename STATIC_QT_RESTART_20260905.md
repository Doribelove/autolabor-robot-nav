# 带静态地图冷重启 Qt（2026-09-05）

## 23:21 最新重启

用户再次明确要求关闭当前 Qt 后带静态地图重启，已完成统一冷停、清理和冷启动。
地图仍为 `map_20260829_221335_gimp_cleaned`；地图纹理 READY，新窗口已置前并目视确认
静态地图和持续 BPU 框图。未修改源码、导航 release 或原项目。

- 新服务 MainPID=339252，Qt PID=347147；原版 inactive。
- BPU 会话 `qt_20260905_231959_508155`，桥 PID=361402、J6M worker PID=1612119，
  持续模式，新结果递增。旧会话按请求停止，未混用两个 worker。
- 23:20:00 起 20.09 秒静止观察 faults=[]，989 条最终速度命令全零，安全速度和左右轮速
  同样全零。RGB/深度约 14.95 Hz，400 条定位状态均 WAITING_INITIAL_POSE。
- 两个运动门 false；没有发送目标或代填初始位姿。原项目 1287 项快照 unchanged=true。
- 启动时 TEB 参数服务事务同步警告仍保留，没有据此声称规划或移动导航已验收。

证据：[本轮重启日志](validation/static_qt_restart_20260905_2312.log)、
[静止检查](validation/static_qt_restart_stationary_20260905_2319.json)、
[实际地图与 BPU 截图](validation/static_qt_restart_visible_20260905_2320.png)。

## 22:58 上次重启记录

用户要求“现在带着静态地图重启qt”。Qt 是 required 节点，遵循现有生命周期完整冷重启
候选双机栈，没有单独强杀/热重启 Qt，没有改源码、系统配置或发布导航 release。

- 保留启动前实际地图：`map_20260829_221335_gimp_cleaned`，fused 模式。
- 先用伴随入口 `--stop` 结束旧 BPU：`qt_20260905_222933_179060`，
  最终 1205.28 秒/525 帧，reason=stop_requested。实际确认两端所属桥/worker 均退出。
- 统一冷停验证 CAN 无占用、所属进程清除后才冷启动；完整 runtime 检查返回 0。
- 新服务 MainPID=228378，Qt PID=236245；原版服务 inactive。
- 新 BPU 持续会话：`qt_20260905_225631_152804`，桥 PID=249731，
  J6M worker PID=1564792；continuous=true / duration_seconds=null，真实框图已恢复更新。
- Qt 地图纹理 READY：1311 × 670，0.1 m；实际截图已目视确认。
- 22:56:32 起 20.16 秒被动检查 faults=[]：985 条最终速度/401 条安全速度全零，
  左右轮 821/808 次反馈全零；RGB 和深度约 14.96 Hz，必需传感器持续更新。
- 402 条定位状态均为 WAITING_INITIAL_POSE。需现场操作员重新设置初始位姿，
  未发送目标、未复用旧位姿；MOTION_ENABLED=false、FOD_MOTION_ENABLED=false。
- 原项目 1287 个文件/配置/地图/软链接完整性核验 unchanged=true。

Qt 的启动规划参数事务同步超时警告仍存在（等待 TEB 动态参数服务）；未绕过或删除诊断。
此次确认的是重启、地图显示、真实 BPU 画面及静止链路，不是定位/规划/移动导航验收。
保持当前导航、Qt 和持续桥运行。后续按实际 PID/所有权重新检查，不使用这里的历史 PID 杀进程。

证据：

- [完整重启日志](validation/static_qt_restart_20260905_2249.log)
- [静止观察 JSON](validation/static_qt_restart_stationary_20260905_2256.json)
- [地图与 BPU 实际截图](validation/static_qt_restart_visible_20260905_2257.png)

BPU 不再十分钟退出，后续启动/停止方式见 [持续预览记录](CONTINUOUS_BPU_20260905.md)。
