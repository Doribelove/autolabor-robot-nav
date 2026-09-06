# BPU 持续预览修复（2026-09-05）

2026-09-06 后续：模型常驻和吞吐优化已完成，默认上限由 0.5 Hz 提升为 30 Hz；
持续模式、隐藏停算和会话保护保留。最新实测、部署与停止状态以
[FCOS 性能记录](FCOS_PERFORMANCE_20260906.md) 为准。下文频率/进程/逐帧落盘描述为旧版历史。

23:35 后续：`qt_20260905_231959_508155` 桥进程以 137 退出，没有执行正常清理；
状态文件遗留 active=true，但实际 PID 已不存在。最后实际结果为第 40 帧，往返 1243 ms，
没有推理异常记录，也未找到相应内存耗尽日志，强制结束来源尚不能确定。
当前已用已有 tmux 建立独立后台会话，避免依赖临时执行终端；新 BPU 会话
`qt_20260905_233426_560948` 正在更新。不改系统服务、导航或安全门。
私有 socket 为候选 `runtime/run/bpu_preview.tmux.sock`，session 名 `j6m-bpu-preview`。
仍可用 `start_qt_bridge.sh --stop` 只停止本会话；主导航/Qt 消失时桥自行结束。

22:58 接续：用户随后要求带静态地图冷重启 Qt。旧持续会话已按请求停止，新的持续桥已
恢复，当前会话和实物检查见 [重启记录](STATIC_QT_RESTART_20260905.md)。下方保留修复试验数据。

## 原因与本次范围

用户报告综合页 BPU 画面卡住，随后明确要求“不要演示10分钟，要一直运行”。
原会话 `qt_20260905_220615_867705` 的结束记录为：293 帧，600.913 秒，
`reason=time_limit`。不是该会话 BPU 推理报错，而是此前演示程序的十分钟退出逻辑。
板端 worker 另有 660 秒截止，必须一起处理，不能只延长界面端。

本次只更新候选版的实验桥和 J6M 私有实验 worker；不重启导航、不改 Qt 二进制、
正式检测后端、运动门、系统服务、依赖或驱动。旧会话日志保留。

## 修改

- 导航伴随入口无参数默认 `--continuous`；两端持续模式不设运行时长上限。
  显式 `--seconds 600` 仍是限时测试，不用于持续显示。
- 持续模式只允许已存在的候选导航 master；每五秒核对 master PID 与 Qt 注册身份。
  主会话关闭、替换或通信检查失败时退出本桥，不自动启动导航或无限重试异常。
- 保留 0.5 Hz 上限、一帧在途、最新帧、时间/身份校验、EOF 关闭、8 秒传输/推理超时、
  板端 10 秒无请求退出、独占锁、单线程/低优先级和 512 MiB 磁盘余量检查。
- 离开综合/BPU 两页或最小化时仍停止实验推理；恢复时仅重建本会话 worker。
  持续会话重复使用自己的临时 tensor 目录；启动率限制为十分钟内最多 30 次，
  不再把显示切换累计到固定 30 次后永久退出。
- 逐帧结果、SSH 错误和板端推理日志各自轮转：每份 4 MiB，保留当前及两份备份。
  只覆盖该新会话自己的旧日志备份，未删除历史试验日志。最新原图/框图仍只保留一份，
  每十个结果更新一次。控制台每 30 帧报告一次，避免逐帧刷屏。
- 新增 `--stop`：验证私有会话路径后写其 STOP 标记，不按旧 PID 杀进程。
  本桥退出会先结束自己创建的 SSH worker，再关闭其日志。

修改文件位于 `experiments/j6m_bpu_20260905/tools/`：
`qt_bpu_bridge.py`、`bpu_preview_worker.py`、`start_qt_bridge.sh`；
新增 `preview_lifecycle.py`、`test_preview_lifecycle.py`。
修改前副本在本机实验 `continuous_before_20260905/` 及 J6M 同名私有实验子目录。

J6M 两份部署文件 SHA256 已与本机一致：

- worker：`e372052e8596482e7f0ecc826711d195c4f07fcc736b42f8ecce10afed0612dc`
- lifecycle：`8a277f974dfa9165ebf5f7217662e9c6a01d02426be3e49fb3f215b534789e6c`

## 运行和验证记录

22:29:33 启动持续会话 `qt_20260905_222933_179060`，本机桥 PID=121784，
J6M worker PID=1514251；握手 `continuous=true`，状态文件 `duration_seconds=null`。
原导航服务和 Qt 没有重启，仍为 MainPID=4193151 / Qt PID=7510。
原版 inactive，两个运动门 false，没有发出任何导航目标或运动指令。

- 35 项实验单元测试通过，包含持续模式跨 600/660 秒边界、UTF-8 日志轮转限额、
  超大日志/软链接拒绝、SSH stderr 排空、master/Qt 身份和受限停止标记测试。
- shell 语法检查通过，两端 CLI 参数检查通过；没有改 C++，不需要重新构建 Qt。
- 实际综合页已经恢复框图，显示“持续预览”。
- 22:29 初次 15 秒检查 faults=[]，743 条最终速度命令、左右 637/625 次轮速反馈均为零。
- 22:41 实测通过旧截止：同一会话运行 696.32 秒、340 帧，generation=1，
  同一 J6M worker 运行 693 秒，均没有重启；状态仍 active/continuous=true，
  两端 600/660 秒边界已实际越过，综合页仍收到新框图。
- 340 帧往返均值 1227.41 ms、最大 1269.18 ms；发布时源帧年龄均值 1347.78 ms，
  全部结果保持 demo_only=true / motion_eligible=false。这不是单独 BPU 算子时间。
- 22:40 起跨截止窗口 55.15 秒检查 faults=[]：2708 条最终命令与 1101 条安全速度全零，
  左/右轮 2350/2294 次反馈全零；RGB/深度均约 14.89 Hz。
  定位仍等待现场初始位姿，未验收运动导航。

22:31:58 与 22:41:07 的进程 RSS（KiB）对照：桥 85104 → 85296，
Qt 204968 → 204968，板端 worker 61440 → 61440，短窗口无明显增长。
这不包括 `hrt_model_exec` 子进程峰值，不是长期无泄漏证明或 GPU 迁移 A/B。
J6M 后两次 vmstat 样本 CPU idle=88%/86%，没有 swap；日志及 scratch 在会话目录内。

交付时持续会话仍运行；不会因本轮检查结束或十分钟到点退出。

相机朝向和局部里程计相较 22:06 的记录发生变化，22:20 与 22:29 的被动窗口均为
零速且局部位姿小幅变化；已向用户询问是否人工挪动/转向。不得将这些有限窗口说成
机器人从始至终没有移动，也不猜测位姿或放行运动。
22:36 用户回复“是有变化”，已记录现场变化；这不构成新的运动测试授权。

证据：

- `validation/bpu_continuous_unit_tests_20260905.log`
- `validation/bpu_timeout_stationary_20260905.json`
- `validation/bpu_continuous_initial_stationary_20260905.json`
- `validation/bpu_continuous_graph_20260905.txt`
- `validation/bpu_continuous_resources_initial_20260905.txt`
- `validation/bpu_continuous_resources_after_20260905.txt`
- `validation/bpu_continuous_boundary_stationary_20260905.json`
- `validation/bpu_continuous_after_limit_status_20260905.json`
- `validation/bpu_continuous_measurement_20260905.json`
- [持续预览初始截图](validation/bpu_continuous_initial_20260905.png)
- [超过原截止后的实物截图](validation/bpu_continuous_after_limit_20260905.png)
- 实验 `live/qt_20260905_222933_179060/`，以及 J6M 相同会话名目录。

## 正确启动和停止

完整候选导航先按原安全静止流程启动并通过检查；随后在 NVIDIA 的另一个终端：

```bash
cd /home/slam/robot_j6m_ws_optimized_20260905
bash experiments/j6m_bpu_20260905/tools/start_qt_bridge.sh
```

不再加 `--seconds 600`。保持启动终端打开；持续指没有演示时限，不表示已安装后台系统服务。
它不会自动随下一次导航启动，需要上述伴随命令。当前已启动，不要重复启动抢占。
结束预览可在其终端 Ctrl-C，或另一个终端执行：

```bash
bash experiments/j6m_bpu_20260905/tools/start_qt_bridge.sh --stop
```

单独停止 BPU 不会停止导航/相机；Qt 保留旧图并标记停止/过期。
结束完整系统时使用 `./scripts/optimized.sh stop </dev/null`，主会话消失后持续桥也退出。
原项目完整性和切回原版的方法仍见 `FULL_NAV_BPU_OVERVIEW_20260905.md`；不得并行控制底盘。

## 限制

持续显示不等于实时视频：当前仍每两秒最多一帧，逐帧厂商 CLI 加载、dump 和解码尚未替换。
FCOS 仍仅作通用物体显示，原 YOLO/五材质正式后端仍在 NVIDIA，不能据此声称 GPU 已迁走。
尚未完成多小时/多天运行、实机通信中断故障注入、持续模式反复隐藏恢复、定位稳定性、
深度精度、模型替代精度和实车运动导航验收。所有既有安全门保持不变。
