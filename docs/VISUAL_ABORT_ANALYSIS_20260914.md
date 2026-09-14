# 2026-09-14 视觉中止停车诊断

本轮仅只读查询和保存诊断证据，没有调用启停/运动服务、发送目标、修改业务源码、参数或授权，也没有构建、部署、提交或推送。工作分支仍为 `0912`，保留原未提交修改。

## 结论

本次视觉会话在 ACQUIRE（初始目标获取）停留约75秒，因绝对总时限中止，没有进入 APPROACH、消失确认或0.30米尾段。当前可见目标被仍保留的初始图像纵向捕获门拒绝，这是复现出的获取阻塞原因；故障时75秒内的逐帧拒绝原因未保存，不能把当前复现冒充故障全程记录。

“视觉中止停车”对应 `FOD_ABORTED`：控制器 ABORT 后，仲裁器保持零速、暂停导航并禁止新 move_base 目标，只有显式退出/返回才离开该状态。它不是清扫完成状态，也不是进程崩溃。

## 日志时间线

以下均为北京时间；J6M Python 日志时间为 UTC，已加8小时。

| 时间 | 证据 |
| --- | --- |
| 07:41 | 当前导航/Qt完整会话启动，静态地图为 `map_20260914_064202` |
| 08:04:54.854 | 覆盖最后一段（12）完成出口判定；当前覆盖状态 COMPLETED、12/12、active=false |
| 08:07:18 | Qt 操作日志记录检查最近FOD距离，准备切入视觉 |
| 08:07:19.420 | 视觉启用请求，建立会话 `fd442170a9ad45d58cfdb556f6538067` |
| 08:07:19.547 | 预检查通过，进入 ACQUIRE |
| 08:08:34.449 | `FOD visual servo ABORT: visual-servo mode reached its absolute time limit` |

启用至ABORT共75.029秒，实测运行参数 `mode_timeout_sec=75.0`。期间视觉节点日志没有任何进入 APPROACH 的状态迁移；终态 `target_locked=false`、`approach_path_m=0`、`blind_forward_m=0`、`completed=false`，会话完成序号0。没有本次成功回执。不能用缺少历史轮速记录声称全程底盘绝无外部移动。

原始日志：

- `validation/visual_abort_20260914_0818/fod_visual_servo-16.log` 第39–41行。
- `validation/visual_abort_20260914_0818/gui.log` 末两行。
- `validation/visual_abort_20260914_0818/coverage_manager-13.log` 第127行。

## 为什么有框却无法锁定

运行配置仍要求 `min_acquire_anchor_v_fraction=0.20`、`max_acquire_anchor_v_fraction=0.80`，计算式为检测框底部锚点 `anchor_v/(image_height-1)`。图像640×360，因此允许的纵向锚点约71.8～287.2像素。模式入口的≤8米距离检查通过，不代表视觉初始捕获条件通过。

08:18初次单帧查询看到 paper，置信度约0.83、深度约1.67米、锚点348像素，纵向比例约0.969，超过0.80。

08:19:19.782–08:19:29.971再次被动采集，并将收到的检测交给当前源码的 `_acquisition_candidates()` 纯计算回放。使用 `__new__` 构造仅含数据的对象，没有初始化控制器、发布器或控制服务：

- 82帧全部先通过类别、置信度及有效深度筛选。
- 深度1.734～1.741米，置信度0.860～0.876。
- 82帧的纵向比例均为0.952646。
- 82/82均被拒绝，0帧通过；原因均为 `target vertical fraction 0.953 is outside capture range 0.200..0.800`。

这是故障后的当前数据复现，不是08:07–08:08的历史检测回放。两次查询目标深度和框位置有变化，不假定现场目标从未移动或检测身份始终相同。

此前0912同步取消的是锁定后的底部/居中完成门，初始捕获区间仍保留；`docs/VISUAL_SERVO_SYNC_0912.md` 已记录此前出现过 q=1.0 被拒绝且未修。此次并非旧5cm/20cm尾段停车门再次触发。

## 当前状态及版本

08:19的10秒采样中：视觉 ABORT/active=false；仲裁 FOD_ABORTED/STOP；覆盖 COMPLETED/12/12/active=false，paused=true；待清扫 IDLE，AI IDLE/task_active=false。覆盖的paused标记与视觉仲裁保持暂停同时存在，不应据此解释为覆盖任务尚未完成。

定位采样为 LOCALIZED；底盘四类急停为false。487份 `/cmd_vel`、201份 `/cmd_vel_safe`、201份 `/cmd_vel_fod` 及101份 `/odom` 中线速x与角速z全部为0。该结论限于采样窗口。

导航服务正在运行。J6M实际 current 为 `/opt/autolabor/dual_host/releases/20260912_165656/install`；该安装版本的视觉控制节点及 `visual_servo.yaml` 与本地 SHA256 完全一致，分别为：

```text
7950d3bec1c6499b6d5f0346be2f6d68ab699e0955da0ed83fca643f93eef461
81b1ce9e81b2e87793b66eb423802486858674ba51f55a1795fd28e26e30ccda
```

## 后续处理建议

应处理初始捕获门与近处目标的适配，并让模式接管前的准入检查与控制器实际锁定条件一致。单纯延长75秒时限不会让处于画面底部的目标通过锁定。应同时持久化获取阶段的具体拒绝原因、累计帧数，并在超时原因中保留最后阻塞原因，避免日志仅剩笼统总超时。

当前界面提供“退出视觉模式并恢复相对导航”入口；本轮没有替操作员执行。上述源码/参数修复尚未实施，未做运动复测。

完整采样与逐帧回放结果位于 `validation/visual_abort_20260914_0818/snapshot.json`、`acquisition_replay.json`，只读采样脚本为同目录 `observe.py`。采样工具运行成功，`git diff --check`通过；无业务变更，无需构建测试。
