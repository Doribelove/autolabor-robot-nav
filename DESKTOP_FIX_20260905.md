# ToDesk 操作桌面侧栏无响应 — 2026-09-05

## 处理结果

17:55 复测：Ubuntu 应用侧栏点击恢复。导航窗口已经恢复最大化，并在这个状态下再次
用真实 XTEST 鼠标点击打开了应用列表；GNOME 收到 ButtonPress，overview/apps 都变为 true。
验证后关闭应用总览，保留导航窗口和后台运行。需要用户在 ToDesk 客户端再确认实际操作体验。

只重新执行了当前 X11 会话的 GNOME 桌面外壳，没有注销、重启主机、Xorg、ToDesk 服务或导航。
没有修改系统配置文件、驱动、依赖、桌面刷新率、机器人算法或运动安全参数。
Ubuntu Dock 的持久设置前后完全一致。原项目 1287 项文件/链接再次核验未改变。

## 诊断证据与边界

- 故障时 CPU 未满载：10 秒窗口全机 CPU 约 34.4%，可用内存约 44571 MiB，swap 为 0。
  X11 往返中位数 0.122 ms、P95 3.002 ms；没有证据支持“内存耗尽导致整机卡死”。
- GPU 的 15 个样本均值 72.27%、峰值 91%。临时最小化导航 GUI 后另一个 15 样本窗口
  均值降为 49.67%，但侧栏点击仍失败。这只是显示负载对照，不能把 GPU 占用直接认定为
  点击失效的原因，也不是永久性能优化。
- XTEST 鼠标移动正常，按钮映射正确，没有其他客户端持续独占鼠标。
  同一输入机制创建的临时普通 X11 窗口能够收到 ButtonPress/Release；
  Ubuntu 侧栏却没有收到 GNOME captured-event，应用列表不打开。
- 更新输入区域、只在内存中重载 Ubuntu Dock 均未恢复点击；重新执行 GNOME 外壳后，
  原测试立即成功。恢复最大化导航窗口后复测仍成功。
- 结论限于本次故障出现在 GNOME 桌面输入处理链、重启外壳恢复；
  具体触发原因和长期复发条件尚未确定，未为此改导航源码或放宽任何保护。

## 机器人与连接保护

Xorg PID 3311、ToDesk_Service PID 2638699、ToDesk_Session PID 2946025、
导航监督器 PID 2709200、导航 GUI PID 2716998 均保留。
原版 supervisor 仍 inactive，候选 supervisor active；没有两套底盘控制系统并行。
候选两项运动放行保持 false，未发送导航目标或初始位姿。
17:55 额外 10 秒被动采样无数据连续性/零速故障，速度指令和左右轮反馈仍全部为零。
本轮没有进行物理移动、急停触发或完整导航验收。

临时输入测试窗口已销毁，GNOME 测试事件监听器已断开，导航窗口的临时最小化已撤销。
原始诊断记录保留在 `validation/desktop_*`：

- `desktop_lag_before_top.log`、`desktop_lag_before_tegrastats.log`
- `desktop_lag_gui_minimized_tegrastats.log`
- `desktop_dock_settings_before.txt`、`desktop_dock_settings_final.txt`
- `desktop_click_after_full_gui.log`
- `desktop_fix_robot_safety.log`、`desktop_fix_stationary_safety.json`

`desktop_lag_after_shell_*` 和早期 `desktop_lag_final_full_gui_*` 采样跨越了
应用总览/窗口恢复操作，不应作为严格的恢复前后 GPU 性能对比。
