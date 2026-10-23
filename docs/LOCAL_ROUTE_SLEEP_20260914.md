# 09:35“局部路线休眠”诊断

“局部路线休眠”是Qt在仲裁状态FOD_ACTIVE时显示的固定提示（main_window.cpp:5283），表示视觉接管期间暂停普通导航，不表示局部规划器崩溃。

本次查询开始于09:35:23；取得状态时会话已转为FOD_ABORTED。板端当前版本为20260914_091120，导航服务active，定位LOCALIZED，覆盖IDLE且无活动任务，导航桥paused=true且没有保留目标。视觉会话ID为ff407449e9364b5bb253ae87b521dc08。

板端日志为UTC，下表已转北京时间：

- 09:34:11.913：视觉启用。
- 09:34:11.991：进入ACQUIRE，等待初始稳定目标。
- 09:35:26.958：触发75秒绝对总时限，进入ABORT。没有APPROACH或锁定记录；状态target_locked=false、approach_path_m=0、blind_forward_m=0，完成回执为空。

运行参数确认为min_acquire_anchor_v_fraction=0.05、max_acquire_anchor_v_fraction=0.90、mode_timeout_sec=75。新版已生效。

09:35:46.378–09:35:56.551只读采集81帧检测，并使用当前获取候选函数离线回放。81/81均被拒绝，原因一致：`target vertical fraction 0.953 is outside capture range 0.050..0.900`。目标检测框底部锚点的纵向比例为0.952646，深度1.722～1.745米。比例按anchor_v/(image_height-1)计算，表示图像自上向下的位置，不是检测框面积或目标距离。故障后81帧复现能够说明当前摆放位置无法锁定，不能冒充之前75秒的逐帧历史证据。

视觉中止后，仲裁器保持STOP并暂停普通导航，等待显式退出/返回。本次采样489份/cmd_vel、201份/cmd_vel_safe、200份/cmd_vel_fod及101份/odom，线速x和角速z均为零。没有调用退出、恢复、启停或运动服务，没有修改源码/参数/授权。

如需按现有范围锁定，应让检测框底部锚点回到90%以内；若实验要求接受当前95.3%位置，需要另行扩大初始捕获范围。退出视觉后可以恢复普通导航，但退出本身不会改变捕获门。

原始证据位于validation/local_route_sleep_0914/：status.log、visual.log、snapshot.json、acquisition_replay.json和replay_summary.log。
