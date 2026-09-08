# 2026-09-08 仅视觉 Qt 会话

> 后续更新：用户要求部署导航修复并继续实验，本视觉会话已于 21:39 通过
> `stop.request` 正常结束，相机/Qt 子进程退出、私有 BPU stop 返回 0。
> 随后完整导航已启动，见 [入口恢复部署记录](ENTRY_RECOVERY_TRIGGER_FIX_20260908.md)。
> 下文保留原视觉实验历史。

用户要求使用当前垃圾误框优化版打开 Qt，只运行视觉识别。分支仍为
`feature/litter-perception-optimization-20260908`，本轮没有切换分支、修改业务代码、
构建、部署、提交或推送；既有未提交导航实验修改均保留。

## 当前运行范围

21:06 起，本机独立 master `http://127.0.0.1:11571` 运行 ZED QUALITY RGB-D、
相机坐标变换、当前 FCOS RGB-D 几何筛选和 Qt。Qt 默认进入 BPU 预览页，可切换原始框
进行对照。ZED 位置跟踪、深度稳定及 RViz 关闭。

此前完整导航经 `./scripts/optimized.sh stop` 停止，返回 0；原被动覆盖记录器也已退出。
导航、定位、覆盖、雷达、CAN 底盘、AI 及 CenterPoint ROS 节点均未启动。
本机与 J6M 原 11311 端口无监听。既有板端常驻服务仍预加载 FCOS/CenterPoint 两个模型，
本会话只请求 FCOS，没有 CenterPoint 点云输入或推理请求。

既有主运动配置 true、FOD=false、1.60 m/s 上限和授权标记均保留；这些配置不表示
当前启动了运动链路。J6M current 仍为 `20260908_192331`。完整导航旧会话已结束，
以后恢复完整导航应先关闭此视觉会话，再通过统一入口启动，由操作员设置新会话真实初值。

## 运行验证

25.05 秒被动观察中，RGB/深度各 375 帧，FCOS 有效且已标定结果 373 帧；
结果及预览约 14.91 Hz。没有收到速度、导航目标或初始位姿消息。
实际业务节点为 Qt、fcos_rgbd、ZED 驱动及相机 state publisher，另有 rosout；
验证期间的观察节点已正常退出。

末帧保留 1 个 REVIEW 框，原因是地面估计不可用；本轮证明视觉链路持续输出，
不代表已经完成垃圾误检率、漏检率或长时间稳定性验收。

## 持续运行与结束

会话无运行时限，使用本目录私有 tmux：

```text
socket: runtime/run/vision_only_20260908.tmux.sock
session: vision-only-20260908
```

关闭 Qt 会由会话管理器结束相机、识别及其私有板端 BPU 服务。需要从终端结束时，
在本工作区执行以下命令，交由管理器清理其拥有的进程：

```bash
touch validation/vision_only_20260908/stop.request
```

本轮没有执行上述停止命令，窗口留给用户测试。停止后的运行结果写入 `stopped.json`。

## 本地证据

证据目录为 `validation/vision_only_20260908/`，由 Git 忽略，留存本机：

- `stop_full_stack.log`、`stop_prior_bpu.log`：原导航和原私有 BPU 停止记录。
- `session.json`、`session.log`、`roslaunch.log`：当前会话身份与运行日志。
- `runtime_check.json`、`observe.log`：25 秒节点、参数、频率和消息检查。
- `vision_only.launch`、`launch.sh`、`session.py`：本次独立持续运行入口。
- `observe.py`：只读验证脚本。

这些临时入口并非已提交的通用启动功能，不应假设可以仅从 Git 恢复。
