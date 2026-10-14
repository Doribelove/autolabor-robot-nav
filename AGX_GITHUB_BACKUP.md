# AGX Orin 当前项目的 GitHub 版本

版本日期：2026-10-14。账号：`Doribelove`。
主仓库：`Doribelove/autolabor-robot-nav`，分支：`agx-orin-20260929`。
当前实测版本标签：`agx-pt3-recovery-20261014`。
来自本机 `/home/slam/robot_ws_agx`，基于原 `gnss_0913_lianxinag` 的
`eb2353ade16432b8dc635742679a33d4ac8a379b`，包含其后本机源码、配置和文档修改。

## 2026-10-14 pt3 实车回收成功版本

现场用户确认本轮垃圾已回收。录包证明本轮使用视觉与 GPS 接续：先按原始
pt3 接近，随后由 ZED 注册深度和相机到车体的变换对准单个 FOD、修正其记录
位置；目标离开视野后，GPS 执行记录 FOD 前方 0.8 米的终点。

- 12:53:32，距原目标约 9.91 米，联动升杆 3 秒与清扫电机。
- 12:53:38，视觉进入 APPROACH，使用 registered_depth 转向。
- 12:53:44，视觉进入 GPS_HANDOFF；12:53:46，GPS_FOD_RECOVERY 实际接管。
- 12:53:55，到达终点并停车；12:53:58，关闭清扫和收杆 3 秒的流程结束。

垃圾入仓结论来自用户现场观察。软件到点本身不代表入仓，液压设备也不回传
杆位置。上述时间均为 2026-10-14 Asia/Shanghai。

本版本一并保存当前工作区的液压联动、Qt 机场地图和 GPS 禁入区、目标点导入、
直接雷达障碍云与地面过滤、深度转向和短漏检重接、近距离 FOD 位置修正、
ZED/Qt 刷新优化及相应测试。子模块继续使用已经发布的精确提交。

本轮部署前的验证记录为 124 项相关单元测试、4 项隔离 ROS 整链路仿真及 Qt
构建通过；随后进行了上述 pt3 实车测试。本次保存版本不重新运行车辆。
相机横向安装补偿仍属于现场图像估计，完整外参尚未尺量标定；0.8 米为设置的
终点偏移，实际停车使用现有导航到点容差。

机场通行范围、1 米外沿余量、当前 pt3 目标和测试摘要保存在
`config/field_snapshots/pt3_20261014/`，恢复方式见其中的 `README.txt`。
本机原始录包保留在
`log/operator_fod_depth_final_20261014_124519/pt3_fixed_record_20261014_125205/`，
不上传 rosbag、原始图像、日志或构建目录。

固定获取本次已实测版本：

```bash
git clone --recurse-submodules --branch agx-pt3-recovery-20261014 \
  https://github.com/Doribelove/autolabor-robot-nav.git robot_ws_agx
```

## 下载

在新目录克隆，并递归获取子模块：

```bash
git clone --recurse-submodules --branch agx-orin-20260929 \
  https://github.com/Doribelove/autolabor-robot-nav.git robot_ws_agx
```

已有该仓库副本、且已切到本分支时，可补齐子模块：

```bash
git submodule sync --recursive
git submodule update --init --recursive
```

四个有本地修改的子模块（FAST_LIO、arena-rosnav-3D、teb、livox_ros_driver2）
各自在 Doribelove 对应仓库的同名分支保存，主仓库记录精确提交。TEB 子模块
URL 使用 `https://github.com/Doribelove/teb.git`，确保能获取本地定制版本。

## 当前功能与操作资料

- AGX Orin ARM 适配、双天线 GNSS 单主机导航、ZED 深度与 GPU YOLO、Qt 清扫控制。
- 1.6×0.8 m 车体轮廓、导航默认速度 1.6 m/s，以及航向和局部规划连续性修改。
- 相机回收在原目标 10 米内联动清扫，6 米内记录稳定最近的单个 FOD；以
  0.6 m/s 结合深度靠近，目标丢失后接续到记录 FOD 前方 0.8 米的 GPS 终点。
- 已按用户要求删除 0.35 秒检测断流、0.25 秒控制更新及穿越 4°/8 厘米中止条件。
- [完整启动和关停命令](AGX_START_STOP.txt)、[现场操作](AGX_STARTUP.md)。
- [迁移报告](AGX_MIGRATION_REPORT.md)、[回收仿真报告](FOD_RECOVERY_STABILITY_REPORT.md)、
  [三项限制解除验证](FOD_CONTROL_LIMITS_REMOVAL_REPORT.md)。

原始 FOD 权重在 `src/yolo/fod_yolo11n_img640_e300_orig/weights/best.pt`，
SHA256：`7bf99d4c61343e8cdb37289f2eece6cf18342b508f9b7f80723592edce398500`。

这是源码和项目资料备份。`build/`、`devel/`、`install/`、运行日志、rosbag、
系统 ROS/CUDA/ZED SDK、Python 虚拟环境和本机 TensorRT 缓存不属于 Git 源码。
新机器需要按 [AGX 迁移指南](AGX_MIGRATION_GUIDE.md) 安装匹配环境并重新编译；
按 [ZED 配置](ZED_PERFORMANCE.md) 配置相机和重建推理引擎。
启动命令中的设备路径适用于当前同一套硬件，更换设备时按实际串口调整。

本次上传沿用已有验证记录，未因上传重新启动车辆或执行实车运动。
