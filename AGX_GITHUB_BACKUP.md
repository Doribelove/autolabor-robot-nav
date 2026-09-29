# AGX Orin 当前项目的 GitHub 版本

版本日期：2026-09-29。账号：`Doribelove`。
主仓库：`Doribelove/autolabor-robot-nav`，分支：`agx-orin-20260929`。
来自本机 `/home/slam/robot_ws_agx`，基于原 `gnss_0913_lianxinag` 的
`eb2353ade16432b8dc635742679a33d4ac8a379b`，包含其后本机源码、配置和文档修改。

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
- 相机回收选择最近一个 FOD，0.6 m/s 接近，进入中下回收区后消失则连续推进 0.8 m。
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
