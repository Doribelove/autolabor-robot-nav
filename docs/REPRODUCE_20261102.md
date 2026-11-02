# 2026-11-02 项目交接

远端：`Doribelove/autolabor-robot-nav`，本次更新分支：`1027`。
接收方使用 Intel/AMD Ubuntu，自行完成环境适配。本次发布保留实车工程的已有运行方式，
没有将其改造成开箱即用的 x86 版本。

```bash
git clone --branch 1027 --single-branch https://github.com/Doribelove/autolabor-robot-nav.git robot_j6m_ws_conbine
cd robot_j6m_ws_conbine
python3 tools/reproduction.py verify
# 可选：将地图与相机采集快照恢复到工程原来的目录；不覆盖不同内容的已有文件。
python3 tools/reproduction.py restore
python3 tools/reproduction.py environment
```

这些命令仅检查或复制文件，不启动相机、导航或底盘。私有仓库请使用有权限的 GitHub
账号；也可把克隆 URL 改为 `git@github.com:Doribelove/autolabor-robot-nav.git`。

## 本次包含的内容

- 四路原图 YOLO11-GAM/best6 检测、地面投影、BEV 合并与四路 Qt 识别预览。
- 垃圾二维定位、目标记忆及逐点清扫、区域覆盖、在线建图、录包和 Qt 回放工具。
- 已有 YOLO 权重、ROS 消息/节点、Qt 源码、配置、测试及 HEAL/J6M 研发工具。
- `reproduction/assets/maps/map_20261102_171534/`：11 月 2 日新建地图，包含 PCD、
  原扫描二维图和融合二维图。对应 `config/indoor_mapping_20261102.json`。
- `reproduction/assets/capture/`：当前 ISX 内参、GDC 参数/LUT、采集清单和小型采集库快照。
  其中 `libcapture_pair.so` 是 J6M/aarch64 程序，需要板端已有 VIO/GDC 运行库，不能在 x86 加载。
- `reproduction/manifest.json`：上述文件的原目录、大小与 SHA-256。
- `reproduction/source_environment.json`：本机实际环境版本记录。

当前生产识别运行在 Jetson CUDA；CPU 做几何 BEV。HEAL/BPU 特征融合目录属于研发试验，
不代表当前垃圾清扫正在使用完整 BPU BEV 网络。四路标定来自现车参数，独立米制精度和
曝光同步尚未完成验收。新地图保留原始结果，固定地图 Z 切片把部分地面/树叶计入障碍的问题
没有在本次上传中修改。

## 接收方需要适配的外部环境

本机为 Jetson AGX Orin、Ubuntu 20.04.6、ROS Noetic、Python 3.8、Qt 5.12、CUDA 11.4。
Qt、ROS 节点及消息需要在目标电脑重新编译，不复制本机的 ARM `build/devel` 到 x86 使用。

| 用途 | 现有路径或依赖 | 交接说明 |
|---|---|---|
| ROS 依赖 | `/opt/ros/noetic`、`/home/slam/robot_ws/.deps` | 不在 Git；x86 按目标环境安装/编译 |
| 历史导航构建 | `/home/slam/robot_j6m_ws_navigation_20260907/devel` | 部分脚本依赖此路径；目标机需要重建或调整路径 |
| YOLO 运行环境 | `/home/slam/robot_ws/.venv/fod_yolo` | ARM CUDA wheel 不适用于 x86；权重和自定义 Ultralytics 源码在工程内 |
| ZED | `/usr/local/zed`、旧 `j6m-dev-b/runtime/zed_link_20261019_6y_rm3_5` 的标定/锁路径 | 只做录制结果回放时无需实机采集；重新推理/实车采集需另行准备 |
| J6M 导航 | `/map/autolabor_runtime/rootfs`、`/map/robot_j6m_navigation_20260907/rootfs` | 板端 ROS/运行库和 release 不在 Git |
| MID360 | 板端 `/map/j6m-dev-b/livox2_20261019_2zho_g16` | 既有 Livox SDK2/驱动环境，未随源码上传 |
| HEAL/OE | HEAL 0.0.2、OE 3.9.1、插件 3.3.10、编译器 4.11.11 | 授权安装包、官方权重/HBM 不在 Git |

`conbine_live.sh` 仍使用 `/home/slam/robot_j6m_ws_conbine`、实车网段、底盘串口 ID。
`setup_env.sh`、`bev_perception.sh`、回放脚本和板端 namespace 也有上述历史依赖。
未在接收方 x86 上验证完整构建/回放，不把源码上传等同于环境已经迁移。

## 录包与启动入口

录包、Qt 视频、运行日志、编译产物和完整 SDK 不上传 GitHub。实验包另行复制**整个目录**，
保留 `assets/`、`visual/`、所有分段 bag、参数及元数据。11 月 2 日最后三组在原电脑桌面：
`室外落叶清扫包`、`室外积水清扫包`、`室内垃圾定点清扫包`。

完成目标机环境适配后，可使用原回放入口：

```bash
CLEANING_REPLAY_RECORDING_DIR=/你的录包目录/室内垃圾定点清扫包 \
  bash scripts/replay_cleaning_experiment.sh start 室内垃圾定点清扫包 recorded
bash scripts/replay_cleaning_experiment.sh stop
```

回放现在优先读取包内 `assets/static_map`，避免搬到其他电脑后引用原电脑会话地图。
`mapping` 模式还需 FAST-LIO、Livox 消息和规划节点，且旧脚本含 ARM 构建路径，需接收方适配。
只查看原 Qt 录像可直接播放 `visual/take_*/qt_*.mkv`。

本机实车启动、停止、清扫及录制命令见 [README](../README.md)。默认入口仍是历史八月地图，
使用本次新图需显式选择 `config/indoor_mapping_20261102.json`，并在 J6M 部署同一地图。
清扫由操作者在 Qt 手动开始。提交时 Qt、导航、四路相机和录制均保持关闭。
