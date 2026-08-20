# 2026-08-20 静态地图与 FAST-LIO 定位调试对话记录

> 本文件是当前对话的工程交接摘要，不是聊天界面的逐字导出。原始逐条对话由会话
> 系统保留。为了避免泄露凭据，用户在对话中提供过的密码未写入本文件。

## 用户需求轨迹

本次连续对话围绕以下任务展开：

1. 阅读整个工程，确认 `start_dual_host.sh` 是否自动加载全局地图以及 move_base
   全局代价地图的来源。
2. 梳理 MID360 与前后两台 LD19 二维雷达的融合逻辑，并确认历史建图是否实际使用
   了两台 LD19。
3. 通过物理插拔确认固定 USB 接口：USB `1-4.4` 为前雷达，USB `1-4.3` 为后雷达。
4. 使用 MID360、前 LD19 和后 LD19 建立三类地图：FAST-LIO 三维 PCD、双 LD19
   二维占据图、两者的二维融合图。
5. Qt 开始录入静态地图时后台建图，结束时自动保存完整地图集。
6. 将已知地图重定位从直接修改 FAST-LIO IEKF 的方案改为独立
   `fast_lio_localization`，并更新一键启动及 README。
7. 排查静态地图启动后坐标漂移到数十万米的问题，并在车辆静止时以零初始位姿做
   现场验证。
8. 恢复运动链并尝试沿车头方向行驶 2 m；测试中发现 LD19 车体反射和规划绕行问题。
9. 将最终源码保存到 Git，并建立可恢复的固定标签。

## 最终定位架构

静态地图模式采用以下 TF 和职责分工：

```text
map --独立多尺度 ICP--> camera_init --原始 FAST-LIO--> body --静态外参--> base_link
```

- 原始 FAST-LIO 只负责实时激光惯性里程计，发布 `camera_init -> body`、
  `/Odometry` 和 `/cloud_registered`。
- `fast_lio_map_localizer` 加载 `map_3d/map.pcd`，收到 `/initialpose` 后执行粗、精
  两级 scan-to-map ICP，发布 `map -> camera_init`、`/localization` 和定位状态。
- `map_server` 加载地图集中的二维占据图，供 move_base 全局和局部 costmap 使用；
  它不负责定位。
- 本架构不再启动 AMCL。导航速度只有在定位状态为 `LOCALIZED` 时才能通过
  `fast_lio_localization_cmd_vel_gate`。

## 当前地图与部署

- 当前地图集：`global_maps/map_sets/map_20260820_160221`
- `global_maps/map_sets/latest` 指向上述地图集。
- 地图集包含：
  - `map_3d/map.pcd`
  - `map_2d/map.yaml` 和 `map.pgm`
  - `map_fused_2d/map.yaml` 和 `map.pgm`
- 当前 J6M 已部署发布：`20260820_172416`
- 旧问题发布：`20260820_141651`

## 数十万米漂移故障

现场确认 J6M 当时仍运行旧发布 `20260820_141651`。该版本没有独立的
`fast_lio_map_localizer`，而是把固定地图匹配直接接入 FAST-LIO 的 IEKF。出现
`No Effective Points` 后，FAST-LIO 状态被污染，位置从几十米继续发散到数十万米。

处理结果：

- 删除 FAST-LIO 内部的固定地图定位改动，恢复其纯里程计职责。
- 新增独立 `fast_lio_localization` 包。
- 静态地图启动前检查 J6M 当前发布是否包含独立定位器；旧发布会被拒绝启动。
- 运行态健康检查会读取本次托管启动保存的地图模式，并强制检查
  `/map_server`、`/fast_lio_map_localizer` 和定位速度门。
- README 中的建图起点示例改为 `/initialpose=(0,0,0)`；定位器内部负责
  `base_link` 与 MID360/`body` 安装偏置的换算。

## 静止现场验证结果

车辆位于建图起点附近时发送：

```text
x=0, y=0, yaw=0
```

结果：

- 一次进入 `LOCALIZED`。
- ICP 重叠率约 `99.8%～100%`。
- RMSE 约 `0.16 m`。
- 有效匹配点约 `2300～2400`。
- 45 秒内 FAST-LIO 平面波动约 `1.6 cm`。
- 45 秒内地图定位平面最大波动约 `6.5 cm`。
- 2252 条 `/cmd_vel` 采样全部为零。
- 测试汇总曾达到 `383 tests, 0 errors, 0 failures`。
- 未再次出现 `No Effective Points` 或巨大坐标发散。

## LD19 自反射修复

恢复运动前发现融合 `/scan` 在正前方反复出现约 `0.4 m` 的近点。分源检查确认：

- `/mid360/scan` 没有该近点。
- 近点来自 `/dual_lidar/scan`，是 LD19 极短距离车体反射经安装外参变换后的结果。

已在 `dual_laser_fusion.py` 中加入变换到 `base_link` 后的车体矩形裁剪：

```text
x=[-0.75,+0.75] m
y=[-0.50,+0.50] m
```

过滤后，正前方 15 度最近有效障碍由约 `0.4 m` 恢复为 `4.58 m`，双雷达融合的
5 项单元测试通过。该过滤也会作用于以后使用 `/dual_lidar/scan` 建立的二维地图。

## 2 m 运动测试结果

用户明确授权后，曾临时执行以下操作：

- `MOTION_ENABLED=true`
- 创建临时运动授权标记
- 静态地图模式冷启动并发送零初始位姿
- 检查正前方净空和 Navfn 规划结果
- 下发沿起始车头方向 2 m 的 move_base 目标

规划器受当前静态代价地图影响生成了约 `3.8 m` 的绕行路径，而不是直线路径。主动
安全监控在总位移超过 `3.5 m` 时取消目标。取消时记录：

- 地图平面总位移约 `3.60 m`
- 起始车头方向投影约 `3.43 m`
- 横向偏移约 `-1.07 m`
- 最大线速度 `0.30 m/s`
- 定位全程保持 `LOCALIZED`
- 取消后 `/cmd_vel` 连续为零

因此该次测试证明了底盘运动链可以工作，但没有证明“准确直行 2 m”。再次做运动测试
前应先检查地图中起点附近的静态障碍、膨胀层和全局路径，不能直接复用零初始位姿。

## 当前物理与安全状态

对话保存时：

- `autolabor-dual-host.service`：`inactive`
- J6M ROS/navigation 栈：已停止
- `MOTION_ENABLED=false`
- `FOD_MOTION_ENABLED=false`
- `runtime/motion_authorized.ok`：不存在
- CAN 设备：无托管进程占用

重要：车辆经过运动测试后已经不在原来的建图零点。最后一次定位记录约为
`map/base_link=(3.314,-0.983)`，该值仅供重新定位时作为近似初值，实际重启前应结合
现场位置和 Qt 地图确认，不能无条件再次发送 `(0,0,0)`。

## Git 保存点

- 分支：`feature/820-three-map-relocalization`
- 源码恢复提交：`c830bcc`
- 固定标签：`recovery-20260820-static-localization`

查看保存点：

```bash
git show --stat recovery-20260820-static-localization
```

将当前分支强制恢复到该源码保存点：

```bash
git switch feature/820-three-map-relocalization
git reset --hard recovery-20260820-static-localization
```

`reset --hard` 会丢弃未提交修改，执行前必须先检查 `git status`。

## 后续安全启动建议

1. 保持运动开关为 `false`，先启动静态地图与定位链：

   ```bash
   ./scripts/start_dual_host.sh --start \
     --map-set global_maps/map_sets/latest
   ```

2. 根据车辆运动后的实际位置，在 Qt 地图上发送近似初始位姿；不要发送原点零位姿。
3. 确认定位持续为 `LOCALIZED`，并在 Qt 中检查全局路径是否绕行。
4. 只有在实体急停可用、路径净空且用户再次明确授权后，才临时恢复运动开关。
5. 测试完成后立即停止系统、关闭运动开关并撤销授权。

## 未纳入 Git 的用户目录

以下目录始终保持未跟踪，没有修改或提交：

- `YOLODATABASE/`
- `ultralytics_yolo11_custom/`
