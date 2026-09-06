# 双感知实验

总架构、实测、限制和回退说明见 [DUAL_PERCEPTION_20260906.md](../../DUAL_PERCEPTION_20260906.md)。

本目录的原生服务只允许部署到 J6M `/map/robot_j6m_optimized_20260905/bpu_perception_20260906`。
先按 `scripts/deploy_perception_bpu.sh` 核验固定模型和依赖，再编译 J6M 原生库。
这里的参考模型不能作为自动捡拾或已验收的导航障碍物输入。

相机/Qt＋合成点云通路验证（需要两个导航服务均停止，J6M 候选 chroot 已由现有入口挂载，私有 BPU 服务已就绪）：

```bash
./scripts/optimized.sh run bash -c '
  source scripts/setup_env.sh
  export ROS_MASTER_URI=http://192.168.10.50:11571 ROS_IP=192.168.10.50 DISPLAY=:0
  /home/slam/robot_ws/.venv/fod_yolo/bin/python3 -u \
    experiments/dual_perception_20260906/tools/rgbd_test.py --seconds 60 --replay --qt
'
```

`--replay` 明确生成合成平面点云，仅验证 ROS、模型执行、结果过期与生命周期。
真实传感器改用 `--mid360`，并先确认 MID360 有载波/地址可达；本轮真实传感器测试尚未通过。
只测 RGB-D 时 master/IP 用 `127.0.0.1`，可选 `--centerpoint-load` 并行施加原生合成负载。

测试均有最长时间和所有权清理。正常伴随模式由 `nvidia_ui.sh` 启动，无演示截止时间。
模型、现场图像/观测、编译产物和第三方运行库均保留在私有路径，Git 仅记录源码和模型固定清单。
