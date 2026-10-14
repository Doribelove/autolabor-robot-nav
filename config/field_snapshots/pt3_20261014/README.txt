2026-10-14 pt3 实车回收成功版本的现场配置
版本标签：agx-pt3-recovery-20261014

本目录保存当次测试实际使用的机场通行范围、外沿 1 米余量和 pt3 目标。
gps_keepout.json 内嵌 WGS84 地图几何，不依赖原始“第二次修改.zip”。
agx_gps_map_targets.json 保存目标点名、经度和纬度。
manifest.json 保存来源、文件校验值和版本标记。
test_result.json 保存用户观察与录包核对结果；原始录包仍保留在本机 log 下。

在新机器还原同一现场配置时，先保存已有的 ~/.config/autolabor 中同名文件，
再将本目录的 gps_keepout.json 和 agx_gps_map_targets.json 复制到
~/.config/autolabor/，下次启动会读取它们。机场几何按当前 GPS 原点重新投影。
仅复制这些文件不会发送导航目标。

新机器仍需按 AGX_MIGRATION_GUIDE.md 安装 ARM / ROS / CUDA / ZED 环境并编译。
TensorRT 引擎是设备相关缓存，按 ZED_PERFORMANCE.md 从原始权重重新生成。
现场 MAXN 下运行过 sudo jetson_clocks；该频率锁定不跨断电。
当前保存和上传操作没有重启导航或改变现有现场配置。
