# 独立发布副本验证（2026-09-09）

以下是维护者本机对独立白名单导出的验证，不是 GitHub Actions 运行结果。
导出没有挂载原工作区的 build/devel/install、私有依赖 sysroot、设备或现场地图。

## 环境

* 宿主机 ARM64 Linux；容器 Ubuntu 20.04 / ROS Noetic。
* 实际构建基础镜像：`ros:noetic-ros-base-focal`，本次解析摘要
  `sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d`。
* 依赖由仓库 Dockerfile 从公开 apt 源安装，源码通过 COPY 导入。
* `tools/build.sh` Release 编译退出 0，三包及全部消息、插件、TEB 和 benchmark 构建通过。
* 测试容器 `--network none`，仅 lo；没有连接真实 ROS 图或硬件。

## 实测结果

| 检查 | 独立用例数 | 结果 |
|---|---:|---|
| 覆盖几何 Python | 26 | 通过 |
| 覆盖状态机 Python | 149 | 通过 |
| 固定挡位命令 mux Python | 3 | 通过 |
| Hybrid / Reeds–Shepp C++ | 8 | 通过 |
| 定制 TEB C++ | 16 | 通过 |
| 底盘速度/状态轮询 C++ | 19 | 通过 |
| 导航所有权/暂停桥 Python | 24 | 通过 |
| 清扫真实插件：5 m、2 m、0/-1/NaN/Inf | 36 | 通过 |
| 首入口真实 ROS 组件场景 | 14 | 通过 |

前六行合计 221 个独立 XML testcase，另有 24 个桥测试；共 245 项单元检查。
本环境 `catkin_test_results` 原始汇总显示 264，其统计对 GoogleTest 嵌套汇总重复计算
43 个 C++ 用例，不能直接当成独立用例数。`tools/test_summary.py` 按 testcase 节点
只计一次。所有 XML failure/error/skipped 为 0，两套容器 runner 均退出 0。

清扫两组有效窗口各 14 项，四组无效窗口各 2 项，共 36；首入口本次没有提供历史
旧版资产，当前版本的 14 项全部通过。绕墙路径 462 点、23.272 m，晚订阅首入口路径
仍保留；取消、换图和无路检查通过。场景中没有 move_base/Hybrid 运动目标，也没有
cmd_vel 消息。这里的 50 项场景检查不计作实车移动测试。

标准库几何示例生成 7 条清扫线、0.85 m 线间距、60 m² 合成请求区域，输出 JSON/SVG。
源项目独有的 15 项整机文件契约检查另行通过，未混入公开版的独立用例计数。

## 尚不能据此声称

* 未把本地通过写成远端 CI 通过；仓库 Actions 以推送后的实际运行结果为准。
* 本次干净容器实测为 ARM64；x86_64 CI、不同底盘和传感器接入需要各自结果。
* 文档和发布脚本修改不等于修复 KNOWN_ISSUES 中的转场执行问题。
* apt 包下载及浮动镜像 tag 可能变化；本记录保留本次基础镜像摘要，源码由
  SOURCE_MANIFEST.json 逐文件校验。没有宣称未来下载永久逐位相同。
