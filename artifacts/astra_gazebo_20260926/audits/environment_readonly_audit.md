# Astra 2026-09-26 现场环境审计

只读审计完成。未启动 ROS master、Gazebo、训练、硬件驱动；未修改 `/home/robot/robot_ws` 或原 source 树。后续独立副本构建由主代理追加授权，构建结果另存阶段 `environment_manifest.yaml`。

## 当前真实环境

- `/etc/os-release`: Ubuntu 20.04.6 LTS；`uname -srmo`: Linux 5.15.0-139-generic x86_64 GNU/Linux。
- `lscpu`: Intel Core i7-14650HX，24 个在线逻辑 CPU；`free -h`: RAM 31 GiB，约24 GiB available，swap27 GiB未使用。
- `df -h`: `/home` 193 GiB可用；根分区36 GiB可用。所有新日志/快照应留在 `/home`。
- `dpkg-query`: ROS roscpp/roslaunch1.17.4，gazebo_ros2.9.3，Gazebo11.15.1，DWA1.17.3，move_base1.17.3；系统TEB0.9.1。
- 源码 `teb_local_planner/package.xml` 为0.8.4，submodule commit b4cf0639775e4521cdf7681158043ad3eef4b01a，包含dirty修复；新基线必须用完整源码树编译该fork。
- `gcc --version`:9.4.0；`cmake --version`:3.16.3；catkin_make、rospack、roslaunch、gzserver、gzclient全部在本机。
- `nvidia-smi --query-gpu=name,driver_version,memory.total,memory.used`: RTX4060 Laptop GPU，580.82.09，8188MiB，约1046MiB使用；GPU存在不等于当前RL使用CUDA。
- `glxinfo -B`: DISPLAY=:0，direct rendering=Yes，NVIDIA OpenGL4.6；`xrandr --current`:逻辑4096x2560，面板2560x1600；gnome-screenshot、xwd、xwininfo、xdpyinfo、ffmpeg的x11grab存在；Xvfb未安装。未捕捉私人桌面，也未产生实验截图。
- `ps` 未发现活跃roscore/rosmaster/roslaunch/gzserver/gzclient/训练进程。后续运行前须再核对。
- `/proc/sys/net/ipv4/ip_local_port_range`:32768–60999；新独占ROS/Gazebo端口应远离此范围，例如14700/14701且先检查占用。

## Python冻结栈与污染

原 `.venv` 仅以清空PYTHONPATH/PYTHONNOUSERSITE=1/PYTHONDONTWRITEBYTECODE=1的直接解释器做只读导入，没有source历史devel。

```
env -u ROS_PACKAGE_PATH -u CMAKE_PREFIX_PATH -u PYTHONPATH PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B scripts/verify_rl_stack.py
# exit0: {"errors": [], "status": "valid"}
env -u PYTHONPATH PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B -m pip check
# exit0: No broken requirements found.
```

实测Python3.8.10，Torch2.4.1+cpu（cuda=False），Gymnasium1.0.0，SB3 2.4.1；system-site-packages提供Scipy1.3.3/Pillow7.0.0及ROS依赖。requirements/thesis-base.txt、thesis-rl-lock.txt和RECORD锁完整可读。原lock只允许CPU，不能原地改成GPU环境；若训练计划需要CUDA应建立独立版本环境与新锁。

登录shell已混入arena_ws/catkin_ws到PYTHONPATH、ROS_PACKAGE_PATH；直接沿用会重现历史包解析错误或载入旧SB3。新环境必须清空CMAKE_PREFIX_PATH、ROS_PACKAGE_PATH、ROSLISP_PACKAGE_DIRECTORIES、PKG_CONFIG_PATH、PYTHONPATH以及Gazebo/动态库overlay，再按 `/opt/ros/noetic` → 新副本devel → 新副本venv顺序加载。不要source旧devel或稳定workspace。

## 源码完整性及最小构建

`git submodule status --recursive` 返回14条全部已初始化，主分支research/pct-teb-dwa-gazebo-20260810，dirty状态存在。不能从HEAD重建而遗漏dirty源码。

os.walk核对src/scripts/config/requirements：644个软链接，0 broken。仅两个外部链接为src/CMakeLists.txt与src/navigation_arena/CMakeLists.txt → `/opt/ros/noetic/share/catkin/cmake/toplevel.cmake`，目标存在。核心入口权限775。

catkin_pkg扫描76包。按package.xml的本地build/export/exec闭包，覆盖benchmark的9包为：

```
autolabor_canbus_driver;m2_gazebo;nav_world_model;teb_local_planner;teb_mode_manager;teb_rl_tuner;thesis_experiment;v3_scope_planner;v5_e2e_scope_rl
```

所有非Python外部ROS依赖包目录均在 `/opt/ros/noetic/share`；Gazebo dev、g2o、SuiteSparse的最终可链接性由clean build确认。原CMakeCache白名单仅4包，依赖以前的完整devel，不能复制到全新环境。新构建需 `catkin_make -j4 -l4 -DCATKIN_WHITELIST_PACKAGES='上述9包' -DPYTHON_EXECUTABLE=/usr/bin/python3`。编译autolabor包只用于消息/服务及链接，不启动其硬件节点。

## 路径和运行链

- activate_thesis_env.sh动态推导THESIS_WS，但默认source已有devel与venv，fresh build前不可直接用它；新脚本将区分构建/运行。
- runner `run_development_matrix.py` 以__file__推导ROOT；launch用 `$(env THESIS_WS)/.venv/bin/python`，核心路径可迁移。旧venv/bin激活脚本和console入口包含绝对路径，不能直接复制bin后使用。
- benchmark launch含真实Gazebo ODE world、spawn_m2、scan/odom/TF、topic relay、真实move_base/TEB或DWA，方法节点从传感器闭环读取。physics/world/node在线可用性尚未测试，不把静态检查当smoke通过。
- runner独立ROS_MASTER_URI、GAZEBO_MASTER_URI、ROS_HOME、ROS_LOG_DIR，禁用model database；环境目前继承dict(os.environ)，需要在新干净环境下启动。
- benchmark共有`/cmd_vel_navigation`→`/cmd_vel`仿真relay；不得作为实车部署链。
- 初始paused=true；episode节点必须先unpause并验证clock正向，再等待依赖，不能沿用历史bootstrap死锁。

## 最短可运行路线

1. 等待主代理快照报告copy_content_verified；只在新workspace写构建与环境脚本。
2. 新建venv并迁移已审计site-packages，生成新绝对路径入口；核对lock/pip/import来源。
3. 清空overlay，仅sourceNoetic，从零编译9包；source新devel后核对rospack、插件.so链接及import。
4. 主代理在新合同/种子/日志身份下运行4方法smoke，并验证clock/physics/TF/scan、真实执行、模型实例/相交与截图。

本次没有外部主机/管理员阻塞。Xvfb缺失不妨碍现有DISPLAY :0或gzserver；GPU训练未验证，但CPU冻结栈已可用。新的Gazebo性能结果、截图库和hardware/shadow验证均未由本审计生成。
