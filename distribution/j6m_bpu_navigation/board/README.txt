YOLO11 板端准备包（2026-10-19）

本包准备正式单类垃圾检测 YOLO11s-GAM 和五类材质分类模型的板端环境。
包含两份正式 ONNX、nash-m 编译配置、导出记录、模型契约、各一份校准输入样本，
以及已有张量执行器接口、通信/后处理源码和离线边界测试。

完整 40/50 份校准输入保留在 AGX 当前 vision-bpu 工作区的
runtime/j6m_bpu_migration_20261010/private_ptq/，供 x86_64 编译环境使用。
板端 ptq/calibration/ 只带原始样本清单，不包含完整量化数据；不能用本包两份
自检样本代替完整校准集。本包不是可直接编译的完整 PTQ 交付包。

J6M 部署布局：
  /map/j6m-dev-b/vision-bpu/releases/<release>/
  /map/j6m-dev-b/vision-bpu/current

在所选候选目录运行：
  python3 -B environment.py status
  python3 -B environment.py verify
  python3 -B environment.py self-test
  python3 -B environment.py check-models
  python3 -B environment.py rollback

self-test 校验文件/依赖哈希、正式 ONNX 与配置、样本张量、NumPy 和执行器动态库，
并调用只查询版本的 hbDNNGetVersion API；不会加载模型或启动 BPU 推理。
check-models 明确返回 MISSING_FORMAL_HBM 和退出码 2。这是预备版本的固定状态；
生成 HBM 后需要新建发布版本、记录真实张量属性/哈希，并另做实际推理验证。

复用的板端执行器：
  /map/robot_j6m_navigation_20260907/bpu_production_20261010/build_v5/libj6m_tensor.so
复用的单模型 worker：
  /map/robot_j6m_navigation_20260907/bpu_production_20261010/tools/tensor_stdio.py
私有运行依赖在 DEPENDENCIES.json 中锁定，不修改系统库、既有参考服务或独占锁。
tools/ 内保留的原 worker/client 源码带历史路径保护；它们用于后续适配审阅，
不能从新候选目录原样启动。双模型服务及原 ROS 视觉后端接入仍待实现。

activate / rollback 只操作本准备包的 current 符号链接，不启动或停止任何进程，
不修改原导航、ZED 或 Livox 的 current。rollback 保留模型、日志和旧版本。

仍需配套的 x86_64 OpenExplorer / horizon-tc-ui / hbdk4 / hb_compile。
编译器与现有 DNN 3.14.5、HBRT 4.9.2 的配套版本尚未核定，不能随意混装。
检测网络保留 GAM，需检查 MatMul/Softmax 等算子实际分配；完成 INT8 精度、
0/1/4/8 crop 整帧速度和异常处理验收前，production_ready 始终为 false。
