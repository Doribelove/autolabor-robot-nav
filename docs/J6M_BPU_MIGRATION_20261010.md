# 0914 J6M/BPU 迁移初稿与当前验收边界

后续已恢复网线访问并完成隔离板端 BPU 执行层部署，最新状态见
[2026-10-10 板端续接记录](J6M_BPU_BOARD_ADAPTATION_20261010.md)。下文保留本阶段
原始结果；“无法连接、未部署”仅描述本阶段，正式垃圾模型尚未完成 HBM 迁移仍然成立。

本轮工作仅在 `/home/slam/robot_j6m_ws_navigation_20260907` 的 `0914` 当前工作树进行。
HEAD 为 `5d1028efa93cf34d2179b03384085c87a456402a`，未提交修改也是基线的一部分。
开工时26份已有修改/未跟踪文件已记录 SHA；收尾全部一致，私有运行配置和运动授权标记
也保持原哈希。没有修改其他项目、切分支、提交、推送、启停导航或发送车辆控制指令。

## 本轮完成

- FCOS 桥接 `Client` 与 ROS launch 增加可配置 SSH 目标、私有 `/map` relay 路径。
  默认仍使用现场地址/路径，保留原消息序号、结果身份、过期和候选属性；对 SSH 远程
  shell 元字符及路径逃逸在启动子进程前拒绝。
- 新增 `distribution/j6m_bpu_navigation`：CPU/BPU 分工、模型契约、PTQ 准备、校准输入
  生成/检查、锁定 SDK 版本的编译入口、参考算法依据和部署/验收步骤。
- 只读复用归档中的正式检测/分类 ONNX，实际模型哈希、checker、输入输出尺寸与类别
  通过核验；没有删除 GAM 或用 COCO 类别冒充五材质。
- 从训练配置确认的 `trash_detection_dataset/images/train` 选40张带标签训练图；
  从 `material_classification_dataset/train` 每类选10张，共50张。固定随机种子
  `20261010`，源文件/处理张量均记录哈希。分类原 checkpoint 的 saved transforms
  与校准前处理一致；对当前640×360输入，固定576×1024 LetterBox与原1024 rect
  LetterBox逐像素相同。样本只是初步校准选集，尚未做业务代表性/INT8精度评测。
- 准备两份 `nash-m`、单 core PTQ 作业；输入使用已经完成原预处理的 float32
  `featuremap`。校准数据、模型和 SDK 版本完整校验通过后才允许运行编译器。
- 新增白名单源码导出工具，生成审阅压缩包与逐文件 SHA256；另将正式模型与90份校准
  张量打成私有 x86 PTQ 交接包。两种包用途不同，私有包不用于公开投稿。

## 验证

| 检查 | 本轮结果 |
|---|---|
| BPU 协议、几何、CenterPoint、障碍候选测试 | 23项通过 |
| FCOS RGB-D 测试 | 5项通过 |
| 新 PTQ 工具边界测试 | 9项通过 |
| 两份实际 ONNX checker/接口/哈希 | 通过 |
| 90份训练校准张量、五类覆盖、哈希/尺寸/有限值 | 通过 |
| 两种 detector LetterBox / 分类 saved transforms | 通过 |
| 可配置 launch 参数展开、Python 语法 | 通过 |
| `optimized.sh check --static` | 通过；648为原有测试结果汇总，不是本轮新增648项 |
| ARM64 编译入口保护 | 明确拒绝执行 x86 `hb_compile` |
| 审阅包逐文件哈希/无软链接与路径逃逸 | 通过 |
| 开工前26份已有改动、配置/运动标记保护 | 通过 |

初次直接测试缺 ROS 环境；第二次额外把源码放到 PYTHONPATH 首位，遮蔽 catkin 生成消息。
已改用项目规定的 `setup_env.sh`，23项正常通过；没有为测试修改业务包的导入机制。
初版打包清单引用了不存在的独立 CAN 消息包，已纠正：消息实际在底盘驱动包内。
初版包仍保留，最终包使用收尾后的工具源码。

证据目录：`runtime/j6m_bpu_migration_20261010/`。其中 `baseline.json`、
`preservation_check.json`、`calibration_verification.json`、`*_tests*.log`、
`static_check.log`、`fcos_launch_params.yaml` 与 `compiler_guard.log` 可复核本轮结论。

交付物：

- `source_draft_final.tar.gz` / `.sha256`：供地平线审阅的源码初稿，不附私有权重/运行库。
- `private_ptq_handoff.tar.gz` / `.sha256`：内部 x86 开发端编译交接，含固定 ONNX 与校准输入。
- `private_ptq/`：本地准备目录；`STATUS.json` 是准备时快照，完整校准状态以
  `verify --require-calibration` 与 `CALIBRATION_STATUS.json` 为准。

## 未完成与下一步

本机为 ARM64；本轮没有可用的 x86_64 OpenExplorer 量化编译环境，所以尚未执行
INT8/HBM编译。两处已知 J6M 地址连接失败；没有部署、挂载/修改板端 rootfs、切换
`current`、启动 BPU/ROS 或确认板端当前进程。
`20260914_095005` 只是最近保存的部署记录，不能当作本轮实时查询。

正式 YOLO11-GAM + 五材质后端仍在 AGX。专用 BPU 实时服务、GAM 算子落点审查、
INT8精度、HBM golden-output、分类多 crop 延迟及 GPU A/B 均未完成。
CenterPoint 原控制接入禁止条件保留；尚未接入 TEB，也没有新的实车运动验收。

下一步提供独立 x86_64 开发环境及确切 OE/HBDK 版本，使用私有交接包编译并保留报告；
随后恢复 J6M 连接，核对厂商运行库配套、HBM tensor 属性和同输入输出一致性。
通过无运动观察链和断连/延迟测试后，再实现并配套部署正式垃圾 BPU 后端。
车辆实验另按当前现场授权与项目安全条件执行。

已查阅官方 `hb_compile` 文档并使用其 `nash-m` 与校准输入接口。
`oe.horizon.auto/labs` 页面可读，但本次没有平台投稿账号/已核实的提交接口；
压缩包已备好，尚未对外上传或提交。

完整命令和官方资料链接见
[J6M/BPU 工具链初稿](../distribution/j6m_bpu_navigation/README.md)。
