# J6M 四路环视实验框架

开发目录 `/home/slam/j6m-dev-b-surround`，分支 `vision-bpu-surround`。
父目录 `/home/slam/j6m-dev-b` 的 `vision-bpu` 继续保存原 ZED 方案。
稳定基线是 `/home/slam/robot_j6m_ws_navigation_20260907` 的 `0914`。
新旧视觉分支共享父目录的 `.git`；不要移动或删除父目录。

本目录提供**观察实验框架**。截至 2026-10-23，三个相机端口已接通并完成 GDC/BPU
实时显示联调；正式安装外参、曝光时钟映射和环视精度尚未验收。
正式四路配置的待标定项仍保留空值，三路联调使用独立入口及已核验资产。
`production_ready=false`、`motion_eligible=false` 固定保留。
已完成本机开发、独立板端识别候选及实机 Qt 联调，没有刷机或启动车辆控制。

## 实时透视图改为 GDC（2026-10-21）

### 三路输入与原 Qt 端口选择（2026-10-23）

已增加独立三路识别入口：RX1 数值端口 0/1/2 的 GDC 透视图分别进入同一套
YOLO11-GAM + 五类材质分类 BPU 模型。每路取最新帧、公平轮询，不随 Qt 显示选择
停止其他端口的推理。Qt 原“视觉”页的视频顶部新增三个端口按钮，其他页面/分栏保留。
此显示入口不依赖安装外参，也不会生成距离或地图目标；正式四路 BEV 配置保持原样。

实现：`gdc_diagnostic.py` 读取已校验的私有 GDC 资产；`operator_server.py`
复用 `inference.BpuEngine` 及原独占锁、原 HBM/前后处理；原 C++ Qt 新增
`surround_preview_client` 和 `surround_integration`，通过 SSH loopback 预览接口
取回同一报文内配对的图像/检测结果。Qt 只解码 JPEG、绘框和显示，模型不在 AGX 执行。

每帧保留相机会话、数值端口、序号、SDK 两种原始时间字段和 LUT/模型身份。
Qt 校验当前端口、图像/结果配对、模型身份、尺寸、检测框/概率、重复/倒序/旧会话；
切换时清除旧图，并拒绝迟到请求。超过 500 ms 的 SDK 读帧年龄会清图。
曝光时钟映射未校验，SDK 读帧年龄不能当作曝光到显示的延迟，也不生成假的 ROS 源时间戳。
本入口没有 ROS 检测/控制发布；`production_ready`、`motion_eligible`、`accuracy_verified`
和 `source_clock_verified` 均保留 false。

本地构建/运行目录为 `runtime/surround_operator/`；可用入口：

```bash
./distribution/j6m_bpu_navigation/surround/operator_qt.sh build
./distribution/j6m_bpu_navigation/surround/operator_qt.sh start
./distribution/j6m_bpu_navigation/surround/operator_qt.sh status
./distribution/j6m_bpu_navigation/surround/operator_qt.sh stop
```

构建应在本工作树可写、其余目录只读、网络隔离的 bwrap 环境运行，参考本轮交接。
启动仅使用专属 Qt/ROS 会话（11342）和环视预览隧道（18781），不启动导航或底盘。
运行配置 `runtime/surround_operator/session_config.json` 指向独立板端候选目录。
原 ZED BPU 服务正在占用唯一 BPU 锁时，启动入口要求已获明确的临时交接授权；
结束环视 Qt 会停止本次相机/BPU 进程，并恢复此前由本次暂停的原 BPU 接收/自动恢复服务。
不会关闭原 ZED 采集或时钟服务。不能通过更换锁文件并行抢占 BPU。

2026-10-23 本轮已完成源码、Release 构建、10 项 Python 协议测试、5 项实际 Qt 网络/图像
测试及 52 项原 Qt 契约测试。原 C++ Qt 已实机接入三路，按钮逐项切换通过，每路约 5 fps；
未选中的端口仍持续识别。实际加载原 HBM 的 J6M BPU 与 GDC 设备句柄已核验。
当前候选为 `operator_20261023_2`，首次缺少模型契约依赖的 _1 失败已保留并修复；
完整状态见 `docs/team/handoffs/B/20261023_surround_operator_qt.txt`。

### 原四路框架

实时检测、四画面预览及 `bev.sh camera` 统一使用
`RX4/CIM -> PYM NV12 layer0 -> GDC -> 去畸变透视图`。
四路 GDC 调用继续串行，每路透视 LUT 启动时校验并加载一次。
实时路径没有 CPU 去畸变回退；缺 LUT、哈希不符、VPM 画幅不匹配时在打开设备前拒绝。
GDC 运行失败、未返回完成标志、尺寸或格式不符时丢弃该帧，保留原曝光身份。

默认 `config/surround.json` 和本机 `runtime/surround_bev_config/capture.json` 已选
PYM/NV12 与 `runtime/surround_vio_candidate_20261020/` 的候选 VPM。
`layout=NV12` 是要求的格式，`format_verified=false` 仍待实测；端口、SDK、
曝光时钟和 `gdc_bin`/标定 `gdc_sha256` 仍保留待填值，不伪造相机就绪。
捕获库更新为 `runtime/surround_gdc_native_20261021/libj6m_surround_capture.so`。

`bev.sh watch` 的外部输入也要求带正确 `rectification_id` 的 GDC 透视图；
实时 BEV 不构建原始鱼眼映射表。BEV 的透视图到地面投影/融合仍由 CPU 完成，
YUV 转 BGR、模型前后处理和 SDK 调用也仍使用 CPU，因此不是整条视觉链 CPU 零占用。
CPU 去畸变仅在明确的离线 `replay` / 参考测试中可用；`images` / `demo`
继续支持原始鱼眼图直接计算地面 BEV。标定采图单独采原图，不做 CPU 去畸变。
详情见 `docs/team/handoffs/B/20261021_surround_gdc.txt`。

## 独立 BEV 环境与入口（2026-10-20 后续）

用户随后要求先完成配套软件环境，暂不处理相机安装和标定。
已经增加可独立运行的 `bev.py` / `bev_cli.py`，直接接收四路图像计算车体坐标下的
地面 BEV，不需要启动 YOLO、BPU、ROS 或地图定位。地面映射沿用已有参数接口，
不修改真实标定模板。启动时预计算查找表，实时输入为 GDC 透视图；
离线入口也支持原始鱼眼图直接一次 remap 到 BEV。
重叠区域按有效区域边缘距离加权，未覆盖区域保持黑色。
这是标定地面平面的 IPM/环视图，不是学习式三维 BEV 特征或高度重建。

本机专用环境在 `runtime/surround_env/`，不继承系统/用户 site-packages：

| 依赖 | 锁定版本 |
| --- | --- |
| NumPy | 1.24.4 |
| OpenCV headless | 4.10.0.84 |
| Pillow | 10.4.0 |
| pip | 24.3.1 |

Qt 仅在运行 `view` 时只读使用主机已安装的 PyQt5。BEV 计算与文件输出不依赖 Qt。
原 ZED、系统 Python 和全局 ROS 环境没有升级。J6M 不需要安装 Qt 即可生成 BEV，
可经只读预览传输在 AGX 上查看。

在分支根目录直接运行：

```bash
cd /home/slam/j6m-dev-b-surround
./distribution/j6m_bpu_navigation/surround/bev.sh check
# 本机软件检查应输出 software_ready=true；不检查或启动相机。

./distribution/j6m_bpu_navigation/surround/bev.sh view \
  --input runtime/surround_bev_demo_20261020/result_final

# 重新生成四路合成鱼眼图及 BEV；输出目录每次用新名字。
./distribution/j6m_bpu_navigation/surround/bev.sh demo \
  --output runtime/bev_demo_new
```

已生成的结果在 `runtime/surround_bev_demo_20261020/result_final/`：

- `bev.png`：不带文字叠加的 BGR/PNG 地面鸟瞰；
- `coverage.png`：每个像素的贡献相机数，0 表示未覆盖，1～4 表示有效贡献；
- `bev.json`：米/像素、坐标方向、每路源帧/时间、标定与配置 SHA、处理耗时；
- `preview.jpg`：四路画面加 BEV，合成演示明确标记 SYNTHETIC/OFFLINE；
- `state.json`：有效/等待/过期/停止状态；`metrics.jsonl` 记录每次输出统计。

默认范围为车体 X=[-4,6] m、Y=[-5,5] m，分辨率 0.02 m/px，
输出 500×500；图像上方是车头 +X，左方是车体左侧 +Y。
这些仅是可修改的显示网格，不代表真实相机已经覆盖该范围。
默认四路齐全、源帧不超过 500 ms、最大曝光差 50 ms，配置输出上限 5 Hz。
相机丢帧或过期时报告无效；原图不会被修改时间后重复当作新 BEV。
光学参数或画幅改变后需要重启本次 BEV 会话以重建映射。

本机私有配置已初始化在 `runtime/surround_bev_config/`：
`bev.json` 控制画布/帧龄/频率，`capture.json` 接已有采集适配器，
`calibration.json` 与 `clock.template.json` 保留待输入值。
原始模板仍保存在源码 `config/`。后续替换私有输入配置即可，无需改计算代码。

三种输入方式共用同一个计算模块：

```bash
# 一次处理已有四路图片；格式样例见已生成 demo/input/frames.json。
./distribution/j6m_bpu_navigation/surround/bev.sh images \
  --frames DATA/frames.json --calibration DATA/calibration.json --output runtime/bev_images_new

# 接其他相机进程输出的 GDC 透视图及原子更新的 frames.json，不运行推理或 ROS。
./distribution/j6m_bpu_navigation/surround/bev.sh watch \
  --frames DATA/frames.json --output runtime/bev_stream_new --duration 60

# 后续显式使用 J6M 原生采集；本轮没有执行设备采集。
./distribution/j6m_bpu_navigation/surround/bev.sh camera \
  --output runtime/bev_camera_new --duration 60
```

`images` 是离线模式，使用数据文件内的源时钟；`watch`/`camera` 是实时模式，
使用当前时钟校验曝光时间，不接受合成标定或合成帧。
外部生产者先写新图像文件，再原子替换 `frames.json`；图像放在清单目录内部，
同一帧身份对应不可变图像，时间来自实际曝光映射。会话/序号重置须重开 BEV 服务。
只处理最新四帧，不积压队列；按 Ctrl+C 回收本服务资源。
`camera` 使用镜像配套 VIO/hbmem 库、必需的 GDC 透视 LUT 及原有相机锁，不加载 BPU。

如果修改画布配置，可编辑私有 `bev.json`，或将 `--bev-config FILE` 放在子命令前。
下游读取 `bev.png` 时应同时读取 `bev.json` 和 `state.json`，检查 `bev_valid`、
源帧时间和 `bev_png_sha256`；跨文件读取若 hash 不匹配，应重读，不能将中间状态视为完整帧。

已准备 `runtime/surround_wheels_20261020/`，包含 ARM64 Python 3.8、3.11 的离线 wheel
及逐文件 SHA256/requirements.lock。Linux ARM64 Debian 12 / Python 3.11 的隔离容器
已在无网络、无相机/BPU 设备映射条件下完成离线安装、BEV 合成回放和模块测试。
这验证了通用用户态计算环境；不代替 J6M 板端 VIO/GDC 驱动兼容性验证。
完整回归 62 项通过，含新增 15 项 BEV 测试。两个专用 Python 环境各通过 59 项，
3 项 ROS 消息测试只在原 ROS 环境运行；专用 BEV 环境不引入 ROS。

携带离线依赖的新候选包为 `runtime/surround_bev_candidate_20261020.tar.gz`。
后续在目标机独立目录解压后，从该目录运行：

```bash
./distribution/j6m_bpu_navigation/surround/bev.sh setup
./distribution/j6m_bpu_navigation/surround/bev.sh init-config
./distribution/j6m_bpu_navigation/surround/bev.sh check
```

`setup` 只用离线 wheel 创建私有 venv；目标系统需已有 Python 3.8 或 3.11 标准库
及 `venv` 模块。不会 apt 安装、升级系统依赖或复制 AGX 的 venv 到 J6M。
已有 venv/配置目录会拒绝覆盖；已安装时直接 `check`。这份包未部署到 J6M。
环境准备与验证记录在 `validation/surround_bev_environment_20261020/`，
交接见 `docs/team/handoffs/B/20261020_bev_environment.txt`。
依赖准备采用 [pip 官方跨平台下载机制](https://pip.pypa.io/en/stable/cli/pip_download/)；
映射使用 [OpenCV remap](https://docs.opencv.org/4.13.0/da/d54/group__imgproc__transform.html)。

## 已整理的相机镜像

你上传的 `bev/matrix_cam_new.zip` 正是截图标出的 `matrix_cam_new` 文件夹。
本次直接使用这份完整本地包，没有重复从网络下载。
独立副本及解压资料：

```
/home/slam/j6m-dev-b-surround/runtime/surround_vendor_20261020/
  matrix_cam_new.zip
  manifest.json
  extracted-sha256.json
  package/matrix_cam_new/matrix_cam_new/
    img_packages.tar.gz
    j6使用说明.doc
    S10_Camera.tar.gz
    sample.tar.gz
    xmodem_tools.tar.gz
  extracted/
    img_packages/       # 完整镜像文件，未执行其中脚本
    S10_Camera/
    sample/
    xmodem_tools/
  j6使用说明.txt
  isx031std/            # 用户提供的 rclk 修正驱动源码副本
```

完整 ZIP：2,366,565,914 字节，SHA256：
`3064e59c0ed68294102aeff0d1a2531f8b47c2e56f27644da11c8d7777759c58`。
ZIP 成员 CRC 已通过；四个内部 tar.gz 的 gzip CRC/尾部校验也已通过，
3048 个普通文件记录了 SHA256。
厂商归档中的链接仍完整保留在原压缩包；工作解压目录只提取普通文件与目录，
不会把归档链接写到系统路径。因此这些解压目录用于审核，不等同于可以直接覆盖板端的根文件系统。

相机镜像 `img_packages.tar.gz`：2,045,043,852 字节，SHA256：
`24c0b6cd669d120f3cf92f0a4943a16e6a9a8a915863982aa160bcc6d1396716`。
只读查看 `system.img` 确认 Debian 12，`/etc/version` 内容为
`wei.lu@25d246e69553 debug 20260714-200522`；这是包内版本字符串，不是整套 SDK 的兼容性证明。
包内有 `disk/flash_disk_J6M_Matrix_V2.0_nonsec.img`，其 SHA256 为
`af4394c34f04a9798e23ce206b4975ad33cdfd86d677d3c1a4f16a68de1d9930`。
不能仅凭文件名确认当前板子的安全启动模式和 MCU 版本。

另外的 `/home/slam/j6m-dev-b/bev/bev环视.zip` 收尾复核仍缺 ZIP 中央目录，
没有把它作为可用依赖，也没有修改原文件。

## 相机资料核对结果

供应商说明指定 `S10_Camera/intergrationtest/cfg/linux/j6e/case_matrix/Nx_ISX031_RX4/`。
对应配置为 ISX031、RX4、MAX96724、1920×1536@30、`extra_mode=10`，
四个 `deserial_port=0..3`。这是**端口编号**，不能当作前/右/后/左的安装方向。
模块具体料号、线束顺序和驱动 rclk 补丁仍需实物核对。

这个案例输出 CIM YUV422；`config_index=0`、未配置 LPWM，不能声称已四路同步。
字节布局必须实测，适配器支持 YUYV/UYVY/NV16；PYM 路径限定为 NV12，仍需实板核对。
原 `test.sh` 会 `killall -9 camera_sample`、尝试 bind mount 驱动并周期落盘，
所以本框架没有调用它，也没有把这些行为放入启动入口。

## 数据路径与实现边界

```
四路 ISX031 -> RX4/CIM -> PYM NV12 layer0 -> GDC 透视去畸变
  -> CPU YUV 转 BGR -> 每路最新帧槽（总共最多四帧）
  -> 单线程驻留 BPU 双模型 -> 框底点 + 源曝光位姿 -> 地面观测
  -> 跨相机融合 -> 地图候选目标 -> 独立 ROS 环视接口
  -> 低频独立显示线程 -> 四画面 + 地面 IPM 鸟瞰 -> Qt
```

- `capture.py` / `native/capture.cpp`：SDK 初始化、四路读帧、stride 去除、cache
  invalidate、源计数器保留、失败释放 buffer、退出回收。四路 GDC 调用串行。
  原生库使用与镜像配套的头文件构建；运行时只加载明确指定的 VIO/hbmem 库。
  已本机构建并通过假 SDK 的资源/stride 测试，未接真实摄像头。
- `prepare_vio.py`：从供应商 11V 案例提取四路 RX4 的 PYM 路径，生成四路候选配置。
  修正示例把 1536 行裁成 1280 行的问题，仅启用完整画幅 layer0，移除旧 LPWM 绝对路径，
  添加 GDC 节点。已生成 `runtime/surround_vio_candidate_20261020/`。
  PYM 格式转换、slot、GDC LUT 与四路负载仍需实板验证；现已作为默认候选路径，
  配置中的硬件准入标志仍为 false。
- `scheduler.py`：前向初始目标 15 Hz、左右各 3 Hz，转向时后向 2 Hz；倒车后向升为
  主路，原前向降为次路；方向未知时保留低频后向观察。频率是初始预算，不是实测性能。
  最新帧覆盖旧帧，500 ms 过期拒绝；同一路会话重启/序号回退要求新开服务会话。
- `inference.py`：原 YOLO11-GAM + 五类材质 HBM 各加载一次，单 BPU 串行执行，
  沿用原服务的独占锁。默认每帧至多 4 个 crop、分类软预算 12 ms；
  一个已启动的推理不可抢占，实际分类阶段可能超过软预算一个 crop 的耗时。
  未分类的框仍保留，材质为 unknown。CPU 前后处理复用原经过核验的实现。
  编译模型名 `detector_zed` 是原 HBM 的固定名称，不表示新框架依赖 ZED。
- `geometry.py`：支持鱼眼/普通针孔内参，实时推理只接收 GDC 校正透视图；
  CPU 去畸变必须显式指定 `offline_reference=True`，仅供离线回放与参考测试。
  明确使用 `T_base_camera_optical`（光学 x 右、y 下、z 前；车体 x 前、y 左、z 上），
  不虚构恒等外参。地面是 `n·p+d=0` 的标定平面，不默认车体 z=0。
  过滤无效接触点、近地平线、负射线、0.3～10 m 外点和过大协方差。
  固定地面近似不解决坡面、车身俯仰、遮挡底点问题，须在实验有效域内验收。
- `PoseBuffer`：源曝光时刻的完整 `T_map_base`，最多 100 ms 间隔内插值，
  不外推、不用最新 TF 代替历史 TF。地图/定位会话改变后清空旧目标。
- `tracking.py`：100 ms/0.4 m 是候选门限，还要求实测重叠关系、双向最近匹配、
  歧义拒绝和每相机至多一框。保留各相机证据；材质不一致不会直接拆成两件垃圾。
  首次建 tentative，至少 3 次独立时刻观测才 confirmed；同一曝光多相机不重复计数。
  漏检仅计入有真实处理帧且目标处于有效视野的情况。确认目标失联后 stale，
  不按“未调度五帧”删除；completed 墓碑抑制重复清扫，map/session 切换重新建图。
  目标任务状态 API 已具备，正式任务回执接入仍由 E 侧对接，未改清扫控制器。
- `display.py`：复用同一 GDC 透视图，CPU 只做 IPM/绘制/编码，独立线程、最多一个
  显示待处理项。实时显示不再调用 CPU 去畸变；显示专用 STITCH 拼接尚未实板调优。
  YOLO 不接收 IPM/BEV 图。
- `ros_adapter.py` 和 `src/application/autolabor_surround_vision/`：新增带版本、源时间、
  时钟映射/校正图像标识、标定版本、模型 SHA、位置来源和协方差的消息，
  不更改原消息 MD5。
  不发布 `/cmd_vel`、导航目标、障碍物或任何清扫动作。

两份模型 SHA256：

| 模型 | SHA256 |
| --- | --- |
| YOLO11-GAM | `f2f1f7e9753b53b0494495be44af92a0603b3ee6c71e69e0f58a0d50544a8cb8` |
| 五类分类器 | `d4cb80f5f7364d1156ea2275eba9e522991dec309f2c397e74649254fe36a89f` |

类别顺序固定 `metal, plastic, paper, glass, kitchen_waste`。未删除 GAM，未替换模型，
未用合成回放冒充 BPU 推理或环视精度验证。新镜头分布改变后，需要真实环视验证集，
必要时重新做 PTQ 校准；旧 ZED 校准集不能证明新相机精度。

## 现在可直接运行的离线入口

在新分支根目录运行；这些命令不打开相机、BPU 或 ROS：

```bash
cd /home/slam/j6m-dev-b-surround
python3 -m distribution.j6m_bpu_navigation.surround doctor
# 当前返回 2：列出待填标定/硬件项，属于预期结果。

python3 -m distribution.j6m_bpu_navigation.surround demo \
  --output runtime/surround_demo_01
python3 -m distribution.j6m_bpu_navigation.surround view \
  --input runtime/surround_demo_01/result
```

每次选择新输出目录，防止覆盖实验。已跑的回放在
`validation/surround_framework_20261020/demo_release/`，界面截图在同级
`qt_delivered.png`。合成标定只放在演示目录，不能拷进硬件配置。
`replay --events DATA/events.jsonl --calibration DATA/calibration.json --output NEW_DIR`
使用已有检测框验证几何/融合，不运行神经网络；回放结果被 ROS 实时桥明确拒绝。
事件格式见 `demo.py`：pose、frame、可选 task_status，图像相对数据目录寻址。

## 安装后的操作顺序

以下是未来接机步骤，本次没有执行其中的设备操作。

1. **确认与备份。** 记录板卡丝印/版本、安全启动模式、当前 MCU/Acore/SDK、
   网络、时间、分区和 `/map` 空间；备份 `/map`、`/userdata` 中的地图、ROS chroot、
   已部署版本、模型、配置与授权文件到外部介质，计算 SHA256，验证可读和可恢复。
   保存当前能运行的镜像和对应烧录工具。不把包中 Matrix V1/V2、SIP、secure/nonsec 混用。
2. **审查烧录清单。** 供应商 `j6使用说明.doc` 给出了 DFU→MCU→Acore 的步骤；
   文中涉及 V1.0/V1.1/V2.0，多处型号必须与实际板子逐项对应。
   只有获得本轮明确刷机授权并确认可回退后才执行。
   包内 `load.sh` 写两套 A/B 的 acore_cfg、bl31、optee、uboot、vbmeta、boot、
   system、system_verity、basesystem、middleware，并不写 app；
   `flash_all_keymgt_*` 还涉及 GPT、app 等，不能当成等价的一键脚本替换。
3. **烧录后恢复最小环境。** 先验证启动日志、板型、网络、RTC/时钟和分区；
   恢复私有 `/map` 运行目录。核对 libvio/libcam/libhbmem、传感器驱动与补丁 ABI；
   旧 BPU 执行器原验证环境为 DNN/UCP 3.14.5、HBRT 4.9.2，不能假定新镜像一致。
   对两份 HBM 分别做输出契约和固定输入推理比较后再启用 BPU 服务。
4. **单路再四路出图。** 先按实物确认 RX4、PoC 供电、线缆和各模组。
   将厂商案例复制到新的 `/map/.../camera_config/` 修改，保留原案例；
   不直接执行带 killall/mount/自动落盘的 `test.sh`。记录每路序号、分辨率、
   SDK vendor_format、stride、YUYV/UYVY/NV16 或 PYM NV12 布局、丢帧和颜色正确性。
   再逐一遮挡镜头确认 front/right/rear/left → pipeline 的真实映射。
5. **建立同步与曝光时间映射。** 当前 Nx 配置不启用触发。
   按模块支持情况配置 LPWM/外触发，测四路曝光差；保留 CIM 原始计数器。
   区分 FrameStart、HAL 接收时刻、曝光中点和 ROS 时间，确认单位/时钟域、
   offset、漂移、有效期和误差。滚动快门/HDR、自动曝光变化不能用固定延迟掩盖。
   `clock.template.json` 需要实测填入；校准曝光固定时可验证固定 offset，
   其他情况需由 embedded data/时钟服务更新映射。
   框架不提供自动猜时钟或以接收时刻替换曝光的入口。
6. **采内参图像。** 在 `surround.local.json` 填入已核验的相机 SDK 路径、
   四路端口/格式，设 `board_verified`、`port_mapping_verified` 和各路
   `format_verified` 为实测结果。保存到私有配置，不改源模板。
   此原图采集专用配置的四路 `gdc_bin` 必须为空，可选实测 CIM 或 PYM 原始输出；
   不能把已启用 GDC 的实验配置直接用于内参采图。
   车静止，棋盘格覆盖画面中央、四周及不同角度/距离。以下命令只启动相机采图，
   不启动 BPU/ROS；其曝光时间标记为 UNVERIFIED，不可送入实时定位：

   ```bash
   python3 -m distribution.j6m_bpu_navigation.surround \
     --config runtime/surround.local.json calibration-frames \
     --output runtime/calibration_raw_01 --count 25 --interval 3
   python3 -m distribution.j6m_bpu_navigation.surround calibrate-intrinsics \
     --images 'runtime/calibration_raw_01/front_*.png' \
     --columns 9 --rows 6 --square-m 0.03 --output runtime/front_intrinsics_candidate.json
   ```

   上面的 9×6、0.03 m 仅为命令示例，必须换成实际棋盘格内角点数和测量尺寸。
   其他三路分别处理。工具至少要求 15 个成功视角，输出 RMS、逐图残差、图像 SHA；
   输出为候选，不会自动激活标定。用独立图像检查边缘残差与稳定性。
7. **外参与地面。** 明确 base_link 原点，测相机安装位置和姿态；采用棋盘/地面
   控制点及激光联合校准。`solve-extrinsics --points POINTS.json --output NEW.json`
   接受 `points_base_m`、`points_rectified_px`、`K_rectified`，至少 6 个非共线点；
   输出 `T_base_camera_optical` 候选。平面 PnP 可能有歧义，需核对真实高度/方向，
   并用未参与求解的地面点验算。填真实地面平面、有效接触点多边形、误差参数和证据。
8. **透视去畸变与 GDC。** CPU 校正仅用于离线参考核验，实时服务必须使用 GDC。
   确定每路透视输出尺寸、K、R=I、有效视野后，用匹配版本厂商 GDC 工具生成
   **透视去畸变** bin，记录 `gdc_sha256`、`rectification_id`。
   不可把 APA/IPM 鸟瞰 bin 当检测输入。默认已选择候选 VPM、`kind=pym`、`layout=NV12`、
   `pym_layer=0`；填入各路 bin，实测 PYM/GDC 与离线 CPU 参考的像素和几何一致性。
   裁剪/缩放/FOV 一旦改变，必须同步 K、有效区域、GDC bin 与标定版本。
9. **源时刻定位接入。** 新 ROS 桥订阅 `/Odometry`、`/fast_lio/localization_status`，
   按 odom 源时刻查 `map→base_link` 和 `map→camera_init`，写带真实源时间的 pose 文件。
   它要求明确 map_id 和地图定位协方差下限；定位失效、TF 缺失或时间过期不造位姿。
   将 `pose_input` 指向同一文件；chroot 和原生服务需由 A 配置同一个可见路径。
   地图切换必须更新 map_id 并新开会话；定位丢失/恢复或明显 map 修正会更换定位会话。
10. **开始无运动观察实验。** 在新的 `/map/j6m-dev-b/vision-bpu-surround/<版本>/`
    放候选包，按新镜像 SDK 重新构建捕获库并验证原生执行器 hash/ABI。
    先 doctor 清项，再启动 `run`。首轮 60 秒，检查四路画面、材质概率、目标数量、
    地面距离误差、源帧龄、CPU/BPU、丢帧和分类截断情况：

    ```bash
    python3 -m distribution.j6m_bpu_navigation.surround \
      --config runtime/surround.local.json doctor
    python3 -m distribution.j6m_bpu_navigation.surround \
      --config runtime/surround.local.json run \
      --output runtime/observation_01 --duration 60
    ```

    `run` 明确打开设备；默认配置不能执行成功。按 Ctrl+C 仅回收本进程拥有的
    相机 pipeline 与 BPU 句柄，不调用旧 ZED 启停脚本。相机新会话需新开整个服务，
    不把重连前的目标当本次有效目标。
11. **验收后接清扫。** 近/中/远距、四路重叠区域、小目标、夜间、逆光、阴影、
    车体俯仰、遮挡、相机掉线、时钟异常、定位跳变分别验证。
    记录整帧 P50/P95/P99、吞吐、源帧龄、漏检/误检、五类分类混淆矩阵、
    平面定位误差和近邻垃圾去重错误。正式清扫需 E 接新的地面定位语义，
    原视觉伺服要求实测深度，不能把 plane range 填到 depth_valid=true。
    避障继续使用 MID360，本框架不进入障碍规划。

## 构建、显示和接口

本机已经构建的捕获库：
`runtime/surround_gdc_native_20261021/libj6m_surround_capture.so`。
按镜像匹配的 SDK 重新构建：

```bash
cmake -S distribution/j6m_bpu_navigation/surround/native \
  -B runtime/surround_build_new -DJ6M_ACORE=/path/to/matching/Acore \
  -DCMAKE_BUILD_TYPE=Release
cmake --build runtime/surround_build_new --parallel 2
```

ROS 包与原 `autolabor_fod_msgs` 在独立 `runtime/surround_ros_ws` 构建；未运行 roscore
或连接车辆 ROS。启动桥时需要显式提供 launch 的 `code_root`、`session_dir`、
`pose_output`、`map_id`、`map_covariance_floor_m2`，不替使用者捏造定位精度。

| 观察 topic | 类型 | 语义 |
| --- | --- | --- |
| `/surround/observations` | `SurroundObservationArray` | 每个观测内有自己的源曝光时间、相机、标定版本、协方差 |
| `/surround/targets` | `SurroundTargetArray` | 地图候选列表，last_observed 决定有效期，stale 不是新检测 |
| `/surround/status` | `std_msgs/String` | 观察状态或拒绝原因 |
| `/surround/fod_ground_observations` | 原 `FodGroundObservationArray` | 可选兼容观察输出，默认关闭，不接原控制 topic |

源 camera frame_id/sequence、map_id、定位 session、calibration_id 必须同时追溯。
实时桥拒绝 synthetic、replay、停止服务和超过 500 ms 的状态；
消费者仍需按观测/目标自己的源时间拒绝过期数据。
当前目标列表仅在一次服务会话内维护；进程重启不会自动恢复历史清扫任务。
历史 JSONL/state 文件保存作为审计证据，恢复任务要由 E 侧确认。

Qt 可以本地读取会话目录。跨机查看时在 J6M 上单独启动只读预览服务，
默认仅监听 loopback，再从 AGX 建 SSH 隧道：

```bash
# J6M；不打开相机，只读取给定会话
python3 -m distribution.j6m_bpu_navigation.surround serve-preview \
  --input runtime/observation_01 --port 18776
# AGX；地址以现场实际 SSH 配置为准
ssh -N -L 18776:127.0.0.1:18776 hobot
python3 -m distribution.j6m_bpu_navigation.surround view --url http://127.0.0.1:18776
```

服务只提供六个固定只读文件：调试 JPEG、状态、预览时间元数据，
以及独立 BEV 模式的 bev.png、bev.json、coverage.png，不提供控制入口。
显示有合成标识、过期判定和未标定提示，不使用旧画面冒充实时结果。

## 离线验证与交付

```bash
cmake -S distribution/j6m_bpu_navigation/surround/native \
  -B runtime/surround_build_20261020 -DJ6M_ACORE=/home/slam/horizen-ch/Acore \
  -DSURROUND_BUILD_TEST_DOUBLE=ON
cmake --build runtime/surround_build_20261020 --parallel 2
source runtime/surround_ros_ws/devel/setup.bash
SURROUND_NATIVE_TEST_DIR=runtime/surround_build_20261020 \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python3 -m unittest discover -s distribution/j6m_bpu_navigation/surround/tests -v
```

不加载真实 VIO/BPU 库；fake SDK 只供测试，不进入候选发布包的 lib 目录。
本轮 47 项测试通过，原生捕获库和独立 catkin 构建成功；
72 帧合成回放全部处理，形成 2 个确认目标。日志在
`validation/surround_framework_20261020/`，最终测试为 `tests_v7.log`。
新代码、两份原 HBM、消息源码和校验清单可以打成私有候选包：

```bash
python3 -m distribution.j6m_bpu_navigation.surround.bundle \
  --output runtime/surround_candidate_new
```

此命令只生成本地目录和 tar.gz，不连接板卡或部署。厂商完整镜像和 SDK 不打入源码包，
避免把多 GB 系统镜像与应用发布混在一起。新分支所有修改仍未提交、推送或合并。
本轮交付的本地候选为 `runtime/surround_candidate_20261020.tar.gz`，
同名 `.sha256.json` 保存归档大小和校验值；目录内 `MANIFEST.json` 逐文件记录 SHA256。

实验结束后可以离线汇总结果日志：

```bash
python3 -m distribution.j6m_bpu_navigation.surround.report \
  --results runtime/observation_01/results.jsonl --output runtime/observation_01_metrics.json
```

汇总包含各路处理数、源帧龄和处理时间的 P50/P95/P99、未分类数等；
合成/固定框回放会明确标记 `fixture_not_inference`，不作为性能结果。

实现依据：用户提供的 `方案.txt`、`matrix_cam_new.zip`、`isx031std`，
本地匹配 VIO API 头文件与样例；算法接口另核对了
[地平线 GDC/STITCH 说明](https://developer.horizon.auto/blog/10173) 与
[OpenCV 鱼眼模型文档](https://docs.opencv.org/4.x/db/d58/group__calib3d__fisheye.html)。
本轮测试与完整交接见 `docs/team/handoffs/B/20261020_surround_framework.txt`。
