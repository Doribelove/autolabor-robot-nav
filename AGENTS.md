# 当前 AI 开发规则与运行约定（2026-10-24）

修改本项目代码、脚本、launch、配置、界面或运行方式前，必须阅读并遵守
[DEVELOPMENT_CONSTRAINTS.md](DEVELOPMENT_CONSTRAINTS.md)。
本节与该文件是当前规则，下方旧记录中的固定禁动、仅观察及未授权表述属于历史；
在本地项目规则范围内，用户最新明确要求优先于项目文件和历史交接。





2026-11-02 用户要求结束并关闭Qt和导航。已正常live-stop，本次
session-20261102_171952-2的板端guardian退出码0；主Qt/导航/底盘/清扫节点停止，
独立YOLO、BEV、四路采集及预览Qt均inactive/MainPID0。四路退出后可能恢复的旧
j6m-b-camera-sender/link及板端receiver已一并停止；本机ZED与板端ISX锁实际可取得
并释放，11583/11587/18882～18885无监听，板端17441/17442亦无监听。两录制服务
及独立建图服务继续inactive/MainPID0。桌面室外落叶清扫包、室外积水清扫包、
室内垃圾定点清扫包均保留、recording_state=stopped、无.active；新静态图
map_20261102_171534及其他旧图/录包保留。证据validation/full_shutdown_20261102/。
当前整套Qt/导航/相机采集关闭，软件停止不代表相机断电；时钟服务保留。
未更改地图/算法参数、提交或推送，本条取代下方导航及相机仍运行的状态。

2026-11-02 用户“可以了结束了”。已正常停止室内垃圾定点清扫包的ROS与视觉/Qt录制，
两服务inactive/MainPID0；6个ROS包+5个视觉包+5段Qt录像，ROS跨度298.551秒（约4分59秒），
4692906269字节（约4.693GB），全部bag索引和视频容器检查通过，无.active残留。
按既有桌面保存偏好，完整实际目录已原子移动至/home/slam/Desktop/室内垃圾定点清扫包，
54文件移动前后inode/大小全部一致；内部visual/current用相对链接，项目名称入口和
localization/20261102_182750兼容链接保持可读。metadata记录stopped、not_evaluated及
用户原话，未自行判定成功/失败；说明和finalization_check.json随包，已sync落盘。
证据validation/indoor_garbage_spot_recording_20261102/finalization/desktop_saved.json。
当前没有实验录制；导航live276675、Qt和四路视觉继续运行，未另发清扫start、目标、
取消或速度。桌面落叶/积水包及其他地图数据保留，未修改算法配置、提交推送。
本条取代下方该包持续录制状态。

2026-11-02 用户要求现在录制“室内垃圾定点清扫包”。已18:27:50新建
rosbags/室内垃圾定点清扫包 -> localization/20261102_182750，ROS及视觉/Qt两服务active，
三秒实读ROS字节及Qt/四路/识别帧递增，错误均空。继续原导航
session-20261102_171952-2/live276675与map_20261102_171534，未因实验名称变更切换地图；
相机/BEV/YOLO不重启，Qt置前，未发送清扫start/目标/速度/取消。metadata记
in_progress/user_manual_only，当前地图及配置随包保存，结束后沿用桌面保存偏好。
证据validation/indoor_garbage_spot_recording_20261102/。当前持续录制等待用户结束，
桌面落叶/积水包保持，积水前两次已删除状态保持；未改算法配置/提交推送。
本条取代下方录制停止状态，清扫由用户手动开启。

2026-11-02 用户要求停止当前积水录包、保存到桌面并删除前两次。已正常停止两项录制，
均inactive/MainPID0；第三次181306录制为5个ROS包+5个视觉包+5段Qt视频，ROS跨度
242.597秒（约4分03秒）、4005793222字节（约4.006GB），bag索引/视频容器检查通过，
无.active。完整实际目录已原子移动至/home/slam/Desktop/室外积水清扫包，53个文件
移动前后inode/大小完全一致；visual/current为相对链接，项目rosbags/室外积水清扫包及
localization/20261102_181306兼容链接仍可访问。实查前两次180839和181043目录均不存在，
已按之前指令删除，本次没有误删第三次或室外落叶清扫包。metadata记录stopped/
not_evaluated（未收到成功/失败评价），说明及finalization_check.json随包保存，已sync。
证据validation/outdoor_water_cleaning_20261102/finalization/desktop_saved.json。
导航live276675、Qt及相机保持运行，未发送清扫start/取消/目标/速度；当前无实验录制。
本条替代下方持续录制状态；旧地图和其他录包保留，未改算法配置、提交或推送。

2026-11-02 用户“再来一次”，按紧邻上一轮删除当前积水录包后重录处理并已告知。正常
停止录制后仅删除第二段localization/20261102_181043及其名称/current链接，43文件
约2.054GB；首段180839此前已删除。18:13:06同名新建rosbags/室外积水清扫包 ->
localization/20261102_181306，两录制服务active，三秒ROS/Qt/四路/识别实读递增且无错误。
metadata保持in_progress/user_manual_only，当前地图/导航配置随包保存，结束后再封包
移至桌面。未重启导航session-20261102_171952-2/live276675或相机，未发start/目标/
取消/速度；桌面室外落叶清扫包及其他数据保留。证据
validation/outdoor_water_cleaning_20261102/deleted_second_take.json与restart2/。
当前新包持续录制，本条取代第二段进行状态；未改算法/地图/配置或提交推送。

2026-11-02 用户要求刚才积水录包不要、删除后重新开始。已正常停止两项录制，核实
inactive/MainPID0及无.active后，仅删除室外积水清扫包首段localization/20261102_180839
及对应名称/current链接，41个文件约1.706GB，证据validation/outdoor_water_cleaning_20261102/
deleted_first_take.json。18:10:43同名重新开始，当前rosbags/室外积水清扫包 ->
localization/20261102_181043，两录制服务active，三秒实测ROS/Qt/四路/检测帧递增且
无错误。新metadata记录重录/in_progress、user_manual_only；结束后沿用桌面保存偏好。
当前导航session-20261102_171952-2/live276675、Qt/相机/地图保持；未发清扫start、
目标、取消或速度。桌面室外落叶清扫包及其他历史数据保留，未改算法参数/提交推送。
证据validation/outdoor_water_cleaning_20261102/restart/。当前持续录制，本条替代下方
首段录制进行状态；原首段已按用户要求删除，不得当成保留数据。

2026-11-02 用户指定“室外积水清扫包”并要求现在开始录包。已18:08:39启动
rosbags/室外积水清扫包 -> localization/20261102_180839，ROS/视觉Qt两服务active；
三秒样本ROS文件字节及Qt/四路/识别帧均递增，错误为空。当前完整导航
session-20261102_171952-2/live276675、map_20261102_171534及相机/BEV/YOLO保持，
Qt置前，未发清扫start/目标/速度/取消。metadata为in_progress、user_manual_only，
assets含当前静态图及导航配置；沿用上一包桌面保存偏好，preferred_final_directory为
/home/slam/Desktop/室外积水清扫包，当前正在项目路径写入，结束后再正常封包和移至桌面，
不可在录制中移动目录。证据validation/outdoor_water_cleaning_20261102/及包内检查。
当前持续录制，等待用户结束；上次桌面室外落叶清扫包保留，未改算法/地图/参数或提交推送。
本条取代下方录制停止状态，清扫仍由用户手动开启。

2026-11-02 用户结束录包，指定命名“室外落叶清扫包”并要求保存在桌面。已正常停止
室内定点清扫2的ROS/视觉Qt录制，两服务inactive/MainPID0；7个ROS包+7个视觉包+7段
Qt录像，ROS跨度362.443秒（约6分02秒）、约6.215GB，所有bag索引/视频容器检查通过，
无.active残留。完整实际目录已在同一文件系统原子移动到/home/slam/Desktop/室外落叶清扫包，
59个文件移动前后inode/大小逐项一致；内部visual/current改成相对链接。项目入口已改名
rosbags/室外落叶清扫包，原物理路径localization/20261102_174852保留兼容软链接指向桌面，
runtime/localization_recording/current及旧绝对引用仍可解析。recording.json记录新名称、
原名称及stopped，实验结果not_evaluated（用户未确认成功/失败），封包/桌面保存证据在
validation/indoor_spot_cleaning2_20261102/finalization/，包内保留finalization_check.json及
录制说明.txt，已sync。新静态地图及所有原始数据保留；live276675、Qt与相机继续运行，
未另发清扫start/取消/目标/速度，未修改算法参数。本条取代下方持续录制状态；未提交推送。

2026-11-02 用户“算了，现在开始录包，开始后告诉我”。已17:48:52新建
rosbags/室内定点清扫2 -> localization/20261102_174852，ROS及视觉/Qt两服务active；
三秒实读ROS文件字节、Qt/四路RGB/检测帧均递增，视频及两视觉流错误为空。保持当前
session-20261102_171952-2/live276675、新静态图map_20261102_171534及原相机/BEV/YOLO，
未因上一轮高度讨论修改地图/参数，也未重启导航。Qt已置前显示；本包assets保存当前
完整静态地图及独立导航配置，metadata记in_progress、user_manual_only。助手未发清扫
start、目标、速度或取消；仍由用户手动开始清扫，等待用户通知结束录包和评价结果。
证据validation/indoor_spot_cleaning2_20261102/及包内recording_initial_check.json。
当前持续录制，本条取代下方录制停止状态；旧包/地图保留，未提交推送。

2026-11-02 用户询问新静态地图中央蓝色大块障碍（现场只有树叶）。只读分层核查确认：
ROI map X[3.5,12.5)/Y[6,12)m中静态障碍624格，原二维scan图仅61格，三维切片新增563格
（90.2%）；实时膨胀半径1.2m使它连成大片蓝色。建图固定Z切片[-.6,-.2]未随局部地面变化。
PCD低表面RANSAC估计中心(8,9)地面Z约-.410，4351个切片点中85.6%在拟合地面±.15m内，
说明该固定带已包含地面附近点；拟合不是独立地面测量/树叶语义真值。原实时scan筛选仍为
base_link Z[.4,.8]，不要将问题等同实时上限过高或只缩膨胀半径。诊断/分层对照图在
validation/indoor_map_obstacle_review_20261102/，含static/global数组、原PCD分析及统计。
本轮未更改地图、参数、算法或重启，未发运动命令/清扫start；有图Qt导航继续运行、录制
保持停止。若后续要求修复应优先按局部地面相对高度重新生成候选静态图，保留墙柱等障碍
及原图；原始三维PCD可用于离线验证，不直接把低点全部删成自由区域。

2026-11-02 用户确认建图完成，要求用刚建静态地图完整打开Qt。已向核实归属的mapper
脚本254570发送TERM并正常保存，建图服务inactive/MainPID0；新图
 global_maps/map_sets/map_20261102_171534的manifest complete，三维714874点、二维
668×461格/0.10m（含未知外接66.8×46.1m），整套11文件15152943字节。部署至板端
/map/conbine_navigation/maps/map_20261102_171534，11文件SHA-256全部一致，旧图不变。
新config/indoor_mapping_20261102.json独立指定该图；重启前实测camera_init/base
(6.76259,2.18614)、bodyZ .389120仅作ICP初始化种子。正常停止无图旧会话后启动
session-20261102_171952-2/live276675、完整有图Qt/导航/底盘及BEV清扫节点；相机
42125/BEV42127/YOLO42132连续、深度NONE。发/initialpose后实读LOCALIZED
(overlap .9492/rmse .1491/inliers4555)，是算法匹配指标而非独立精度。Qt已最大化前台，
真实截图确认新图/BEV；地图/定位/底盘/避障ready，规划参数事务已同步，局部代价地图
收到，cleaner IDLE/active=false/navigation_available=true，覆盖IDLE。未发运动目标或
清扫start；两录制服务继续inactive。用户手动开始清扫规则保持；本条替代上条建图进行
状态。复开入口：bash scripts/conbine_live.sh start --config config/indoor_mapping_20261102.json。
证据validation/indoor_mapping_20261102/，包含保存/校验/位姿种子/定位/实际Qt/状态；
原地图和室内定点清扫1保留，未修改导航算法或标定文件，未提交推送。

2026-11-02 用户要求先停录包、重新建图，等其通知建图结束后用新地图打开Qt。已正常停止
室内定点清扫1，两录制服务inactive/MainPID0；25个ROS包+25个视觉包+25段Qt录像，
ROS跨度1467.761秒、约27.812GB，全部bag索引及视频容器核查通过，无.active；实验结果
未评价，旧自动清扫回执仍保留。随后正常live-stop，启动无图session-20261102_171409-2/
live247924及Qt；不加载旧图，无自动清扫节点或新目标，四路42125/BEV42127/YOLO42132连续。
用户再次“开始建图”后17:15:34正式启动既有global_mapping_session.sh，独立服务
 autolabor-conbine-static-mapping.service（MainPID254569 bwrap，脚本254570，voxel255312，
 grid255313），输出global_maps/map_sets/map_20261102_171534。实际RECORDING，历史持久
点9546、camera_init坐标，Odometry/scan时效约.135秒。复用当前TF的body->base
(-.011,-.02329,-.60588)及当前车体裁剪前1.22/后.52/半宽.40，仅本次环境传参；原算法和
标定文件不改。二维扫描地图及三维持久体素正累积，融合地图在正常结束时生成。
当前不录包，等待用户明确建图结束；到时先向经来源核实的脚本254570发送TERM触发
正常保存（不能先停ROS或强杀整个进程组），等manifest complete后部署独立地图配置，
再打开有图Qt，保留旧地图。Qt本身未管理该独立mapper，不要用Qt建图按钮重复启动。
证据validation/indoor_mapping_20261102/，运行日志runtime/indoor_mapping_20261102/。
不擅自停止建图或开始清扫；未提交推送。本条替代下方录制进行和静态旧图运行状态。

2026-11-02 用户最新明确清扫由其手动开启，助手只确认录包；此要求替代之前自主开始和
“重新开始冲”的授权。室内定点清扫1从16:49:01持续录制，路径localization/20261102_164901，
ROS96859及视觉/Qt96892均active，四路/BEV后处理/Qt约5fps写入且错误为空。主导航
session-20261102_164336-2/live68309、室内8月地图、Qt及相机不重启。此前按原授权16:50:06
启动第一轮，记录1次轨迹通过后FAILED（TEB无法找到可用控制，12个未完成）；不等于拾取成功。
16:51:13重试因无可见目标拒绝，随后短时观察在16:51:49检测到目标后已接受再启动，该轮
实际PARTIAL/active=false、0通过、1个目标因无无碰撞路径跳过。观察进程已退出，无后台
自动重试；收到最新要求后未再发start、取消或速度。当前清扫无活动任务，由用户在Qt手动
开始，录制继续，实验结果仍in_progress。证据validation/indoor_spot_cleaning1_20261102/
及包内manual_start_handoff.json；metadata已改user_manual_only，原启动回执保留。
未改算法、阈值或标定，未提交推送。本条取代下方未录制状态。

2026-11-02 用户明确要八月底室内通道地图。已正常停止地下车库会话，改用原
config/conbine_live.json启动session-20261102_164336-2/live68309，实际地图
map_20260829_221335_gimp_cleaned，1311×670格、0.1m，原地图校验通过。Qt前台综合页
已显示室内通道地图；用户在Qt设置初值后16:44:44实读LOCALIZED（后续overlap .9903、
rmse .1636、inliers4782），规划参数同步成功，覆盖IDLE/active=false且地图/定位/底盘/
避障均ready。助手只切换显示页，未代发初值/目标/清扫start。四路42125/BEV42127/
YOLO42132连续运行、深度NONE；两录制服务仍inactive/MainPID0。证据
validation/indoor_map_qt_start_20261102/，含真实地图截图及localized_status.json。
当前沿用室内通道有地图模式；本条取代下方地下车库地图及等待初值状态。
未改算法配置、地图或录包，未提交推送。

2026-11-02 用户要求打开有地图模式。已正常停止上述无地图live/Qt并用
config/garage_manual_coverage.json启动session-20261102_164145-2/live58817，地图为
garage_mapping2_20261101，不加载固定全场覆盖计划。Qt已最大化前台显示整张地图，
/map=926×1147、0.1m、frame=map，static_map_mode=true；四路42125/BEV42127/YOLO42132
保持连续运行。实际定位WAITING_INITIAL_POSE，覆盖IDLE/active=false/map_ready=true，
底盘/避障ready，等待用户按真实位置和朝向设置初始位姿；未套用昨日XY或发送初值/目标。
move_base因map/base TF未建立等待，Qt启动参数同步超时已留证，未称规划完全就绪。
两录制服务inactive/MainPID0，未改算法/配置或新建录包。证据
validation/mapped_qt_start_20261102/。当前有地图Qt/导航与相机保持运行，未提交推送；
本条替代下方无地图运行状态，历史数据及F盘备份保留。

2026-11-02 用户要求完整打开Qt。沿用上次无地图模式启动session-20261102_163822-2，
live42161、四路42125/BEV42127/YOLO42132均active；主Qt已最大化并置于前台，真实桌面
确认实时点云、BEV及各页入口可见。MID360/scan/FAST-LIO与底盘数据收到，camera_init
至base_link TF时效约.058s，40×40m全局/20×20m局部滚动代价地图收到、motion_enabled=true。
四路两次采样帧号递增，CUDA YOLO11-GAM best6约4.5fps；ZED深度NONE、独立预览隐藏。
两录制服务inactive/MainPID0，未新建录包或发初值/导航目标/清扫start/速度。
证据validation/full_qt_start_20261102/。当前Qt、无地图导航及四路视觉保持运行；
原地图/录包/F盘备份保留，未改算法配置、提交或推送。本条替代下方全部关闭状态。

2026-11-01 用户要求把今天录包复制到F盘。当前接入的移动盘为Doribelove，exFAT、
UUID B405-D4B7，挂载/media/slam/Doribelove；已复制到其2026-11-01_实验录包目录。
12组、773个实际文件、122446454377字节（122.446GB），全部源/目标SHA-256一致；
逐文件大小/清单及源文件未改变检查通过，已sync落盘。exFAT不支持软链接，12个
visual/current别名保存为Linux软链接.tar（可移植相对目标）及JSON，真实take数据均已
复制且不重复；权限/时间在FILE_MANIFEST.json保留。目标含说明、SHA256SUMS及校验报告。
证据validation/data_copy_20261101/。原始数据、地图及成功标记保留；导航/相机继续关闭，
没有提交推送。本次复制已完成，没有后台复制任务。

2026-11-01 用户要求关闭导航。已正常live-stop，session-20261101_224840-2板端guardian
退出码0；主Qt/导航/底盘接口及YOLO、BEV、四路采集均停止，对应服务inactive/MainPID0。
旧sender/link和板端receiver保持inactive；本机ZED与板端ISX相机锁均实测可取得后释放，
11583/11587/18882～18885相关端口无监听，两项录制继续inactive。无地图导航1成功包
（894.397秒、约11.493GB）及其他录包、地图、配置保留，未改写原始数据。
证据validation/mapless_navigation_restart_20261101/shutdown/。当前完整导航、Qt与
相机采集保持关闭；软件停止不代表切断相机电源。未提交推送，本条替代下方运行状态。

2026-11-01 用户“成功,保存数据包”，随后询问大小和时长。已正常停止无地图导航1的
ROS及视觉/Qt录制，两服务inactive/MainPID0；15个ROS包、15个视觉包、15段Qt录像，
ROS跨度894.397秒（约14分54秒），11.493GB/10.704GiB。bag索引和视频容器核查通过，
无.active残留；rosbags/无地图导航1（localization/20261101_225002）的recording.json
及实验结果.txt标success，来源user_report。真实末段状态、录制日志及核查报告已保存，
不将用户反馈补造为独立轨迹精度验收。证据validation/mapless_recording1_20261101/finalization/。
无地图live663005及Qt保持运行，未启停相机或发送目标/速度，未修改算法配置。
当前无实验录制；旧地图/成功包保留，未提交推送。本条替代下方持续录制状态。

2026-11-01 用户明确“开始录制”。已于22:50:02新建rosbags/无地图导航1 ->
localization/20261101_225002，ROS670054及视觉/Qt670088均active，三秒采样ROS包
43.6->61.1MB、Qt帧30->45、识别帧33->49、四视图帧33->48，错误均空。
当前无地图live663005/session-20261101_224840-2、Qt及后台相机/BEV/YOLO保持运行；
未重启导航、代发目标或速度。recording.json记录mapless/in_progress，实读板端状态
存initial_live_check.json；证据validation/mapless_recording1_20261101/。
当前持续录制，等待用户结束并确认结果；本条取代下方未录制状态。既有成功包保留，
未改算法配置、提交或推送。

2026-11-01 用户要求重启完整导航展示无地图导航，并明确等他说录制才开始。已正常
停止旧live/Qt并确认板端guardian退出，使用原conbine_live.sh start --mapping启动
session-20261101_224840-2/live663005。不加载旧地图或静态定位器，未启动online-coverage；
实读Qt static_map_mode=false、导航坐标camera_init，move_base/TEB及底盘运动链路启用。
MID360/scan/FAST-LIO约10Hz、IMU约192Hz，camera_init->base_link TF时效约.039s；
40×40m全局滚动代价地图及20×20m局部代价地图收到。Qt真实桌面已显示实时点云、
四路BEV及YOLO，后台采集498654/BEV498657/YOLO498662连续保留；未弹独立相机窗口。
两录制服务inactive/MainPID0，没有新录包，必须等待用户明确开始录制。旧自选覆盖1
成功包及其他地图/录包保留。未发初始位姿/导航目标/非零速度或改变算法标定参数，
不将启动就绪称为无地图行驶验收。证据validation/mapless_navigation_restart_20261101/。
完整无地图导航与Qt保持运行，未提交推送；本条替代下方静态地下车库导航运行状态。

2026-11-01 用户“结束,成功”。已正常停止地下车库自选覆盖1的ROS及视觉/Qt录制，
两服务inactive/MainPID0；9个ROS包、9个视觉包、9段Qt录像，ROS跨度480.950秒，
约7.954GB，bag索引/视频容器核查通过，无.active残留。rosbags/地下车库自选覆盖1
（localization/20261101_223840）的recording.json及实验结果.txt已标success，来源user_report。
final_live_status.json保留停止前实读覆盖READY/active=false、定位LOCALIZED及零速度；
未将用户成功反馈补写为自动任务COMPLETE或覆盖率验收。证据
validation/garage_manual_recording_20261101/及包内finalization_check.json。
完整导航live598352与Qt仍运行，未另发取消/目标/速度，未启停相机或修改算法配置。
当前无实验录制；旧包及成功标记保留，未提交推送。本条取代下方持续录制状态。

2026-11-01 用户要求重启完整导航并自己选择区域覆盖。已正常取消旧PAUSED任务并封存
地下车库覆盖清扫1，保留用户success；15个ROS包+15个视觉包+15段Qt视频，897.264秒，
约15.245GB，索引/视频可读、无.active残留。旧live正常停止，guardian退出确认。
新增config/garage_manual_coverage.json，使用相同地下车库地图2及标定，去掉固定
coverage_plan_file；initial_body_z仅按重启前新鲜map/body实读-.201389设初始化种子。
新session-20261101_223810-2/live598352、主Qt已启动；四路498654/BEV498657/YOLO498662
保留连续运行。用重启前实测map/base(-3.1793,.7202,yaw约-.0248)作ICP种子，已LOCALIZED，
新/odom时效约.05s；覆盖IDLE/active=false、plan_id空，external_plan_file实读空，地图/
底盘/避障ready；宽1.2m/15%重叠及原运动参数保持。旧固定路线一键窗口保持inactive。
Qt已切清扫页并准备框定工具，由用户点选顶点、确认生成及开始；未代选区域/发新目标。
22:38:40新建rosbags/地下车库自选覆盖1 -> localization/20261101_223840，ROS601243与
视觉Qt601280持续录制，帧递增且无错误。README补充自选入口；证据
validation/garage_manual_restart_20261101/，旧包finalization_check及成功标记保留。
当前完整导航/相机/新录制保持运行；未改规划算法/安全阈值或板端current，未提交推送。
本条替代下方旧任务仍活动和旧实验持续录制状态。

2026-11-01 用户“可以,标记为成功”。已将rosbags/地下车库覆盖清扫1的recording.json及
实验结果.txt记为success，来源user_report。用户已点击独立一键按钮，StartCoverage实际
accepted=true（1793543277.710198）；标记时节点仍活动，状态PAUSED。因用户本轮只要求标记，
没有停止录制或取消导航；初始拟停止录制的说明已在操作前依据活动状态纠正。
ROS/视觉Qt录制及live均active，继续记录，success_report_snapshot.json保留当时完整状态、
定位、按钮启动回执、录像帧和服务证据。不将用户成功标记伪写为全场66.9%任务完成。
未修改规划/模型/配置，未代发目标/取消/速度；未提交推送。

2026-11-01 用户要求启动66.9%覆盖版、完整录制，并提供一键开始按钮。已启动真实会话
session-20261101_222206-2/live498690，Qt/四路498654/BEV498657/YOLO498662运行。
22:22:44开始rosbags/地下车库覆盖清扫1 -> localization/20261101_222244，ROS502279与
视觉Qt502312持续录制，帧递增且无错误。用上次实测位姿仅作ICP种子，首次短发布未到
localizer，保持发布器后实际接收并LOCALIZED（overlap约.999/rmse约.156-.158m）。
375段新计划在板端加载96.1秒，plan_id 8d26ac62d8824c34a3fc79d46a3b4d5f，READY/active=false；
定位、底盘、避障均ready。Qt已实际接收后端计划身份及1.2m参数。新独立左下角按钮
scripts/garage_coverage_button.py/.sh（service509677，PartOf=live）仅在用户点击时调用
原StartCoverage，一次点击、不弹二次确认；双击去重/禁用点击两项隔离检查通过。
旧Qt开始按钮仍保留原确认。完整导航/录制持续运行，助手未点击开始或发运动目标。
初始参数事务因等待map/base和TEB未就绪失败，定位恢复后重试成功；helper避免对
同值宽度重复提交defaults。证据validation/garage_coverage66_start_20261101/，真实按钮
截图、READY状态及录制状态已保存，计划/配置随本包保留；README已更新。
本条替代下方关闭状态。新代码随独立会话overlay加载，未改板端current，未提交推送。

2026-11-01 用户要求尽快得到车能行驶的避障覆盖路线，先允许80%，看到66.9%后明确
“66也够了”。已固定本版：375扫线/375真实连接，3640.51/5443.83m²（66.87%）；
全程约9483m/倒车1387m，回型主线+局部空地补线，非最短路线。
保持1.2m作业宽、1.94×1.00m含余量外廓及R>=1.35m；5cm采样、完整外廓内部/边界、
未知格/曲率/侧移及连续性检查通过，共252387位姿；面积为假定全程清扫的离线前刷并集，
不是实车清扫验收。停止未完成搜索，未把丢弃区域称已证明不可达。
外部计划v2保存真实曲线，生产加载器实读和控制器分段通过（1562执行段）；启动时仅重算
实时入场，后续复用曲线和原动态避障。缺少连接时不再画穿障碍的虚假Path直线。
Qt后台READY计划ID/实际参数同步已修复并构建；2项GUI、5项几何、5项导入/执行结构、
172项状态机检查通过。config/garage_navigation1.json改选新verified_plan.json；
start_garage_coverage.sh --plan-only预览后可Qt开始，默认则规划后请求开始，本轮未执行。
成果/报告/用户接受记录在validation/garage_drivable_coverage_20261101/；README/说明已更新。
导航/Qt/相机/录包继续inactive，未发新目标/速度/清扫start，无新板端部署，current未改；
上一条已关闭会话及封包结果保持。未提交推送，旧地图、录包、1027和其他改动保留。

2026-11-01 用户反馈Qt开始按钮变灰，核实急停已解除但GUI未关联后台plan_id；尝试在
Qt载入同框，实读规划失败为GUI旧清扫宽度与后台1.2m不一致，旧READY计划保持。
随后用户明确关闭导航，称路线不可行、部分位于障碍物内，再要求离线重规划，允许约80%
覆盖。已取消旧覆盖计划，正常封存完整导航1_正式（18个bag索引可读、无.active），
记录stopped_for_replanning及用户反馈；正常live-stop，板端guardian退出0。导航/Qt/
YOLO/BEV/四路采集及录包均inactive/MainPID0；四路退出自动恢复的旧sender/link及板端
receiver也已停止，时钟保留，ZED与ISX锁可取得释放，无11583/11587/18882～18885监听。
证据validation/garage_navigation1_shutdown_20261101/。目前全部保持关闭，不自动发车。
正进行独立离线规划：tools/garage_hybrid_offline.cpp调用已有Hybrid A*，Navfn只提供
绕障通道，各可执行连接重新用Hybrid计算；garage_route_validation.py独立检查完整
非对称车体、未知格、半径及横向/原地转动。旧494段有9段未通过更密检查；5项几何
检查通过。当前计算在validation/garage_drivable_coverage_20261101/，尚未最终完成。
已修改Qt源码以接收后端READY的计划身份与实际参数，当前正在正确依赖环境下构建和
验证，未启动新版Qt。旧蓝色直线参考连接不能当作可行路径，下一版需展示真实验证曲线。

2026-11-01 用户询问如何启动覆盖导航。核对发现Qt仅显示后台READY，内部coverage_plan_id
未关联外部计划，因此不能声称其开始按钮可启动。已新增scripts/start_garage_coverage.sh，
加载现有ROS环境并调用已保存plan_id的原StartCoverage入口；shell语法通过，未实际执行。
Qt已切清扫页；尝试载入同一框时界面正在交互滚动，实际未进入选区且未产生新计划，
不把这次尝试称Qt接管成功。已明确改用该一条命令，避免重启Qt打断正式录制，并说明
现有覆盖仅导航、不会自动开刷/风机/喷淋。README/说明及正式录包coverage_plan已同步。
正式录制持续，Qt823帧、四路与识别递增无错误；原READY计划/急停检查保持，未发start。

2026-11-01 21:46 用户要求关闭录包并重新录包，准备开始。已正常停止并封存完整导航1
（localization/20261101_213955），14个bag索引可读、无.active残留，约6.095GB；保留
原数据，metadata记录用户要求重录、准备段未评价，未标记成功或失败。21:46:08新建
rosbags/完整导航1_正式 -> localization/20261101_214608，ROS325894/视觉Qt325925 active，
两次采样Qt22→100帧、四路/检测递增且无错误，ROS active包增至约200MB，桌面已解锁。
原导航284725/Qt/相机/识别不重启，494段回型READY/active=false，ICP LOCALIZED，避障
新鲜；底盘仍报告遥控/车体急停。已告知录制就绪及现有急停状态，未发送start/目标/速度。
新包保留coverage_plan及initial_live_status.json，证据validation/garage_navigation1_record_restart_20261101/。
当前新录制持续进行，旧完整导航1及弓字准备/历史建图清扫包保留；未改算法、提交推送。

2026-11-01 21:43 用户要求用地下车库地图2启动完整导航1、自行匹配位姿、1.2m宽全场
覆盖，并追加必须考虑车体约束、采用回型。已结束旧在线建图会话及被取消后仍计算的
弓字准备任务，准备包保留为完整导航1_弓字准备。地图/PCD独立部署为
garage_mapping2_20261101，config/garage_navigation1.json通过新增--config选择，默认旧图
不变。新session-20261101_213932-2/live284725/Qt285469，原四路12869/BEV12872/YOLO12877
继续运行、深度NONE。用重启前实测map/base位姿(-1.89492,-.48778,yaw.00437)及推导bodyZ
-.16920播初值，ICP实际LOCALIZED，最近overlap.9988/rmse.1569/inliers4324；非独立精度。
回型离线生成、外部计划导入保序，494条有向清扫段逐点校验非对称外廓前1.32/后.62/
半宽.50，圈间距1.02、最小转弯半径1.35。首入场Navfn/TEB、段间Hybrid/TEB现算，未宣称
转弯全部验证；5项新测试和172项原状态机检查通过。实际plan_id ced4819ce25242e59c629ebe4e0f0245，
READY/active=false。参考清扫带约3094.69m²/已知自由格5443.83m²，仍有2349.14m²未覆盖，
不能称全车库覆盖完成或全部遗漏均不可达；蓝色参考连线不是已验证轨迹。底盘仍报告遥控/
车体急停，未对新回型发送start/运动目标，已请用户现场解除；避障新鲜，未改限速/急停。
rosbags/完整导航1 -> localization/20261101_213955，ROS及视觉Qt两服务持续录制。起初
锁屏，21:41后已解锁；Qt已点“显示整张地图”，实图见qt_full_route.png。路线/CSV/状态/
配置/说明保存在包内coverage_plan，证据validation/garage_full_navigation1_20261101/，
docs/GARAGE_CONTOUR_COVERAGE_20261101.txt及README已更新。当前导航/相机/录制保持运行，
等待急停解除后继续已授权导航；未实车验证回型转场或清扫完成，未提交推送，保留1027。

2026-11-01 21:15 用户确认地下车库建图2成功并要求记录大小。已正常停止ROS与视觉/Qt
录制，均inactive/MainPID0；recording.json/实验结果.txt标记success（用户确认）。共16个
ROS包、16个视觉包、16段Qt视频，ROS跨度955.413秒，约30.86GB/28.74GiB；索引/视频
核查通过，无.active残留。二维地图926x1147格、0.10m分辨率，外接92.6x114.7m（含未知），
PGM1062138字节。另从末包最后一帧/static_mapping/history_cloud导出完整持久体素PCD，
1587414点/31748491字节，与mapper的persistentPoints筛选相同，无额外降采样；坐标系
camera_init与本次map原点一致，来源/时间见pointcloud_snapshot.json。地图和PCD在包内
online_map_at_recording_end/，精确大小见大小统计.json；PCD为录制终点快照，未停在线mapper。
导航65352、Qt和相机/识别继续运行，未发取消/目标/速度或切换默认地图；当前没有实验录制。
证据validation/garage_mapping2_start_20261101/finalization_check.json与final_size_statistics.json。
第一次失败数据及清扫1～4保留，未提交推送。本条取代下方持续录制状态。

2026-11-01 20:59 用户要求第二次地下车库建图，完整导航与完整包，就绪后等待开始通知。
已启动新online-coverage会话session-20261101_205923-2/live65352，复用四路12869/BEV12872/
YOLO12877；未加载旧地图、未沿用覆盖框或平地先验。实际MID360/IMU/FAST-LIO及/map
新鲜，Qt地图READY，覆盖IDLE/active=false、定位/底盘/避障均true，cmd_vel采样0。
rosbags/地下车库建图2 -> localization/20261101_205932，自20:59:32开始录制，ROS66228/
视觉Qt66262均active；两次样本ROS包56→82MB、视觉53→72MB，Qt帧38→53，无视频或
视觉错误，桌面未锁屏。当前持续录制等待用户通知，未自动开始任务或发送导航目标。
证据validation/garage_mapping2_start_20261101/及包内initial_live_check.json/README。
第一次失败记录与清扫1～4保留，未改算法、提交或推送。本条更新上轮导航inactive状态。

2026-11-01 20:58 用户将地下车库建图1标记失败。此前状态询问发现Qt退出，主会话按既有
生命周期关闭，ROS master已退出；Qt最后有效帧20:55:45.730，视觉/BEV继续录制。现已
正常停止视觉录制，因master不存在向该包实际rosbag写入进程SIGINT封包，外层随后退出；
无.active残留，已有包/视频/在线二维地图及PCD/会话日志保留。recording.json和实验结果.txt
标记failure（用户确认），原因写明完整录制中断，Qt退出根因未确定，不推定用户关闭或崩溃。
导航和Qt保持inactive，未重启/发目标/速度；相机/BEV/YOLO仍运行，当前无实验录制。
证据rosbags/地下车库建图1/finalization_check.json及validation/garage_mapping_start_20261101/。
本条取代下方持续完整录制状态，保留清扫1～4，未提交推送。

2026-11-01 20:50 用户抵达地下停车库，要求完整导航、建图及完整录包，就绪后等其通知
开始。已启动online-coverage底层模式但未发送区域/平地先验或运动目标，不沿用旧场地
10x3框；会话session-20261101_205002-2/live12905，四路12869/BEV12872/YOLO12877。
实际MID360 CustomMsg、IMU、FAST-LIO位姿及在线/map正常；Qt地图READY，覆盖IDLE/
active=false、localized/chassis_ready/avoidance_ready=true，速度采样0。启动初期底盘参数
未回复及一条VCU0x24记录保留，末次状态已恢复ready，非行驶验收。CUDA best6约5fps、
深度NONE、四路持续出图，桌面未锁屏；Qt截图可见，终端有遮挡，顶栏旧录包指示不跟踪
后台服务。20:50:13开始rosbags/地下车库建图1 -> localization/20261101_205013，ROS14010/
视觉Qt14042均active，ROS/视觉active包和Qt帧持续增长无录制错误。当前持续录制等待
用户通知；未开始清扫或自行驾驶。证据validation/garage_mapping_start_20261101/及包内
README/initial检查。保留历史清扫1～4，覆盖旧包删除状态不变，未改算法或提交推送。

2026-11-01 用户要求关闭完整导航。已正常live-stop，板端本次guardian退出码0；主Qt、
导航、在线建图、覆盖控制及独立YOLO/BEV/四路采集均关闭，相关服务inactive/MainPID0。
四路退出后旧sender/link恢复运行，已一并停止并关闭板端receiver；ZED/ISX相机锁实测
可取得释放，11583/11587/18882～18885无监听，板端无本次会话或私有采集残留。
录制仍inactive，保持上轮覆盖录包删除结果；时钟服务、配置和历史清扫1～4保留。
证据validation/online_coverage_20261101/shutdown/。当前整套程序与相机采集保持关闭，
此条替代下方仍在运行的状态；软件关闭不代表切断相机电源。未提交推送。

2026-11-01 18:49 用户说明远程电脑没电，要求删除本次覆盖录制、稍后再开始。已正常
停止ROS和视觉/Qt两项录制，均inactive/MainPID0；删除覆盖清扫1及对应准备段的别名与
实际目录localization/20261101_182914、20261101_182438，共约26.63GB，清除指向该包的
runtime/localization_recording/current。此前清扫1～4完整保留，覆盖功能和配置保留。
导航/Qt/相机未另行启停，未发送清扫start或运动目标，当前没有实验录制；待用户通知
重新开始。本条取代下条持续录制状态，证据validation/online_coverage_20261101/recording_deletion.json。

2026-11-01 用户要求新场景在线建图定位与覆盖清扫，确认长10m沿车头、宽3m，中心
前8m/左3m，并由用户在Qt点击开始；随后确认区域平地。已新增online-coverage模式，
本次FAST-LIO原点为map，不加载旧地图/旧定位器；实时扫描建图、既有覆盖/Hybrid/TEB/
底盘链路启用。任务按会话固定身份，在线地图增长不清空任务。仅确认框内无障碍证据
的未知格采用可通行先验，真实障碍及候选保留，原始观测地图另存；框外未知保持。
入口scripts/online_coverage.sh自动录制后画框，Qt预设0.80m作业宽/15%重叠并生成预览。
6项新检查、172项原状态机检查、三模式launch展开及Qt构建通过。实际会话
session-20261101_182905-2/live523443，覆盖READY/active=false，定位/底盘/避障均true；
刷新同一固定框后9条清扫线、30/30平方米、预计144.6秒，命令采样为零。未发清扫start
或运动目标，尚未实车行驶验收。rosbags/覆盖清扫1指向20261101_182914，ROS523742及
视觉/Qt523778持续录制；准备段另保留覆盖清扫1_准备。相机396997/BEV396999/YOLO397004
保持运行，ZED深度NONE。最后检查桌面LockedHint=yes，最终Qt截图黑屏，已请用户解锁；
不能将锁屏段宣称有效Qt画面。记录docs/ONLINE_COVERAGE_20261101.txt及
validation/online_coverage_20261101/。保留旧地图、清扫1～4及1027，未提交推送。

2026-11-01 用户结束清扫4并要求标记成功。已正常停止两项录制服务，inactive/MainPID0；
134.225秒，3个ROS包、3个视觉包、3段Qt视频，约1.98GB，索引/视频核查通过，
无.active残留。rosbags/清扫4/recording.json及实验结果.txt标记success（用户反馈）；
末段cleaner实读IDLE/active=false、计数0已存final_live_status.json，未补造自动任务记录。
证据finalization_check.json及validation/cleaning4_recording_20261101/。导航397034、
YOLO397004、BEV396999、四路396997仍active，未另发目标/取消/速度；当前无实验录制。
清扫1/2/3及清扫2工作流保持，未修改算法、提交或推送。

2026-11-01 用户“开始清扫4录制”。已于18:10:36启动，rosbags/清扫4 ->
localization/20261101_181035，ROS录制420455、视觉/Qt录像420491 active；两次采样
四路/检测/Qt帧数递增无错误，ROS active包已写入约61MB。保持原live397034、
YOLO397004、BEV396999、四路396997，不重启导航/相机、不代发初值/目标/清扫start。
证据validation/cleaning4_recording_20261101/，当前持续录制，实验结果待用户确认；
清扫1/2/3数据及此前成功标记均保留。未修改算法、提交或推送。

2026-11-01 用户“结束,标记成功”。已正常停止清扫3的ROS及视觉/Qt录制，两服务inactive/
MainPID0。共151.323秒，3个ROS包、3个视觉包、3段Qt视频，约2.22GB；bag索引和视频
核查通过，无.active残留。rosbags/清扫3/recording.json及实验结果.txt标记success，
来源明确为用户反馈；末段cleaner实际IDLE/active=false、计数0保存在final_live_status.json，
未据现场成功反馈补造自动任务记录。证据finalization_check.json及
validation/cleaning3_recording_20261101/。导航397034、YOLO397004、BEV396999、
四路396997保持active，未取消清扫/发目标或速度；当前无实验录制，等待后续要求。
清扫1/2及清扫2固定工作流均保留，未修改算法、提交或推送。

2026-11-01 用户“开始清扫3录制”。当时实车与回放均inactive，已恢复既有完整实车入口：
session-20261101_180641-2，live397034、后台四路396997/BEV396999/YOLO397004，
原地图/单MID360/CUDA best6/深度NONE不变，不弹独立相机窗口。18:06:55开始清扫3，
rosbags/清扫3 -> localization/20261101_180655，ROS录制398574、视觉/Qt录像398605
active，四视图/检测/视频计数递增且无错误，实际MID360/IMU收到3/41条样本。
Qt地图READY，定位WAITING_INITIAL_POSE，cleaner IDLE/active=false、navigation_available=false，
已通知用户录制开始并需在Qt设置初始位姿后清扫。未代发初值/目标/清扫start/速度。
initial_live_check.json及validation/cleaning3_recording_20261101/保留启动证据，当前持续
录制等待用户结束；清扫2固定回放工作流及原包不变，回放保持inactive。未提交或推送。

2026-11-01 用户要求忽略未对齐的旧全局地图，重新定位建图并回放导航；随后确认效果好，
要求随清扫2保存工作流。已新增mapping回放：原始MID360/IMU重跑FAST-LIO，camera_init
局部坐标、包内body/base外参、体素历史地图及现有无图move_base/TEB代价地图，Qt同步
录制的四路识别/BEV；旧地图/定位/TF/规划输出不重播。完整135.728s实际1346帧新位姿，
预览120885点，正常退出PCD121423点，110次局部代价地图；不是闭环驾驶仿真或米制精度
验收。证据validation/cleaning2_mapping_replay_20261101/，原源码二进制只读复用并私有复制。
已保存rosbags/清扫2/replay_workflow/，含固定脚本/Qt/FAST-LIO/mapper/TEB/配置、原参数、
参考PCD与日志/验证、文件SHA；入口“回放局部建图.sh”默认mapping，可暂停/重播/关闭。
包内固定版本通过16项只读挂载启用，实测启动及从头重置通过，重播新位姿回到局部原点；
仍依赖本机Jetson/ROS/现有依赖，不称跨平台独立包。原始6个bag及3段视频未改写。
当前回放service346773 active，session_20261101_175337，已通过包内入口启动；真实导航/
YOLO/BEV/四路采集继续inactive，无实车目标或速度。该条取代下方纯录制回放运行状态；
README及录包说明已更新，保留1027和其他改动，未提交或推送。

2026-11-01 用户要求关闭导航并复现清扫2。已正常live-stop并获板端guardian退出0，
停止实车YOLO/BEV/四路采集，四服务inactive；新建scripts/replay_cleaning_experiment.sh
及replay_cleaning_session.py，回放服务307243 active，最终会话
runtime/cleaning_replay/session_20261101_174503。独立bwrap网络仅lo，无串口/CAN设备，
私有11593 ROS master、18894 HTTP；图中数据来自录包，未运行定位/规划/清扫/识别后端。
原Qt显示地图/点云/TF/四路识别/BEV，另有暂停/继续/从头播放/原始Qt录像控制窗口。
BV02 JPEG、几何、检测不变，适配Qt墙钟时效与回放ID，保留原时间；从/localization
累积黄色估计轨迹（非规划路径/真值）。实际地图READY、帧递增，11项Qt报文检查和
报文等价/暂停/继续/重播核对通过；实读节点仅Qt/player/bridge/rosout，当前原速播放。
包内清扫节点全程IDLE、定位全程LOST，未录到自动清扫任务/规划路径，已向用户说明；
用户现场成功记录仍保留，未伪造任务终态。证据validation/cleaning2_replay_20261101/，
README同步回放入口。未修改原录包、导航算法/标定，未发实车目标或速度，未提交推送。

2026-11-01 用户确认“清扫结束”“成功”。已正常停止清扫2的ROS与视觉/Qt录像服务，
两服务inactive/MainPID0，3个ROS包、3个视觉包、3段Qt视频均封包保留，无.active残留。
rosbags/清扫2的recording.json与实验结果.txt记录成功，明确来源为用户现场反馈；
final_live_status.json保留停止时cleaner IDLE/active=false及定位LOST实读，不将用户
成功反馈等同独立定位/拾取指标验收。finalization_check.json记录索引/视频核查。
导航、Qt、相机、YOLO保持运行，未另发取消/目标/速度，未修改算法、提交或推送。
当前无实验录制，本条替代上条清扫2持续录制状态；清扫1失败资料继续保留。

2026-11-01 用户“开始清扫2”，按连续录制上下文新建实验录包，未替用户点击清扫。
rosbags/清扫2 -> localization/20261101_173738；ROS267739与视觉/Qt录像267771
均active，四路/识别帧及Qt录像持续写入、无错误；清扫1失败资料原样保留。
启动实读cleaner IDLE/active=false、可见0，定位LOST，已提示先恢复定位再开始清扫。
导航/Qt/相机/YOLO未重启，未发初值/目标/速度，当前清扫2持续录制，等待用户停止。

2026-11-01 用户“停止,记录失败”。已停止清扫1的ROS与视觉/Qt录像服务，均inactive/
MainPID0，所有bag封包无.active残留；本次结果写为失败（用户报告，具体原因待复盘），
原数据保留rosbags/清扫1，结果见实验结果.txt、recording.json，封包核查见
finalization_check.json。停止前cleaner为IDLE/active=false，因此未另发取消/速度。
导航、Qt、后台相机/BEV/YOLO保持运行。正常退出后systemd --collect已卸载录包unit，
旧脚本末尾重复stop报不存在；已改为仅对active unit停止，补齐会话日志保存，未影响
bag索引或连续录制。当前无录制任务，本条替代上一条“持续录制”。未提交推送。

2026-11-01 用户准备清扫，要求完整录包供后续Qt显示，并指定命名“清扫1”。已开始
独立全ROS录制220168及新增HTTP/Qt录像服务227717，持续运行，未结束录制。命名入口
rosbags/清扫1 -> localization/20261101_172946；ROS从17:29:46开始，修正兼容后的
视觉take从17:31:05开始，已明确通知用户录制就绪。新入口
scripts/record_cleaning_experiment.sh start [name]/status/stop，stop只封包不停止清扫。
保存MV01四路RGB/融合图、BV02同组四路识别及BEV报文到独立visual_*.bag，保留时间戳/
帧ID/SHA和配置；Qt窗口录像H264/5fps，每60秒分段。不是原生相机帧率同步无损采集，
互动式Qt回放尚需独立ROS/HTTP回放适配，录像可直接播放。Qt自带录包指示不跟踪该服务。
最初str.removesuffix与本机Python3.8不兼容已修复并只重启视觉录制，初次文件保留在
visual/根目录，当前正常take在visual/take_20261101_173105/；全ROS录制未中断。
已检查封包读取、报文SHA及视频编码，实时无流/录像错误。导航/Qt/YOLO未重启、未发
初值或清扫start；读到定位LOST已提示用户先确认恢复，清扫当时IDLE/可见目标存在。
README与录制目录自述已更新，当前仍持续录制，用户将通知何时停止；未提交推送。

2026-11-01 用户要求启动完整导航，并明确只在Qt显示融合BEV，不再弹出独立相机画面。
已启动真实会话session-20261101_165120-2，live20894、Qt21726、YOLO20843、内嵌
cleaner22220；四格采集39054/BEV39056以新增--headless运行，不构造GTK窗口，
bev_perception.sh默认后台模式，独立预览入口保留原用法，README同步说明。
启动时旧板端相机server777311残留持锁，经原stop.py正常结束后恢复；三组四路源
序号递增、CUDA best6/live、ZED NONE，实际Qt截图确认融合与识别画面持续显示，
窗口树无独立四格/BEV窗口。地图READY、CAN和MID360正常，motion_enabled=true；
FAST-LIO仍WAITING_INITIAL_POSE，move_base在等待map/base TF，尚不能宣称规划已就绪。
需用户在Qt设置真实初始位姿；未代设初值/发目标/开始清扫，清扫IDLE/active=false。
两Python语法、三shell语法、四路配置检查与在线链路核对通过；仅改显示启动模式，
模型/标定/地图/清扫算法未改，HEAL保持inactive。证据
validation/full_navigation_start_20261101_1651/。本条替代此前PID状态，保留1027及
其他工作树改动，未提交推送。

2026-11-01 用户进一步要求整个程序含Qt全部重启。已正常live-stop并收到旧板端退出
确认，再停止YOLO/BEV/四格，随后重新启动全部。新真实会话session-20261101_160719-2，
live2143194、Qt2143993、四格2143154、BEV2143156、YOLO2143166、内嵌cleaner2144162；
旧独立cleaner service已随旧live停止，勿误当新清扫节点未运行。Qt地图READY，三组
四路源序号递增，CUDA best6/live、ZED NONE；清扫IDLE/active=false、记忆字段存在。
原8月29日地图/单MID360/motion_enabled=true保持；FAST-LIO为WAITING_INITIAL_POSE，
需要用户在Qt重新设初始位姿。底盘反馈零速、指令链路新鲜，启动有一条既有VCU0x24
超时记录已留证；未代设初值/发目标/开始清扫或恢复旧任务。HEAL保持inactive。
证据validation/full_program_restart_20261101/，本条取代上轮“保留原Qt/导航定位”的
运行状态；本轮只启停程序及记录状态，未修改算法参数、提交或推送。

2026-11-01 用户要求重启实验。已正常重启几何BEV2134569、YOLO2134672及新版记忆
清扫节点2134848；Qt/导航1467434和四路采集1013642保留。3组四路帧序号递增、
CUDA best6约4.70fps、ZED深度NONE，map/base TF有效；清扫IDLE/active=false，
pending=0/visible=0，导航可用，待用户摆放垃圾并点击开始。没有恢复旧轮次或发送
清扫start/目标/速度，HEAL试验保持inactive。证据validation/bev_memory_restart_20261101/。
本条更新上述程序PID；未修改算法/参数或提交推送。

2026-11-01 用户决定继续YOLO清扫，反馈两件垃圾只扫一件、另一件移动中漏检后任务结束。
旧终态实读COMPLETE/通过1/跳过1；代码有2秒失检删除、接近后2秒未重检即跳过两处。
已增加任务内map目标记忆：当前可见仍2秒时效，本轮pending不因失检过期，规划/结束
依据pending；重检优先校正，仍漏检则按最后记录坐标清扫，保留碰撞及实际刷子轨迹判据。
未完成终态改PARTIAL，新增pending/remembered数量、未完成原因及逐目标日志；Qt通过
已有reason显示待处理/漏检数，无需重编重启。结束/取消后不建长期地图登记，下轮从当前
观测新建任务。19几何/关联+12隔离ROS检查通过，模拟第二个失检7.6s仍两目标通过。
只重载cleaner为2038256/IDLE、active=false，旧计数清零，未自动恢复旧任务。
原BEV窗口当时已停导致YOLO等待图像，恢复几何BEV2026854；Qt/导航1467434、四格
1013642、YOLO1875377不变，实读CUDA best6/live约4.07fps、当次0目标。HEAL试验
inactive/MainPID0，BPU锁可取得释放；源码资产保留。未发目标/速度/清扫start，原
模型/标定/遮罩/深度NONE/导航参数保持。尚待本场景实车拾取验收，非已实测扫净两件。
说明docs/BEV_TASK_MEMORY_20261101.txt，证据validation/bev_task_memory_20261101/。
本条替代下方HEAL预览active及旧cleaner PID；保留1027与其他未提交改动，未提交推送。

2026-11-01 用户提供Downloads/Heal-main.zip并要求部署BPU BEV看效果。已校验解压到
runtime/heal_source_20261101/Heal-main，实际HEAL0.6.0/configs/bev_vtv2_resnet50；
源码权限不再阻塞。新增独立HEAL预览，直接复用官方纯几何/中心解码方法，四路真实RGB
按各自标定缩放裁剪生成S16采样索引，官方六槽位HBM的两个空位在几何上排除。
J6M私有DNN/UCP3.15.8/HBRT4.11.11及常驻tensor executor实跑，14输入/6输出匹配；
后视沿用已有白色车体像素遮罩。四路逐一遮挡均改变输出，两个空位改图输出差0。
100帧实际输入profiling单BPU子图平均16.556ms；常驻调用中位17.241ms，含SSH98.756ms。
3项几何/量化/解码检查通过；固定回放组车旁0个>=0.1候选，全大网格39个远处自行车
pre-NMS候选。随后现场换位，在线出现bus误检；5527/5535/5542/5551组递增、请求到
解码中位195.12ms。不能当垃圾或定位精度通过。当前是官方交通权重的四实拍输入试验，
不是已训练/编译四路垃圾模型，不是新的RGB拼接；尚未部署官方旋转NMS。
独立autolabor-conbine-heal-bev.service/GTK窗口保持active，scripts/heal_bev_preview.sh
可start/stop/status，PartOf=camera-bev，关闭独立窗口也结束该BPU worker。
原Qt/导航1467434、采集1013642、BEV1875372、YOLO1875377及cleaner1712302未动，
原YOLO继续CUDA，ZED深度NONE；未发目标/速度/清扫start或新map垃圾点。
原生x86训练环境、同步米制垃圾标注、正式四路细网格垃圾HBM及地图精度仍待完成。
说明docs/HEAL_BEV_BPU_PREVIEW_20261101.txt，最终证据validation/heal_bpu_masked_20261101/，
初始未套新试验遮罩对照validation/heal_bpu_real_20261101/保留。未提交推送，保留1027。

2026-11-01 用户要求范围按车体四边向外各5m。已新增四向distance_from_edges_m配置，
物理边界front1.22/rear.52/left,right.40，实际观测X[-5.52,6.22]/Y[-5.40,5.40]m，
不包含导航padding。保持600方形等比例显示，侧向画布留白明确排除观测，GTK按边缘显示。
12项定位/边界检查及11项真实报文Qt检查通过；旧配置前后投影像素完全一致。旧投影组
11/12通过，固定点/浮点remap参考一项在修改前代码同样失败，未放宽或称全通过。
新版BEV1875372及YOLO1875377 active，四格1013642、Qt/导航1467434、cleaner1712302
不变；约4.7fps、投影31ms。五组实际边界验证通过，采样无目标，非5m精度验收。
未发初值/目标/清扫start/速度/机构命令，标定/模型/遮罩/深度关闭/导航配置保持。
说明docs/BEV_EDGE_RANGE_20261101.txt，证据validation/bev_edge_range_20261101/。
本条替代下方中心四向4m范围，保留1027及原改动，未提交推送。

2026-11-01 用户反馈后视不能点击清扫。后视plastic已识别，地面估计(-2.29,+.27)m，
但原BEV仅后2m，outside_bev_range使visible_count=0；非车体遮罩或导航未就绪。
已将四路范围统一X/Y[-4,4]m，600x600、1.333cm/像素，原图二维投影坐标不变；GTK
范围文字改读配置。中间600x800方案被Qt固定尺寸校验拒绝，已改上述正方形显示兼容，
未声称Qt支持可变尺寸。真实同帧回放及11项Qt报文检查通过，最终后视候选1个、
active=false、导航可用。仅重开BEV最终PID1811032；采集1013642、YOLO1473043、
Qt/导航1467434、cleaner1712302不变。未发清扫start/初值/目标/速度/机构命令，
未验证后视路径与拾取；原模型、深度关闭、遮罩、标定、导航参数保持。
说明docs/BEV_REAR_RANGE_20261101.txt，证据validation/bev_rear_range_20261101/。
本条替代旧BEV后2m/左右3m/1cm显示范围，保留1027及原改动，未提交推送。

2026-11-01 用户反馈前方已识别目标却不可达。实读front plastic约(3.60,+.56)m，
LOCALIZED/导航可用；发现move_base/GetPlan的Path外层frame为空、逐点均map，清扫节点
错误拒绝有效路径。已兼容外层空header，在逐点坐标系/数值/端点检查后补map，未改外廓、
避障或通过阈值。真实22点报文回放、5类异常拒绝、16项几何及11项隔离ROS检查通过。
只正常重载BEV清扫节点为autolabor-conbine-bev-cleaner.service/PID1712302，PartOf=live；
Qt/导航1467434、YOLO1473043、采集1013642、BEV1013644不变。现场实际_choose只读查询
选到约4.50m接近+通过路径，跳过为空；真实节点IDLE、可见1目标、导航可用，等待重新开始。
用户此前自行初始定位/执行并取消过覆盖；本轮未发初值/目标/清扫start/速度/机构命令。
说明docs/BEV_UNREACHABLE_FIX_20261101.txt，证据validation/bev_unreachable_20261101/。
本条替代下方等待初始定位及旧清扫节点状态；未提交推送，保留1027及原改动。

2026-11-01 用户要求现在开始测试。已停私有Qt/11587，live-start真实导航会话
session-20261101_135447-2（PID1467434），原地图/单MID360/motion_enabled=true；采集
1013642及BEV1013644不变。独立YOLO最初缺ROS Python依赖，bev_perception.sh新增run
加载现有环境，修复后PID1473043 active；隔离live内部systemd调用失败，本次用宿主
bev_perception.sh start恢复，该内部调用限制仍保留。四路CUDA best6约4.7fps、10组0目标。
起初雷达/CAN无数据，随后恢复，实读底盘/避障ready=true、MID360约10Hz、FAST-LIO健康。
实际刷子参考1.22、雷达Z.65、bodyTF Z-.60588及新外廓均已加载。最后仍等待真实初始位姿，
map/base未接通、清扫IDLE；未代发初值/目标/清扫start/非零速度/机构命令，10秒采样命令
均零。Qt/导航/相机/识别保持运行，尚未完成行驶/拾取验收；未恢复深度或换模型。
说明docs/BEV_CLEANING_TEST_START_20261101.txt，证据validation/bev_cleaning_start_20261101/。
本条替代下方导航inactive的运行状态；未提交推送，保留原工作树及1027。

2026-11-01 用户确认刷子在装置最前沿，base_link离地实测25cm，并要求核对官网。
官方参数/尺寸图确认M2原长1040宽700高430mm、离地间隙110mm，但尺寸图/ROS说明/
官方241125驱动均未定义base_link离地25cm；该值作为用户指定参考点实测值使用，
不称官方确认。已设两清扫配置参考1.22m，两个控制器去掉旧1m上限、保留有限非负
校验；MID360 Z=0.65m、FAST-LIO body->base Z=-0.60588m，平面及外廓不变。
16项几何、两模式launch展开、隔离旧视觉控制器初始化通过；Qt/识别重开1438895，
实读BEV清扫参考1.22/IDLE，采集1013642及BEV1013644不变。真实导航inactive，无
目标速度机构命令；雷达外参下次live-start加载。未接通四路无深度连续视觉伺服或
验证拾取。说明docs/VEHICLE_CONTACT_HEIGHT_20261101.txt，证据validation/vehicle_contact_height_20261101/。
本条替代下方待接触点/旧Z与旧1m上限状态；未提交推送，保留1027及原工作树。

2026-11-01 用户确认BEV左右中点、MID360、原底盘中心的XY投影重合，前加70cm清扫
装置、总宽80cm，雷达离地90cm。已写rig_to_base=[0,0,0]，MID360 X/Y=0/0；按原
配置前后各52cm，外廓改前1.22/后0.52/半宽0.40m，保留10cm余量；costmap/TEB/
覆盖几何及私有持久coverage相应字段同步，其他调参/地图不变。MID360自车裁剪前沿
改1.22m，未实车验点云。15项几何及静态地图/无图launch展开通过，Qt/识别重开
PID1367985、采集1013642/BEV1013644不变，实读平面外参configured=true、IDLE、
等待导航TF；真实导航inactive，无目标速度机构命令。仍缺base_link离地高度和新装置
实际接触点（已提问）；雷达0.90m仅确认为离地量，sensor_z仍旧1.0待测，清扫参考
仍旧0.52待确认，新装置清扫几何尚未配齐。若参考点>1m还需改两控制器
旧1m参数上限。记录docs/VEHICLE_GEOMETRY_20261101.txt，证据validation/vehicle_geometry_20261101/。
本条替代下方“rig/base仍空”状态；本地导航配置下次live-start带入，未切正式current。

2026-11-01 用户要求改回项目中的YOLO模型。Qt/独立感知/直接server默认后端已改cuda，
实际加载YOLO11-GAM best6.pt（SHA256 5efa...1f2d0f），本机Jetson GPU推理；未使用旧
不同权重的垃圾HBM替代此模型。保留四原图识别、BV02五画面、后视车体遮罩与地面定位，
深度仍关闭，阈值0.25/输入640不变。10组在线报文模型SHA/四图配对通过，Qt实际CUDA
报文检查及界面截图通过；约4.87fps、四路调用中位107.77ms，左视塑料瓶6/10组检出，
存在间歇漏检，非精度验收。Qt/识别PID1202867，四格1013642/BEV1013644未重启；
真实导航inactive，未发目标速度机构命令，标定/遮罩/清扫配置SHA未变。证据
validation/vision_yolo_switch_20261101/。本条替代下方默认FCOS状态，保留1027及旧改动。

2026-11-01 用户要求Qt视觉页同时显示四路识别及融合结果，并排除后视白色蘑菇头。
已新增五画面布局及同组BV02接口，默认进入视觉页，旧BEV页/接口保留；后视960x540
轮廓遮罩保存在config/bev_detection_masks.json，模型前置黑、模型后按中心/接触点/多数
覆盖过滤，Qt红色标示。真实J6M同图A/B中原toilet/sink车体误判最终移除，置黑后产生的
person框也由后过滤移除；前左右结果一致、左视瓶子保留，遮罩外bed等误分类仍存在。
25组在线BV02均有左瓶二维点，约4.17fps、单路模型调用中位4.262ms、四路131.455ms。
3项Python/11项Qt及GUI构建通过，真实五画面截图已检查。只重开Qt/识别PID1155648，
四格1013642/BEV1013644不变且active；深度关闭、真实导航inactive、清扫未执行，
rig/base仍待确认，未称米制精度或地图定位验收。说明docs/VISION_FOUR_VIEWS_MASK_20261101.txt，
证据validation/vision_views_mask_20261101/。保留1027及既有改动，未提交推送。

2026-11-01 12:23 用户要求启动融合BEV，准备在地面放垃圾测试识别定位。已启动
四格/BEV/Qt，PID1013642/1013644/1013649 active，五组四路源序号递增，FCOS BPU约
4.6fps，Qt接受新会话；ZED仍NONE。启动采样尚无瓶杯候选，不能当作摆放后的测试结果。
真实导航inactive，未发目标速度机构命令；当前保持运行，等待用户摆放实物。
证据validation/bev_trash_test_20261101_1223/，本条替代下方关闭状态。

2026-11-01 11:22 用户再次要求启动BEV。四格采集/BEV/Qt已启动，PID
971223/971225/971229 active；三组实读四路源序号递增，FCOS BPU出图且Qt已接受新会话。
ZED深度NONE，安装角度仍为实测原值，未应用上轮试调建议；真实导航inactive。
证据validation/bev_restart_20261101_1122/。随后用户再次要求关闭，已正常停止Qt/FCOS、
BEV和四格采集；相关服务inactive/MainPID0，本机/板端无对应进程，ZED/ISX/BPU锁均
释放，18882～18885及11587无监听。关闭证据为该目录shutdown/；当前保持关闭。

2026-11-01 角度分析后用户再次要求关闭程序和相机。Qt/FCOS、BEV和四格采集均已
正常停止，相关服务inactive/MainPID0；本机/板端无本次采集推理残留进程，ZED/ISX/BPU
锁释放，18882～18885及11587无监听。证据validation/bev_angle_review_20261101/shutdown/。
当前程序和相机采集保持关闭，替代下方分析时运行状态；未调整物理角度或标定参数。

2026-11-01 用户要求重新启动BEV并分析相机安装角度。已启动四格/BEV/Qt，PID
872814/872816/872825 active，FCOS BPU约4.6～4.7fps；20组真实报文/四路图已保存，
ZED仍NONE、/rgbd404，真实导航inactive，未发目标速度机构命令。观察到桶/箱/墙平面
投影拉伸，按实际K/高度计算首轮机械试调建议：前20度、左右35度、后35度（均水平
向下），未写入配置或假定机械已调整。仅俯角不能消除同一立体点的IPM误投；前视
提高到0.6～0.8m/俯30～40度为可选对照方案，需重测外参及车体遮挡。理想覆盖图不含
遮挡，非实测新视角/清扫覆盖。说明docs/BEV_CAMERA_ANGLE_REVIEW_20261101.txt，证据
validation/bev_angle_review_20261101/。本条替代下方相机关闭状态，当前保持运行。

2026-11-01 用户要求关闭程序和相机。已依次正常停止Qt/FCOS、独立感知入口、BEV、
四格采集；相关服务全部inactive/MainPID0，旧sender/link也保持inactive。
ZED与J6M相机/BPU锁均可取得后释放，18882/18883/18884/18885/11587无监听，
板端无本次FCOS worker及私有相机server，采集日志确认PAIRED_CAMERA_STOPPED。
这是软件停止采集并释放资源，不代表切断相机电源；未改模型/配置或启动导航。
证据validation/bev_shutdown_20261101/。本条替代下方相机/Qt保持运行状态。

2026-11-01 用户要求Model Lab FCOS结合BEV并使用BPU。已复用J6M FCOS-EfficientNet-B3
HBM（SHA1daa...d7fa）及原常驻适配器，新增fcos_bpu后端并设为Qt/感知入口默认，
四路RGB原图识别后地面投影/同类跨相机去重；BPU承担FCOS网络，几何BEV仍CPU，
不是学习式四路特征融合。ZED继续NONE/enable_depth=false，/rgbd404。COCO80保留原类，
scene_ground_objects显示全体，清扫接口ground_objects仅瓶子/杯子候选，其他物体不作垃圾。
Qt显示实际模型及候选说明。11项Python及8项真实报文Qt检查通过，30组/120次板端调用
中位4.25ms、四路含前后处理/传输130.20ms、约4.74fps；后续正式显示约4.86fps。
前视检出雨伞，后视圆盘/右视杂物有误判，未检出瓶杯；不称垃圾专用精度/米制误差通过。
只重开Qt/识别PID794172，采集768840及BEV768842不变；真实导航inactive，清扫IDLE/
active=false，rig_to_base_xyyaw仍空。未发目标速度机构命令，未恢复深度或改源工作树。
说明docs/BEV_FCOS_BPU_20261101.txt，证据validation/bev_fcos_bpu_20261101/。
模型使用已安装旧DNN3.14.5/私有HBRT4.9.2；HEAL精确前后处理等价性未独立验证。
本条替代下方默认CUDA状态；1027分支/其他未提交改动保留，未提交推送。

2026-11-01 用户明确ZED与BEV深度融合很奇怪，“不要这样做，不要深度”。已彻底
撤回在线深度路径：SDK DEPTH_MODE::NONE及runtime.enable_depth=false，实际查询
getInitParameters确认关闭；删除RGBD采集/接口/深度融合分支和启动时深度库构建，
四路恢复RGB地面投影，single_source及垃圾点去重保留。历史深度试验文件/证据保留，
当前入口不加载，不因下方旧记录自动恢复深度。Qt/GTK已同步且真实重开，四格768840/
BEV768842/Qt768846 active，约4.98fps、投影28~30ms；四路序号递增、/rgbd404，
8项真实报文Qt检查通过。说明docs/BEV_NO_DEPTH_20261101.txt，证据
validation/bev_no_depth_20261101/。模型/阈值/标定/导航未改，未发目标速度机构命令。
本条替代下方深度启用状态；未提交/推送，保留1027与其他改动。

2026-11-01 用户质疑周围杂物很多却0目标。实读在线best6四路原图框全部为空，
rejected_localizations为空，故不是定位/去重过滤掉目标。保存当帧原图离线探测：
640输入降到0.05仍四路0框；1024输入仅前视地面亮斑paper=0.1495、后视白圆盘
metal=0.1263两个低分候选，均低于在线0.25，不能当可靠垃圾检测。当前五类材料
模型没有单独的垃圾桶/雨伞/托盘类别，对眼前杂物场景的检测能力未达成用户预期。
证据validation/bev_detection_review_20261101/，含真实原图、在线报文和低阈值框图；
本轮只诊断，在线模型/640输入/0.25阈值及相机/导航状态均未改变。

2026-11-01 用户要求解决前视觉相机垃圾桶拉伸。已启用ZED ULTRA双目深度，同一grab
的左目RGB/毫米轴向深度通过18885/rgbd配对，原/frame兼容；front_depth_enabled=true。
前视所属网格改实测3D点X/Y正交投影+最高表面z-buffer，CPU原生加速，不再地面拉伸桶身；
不可见/无效深度处黑色，仅1像素点绘制扩展。侧后算法/外参/所属区域未变；前视元数据
使用SDK实际校正内参。不是补全桶形/地面，也没修改垃圾接触点算法或宣称地图定位通过。
5项深度、24项原地面及8项Qt检查通过，同图原生/参考像素完全一致。实桶可见表面
X中位1.926m为深度估计，非独立真值或完整桶占地尺寸。原生回放54.15ms；线上投影
约54ms、四路识别约5fps。证据validation/bev_front_depth_20261101/，说明
docs/BEV_FRONT_DEPTH_20261101.txt。启用深度时重开过四格采集，最终四格716025/
BEV738788/Qt738792 active；旧sender保持inactive，未动导航/控制/时钟，未提交推送。

2026-11-01 用户进一步要求消除重叠、以二维垃圾定位为主。四路BEV改single_source，
每格按几何采样密度选一路图像，消除加权双影；映射/支持范围/盲区不变，三路默认
weighted保留，四路完整原图仍识别。地面去重增加15cm内双向原图框核对(3px容差)，
垃圾材质分类不一致不再单独制造重复点；保留每组最多一个同相机观测及完整组内核对。
Qt显示唯一二维目标与合并计数。24项几何/协议、8项Qt报文检查通过；同图61815个
重叠格单路供图，投影27.69->27.96ms，非地面拉伸仍存在。已经在线重开BEV/Qt，
四格源PID669916不变，BEV696187/Qt694295，约4.75~5fps；采样仍0目标，实物交界
去重/二维精度未验收。未改清扫管理器旧20cm关联、导航/控制或rig/base外参。
说明docs/BEV_OVERLAP_FIX_20261101.txt，证据validation/bev_blending_20261101/。
相机保持运行，未发目标/速度/机构命令，未提交/推送。

2026-11-01 用户要求启动查看效果，已通过camera_surround.sh start打开四格采集、
BEV与Qt，默认CUDA best6.pt。服务PID669916/669918/669922均active，四路源序号
连续递增，识别约5.0fps、四路调用106~111ms；Qt真实接收并显示新原图识别/地面
投影结果，采样时0目标。证据validation/bev_ground_live_20261101/，含3次序号、
源元数据与Qt截图。相机现保持运行，替代下方“相机关闭”状态；未启动真实导航、
发送目标/速度或机构命令，定位误差与新rig/base对齐仍待实测。

2026-11-01 用户表示不再纠结复杂环境，目标是BEV效果、垃圾识别及二维定位，可用BPU。
当前入口已改为四路原图识别、相机光线与地面相交、跨相机观测合并，再在BEV绘制米制
点位；默认沿用Jetson best6.pt，可选BEV_DETECTOR_BACKEND=bpu调用已有垃圾HBM。
J6M真实原图推理/HTTP回放贯通，单路vendor约6ms，四路完整调用中位145.83ms；CUDA
113.69ms。权重不同且测试不是受控A/B，不能将BPU核时延当完整链路加速比例。
20项几何/协议及8项Qt检查通过，Qt已构建但未启动；相机继续inactive，未动导航。
当前是观测融合而非学习式四路特征融合，不需要先完成x86/HEAL；原研发资产保留。
20张旧标注诊断BPU匹配19/38、CUDA20/38，训练重叠未知，不当作实车识别精度；
四路实拍回放没有垃圾检测，不能称真实正样本定位验收。输出为相机组地面接触点估计，
重装rig/base对齐仍空、曝光同步未验证，米制误差/地图稳定性/采集到map时延未测。
说明docs/BEV_GROUND_DETECTION_20261101.txt，证据validation/bev_ground_detection_20261101/。
保留1027与旧改动，未提交/推送，未发送目标/速度/机构命令。

2026-11-01 用户要求协助解决x86环境，确认有Intel/AMD Windows10/11电脑。
已准备WSL2 Ubuntu22.04说明及本地迁移包，包含官方4.11.11 compiler/march轮子、
固定六路BEV deploy BC、官方run_docker.sh及独立CPU环境/编译脚本；尚未连到用户
Windows电脑，未声称原生x86安装或编译通过。优先该电脑，无需先租云端或配GPU。
本机另只读复用旧工程QEMU/Box64与x86 Python，私有安装新4.11.11轮子；QEMU
真实import及官方BC的14输入/6输出读取通过，未做完整编译/训练；Box64新版本符号
hbmarchGetMarchConfigByName解析失败，证据保留。仿真不计原生x86验收，旧源树未写。
脚本tools/bev_feature/x86_{probe.py,setup.sh,compile.sh}与pack_x86_compiler.py；
说明docs/X86_WINDOWS_SETUP_20261101.txt，证据validation/x86_solution_20261101/。
相机/导航未启停，未上传SDK或创建云资源。保留1027及旧改动，未推送。

2026-11-01 用户提供 OE3.9.1 py310 开发包并确认先用官方 Hugging Face 仓库。
包已解压至 runtime/oe391_20261101/，wheel 实检 compiler/march4.11.11、plugin3.3.10，
均 x86_64；官方固定 revision 浮点/HBM/三个BC及许可文件已下载并校验。未取得HEAL0.0.2，
GitHub仍Repository not found；当前主机Jetson，无可访问x86环境，未启动OE容器或正式编译。
新SDK七个ARM库私有上传/tmp/conbine-oe391-YSqnpbu5，DNN/UCP3.15.8/HBRT4.11.11实读，
复用旧hrt_model_exec跑100帧，BPU16.36031ms/模型调用16.41243ms，一个BPU子图。
旧常驻程序因固定版本拒绝，新代码支持编译时SDK版本并以新头文件重编，J6M实际14输入/
6输出通过，10次vendor均值16.7451ms，有限/重复差0。原构建默认3.14.5保持，未切换在线库。
新增SDK/容器检查、官方BC编译准备和板端复测工具；3项逻辑检查通过，x86编译尚待执行。
四路相机/BEV/sender/link/receiver保持inactive，两端时钟保留，未改变导航/车辆控制。
仍缺HEAL/x86运行环境、同步米制标注与重装rig/base对齐；不能称四路垃圾BPU已交付。
记录docs/OE391_SETUP_20261101.txt，证据validation/oe391_20261101/。保留1027及既有改动，未推送。

2026-11-01 用户询问软件关闭相机并要求协助，已正常停止四路采集、BEV预览及旧
ZED sender、camera-link和板端receiver，均inactive/MainPID0；ZED与ISX相机锁
均释放，18882/18883/18884/18885无监听，板端记录PAIRED_CAMERA_STOPPED且PID文件
已移除。保留两端时钟服务；未改导航/底盘、配置或物理供电。证据
validation/camera_stop_20261101/。此条取代下条四格/BEV仍active的运行状态；
当前组合尚未做12小时连续运行验收。

2026-11-01 用户要求先按现有资源优化。已优化 ground_bev 固定权重/可见区/定点
remap与有效ROI，BEV每帧缓存一次JPEG，PNG导出转为最多一个待写快照的后台线程。
同图对照投影中位98.94->30.20ms，单路投影/可见区一致、融合最大差1灰度；在线
处理中位96.17->33.01ms，约4.9fps，仍为5Hz配置。14项几何/协议及2项写盘检查通过。
只重启BEV显示(PID507070)，四格源PID303816未变，外参/地图/运动设置未改。
新增可复现投影/在线性能和旧垃圾HBM诊断工具；J6M旧单类HBM在20张当前BEV上推理
完成，vendor_infer中位6.296ms、P95 6.450ms，20张均无>=0.25框，无精度真值。
旧HBM源checkpoint指纹与当前best6不同，未切换在线模型，不是四路BPU特征融合。
首次试验因camera-link自动重启receiver取消stop，已保留失败证据；修正后仅测试期间
暂停旧link/receiver并在finally恢复active，时钟和四路采集未停。正式current未改，
未启动Qt/YOLO/导航或发目标/速度/机构命令。HEAL/匹配x86环境、同步米制标注与
新rig/base对齐仍缺。说明docs/BEV_OPTIMIZATION_20261101.txt，证据
validation/bev_optimization_20261101/。保留1027分支与此前未提交改动，未推送。

2026-11-01 用户指定检查/home/slam/horizen-ch。已核查底层SDK：有BPU/UCP示例、
ISX031驱动/同步配置、全局时间源/Timesync、GDC拼接及六路BEV-CFT nash_m HBM；
其中complex_scene_test/hrt_model_exec/j6p/lib的7个库与当前私有runtime_4_9_2的SHA
全部一致，DNN头文件也一致。不同示例另有HBRT4.4.2，不能混用；未找到HEAL源码或
OE3.9.1训练编译包。板端只读globaltime=2(RTC)、phcindex=0；image_info_t明确tv是
HAL取帧时间，不能替代曝光。RX1/max96712/ISX031参考与实车触发脚/模组配置不同，
未直接覆盖或宣称曝光同步成功；ZED跨主机时间映射仍未验证。未改SDK、运行库、
相机服务或时钟配置。清单docs/HORIZEN_CH_BEV_RESOURCES_20261101.txt，证据
validation/horizen_ch_resources_20261101/。

2026-11-01 用户要求四路特征BEV垃圾中心定位、J6M BPU加速及独立RViz地图标点，
后续确认正式训练/量化/编译在x86_64，OE3.9.1、HEAL0.0.2、plugin3.3.10、compiler4.11.11。
本机实为Jetson AGX Orin；板端hb_version=J6M_Matrix_V2.0。GitHub SSH身份Doribelove
已认证，但HorizonRobotics/Heal的SSH ls-remote返回Repository not found，尚未取得源码；
本机及Docker未找到匹配OE开发包。官方固定revision六路HBM大小/SHA通过，私有4.9.2
runtime实际加载及100帧profiling通过：BPU16.5421ms、模型调用16.6277ms，一个BPU子图，
不能称为本车四路垃圾性能。常驻tensor executor新增S16 SCALE采样输入/量化元数据，
板端14输入/6输出与重复输出验证通过。测试暂停旧receiver并恢复active；四格/像素BEV
持续active，正式current未变。新四路浮点原型在Jetson通过真实图像前后向及ONNX检查，
未训练、未载官方权重、未完成HEAL距离topk/BEV encoder移植或垃圾HBM；没有替换旧预览。
新增独立/fod/bev观测/采集时刻TF/map跟踪/RViz标记，实际私有11591合成20帧回放通过，
不恢复旧Qt地图垃圾页面。6项几何/评价与14项tensor桥检查通过。实拍四图时钟仍未统一，
rig/base新平面对齐、共同实测地面点、同步四视图米制垃圾标注均缺；图中参考点为计算点，
没有实车交界去重/地图稳定性/定位误差或端到端延迟验收。正式训练限定x86；未发任何
车辆目标/速度/机构命令，测试ROS会话收尾停止。说明docs/BEV_FEATURE_J6M_20261101.txt，
证据validation/bev_feature_20261101/。本轮修改未提交/推送，保留既有1027分支与旧改动。

2026-11-01 用户提供重装外参并澄清角度指镜头主光轴：左右/后高0.80m，左右间距
0.43m且中线对称，后在中点后0.24m；前在中点前0.96m、高0.32m；主光轴下俯
左右22度、后15度、前0度。相机组地面原点仍在左右中点正下方，X前/Y左；左右
y=+/-0.215m，yaw按朝车左/车右取+/-90度，后yaw180，前yaw0，左右翻正180度。
三/四路配置均更新，移除旧前视IMU姿态及旧安装遮挡掩膜。旧左右Flash内参与当前
端口互换，已重新读Flash生成私有硬件GDC包runtime/surround_recalibrated_20261101/，
板端同名/map/conbine_camera/目录，入口默认更新。四格源服务和BEV服务均active，
四路源递增、新参数SHA及真实融合核对通过，约5.1fps，13项几何/协议检查通过。
旧camera-connections数字口服务已正常停止，由标准four-camera-view替代；ZED sender
仍由四格窗口持锁期间暂时停止，退出恢复。未启动YOLO/Qt清扫、导航或机构控制。
相机全部重装后，新中点与MID360的垂线关系待用户回复；旧rig_to_base=[0.20,0,0]
已备份并清空，不将本次相机组几何当作车体平面对齐或实测米制精度验收。
证据validation/camera_geometry_20261101/，说明docs/BEV_GEOMETRY_20261101.txt。
本条替代以下旧安装参数、旧服务及旧外参有效性状态。

2026-10-31 用户明确RX1端口0=左、1=右、2=后，0/1原图倒置。已保存独立
config/camera_connections.json并由当前四格预览读取，窗口按前/左/右/后标注，
左/右像素旋转180度且状态记录camera_role、preview_rotation_deg；后视保留原方向。
私有预览已重开、4路连续帧和实际翻正画面核对通过，仍用上述233137运行目录；
证据validation/camera_orientation_20261031/。本条替代下条“未确认映射、未旋转”
状态；仅端口与预览方向已确认，新安装的米制外参仍待测量，旧BEV外参未启用。

2026-10-31 用户说明所有相机均已连接并要求打开，重新探测RX1的0、1、2号口均
启动成功、读到各自Flash内参，加上ZED共4路实际持续出图；3号口返回-3505。
四格仅显示实际4路，按数字端口标注，未确认新左右/后视映射，未旋转鱼眼或套旧
GDC/BEV外参。原预览服务保持active，当前runtime/camera_recheck_20261031_233137/，
证据validation/camera_connections_20261031_233137/；未启动导航、BEV、YOLO或清扫。
本条替代下条的3路接线状态。

2026-10-31 用户换接后要求复查，最新RX1的0、1号口均启动成功且实际持续出图，
加上前视ZED共3路；2、3号口仍返回-3505。数字端口预览服务保持运行，已切换到
runtime/camera_recheck_20261031_225641/；原始鱼眼未旋转，左右映射与新安装标定
仍未确认。新证据validation/camera_connections_20261031_225641/。本条替代下条
“只有RX1/0出图”的上次接线状态，不据此断言原接口已损坏。

2026-10-31 用户明确所有相机位置及接口均已变化，当前连接前视ZED及左右两台相机，
本轮只做接线检查。旧相机端口对应、安装位姿及rig_to_base_xyyaw均作为历史记录保留，
不能当作新安装的有效标定。已打开数字端口原始预览，ZED(23748636)和J6M RX1/0
两路持续更新；RX1/1、2、3启动返回-3505，RX0、RX4初始化返回-3601，另一台尚未出图，
不能由此认定物理接口损坏。鱼眼保持未旋转、未套用旧GDC/BEV标定的原始方向。
当前autolabor-conbine-camera-connections.service运行；占用ZED期间暂时停止旧sender，
关闭窗口或正常stop该服务会释放相机并恢复原先active的sender。没有启动导航、BEV、
YOLO或清扫；截图及连续序号证据validation/camera_connections_20261031_225020/，
私有运行文件runtime/camera_recheck_20261031_225020/。详见docs/CAMERA_CONNECTIONS_20261031.txt。

2026-10-29 用户明确左右相机连线中点与雷达中心的连线垂直于底盘平面，故两者
底盘平面X/Y重合。已依据现导航MID360-in-base=(0.20,0,1.0)m的平面分量，将
BEV rig_to_base_xyyaw保存为[0.20,0,0]（末项yaw，不是Z）；沿用车前X/车左Y方向，
不使用或推算底盘离地高度。launch展开及前/后点转换核对通过，重开私有Qt/YOLO
显示会话后实读参数一致、rig_to_base_configured=true，状态“平面外参已配置，等待
导航定位TF”。四格/融合PID未变，真实导航inactive、清扫IDLE/active=false，未发
目标/速度/机构指令。证据validation/bev_rig_xy_20261029_215635/。本条替代下方
“平面外参仍空/待用户确认”的当前状态；米制精度与拾取效果仍非实车验收。

2026-10-29 用户要求删除Qt“BPU预览”页，以BEV视野清扫替代并适配已接入的后视。
已删除独立BPU页及其订阅/原始帧缓存，唯一BEV页放在原入口位置；综合页复用同一BEV
识别图并链接该页。清扫入口默认选四路配置，原camera_bev.sh三路默认仍保留；检测JSON
及清扫状态按实际来源显示4路环视，新增导航可用性展示，没有增加运行门或修改原控制。
Release及91项相关检查通过，只重开Qt/YOLO私有11587显示会话，四格源与BEV融合PID未变。
实读4路front/left/right/rear、清扫IDLE/active=false、rig_to_base_xyyaw仍空、导航不可用，
真实导航服务仍inactive；没有启动实车导航或发目标/速度/清扫机构指令。
用户追问高度：左右0.71m、后视0.80m已用于地面BEV；没有查到base_link到地面的独立
实测高度。MID360相对base_link的z=1.0是坐标偏移，旧zed_mount.yaml明确非地面高度；
M2 odom的z=0是二维约定，不能当测量值。仍缺BEV到底盘的平面原点对齐，机构自动
控制未接入，不宣称完整自主拾取清扫。说明docs/BEV_SURROUND_UI_20261029.txt，证据
validation/bev_surround_ui_20261029_213616/；本轮未提交/推送，保留此前四路改动。

2026-10-29 用户继续确认后视水平朝后、下俯0度、不倒装，并要求实际融合。
four_camera_bev.json后视位姿已完整保存：[-0.53,0,0.80]米、yaw180/pitch0/rotate_180=false。
真实四路BEV及原Qt/YOLO已启动并验证，约4.9fps，四路源序号递增，后视对应RX1/2。
新增本机127.0.0.1:18885/frame复用四格窗口持有的ZED SDK图像，保留SDK源时间；
四路BEV用--reuse-capture读已有18882/18885，不重复打开相机或停止该源服务。
camera_surround.sh start会自动启动四格源服务、融合及原Qt，三路原参数和默认入口保留。
Qt启动页按bevPreviewPage对象名定位，顶部和状态显示实际4路；GTK显示后视高度0.80/俯角0。
当前四格、BEV GTK、原Qt三项service均保持运行，ZED旧sender仍按原交接约定暂时停止，
关闭四格窗口才释放相机并恢复。导航/底盘/目标/速度/清扫未启动，Qt私有master11587。
同步和独立米制精度仍未验收；水平后视在当前120度透视图中，镜头后约0.82m以内地面
不在画幅中，BEV保留黑色盲区，墙/箱等立体物体会按平面假设拉伸。
证据validation/four_camera_fusion_20261029_192536/，含实际四路源/位姿/同帧YOLO验证和截图；
当前融合输出runtime/surround_rear_20261029_2/bev/。本条替代下方缺角度及未融合的历史状态。

2026-10-29 用户补充后视安装位置：左右相机中点后方53cm、离地80cm、正对后方。
已写入four_camera_bev.json的rear.position_ground_rig_m=[-0.53,0,0.80]及yaw_left_deg=180。
下俯角和是否倒装仍待回复，pitch_down_deg/rotate_180保持null；四路原视角预览继续运行。

2026-10-29 后续用户要求打开各个画面。新增scripts/four_camera_view.sh及GTK四格预览，
J6M复用上述_2私有GDC包的0/1/2，前视通过本机ZED SDK直连HD720/15fps、depth=NONE显示。
因原17446预览无帧，本次按用户打开相机的范围临时停止j6m-b-camera-sender.service，
取得其原camera.lock后显示；关闭窗口释放ZED并恢复此前active的sender，已实际stop/恢复/
再次start验证。clock/link/板端旧receiver未启停，导航和控制未启动。
当前autolabor-conbine-four-camera-view.service保持active，四格均连续出图；左右沿用原
180度翻正，后视提供仅显示旋转开关，没有改安装外参。截图与源序列证据在
validation/four_camera_view_20261029_191258/，当前状态/窗口截图在runtime/four_camera_view_20261029/。
本条是最新预览状态；前一条ZED503指原BPU预览接口，不表示本次直连前视无画面。

2026-10-29 用户要求在本合并目录加入第四路后视。保留原三路参数，新增
config/four_camera_bev.json、camera_surround.sh，以及按配置处理三/四路的采集、
地面投影、YOLO报文与Qt显示。J6M RX1 0/1/2三路已重新读Flash、生成各自GDC LUT，
实际960x540 GDC同源取帧及连续序号通过；新候选在
/map/conbine_camera/surround_rear_20261029_2/，首个同名前缀目录的VIO节点名失败证据保留。
新增2号口拟对应后视，安装位置/高度/yaw/俯角/倒装尚待用户确认，四路配置这些字段保留null；
不得把测试夹具中的后视位姿写入真实配置。原三路check通过，四路check明确报缺后视参数。
Qt构建、13项几何/协议及6项Qt报文测试通过，但没有真实四路BEV融合或Qt四路画面验收。
现有ZED进程仍运行，其17446预览当次返回503，接收端记录frame_age_out_of_bounds；
本轮未改其时间规则/配置或启停旧服务。临时GDC诊断已关闭、相机锁释放；没有启动导航、
清扫、录包或车辆运动，未提交/推送。续接见docs/FOUR_CAMERA_SURROUND_20261029.txt。

2026-10-27 用户要求先保存当前工作到1027分支，再删除旧垃圾地图标注/Qt页面，
改为三路BEV视野内垃圾逐步清扫。已先创建1027并提交保存点48b39af，随后在该分支
完成本地开发：删除旧待清扫垃圾页面、点位注册表、单点清扫节点/服务和旧专用地图
定位发布；新BEV页面接入米制相对位置、开始/停止、滚动可达排序、接近后再次观察、
前进清扫段和实际轨迹通过判据。使用现有导航桥/普通导航，不发布新速度或改变原调参。
用户明确回复“外参先预留，后续再提供，现在先用3路BEV”；真实rig_to_base_xyyaw=[]，
缺外参仍显示BEV识别，不能用零外参伪造map目标。清扫机构原本未接入自动控制，
本次仍如实区分轨迹通过与实际拾取。仅本地构建/隔离验证，未部署、启停实车或发运动
目标/速度。说明docs/BEV_VISIBLE_CLEANING_20261027.txt，证据validation/bev_cleaning_20261027/。

2026-10-26 用户明确要求部署到J6M。五项连续性修复已部署私有目录
/map/conbine_navigation/session-20261026_221305-816370/；TEB库、覆盖脚本、转发器、
launch/配置/插件描述SHA逐项一致，本机M2新驱动SHA与已测构建一致（M2仍接本机）。
板端隔离覆盖172/转发5/TEB23/M2转换14，共214项通过；本地契约15项此前已通过。
原持久coverage/transit/hybrid调参SHA和正式current未变，方向500、前视1m、失败等待
3秒、单MID360、原8月29日图及motion_enabled=true保留。部署前已保存参数/日志及
旧M2运行二进制，再live-stop正常停止旧会话。当前服务inactive/MainPID0、板端无
11583/11681监听；未启动新版、恢复旧任务或发送初值/目标/速度，不能称实车效果验收。
证据validation/navigation_continuity_deploy_20261026/，说明
docs/NAVIGATION_CONTINUITY_FIX_20261026.txt；本条替代下文“仅本地未部署/旧进程仍运行”状态。

2026-10-26 用户在了解M2 ROS驱动的零速保持转角改动后，确认“就先按照这样来改动”，
按下条五项方案完成本地收尾，最终源码/二进制SHA与已通过的229项检查记录一致。
当前运行版本仍旧会话，未因这次确认自动部署、重启或开展实车动作。

2026-10-26 用户希望一次修改多个相关问题，已完成本地五项修复：TEB方向错误后下一
有效解立即恢复（去掉连续5次等待）；Hybrid参考切换删除固定两周期归零，但丢弃旧
参考指令、原有安全/时效判据保留；M2 ROS驱动普通零Twist保持转角且速度仍0，显式
转角0仍回正，未改CAN协议/固件；短换挡段动作位置容差取原配置与段长20%的较小值
（30cm段为6cm），同时用于提前到达判断与TEB目标；清扫前连接失败保留本线跨窗口
自动重试，不再先跳线。500权重、前视1m、3秒controller_patience、原急停/超时均保留。
ARM64构建及229项检查通过（覆盖172/转发5/契约15/TEB23/M2转换14），记录的13个
过早成功位置均不在新完成窗口内；不能称闭环顿挫已解决，也未实现完整移动中后台换路。
用户随后问“改了底盘驱动？”已明确说明：确实修改本地m2_driver.cpp/辅助头并重编译，
目的仅为普通零速不回正。当前旧M2进程430041仍为20:39:55启动版本，服务429225
仍active，J6M旧会话未换包；本批未部署/重启/发速度或目标。下次正常live-start会带入
本地新库/脚本/驱动。说明docs/NAVIGATION_CONTINUITY_FIX_20261026.txt，证据
validation/navigation_continuity_20261026/。下文“尚未修短段/接入分支”是旧源码状态。

2026-10-26 20:50 用户要求结束录包并分析。录制节点已正常shutdown、11包索引可读、
无.bag.active，服务已自动回收；脚本末尾systemctl stop的not loaded不是录包失败。
633.140秒、874133消息、110话题、5.267GiB。导航保持运行，未改控制或发目标/速度。
本轮任务20:44:49.369～20:49:29.486，约280.12秒，4线/8段COMPLETED，无人工暂停，
最后有效活动面积57.84/60.74m²=95.23%；终态清理面积归零，不可写成覆盖0或100%。
旧2秒低进展取消0次；Hybrid149.25秒中最终零指令50.50秒（含正常等待），缓存后缀
接回失败16条、方向不符19条（转场15/清扫4），实际转场权重500。本轮明确找到13段
约0.30m短倒车均SUCCEEDED，但实测净位移4.3～10.2cm；换挡点位置容差0.25m，
缓存连接要求弦误差<=5.7度，实际失败9～44.2度，引起剩余转场重算。不可全归底盘故障。
实际轮角同侧打轮-回正-再打轮4次（上轮31次，但区域/速度不同，非受控A/B）。
CAN接收0x24超时27条、活动期15条；/cmd_vel录包接收间隔6次>200ms、最大250.7ms。
尚无TX逐帧时序，不能由录包间隔锁定发送/USB/VCU根因。原始监控386帧未见刹车/过流
位变化。报告、6组曲线、CSV与复算脚本在该数据目录analysis_coverage/，说明以
实验分析报告.txt、current_run_findings.json及修正后的metrics.json为准；原初算保留。
仅离线分析，未修换挡点/缓存连接或清扫前有限重试分支；替代下条“正在录制”的状态。

2026-10-26 用户希望直接跑一次覆盖分析，已通过live-start启动新版真实会话
session-20261026_203946-2，Qt打开，录制服务active，数据rosbags/localization/20261026_204006/。
保持当前代码/调参，实读controller_patience=3、转场方向权重500、前视1.0m、
motion_enabled=true，单MID360与原8月29日地图。15秒只读采样及首个已关闭60秒包读检
通过：51话题75882消息，含点云/IMU/TF/轮速/转角/控制器监控/最终指令/CAN接收；
录制全部及未来话题。地图文件和启动参数另存coverage_experiment/。
最新观察WAITING_INITIAL_POSE、覆盖IDLE/active=false、底盘与MID360避障ready=true，
需要操作者在Qt设置真实初始位姿并选区执行；未代替用户发初值/目标/速度或恢复旧任务。
启动日志发现VCU 0x24的200ms控制超时记录，已保留，不能将ROS发布频率当实际发送证明。
本轮未新增CAN TX逐帧记录，已有/canbus_msg是接收侧；通信根因仍需后续时间证据。
未修清扫前接入失败的有限重试/部分完成分支；本轮按当前版本记录，不声称已完整修复。
证据validation/coverage_restart_20261026/；本条取代下文“导航停止、尚未启动”运行状态。

2026-10-25 用户要求从当前位置向前接入剩余清扫线，并修改部署到J6M。已将入口与
恢复改为共用前进接入：可直接接上则继续，否则复用Hybrid/解析连接至原线前方，
不强制返回线头、不因旧25度/20度不一致直接人工暂停；滚动转场允许向前交接。
原线端点、路线顺序与事件触发保留，覆盖只计真实轨迹，不将跳过区域冒充已扫。
同时部署此前本地删除2秒/0.1米早取消与controller_patience=3秒的修改；后者仅指
连续控制失败等待，不能覆盖所有有效低速命令执行不动，也不是每3秒重规划。
已上传私有目录/map/conbine_navigation/session-20261025_182936-706299/，脚本/launch/
配置/TEB库SHA回读一致；本机状态机168项/契约15项、板端隔离状态机168项及原地图
记录姿态的原生前进连接验证通过。持久调参SHA与正式current均未变，保留方向500、
前视1.0m、单MID360、原8月29日图、motion_enabled=true；没有新增速度或运动门。
旧真实会话已由live-stop停止，当前服务inactive/MainPID0、板端无导航监听，新包未
启动、未恢复旧任务或发送初值/目标/速度。本条取代下文“仅本地未部署/旧会话仍运行”
状态，不能将静态验证说成实车顿挫已解决。说明docs/FORWARD_SWEEP_REJOIN_DEPLOY_20261025.txt，
证据validation/forward_sweep_rejoin_20261025/。

2026-10-25 普通点到点低速设置只读核查：launch/YAML虽写min_vel_x、
min_vel_x_backwards（均0）、min_vel_theta（-1.5），当前TEB配置结构/读取/速度边
不使用这些参数，不能仅改ROS参数就声称最低速度生效。已有oscillation_recovery=true，
10秒期望命令缓冲、归一化有符号均值阈值0.1/0.1且要求角速度符号多次改变；
恢复仅偏好一个转向方向，不直接触发全局重规划，也不能识别所有恒定小速度。
GPS近目标减速器当前MID360模式关闭。说明docs/ORDINARY_TEB_LOW_SPEED_REVIEW_20261025.txt，
证据validation/ordinary_teb_low_speed_20261025/；本次未新增或改动运行控制。

2026-10-25 用户提醒旧低速不可执行问题后找到明确历史：8月31日指令平均0.028m/s、
轮速0.002m/s且长期无进展；9月1日记录低速/大角速半径约0.197m、TEB仍报告有效。
物理低进展恢复曾用于这类有效命令却卡滞，2秒一刀切不适配与该兜底职责有用应分开。
下条本地“删除早取消+3秒controller_patience”不能覆盖持续SUCCESS小速度不动，
不得宣称完全替代所有物理卡滞检测。当前已有最小半径输出限制，旧根因不能照搬；
本轮还有-0.30m/s指令但实际移动少，不能全归小速度死区。未部署、未新增最低速度或门。

2026-10-25 后续用户选择TEB连续无有效控制等待约3秒。已完成本地修改：
删除coverage_manager中“实际进展2秒不足0.1m就cancel并重试”链路及废弃时间参数，
navigation_j6m.launch的controller_patience由10改3；不是增加零速门，也不是每3秒重规划。
保留原进度/完成/偏离跟踪、避障急停、固定挡位、整段总超时；剩余
hybrid_no_progress_distance_m只用于已完成rolling chunk的独立校验，不是执行中短时打断。
隔离状态机164项、契约15项通过；3个新子场景用旧方法验证均失败、用新方法通过。
真实启动launch离线展开3.0，证据validation/teb_control_patience_20261025/，说明
docs/TEB_CONTROL_PATIENCE_FIX_20261025.txt。仅本地修改，未部署或重启现有真实会话；
实际读取其controller_patience仍10.0，旧2秒链路仍在该旧进程中。未发目标/速度或恢复任务。
入口25度/恢复20度不一致未改；不得称本次已解决全部顿挫或实车3秒已生效。

2026-10-25 后续核对普通Navfn+TEB第一入口，约19.60秒成功接入清扫；
同样存在起步/换向响应不足：16:36:11.684～13.684两秒，最终速度中位数-0.30m/s，
指令积分0.572m，轮速累计0.0584m，ICP净位移0.0295m，FAST-LIO净位移0.0253m；
随后16:36:22.684～24.684两秒指令1.600m、轮速1.559m、ICP1.559m，速度已跟上。
源码Hybrid专用2秒低进展取消只对type=transit生效，普通entry无该取消，
本段没有此类取消日志。说明相似响应不足也在普通入口存在，Hybrid重试可放大问题；
不可据此断言底盘硬故障或调整2秒即解决。证据analysis_coverage/first_entry_motion_check.json、
06_first_entry_control.png/.svg，原导航与参数未改。

2026-10-25 后续用户追问是否速度正确/底盘未执行或进展漏算。已补充
`analysis_coverage/motion_crosscheck.json`：低进展判断源码使用map->base_link投影路径进展，
不能把离线轮速积分当作其直接判据。16:37:15事件前约2秒，最终速度积分0.555m，
轮速累计0.0826m，ICP净位移0.0524m，FAST-LIO净位移0.0637m，路径进展0.0403m。
说明此例确有少量运动而进展不足，未发现已走0.5m却被漏计；近阈值事件需精确重建TF时序。
非零/固定挡位允许不证明执行器时序可行或VCU收到，不能断言规划正确且底盘故障。

2026-10-25 16:54 用户要求结束录制并分析，随后追问末条清扫线停车原因。
录制已通过ROS节点正常关闭：30包、1780.533秒、2308154条、110话题、14.612GiB，
无.bag.active；录制服务inactive，真实导航仍active，未改控制或恢复任务。
分析在 `rosbags/localization/20261025_162452/analysis_coverage/`，含报告、5组PNG/SVG、
轨迹/控制CSV与10倍速二维GIF；原包和原地图保留。正式任务16:36:09开始，
16:41:19首次因清扫走廊恢复失败进入PAUSED，系统覆盖69.03/86.98m²=79.36%，未完成。
末线交接时横向约0.346m/朝向21.82°，入口允许25°，TEB随后反方向候选被归零，
约10秒后move_base ABORTED；恢复要求20.05°，实际仍21.65°，因此自动暂停。
首次暂停时定位/底盘/避障均ready，后续急停不能当作首次停车原因。
转场方向权重500已实包确认；15次Hybrid方向不符、另4次清扫模式1000方向不符；
Hybrid约171秒中零指令48秒（含正常换挡/等待），33次低进展重规划、11次缓存接回失败。
其中28次低进展前2秒已有>=80%时间非零最终指令，但轮速积分不足0.1m，
不能全部归因TEB归零；ROS最终输出不证明VCU收到执行，底层响应不足原因仍待定位。
仅离线分析脚本新增，未修交接/恢复或底盘逻辑，不把本次修复权重宣称顿挫已解决。

2026-10-25 16:33 用户要求录制覆盖清扫实验、供回放及报告动图使用。沿用
`rosbags/localization/20261025_162452/` 连续全话题录制，未重启或切断录制；
核验120话题已订阅，已关闭分包实读95话题580717条消息，点云/TF/地图/定位/
覆盖路径/代价地图/轮速/转角均有实包数据，TEB等任务话题已订阅待实际任务发布。
原8月29日PCD/二维地图、参数、Qt设置及显示配置已另存该目录 `coverage_experiment/`，
含实验时间标记、哈希、回放资料说明与实包读检。此时操作者已完成定位，
LOCALIZED、覆盖READY、底盘与MID360避障ready=true；取代下条冷启动等待初值/急停状态。
仍在持续录制，尚未制作本轮动图；未代替操作者启动清扫任务或发送运动指令。

2026-10-25 16:24 用户表示中断并要求重新开始实验、记录状态。检查旧服务已停止后，
通过真实live-start启动新会话 `session-20261025_162443-2`，Qt已打开；两端时钟一致。
全话题录制服务 `autolabor-conbine-localization-record.service` 已启动，数据位于
`rosbags/localization/20261025_162452/`，包含点云/IMU/TF/定位/规划/控制/底盘与未来新话题。
15秒只读检查确认MID360/FAST-LIO/轮速链路正常、运动使能true、MID360避障ready。
检查时WAITING_INITIAL_POSE，底盘报告手柄/遥控急停及机器人急停，chassis_ready=false；
需操作者设置真实初值并处理现场急停，未替用户发送初值、目标、速度或解除急停。
保持原8月29日地图、转场方向权重500及前视1.0m。证据
`validation/experiment_restart_20261025/`；本条取代下条旧会话及未录包状态。

2026-10-25 用户随后要求启动系统开始实验。真实入口live-start已启动会话
`session-20261025_150837-2`，本机服务active、Qt已打开，使用本工作区修复包与原8月29日图。
板端重启后时钟回退，已在启动ROS前同步本机，测得偏差约-0.7ms；未修改时钟准入规则。
15秒只读检查通过：FAST-LIO/scan/轮速通信正常、MID360唯一避障、底盘和避障ready=true、
原motion_enabled=true。检查结束时仍WAITING_INITIAL_POSE，需操作者在Qt设置真实初值；
未替用户发初值/目标/速度。转场方向权重配置500；move_base尚在等待map->base_link，
不能把此时普通导航权重100或未初始化插件误判为转场权重失效。
证据 `validation/experiment_start_20261025/`；本次没有额外启动全话题录包。
此条取代下条“未启动导航”的运行状态。

2026-10-25 后续用户要求部署到J6M并默认转场TEB方向权重500。已部署至私有目录
`/map/conbine_navigation/session-20261025_150231-783689/`；修复库、覆盖脚本、生成配置和
插件描述逐项SHA一致，板端隔离加载及23项原生测试通过。源码回退值、动态参数默认值、
板端coverage.yaml和Qt保存值均为500；已有其它调参（包括前视1.0m）逐字节保留。
本机会话记录已指向部署目录；未启动实车导航、未发送目标/速度、未切换正式current。
此条取代下条“未上传”的部署状态，不能将静态/隔离测试说成实车顿挫已解决。
证据 `validation/teb_deploy_20261025/`，记录 `docs/TEB_DEPLOY_20261025.txt`。

2026-10-25 用户明确要求先修复两处 TEB 问题。已在本工作区修复：Hybrid 前进/倒车
均使用 Qt 保存的转场方向权重，不再在前进段回退为普通导航权重；单个全局路径采样点
从原始参考路径获取车体朝向/切线，继续建立横向和朝向代价，倒车不重复加 pi。
覆盖状态机163项、原生TEB23项通过；原生快照252次求解仍有错误方向解，不能声称顿挫
全部解决。真实导航打包已包含新版 TEB 库及插件描述，后续 live-start 自动携带；
仅完成本机构建和隔离打包验证，未上传、启动或重启实车导航，正式板端 current 未切换。
前视、重规划、运动权限、停车和底盘逻辑未变。详见 docs/TEB_TRACKING_FIX_20261025.txt，
验证证据 validation/teb_tracking_fix_20261025/。旧诊断中的“这两处尚未修复”现为历史；
旧板端会话若未换包重启仍是旧实现，不可将源码修复冒充实车运行验收。

2026-10-24 20:14 用户要求定位精度测试并录制全部定位相关数据。当前独立录制服务
`autolabor-conbine-localization-record.service`，入口 `scripts/record_localization_test.sh {start|stop|status}`，
数据目录 `rosbags/localization/20261024_201446/`，当前目录链接 `runtime/localization_recording/current`。
录制专用真实 master 的全部话题，LZ4、60秒/2GiB分包；首包实读101话题、81071条消息，
含原始点云/IMU、三维点云、ICP定位/先验图/子图、TF、地图和底盘。原PCD/二维图与参数已另存。
用户已要求结束录包，录制服务已停止，12个包索引均可读取，无 .bag.active 残留；
末包索引已恢复、原件保留于 recovery/，未写入的末尾数据无法补回。
统计见数据目录 recording_summary.json，未停止导航或改变车辆控制。
本次定位已由操作者初始化并 LOCALIZED；下方19:02等待初值是冷启动时的历史状态。
用户后续要求只保留前5分钟并分析位姿；当前工作目录仅含 `localization_first_300s.bag`，
按记录时间精确保留[0,300)秒、420798条消息，原始消息/时间戳/发布者连接逐条校验一致。
完整11分钟原件移至 `/home/slam/robot_j6m_ws_conbine/rosbags/localization_backups/20261024_201446_full_20261024_205139`。
曲线、CSV和分析在工作数据目录的 `analysis/`；脚本 `scripts/analyze_localization_pose.py`，用系统Python `-s` 运行。
不将内部轨迹差或ICP残差冒充独立真值精度；本轮仅离线处理，未改变导航/控制。

- 用户已明确要求删除本次 AI 额外加入的运动限制，使无人车可以运动。真实导航入口
  `bash scripts/conbine.sh live-start` 使用正常运动链路，向原看门狗传入 `motion_enabled=true`；
  删除监视器“收到非零 /cmd_vel 就结束会话”和重复的禁止运动参数。
  后续正常启停保留此行为，不重新关闭、不索取重复授权、不要求额外授权文件。
- 原有急停、MID360 避障、定位、通信超时及速度配置保持兼容；不得借开发、调试、
  合并、改入口或“提高安全性”自行叠加运动门、心跳许可、强制依赖、自动归零等干预。
  用户明确要求调整已有机制时，按其具体要求实施，不用历史约束拒绝或拖延。
- 用户要求完整导航时交付可接收目标并正常控制底盘的链路，不擅自替换为静止观察、
  仿真或只能显示地图的模式。原 `scripts/conbine.sh start` 是明确保留的离线预览，
  不可将它的隔离和零速规则套用到 `live-start` / `mapping-start`。
- 当前只使用一台 MID360 建图、避障和定位，不等待前后 LD19；带图启动继续加载
  `map_20260829_221335_gimp_cleaned` 的原 PCD 与配套二维地图。
- 不将相机/BPU/Qt/日志/诊断或新模块是否运行变成普通导航的新准入条件。
  真实导航运动可用不表示三路视觉控制或标定已通过；不伪造视觉 readiness、深度或验收结果。
- 本次已重启生效：会话 `session-20261024_190236-2`，实际看门狗 `motion_enabled=true`、
  `active=true`，底盘/避障已就绪；冷启动正等待操作者实际初值。隔离非零转发/监视器存活及
  原超时归零验证通过；真实15秒只读检查通过。证据 `validation/motion_enable_20261024/`。
- 本轮授权包含必要的源码修改、私有会话更新和重启；不要求用户反复批准。
  使能导航不等于要求 AI 自行选择目标、发送非零速度、解除硬件急停或启动清扫机构。
- 源工作树保持不改，正式板端 current 不切换，不刷机、不推送；保留用户地图、调参和改动。
  验证只针对实际修改，结果如实报告，不把开发检查增加为车辆运行的许可条件。

以下是合并来源和历次会话的历史记录，用于溯源；其中时态、进程身份、授权边界及
旧的待验收事项不能覆盖以上当前规则或用户后续明确要求。

# 合并分支工作区说明（2026-10-23）

- 当前目录 `/home/slam/robot_j6m_ws_conbine`，分支 `robot_j6m_ws_conbine`。
- 独立 Git 仓库，数据库在本目录 `.git`，不是依赖父目录的 worktree。
- 用户明确要求将当前三路视觉与导航调参两条开发线主要成果合并保存到新分支，并打开 Qt。
  用户称视觉分支 vision_bpu_dev；本机实际名称 vision-bpu-surround，按当前三路版本处理。
- 来源一 `/home/slam/j6m-dev-b-surround` / vision-bpu-surround；来源二
  `/home/slam/robot_j6m_ws_navigation_tuning_20261023` / feature/qt-navigation-tuning-20261023。
  两源 HEAD 均为 5d1028e，但成果包含未提交和未跟踪源码。已在新仓库分别保存来源快照，
  清单与恢复 tar 位于 runtime/conbine_merge_20261023/，不得只以旧 HEAD 代表源码。
- 允许在新目录做合并、修复、构建、隔离验证和保存本地提交；不推送、不改源工作树内容。
- 集成内容：三路 GDC + 原 BPU 双模型 / 原 Qt 视频顶部端口切换；导航调参页、三组参数
  持久化、Qt 与后端最小面积外接矩形；同时保留共同继承的视觉/AI/导航修复。
- 原合并预览入口 `scripts/conbine.sh start` 的 Qt 导航后端必须在只有 lo 的网络命名空间，使用合成地图/TF/零速 odom。
  真实三路视频通过仅允许 GET /state 和 /frame/0/1/2 的独立本地桥读取已运行环视服务；
  不打通车辆 ROS、不发布真实目标/速度，不把模拟调参持久值当作已部署板端。
- 旧 optimized.sh、surround/operator_qt.sh 的来源目录保护保留，不能直接绕过运行。
  本目录新入口 scripts/conbine.sh；独立服务、日志、ROS/Qt 配置保存在本目录 runtime。
- 保留模型身份/帧龄/源会话与端口检查、BPU 独占锁，真实标定模板仍为空；
  production_ready=false、motion_eligible=false 不变。尚未授权整栈部署、刷机、车辆动作。
- Release、339 项独立覆盖/Qt 测试、17 项视频协议/转发测试和 7 项后端持久化检查通过。
  当前运行范围与验证证据见 docs/CONBINE_INTEGRATION_20261023.md。
- 2026-10-24 用户授权将 J6M MID360 接收地址永久保存：eth1（33100000.ethernet）
  为 192.168.1.100/24，无默认网关，已保存到厂商持久配置并当前生效；不能再按历史
  测试的临时地址撤回。仅验证原始点云/IMU UDP 接收，未启动 FAST-LIO/导航/底盘，
  未重启验收；详见 docs/MID360_PERSISTENT_ADDRESS_20261024.txt。Qt 隔离要求继续保留。
- 2026-10-24 后续用户明确要求启动整个导航。新增独立 `scripts/conbine.sh live-start`
  / `live-stop` / `live-status`：J6M 直接接收 MID360 并运行 FAST-LIO、原地图 ICP、
  move_base、覆盖/Hybrid/TEB 后端；本机运行当前 CAN 驱动、固定 motion_enabled=false
  的看门狗和合并版 Qt。使用专用 master 11583 与 /map/conbine_navigation/ 私有会话，
  新后端文件仅放私有会话，不切换正式 current，不套用离线调参持久值，不放宽运动/视觉门。
  当前等待真实初始位姿；底盘报告手柄/机器人急停，LD19 缺失，不能行驶。相机/BPU未启动。
  未发初值/目标、解除急停或授权运动；原预览入口隔离继续保留。见 docs/CONBINE_LIVE_NAVIGATION_20261024.txt。
- 2026-10-24 再后续：用户明确永久拆除前后 LD19，建图/避障/定位仅使用一台 MID360。
  新 `/mid360/scan -> mid360_scan_guard -> /scan` 和 `/avoidance/mid360_active` 为唯一避障链；
  覆盖任务不再订阅或要求 LD19，仍要求 MID360 扫描/源状态新鲜，保留全部底盘/定位/运动门。
  新建图使用 MID360 360° 扫描，旧双雷达模块仅保留作历史资产，不在当前入口启动。
  用户要加载之前建好的 PCD 及配套地图文件：默认固定为 map_20260829_221335_gimp_cleaned，
  配置及原文件 SHA256 在 config/conbine_live.json；不得改用 9月14日地图、mutable current/latest
  或新生成的地图代替。原件只读，旧地图来源元数据不改写。
  真实会话已按该配置重启，MID360 避障准入 true；仍等待真实初值、底盘急停且运动门 false。
  当前说明取代上条“LD19缺失导致准入不满足”的首次启动状态。见 docs/MID360_ONLY_ARCHITECTURE_20261024.txt。
- 2026-10-24 18:29 操作者已在 Qt 设置真实初值，ICP 已 LOCALIZED。随后定位完成暴露独立
  会话遗漏 costmap_plugins.xml/配套库，move_base 加载 UnknownSpaceGuardLayer 失败退出；
  已补齐资源，在同一受管 PID 命名空间仅恢复 move_base，保留操作者定位、雷达和 Qt。
  全局1311x670、局部200x200代价地图及10Hz车体轮廓、map->base_link已实收；Hybrid/TEB
  参数协议和同步恢复。后续启动自动打包所有顶层插件XML与库并预检，move_base设为关键节点。
  当前仍单MID360、原8月29日PCD/二维图、运动门false及底盘急停，不重发初值或恢复任务。

下列两源记录均为历史，目录、服务和已完成测试不自动代表本合并分支的新验收或授权。


## 视觉来源的历史记录

# 环视试验分支工作区说明（2026-10-20）

本工作区由用户明确要求，从现有 vision-bpu 创建，供 J6M 适配相机四路环视方案试验。

- 当前目录：`/home/slam/j6m-dev-b-surround`。
- 当前分支：`vision-bpu-surround`；父分支：`vision-bpu`。
- 父分支目录：`/home/slam/j6m-dev-b`，继续保留 ZED + J6M BPU 原方案。
- 原项目基线：`/home/slam/robot_j6m_ws_navigation_20260907`，分支 `0914`。
- 这是 Git worktree；共享 Git 数据库在 `/home/slam/j6m-dev-b/.git`。不要直接移动或删除父目录。
- 已继承父工作树的未提交源码、未跟踪源码、原方案文档及模型/转换环境；它们仍未提交，不能把 HEAD 当作完整成果。
- 分支创建交接：`docs/team/handoffs/B/20261020_surround_branch_setup.txt`。
- 方案输入：根目录 `方案.txt`、`bev/方案.txt` 及父目录新上传的相机资料。
- 2026-10-20 后续用户已授权整理相机镜像并搭建环视整体框架；本分支已新增
  `distribution/j6m_bpu_navigation/surround/` 和 `src/application/autolabor_surround_vision/`。
  使用说明以新模块 README 和 `docs/team/handoffs/B/20261020_surround_framework.txt` 为准。
- 已核验的镜像包、驱动资料与解压 SHA 清单在 `runtime/surround_vendor_20261020/`。
  此为用户上传包的独立副本，没有执行烧录脚本。
- 2026-10-20 后续用户要求先配好 BEV 软件环境，暂不处理相机硬件或标定。
  已新增独立 `bev.sh` 入口、私有 `runtime/surround_env/`、ARM64 双版本离线依赖包，
  四路图像可独立计算 BEV，不依赖检测/BPU/ROS。最新交接为
  `docs/team/handoffs/B/20261020_bev_environment.txt`；不要将合成演示当实机验收。
- `bev/待同步压缩包.txt` 记录源 ZIP 仍在写入的情况，未将不完整压缩包当作可用输入。
- 父 ZED 会话的 state 目录已在此副本移至 `runtime/inherited_session_state/`，仅作历史证据，不代表本分支有运行服务。
- 2026-10-21 用户要求鱼眼去畸变全部改为 GDC。实时检测/预览/BEV 相机入口现要求
  PYM NV12 layer0 + 已校验的四路透视 LUT；不允许实时 CPU 去畸变回退。
  CPU 参考仅保留显式离线调用，BEV 地面投影仍用 CPU。仅本地修改/隔离验证，
  未部署或启停设备；最新记录 `docs/team/handoffs/B/20261021_surround_gdc.txt`。
- 2026-10-23 用户已连接相机并明确要求打开四路画面；本次授权相机预览及其私有采集程序，
  不扩展到刷机、导航/控制或旧会话启停。实际发现 RX1/MAX96712 的端口 0、1、2 可出图，
  端口 3 仍链路未锁定；不是四路全部验收。板端仍为 20260311 镜像，系统相机库未替换。
  独立原始鱼眼诊断窗口已建立，私有运行文件在 `runtime/surround_raw_preview_20261023/`；
  仅采样缩小和颜色转换，没有 CPU 去畸变。实测 CIM 输出 NV12，私有诊断库补充此读取方式。
  原 GDC/BEV 配置与空标定模板未改；其 RX4 候选不能直接当作当前 RX1 接线配置使用。
  本次记录：`docs/team/handoffs/B/20261023_surround_camera_preview.txt`。
- 2026-10-23 后续用户要求用现有三路经 GDC 显示透视画面；已正常关闭上文原始预览，
  从端口 0/1/2 相机内部 Flash 读出各自镜头参数，按 KB4 鱼眼模型解释生成私有透视 LUT。
  当前三路 RX1 -> PYM1 NV12 layer0 -> GDC0 -> AGX 独立 Qt 窗口已实板出图，960x540、
  中心约 120°，预览约 5 fps；真实 GDC 标记和源序号连续性已核验，无实时 CPU 去畸变。
  参数来源真实，但 SDK 未明确报告畸变模型类型，光学精度尚未独立验证；accuracy_verified=false。
  本次是数值端口调试预览，未替换正式四路配置或填充安装外参，未生成 BEV。
  私有入口 `runtime/surround_gdc_preview_20261023/open_preview.sh`；与原始预览互斥。
  当前状态与复现记录：`docs/team/handoffs/B/20261023_surround_gdc_preview.txt`。
- 2026-10-23 后续用户要求三路接入原 BPU 识别，并在原 Qt 视频顶部加入端口切换。
  用户进一步明确：直接在该分支原有 Qt 内替换 ZED 画面，其余 Qt 部分不变。实际修改
  原 `autolabor_operator_gui/MainWindow`，保留原页面/分栏/RViz，只在视频上方加三个按钮，
  视频、检测列表及原相机/模型状态显示对应端口数据；没有另做替代 Qt 界面。
  三路公平轮询复用原一对 HBM 和独占 BPU 锁，已实板识别/显示，每路约 5 fps；
  Release、15 项新测试及 52 项原 Qt 契约通过，三个实际按钮切换已逐项核验。
  当前板端候选 `/map/j6m-dev-b/vision-bpu-surround/operator_20261023_2/`；_1 为缺少
  接口校验依赖的首次失败版本，日志保留，已补完整依赖预检。
  本轮按明确替换 ZED 识别视频输入的范围执行必要临时 BPU 资源交接：暂停旧接收和
  自动拉起服务，旧 ZED sender/时钟继续运行；关闭本次 Qt 后恢复旧识别，已验证恢复。
  此前待确认/未运行记录为历史；当前 `take_over_existing_bpu=true` 仅限上述交接，
  不扩展到导航、刷机或运动。正式四路/外参/曝光时钟校验和 readiness 标记仍保留待验。
  入口 `distribution/j6m_bpu_navigation/surround/operator_qt.sh`；当前交接为
  `docs/team/handoffs/B/20261023_surround_operator_qt.txt`。

源码开发仅在本目录；2026-10-23 增加上文相机预览、原 Qt 识别接入及必要输入切换操作，不修改父工作树、稳定工作区或其他成员源码。
旧文档中的路径、启动命令、部署状态和运动许可是历史记录，不构成本分支的新执行授权。
复制的工具、环境和配置可能含父目录绝对路径；首次构建/运行前须适配本分支路径、独立日志与服务身份，不能直接运行旧启停/部署脚本或删除路径保护。
相机适配、模型、时间同步、标定与消息接口应按已评审方案分阶段验证；除上文相机识别/Qt 联调及其必要输入交接外，未授权刷机、整栈部署、其他设备启停、录包或车辆动作。
相机已连接做桌面预览，安装方位与标定尚未确认；真实标定模板保留空值。合成标定仅在演示/测试目录，不能复制到硬件配置。
新框架保持 production_ready=false、motion_eligible=false；平面估计不是实测深度，
不得将其填为旧视觉伺服的 depth_valid=true。新消息仅发布 /surround 观察接口。
首次连接硬件前核验新镜像与 SDK/BPU ABI、端口方向、YUV 布局、曝光时钟和源时刻定位；
原生采集库在本机构建成功不等于四路相机或 GDC 实板验收。
保留所有现有修改和模型资产；未经明确要求不清理、重置、提交、推送或合并。
共同协作规则继续使用 `/home/slam/Desktop/地平线挑战杯/J6M_五人协作_Codex开发与合并约束_20261019_v3.txt`。
下面保留继承的历史 AGENTS.md 原文；与本分支目录/状态冲突时，以本说明和用户后续明确指令为准。

---

## 导航调参来源的历史记录

# 2026-10-23 后续：永久调参 / Qt预览最新

用户要求关闭当前Qt，打开本分支调参Qt，并要求每次修改后重启保留。
已由原 `j6m-dev-b-surround/distribution/j6m_bpu_navigation/surround/operator_qt.sh stop`
正常关闭环视Qt会话（原unit已inactive/MainPID0）；原BPU恢复由其原入口完成，未改该分支源码。
本分支新增 `scripts/navigation_tuning_preview.sh {start|stop|status|test}`，当前预览服务
`autolabor-navigation-tuning-preview.service`，仅lo的独立网络/PID命名空间，私有master11543；
真实move_base/TEB/coverage_manager与本分支Qt，合成地图/TF/零速odom，不连接实车/相机/CAN。
界面标题明确离线；导航调参页内有覆盖/转场TEB/Hybrid三个子页签，高级组自动读实际值。
基础参数编辑不再依赖地图渲染/区域库，但仍要求后端状态新鲜、无活动任务/预览；
规划/运动的地图、定位、授权门未放宽。
Hybrid/转场高级参数永久保存到 `$ROS_HOME/autolabor_navigation/{hybrid,transit}_tuning.json`，
启动时已存值覆盖launch/YAML默认；临时文件fsync+原子rename成功后才更新运行态和应答。
保存失败拒绝并回显原值，损坏/越界历史配置拒绝启动；Qt要求后端persistence_protocol=1。
基础组沿用coverage.yaml和Qt设置。当前用户预览配置为runtime/navigation_tuning_preview/，
首次初始化后保留；不是板端部署配置，不自动同步J6M。
验证：Release、333项独立覆盖/Qt用例（Catkin400含重复）通过，真实后端7项保存/失败/重启
过程检查通过。旧失败证据和最终日志均在validation/tuning_persistence_20261023/。
未部署/启停实车导航/发真实目标速度/改运动授权，未查询J6M current，未提交推送。
下面“高级参数仅会话生效”是上一阶段历史，已由本条替代。生产optimized.sh目录保护仍保留。

# 2026-10-23 本工作树最新

本目录现为 `/home/slam/robot_j6m_ws_navigation_tuning_20261023`，独立分支
`feature/qt-navigation-tuning-20261023`。原 `/home/slam/robot_j6m_ws_navigation_20260907`
仍为 `0914`，未改动；新 worktree 带入原未提交源码，未提交/推送。
新增“导航调参”页，基础覆盖参数迁入并保留事务持久化；转场 TEB 16项和 Hybrid 11项
经 dynamic_reconfigure 回读确认，无需重启。覆盖/转场只在无预览/任务时修改，Hybrid
在完整搜索之间切换；高级参数仅本次会话生效。Qt确认、后端单区/批次统一使用点集的
最小面积外接长方形，拒绝退化和面积<0.5m²，原安全/授权门保留。
Release、覆盖222项、Qt105项及真实插件隔离14项全部通过，0失败；327为独立叶子用例，
Catkin390含嵌套重复。测试仅lo/合成数据，不是实车验收。初始环境/夹具/页签断言失败
保留；详见 `docs/QT_NAVIGATION_TUNING_20261023.md` 与 `validation/navigation_tuning_20261023/`。
本目录可用 `scripts/validate_navigation_tuning.sh {build|test}` 隔离验证；原devel仅作只读
依赖，未复制构建缓存、私有配置、地图/模型、授权标记。生产启停/部署路径尚未迁移，
`optimized.sh`旧目录保护保留，禁止绕过保护启动或连接旧会话。
未部署、启停/接管实车、发初值/目标/速度或改授权；未重新查询J6M current，以下版本和
运行状态全部为继承历史，不能作为当前实测状态。

# 导航优化专用候选版约束（2026-09-07）

2026-09-14 11:47 本轮覆盖实验监测最新：用户要求开始实验并由助手持续监测。只读确认
导航服务为11:28新会话、active/MainPID1246402；操作员已为该会话设置真实初值，开始前
LOCALIZED，覆盖/视觉/AI空闲且全部速度与双轮反馈为零。11:35挂载被动记录器后，操作员
先生成并取消5份覆盖预览，均未执行；最终计划c53e5a10bd8b4bba8acab951ecfadd1e于
11:43:06进入PREPARING，11:47:42完成10/10段，实测275.604秒。捕获7次短暂
WAITING_OBSTACLE，分布于2/3/5/9段；第5段覆盖到一次整轮重试耗尽后保留依赖清扫线并
继续自动重规划，最终无PAUSED。26次Hybrid action结果均以SUCCEEDED返回，其中23个结果
直接取得入口，3个报告目标footprint处于占据/未知区后由后续重试恢复；另有9次2秒低进展
取消。11:46:57记录1条coverage_manager未跟踪GoalHandle的晚到回调ERROR，但未中断或
改变最终完成状态；任务开始时optional_cloud_enhancer有1条LD19 stale瞬态告警，记录窗口
内/scan与三雷达采样无超过1秒断流。定位138个变化样本均LOCALIZED，最低overlap0.6497、
最高RMSE0.1936。控制线性/角速度峰值1.60m/s、0.444rad/s，左右轮峰值1.831/1.893m/s；
结束后新记录窗口内导航/FOD/安全/最终指令各20/20/20/49条及双轮各50条全部为零。
J6M整板CPU平均28.55%、峰53.42%，move_base平均8.98%/峰151.04%（约1.51核），coverage
manager平均15.40%/峰103.36%。本轮视觉始终DISABLED/GPS_ACTIVE、AI IDLE，不构成视觉
3%～97%或消失后0.50m实车验收。原始证据完整保留于
validation/coverage_live_20260912/20260914_113507/（311MB）；发现自定义enforced_path
仍被高频完整落盘后，仅修正被动记录器按路径身份/点列变化去重并通过py_compile，未改业务
节点。11:50无缝续录于validation/coverage_live_20260912/20260914_115042/，tmux socket
runtime/run/coverage_monitor_20260914.tmux.sock；当前服务/Qt保持运行、覆盖COMPLETED、
localized=true、视觉DISABLED、AI/待清扫空闲，记录器继续等待后续实验。未删除原证据，
未启停导航、部署、发初值/目标/速度、改授权、提交或推送。

2026-09-14 09:53 参数重部署/完整启动最新：用户明确要求重启、部署后启动实验。部署前
沿用map_20260914_064202（fused），视觉DISABLED/GPS_ACTIVE、无活动覆盖，安全速度为零。
统一stop退出0；同历史停机问题，仅核对并SIGKILL服务cgroup唯一残留SSH881583，原入口
继续按PID停止J6M并退出0，两端停净。统一deploy退出0、43秒，J6M current由091120切至
20260914_095005。新增3%～97%、消失10帧后0.50m参数、节点及两launch安装文件与本地
逐字节一致；板端静态、库加载/Python导入及实际参数核验通过，私有配置/运动授权不变。
随后统一start显式加载原地图退出0，完整runtime健康通过，Qt PID995913、服务active/
MainPID987924；相机、检测、BPU、MID360、CAN、move_base、覆盖、AI和地图就绪。运行参数
实测0.03/0.97/0.5/10，视觉DISABLED/GPS_ACTIVE、覆盖IDLE/active=false，3条安全速度
均零。定位WAITING_INITIAL_POSE/localized=false，操作员须设置本次真实初值后实验；未替
操作员发初值/目标/速度。/map剩余908MB、97%，未删除旧发布物。仍0914未额外提交/推送，
证据validation/redeploy_03_97_0914/。下方“本地未再次部署”和091120均为历史。

2026-09-14 后续参数最新（本地未再次部署）：用户要求初始锁定3%～97%、消失后前进0.5m。
已同步YAML、节点默认/合法范围、两处launch默认及测试/README；仍连续10个正常检测帧
消失，从第10帧当前odom起算0.50m，原0.55m硬上限和安全/停车回执门保留。
98项视觉/完成单元及11项仅lo的真实节点ROS测试通过，launch展开为0.03/0.97/0.50/10帧。
09:35旧采样81帧q=0.953在新范围门离线回放81/81通过，不等于实车锁定验收。
本轮未部署/启停/发真实控制/改授权；最近板端091120仍是5%～90%及0.30m。
仍0914未额外提交/推送，证据validation/visual_range_03_97_0914/，报告
docs/AI_VISUAL_NAVIGATION_0914.md末节。下方09:14部署不包含本次新参数。

2026-09-14 09:14 部署最新：用户明确“现在部署”。部署前实测视觉DISABLED/GPS_ACTIVE，
覆盖COMPLETED/12/12/active=false/paused=false，采到安全速度零，已非08:19中止状态。
统一stop退出0；仅对服务cgroup唯一残留SSH466600核对后SIGKILL，原入口继续按PID记录
结束J6M栈。两端停净后统一deploy退出0、165秒，current由20260912_165656切至
20260914_091120。11包原生Release，两个规划器修改单元均重编译，14项源码/安装文件
SHA256逐字节一致；新增SetOrdinaryGoalPolicy两端MD5为81b3245c2d180492c0e35d90e5dd93a6。
板端两个规划器库即时符号加载、安装Python导入、0.05～0.90参数及两端静态健康通过。
本机AI加载本目录当前源码，私有配置/主运动标记哈希不变。09:14:33两服务inactive/PID0，
板端run空、11311/11681无监听，未启动Qt/导航、发初值/目标/速度或恢复任务，非新实车验收。
/map剩余929MB（97%），未删除旧发布物或日志。仍0914未额外提交/推送，证据
validation/deploy_0914/，报告docs/AI_VISUAL_NAVIGATION_0914.md末节。下方未部署属历史。

2026-09-14 近处锁定/AI朝向最新（本地未部署）：按用户要求先新建0914并提交修改前
完整源码043e28d，原0912引用未移动，未推送。现场08:07视觉停留ACQUIRE至75秒超时，
08:19当前检测82帧均因纵向锚点q=0.953超原0.20～0.80而拒绝；没有故障全程检测证据。
现初始捕获范围改0.05～0.90（闭区间，0.953仍不接受），原其他筛选/安全门保留。
AI按本次新COMPLETE回执确认清扫，另等真实停车及控制权归还再执行下一步。原move_base
不能直接省略四元数表达yaw不限；现MCP角度可省略，默认返回冻结点也只约束x/y，明确
角度保留。配套新增按目标登记服务、导航桥登记后复查取消、普通Navfn计划策略序号及
TEB任意终点yaw到达判定，保留XY/新鲜物理停车门，固定角/Qt/覆盖不继承策略。
Release、AI229项、视觉98项、桥28项、原覆盖177项、AI模拟31组、自由朝向真实插件
隔离10项、原固定挡位5项、视觉真实节点隔离11项及静态通过。ROS只有lo/合成传感器，
不是实车/云端/ASR验收；初期失败与修复证据保留于validation/changes_0914/。
源码修改未额外提交/推送/部署，未启停或接管实车、发真实目标/速度、改授权/私有配置。
08:19导航服务运行，J6M current仍20260912_165656；下方9月12日停机状态为历史。
旧板端无新协议时AI拒绝无朝向目标，需双机配套部署，不能只重开新版AI。
详见docs/AI_VISUAL_NAVIGATION_0914.md与docs/VISUAL_ABORT_ANALYSIS_20260914.md。

2026-09-12 16:58 视觉同步版部署最新：用户随后明确“杀死所有程序，然后部署到J6M”。
通过vision_0908原统一stop结束当前视觉Qt/相机/AI/底盘及板端旧栈；仅对该服务cgroup
唯一残留SSH116777显式SIGKILL，随后原入口按PID记录清理J6M，stop退出0。两名被动
记录器通过其私有tmux发送INT正常退出，日志保留；没有宽泛杀进程或删除源码/Release。
确认两服务inactive/MainPID0、板端两run目录空、11311/11681无监听后，从本目录0912
统一deploy，44秒退出0；导航current由20260912_134337切到20260912_165656，视觉
current仍20260912_163710未变。板端静态健康通过，视觉控制/状态机/回执/配置/launch、
原覆盖连续性源码配置及私有配置共10项两端SHA256一致。本机Release及隔离ROS11/11
已在部署前通过；没有启动Qt/导航/恢复任务，不属于新导航版实车验收。主运动/FOD持久
授权和私有配置未改，两个服务仍inactive/MainPID0，板端无ROS/BPU监听。/map剩余1.5G、
95%，未删除旧发布物。未提交或推送，证据validation/visual_sync_0912_r70Yhl/，报告
docs/VISUAL_SERVO_SYNC_0912.md末节。下方“未部署/保持视觉运行”为同步阶段历史。


2026-09-12 视觉伺服同步最新（本地，未部署/启停）：按用户要求，从同级vision_0908的
robot_j6m_ws_vision_0912当前工作树选择性同步到本目录0912，共同检查点5cb44aa，未切换
分支、提交或推送。初始最大距离/模式接管≤8m；去掉关联框面积及深度平面/纸类拒绝；
锁定后连续10个正常检测帧消失，再从第10帧当前odom起前进0.30m即提交COMPLETE。
取消底部中央、最短接近、5/20cm预停车、回中等待、尾段4°/8cm及最终5mm漂移门；
真实停车另由新odom更新回执，AI等待同会话/新序号/COMPLETE/stopped/控制权归还。
原硬件/时效/控制租约/里程异常保护保留。旧显式空completion_policy兼容坐标覆盖，
当前待清扫visual_servo使用新规则。导航覆盖连续性修复、AI详细原因/本地失败摘要保留，
只局部合并AI延迟回执等待；未同步视觉部署路径迁移或待清扫独立路径头修复。
280项单元/契约、162项原覆盖状态机、31项AI模拟及Release/静态通过；参数展开确认
10帧/0.30m/8m。首次隔离ROS9/11，两项control heartbeat lease expired，原始失败保留；
编译结束后同源码参数正常优先级重跑11/11通过，见docs/VISUAL_SERVO_SYNC_0912.md。
源码24文件与来源逐字节一致，
原覆盖/AI后端/部署隔离脚本/私有配置/运动授权哈希不变。来源视觉版操作员实验曾于
16:41:50和16:50:29返回COMPLETE，尾段0.3081/0.3044m，不等于本导航版部署验收。
早先检测源帧龄0.508/0.506s和初始q=1.0拒绝仍未修，本次仅同步。
只读J6M导航current=20260912_134337、视觉current=20260912_163710；本机vision服务
active/MainPID113399、navigation inactive。本轮未启停/部署/发目标速度或改授权，
不接管当前实验。证据validation/visual_sync_0912_r70Yhl/，详见上述同步报告。


2026-09-12 覆盖连续性修复部署最新：用户明确要求部署。部署前本机vision_0908服务已
inactive，但J6M残留13:18从/map/robot_j6m_vision_0908启动的旧栈（实际库release
20260912_131623）；通过其原统一stop入口退出0，随后11311无监听且受管进程清空。由本目录
0912统一deploy退出0、44秒，J6M原生Release `20260912_134337`安装并切换current。安装的
coverage_manager.py与coverage.yaml SHA256为37e848b3...4829、504bda6d...fd7b，与本地
逐字节一致；0.30m map/odom残差、0.50m/s速度余量、0.15m距离余量、5秒制动记录四项参数
均在安装树。板端静态健康通过。当前导航/Qt/master均未启动，无活动任务可恢复；未发初值、
目标、覆盖请求或速度，原主运动/FOD授权配置未改。/map剩余1.7G、使用率94%，后续Release
需关注空间。源码仍在0912工作树，未额外提交/推送；实车五类回归尚待启动后验收。报告
docs/COVERAGE_POSE_CONTINUITY_RECOVERY_FIX_20260912.md，部署日志与核验位于
validation/coverage_pose_continuity_fix_20260912/。

2026-09-12 覆盖定位连续性/手动恢复修复最新（本地未部署）：现场紧凑日志确认三次
`localization jump detected`；第一次暂停后的1秒窗口导航指令已归零，但左右轮仍在制动，
峰值1.781/1.856m/s、末值1.262/1.138m/s。共记录34次`coverage resuming`和33次立即
`continuous ... history is untrusted`，旧逻辑一旦作废跟踪器，按钮只清外层暂停标志，不能
重建历史。现将大于原0.50m的map位姿步长与新鲜底盘/odom相关：检查时间戳、速度包络，
将odom位移旋转到map后要求残差不超过新增可调0.30m；真实物理运动可继续，不能解释的
重定位仍停车。第一次手动恢复等待两份不高于0.08m/s的停车证据，要求原入口历史、odom
连续性、原线走廊/朝向和不倒退进度，随后保留最大进度并只下发剩余线段；失败信息细分为
缺odom、时间戳倒退、速度包络、map/odom残差或离开走廊。暂停后最多5秒继续记录制动路径，
连续两份停车证据即一次性关闭，后续遥控移动不计入。跟踪器入口准入统一为管理器原
0.40m/25°；入口0.45m捕获段沿用0.40m，进入线后仍为原0.30m质量门，不影响合格入口进入
SWEEPING。新增配置为map/odom残差0.30m、速度余量0.50m/s、距离余量0.15m、制动记录5秒；
原完成/停车/安全门、Hybrid/TEB/定位/驱动/速度/授权未改。最终Release构建、214项coverage
独立测试（162状态机+26几何+15契约+3命令复用器+8 Hybrid C++）、全工作区648项汇总及
静态检查通过，0失败；直接catkin_make曾因未加载依赖前缀在CMake阶段失败，统一隔离环境
重跑通过。源码位于本地0912，尚未提交/推送/部署。最终只读核对显示本工作区导航service
inactive，但另有/home/slam/robot_j6m_ws_vision_0908独立服务自12:31 active且Qt进程存在；
它不包含本地0912修复，本轮未启停或改动。当前工作区直接rostopic因master配置不同未取得
可信任务/速度快照；未发初值/目标/速度。报告
docs/COVERAGE_POSE_CONTINUITY_RECOVERY_FIX_20260912.md，证据
validation/coverage_pose_continuity_fix_20260912/。实车五类场景尚待部署后验收。

2026-09-12 0912保存/视觉结果反馈最新（未部署/启停）：用户要求先保存再复制源码，随后
修改本目录。已新建0912并提交修改前完整源码5cb44aa；同级Vision工作树原未提交修改
先保存到backup/vision-0908-before-0912，提交41f32d4，然后切到新分支
robot_j6m_ws_vision_0912，源码与5cb44aa一致。两个目录共用Git，未改原旧命名分支或推送。
私有配置/地图/模型/日志/构建目录原地保留；Vision副本含导航绝对路径保护，尚不是可直接
启动的迁移版本，不能绕过路径保护使用旧构建产物。后续业务修改仅在本目录0912工作树，
Vision新分支保持修改前快照。MCP补齐visual_reason/mode_reason/service_message；视觉失败
结构化区分未启动、已启动未完成及启动不明，保留清理前原因。本地生成失败摘要，防止云端
把已运动但中止说成从未执行；退出服务应答不作为物理停车证据。原COMPLETE/完成回执、
启动与所有权门、0.50s传感器时效、候选筛选和5cm保护均未改，检测时效/近距离候选导致的
实际ABORT尚未修复。216项AI回归、49个隔离连续任务/存档云端重放、84项视觉单测、52项
Qt契约、Release构建及静态检查通过（639是已有结果汇总）；
不是本轮真实云端/ASR/实车验收。源码修复未额外提交。只读本候选J6M current仍
20260912_011633（导航专属rootfs，非历史基础rootfs）；导航服务active/MainPID=282115、
Vision inactive，不据此推断当前任务/速度/定位。未启停/发初值/目标/速度、部署、改授权，
未停止原被动记录器。详见docs/AI_VISUAL_FEEDBACK_0912.md，证据validation/ai_visual_feedback_0912/。

2026-09-12 01:39 最新启动/首轮实验监测：用户要求启动并在实验期间持续监测。通过本工作区
统一start显式加载map_20260829_221335_gimp_cleaned（fused），J6M current为新部署
20260912_011633；网络、ZED、MID360、CAN、完整ROS图、Qt地图和639项已有runtime汇总检查
通过，定位报告LOCALIZED/initial_pose=applied。操作员在记录器就绪前已开始计划
f797c6317857437cbdac3cf0a5bac383；只读记录从第4/8段中途01:31:51开始，01:34:12完成8/8，
全程捕获窗口无PAUSED。7次Hybrid请求均有有效路径、最长0.443秒；捕获8次TEB固定挡位反号
拒绝、1次2秒低进展取消、3次cusp后缀角差失效（6.1/21.0/36.7度），均恢复完成。两次
WAITING_OBSTACLE各约1秒且仅为retry 1/3，没有耗尽整轮，因此不能据此声称新增“整轮耗尽
后继续”分支已实车覆盖。记录窗口整板CPU均值27.86%/峰50.0%，move_base均值9.34%/峰
155.03%（约1.55核），清扫段实测/odom含停车均速1.078m/s、运动时1.109m/s。结束后覆盖
COMPLETED/active=false，定位保持LOCALIZED，无四类急停，最近1秒所有命令及双轮反馈全零。
首轮原始记录234M，因旧记录器逐次保存高频完整局部路径；完整保留后已只停止记录器并换成
紧凑版，不影响导航，新的持续记录目录为validation/coverage_live_20260912/20260912_013815/，
tmux socket为runtime/run/coverage_monitor_20260912_v2.tmux.sock。紧凑版仍保存覆盖自有完整
路径、动作/Hybrid结果、状态、速度、位姿和0.25秒J6M CPU，只压缩高频move_base路径及里程计
冗余字段。导航/Qt继续active等待后续实验；助手未发送初值、目标或速度，也未暂停/取消任务。

2026-09-12 01:20 全工作空间双机部署最新：用户明确要求把整个当前工作空间源码部署到
相应位置。部署前只读确认旧板端覆盖为COMPLETED/active=false，连续3条/cmd_vel_safe
全零，底盘和避障反馈陈旧；随后由本工作区统一stop完整结束PID记录的旧J6M栈，未改变
原运动授权。统一deploy退出0、44秒，J6M current已从20260911_160159切换为
20260912_011633；11个板端包原生Release构建并安装，板端静态健康通过。覆盖自动续试、
待清扫和视觉会话完成回执源码均已安装，6个关键源码/接口/launch文件与本地SHA256逐项
一致，SetFodCleaningTarget两端MD5均为0c6be3379e5f6ea397f1cbc11681d36b；安装参数保持
2m转场前视、4m入口恢复、5m清扫碰撞窗口及待清扫1--2m视觉交接。本机随后全工作空间
Release构建退出0，Qt、AI/MCP、视觉、消息和导航产物已更新到当前devel；导入及接口字段
检查通过。当前两套本机服务inactive/MainPID=0，J6M旧PID已停且11311无监听，未启动Qt/
导航、未发初值/目标/速度、未恢复任务，也未做实车验收。/map剩余2.3G、使用率91%，
后续继续部署前需关注空间。源码仍在robot_j6m_ws_vision_0911工作树且未提交/推送；
证据validation/full_workspace_deploy_20260912/release_20260912_011633/。下方“未部署”均为
对应阶段的历史状态。

2026-09-12 AI连续任务修复最新（本地，未部署/启停）：用户要求继续修复并模拟验证。
仍在robot_j6m_ws_vision_0911。规划前对新请求尝试冻结可用map位姿，不再用少量返回关键词
触发；保留补答原快照，短停止/取消跳过采集，人的位置/历史点不能由机器人快照冒充。
动作续句等待3秒、连续动作4秒、查询补答1.5秒，停止/说完了即时；不是无限合句或ASR
准确率验收。J6M视觉节点新增ai_completion_protocol=1/visual_session纯结果元数据，
实际启用分配新ID，原最终停车COMPLETE提交处生成递增回执且跨DISABLED保留。AI要求同ID/
新序号/COMPLETE/停车/视觉不活动/仲裁回导航模式才成功，ABORT/手动退出/旧回执/报错不
放行下一步；启动响应丢失保留归属门，未知或外部会话不擅停；启动中取消在回执到达后补
退出。泛指“去区域清扫”无区域依据不得默认选A。未改视觉运动/5cm保护、综合页导航、
桥、覆盖/TEB/底盘；原未提交待清扫/Qt及另一任务覆盖自动重试修改保留。
最终真实云端基础12/12、扩展8/8、最低原话5/5（合计25，一次响应，平均2.00s/最长2.70s）；
隔离47/47（17控制、10停顿、2四步对照、18份最终云端三段计划回放），AI206/视觉84/Qt52
单测契约及真实视觉节点11项合成传感器ROS通过。只有lo的网络/PID隔离，非实车、非录音/
Whisper重测。Release/静态/暂存安装及模块导入通过，静态639为已有结果汇总非新增测试数。
中间云端11/12、旧夹具/路径/ROS相对XML路径错误均保留，详见
docs/AI_CONTROL_REPAIR_20260912.md与validation/ai_control_repair_20260912/；README全部15
工具及AI逻辑已更新。私有配置/主运动标记/真实区域哈希不变，无新增测试区域。本轮未
部署、启停Qt/实车、发真实初值/目标/速度、改授权、提交或推送，未重新查询板端状态。
最近历史J6M版本仍20260911_160159且旧master曾运行，不可宣称两端停净。此次需要配套
部署J6M视觉元数据，旧板端无协议时AI拒绝启用视觉，不能只重开Qt；后续须用户确认后
统一停净/部署/启动和新真实初值。下方“仅模拟未修复/无需J6M部署”属历史。

2026-09-12 AI模拟与README最新（未改业务/未启动）：用户要求模拟多组实验，随后明确
最低原话“清扫A区后回到当前点进行视觉清扫”，最后要求更新全部MCP/AI逻辑到README。
最低用例真实云端5/5、对应隔离执行5/5：规划前只读记录一次map坐标/朝向，A区匹配真实
名称A，覆盖完成/所有权释放后回到冻结点，再开视觉；模拟清扫后位姿变化不改变返回点。
既有187项AI/MCP/ASR和52项Qt契约重新通过；基础云端12例重复3轮36/36。新增返回表达
仅3/8，“记住现在位置完成后回来”“返回现在这儿”“回原地”等5例因本地正则未触发
位姿采集而整单追问。文本停顿1.6/2.5秒且前半句完整时仍拆成两个READY。视觉执行器仅
凭恢复导航模式就判成功：ABORT后显式退出、无COMPLETE退出、异步仍active及工具报错
注入均误报；四步扩展中视觉未完成仍调用后续导航。正常FOD_ABORTED/FAULT_STOP可拒绝，
不能断言9月11日现场也发生同一种误报。最终49个模拟场景42通过/7未满足预期，包含23份
云端三段计划回放；失败不是通过项，不与云端/单测累加作实车验收。Qt无心跳超过5秒的
真实等待及600秒合成年龄场景均正常完成，授权未误撤。新增simulate_ai_control.py和云端
测试--minimum/--extended，只有lo的网络/PID隔离、内存机器人反馈，无真实ROS控制、无
录音/播放/Whisper实测。本轮未修上述三类业务问题；测试夹具早期错误保留并已修正。
详情docs/AI_CONTROL_SIMULATION_20260912.md，证据validation/ai_control_simulation_20260912/。
项目README AI章节现含全部15工具（5只读/10变更）、参数、授权、合句/上下文、动态区域、
冻结点、顺序/清理/日志及未修边界；包README已同步。主运动/私有配置/真实区域哈希不变，
未新增测试区域、构建、部署、启停、提交或推送，原未提交业务修改保留。分支仍
robot_j6m_ws_vision_0911；本机两服务inactive/PID0，J6M只读current仍20260911_160159、
11311仍为晚间旧rosmaster PID98896，未接管/停机/发目标，也未测当前定位或速度。

2026-09-12 覆盖转场自动续试最新（本地，未部署）：用户要求Hybrid/TEB连续无进展重试
耗尽，以及转场路径偏离/边界交接后未确认停车时不再进入人工暂停。已在当前
robot_j6m_ws_vision_0911工作树修改coverage_manager：Hybrid线间转场和清扫入口rolling
Hybrid对准的一轮有限重试耗尽后，保留同一段及其依赖清扫线，进入WAITING_OBSTACLE并按
原任务transit_replan_period持续开启下一轮，直至成功、安全暂停、取消或节点关闭；不再
要求Qt点击恢复。路径偏离、越过转场边界或进入清扫入口后仍先精确取消旧Goal，原3秒内
未停车则持续等待新鲜物理里程计，连续2个样本|v|<=0.08m/s后才自动重规划/交接，等待时
不提交新目标。操作员暂停/取消、FOD仲裁、定位丢失、雷达失效和VCU故障仍优先中断；首次
Navfn入场重试耗尽仍人工暂停。未修改0.40m入口准入与0.30m清扫tracker武装差异，该问题
仍待另修。Release全工作区构建通过；覆盖包独立XML共205项（状态机153、几何26、契约15、
mux3、Hybrid C++ 8）0失败，catkin汇总213含同一8项C++嵌套重复；新增自动重试、停车等待
及并发人工/安全暂停回归；直接catkin测试
曾因绕过依赖环境缺costmap头失败，规定optimized环境重跑通过。报告
docs/COVERAGE_AUTOMATIC_TRANSITION_RETRY_20260912.md，证据
validation/coverage_auto_retry_20260912/。未部署/启停导航/发目标或速度，
未改配置、授权、提交或推送；此前未提交修改全部保留。

2026-09-11 深夜AI审计/修复最新（本地，未启动）：用户要求删除AI看门狗、修复区域覆盖
→返回当前点→视觉清扫，随后要求核对今天两轮日志。凌晨覆盖b9000178最终完成，后续
00:33:27普通导航控制失败；00:38:23视觉LOSS_CONFIRM期间移动0.063m超过原0.050m门
而ABORT，不能凭时间断言这些后续动作均属同一AI计划。晚间22:01:02有点位依据校验
拒绝；22:02的AI批次2faf091e在完成3条线后于22:04:57因Qt心跳超3秒撤权取消，Qt直到
22:44:13才退出。两轮原话/完整云端计划未持久化，不能声称已逐项还原语义；详见
docs/AI_CONTROL_AUDIT_FIX_20260911.md。现仅在robot_j6m_ws_vision_0911删除Qt3秒
自动撤权/取消逻辑，保留一次令牌认证、主动停止/撤权/节点关闭及底盘/定位/急停保护，
J6M真实AI后端失联GoalID租约保留。Qt不再重复已认证会话心跳。AI规划前可只读冻结
新鲜LOCALIZED map->base_link及地图摘要，作为本轮返回点，保持朝向；不把camera_init
当map、不在覆盖后重新取当前位置。补答保留本轮快照，独立新任务不复用旧预览；地图
变更阻止执行。云端通用提示和本地门检查返回/视觉步骤的存在与顺序；新增关键AI事件
request_id/原话/计划/步骤/拒绝原因脱敏日志。综合页导航、桥、覆盖/TEB/底盘及原视觉
0.050m保护未改。187项AI/MCP/ASR离线及52项Qt契约通过；真实目录+合成位姿云端最终
12/12 plan-only、均一次响应、平均2.04s/最大2.39s，首轮10/12缺信息格式拒绝证据保留。
新增日志测试首轮离线时间夹具失败后修复通过，最终Release与Qt重链退出0。
证据validation/ai_control_fix_20260911/。
本轮不需要更新J6M控制接口，最近已核对板端仍20260911_160159；本机两服务inactive/PID0，
但23:56只读发现J6M晚间21:52–21:53会话仍有rosmaster PID98896/11311监听及导航
roslaunch PID99133，不能声称两端已停净，未据此推断当前定位/任务/速度。助手未开
Qt/导航、未停止或接管板端旧会话、未发初值/目标/速度、未改私有配置或运动授权、未
提交/推送。后续重启须经用户同意先统一停止并核对两端停净，再同目录带原静态地图
完整启动Qt和AI，不能新Qt连接旧AI或叠加板端旧会话。实车连续任务尚未验收。

2026-09-11 16:41 再次启动最新：用户要求再次打开Qt。启动前两套服务inactive、无Qt，
J6M current实测仍20260911_160159，时钟正常，无11311监听。通过本工作区统一start
显式带原map_20260829_221335_gimp_cleaned（fused），退出0，内外runtime通过。
导航服务active/MainPID=231723，Qt PID=239306来自本工作区；地图READY、原5条区域
已加载，当前WAITING_INITIAL_POSE。本会话VCU报告硬件/机器人急停，未解除；待清扫/
覆盖/AI均无活动任务，ASR可用medium/cuda，三道AI授权false。16:39:24–39被动观察
三条速度指令及双轮反馈全零。主运动/FOD授权及配置哈希未改，未发初值/目标/速度、
未恢复旧任务。本轮未构建/部署/改业务逻辑/提交/推送；原未提交修改保留。证据目录
validation/pending_navigation_0911/reopen_20260911_1637/。须操作员设置本次真实初值，
不能沿用16:06会话；下方PID和运行快照均为历史。

2026-09-11 16:06 待清扫新版部署并完整启动最新：用户要求启动修改后的Qt，并明确带
静态地图、全部功能。仍在robot_j6m_ws_vision_0911，未提交/推送。统一stop退出0，
清理断电遗留记录；板端时间从2025-06-26校正至当前，随后deploy退出0、47秒，
J6M current已从20260911_014849切换为20260911_160159。板端静态检查通过，
共享接口两端MD5核对及实际运行服务握手为0c6be3379e5f6ea397f1cbc11681d36b，
视觉状态pending_target_protocol=2，待清扫IDLE/visual_servo/stand_off_m=1.5。
统一start显式加载map_20260829_221335_gimp_cleaned（fused），退出0，内外runtime
检查通过；635为已有Catkin结果汇总，不是本轮新测635项。导航服务active/MainPID=54061，
Qt PID=62108来自本工作区，旧Vision服务inactive。导航/覆盖/视觉/相机/BPU/AI节点均在，
Qt地图READY，加载原5条区域。当前WAITING_INITIAL_POSE，须操作员设置本次真实初值；
VCU报告遥控/机器人急停，未解除。AI/覆盖/待清扫无活动任务，ASR可用medium/cuda，
语音/解析/控制授权均false。16:05:39–54只读观察3条速度指令及双轮反馈全部为零。
主门true/FOD=true及两端私有配置、授权标记哈希未改；未发初值/目标/速度或恢复旧任务。
启动期Qt有等待TEB动态重配置服务超时提示，尚未定位时move_base未完成初始化，不能
据启动检查宣称任务/实车验收。记录docs/PENDING_LITTER_READY_20260911.md，证据
validation/pending_navigation_0911/launch_20260911/。下方“未部署”均为上一阶段历史。

2026-09-11 待清扫修改交付最新（本地可部署，未部署）：用户确认按方案实施且仅允许
robot_j6m_ws_vision_0911分支，当前分支已核对，原0911仍为e648da6。已修正普通目标
订阅者身份检查，允许Qt显示；待清扫独立选择默认1.5m、朝向垃圾的可达接近点，
检查现有代价地图完整外廓并查询原普通规划。近处侧后方/过近目标仍导航调整；到
1–2m且朝向合格，精确结束本次目标、确认不可重发及连续停车后交接FOD。原综合页
主窗口/RViz、导航桥、模式仲裁、覆盖/TEB/底盘移动逻辑均未改。仅待清扫指定新的
visual_servo完成策略：允许获取时以唯一近邻map证据重关联检测ID，尾段用原视觉
滚刷通过流程，不再额外要求map10cm；保留本轮令牌/新完成序号/停车证据门。Qt中文
状态和失败日志已补齐。SetFodCleaningTarget新增策略/令牌字段，MD5为
0c6be3379e5f6ea397f1cbc11681d36b，两端部署前后自动核对，旧空策略兼容保留。
Release、121项控制单测、108项Qt/导航桥/双机契约、7项真实协调器/桥合成后端ROS场景、
11项真实视觉控制合成传感器测试、Qt页12项/注册存储20项、静态及本地暂存安装检查
通过。ROS测试网络仅lo，不等于实车/完整TEB移动验收；首轮序号关联与夹具失败证据
保留。报告docs/PENDING_LITTER_READY_20260911.md，证据validation/pending_navigation_0911/。
本轮未部署J6M、未开实车Qt/导航、未发真实目标/速度。主门/FOD授权及私有配置哈希
未改；本机两服务inactive/MainPID=0，最近已验证J6M版本为20260911_014849，本轮
未重新实测板端current。用户后续自行完整停机后执行本目录optimized.sh deploy。
修改尚未提交/推送；不要把下方“待确认”当作当前状态。

2026-09-11 待清扫入口分析/新分支最新：用户要求先保存、再提出方案，确认后才改。
已把当前完整源码保存到原0911，提交e648da6，随后在本工作区建立并切换至
robot_j6m_ws_vision_0911；未另建目录、未改旧Vision工作树、未推送。已离线复现
待清扫前置条件把simple_goal订阅总数必须为1，和Qt已启用的Navigation goal显示
订阅冲突：2个订阅者即拒绝，综合页发点则无此门；另复现1m后方目标跳过导航后因
180°朝向拒绝。现场点击日志未取得，J6M SSH当前超时，不能声称该次现场原因已经
逐条实测。本机两服务inactive/MainPID=0；最近已验证部署仍为20260911_014849，
本轮未改业务代码/构建/部署/启停/授权、未发目标或速度。待确认方案见
docs/PENDING_LITTER_NAVIGATION_PLAN_20260911.md：同普通导航入口、1–2m接近停车后
视觉交接，并按用户新要求拟恢复普通视觉完成逻辑，取消旧10cm额外完成门；尚未实施。

2026-09-11 转场交接修复部署最新：用户明确“部署”，已统一停机退出0，清理断电遗留
PID记录；无可达本候选ROS master/活动目标。统一deploy退出0、169秒，J6M current已
从20260910_233119切换为20260911_014849。断电后板端时间回到2025-06-26，构建早期
有clock-skew警告，已用原校时脚本修复（偏差-2ms）；日志确认两个修改单元均原生重编译。
6个源码/配置及安装头文件/两配置SHA256与本机一致，新库包含标记和恢复代码，两库即时
符号解析加载通过，板端静态健康和两个交接接口MD5一致性通过。安装launch展开2m前视、
4m入口恢复，安装碰撞配置5m保持。私有配置/运动授权哈希保持，主运动及FOD持久授权
未改。当前两服务inactive/MainPID=0、板端本候选run目录为空、11311无监听；未启动
导航/Qt、未发真实初值/目标/速度、未恢复任务。后续由操作员正常完整启动Qt，设置
本次真实初值并等LOCALIZED后实验，实车摆动改善尚待验收。报告已追加到
docs/FIXED_GEAR_HANDOFF_FIX_20260911.md，证据validation/fixed_gear_handoff_20260911/
deploy_20260911_014837/。0911修复仍未额外提交/推送，修改前检查点8bdd8e2保留。

2026-09-11 转场/TEB交接修复最新（本地未部署）：用户要求保存当前源码到0911后添加
动作标记和处理逻辑。已从0909建立0911，修改前完整源码检查点为8bdd8e2（包含此前
待清扫垃圾/视觉/Qt修改，不含忽略的私有配置和运行资产），未推送。随后覆盖全局规划器
每次同步装载路径分配动作序号，经PoseStamped.header.seq传给TEB，周期规划/租约刷新
不换号；同挡位新段及同线恢复均清除旧优化带和振荡/不可行历史。错误挡位仍输出零，
每次故障首次只重置一次，要求连续5次可行且明确同向的解后放行；零速和其他失败不计
稳定证据。新增互斥锁保护优化/重配/装载，覆盖管理器及2m前视、4m入口恢复、5m碰撞
窗口未改。Release、220项独立单测、5个真实插件隔离交接场景及静态检查通过；合成
目标/速度仅在lo网络命名空间，未接实车。报告docs/FIXED_GEAR_HANDOFF_FIX_20260911.md，
证据validation/fixed_gear_handoff_20260911/，最终模拟20260911_014531_845048。
用户中途报告断电后继续；只读SSH确认板端已重新在线，current仍20260910_233119。
本轮未部署、重启导航、恢复任务、发真实初值/目标/速度；断电前LOCALIZED不可视为当前
会话验收。主运动/FOD持久授权保留。修复源码和报告仍在0911工作树未额外提交/推送，
实车摆动改善需后续部署和现场实验验证。

2026-09-10 FOD持久运动授权最新：用户明确“运动授权打开，后续Qt打开要能实现全部功能”。
主门true及已有motion_authorized.ok保留，将本目录私有配置FOD_MOTION_ENABLED从false
改为true，并同步J6M本候选配置；两端SHA256一致。此为用户要求的持久配置，下次正常
完整启动无需额外--authorize-fod-motion；不能再按下方历史false在收尾撤回。没有改公开
launch默认值、速度上限、急停/定位/传感器/唯一速度源保护，也未自动开启麦克风或AI
控制授权。两端配置读取及板端安装launch展开验证allow_motion=true、external_estop_override=false
通过；只是启动参数验证，没有运行态或实车移动验收。Qt由用户自行启动，两服务仍
inactive/MainPID=0，J6M current仍20260910_233119，无导航运行PID/11311监听；未发初值/
目标/速度，本轮无需重新构建部署。完整启动后须操作员给本次真实初值并等LOCALIZED，
语音/解析/AI控制继续通过Qt各自授权入口使用。备份及证据位于
`runtime/fod_motion_authorization_20260910_DgGBPN/`；板端原配置保留为
`dual_host/config/dual_host.env.before_fod_motion_20260910_DgGBPN`。未提交/推送。

2026-09-10 待清扫垃圾部署最新：用户明确“部署，部署后告知，我再打开Qt”。已通过本目录
统一deploy入口在J6M原生安装release `20260910_233119`，退出0、53秒，current已从
`20260909_003347`切换到该版本。新增共享消息包随release安装；板端静态健康通过，
5个消息/服务MD5与本机一致，11个关键源码/配置/接口文件哈希一致（两节点忽略catkin
改写的shebang）。板端实际安装的协调器/视觉控制节点导入及launch节点展开通过，包含
`/pending_litter_cleaner`；未启动ROS节点。两服务最新仍inactive/MainPID=0，J6M11311
无监听、无本导航运行PID记录。主门true、FOD=false保留，未开Qt、未发初值/目标/速度，
由用户自行通过本工作区统一入口完整启动；不要自动打开Qt或开启FOD运动门。本机Qt
已构建，本轮未再改业务逻辑、提交或推送，未进行实车移动验收。证据位于
`validation/pending_litter_cleaning_20260910/deploy*.log`与`deploy_verification.json`，
部署记录已追加到`docs/PENDING_LITTER_CLEANING_20260910.md`。下方“未部署”属前一阶段。

2026-09-10 待清扫垃圾合并/控制最新（本地未部署）：按用户要求，将独立视觉工作树
`feature/vision-map-localization-0908` 的 Qt 待清扫界面及地图定位/消息依赖合入本目录
`0909`；来源工作树未改。本地新增独立 pending_litter_cleaner：普通 move_base 导航到
距所选点 1.50m 的朝向目标位姿，等待确切 GoalID 完成及禁止旧目标重发，再经原 FOD
仲裁器交接视觉。仅此入口启用选定 ID/map 坐标绑定和前方参考点轨迹 10cm 完成判据；
倒车、零速漂移、方向切换段不计，且要求停车确认。普通视觉/智能语音保留原 0.50m
消失后流程；覆盖、move_base/TEB、导航所有权桥和底盘驱动移动逻辑未改。GPS_ACTIVE
本身不作为成功，必须匹配本次新序号、ID、坐标和前向覆盖证据；取消末端竞争也拒绝
提交成功。Qt 异步服务/中止反馈/精确删除已接通，旧锁存结果不会删除新任务点。
相机参数同步 Vision 中用户确认的前0.42/右0.13/低0.03m、42°；清扫参考点0.52m仍需
现场核对，不能把代码10cm判据当物理精度验收。Release及最终Qt重链通过，控制90项、
地图/Qt契约75项（含检测解释器包装，内部15项不重复计）、依赖24项、Qt页面12项、
注册/存储20项及隔离真实视觉控制节点10项通过；静态健康通过。隔离测试只有lo和合成
传感器，无实车ROS连接；选点约0.20m消失后前进即完成，横偏30cm触发行程门而不完成。
证据 `validation/pending_litter_cleaning_20260910/`，报告
`docs/PENDING_LITTER_CLEANING_20260910.md`。两服务最新只读均inactive/MainPID=0，J6M
导航current仍`20260909_003347`，本轮未部署、未开真实Qt、未发初值/目标/速度。主门true、
FOD=false保留，私有配置/授权及真实区域未改。部署脚本已补齐新消息包白名单和服务/
安装检查，仅本地语法校验，未实际部署。新源码未提交/推送；后续须先部署共享
消息及J6M控制代码再完整启动。用户要求修改后先询问是否打开Qt，不能自动启动/放开FOD。

2026-09-10 19:40 云端计划格式修复/断电恢复最新：现场智能语音请求 HTTP 200，但两次
`submit_plan.arguments` 均非合法 JSON，本地在计划阶段拒绝，覆盖 IDLE、控制授权 false，
没有执行动作。旧解析只接受裸 JSON 字符串。现增加有边界的兼容解析：对象参数、唯一完整
对象的围栏/文字包装、字符串外结尾逗号、原始控制字符和单层双编码可归一化；缺括号/引号/
值、截断或两个对象仍拒绝。归一化后原唯一计划、字段、工具参数、动态区域、点位和完整性
校验全部保留；首次失败会要求基于原指令从头提交严格 JSON，二次失败继续零调用。新增格式
回归后 9 个隔离 Nose 文件共 170 项通过，Release 构建通过；真实目录云端 plan-only
28/28，均一次响应，平均 1.55 秒/最长 2.13 秒，证据
`validation/voice_region_cloud_20260910_0909_v7.json`，报告已追加到
`docs/VOICE_DYNAMIC_REGION_SYNC_20260910.md`。首次统一重启停机时现场断电；供电恢复后确认
无残留再冷启动，退出 0、583 项 runtime 通过。当前服务 active/MainPID=26730，Qt/AI
PID=34579/34575，地图 READY、WAITING_INITIAL_POSE、覆盖 IDLE，三道 AI 授权 false，
3 条最终速度全零；底盘报告 VCU 遥控/机器人急停，未解除。断电后 Deli USB 音频虽为
ALSA card0，但两个设备节点异常为 root:root 0600，AI 因当前用户打不开而 ASR UNAVAILABLE；
无免密 sudo，未强改权限。需现场拔插 Deli 接收器让 udev 重建设备，确认 arecord 可见后
再重启/切模型；在此之前不能称现场语音回归通过。J6M 未重新部署，未发初值/目标/任务。

2026-09-10 19:12 现场语义二次修正最新：操作员指出“研究院大楼前门区域”就是当前
目录的“研究院大楼前大厅”，且“看周围有没有垃圾，有垃圾就清扫”应直接启动视觉伺服；
另明确“原点”默认地图原点。只读复现确认云端虽已判断前大厅为唯一前部候选，却因门/大厅
字面差异追问；此前还把正确的“地图上面的原点/地图零点”被本地点位证据门拒绝。现通用
规则改为主体+方位优先，入口/门口/门区域与厅/大厅/门厅在唯一候选时兼容；没有为当前地图
加固定别名。同地点同方位多候选增加本地兄弟候选门，防止模型按目录顺序任选。
“查看/搜索周围垃圾，有垃圾即清扫”现映射一次 `start_spot_cleaning`，目标存在性由 FOD
视觉伺服内部处理；“原点/零点/地图上面的原点”均作为 map(0,0,0) 的明确原文依据。
同时修复长计划完整性：未保存的“现在/刚才/说话时的位置”不得被静默跳过，必须空计划
询问其 map 坐标，其余动作仍在 understood_sequence 保留。真实目录 28 种口语 28/28、
内存动态目录 9/9、整段历史 large 转写语义/噪声检查及 167 项隔离单测通过，均为
plan-only，无 ROS 机器人动作。中间 `_v2/_v3/_v4/_v5` 失败证据保留，最终证据为
`validation/voice_region_cloud_20260910_0909_v6.json`、
`validation/voice_dynamic_region_cloud_20260910_0909_v4.json`、
`validation/voice_turn_fix_20260909/cloud_and_noise_visual_v3.json`。
统一 restart 退出 0、583 项 runtime 通过；当前导航服务 active/MainPID=265424，视觉服务
inactive，AI/Qt PID=273119/273132，地图 READY、WAITING_INITIAL_POSE、覆盖/AI IDLE，
三道 AI 授权 false，3 条最终速度全零。J6M 仍为 `20260909_003347`，未重新部署；未发送
初值/目标/任务。后续修正保存于本地 `0909`，未推送，真实区域配置未增加测试区域。

2026-09-10 18:44 智能语音同步/重启最新：已确认操作员此前确实从本目录启动；问题不是
启动目录错误，而是通用动态区域语义修复当时只存在于独立视觉工作树，本目录 `0909`
仍运行旧规划逻辑。现已把语音防伪触发、自然停顿合句/短片段等待、三轮上下文，以及
动态区域目录、近似地名匹配、区域清扫不拆导航、模糊区域禁止充当导航点等完整同步到
`0909`，并完成本机 Release 构建。系统提示词只描述通用匹配规则；“研究院后大厅”等
仅为可替换推理示例，运行时每次读取完整真实目录并在目录变化时清除旧区域上下文。
新增区域必须能由同一规则生效；多候选、方位冲突、未知区域均要求澄清。实验虚拟区域
仅以内存目录注入测试，未写入或污染真实配置。9 个语音/MCP 测试文件逐个隔离运行共
164 项通过；真实云端现有目录 10/10、内存新增目录 6/6 计划预览通过。目标原话
“请你到研究院前门去，在那里清扫一下”只返回一次
`start_coverage_cleaning(研究院大楼前大厅)`。验证均为 plan-only，未执行机器人工具。
统一 restart 带原静态地图退出 0，583 项严格运行检查通过；当前服务
`autolabor-navigation-20260907` active/MainPID=122771，视觉服务 inactive，Qt/AI PID
130311/130307，地图 READY、WAITING_INITIAL_POSE、覆盖 IDLE。语音/解析/控制三门默认
false，3 条最终速度全零；未发送初值/目标。J6M 仍为 `20260909_003347`，本轮未重新
部署。报告 `docs/VOICE_DYNAMIC_REGION_SYNC_20260910.md`，云端证据位于
`validation/voice_region_cloud_20260910_0909.json` 与
`validation/voice_dynamic_region_cloud_20260910_0909.json`。修改保存于本地 `0909`，未推送。

2026-09-09 21:57 语音重启交接：用户现场实际运行的是独立视觉目录
`/home/slam/robot_j6m_ws_vision_0908`，21:39 的 submit_plan 拒绝来自其旧语音节点。
按用户“用新的、全部重启”要求，新语音源码已合入该视觉分支并本机独立构建，保留其
视觉二维坐标/标定及 false/false 运动门。该目录 full-restart 带原地图退出 0，
内外 runtime 通过；服务 autolabor-vision-0908 active/MainPID=2146813、Qt=2154494、
AI=2154507，新 WebRTC/1000ms/medium 已生效。地图 READY，仍需本次真实初值；
语音/解析/控制授权默认关闭，15 秒指令与轮速全零，无活动覆盖。未发送初值/目标。
视觉 J6M current=`20260909_194502`，本目录 J6M current=`20260909_003347` 不变。
本目录未启动、未部署，0909 原语音源码及未提交修改保留；两目录均未提交或推送。
后续现场启停/监测以视觉目录及其 `docs/SMART_VOICE_MERGE_RESTART_20260909.md` 为准，
不要把下方旧完整会话仍运行的记录当现状，也不要同时启动本目录争抢硬件。

2026-09-09 智能语音自然停顿修复（本地候选）：用户要求解决伪指令和长句切碎，已在
0909 工作树加入 WebRTC/平稳噪声/识别置信及重复输出过滤，声学切段1秒，语义等待
1.5/4秒、孤立短片段6秒过期、最近3轮/60秒上下文。Qt显示合并草稿和待补充动作理解。
实际60秒操作员录音数字回放，small/medium/large对此前large整段机器转写差异率为
52.17/18.84/10.14%，非人工真值准确率；small后15秒重复“车”字被拒绝，medium漏句，
不能宣称全部模型识别通过。large保留主要任务顺序；返回说话时位置和条件清扫工具缺口
仍存在，返回空可执行计划并要求补充。真实云端区域补答及支持的多区域→原点顺序通过。
本机Release、143项语音包及Qt的52项契约/11项存储/6项隔离窗口检查均通过；
真实云端与噪声10项检查通过。Qt窗口检查包括合并草稿及待补充序列，未启动实际渲染。
未重启完整栈、未开麦/发目标/改运动设置、未部署J6M；只读current仍20260909_003347。
运行中Qt/AI仍为重启前逻辑，新代码待下一次统一重启生效。未提交/推送，私有配置未改，
ASR venv新增webrtcvad。报告docs/SMART_VOICE_TURN_FIX_20260909.md，原始证据
validation/voice_turn_fix_20260909/；最终文件为small_verified/medium_verified/large_final.json。

2026-09-09 智能语音优化最新：用户要求先保存当前代码到 0909，再优化并测试
small/medium/large。已创建本地分支 `0909`，优化前检查点为 `d36febb`；其后的覆盖导航
公开发布提交 `febc449` 原样保留。Whisper 三个独立冷进程均复现首句 12.68/19.11/24.98 秒
且输出无效连续感叹号，连同模型装载的冷路径为 17.75/29.21/42.20 秒。现改为语音授权后、
开麦前异步加载并静音预热，Qt 的 LOADING 阶段禁用录音/智能监听；撤权仍使结果失效并在
CUDA 调用退出后卸载。文本解码关闭时间戳，VAD 尾静音 800→600 ms，并记录加载、排队和
纯解码耗时。16 条 edge-tts 中文指令合成集（两音色，一组 18 dB 固定噪声）预热后
small/medium/large 平均 0.99/1.89/2.48 秒，CER 4.81/2.88/0.48%，槽位召回
90.74/96.30/98.15%；默认 medium 是当前速度/准确率折中。合成集不是现场准确率。
32 项 worker、29 项授权和 Qt/构建等验证以最终报告为准。未启动导航/Qt、未开麦、未调用
DeepSeek、未部署 J6M、未发送控制。详见 `docs/SMART_VOICE_OPTIMIZATION_20260909.md`，
原始数据 `validation/smart_voice_optimization_20260909/`。

2026-09-09 覆盖导航已公开发布：源项目发布准备提交为 `5ddce5d`，独立公开仓库 main
为 `0350307f9a41e140c3fc5a73a8dc4ec00c00541b`，目标
`https://github.com/cl4mat/autolabor-coverage-navigation`。GitHub 授权账号已核对为
cl4mat，远端哈希与本地发布提交一致，About 简介已更新；HTTPS 全新克隆后几何示例及
26 项测试通过。GitHub Actions `build-and-planner-tests` run 34325629265 成功：干净
x86_64 Noetic 容器构建、单元测试和真实插件合成场景均通过。此前本地 ARM64 干净容器
验证为245项独立单元检查、36项清扫插件及14项首入口场景通过。仓库公开版包含覆盖包、
定制 TEB、底盘接口和所有权桥；不包含整机历史、运行资产、地图、凭据或运动授权。
本次未启动/部署/控制实车，运动设置未改。源项目原 origin 未修改或推送。
完整发布方法见 docs/COVERAGE_OPEN_SOURCE.md；本地证据在
runtime/coverage_publication_20260909/。工作树随后出现两个与本任务无关、未跟踪的 ASR
脚本，未纳入提交，不要清理或误称工作树完全干净。

2026-09-09 覆盖导航开源准备：用户明确要求在源项目补齐架构/测试/提交步骤，再提交到
其新建公开仓库 `https://github.com/cl4mat/autolabor-coverage-navigation.git`。现有 origin
属于 Doribelove，不是用户目标，不向它推送，也不改原远端。独立白名单导出包含覆盖包、
定制 TEB、底盘接口/驱动和所有权桥；不复制原 Git 历史、运行资产、地图、凭据或授权。
源文档和公开模板见 `docs/COVERAGE_OPEN_SOURCE.md`、`distribution/coverage_navigation/`，
出口为 `scripts/export_coverage_navigation.py`。README/简介已说明四轮前轮转向、阿克曼
运动学、自选区域以及区域外需留入场/转弯/换向自由空间；区域不是硬边界。
干净官方 Noetic/Focal ARM64 容器 Release 构建通过；245 项独立单元检查、36 项清扫
插件及 14 项首入口场景通过，原工作区 15 项源码契约也通过。Catkin 原汇总264含嵌套
C++重复，独立 XML 为221，另桥24，勿夸大。验证详见公开模板 docs/VALIDATION.md。
运行算法和实车配置未改、未重启/部署 J6M、未恢复实验；已有5米修复一并纳入源码保存。
目标仓库已确认公开且为空。现有 SSH 身份 Doribelove 被 GitHub 明确拒绝目标写入；
需要 cl4mat 授权后才能远端推送及更新 About，不能声称已公开提交。发布工作与验证记录
位于 `runtime/coverage_publication_20260909/`，本地身份按用户提供的 cl4mat 账号设置，
不改全局 Git/SSH 配置。原主门 true/FOD=false/1.60 与授权保持，后续不得自动恢复运动。

2026-09-09 11:22 实验记录最新：操作员已自行启动本候选并设置本次初值，实测 LOCALIZED，
J6M current 为 `20260909_003347`，运行参数清扫前方 5.0 m / 入口恢复 4.0 m。
本组 3 轮均最终 COMPLETED；第三轮先因 VCU robot emergency 暂停 17.5 秒，再因
转场执行重试耗尽暂停 21.5 秒，恢复后完成，不能算无人干预通过。清扫实测均速
1.08/0.98/0.91 m/s；71 次 Hybrid 请求得到 69 条有效路径、2 次目标外廓无效，无搜索
超时。第三轮换向接续缓存失效 25 次，规划有路径但仍出现执行无进展；不是 CPU 持续满载。
用户宣布实验结束后，被动监测于 11:22:19 正常退出，未发控制指令、修改运动设置或停机。
本次未改业务代码/构建/部署/提交；主门 true/FOD=false/1.60 上限及原授权保留。
不要将下方凌晨停止或等待初值当成现状，也不要自动恢复旧实验。
报告 `docs/HYBRID_CPU_EXPERIMENT_20260909.md`，原始数据
`validation/coverage_live_20260909/20260909_110209/`，含整板/进程/线程 CPU、动作、速度与 perf。

2026-09-09 00:36 部署最新：用户要求先保存当前代码到 0908，再把清扫碰撞检查限制为
可配置的前方 5 米并部署 J6M。修改前代码/交接已提交到本地分支 `0908`，提交
`9c69b7d`，未推送；本轮修复仍留在该分支工作树，未额外提交。
新增 coverage_global_planner.yaml 的 sweep_collision_check_horizon=5.0，ENFORCED_SWEEP
每次从实时位置沿剩余清扫线检查前方窗口，完整外廓/TEB/未知空间/边界检查保留；
覆盖入口准入、状态机、Hybrid 3 m 和 entry_recovery 4 m 均未改。本机 Release、193 项
覆盖 Python 测试、36 项真实插件隔离检查及静态检查通过。J6M 原生发布
`20260909_003347`，板端静态检查、安装/编译源哈希、安装库及 launch 展开 5.0 m 均通过。
原任务停机前为 PAUSED，已保存快照并统一停净。MID360 固定网口无载波、底盘与雷达
反馈此前已长时间中断；当前完整导航 inactive/MainPID=0，未恢复启动/旧任务。
无当前实机运行态/移动验收结论，需现场链路恢复后完整启动并设置新的真实初值。
主门 true/FOD=false/1.60 上限和授权保留，私有配置未改。被动记录器已随原服务结束。
详见 docs/SWEEP_COLLISION_HORIZON_20260909.md、validation/sweep_horizon_fix_latest.txt。

2026-09-08 22:14 当前清扫停滞只读审计：本次操作员初值已设置，LOCALIZED，不能再
沿用下方 WAITING_INITIAL_POSE。当前计划 a902200d614d4f0cbe9dd285f59208fc，第 8/8 段，
79.9652%，active=true、paused=false；SWEEPING 和短暂 WAITING_OBSTACLE 循环，指令及
双轮零速，底盘/避障就绪。22:06:06 起全局规划器反复拒绝强制清扫线，非入口/4 米门。
实时地图复算命中前方约 15.126 m 的单个致命栅格（16.75,-26.95），原静态图为空地。
整条剩余约 19.1 m 清扫线碰撞检查导致入口立即拒绝；实际清除射线范围 11 m，未确定
远端障碍记录是否残留或现场实物。已截图、保存地图/日志，未清图、改控制或暂停/取消。
运行版仍 20260908_214006，主门 true/FOD=false/1.60 上限保留。证据与运行边界见
docs/SWEEP_BLOCKED_AUDIT_20260908.md、validation/sweep_blocked_audit_latest.txt。

2026-09-08 21:45 部署启动最新：用户要求“现在部署，进行实验”。已通过视觉会话的
stop.request 正常关闭 Qt/相机与其私有 BPU 服务（返回 0），主导航此前已统一停净。
本地已验证代码通过候选部署入口原生发布为 J6M `20260908_214006`，板端静态检查
通过，管理器与 coverage.yaml 两端 SHA256 一致。带原静态地图完整启动返回 0，
严格 runtime 通过；active/MainPID=1092384，管理器 PID=824590，Qt 地图 READY。
运行参数 4.00 m、入口 0.40 m/25°、规划 0.30 m/20° 已核对，新触发代码生效。
当前为本次冷启动的 WAITING_INITIAL_POSE，需要操作员按真实位置设置新初值。
覆盖 IDLE/无活动批次，急停反馈已解除、底盘/避障就绪；30 秒最终指令 1483 条、
双轮各 1507 条全部零速。启动期记录 2 次 VCU ControlTimeout 和急停状态，后续
30 秒未复现控制超时。完整归档另见 21:43:38–43 的双轮非零反馈（最高
0.458/0.433 m/s），以及零星 ±0.01237 m/s 反馈，最后一次为 21:46:15；全会话
ROS 最终指令均零，来源尚未确认，不能把上述 30 秒静止窗口推广到整个会话。
当前双轮已归零。等待初值期间 move_base/FOD 的 TF 告警仍存在，不能宣称
定位或实车恢复已验收。当前沿用操作员手动实验，ABC 自动控制 STOP 保留。
被动记录器恢复于 `validation/coverage_live_20260908/20260908_214201/`，
部署证据见 `validation/entry_recovery_deploy_latest.txt`。主门 true、FOD=false、
1.60 上限与配置/授权均保留；助手未发送初值、运动目标或非零速度。未提交或推送。

2026-09-08 入口恢复触发修复（本地未发布）：用户要求恢复 4 米上限，只收紧触发，
且不得影响入口合格后进入清扫。配置和默认值现为 4.00 m；不再因普通转场失败距入口
2 m 内或单独清扫前入口超差就启用特殊恢复。必须有最终接近段可信连续记录、有向进度
达到 85%、沿实际行进方向越过原入口平面至少 0.02 m、当前在原 0.40 m/25° 容差之外
且距入口不超过 2 m，才应用原恢复代价及 4 m 门。每次规划前重判并清除过期标记；
入口合格仍直接进入原停车/清扫交接，不新增 85% 或越界门槛。原代价数值与运动保护不变。
2 项旧代码误触发断言复现，修复后 149 项状态机、193 项覆盖包 Python 测试通过；
本机 Release 构建和静态检查通过。J6M current 只读确认仍为
`20260908_192331`（10 m 旧触发版），未部署、未启动导航、未发运动目标。
导航 inactive/MainPID=0，独立视觉会话保留。未提交或推送；详见
`docs/ENTRY_RECOVERY_TRIGGER_FIX_20260908.md` 和 `validation/entry_recovery_trigger_latest.txt`。

2026-09-08 21:08 最新：用户要求只打开本版 Qt 视觉识别。已通过统一入口完整停止导航
（退出 0），被动覆盖记录器随原服务退出；随后在本机独立 ROS master
`http://127.0.0.1:11571` 启动 ZED QUALITY RGB-D、FCOS 几何筛选和 Qt BPU 预览。
导航、定位、雷达、覆盖、底盘及 CenterPoint ROS 节点均未启动；ZED 跟踪及 RViz 关闭。
25 秒观察收到 373 帧有效且已标定结果，约 14.91 Hz，未收到速度、目标或初值消息。
当前场景保留不确定框，不能据此宣称垃圾精度验收通过。窗口持续运行，无演示截止；
关闭 Qt 会结束本次相机、识别及私有 BPU 会话。主门 true、FOD=false、1.60 m/s 和
授权标记保留，J6M current 仍 `20260908_192331`。未构建、部署或提交；原有未提交
导航修改保留。完整导航会话已结束，后续重启导航需新会话真实初值，不能沿用旧定位。
运行记录和停止方式见 `docs/VISION_ONLY_SESSION_20260908.md`。

2026-09-08 后续只读转场审计：用户手动任务 `8c3ae04f72e84a73b6277e6b0e0aea3b`
在 19:50–19:51 的第 2→3 条线转场失败，停于第 5/8 段、56.6495%。11 次 Hybrid
请求均在 6×6 m 窗口成功，最长约 0.757 s；包含获准执行的 6.90 m 全倒车大圆弧，
取消前定位反馈累计约 1.779 m，横偏 0.357 m 后重规划，再因低进展耗尽重试暂停。
两搜索线程只区分启发权重，并无前/倒车起步分工；日志未保存选中 lane。
发现短 cusp 的 SUCCEEDED 终态直接返回、未复核有向进度的代码缺口，与 0.30 m
小段仅挪约 0.07–0.08 m 就报告到达、随后无法接回后缀的现象吻合，尚未修复。
当前恢复代价按 F/R 0.20/0.80 m/s 计算、实际倒车上限 0.30；候选选择优先换向次数。
详细时间线、证据及边界见 `docs/TRANSITION_REVERSE_PATH_AUDIT_20260908.md`。
本次仅只读查询和保存分析，未改控制/参数或恢复任务，自动实验 STOP 仍保留。

2026-09-08 19:40 最新：用户明确要求“解除暂停，恢复手动实验”。操作员已提交本次
真实初值，实时 LOCALIZED（overlap=0.9992、RMSE=0.1645、inliers=4854），无需再次
设置本会话初值。底盘/避障就绪，无活动任务、无保留目标；助手已调用导航暂停服务
false，返回 `navigation resumed without a retained target`，导航与覆盖 paused 均
确认 false。恢复瞬间覆盖 IDLE、无批次，指令/双轮零速；此后由操作员手动实验。
入口恢复总长运行值仍 10.0 m，release `20260908_192331` 不变。后台被动记录器继续
运行，自动多轮实验 STOP 标记保留；不据此恢复助手自动巡扫或重发旧目标。
证据 `validation/manual_experiment_resume_20260908_194029.json`。本次仅解除暂停并记录，
未重启、构建、部署或发送初值/运动目标，保留既有运动授权和速度上限。

2026-09-08 19:30 最新：按用户要求，将入口恢复路径总长从 4.00 m 调整为 10.00 m，
配置及两处默认值已同步；入口容差、恢复分类、速度和其他保护不变。140 项状态机
测试（含 10 m 接受/超限拒绝）通过，本机 Release 构建、静态检查、J6M 原生发布与
板端静态检查通过。current 已更新为 `20260908_192331`，配置和管理器两端 SHA256
一致。统一停机、带原静态地图冷启动均退出 0，runtime 检查通过；候选 active，
MainPID=523119，覆盖管理器 J6M PID=781319，Qt 地图 READY。
19:29:36–19:30:36 静止观察中，实际参数前后均 10.0；3022 条最终指令、双轮各
3067 条反馈全零，覆盖 IDLE/无活动批次、导航暂停 true，传感器持续更新。
当前新会话为 WAITING_INITIAL_POSE；须由操作员在下次实验前重新设置真实初值，
不能沿用 17:33:51 的旧会话初值。初值前 move_base 尚无 status 发布、记录 12 条
等待 map→base_link TF 的告警属该阶段预期，不能宣称已完成定位/导航移动稳定性验收。
用户要求读取和运行稳定后告知再开始实验，本轮仍保持暂停，ABC STOP 标记保留，
没有发送初值、目标或非零速度。主门 true、FOD=false、1.60 m/s 和授权标记保留。
被动记录器已恢复至 `validation/coverage_live_20260908/20260908_192709/`。
变更、验证及当前生效状态见 `docs/ENTRY_RECOVERY_LIMIT_20260908.md`，证据路径见
`validation/entry_recovery_10m_latest.txt`。未提交或推送。

2026-09-08 18:13 最新：用户因下课高峰期明确暂停实验，并要求在切换网络前保存
实验进度与终端数据。18:09 已设导航暂停 true，A 批次此前已按精确 ID 取消；
18:13 连续 15 秒确认无活动目标，730 条最终指令、双轮各 755 条反馈全零。
保留主门 true、FOD=false、1.60 m/s 和授权标记，完整栈及被动记录器留存运行。
`validation/abc_trials_20260908_1752/STOP` 保留；未经用户明确继续，不得自动恢复
实验、发送目标或位置调整。切换网络后的运行/连接状态须重新实测。
第一轮 A 在 27.5172% 的第 3/8 段转场复现恢复路径 4.20 m 超过 4.00 m 门限，
已取消，B/C 与提速对比尚未执行，辅助 reposition.py 未运行，尚无最佳参数结论。
本次 17:27 启动会话已有操作员真实初值，不重复索取；如后来完整冷启动则须新初值。
交接见 `docs/EXPERIMENT_HANDOFF_20260908_PAUSED.md`，固定归档位置见
`validation/network_switch_20260908_latest.txt`。本轮未构建、部署或提交。

2026-09-08 17:51 多区域实车实验授权：用户明确将底盘控制权限交给助手，授权对 Qt
新保存的 A、B、C 反复进行多轮实验，包括轮流清扫、约半程取消后转另一区域；突发
行人情况由现场操作员告知。随后追加重点关注转场停滞及覆盖管理器拒绝运动，并允许
卡住时经受保护控制入口调整至下一条轨迹附近，研究入口恢复路径总长上限。
沿用已确认的封闭测试区/实体急停/1.60 m/s 最终上限，主门 true、FOD=false 保留。
助手可在本次授权范围内发起任务，不得再把下方历史“助手未发目标”当作禁止本轮执行。
本轮先以清扫/转场前后向不超过 0.30 m/s 验证；保留定位、避障、运动门、唯一最终
速度发布者和完整任务所有权，不允许绕过保护直接发布第二路最终 cmd_vel。
每次人工式位置调整、任务取消和暂停必须记录，不能当作无人干预成功。实验定义及
控制证据位于 `validation/abc_trials_20260908_1752/`，原被动记录器继续运行。

2026-09-08 16:56 主运动授权最新：用户明确要求“开启主运动授权”。沿用已记录的封闭净空
测试区、人员远离、实体急停可用与 1.60 m/s 上限确认；当前 CAN 固定端口已确认。
预检 LOCALIZED、底盘及避障就绪、无活动目标、指令及双轮零速。统一停机退出 0 后，
通过本候选授权入口创建 `runtime/motion_authorized.ok`，私有配置仅将 MOTION_ENABLED
改为 true；FOD=false、导航 0.80/倒车 0.30 和最终 1.60 上限不变。
带原静态地图完整冷启动退出 0，严格 runtime 通过；active/MainPID=980973、Qt PID=988649，
地图 READY。实际网关参数 motion_enabled:=true，看门狗 true/1.60/0.25 秒，J6M 配置
同步为主门 true/FOD=false；current 仍 `20260907_214928`，本次未构建或重新部署。
16:56 快照为新会话 WAITING_INITIAL_POSE、覆盖 IDLE/无计划；需操作员按真实位置重新
设置初值，不能复用上次。新监测会话累计 5446 条最终指令、双轮各 5544 条全零。
助手未发送初值、目标或非零速度，不自动恢复旧任务。不要在收尾撤销此次授权或按下方
历史记录恢复 false。后台监测已恢复于 `validation/coverage_live_20260908/20260908_165422/`。
配置备份、运行证据和范围见 `MOTION_AUTHORIZATION_20260908.md`。

2026-09-08 16:36 最新：用户要求停止正在运行的 GNSS 版、保存其进度，再启动本目录。
GNSS 版已通过其统一入口完整停机（退出 0），源码与文档保存到其原本地分支，提交
`9603e23`，工作树干净、未推送；私有配置和运行资产仍留本地。
本目录仍为 `feature/litter-perception-optimization-20260908`，统一入口带
`map_20260829_221335_gimp_cleaned` 启动退出 0，严格 runtime 检查通过；候选
active/MainPID=880803，Qt 地图 READY（1311x670、0.10 m）。J6M 与 MID360 网络正常，
FCOS 筛选状态及 CenterPoint 观察状态持续更新。本次未重新构建或部署，导航 J6M current
仍 `20260907_214928`，GNSS 独立 current 仍 `20260908_143728`。
16:35:43 起约 20 秒观测：990 条最终指令、左右轮 809/781 条反馈全零，覆盖 inactive；
定位全部 WAITING_INITIAL_POSE，需操作员在本次 Qt 按真实位置设置初值，不能复用旧值。
两运动门保持 false，未创建授权、发送初值、目标或非零速度；程序启动不等于实车移动验收。
证据 `validation/start_navigation_20260908_163602_runtime.json`。此条取代下方停止交付状态。

2026-09-08 垃圾误框优化候选：当前本地分支为
`feature/litter-perception-optimization-20260908`，从导航验收提交 `99e8ac9` 分出。
本轮用户要求本地保存、次日亲自启动验收；不启动实车、不部署 J6M、不改变运动配置。
AGX FCOS RGB-D 后处理新增类别无关的地面/前景几何及连续确认，Qt 默认显示筛选结果，
保留橙色不确定框，并可切换全部原始框。原始 `/perception/fcos/objects` 保留，新增候选与
逐框审计 topic 全部仅观察；没有迁移垃圾专用权重或接入自动清扫。
J6M 只读确认 current 仍 `20260907_214928`；原导航分支及其他工作树保留原提交。
当前目录会留在新分支，本机产物供此次验收使用；停止状态切回旧分支后必须重新构建，
不能让旧源码与新 Qt 产物混用。原因、验证与启动/对照步骤见
`docs/LITTER_PERCEPTION_OPTIMIZATION_20260908.md`。下方为此前导航交接历史。

本日收工保存：用户要求将当前全部进度保存到 Git 分支，供明日验收；当前分支为
`feature/navigation-optimization-20260907`。验收流程与证据归档说明见
`docs/ACCEPTANCE_HANDOFF_20260908.md`。此保存操作不启动程序、不改变现场运动设置。

2026-09-07 22:46 仿真验证最新：按用户要求，首入口修复已完成隔离 ROS 场景验证，15 项
集成检查通过。独立网络/PID namespace 只含 lo；运行真实 move_base、Navfn、TEB 插件、
覆盖管理器和导航所有权桥，用合成地图/TF/状态输入。旧 release 20260907_212100 的
管理器生成 307 点后约 7 ms 被空消息覆盖，晚订阅者为 0 点；修复版连续 3 次保留
307 点，绕墙场景保留 462 点/23.272 m。取消、换图、新批次替换、规划中取消和 Navfn
封闭无路检查通过，0 个 move_base/Hybrid goal、没有 cmd_vel 消息。仿真进程已全部结束。
本轮新增仿真工具和原因报告，未改业务代码、未重新部署或启动实车。候选 current 仍为
20260907_214928；用户主动断开底盘及雷达供电，实车服务继续停止。此结果不等于 Qt/Ogre
实际渲染、TEB 行驶、J6M Hybrid CPU 或实车验收。详见 `FIRST_ENTRY_SIMULATION_20260907.md`。

本轮随后现场确认：用户已主动断开小车底盘供电，雷达因此无电，要求先汇报工作进度。
这解释了下述 MID360 无载波，不能继续误报为待查网线故障。停止恢复启动和待连接的监控器；
当前程序保持停止，修复已部署但实机回归未完成。待用户恢复供电并继续实验后再统一启动。

2026-09-07 22:08 最新：用户覆盖实验复现首入口预览仍为空。批次因主运动门 false
失败时，`_record_batch_region_result` 先清掉路径和 plan_map_digest，导致后续 finalizer
保留逻辑失效，上一轮修复未闭环。本轮已在两级清理中统一保留判定，预览独立记录地图摘要
和批次令牌；取消、跳过、执行过、地图变化和旧批次均不保留。先复现再修复的回归通过；
206 项针对性测试、AGX Release 构建、静态检查、J6M 原生部署及板端静态检查通过。
候选 current=`20260907_214928`，板端管理器与本地 SHA256 一致。
完整双机栈已统一停净，但带原静态地图恢复启动退出 1：MID360 网卡
`50:54:7B:E3:C9:10`（本次 eth0）无物理载波，192.168.1.112 不通；J6M 双向通信正常。
当前原版/候选均 inactive、MainPID=0，J6M 11311 无监听、候选 PID 记录空。已请求现场
检查雷达供电与网线；不要称程序已恢复。链路恢复后完整冷启动，再由操作员给真实新初值。
两个运动门仍 false，助手未发送初值、目标或非零速度。21:39:44–21:44:37 实验快照最终
命令 14149 条、左右轮 14625/14622 条全零；没有 Hybrid action，CPU 仅为空闲基线。
另捕获 7 条 VCU ControlTimeout 日志，后续 60 秒未复现，根因未定，不能称无故障。
详见 `COVERAGE_CPU_EXPERIMENT_20260907.md` 与 `validation/coverage_live_20260907/`。

2026-09-07 21:27 首线入场预览修复最新：用户要求修复“当前位置到第一条清扫线入口没有
规划路径”。根因是旧实现只在运动安全预检通过后提交首个 move_base 目标，主运动门为 false
时预检提前拒绝，因此 Navfn 从未运行，Qt 也没有独立的真实入场路径 topic。候选现增加
`/coverage/first_entry_path`：覆盖几何和实时位姿确定后，通过插件只读
`/move_base/CoverageGlobalPlanner/make_navfn_plan` 查询 Navfn，校验 map 坐标及首尾位置，
Qt 以深蓝色常显；执行前失败时保留供检查，新任务、地图变化或取消会清空。该服务不发送
action 目标。插件内所有 Navfn 调用现由同一互斥锁串行，避免预览、move_base 规划和 Hybrid
拓扑查询并发访问同一 NavfnROS 实例。AGX Release 构建、204 项针对性测试、283 项汇总检查、
J6M ARM64 原生部署和板端静态健康均通过；J6M 候选 release 为 `20260907_212100`。统一入口
带原静态地图冷启动返回 0，服务 active/MainPID=2724377，Qt PID=2731384，地图 READY。
当前新会话为 WAITING_INITIAL_POSE；move_base 插件及其预览服务会在操作员按真实位置设置
初值后初始化，尚缺同一区域的真实首入口路径回归。不要复用旧初值。`MOTION_ENABLED=false`、
`FOD_MOTION_ENABLED=false`、候选授权标记不存在，10 条最终速度全零；助手未发送初值、目标
或非零速度。详见 `COVERAGE_CPU_EXPERIMENT_20260907.md`。

2026-09-07 Qt 闪退修复最新：用户已授权修复，已在本候选完成实现、构建与完整冷启动。
实际 GUI 回归先复现全局 AsyncSpinner 抢走 RViz GUI 队列回调；已改私有业务队列。
真实 Ogre 压测又复现删除后第二次建窗的纹理错误/SIGSEGV，现 ROS master 恢复仅重置
显示缓存并重新注册本进程 ROS 端点，保留同一原生窗口与 manager。ZED 项目启动入口
同时关闭位置跟踪和会隐式开启跟踪的深度稳定，QUALITY RGB-D 保留。283 项汇总测试、
1000 次实际页签切换与 5 次真实渲染重连均通过，未改故障级联停机或运动安全策略。
共享 robot_bringup 已随 J6M 原生 release `20260907_191248` 发布，板端静态健康通过。
统一入口带原静态地图启动与严格 runtime 均返回 0；候选 active/MainPID=2161491，
Qt PID=2169173、地图 READY（1311x670、0.10 m），ZED PID=2167341、诊断跟踪 INACTIVE。
19:16:13 起 60 秒观测 faults=[]，2907 条最终速度及左右轮反馈全零。当前新会话仍
WAITING_INITIAL_POSE，需要操作员按真实位置设置初值；不要复用旧初值。两个运动门
保持 false，候选授权标记仍不存在，助手未发真实初值、目标或非零速度。麦克风仍未枚举。
原工作区未改，CenterPoint 仍只观察。详见 `QT_CRASH_FIX_20260907.md`，短时通过不等于
小时级老化、ToDesk 热重启/显示热插拔或实车移动验收。

2026-09-07 17:48 最新：用户准备继续实验时 Qt 消失。只读日志确认不是 Qt/Ogre 自身断言：
ZED 先报告位置跟踪发散并重置里程计，随后必需节点 `/zed2/zed_node` 以 `-11`（SIGSEGV）退出；
其 roslaunch 关闭了视觉链，UI 伴随进程也随生命周期退出。已按统一入口带原静态地图做完整冷重启，
不是单独拉起 Qt；命令退出码 0，服务 active/MainPID=1667551，195 项检查和严格 runtime 检查
均通过，Qt 地图 `READY`（1311x670、0.10 m），ZED、正式检测、双 BPU 感知、导航与覆盖节点
均在线。新会话当前 `WAITING_INITIAL_POSE`，须由操作员按真实位置重新设置初值；不要复用旧会话
初值。`MOTION_ENABLED=false`、`FOD_MOTION_ENABLED=false`，watchdog inactive，10 条最终
`/cmd_vel` 样本全零。助手未发送初值、目标或非零速度。J6M `current` 仍为
`20260907_170157`，本轮没有重新构建或部署。

2026-09-07 17:10 最新：用户要求打开本目录的静态地图完整功能 Qt；MID360 物理链路恢复后，
已修复独立候选遗漏的 BPU 私有路径迁移和 `live_protocol.py` 运行依赖。旧候选遗留 BPU worker
仅按其 `service_manager.py` 的 PID/启动身份记录精确停止；旧工作区源码未修改。AGX 全量 Release
构建、BPU 包 21 项测试、195 项静态检查、J6M 原生部署及板端静态健康检查通过；J6M `current`
现为 release `20260907_170157`。候选统一入口带
`map_20260829_221335_gimp_cleaned` 启动返回 0，服务 active/MainPID=1443773，Qt 地图
`READY`（1311x670、0.10 m），严格 runtime 健康检查通过；FCOS BPU 结果约 7--10 Hz、
CenterPoint 约 5 Hz。当前是本次冷启动的 `WAITING_INITIAL_POSE`，必须由操作员按车辆真实位置
设置初值；不要复用历史初值。`MOTION_ENABLED=false`、`FOD_MOTION_ENABLED=false`、候选运动
授权标记仍不存在，10 条最终 `/cmd_vel` 样本全零；助手未发送初值、目标或非零速度。当前未枚举
实体麦克风，Qt/AI 节点在线但语音采集不可用。CenterPoint 仍仅候选观察，不接入导航控制。

2026-09-07 最新：本目录是独立工作树
`/home/slam/robot_j6m_ws_navigation_20260907`，分支
`feature/navigation-optimization-20260907`。开工时完整保留了
`robot_j6m_ws_optimized_20260905` 的 tracked/untracked 源码修改，并补齐当前地图、私有配置、
ASR 与 CLIP 运行资产；原工作区未清理、重置或修改。覆盖转场 Hybrid A* 已改为可取消 action，
每层两路并行搜索，按 6x6 m、10x10 m、全图逐级回退，滚动/恢复总预算 5 s。AGX 全量 Release
编译通过；覆盖包 187 项测试、全局汇总 195 项和隔离生命周期 10 项均零失败。J6M 已在独立
ARM64 chroot 原生构建并部署 release `20260907_141331`，其独立 `current` 已通过静态健康检查。
部署后原/新导航均停止，11311 无监听，`MOTION_ENABLED=false`、`FOD_MOTION_ENABLED=false`，
没有复制运动授权标记、发送目标、发送非零速度或做实车移动测试。详见
`NAVIGATION_OPTIMIZATION_20260907.md`；一键启停以 README 顶部命令为准。

2026-09-06 19:36 最新：已修复已知地图 localizer 在人工初值附近被 5 m 粗 ICP 拉入
重复走廊错误极值并持续 LOST 的问题；改为 0.50 m 局部精 ICP 优先，原粗到精路径仅作
质量门失败后的回退。J6M current 已部署为 20260906_190656，实机进程和运行参数归属已
核对。操作员在冷启动后重新设置真实初值，当前 LOCALIZED、Qt 地图 READY，无活动目标，
最终指令与双轮反馈为零。固定同一人工初值完成 5 个完整组、每组 12 次重定位，5/5 组
均满足两轴绝对偏差 P75 <= 0.05 m。按追加试验前固定的最差轴最小规则，第4组最佳：
X/Y P75=0.01442/0.02330 m；全部60次合并 P75=0.01289/0.03592 m。该结果只证明
重复定位一致性，没有外部真值，不能写成绝对地图坐标精度。操作员说明此前压力试验期间
有人走动，那两组数据不纳入完整组；另有一次旧24秒窗口内中止尝试，也未拼接进完整组。
详见 `FAST_LIO_REPEATABILITY_20260906.md`。本轮未发送导航目标或非零速度，主运动门
true、FOD=false 和既有授权标记保持不变。

2026-09-06 17:12 Qt崩溃后重启最新：用户要求再次实验，已通过候选统一入口带原静态地图
完整冷重启，退出码0、997项检查及runtime健康检查通过。候选服务active/MainPID=1261687，
Qt PID=1266732且未再出现Ogre断言，原版inactive；J6M进程仍来自release
20260906_152402。地图READY，操作员已在新会话设置真实初值，最新LOCALIZED，overlap=0.9846、
RMSE=0.1786、inliers=4339；不要再要求重复本次初值。主运动true、FOD=false、1.60上限和
已有授权保留。本轮用户在Qt先发起28.00 m²批次任务；因其与初值提交时序交叠，助手只调用
`/coverage/set_paused=true`作短暂停车核对，确认指令/轮速归零。随后操作员从Qt新建了不同的
61.49 m²计划9f6498e3，非旧任务倒退；17:12实测其TRANSITING、5/6段、进度66.21%，车辆
正在执行，不能沿用此前零速状态。助手未发送初值、目标或非零速度。本轮没有改业务代码、
构建或部署；第二条CenterPoint仍为候选观察，未接入正式导航控制。详见
FULL_PROJECT_START_20260906.md 的17:12追加记录。

2026-09-06 16:34 完整重启最新：用户暂停第二条链路开发后，单独要求重启整个程序。
已通过候选统一 restart 入口带原静态地图完整冷重启，退出码0、运行态检查通过。
候选 active/MainPID=1131832，Qt PID=1136854，原版 inactive；原生双模型 BPU 新PID=46646，
J6M current 仍20260906_152402。现场已在新会话设置初值，最新 LOCALIZED，overlap=0.9998、
RMSE=0.1704、inliers=4462；不要再次要求本次已完成的初值。地图READY，20秒最终指令与两轮
反馈全零、faults=[]、末次无导航目标。主运动true、FOD=false、1.60上限和已有授权保留。
本次场景FCOS RGB-D约13.10 Hz、CenterPoint约4.99 Hz；这不是此前多框瓶颈的修复或性能保证。
第二条导航接入及Qt三维框开发仍暂停，CenterPoint仍候选观察、检测为0框。本轮未改业务代码、
构建或部署，未由助手发送初值/目标/非零速度。详见 FULL_PROJECT_START_20260906.md 的重启追加记录。

2026-09-06 15:53 完整启动最新：用户明确要求启动完整项目，并确认MID360已接线供电；
链路已恢复，经候选统一入口带原静态地图启动成功。候选服务active，MainPID=919104，
Qt PID=927465，原版inactive，J6M current仍20260906_152402。地图READY，当前本次冷启动
仍WAITING_INITIAL_POSE，未由助手发送初值/目标/非零速度。20秒最终指令/轮速全零，faults=[]。
主运动运行态true、上限1.60、FOD=false，已有授权保留。FCOS RGB-D、CenterPoint和原正式后端
均运行，J6M原生BPU服务PID=10033；真实MID360去畸变点云约10 Hz、CenterPoint约5 Hz。
全栈多框场景下FCOS有效深度输出只有约1.7–2.4 Hz，源帧龄接近0.35 s导致大量过期丢弃；
不能用下方独立测试10–14 Hz代表当前完整项目性能。CenterPoint仍仅候选观察，精度未验收。
本次只启动与被动检查，没有改业务代码/配置、构建或部署。项目保持运行，详见
FULL_PROJECT_START_20260906.md。本条取代下方“网口未接/全部停止”的历史交付状态。

2026-09-06 双感知接入最新：用户已确认 ZED 左目相对 MID360 前0.42 m、右0.14 m、等高，
光轴水平向下41°，无偏航/侧倾。先完成 GitHub 修改前备份分支和标签（e0cbf76），
本地 runtime/backups/before_dual_perception_20260906 保留独立解压副本、归档和 bundle；
2531个文件校验一致。已部署新 J6M ROS current=20260906_152402，新增 FCOS RGB-D 与
CenterPoint ROS 包、独立双模型常驻服务。候选 BPU_PERCEPTION_ENABLED=true，随下次正常
UI 启动伴随链路。主运动 true、FOD=false、1.60上限和已有授权标记保留。
真实 QUALITY 深度+Qt+合成 CenterPoint 并行实测 FCOS约10–14 Hz；997项测试通过。
MID360现场网口无载波、不可达，真实点云验收未完成，不能把263帧合成ROS数据写成MID360实测。
CenterPoint仅候选观察；发现旧TEB自定义障碍物保留旧帧及TF失败单位变换问题，本版明确拒绝
开启学习型障碍物控制入口。COCO/nuScenes不是五材质垃圾模型，正式后端未替换，未验收GPU节省。
本轮没有启动完整导航、发送初值/目标/速度；最后相机、Qt、远端测试节点和私有BPU服务均退出，
原版/候选导航服务均inactive，无当前运行态LOCALIZED/watchdog结论。详见 DUAL_PERCEPTION_20260906.md。
此记录优先于下方历史存活状态，不自动重启导航，也不删除或重新索取已完成的历史主运动授权。

2026-09-06 12:39 最新：本轮 FCOS 性能优化已部署到 J6M 私有实验，候选 Qt 已构建。
旧桥 0.5 Hz 改为默认上限 30 Hz、模型常驻；独立 15 Hz 相机实测 14.90 Hz，
30 Hz RGB 相机实测 28.22 Hz、结果源帧年龄均值 84.57 ms。正常导航相机仍为 15 Hz，
不能把独立 depth=NONE 测试写成完整导航性能验收。详见 `FCOS_PERFORMANCE_20260906.md`。
本轮进入时两导航服务已 inactive、J6M 刚重启，旧 BPU 已停止；仅启动隔离相机/Qt 测试，
现测试全部正常退出，两导航服务仍 inactive，BPU ratio=0。没有启动导航、发送初值/目标/
非零速度或恢复旧目标。保留主运动配置 true、FOD=false、1.60 上限和既有授权标记；
没有当前运行态 watchdog/LOCALIZED 结论。候选 J6M current 仍为 20260905_160700。
J6M 墙钟重启后回到 2025-06-26，后续完整启动使用现有同步流程；本轮测量不依赖板端墙钟。
下方“主链/BPU 存活”是此前会话历史，不能据其自动恢复导航或要求重复本次未启动会话的初值。

本轮后续最新：已收到现场初始位姿，实测 LOCALIZED（overlap=0.9994、RMSE=0.1732、
inliers=5156）。主运动门仍 true、BPU tmux 存活；不要再要求重复设置本次已经完成的初值。
助手未发送任何目标或非零速度；实际移动导航/高速制动仍未由助手验收。

2026-09-06 00:13 最新：主运动授权已完成统一冷重启并实际生效，watchdog motion_enabled=true、
最终线速上限 1.60、超时 0.25 秒，FOD=false。完整 runtime 返回 0，原版 inactive。
候选服务 MainPID=577140，Qt PID=584969，BPU 持续会话 qt_20260906_001122_676676。
10 秒检查最终指令/轮速全零、无保留目标；现为 WAITING_INITIAL_POSE，必须由操作员在
新 Qt 重新标真实位姿。没有自动发目标/恢复旧目标/助手实车移动测试。用户要求删除安全逻辑，
未删除任何避障、定位门、急停或失联保护；已落实其主运动授权诉求。
现场封闭净空/人员清场/实体急停可用已由用户明确确认，不再重复此前不等价的遥控状态位要求。
当前交付状态不再是 false，后续不得把下方历史静止阶段误当现状。详见
`MOTION_AUTHORIZATION_20260906.md`，该记录包含实测证据、诊断探针误报修正及未验证场景。

2026-09-06 00:04：用户再次明确要求开启主运动授权，并已回复确认测试区封闭净空、
人员远离车辆且实体急停有效；这是 README“安全启用运动”中的封闭测试区路径，
没有声称车辆架空。此前仅凭 gamepad_emergency=false 判断遥控停车不可用不成立，
不要再让用户重复该不等价按键检查。只开启主运动门，不开启 FOD、不恢复旧目标、不发非零速度。
00:02:33 起 12 秒预检定位全部 LOCALIZED，最终指令和两轮全零、归属唯一、faults=[]。
已通过现有授权入口生成候选专属 motion_authorized.ok，config 主门改 true、最终线速
上限由 1.70 收紧至用户上限 1.60；NAV 默认 0.80 和 FOD=false 未改。正在统一带原地图
冷重启使参数生效；本条只记录配置和授权，不能据此声称运行态已经 true 或实车导航已验收。
原始配置备份 validation/motion_enable_config_before_20260906.env。保留全部安全保护。

23:45 最新：用户已明确确认在测试区、遥控器停车，要求最高 1.6 m/s；不要再将
测试区域、速度要求和停车方式写成未提供。尚未取得遥控停车键触发的实测反馈，
也没有完成当前现场的制动/失联停车验证或架空条件确认，不能虚构授权标记。
本轮只在验证 candidate master 归属后取消活动 move_base 目标并设置 navigation_pause=true，
服务返回 `move_base goal canceled and retained`（保留的目标不可未经确认恢复执行）。
没有创建 motion_authorized.ok、没有修改速度配置、没有删除看门狗、没有改动两个 false
运动门、没有发送非零速度或目标。23:43:55 起 25 秒只读检查：500 次定位均 LOCALIZED，
MID360/IMU/LD19/融合 scan/ZED 持续更新，1232 条最终指令和两轮反馈全零，faults=[]；
证据 `validation/motion_preflight_20260905_2344.json`。另 25 秒急停断言订阅未收到任一
急停字段 true（timeout 124），这不证明急停故障，也不证明其有效，需现场按键配合。
ControllerMonitor 未更新不单独判故障：源码明确其为固件可选广播，四项必需 CAN 状态
仍新鲜。候选导航/Qt 保持运行、原版 inactive；BPU 独立 tmux 同一会话已达 639 秒/312 帧。

23:35 最新：用户要求启动车辆测试，并提出删除看门狗/设置 true。尚缺测试区域、
最高速度和实体急停确认；未删除看门狗、未改变两个运动门。实时定位 LOCALIZED，
已有活动导航目标，上游曾请求约 0.70 m/s，最终 watchdog 因 motion=false 输出零。
不得未经现场条件确认直接放行旧目标。BPU 旧桥 361402 退出码 137，无正常退出记录，
不能确定强制终止来源；非十分钟到期。已在隔离环境中启动独立 tmux 后台会话恢复，
私有 socket=`runtime/run/bpu_preview.tmux.sock`，session=`j6m-bpu-preview`；
当前 BPU 会话 `qt_20260905_233426_560948`，不依赖本轮临时终端。没有修改系统服务。
停止仍用实验 `start_qt_bridge.sh --stop`，不要删除私有 socket 或杀未知进程。

23:21 最新：用户再次要求关闭当前 Qt 后带静态地图重启，已统一完整冷重启并恢复持续 BPU。
地图仍为 `map_20260829_221335_gimp_cleaned`，Qt 地图 READY、实际框图更新；
20 秒静止检查命令/轮速全零，等待新的初始位姿，两个运动门 false，原版 inactive。
当前运行会话和证据见 `STATIC_QT_RESTART_20260905.md`；本轮没有代码或导航 release 变更。

22:42 最新：用户明确要求 BPU 一直运行，不要十分钟演示。已查明旧桥 `time_limit`
正常退出，改为导航伴随无参数默认持续模式，同时处理板端 660 秒截止；保留超时、
帧龄、限频、资源余量和独占保护。当前持续桥运行，同一进程已实测超过 696 秒/340 帧，
跨过两处截止，55 秒静止检查零速/无故障，详见
`CONTINUOUS_BPU_20260905.md`。下方 600 秒描述只适用于显式限时演示。
没有重启完整导航或切换正式后端；原版仍停止、两个运动门 false。
只写候选/私有实验，无系统服务变更。相机朝向/局部位姿较 22:06 变化，已询问现场
是否人工挪动；最近两段检查输出/轮速均零，不据此宣称全程无移动。
用户随后回复“是有变化”，记录现场变化，不解释为允许发送运动目标。

22:08 最新接续：用户后续明确要求“你帮我启动”，已授权保持静止的完整导航启动，
不是运动授权。21:44 的完整栈和 21:50 的 BPU 伴随运行均完成零速短测；
最新又要求综合页直接显示 BPU，因此仅修改候选 Qt 的综合页显示源，构建和 Qt 测试通过，
已统一停止旧实例、冷启动新 Qt 并确认综合页实际 BPU 框图。22:06 起 20 秒实测速度命令/
轮速全零，地图 READY、等待初始位姿；候选导航运行，原版 inactive。实验桥约 22:16 到期。
实际最新状态、证据和伴随桥接入口以
`FULL_NAV_BPU_OVERVIEW_20260905.md` 及 `validation/overview_bpu_*` 为准。
这条更新取代下方“全部停止/不得自动重启”的历史交付状态；仍不授权运动、猜测初始位姿
或放开两个 false 运动门。18:22 的轮速脉冲根因、全局定位稳定性仍未查明。
FCOS 仅显示，未替代正式 YOLO/五材质后端，未验证 GPU 节省量；BPU 伴随预览最长 600 秒。

21:16 最新补充：用户要求 BPU 与导航 Qt 一起显示。候选 Qt 新增独立预览页，
通过私有相机 master 的真实显示/隐藏停算/恢复/桥接停止留图验证，135 帧 BPU 结果。
当前相机、Qt、桥接和 BPU worker 均停止；两个完整导航栈仍停止。源码/Qt 本机构建已更新，
正式后端和 J6M 导航 release 未改。详见 `experiments/j6m_bpu_20260905/QT_PREVIEW_RESULTS.md`。
正常导航并行运行、原模型 BPU 量化与 GPU 节省量未验收，不自动恢复运动或完整栈。
开工时主机资源未饱和；用户反馈卡顿来源尚未确认，不得擅自重启 ToDesk/修改系统服务。

本副本为 `/home/slam/robot_j6m_ws_optimized_20260905`，当前交付状态见
`OPTIMIZATION_REPORT.md`。下方从原项目继承的历史描述不授权改动原目录。
所有修改、日志和构建仅在副本及 J6M `/map/robot_j6m_optimized_20260905` 中进行；
本机运行统一经 `scripts/optimized.sh` 隔离入口。不得为消除权限错误解除只读隔离。
原版与候选版不能并行控制底盘。运动区域、速度和急停尚待用户确认，不能复制旧运动授权标记。
不得把回放、单元测试或相机静态测试写成实车导航验收。
用户最新明确要求“不要移动，可以启动导航，MID360 已启动”。16:59 已通过隔离入口启动
候选完整栈，双运动放行 false、地图 READY；原版 inactive。17:08:57 起 Qt 操作员
已提交初始位姿，costmap 开始发布。17:11 能进入 LOCALIZED，但 30 秒窗口仍有
DEGRADED/ALIGNING；RMSE 接近 0.35 m 门限，不能宣称定位稳定。详情及实测 JSON 见报告。
18:22 后状态更新：静止续测检测到 4 次左轮 ±0.01237 m/s 反馈，命令均为零，
已通过统一入口停止双机栈；原版也停止。相机画面有近距离白色遮挡、深度有效比例低。
在现场确认车辆静止、轮速脉冲来源与相机遮挡之前，不得自动重启。
新增相机内参别名修复已本机构建并通过 949 项测试，但未做修复后实机启动，J6M 发布未变。
最新状态和证据见 `STATIONARY_FOLLOWUP_20260905.md`。不得猜测初始位姿、发送目标或恢复运动放行。
用户随后确认“完全静止”，但轮速反馈来源和相机遮挡未查清，不自动重启完整栈。
用户已授权把神经网络推理迁至 J6M BPU、释放 NVIDIA GPU；ZED SDK/深度仍留 NVIDIA。
19:27 BPU 实验阶段结果见 `experiments/j6m_bpu_20260905/RESULTS.md`：
参考 FCOS 实机离线运行成功，原模型三份 ONNX 导出及 30 项浮点对照通过；
没有生产切换、实时跨机适配或 GPU A/B，缺 x86 OpenExplorer 量化编译环境。
实验只写独立实验目录，原/候选导航均停止，默认后端和两端导航 release 未变。

# 原项目继承的工作约束

本文件适用于 `/home/slam/robot_j6m_ws` 整个目录树。进入子目录工作时也必须遵守。

## 信息源与优先级

开始任何修改前，至少阅读：

1. 本文件；
2. `README.md`；
3. 与任务有关时再读 `docs/ARCHITECTURE.md`、`docs/HANDOFF.md` 和对应脚本。

`/home/slam/AGENTS.md` 要求读取的全局交接文档记录的是更早期的
`/home/slam/robot_ws` 迁移状态，其中旧网络拓扑、旧设备归属和“尚未完成”结论可能
已经过时。发生冲突时，本项目当前的 `AGENTS.md`、`README.md`、
`config/dual_host.env`、实际脚本和实时只读检查优先。

本项目是室内 J6M 双机版本；`/home/slam/robot_ws` 继续承担机场 GPS 模式。不要用
旧工作区的一体化脚本启动本项目，也不要顺手修改、同步或清理旧工作区。

## 源码和工作区保护

- 工作树可能包含用户已有的 tracked、untracked 和子模块修改。先运行
  `git status --short`，只改任务直接涉及的文件，保留所有无关变化。
- 未经明确要求，不得执行 `git reset`、`git checkout --`、`git clean`、提交、推送、
  删除地图/rosbag/日志/发布版本或重写用户配置。
- 不得刷写 J6M 的 ACORE、MCU、boot、system、整盘镜像或机器人固件。
- 不要把 `/home/slam/robot_ws` 或 J6M 上的无 Git 源码副本当作本项目权威源码；权威
  开发目录是 `/home/slam/robot_j6m_ws`。
- 不要为了抢占串口或 ROS 节点而使用宽泛的 `pkill`/`killall`。项目脚本只停止具有
  PID、运行令牌或严格工作区来源证据的进程；无法证明归属时应报告并停止操作。

## 双机职责和 ROS 拓扑

NVIDIA 本机负责：

- MID360 物理网口驱动；
- USB-CAN/M2、前后 LD19；
- ZED、CUDA、YOLO11、Qt/RViz；
- `/cmd_vel` 最终看门狗。

J6M 负责：

- ROS master；
- Livox topic relay、FAST-LIO、已知地图 ICP 定位；
- MID360/LD19 避障融合、map_server、move_base + TEB；
- FOD 安全仲裁。

两端是两个独立进程环境，但加入同一 ROS1 图：

```text
J6M:    ROS_MASTER_URI=http://192.168.10.100:11311
        ROS_IP=192.168.10.100
NVIDIA: ROS_MASTER_URI=http://192.168.10.100:11311
        ROS_IP=192.168.10.50
```

不得设置 `ROS_HOSTNAME`。ROS1 节点通信使用动态 TCP 端口，两端必须双向可达，不能
只验证 11311。不要默认跨机发送 ZED 未压缩图像、深度图或完整视觉点云；跨机 topic
应维持现有最小数据链。

节点运行位置由两端不同的启动入口决定，不是由 ROS 自动调度：

- NVIDIA：`scripts/nvidia_gateway.sh` / `nvidia_gateway.launch`；
- J6M：远程 `deploy/j6m/start.sh` 最终执行已安装的 `j6m_stack.sh` 和
  `j6m_fastlio_navigation.launch`；
- `scripts/start_dual_host.sh` 通过 SSH 启动 J6M，再启动 NVIDIA 端，并由用户级
  `autolabor-dual-host.service` 托管完整进程树。

## 本机编译与 J6M 部署是两套产物

严禁假定本机 `devel` 会自动出现在 J6M。

### NVIDIA 本机产物

`./scripts/build_workspace.sh` 在 NVIDIA 本机执行 `catkin_make`，产物位于本项目的
`build/` 和 `devel/`。`scripts/setup_env.sh` 加载本机 `devel/setup.bash`，因此本机
节点使用本机产物。

### J6M 产物

`./scripts/deploy_j6m.sh` 执行另一条独立部署链：

1. 通过 `rsync` 只发送选定源码，明确排除本机 `build`、`devel`、`install`；
2. 通过 SSH 在 J6M 的 Ubuntu 20.04 ARM64 chroot 内原生执行
   `catkin_make install`；
3. 生成 `/map/autolabor_runtime/rootfs/opt/autolabor/dual_host/releases/<时间戳>/install`；
4. 完成 `rospack`、共享库和 launch 检查后，原子切换
   `/opt/autolabor/dual_host/current` 符号链接；
5. J6M 启动时依次加载 `/opt/ros/noetic`、基础
   `/opt/autolabor/ros/install` 和项目版本 `/opt/autolabor/dual_host/current`。

不要复制 NVIDIA 的 `devel` 或本机 ELF 到 J6M，也不要让 J6M 直接运行源码目录中的
未安装脚本。J6M 使用版本化 catkin install 空间，回滚只允许在停止主链后通过
J6M 上的 `/map/autolabor_runtime/dual_host/bin/rollback.sh` 切换 `current`。

当前 `deploy_j6m.sh` 只同步并编译其 `paths` 和
`CATKIN_WHITELIST_PACKAGES` 中的包，主要包括：

```text
conventional
teb_local_planner
fast_lio
fast_lio_localization
robot_bringup
autolabor_dual_lidar
autolabor_fod_control
autolabor_dual_host
```

`livox_ros_driver2` 等当前来自 J6M 基础
`/opt/autolabor/ros/install`。若修改了不在部署白名单中的包，不得声称普通
`deploy_j6m.sh` 已让修改在 J6M 生效；应先核对依赖，再明确扩展同步路径和白名单，
或按用户授权更新基础 install。自定义消息有变化时必须保证两端消息定义和 MD5 一致。

### 修改后的选择

- 只影响 NVIDIA 节点：本机重新构建、相关测试、完整冷重启；通常无需部署 J6M。
- 影响 J6M 节点、J6M launch/config 或两端共享包：停止双机栈，本机构建和静态检查，
  执行 `deploy_j6m.sh`，同步时间，再完整冷启动。
- 同一个包在两端都有节点：本机 `devel` 和 J6M 版本化 `install` 都必须更新。
- 静态地图不是程序发布物；使用现有 `sync_static_map.sh` 或带 `--map-set` 的一键启动
  同步，不要塞进程序 release。

推荐发布流程均在 NVIDIA 执行：

```bash
./scripts/start_dual_host.sh --stop
./scripts/build_workspace.sh
./scripts/health_check.sh --static
./scripts/deploy_j6m.sh
./scripts/sync_j6m_time.sh
```

部署属于外部状态修改；若用户只要求解释、审查或诊断，不要擅自部署。

## 网络与设备身份

当前固定拓扑：

```text
MID360 192.168.1.112
  ↕
NVIDIA WCH USB Ethernet 192.168.1.50

J6M eth0 192.168.10.100
  ↕ 交换机
NVIDIA ASIX USB Ethernet 192.168.10.50

NVIDIA wlan0：互联网默认路由
```

NVIDIA USB 网卡的 `eth0/eth1/eth2` 名称会在重启后变化，绝不能把接口名当作硬件身份：

```text
6C:1F:F7:C4:96:B8  ASIX -> 交换机/J6M
50:54:7B:E3:C9:10  WCH  -> MID360
```

- 使用 `scripts/load_config.sh` 的硬件身份解析结果；永久 MAC 未命中时，只允许用
  `config/dual_host.env` 中同时配置且唯一匹配的 USB VID:PID + serial 兜底。不得仅凭
  VID:PID、驱动名或现存 `ethN` 自动认领网卡。
- 正常启动会在任何 J6M 远程停机前调用 `scripts/network_prepare.sh`，恢复
  NetworkManager 托管、修正 profile、等待载波/地址/两端可达；不要把这个顺序改回
  “先远程 stop，后修网络”。同步停机后还必须再次复检网络。
- NetworkManager 持久 profile 应绑定永久 MAC、`connection.autoconnect=yes`，并让
  `connection.interface-name` 保持为空。空接口名是预期状态，不要“修复”为某个
  易变的 `ethN`；激活时可以临时传入当前 `ifname`。
- 两个机器人有线接口必须位于不同 `/24`，且不得接管 Wi-Fi 默认路由。
- J6M 固定地址由厂商硬件持久配置 `hrut_ipfull` 保存；不要依赖易失 overlay 中的
  `/etc` 网络文件，也不要未经确认删除 J6M 默认路由。
- `scripts/network_check.sh` 同时检查 J6M 和 MID360。若它失败，要逐项区分是哪条链路，
  不得把 MID360 未供电误报成 J6M 网络仍故障。

首次网络配置只在两端设备均已接线供电且用户授权时执行：

```bash
sudo ./scripts/configure_network.sh --apply
./scripts/network_check.sh
```

常用只读诊断：

```bash
ip -br address
ip route
nmcli -t -f NAME,TYPE,DEVICE connection show
./scripts/network_check.sh
```

J6M 重启后时钟可能失效；网络恢复后运行 `scripts/sync_j6m_time.sh`。不要把错误时间
引起的启动拒绝误判为编译或 ROS 故障。

## 串口规则

不要猜测 `/dev/ttyUSBn`：

- J6M 串口控制台：
  `/dev/serial/by-path/platform-3610000.xhci-usb-0:4.1.1:1.2-port0`，`921600` baud；
- CAN：`/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B400CG51-if00-port0`；
- 前后 LD19：`/dev/autolabor/lidar_front`、`/dev/autolabor/lidar_rear`，由稳定物理 USB
  路径产生；改变插口后必须重新做逐个拔插确认。

先运行 `./scripts/discover_devices.sh`，再核对 `config/dual_host.env`。出现
`IN_USE_BY_PID` 时只报告占用者或安全停止能证明属于本项目的进程，不得杀死未知进程。
使用 screen 诊断后必须退出并释放串口。

## ZED 相机就绪判定

- ZED 2 视频端 `2b03:f780` 必须以 `5000M` 或更高的 USB 3.x 速率枚举；`480M`
  是 USB 2.0 降级，不能仅凭 `2b03:f780/f781` 出现在 `lsusb` 就判定正常。
- 启动或诊断先运行 `./scripts/zed_camera_check.sh --wait 0`。`f780/f781` usbfs 必须对
  `slam` 可访问；hidraw 是可选内核接口，不得因它未生成而单独拒绝已能被 ZED SDK
  识别的相机。重启后 usbfs 若遗留为 `root:root 0600`，使用已安装的
  `autolabor-zed-coldplug.service`，首次安装入口为 `scripts/install_zed_udev.sh`。
- `/zed2/zed_node` 存活不是相机 ready。必须确认 `/fod_camera/image_raw` 和
  `/fod_camera/depth_registered` 有新鲜消息；否则检查 `log/nvidia_ui_*/vision.log`。
- 不要用可能争抢相机的 ZED Explorer/Diagnostic 作为运行栈内健康探针；确需运行时
  先通过统一入口完整停止双机栈，诊断完再完整冷启动。

## 启停、地图和运行态

正常操作只使用统一入口：

```bash
./scripts/start_dual_host.sh             # 无图 FAST-LIO 模式
./scripts/start_dual_host.sh --status
./scripts/start_dual_host.sh --restart   # 完整冷重启
./scripts/start_dual_host.sh --stop      # 同步停止并验证残留
```

- 不要只重启 `nvidia_gateway.sh`；Livox 断流而 J6M FAST-LIO 未同步重置曾导致位姿严重
  发散。修改和故障恢复优先完整冷重启。
- 手工分端启动仅用于诊断，不能替代一键脚本的地图同步、时间同步和生命周期管理。
- 静态地图模式使用 `--map-set global_maps/map_sets/latest`。切换地图模式必须冷重启。
- 静态地图冷启动不会复用历史位姿；必须由操作员根据车辆真实位置发送
  `/initialpose`，并等 `/fast_lio/localization_status` 为 `LOCALIZED`。不得猜测初值，
  也不得把 FAST-LIO 健康等同于全局定位成功。
- `map_server` 只加载二维地图；全局定位由 `fast_lio_localization` 提供
  `map -> camera_init`，本模式不使用 AMCL。
- 定位前 Qt/RViz 依靠 `operator_gui.launch` 的
  `map -> autolabor_map_display_anchor` 静态叶子识别二维地图根帧。该叶子绝不能改成
  `map -> camera_init/base_link`，也不得据此声称车辆已定位；它仅解决初始位姿前的
  地图渲染循环依赖。
- Qt 收到 `/map` 不等于内嵌 RViz 已完成渲染。静态地图启动验收还必须要求
  `/autolabor_operator_gui/map_display_status` 为 `READY`；保留 MapDisplay 自动重订阅和
  实际宽、高、分辨率校验，禁止把这项检查降级成仅检查 `/map` 有消息。
- 静态建图必须从无图模式开始，并确认 MID360、IMU 和要求的固定物理口 LD19 在线；
  普通 rosbag 录制不会自动生成地图。

日志位置：

```text
NVIDIA: /home/slam/robot_j6m_ws/log/
J6M:    /map/autolabor_runtime/logs/dual_host/
```

## 运动安全与运动门交付状态

本项目的正常交付状态为 `MOTION_ENABLED=true`。诊断、部署、停机或排障过程中可以按需
临时改为 `false`，但任务结束前必须恢复为 `true`，同步到 J6M，并核对实际运行的
`nvidia_gateway.launch` 参数也是 `motion_enabled:=true`。除非用户明确要求，禁止把
`MOTION_ENABLED=false` 作为最终状态遗留。`FOD_MOTION_ENABLED` 是独立功能门，不得因
恢复主底盘运动门而擅自联动修改。

`runtime/motion_authorized.ok` 是独立的启动安全门：用户已确认车辆架空、人员远离车轮、
实体急停可用且 CAN 端口已确认后，可以创建并保留；不得在部署、重启或任务收尾时静默
删除。若现场条件未确认或标记缺失，不得代替用户虚构确认，应先询问用户。

即使主运动门已经恢复，除非用户在当前任务中明确授权运动测试，并且现场同时满足车辆
架空、人员远离车轮、实体急停可用、CAN 端口逐个确认、速度限制不高于 `0.3 m/s`，否则：

- 不发布导航目标、非零 `/cmd_vel` 或会恢复旧目标执行的控制指令；
- 不把“启动成功”“`/cmd_vel` 非零”或“定位为 LOCALIZED”当作底盘已经安全执行；
- 诊断控制链时只允许观察、健康检查和零速验证。

只有用户明确要求撤销运动授权时，才执行：

```bash
./scripts/authorize_motion.sh --revoke
```

MID360/IMU 缺失、静态定位未完成或退化、CAN 未确认、指令过期、节点所有权异常时，
现有安全门应保持零速。修改相关代码时必须保留这些 fail-closed 行为，并为安全边界添加
或更新测试。

## J6M chroot 和持久数据

- J6M rootfs：`/map/autolabor_runtime/rootfs`，Ubuntu 20.04 ARM64 用户态，使用 J6M
  自身内核。
- `/dev`、`/proc`、`/sys`、`/run`、`/tmp` 及持久配置/地图/日志目录由脚本 bind mount。
- 不要把地图、日志、FAST-LIO PCD、ROS home 或其他运行数据写进 overlay 根目录；使用
  `/map/autolabor_runtime/{config,maps,fast_lio,ros-home,logs}` 对应持久目录。
- 不要直接修改某个历史 release。通过新部署创建新 release，失败时保持 `current`
  不变；回滚前必须先停止 J6M 主链。

## 验证和交付

根据改动风险选择验证，不能只说“应该可以”：

- Shell 修改至少运行 `bash -n` 和 `git diff --check`；
- Python/ROS 包运行对应单元测试或 rostest；
- 通用静态检查使用 `./scripts/health_check.sh --static`；
- 网络修改使用 `./scripts/health_check.sh --network` 或 `scripts/network_check.sh`，并区分
  J6M 与 MID360 的结果；
- 部署后运行 J6M 的
  `/map/autolabor_runtime/dual_host/bin/health_check.sh`；
- 主链启动后才运行 `./scripts/health_check.sh --runtime`；
- 改动两端消息、launch 或控制链时，验证节点实际运行主机、topic 发布者/订阅者、消息
  MD5、频率、时间新鲜度和双向网络；
- 未经明确要求不要为验证重启整机、部署、启动实车主链或连接运动授权。

最终交付必须说明：改了哪些文件、验证了什么、是否部署到 J6M、J6M `current` 指向
哪个 release，以及仍受哪些离线硬件或安全条件限制。不要把本机构建成功描述成远端
部署成功，也不要把静态检查描述成实机运行验收。
