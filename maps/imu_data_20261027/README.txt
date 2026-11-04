IMU data.rar — GPS 地图提取与项目接入

Git 保存内容（2026-11-04）
  保存原始 source/IMU data.rar、文件校验清单和运行所需的 derived 参考底图/预览。
  解压的 raw/ 与可重建的 derived/gps_samples.csv 仍完整保留本机，不重复上传。
  新克隆需要完整采样时，先在本目录 mkdir -p raw，
  然后 unrar x -o- "source/IMU data.rar" raw/，再运行文末提取命令生成新的输出目录。

数据位置
  本目录：/home/slam/robot_ws_agx/maps/imu_data_20261027/
  原来源：/media/slam/69B4-43B3/IMU data.rar
  用户已确认坐标系为 WGS84，并说明采集沿可通行区域进行。

提取结果
  原压缩包包含 17 份 TXT。4 份是逐字节相同的重复副本，1 份仅有表头。
  生成地图时使用 12 份非空、非重复记录，共 489,159 条 GPS 数据。
  保留滑行道两次采集及补充、505/506/507 机位、三个机位合并采集，共 7 类图层。
  原始 17 份文件全部保留，未删除重复文件或空文件。

主要文件
  derived/地图查看.html          离线交互地图，可缩放、平移、开关图层、查看经纬度。
  derived/地图预览.png           总览、滑行道、机位三视图。
  derived/地图预览.svg           可缩放矢量预览。
  derived/gps_tracks.geojson     WGS84 轨迹线，标准经度在前、纬度在后。
  derived/gps_samples.csv        全部 489,159 条独立采样，保留经纬度原始数字精度。
  derived/extraction_summary.json 逐文件统计、重复对应关系、空文件、状态码、来源信息。
  source/IMU data.rar            原压缩包副本。
  raw/IMU data/                 解压后的全部原文件。
  source_manifest.json          压缩包和每个原文件的大小、SHA256。

查看方式
  用浏览器打开 derived/地图查看.html 即可，不需要网络或 ROS。
  Qt 导航的嵌入式 RViz 已增加 “IMU GPS survey reference” 图层。
  下次按原完整导航入口启动，GPS 原点就绪后自动按原点对齐这些轨迹。
  在 RViz 显示面板中可隐藏该图层。
  只单独显示时，在已经运行的项目 ROS 环境执行：
    cd /home/slam/robot_ws_agx
    source devel/setup.bash
    roslaunch autolabor_gps_keepout reference_map.launch
  此命令不启动导航、传感器、底盘或机构，只有地图显示节点。

地图含义与当前边界
  当前接入的是实测轨迹参考图，不是新增可通行面或障碍物地图。
  机位数据明显包含多条蛇形往返线，且多份记录的起终点并未闭合。
  已向用户核实这些线是沿区域边界还是区域内部来回采集，目前具体边界含义待补。
  因而没有使用凸包、臆造宽度、自动首尾连线、填满蛇形线间隙或连接两片采集区域。
  原来使用的“第二次修改”机场地图、其禁区及既有 pt5 更正保持原样。
  要把本数据用于新的可行区域规划，还需确定真实闭合边界及内部禁区。

处理约定
  坐标字段 dLongitude、dLatitude 直接作为 WGS84 十进制度，不作 GCJ/BD 等坐标偏移。
  轨迹只用于二维位置显示；没有将 fAltitude 当地面高程图或重建三维点云。
  CSV 中保留 source_path/source_line，及原始 Time、ullTimestamp 字符串，未猜时间单位或转换日期。
  GPS_Status 的原值 4 共 488,539 条，1 共 620 条；未提供设备码表，不擅自解释成 RTK 等级。
  489,159 条经纬度均为有限且范围有效的数值，但这不等于测量精度已验收。
  每份源文件保持独立轨迹，不跨文件自动连线；显示线按 0.05 m 容差简化为 1,177 个顶点。
  此 0.05 m 是显示简化误差上限，不是声称 GPS 达到 5 cm 精度。
  完整采样保存在 CSV 与原 TXT；解析脚本遇到无效坐标或相邻点超过 5 m 时断开显示线。
  这批有效数据中未发现超过 5 m 的相邻坐标跳变。
  显示坐标转换与项目 gps_to_xy 一致：当前 GPS 原点、东向 X、北向 Y、camera_init 平面。

复现与验证
  使用新的输出目录执行，不覆盖既有提取结果：
    python3 scripts/extract_imu_gps_map.py --dataset maps/imu_data_20261027 --output maps/imu_data_20261027/derived_new
    python3 scripts/render_imu_gps_map.py maps/imu_data_20261027/derived_new/gps_tracks.geojson
  5 项单元检查通过；真实提取文件的隔离 ROS 显示测试通过，覆盖原点变化、全部轨迹
  和顶点数量，以及节点仅发布参考 Marker/状态而不发布速度、目标或 TF。
  原压缩包与 17 个 TXT 已校验 SHA256；断电后再次核验完整。
  CSV 数量、每段首尾坐标和原始时间字段与提取结果一致；原导航地图 SHA256 未变化。
  使用脚本、构建日志、隔离验证及修改前备份见 log/imu_map_import_20261103_115042/。
  交互页已做 JavaScript 语法检查，未在浏览器中验证全部交互；PNG 已打开核查。
  本任务没有启动完整导航或实车动作，也没有更新历史实验数据或报告。
