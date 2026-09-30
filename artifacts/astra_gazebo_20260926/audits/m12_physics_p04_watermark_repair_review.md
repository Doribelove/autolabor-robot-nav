P04 暂停源水位修复通过独立离线审查。

冻结 probe SHA：`e1c86f35a652d9473376318023ee18862b5035c9d5d2858d2bcabad49157ee5b`。作者测试34/34；独立测试32/32，后者提取当前真实 finally 与 service helper，在纯合成服务、时钟、队列和临时输出下测试全部十流延迟尾包、永久缺包、内部缺包/重复/未来包、暂停/服务异常、服务耗时共用预算、双查询漂移、四项join失败、原异常优先、durable失败输出及锁内关闭。没有运行ROS/Gazebo或编译。

assess及所有旧数学helper的AST完全相同，source_packet_integrity只新增final_paused_source_watermark_drained。完整解析toy基线121/121通过，原120门值及全部数值metrics与P02精确相同；NaN反例仍拒绝。旧P03 Kd6原54项检查精确重放，新门对未记录服务水位的旧raw为false，未重写任何历史资格。假设37.144s水位的离线复核恰好定位dense_links与contacts缺尾，不当作真实服务记录。

GetWorldProperties.sim_time取自World::SimTime；GetPhysicsProperties.pause取自IsPaused。已核对本地srv字段及[ROS上游实现](https://raw.githubusercontent.com/ros-simulation/gazebo_ros_pkgs/noetic-devel/gazebo_ros/src/gazebo_ros_api_plugin.cpp)。远端源码仅提供接口语义，不冒称精确安装二进制的源码版本证明。

关闭前四项join在同一锁内计算并保留，随后active=false；false join会由原评估器保留为失败，不可能得到总pass。新门要求十流从各自首stamp至源水位最后应有slot完全连续，尊重50Hz的两个不同相位；不裁尾、不插值、不增加仿真时间。服务、20ms双查询间隔、等待锁和join计算计入同一个暂停后5s验收预算。该预算不是强制打断Python计算或fsync的实时执行保证，超预算会拒绝。

结论仅为离线测量修复审查通过；不授予执行权限、不证明物理/导航性能，也不改变P03停机、失败或预算退休。
