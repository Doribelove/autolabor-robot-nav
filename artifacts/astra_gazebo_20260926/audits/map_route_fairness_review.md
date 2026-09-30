当前 PCT/R2 与实际 TEB/DWA 的地图和全局路径输入不对等。PCT/analytic 直接使用完整 static_boxes/static_circles/走廊宽度做静态点分类、候选净空和路线保持；当前 TEB/DWA 仅从激光维护 rolling costmap，Navfn 未收到同一静态地图。PCT 也未消费 Navfn 全局 path。同起终点不足以称共享全局路径。冻结 Arena Q1 提供同一地图字节的设计参考，但未解决共同 path 消费。

保留原 frozen R2 实现作为必须的独立对照，明确其轴对齐盒解释、固定直线 guide 和未使用发布的共享 path 的限制。不得用 R2+adapter 冒名替代；如另设该探测，应作为独立第六方法。新 PCT/analytic/实际 TEB/DWA 才接入新的共同 map/route 接口。

本轮只新增 static_map_export.py 与离线测试。Exporter 从实际 SDF 静态 collision 生成同一0.05m PGM/YAML；即使 actor 的 static=true，也按声明与已知控制插件排除。保留 OBB 姿态、box/cylinder 尺寸及碰撞原点。未知/域外共同阻塞；不雕空起终点，不隐式加机器人半径。当前已知域固定40×30m，避免把开放场景人为限制成路线两侧2m的未知走廊。旧原型输出保留于 shared_map_route_export_samples，当前样例在 shared_map_route_export_samples_r1。六族各一布局均导出成功；两个 actor replicate 的地图字节逐个相同。

13项测试通过，包括4860像素与独立SAT参考、圆柱覆盖误差、极小物体、旋转盒、actor排除、PGM上下翻转、unknown边界、非恒等map/odom变换和不可覆写输出。只生成 route_request.json，global_path_polyline=null；未将直线请求伪装成已验证的共享路径。

下一接口应绑定同一 map-server 归一化栅格和同一带弧长/切向的 map-frame全局折线，统一投影/裁剪规则。实际TEB/DWA由新的nav_core共享route适配器输入setPlan；新PCT/analytic由新的route-relative接口消费同一参考，仍保留感知动态障碍。地图字节可从现有Arena实现获得设计参考，但新代码须独立命名，runtime/launch/plugin历史文件不修改。是否启用动态全局重规划需另定共同协议。

精确证据SHA、consumer验证门、源文件所有权计划和运行限制均在 map_route_fairness_review.json。本轮未启动ROS、训练或新episode；当前地图/路径公平性尚未具备live资格。
