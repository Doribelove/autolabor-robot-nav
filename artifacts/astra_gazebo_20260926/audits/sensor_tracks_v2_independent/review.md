M08 v2 独立离线组件审查通过；runtime_ready / full_planner_qualified / live_deadline_qualified 均为 false。

独立重跑作者47项和新增/继承53项全部通过。五个公开入口均拒绝早于真实接纳时间1ns的查询，原200ms源新鲜度不扩窗；有效request字段及SHA与v1相同，跟踪算法直接继承冻结v1。实际九组件16平面robot envelope与独立未剪枝GEOS对照通过，全部支持保留，仅额外保守减1e-10m。

独立执行4负载×10次配对benchmark，构造+248查询最大20.023795ms。因此20ms诊断不通过，50ms有限kernel测量通过；原作者18.79ms观测不被当作一般上限。扫描/跟踪与整planner仍未计入，既知40ms密集tracking加上kernel不能证明端到端50ms。旧v1两个失败及原负时延不变。

首轮独立fixture错误使用foot.at(*pose)产生4个TypeError，原源码/日志保留；r02只更正测试签名，未改实现。
