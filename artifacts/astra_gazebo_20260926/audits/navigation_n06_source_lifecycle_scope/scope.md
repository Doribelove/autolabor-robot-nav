# N06 最终 source/lifecycle 独审范围（未通过声明）

本文件只准备审查范围，`independent_source_review_pass=false`。N06 必须使用自身 stage `astra_navigation_n06_lookahead_static_pilot`；N05 source_review 仅作源/native 继承参考，不复制其身份、动态 actor 或结果资格。

已只读重哈希 prior preparation 资源，复核八行实际 contract/launch/world/URDF，提取真实运行入口、递归本地 import 静态超集、xacro include、三个 custom native plugin 与非 actor native 角色。完整表见 required_source_bindings.json。静态超集含分支未激活的旧依赖，不声称这些分支在 N06 运行。

最终必须覆盖：①四个新 D04 static parent 与原失败 witness、八份 input/conversion、0.7→1.2 唯一 factor；②required collector 的唯一 spawn/unpause/pause gateway 与失败取证；③公共 base/source 终态及未来 odom 排队；④实际 scan/TF/odom/joint、map/path 和纯 analytic route core；⑤dense/contacts/physical source、有限 watermark drain、callback freeze、任务/安全/图像完整性分离；⑥P05 仅有限力驱动和纯 observer，native ODE/Gazebo/libs 与 graphics 有界例外；⑦新 r03 one-shot/process/receipt/行政投影；⑧真实运行 ELF/maps 与离线 converter 的对应。

仍缺：r03 最终独审；N06 runtime 磁盘闭包冻结与独审；新 N06 完整资源 source-review API；未来 exact release/freshness 检查。现场物理、实际回调 source join、mapped ELF 与闭环效果尚未发生。本轮未重新执行 launch dump、native build、ROS 或仿真；prepared 的既有真实 XML 展开仅引用此前独审记录。r03 冻结后优先回归其两个回执负例，不扩测试到无关算法。
