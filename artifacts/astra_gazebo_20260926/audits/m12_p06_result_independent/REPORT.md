# P06 独立终止诊断

1 consumed / 5 retired，首回合完成命令日程后因terminal drain deadline失败。原报告没有122门assessment；不把离线补算写回资格。573项release和578项回合活动/归档绑定均匹配。

已闭合bag独立证实：唯独raw缺world_links源时刻37.142 s这一帧，bag完整保留；其他九条规则源流全等，全部formal数据覆盖。raw恰一条GetWorldPropertiesResponse不可调用错误：finally把world回调函数名覆盖为服务响应，迟到world_links回调解析该闭包时失败。暂停watermark37.146 s稳定；source complete、terminal事件与driver实际37.114 s零命令接收均已齐备，只有尾包条件失败。

原生20ms world-link formal段斜率描述性峰值0.856484123849 rad/s，高于冻结0.85门。非轮地/非地contact计数均0。这些是保留infra_failure下的离线描述，不是补发资格或一般安全保证。PHY04按预注册not_complete；无替代图。

审查盲点明确保留：先前独立finally AST夹具未把真实callback词法闭包与服务响应赋值放在同一执行作用域，因而漏掉world名称覆盖。后继仅应以新身份修复该绑定，并增加完整同作用域迟到callback回归。旧源代码、报告和预算不得改写。
