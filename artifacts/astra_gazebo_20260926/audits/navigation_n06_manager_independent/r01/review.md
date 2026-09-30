# N06 manager r01 独审：三项阻断

352 个冻结资源前后重哈希一致，23 个递归本地模块全部在名册；48 个作者测试独立复跑通过。实际普通 Python 子进程、另建session的孙进程、pidfd/starttick拒绝与subreaper清理通过，无关兄弟进程存活。未运行ROS/Gazebo或创建执行release/seed，真实三CSV字节/inode/mtime均未变。

三项最小反例均已实际复现：

1. 首CSV替换后绑定receipt被修改，行政commit仍写出全部after表及completion。缺少每表/末端validated-input复验。
2. wrong stage、integrity=true但error非空/assessment=None的consumed receipt仍被投影成terminal_timeout。缺少完整receipt/真实assessment闭包。
3. 只有input/converted/resources三键的假conversion通过release结构检查；conversion_resources不是出处认证。分配前应要求八份diagnostic authenticate通过，真实runtime sdformat对应仍留实际收尾门。

原r01不可用于release资格或行政apply；新r02修复后再审。N05外层143原因未知，本清理测试不反推历史原因。所有反例只在临时文件/普通子进程运行，native/runtime stand-in仅证明结构边界。
