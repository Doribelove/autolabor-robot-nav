# N06 八份离线转换独审

438 原始资源前后重哈希一致；8份URDF输入、prepared顺序及每份2次转换输出逐字节一致。独立解析16份raw maps并核对真实PID/startticks、argv/input、stdout、全部ELF的执行映射device/inode和当前文件SHA；16个进程生命周期均不同。四对SDF字节分别一致；每份7个合并刚体、9个collision、质量79.6kg，P05/Kd6/seed保持。

8次diagnostic authenticate通过；错schema、重复process、假重现标志、缺sdformat角色均拒绝。没有实际Gazebo maps时diagnostic=False按预期拒绝。未重跑converter、未启动ROS/Gazebo、未登记seed或创建release；这里只认证已保存转换出处和字节重现，实际运行sdformat对应与物理跟踪仍未资格。
