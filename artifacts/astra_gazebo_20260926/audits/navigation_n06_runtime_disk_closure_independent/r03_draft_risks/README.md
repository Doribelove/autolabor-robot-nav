# N06 r03 磁盘闭包草案：独立环境风险复核

范围仅是已完成且不再变化的 `collection_attempt02`（作者报告5203磁盘资源、602 ELF节点、1799命令、64 resolved node行）。本次没有重哈希全部5203资源，没有给最终闭包通过结论；r03和后来的r04草案都不能以本报告作为release审查。

已独立保留两个具体反例。

1. 精确抽取 frozen r03 manager 的 `env=dict(os.environ,...)` 语句，给合成父环境注入 LD_PRELOAD、LD_AUDIT、PYTHONHOME、PYTHONUSERBASE、EGL vendor路径值。只执行字典构造，不加载任何假库。所有值进入子环境；同一合成环境在作者collector的safe_environment函数中均被排除，astra_env源码也未unset这些键。因此磁盘探针的环境域与执行入口环境域并不相同。这不等于当前实际机器存在污染。
2. 对实际 `/opt/ros/noetic/lib/gazebo_ros/gzserver` 包装器仅把最后 `gzserver $final` 换成 `/usr/bin/env`，在已保存、显式传入的probe环境中运行其前段。没有ROS/Gazebo主进程。它再次source Gazebo setup，改变plugin/library/model/resource搜索路径，且空模型数据库URI变成 `http://models.gazebosim.org`。当前9个world/URDF/系统插件仍选中与草案相同的canonical文件，因此没有捏造已发生不同插件加载/远程下载；但最后gzserver实际环境不能用source astra_env后的环境冒充。

r05及新closure的验收条件：

- 在manager第一次Python/ELF加载之前，就由可信受限父入口建立并绑定解释器、PATH/PYTHONPATH、venv配置、loader搜索和graphics选择环境；manager内部check不能追溯证明已加载代码没有受先前环境影响。限定正常受控启动闭包，不声称能抵抗恶意预加载构造器。
- 每次child和assessment启动均使用同一已审核环境构造；与测得磁盘探针环境逐键比较。run-specific端口、ROS_HOME/日志根和非凭据nonce列为唯一显式替换；loader/Python注入变量必须有明确absent规则。DISPLAY/XDG、graphics vendor/software-render选择和locale/线程设置需要记录或明确约束，不能因为不记录就认为固定。
- 对Gazebo wrapper第二次setup后的最终环境另存证据；绑定canonical wrapper/libcommon/setup/pkgconfig/catkin-find和系统插件。模型数据库网络边界必须明确，不能把空上游变量等于已禁网；当前世界全展开，不据此推断已下载。
- 遍历8份实际launch、manager/roslaunch/master/rosout、Python不同入口、插件与动态加载配置；对ELF loader/DT_NEEDED/RPATH/RUNPATH及ctypes/dlopen来源绑定所选磁盘文件、alias与解析证据。父RPATH上下文补充只能引用实测父root ldd表，保留standalone失败，不造搜索路径。
- 冻结新revision全部closure资源并独审。磁盘ldd/readelf、import探针及包范围只描述该环境下可解析内容；独立run仍须PID/start ticks、实际proc/maps、native/graphics/selected plugin匹配及源闭包，不能把磁盘文件存在写成已经被运行进程加载。

没有执行release、种子登记、训练、ROS或Gazebo。原草案与反例不修改；待r05及全新作者closure正式冻结后再作完整独审。
