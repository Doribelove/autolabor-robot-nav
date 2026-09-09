# autolabor_coverage

ROS 1 静态地图覆盖导航包，包含 Python 覆盖几何与任务管理器、C++ Hybrid A* / Navfn
全局规划插件、固定挡位 TEB 命令 mux 和 ROS msg/srv/action。

算法实现、接口与已有测试均在本目录。仅几何算法可用 Python 3 标准库直接运行：

```bash
python3 test/test_coverage_geometry.py
```

完整构建依赖 ROS Noetic、autolabor_canbus_driver 消息和项目定制 TEB。只复制本包而
遗漏这些依赖不能完成当前运行链。源项目提供独立白名单导出，保留配套依赖和无硬件示例。

在源工作区中，发布步骤见 `docs/COVERAGE_OPEN_SOURCE.md`，公开文档模板位于
`distribution/coverage_navigation/repository/docs/`；在独立公开仓库中，直接阅读仓库根
`README.md` 与 `docs/{ARCHITECTURE,TESTING,PORTING,KNOWN_ISSUES}.md`。

`AUTOLABOR_COVERAGE_WORKSPACE_CONTRACT_TESTS` 默认 ON，执行需要完整整机文件的
源码契约检查。公开三包构建将其明确设为 OFF；其余几何、状态机、mux 与 C++ 测试保留。
该构建选项不修改运行时行为。

项目原有代码按 MIT 声明整理；`src/reeds_shepp.cpp` 保留 Rice/OMPL 的 BSD 3-clause
许可和署名。实车接入需要真实状态与最终命令保护，合成 fixture 仅在隔离 ROS 图中使用。
