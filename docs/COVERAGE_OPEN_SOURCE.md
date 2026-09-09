# 覆盖导航公开发布流程（2026-09-09）

目标：`https://github.com/cl4mat/autolabor-coverage-navigation.git`，由用户明确提供。
本源项目的 `origin` 是其他账号的整机仓库，不作为本次发布目标，也不修改其远端配置。
权威修改仍在本工作区完成；公开仓库通过白名单导出生成独立历史，不推送原整机历史。

实际发布结果：源项目准备提交 `5ddce5d`；独立仓库 main
`0350307f9a41e140c3fc5a73a8dc4ec00c00541b` 已推送。授权账号核对为 cl4mat，远端
哈希和 About 简介核对通过；HTTPS 全新克隆验证通过。GitHub Actions run
[34325629265](https://github.com/cl4mat/autolabor-coverage-navigation/actions/runs/34325629265)
完成干净 Noetic 构建、单元测试和真实插件合成场景，结论 success。

## 提交哪些文件

导出清单是 `distribution/coverage_navigation/manifest.json`：

| 内容 | 源项目位置 |
|---|---|
| 覆盖几何、管理器、Hybrid A*、全局插件、mux、接口、配置和测试 | `src/application/autolabor_coverage/` |
| 定制 TEB | `src/navigation_arena/forks/navigation/local_planner/teb/` |
| 当前管理器依赖的底盘消息/服务及原驱动 | `src/autolabor_core/autolabor_canbus_driver/autolabor_canbus_driver/` |
| 导航所有权/暂停桥及其测试 | `src/platform/autolabor_dual_host/{scripts,test}/` 中清单指定的两个文件 |
| 公开 README、架构/测试/移植文档、许可证、Docker、CI、工具 | `distribution/coverage_navigation/repository/` |

公开架构文档为
[`repository/docs/ARCHITECTURE.md`](../distribution/coverage_navigation/repository/docs/ARCHITECTURE.md)，
测试文档为 [`repository/docs/TESTING.md`](../distribution/coverage_navigation/repository/docs/TESTING.md)。
这些模板本身保存在源项目，导出时放到公开仓库根目录相应位置，不维护另一份手写算法源码。

不提交 Qt、视觉模型、相机/雷达资产、FAST-LIO 部署、现场地图、日志、rosbag、
build/devel/install、个人环境变量文件、SSH 凭据、运动授权标记或本源项目历史。
第三方 TEB 和 Reeds–Shepp 的许可证、版权声明原样保留；根 MIT 不覆盖它们。

## 发布步骤

1. 在源项目修改算法及文档，先检查 `git status --short`，保留其他工作。
2. 运行源项目针对性测试，并在全新目录导出：

   ```bash
   python3 scripts/export_coverage_navigation.py runtime/coverage_publication_NEW/candidate
   ```

   导出器拒绝覆盖已有目录，检查清单、软链接、运行资产和常见凭据格式，生成逐文件
   SHA-256 清单。它不复制 `.git`，不修改运行配置、不连接 ROS 或部署 J6M。

3. 从导出目录构建及测试公开版，而不是只复用源项目已编译产物：

   ```bash
   cd runtime/coverage_publication_NEW/candidate
   python3 tools/geometry_demo.py
   docker build -t coverage-navigation:local .
   docker run --rm --network none coverage-navigation:local
   docker run --rm --network none coverage-navigation:local bash tools/run_demo.sh sweep
   docker run --rm --network none coverage-navigation:local bash tools/run_demo.sh preview
   ```

4. 检查生成物、依赖、许可证和验证记录；在源项目明确 `git add` 本次相关文件后提交。
   本次用户授权包括源项目本地提交及向指定个人仓库推送；不因此向原 `origin` 推送。
5. 提交后重新导出最终源码快照，使 `SOURCE_MANIFEST.json.source_commit` 可追溯。
6. 在独立发布目录建立 Git 历史，使用用户指定目标：

   ```bash
   git init -b main
   git add .
   git commit -m "Publish coverage navigation algorithms, tests and documentation"
   git remote add origin https://github.com/cl4mat/autolabor-coverage-navigation.git
   git push -u origin main
   ```

   推送需要对该仓库有写权限的 GitHub 身份；只读克隆成功不证明有写权限。若远端已有
   内容，先读取并整合，不能强推覆盖。HTTPS 可使用本机凭据助手/GitHub CLI 登录；
   SSH 可使用该仓库授权的密钥。不要在命令 URL 或提交文件中放 token，也不要在聊天中
   传私钥。公开与私有可见性以目标仓库设置为准，不更改原仓库可见性。
7. 推送后用 `git ls-remote` 核对提交，再从 HTTPS 地址全新克隆复查；GitHub Actions
   成功后才能把远端 CI 标记为通过。
8. 仓库 About 简介使用公开模板 `.github/repository-description.txt`。该文件会随 Git
   提交，但 GitHub 不会自动把文件内容应用到 About；需要仓库设置权限，使用网页
   About 编辑入口或 `gh repo edit cl4mat/autolabor-coverage-navigation --description ...`。
   README 首段同步说明四轮前轮转向、运动学约束、自选区域及区域外转弯空间要求。

这是 GitHub 官方推荐的本地 Git 初始化、添加远端和推送流程的本项目化版本：
[GitHub：添加本地代码](https://docs.github.com/en/migrations/importing-source-code/using-the-command-line-to-import-source-code/adding-locally-hosted-code-to-github)。

## 为可克隆使用所做的调整

* `test_coverage_contract.py` 默认路径改为从源码位置推导，仍支持环境变量覆盖。
* 整机部署契约测试有独立 CMake 开关，源项目默认 ON；公开三包构建明确 OFF，其他测试保留。
* 首入口 fixture 的历史旧版对照可显式提供；没有本地旧资产时仍执行当前版本全部检查，
  记录未提供历史基线。源项目现有基线存在时继续执行原对照。
* 公共示例在合成地图和隔离 ROS 图中运行；首次克隆无需现场地图/机器人。
* 包元数据补齐测试依赖及 Reeds–Shepp BSD 声明，不改变实时规划/控制算法。

## 验证和当前边界

发布验证结果写入公开模板 `docs/VALIDATION.md`，完整日志留本地
`runtime/coverage_publication_20260909/`。不把本地验证当成远端 CI，不把合成示例当成
新机器人实车验收。现有 5 米清扫窗口修复随代码导出，4 米恢复触发及入口准入保持现状。
换向接续、短 cusp 终态复核、执行无进展和获选搜索线程记录缺项仍列为已知问题。
