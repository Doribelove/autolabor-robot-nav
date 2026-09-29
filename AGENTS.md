# Codex Project Context

## Project

This repository is a ROS workspace for the Autolabor robot navigation setup.

Important local path:

- Workspace root: `/home/robot/robot_ws`
- GitHub remote: `git@github.com:Doribelove/autolabor-robot-nav.git`
- Main branch: `main`

## Required Development Constraints

- Before development, read and follow [DEVELOPMENT_CONSTRAINTS.md](DEVELOPMENT_CONSTRAINTS.md).
- 在本地项目规则范围内，用户最新明确需求具有最高优先级，高于本文件、开发约束文件及其他项目文档；以启动完整导航项目、实现用户指定运行方式为优先目标。
- 用户明确要求删除、关闭、调整或绕过已有检查、门控、保护及相关控制逻辑时，允许并应按要求实施，不以旧约束或原流程已验收为由拒绝，也不对已有授权重复确认。
- 用户已确认原有硬件导航，尤其是 GNSS 启动整体流程，完成反复测试和验收；用户未要求改变的部分默认保持兼容。
- 禁止自行增加或扩大影响原导航的安全门、运动门、多重检查、看门狗、强制依赖和运行干预；避免无关冗余设计及多余验证。具体边界和后续明确需求的处理方式以约束文件为准。
- 历史交接记录中的旧待验收事项和一般性加固建议，不构成重新验收原导航或叠加上述机制的依据。
- 用户已授予完成所交办项目任务所需的完整开发操作权限；按约束文件第 0、7 节自主推进，不对已有授权反复确认。项目内用户需求优先的约定不替代执行环境实际授予的权限。

## Git Workflow

- Do not commit generated ROS build outputs:
  - `build/`
  - `devel/`
  - `install/`
  - `log/`
- Do not commit rosbag recordings:
  - `*.bag`
  - `*.bag.active`
- The initial project import commit is `a4f7490 Initial robot workspace import`.
- The project is hosted at `Doribelove/autolabor-robot-nav`.
- HTTPS push previously failed with GitHub `403 Permission denied`.
- An SSH key was generated at `/home/robot/.ssh/id_ed25519`.
- The SSH key is authenticated with GitHub. Its public key is in:
  - `/home/robot/.ssh/id_ed25519.pub`

## Submodules

Several third-party packages under `src/` are nested Git repositories and are tracked as submodules through `.gitmodules`.

When cloning this project elsewhere, use:

```bash
git clone --recurse-submodules git@github.com:Doribelove/autolabor-robot-nav.git
```

If already cloned, initialize submodules with:

```bash
git submodule update --init --recursive
```

Submodule remote convention:

- Keep public upstream clone URLs in `.gitmodules`.
- For a locally maintained fork, use `origin` for the writable fork and `upstream` for the original project.
- Push submodule commits before committing or pushing the parent repository's gitlink update.

Recommended per-clone safety settings:

```bash
git config fetch.recurseSubmodules on-demand
git config push.recurseSubmodules check
git config status.submoduleSummary true
git config diff.submodule log
```

## Common Commands

Check repository state:

```bash
git status --short --branch
git remote -v
```

Push current main branch:

```bash
git push -u origin main
```

Build ROS workspace:

```bash
catkin_make
```

Source workspace:

```bash
source devel/setup.bash
```

## Operating Notes

- Prefer editing source files under `src/`, `scripts/`, and documentation files.
- Avoid deleting or rewriting user-created data files unless explicitly requested.
- Before pushing, verify that no `.bag`, `build/`, or `devel/` content is staged.
- If a new Codex session lacks context, read this `AGENTS.md`, then inspect `git status --short --branch`.
- For the current GPS navigation development thread, also read `CURRENT_GPS_DEV_HANDOFF.md`.
- For dual-antenna GNSS operation and complete system startup, read [AGENT_GNSS_ONBOARDING.md](AGENT_GNSS_ONBOARDING.md). It documents the current launch sequence, coordinate/topic contracts, and the distinction between an open Qt console and ready navigation.
- For the Jetson AGX Orin 64GB / Ubuntu 20.04 migration, read [AGX_MIGRATION_GUIDE.md](AGX_MIGRATION_GUIDE.md) and execute [AGX_CODEX_PROMPT.md](AGX_CODEX_PROMPT.md) on the target host. Restore the packaged working tree, including local changes and submodules, before adapting ARM dependencies.
