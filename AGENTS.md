# Codex Project Context

## Project

This repository is a ROS workspace for the Autolabor robot navigation setup.

Important local path:

- Workspace root: `/home/slam/robot_ws`
- GitHub remote: `git@github.com:Doribelove/autolabor-robot-nav.git`
- Main branch: `main`

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
- An SSH key was generated at `/home/slam/.ssh/id_ed25519`.
- The SSH key is authenticated with GitHub. Its public key is in:
  - `/home/slam/.ssh/id_ed25519.pub`

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
