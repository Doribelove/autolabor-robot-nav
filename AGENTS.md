# Codex Project Context

## Project

This repository is a ROS workspace for the Autolabor robot navigation setup.

Important local path:

- Workspace root: `/home/robot/robot_ws`
- GitHub remote: `git@github.com:Doribelove/autolabor-arena-nav.git`
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
- The project was prepared for pushing to `Doribelove/autolabor-arena-nav`.
- HTTPS push previously failed with GitHub `403 Permission denied`.
- An SSH key was generated at `/home/robot/.ssh/id_ed25519`.
- The public key that needs to be added to GitHub is in:
  - `/home/robot/.ssh/id_ed25519.pub`

## Submodules

Several third-party packages under `src/` are nested Git repositories and are tracked as submodules through `.gitmodules`.

When cloning this project elsewhere, use:

```bash
git clone --recurse-submodules git@github.com:Doribelove/autolabor-arena-nav.git
```

If already cloned, initialize submodules with:

```bash
git submodule update --init --recursive
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
