# 项目源码快照与恢复说明

本目录保存 2026 年 9 月 30 日同步时，主仓库普通提交不能直接包含的本地源码改动：六个第三方子模块的补丁，以及独立 ASTRA 工作区的 326 个新增或修改文件。补丁均已在临时 Git 索引中恢复，并核对完整树哈希；原有子模块工作树和暂存区保持原状。

本次 GitHub 同步范围为代码、配置、论文材料和结果摘要。原始实验轨迹、ROS 日志、重复数据归档、构建产物和虚拟环境保留在原机器。已有受 Git 跟踪的历史证据继续保留。论文目录中的校验清单可能引用未上传的原始数据，因此克隆仓库不等于获得完整实验数据备份，也不授权重新执行已终止的实验。

## 获取当前研究分支

在新目录中克隆，避免覆盖现有工作区：

```bash
git clone --recurse-submodules --branch research/pct-teb-dwa-gazebo-20260810 \
  git@github.com:Doribelove/autolabor-robot-nav.git robot_ws_research
cd robot_ws_research
```

## 恢复子模块源码改动

仅在新克隆、尚未修改的仓库根目录运行下面的代码。它先核对子模块基线、补丁 SHA256 和所有补丁的适用性，再逐一应用。不要在原机器已有这些改动的工作区重复应用。

```bash
python3 - <<'PY'
import hashlib
import json
import subprocess
from pathlib import Path

root = Path.cwd()
bundle = root / 'snapshots/local_sources_20260930'
manifest = json.loads((bundle / 'manifest.json').read_text())
for item in manifest['submodules']:
    repo = root / item['path']
    patch = bundle / item['patch']
    head = subprocess.check_output(
        ['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip()
    if head != item['base_commit']:
        raise SystemExit('Submodule base mismatch: ' + item['path'])
    if hashlib.sha256(patch.read_bytes()).hexdigest() != item['sha256']:
        raise SystemExit('Patch checksum mismatch: ' + str(patch))
    subprocess.run(['git', '-C', str(repo), 'apply', '--check', str(patch)], check=True)
for item in manifest['submodules']:
    subprocess.run(['git', '-C', str(root / item['path']), 'apply',
                    '--whitespace=nowarn', str(bundle / item['patch'])], check=True)
print('Restored six submodule source patches.')
PY
```

`manifest.json` 记录每个补丁的基线提交、文件列表、哈希及排除的本地日志。`robots` 子模块仅有已跟踪 Python 缓存的本地删除，没有需要恢复的源码补丁。

## 查看和恢复独立 ASTRA 源码

`astra_workspace_overlay/` 保存原路径
`artifacts/astra_gazebo_20260926/workspace/` 中相对主工作区不同的源码与配置，包含 196 个 ASTRA 脚本及测试文件、环境脚本、依赖清单及仿真组件。其余相同文件由本次主仓库提交和上面的子模块补丁提供。

如需恢复这份独立源码，另建一个同分支克隆，先应用上面的子模块补丁，然后在该新克隆根目录执行：

```bash
cp -a snapshots/local_sources_20260930/astra_workspace_overlay/. ./
```

这会将该独立克隆切换为保存的 ASTRA 源码状态。清单中的 `base_sha256` 是同步时主工作区对应文件的哈希，`sha256` 是 ASTRA 文件的哈希。运行环境、原始实验数据和绝对路径绑定需另行恢复；本次同步未重新构建、训练或运行 ROS/Gazebo。
