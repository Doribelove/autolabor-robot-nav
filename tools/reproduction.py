#!/usr/bin/env python3
"""Verify/restore the published assets without starting ROS or hardware."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def inside(root, relative):
    path = (root / relative).resolve()
    path.relative_to(root.resolve())
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['verify', 'restore', 'environment'])
    parser.add_argument('--destination', type=Path, default=ROOT,
                        help='Restore into this workspace; existing differing files are preserved')
    args = parser.parse_args()
    if args.action == 'environment':
        paths = [
            '/opt/ros/noetic/setup.bash',
            '/home/slam/robot_ws/.deps/setup.bash',
            '/home/slam/robot_ws/.venv/fod_yolo/bin/python3',
            '/home/slam/robot_j6m_ws_navigation_20260907/devel/setup.bash',
            '/home/slam/robot_j6m_ws_navigation_20260907/devel/lib/fast_lio/fastlio_mapping',
            '/home/slam/j6m-dev-b/runtime/zed_link_20261019_6y_rm3_5/settings/SN23748636.conf',
            '/home/slam/j6m-dev-b/runtime/zed_link_20261019_6y_rm3_5/state/camera.lock',
            '/usr/local/zed/include/sl/Camera.hpp',
            '/usr/local/cuda/include/cuda.h', str(ROOT / 'devel/setup.bash')]
        print(json.dumps(dict(machine=platform.machine(), python=platform.python_version(),
            workspace=str(ROOT), expected_live_workspace='/home/slam/robot_j6m_ws_conbine',
            paths={p: Path(p).exists() for p in paths},
            commands={c: shutil.which(c) for c in ('bwrap', 'systemctl', 'roscore', 'catkin_make', 'ffmpeg')},
            note='Read-only inventory, not a live readiness or board compatibility check'), indent=2))
        return 0
    bundle = ROOT / 'reproduction'
    rows = json.loads((bundle / 'manifest.json').read_text())['files']
    errors = []
    pending = []
    for row in rows:
        source = inside(bundle, row['source'])
        if not source.is_file() or source.stat().st_size != row['bytes'] or digest(source) != row['sha256']:
            errors.append('Snapshot checksum mismatch: ' + row['source'])
            continue
        if args.action == 'restore':
            target = inside(args.destination, row['destination'])
            if target.exists():
                if not target.is_file() or digest(target) != row['sha256']:
                    errors.append('Existing file differs; preserved: ' + str(target))
            else:
                pending.append((source, target))
    if errors:
        print('\n'.join(errors), file=sys.stderr)
        return 1
    for source, target in pending:
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open('xb') as output, source.open('rb') as data:
            shutil.copyfileobj(data, output)
        shutil.copymode(source, target)
    print('%s: %d verified files; %d restored; no services started' %
          (args.action, len(rows), len(pending)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
