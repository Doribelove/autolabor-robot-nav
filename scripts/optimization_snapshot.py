#!/usr/bin/env python3
"""Read-only content inventory of the authoritative sources; write only to this candidate."""
import hashlib
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
ORIGINAL = Path('/home/slam/robot_j6m_ws')
AREAS = ('src', 'scripts', 'config', 'deploy', 'manifest', 'global_maps')


def inventory():
    records = {}
    for area in AREAS:
        for parent, dirs, files in os.walk(str(ORIGINAL / area), followlinks=False):
            dirs[:] = sorted(d for d in dirs if d not in ('.git', '__pycache__'))
            for name in sorted(files + [d for d in dirs if (Path(parent) / d).is_symlink()]):
                path = Path(parent) / name
                if name.endswith('.pyc'):
                    continue
                key = str(path.relative_to(ORIGINAL))
                if path.is_symlink():
                    records[key] = {'link': os.readlink(str(path))}
                elif path.is_file():
                    digest = hashlib.sha256()
                    with path.open('rb') as stream:
                        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
                            digest.update(block)
                    records[key] = {'sha256': digest.hexdigest(), 'size': path.stat().st_size,
                                    'mode': path.stat().st_mode & 0o7777}
    return records


def main():
    destination = ROOT / 'validation' / 'original_inventory.json'
    current = inventory()
    if '--verify' in sys.argv:
        expected = json.loads(destination.read_text())
        changed = sorted(k for k in set(expected) | set(current) if expected.get(k) != current.get(k))
        result = {'files': len(current), 'changed': changed, 'unchanged': not changed}
        (destination.parent / 'original_integrity_final.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result))
        return int(bool(changed))
    destination.parent.mkdir(exist_ok=True)
    with destination.open('x') as stream:
        json.dump(current, stream, indent=2, sort_keys=True)
        stream.write('\n')
    print('Recorded {} original files and links'.format(len(current)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
