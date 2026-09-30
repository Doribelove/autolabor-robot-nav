#!/usr/bin/env python3
"""Verify an extracted positive-focus thesis update package."""

import hashlib
import json
from pathlib import Path

root = Path(__file__).resolve().parent
manifest = json.loads((root / "PACKAGE_MANIFEST.json").read_text())
for item in manifest["files"]:
    path = root / item["path"]
    assert path.is_file(), item["path"]
    assert path.stat().st_size == item["size_bytes"], item["path"]
    assert hashlib.sha256(path.read_bytes()).hexdigest() == item["sha256"], item["path"]
print(f"PASS: {len(manifest['files'])} files verified")
