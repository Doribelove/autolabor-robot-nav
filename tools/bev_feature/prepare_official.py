#!/usr/bin/env python3
"""Download pinned official artifacts, preserving vendor license and NOTICE."""
import argparse
import hashlib
import json
import shutil
import time
import urllib.request
from pathlib import Path

REVISION = '1aada6677d1686efca1281a83b137c4592c267fe'
HASHES = {
    'nash_m/compile/model.hbm': '05443372d993714db0b226de6771a114876217fb02248c7bb1ec077a8b1af2e6',
    'float-checkpoint-best.pth.tar': '1709ce8271a67e0beaba135070a1ac6f3ab447eb389f542cebf060de11404fde',
}
COMPILER_HASHES = {
    'nash_m/compile/quantized_deploy.bc': '5cdbdba85f398d16b470eb02a6851b358fc252bd35e6fef7c8600112b671ff21',
    'nash_m/quantized_infer.bc': '800f24b7a4a7be0517b895d9b87ed277bcfc28c6504e84bfc8a7c518d5acc8fb',
    'nash_m/qat.bc': '78798bbb2f5a977a0d201fcf080381320bd8c1c83f90430b6c0345829ca9a794',
}

def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4*1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def download(url, partial):
    # Public pinned artifacts need no token. Keep TLS verification enabled, and
    # use the download endpoint to avoid stale cached CDN redirects.
    for attempt in range(3):
        try:
            with urllib.request.urlopen(url+'?download=true', timeout=30) as response, partial.open('wb') as stream:
                shutil.copyfileobj(response, stream, 4*1024*1024)
            return
        except OSError:
            if attempt == 2:
                raise
            time.sleep(attempt+1)

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', default='runtime/bev_feature_20261101/official')
    p.add_argument('--compiler-artifacts', action='store_true',
                   help='Also fetch pinned six-camera nash_m BC graphs for compiler validation')
    a = p.parse_args()
    root = Path(a.output)
    files = ['README.md', 'LICENSE', 'LICENSE.en', 'LICENSE.zh', 'NOTICE'] + list(HASHES)
    files += [name for name in COMPILER_HASHES if a.compiler_artifacts or (root/name).exists()]
    expected = dict(HASHES, **COMPILER_HASHES)
    manifest = dict(repository='OpenExplorer/bev_vtv2_resnet50', revision=REVISION, files={})
    for name in files:
        dest = root/name
        dest.parent.mkdir(parents=True, exist_ok=True)
        if not dest.exists():
            url = 'https://huggingface.co/OpenExplorer/bev_vtv2_resnet50/resolve/'+REVISION+'/'+name
            part = dest.with_name(dest.name+'.part')
            download(url, part)
            if name in expected and digest(part) != expected[name]:
                raise ValueError('Downloaded artifact SHA mismatch; partial file preserved: '+str(part))
            part.rename(dest)
        sha = digest(dest)
        if name in expected and sha != expected[name]:
            raise ValueError('Existing/downloaded artifact SHA mismatch: '+str(dest))
        manifest['files'][name] = dict(sha256=sha, bytes=dest.stat().st_size)
        print(name, sha, flush=True)
    temporary = root/'download_manifest.json.tmp'
    temporary.write_text(json.dumps(manifest, indent=2)+'\n')
    temporary.replace(root/'download_manifest.json')

if __name__ == '__main__':
    main()
