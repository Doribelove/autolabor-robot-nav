#!/usr/bin/env python3
"""Prepare a local transfer bundle for the owner's Intel/AMD computer."""
import argparse
import io
import json
from pathlib import Path
import tarfile
import time

from oe391_environment import sha256
from prepare_official import COMPILER_HASHES

ROOT = Path(__file__).resolve().parents[2]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--sdk', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    bc = 'nash_m/compile/quantized_deploy.bc'
    official = ROOT/'runtime/bev_feature_20261101/official'
    if sha256(official/bc) != COMPILER_HASHES[bc]:
        raise ValueError('Official BC digest mismatch')
    files = {}
    for pattern in ('hbdk4_compiler-4.11.11-cp310-*', 'hbdk4_march-4.11.11-cp310-*'):
        path, = (a.sdk/'package/host/ai_toolchain').glob(pattern)
        files['wheels/'+path.name] = path
    for name in ('oe391_environment.py', 'compile_reference.py', 'prepare_official.py',
                 'x86_setup.sh', 'x86_compile.sh'):
        files['tools/bev_feature/'+name] = ROOT/'tools/bev_feature'/name
    files['run_docker.sh'] = a.sdk/'run_docker.sh'
    files['README.txt'] = ROOT/'docs/X86_WINDOWS_SETUP_20261101.txt'
    files['runtime/bev_feature_20261101/official/'+bc] = official/bc
    for name in ('README.md', 'LICENSE', 'LICENSE.en', 'LICENSE.zh', 'NOTICE'):
        files['runtime/bev_feature_20261101/official/'+name] = official/name
    manifest = {name:dict(bytes=path.stat().st_size, sha256=sha256(path)) for name,path in files.items()}
    sums = ''.join(row['sha256']+'  '+name+'\n' for name,row in sorted(manifest.items())).encode()
    a.output.parent.mkdir(parents=True, exist_ok=True)
    # Wheels/BC are already compressed; tar avoids a costly second compression.
    with a.output.open('xb') as stream, tarfile.open(fileobj=stream, mode='w') as archive:
        for name,path in files.items():
            archive.add(path, arcname='bev_x86_compiler/'+name, recursive=False)
        info = tarfile.TarInfo('bev_x86_compiler/SHA256SUMS')
        info.size, info.mtime = len(sums), int(time.time())
        archive.addfile(info, io.BytesIO(sums))
    result = dict(file=str(a.output.resolve()), bytes=a.output.stat().st_size,
                  sha256=sha256(a.output), files=manifest, native_x86_executed=False)
    a.output.with_suffix('.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='files'}, indent=2))


if __name__ == '__main__':
    main()
