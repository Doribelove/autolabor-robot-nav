#!/usr/bin/env python3
"""Export an allowlisted standalone source snapshot; never copy Git history."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess

SOURCE = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path, help='New, nonexistent directory')
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        parser.error('Output already exists; use a new directory to preserve earlier snapshots')
    if SOURCE == output or output in SOURCE.parents:
        parser.error('Output must not contain the source workspace')
    config = json.loads((SOURCE/'distribution/coverage_navigation/manifest.json').read_text())
    selected = set(config['files'])
    for directory in config['directories']:
        paths = subprocess.check_output(
            ['git','ls-files','-z','--cached','--others','--exclude-standard','--',directory],
            cwd=SOURCE).decode().split('\0')
        selected.update(p for p in paths if p)
    templates = SOURCE/config['templates']
    copies = [(SOURCE/p, Path(p)) for p in sorted(selected)]
    copies += [(p,p.relative_to(templates)) for p in sorted(templates.rglob('*')) if p.is_file()]
    records = []
    destinations = set()
    # Check the complete manifest before writing the export.
    for source, relative in copies:
        if source.is_symlink() or not source.is_file():
            raise ValueError('Missing or symlink source: '+str(source.relative_to(SOURCE)))
        if relative in destinations:
            raise ValueError('Duplicate export destination: '+str(relative))
        destinations.add(relative)
        if any(p in {'.git','runtime','validation','build','devel','install','__pycache__'} for p in relative.parts):
            raise ValueError('Runtime/history path in export: '+str(relative))
        if source.suffix.lower() in {'.bag','.pcd','.pt','.onnx','.bin','.env','.pem','.key','.pyc'}:
            raise ValueError('Private or binary asset in export: '+str(relative))
        data = source.read_bytes()
        gif_demo = relative.parts[:1] == ('assets',) and source.suffix.lower() == '.gif'
        size_limit = 30*1024*1024 if gif_demo else 5*1024*1024
        if len(data)>size_limit:
            raise ValueError('Unexpected large file: '+str(relative))
        if re.search(rb'-----BEGIN (?:RSA |OPENSSH |EC )?PRIVATE KEY-----|gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{30,}',data):
            raise ValueError('Credential pattern in export: '+str(relative))
        records.append(dict(path=str(relative),sha256=hashlib.sha256(data).hexdigest(),bytes=len(data)))
    output.mkdir(parents=True)
    for source,relative in copies:
        target=output/relative
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(source,target)
    manifest=dict(source_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=SOURCE,text=True).strip(),
                  note='Snapshot of working files; per-file hashes identify changes not yet in source_commit. No source Git history included.',
                  files=records)
    (output/'SOURCE_MANIFEST.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(dict(output=str(output),files=len(records),bytes=sum(x['bytes'] for x in records))))


if __name__ == '__main__':
    main()
