#!/usr/bin/env python3
"""Export a reviewable J6M/BPU source draft and checksummed tarball."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import tarfile

SOURCE = Path(__file__).resolve().parents[1]
EXCLUDED_PARTS = {'.git', 'build', 'devel', 'install', 'runtime', 'validation', 'log',
                  'logs', 'models', 'vendor', 'exports', 'results', 'inputs', '__pycache__',
                  '.pytest_cache', 'PCD', 'Log'}
SOURCE_SUFFIXES = {'.py', '.cpp', '.cc', '.c', '.h', '.hpp', '.hh', '.cmake', '.xml',
                   '.launch', '.yaml', '.yml', '.json', '.md', '.rst', '.txt', '.sh',
                   '.msg', '.srv', '.action', '.rviz'}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def selected_files():
    config = json.loads((SOURCE / 'distribution/j6m_bpu_navigation/export_manifest.json').read_text())
    selected = set(config['files'])
    for directory in config['directories']:
        root = SOURCE / directory
        if root.is_symlink() or not root.is_dir():
            raise ValueError('Missing regular source directory: ' + directory)
        for path in root.rglob('*'):
            relative = path.relative_to(SOURCE)
            if any(p in EXCLUDED_PARTS for p in relative.parts):
                continue
            if path.is_symlink():
                raise ValueError('Symlink in source snapshot: ' + str(relative))
            if path.is_file() and (path.suffix in SOURCE_SUFFIXES or path.name in
                                   {'CMakeLists.txt', 'LICENSE', 'LICENSE.txt', 'NOTICE'}):
                selected.add(str(relative))
    return sorted(selected)


def export(output):
    output = output.absolute()
    archive = output.with_name(output.name + '.tar.gz')
    checksum = archive.with_name(archive.name + '.sha256')
    if any(p.exists() or p.is_symlink() for p in (output, archive, checksum)):
        raise FileExistsError('Use a new output name; earlier artifacts are preserved')
    if output.resolve() == SOURCE or output.resolve() in SOURCE.parents:
        raise ValueError('Output must not contain the source workspace')
    records, copies = [], []
    for relative in selected_files():
        path = SOURCE / relative
        if (path.is_symlink() or not path.is_file() or SOURCE not in path.resolve().parents
                or any(p in EXCLUDED_PARTS for p in Path(relative).parts)):
            raise ValueError('Non-source or escaped file in export: ' + relative)
        data = path.read_bytes()
        if len(data) > 5 * 1024 * 1024 or b'\x00' in data:
            raise ValueError('Unexpected binary/large source: ' + relative)
        if re.search(rb'-----BEGIN (?:RSA |OPENSSH |EC )?PRIVATE KEY-----|gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{30,}|sk-[A-Za-z0-9_-]{30,}', data):
            raise ValueError('Credential pattern in export: ' + relative)
        records.append({'path': relative, 'sha256': digest(data), 'bytes': len(data)})
        copies.append((path, relative, data))
    output.mkdir(parents=True)
    for path, relative, data in copies:
        target = output / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        shutil.copymode(path, target)
    readme = (output / 'distribution/j6m_bpu_navigation/README.md').read_bytes()
    (output / 'README.md').write_bytes(readme)
    records.append({'path': 'README.md', 'sha256': digest(readme), 'bytes': len(readme)})
    metadata = {
        'source_branch': subprocess.check_output(['git', 'branch', '--show-current'], cwd=SOURCE, text=True).strip(),
        'source_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=SOURCE, text=True).strip(),
        'snapshot': 'Current working files including uncommitted changes; identified by per-file SHA256.',
        'production_bpu_ready': False, 'submitted_to_horizon': False, 'files': records}
    (output / 'SOURCE_MANIFEST.json').write_text(json.dumps(metadata, indent=2) + '\n')
    with tarfile.open(archive, 'x:gz') as stream:
        stream.add(output, arcname=output.name, recursive=True)
    value = hashlib.sha256()
    with archive.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            value.update(block)
    checksum.write_text(value.hexdigest() + '  ' + archive.name + '\n')
    print(json.dumps({'archive': str(archive), 'sha256': value.hexdigest(),
                      'source_files': len(records), 'submitted': False}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path, help='A new directory, usually runtime/<draft-name>')
    export(parser.parse_args().output)
