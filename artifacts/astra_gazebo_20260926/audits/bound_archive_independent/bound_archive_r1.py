"""Content-addressed exact-byte archives with per-run immutable hard links.

No source file is edited. The shared store is for this study only; these checks
detect accidental mutation, not a hostile writer with the same Unix account.
"""
import hashlib
import os
from pathlib import Path
import re
import stat
import tempfile


def _read_regular(path):
    fd = os.open(str(path), os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    try:
        if not stat.S_ISREG(os.fstat(fd).st_mode):
            raise ValueError('archive input must be a regular file')
        with os.fdopen(fd, 'rb', closefd=False) as stream:
            return stream.read()
    finally:
        os.close(fd)


def _verify(path, digest):
    if hashlib.sha256(_read_regular(path)).hexdigest() != digest:
        raise ValueError('archive hash mismatch: '+str(path))


def _verify_archive(path, digest, source_identity, required_identity=None):
    _verify(path, digest)
    details = os.stat(str(path), follow_symlinks=False)
    identity = (details.st_dev, details.st_ino)
    if details.st_mode & 0o222:
        raise ValueError('existing archive has writable permission bits')
    if identity == source_identity:
        raise ValueError('archive must not alias its original source')
    if required_identity is not None and identity != required_identity:
        raise ValueError('existing run archive is not linked to shared content')
    return identity


def _sync_directory(path):
    descriptor = os.open(str(path), os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def archive(root, item, store):
    """Return a bound run-local hard link, never replace existing evidence."""
    root, store = Path(root), Path(store)
    digest = item['sha256']
    if not isinstance(digest, str) or re.fullmatch('[0-9a-f]{64}', digest) is None:
        raise ValueError('invalid content digest')
    # Sources may intentionally be system-library symlinks, but resolved source
    # bytes must still have the exact preregistered digest.
    source = Path(item['path']).resolve(strict=True)
    source_stat = source.stat()
    source_identity = (source_stat.st_dev, source_stat.st_ino)
    payload = _read_regular(source)
    if hashlib.sha256(payload).hexdigest() != digest:
        raise ValueError('bound resource changed: '+str(source))
    store.mkdir(parents=True, exist_ok=True)
    if store.is_symlink():
        raise ValueError('shared archive directory may not be a symlink')
    target = store/digest
    if not target.exists():
        descriptor, temporary_name = tempfile.mkstemp(prefix='.pending-', dir=str(store))
        temporary = Path(temporary_name)
        try:
            with os.fdopen(descriptor, 'wb') as stream:
                stream.write(payload); stream.flush(); os.fchmod(stream.fileno(), 0o444)
                os.fsync(stream.fileno())
            _verify(temporary, digest)
            try:
                os.link(str(temporary), str(target), follow_symlinks=False)
            except FileExistsError:
                pass
            _verify_archive(target, digest, source_identity)
            _sync_directory(store)
        finally:
            temporary.unlink()
    target_identity = _verify_archive(target, digest, source_identity)
    directory = root/'bound_blobs'
    directory.mkdir(exist_ok=True)
    if directory.is_symlink():
        raise ValueError('run archive directory may not be a symlink')
    local = directory/digest
    try:
        os.link(str(target), str(local), follow_symlinks=False)
    except FileExistsError:
        pass
    _verify_archive(local, digest, source_identity, target_identity)
    _sync_directory(directory)
    return dict(path=str(local.resolve()), sha256=digest, original_path=item['path'])
