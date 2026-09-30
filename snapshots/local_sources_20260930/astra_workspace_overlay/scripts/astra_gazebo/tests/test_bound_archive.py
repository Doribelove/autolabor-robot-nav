import hashlib
import os
from pathlib import Path
import sys
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bound_archive import archive


def fixture(tmp_path, data=b'exact immutable bytes'):
    source = tmp_path/'source'; source.write_bytes(data)
    first = tmp_path/'first'; first.mkdir()
    second = tmp_path/'second'; second.mkdir()
    item = dict(path=str(source), sha256=hashlib.sha256(data).hexdigest())
    return source, first, second, item, tmp_path/'store'


def test_two_runs_share_exact_inode_and_source_remains_independent(tmp_path):
    source, first, second, item, store = fixture(tmp_path)
    a = Path(archive(first, item, store)['path'])
    b = Path(archive(second, item, store)['path'])
    assert a.stat().st_ino == b.stat().st_ino == (store/item['sha256']).stat().st_ino
    assert source.stat().st_ino != a.stat().st_ino
    assert a.stat().st_mode & 0o222 == 0
    assert a.read_bytes() == source.read_bytes()
    assert archive(first, item, store)['sha256'] == item['sha256']


def test_changed_source_not_archived(tmp_path):
    source, first, _, item, store = fixture(tmp_path)
    source.write_bytes(b'changed')
    with pytest.raises(ValueError, match='bound resource changed'):
        archive(first, item, store)
    assert not store.exists()


@pytest.mark.parametrize('where', ['global', 'local'])
def test_existing_corrupt_evidence_not_overwritten(tmp_path, where):
    _, first, _, item, store = fixture(tmp_path)
    directory = store if where == 'global' else first/'bound_blobs'
    directory.mkdir(); bad = directory/item['sha256']; bad.write_bytes(b'bad')
    with pytest.raises(ValueError, match='archive hash mismatch'):
        archive(first, item, store)
    assert bad.read_bytes() == b'bad'


@pytest.mark.parametrize('where', ['global', 'local'])
def test_archive_symlink_rejected_without_mutating_target(tmp_path, where):
    source, first, _, item, store = fixture(tmp_path)
    directory = store if where == 'global' else first/'bound_blobs'
    directory.mkdir(); (directory/item['sha256']).symlink_to(source)
    with pytest.raises(OSError):
        archive(first, item, store)
    assert source.read_bytes() == b'exact immutable bytes'


def test_bad_digest_cannot_escape_store(tmp_path):
    _, first, _, item, store = fixture(tmp_path)
    item['sha256'] = '../escape'
    with pytest.raises(ValueError, match='invalid content digest'):
        archive(first, item, store)
    assert not store.exists()


@pytest.mark.parametrize('readonly', [False, True])
def test_source_hardlink_preexisting_in_store_is_rejected(tmp_path, readonly):
    source, first, _, item, store = fixture(tmp_path)
    store.mkdir(); target = store/item['sha256']; os.link(source, target)
    if readonly: source.chmod(0o444)
    before_mode = source.stat().st_mode
    with pytest.raises(ValueError, match='writable|alias'):
        archive(first, item, store)
    assert source.stat().st_mode == before_mode
    assert not (first/'bound_blobs').exists()


def test_existing_writable_correct_copy_is_rejected_unchanged(tmp_path):
    source, first, _, item, store = fixture(tmp_path)
    store.mkdir(); target = store/item['sha256']; target.write_bytes(source.read_bytes())
    before = target.stat().st_mode
    with pytest.raises(ValueError, match='writable'):
        archive(first, item, store)
    assert target.stat().st_mode == before


def test_existing_local_duplicate_copy_rejected_without_replacing(tmp_path):
    source, first, _, item, store = fixture(tmp_path)
    directory=first/'bound_blobs'; directory.mkdir()
    local=directory/item['sha256']; local.write_bytes(source.read_bytes()); local.chmod(0o444)
    before=local.stat().st_ino
    with pytest.raises(ValueError, match='not linked'):
        archive(first, item, store)
    assert local.stat().st_ino == before


def test_fifo_source_rejected_without_waiting_for_writer(tmp_path):
    source, first, _, item, store = fixture(tmp_path)
    source.unlink(); os.mkfifo(source)
    with pytest.raises(ValueError, match='regular file'):
        archive(first, item, store)
