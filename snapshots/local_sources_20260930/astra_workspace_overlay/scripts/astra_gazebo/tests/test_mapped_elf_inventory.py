"""Independent Linux-map/parser/lifetime fixtures, no ROS or simulator."""
import json
import mmap
import os
from pathlib import Path
import sys
import tempfile

import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import mapped_elf_inventory as inv


ROLES={'libneeded.so':'engine'}


def row(path,perms='r-xp',start=0x1000,inode=10):
    return f'{start:x}-{start+0x1000:x} {perms} 00000000 08:01 {inode} {path}'


def fake_inspector(path,item):
    return {'path':path,'sha256':'a'*64,'build_id':'b'*40,'device_major':item['device_major'],
      'device_minor':item['device_minor'],'inode':item['inode'],'roles':[]}


def test_spaces_and_pathless_mappings_keep_evidence():
    text=row('/path with spaces/libneeded.so')+'\n'+row('',perms='rw-p',start=0x2000,inode=0)
    rows=inv.parse_maps(text)
    assert rows[0]['backing_path']=='/path with spaces/libneeded.so'
    assert rows[1]['backing_path']==''
    assert rows[0]['raw_line']==text.splitlines()[0]


@pytest.mark.parametrize('bad',['garbage','1000-1000 r-xp 00 08:01 1 /x','2000-1000 r-xp 00 08:01 1 /x',
    '1000-2000 rw-z 00 08:01 1 /x',row('/x')+'\n'+row('/y')])
def test_malformed_or_overlapping_maps_rejected(bad):
    with pytest.raises(inv.InventoryError):inv.parse_maps(bad)


@pytest.mark.parametrize('path',['/dev/shm/deleted-data','/run/shm/space name','/memfd:shared','/SYSV00000001'])
def test_nonexecutable_unlinked_shared_data_does_not_claim_elf(path):
    text=row('/lib/libneeded.so')+'\n'+row(path+' (deleted)',perms='rw-s',start=0x2000)
    result=inv.inventory_from_text(text,ROLES,{'engine'},fake_inspector)
    assert result['loaded_modules'][0]['roles']==['engine']
    data=result['excluded_unlinked_shared_data'][0]
    assert data['deleted'] and not data['executable']
    assert 'not asserted' in data['classification']['basis']


@pytest.mark.parametrize('path,perms',[('/dev/shm/x','r-xs'),('/tmp/x','rw-p'),('/lib/libneeded.so','r--p'),
 ('/dev/shm/libneeded.so','rw-s'),('/dev/shm/libunknown.so.1','rw-s'),('/dev/shm/libastra_hidden','rw-s')])
def test_deleted_code_role_or_unknown_file_fails_closed(path,perms):
    with pytest.raises(inv.InventoryError):
        inv.inventory_from_text(row('/lib/libneeded.so')+'\n'+row(path+' (deleted)',perms,start=0x2000),ROLES,{'engine'},fake_inspector)


def test_required_role_missing_and_nonexecuting_role_cannot_pass():
    for text in (row('/dev/shm/x (deleted)','rw-s'),row('/lib/libneeded.so','r--p')):
        with pytest.raises(inv.InventoryError):inv.inventory_from_text(text,ROLES,{'engine'},fake_inspector)


def test_unknown_astra_plugin_rejected_even_without_execute_segment():
    with pytest.raises(inv.InventoryError):inv.inventory_from_text(row('/lib/libastra_unknown.so','r--p'),ROLES,set(),fake_inspector)


def test_same_library_segments_hashed_once_distinct_inode_rejected():
    calls=[]
    def inspect(path,item):calls.append(path);return fake_inspector(path,item)
    text=row('/lib/libneeded.so')+'\n'+row('/lib/libneeded.so','r--p',0x2000)
    assert len(inv.inventory_from_text(text,ROLES,{'engine'},inspect)['loaded_modules'])==1
    assert calls==['/lib/libneeded.so']
    with pytest.raises(inv.InventoryError):
        inv.inventory_from_text(row('/one/libneeded.so')+'\n'+row('/two/libneeded.so','r-xp',0x2000,11),ROLES,{'engine'},inspect)


def test_durable_raw_and_failure_audit_exist_before_policy_failure(tmp_path):
    process=tmp_path/'proc'/'123';process.mkdir(parents=True)
    payload=(row('/dev/shm/unknown (deleted)','r-xs')+'\n').encode()
    (process/'maps').write_bytes(payload);(process/'stat').write_text('123 (fixture) '+' '.join(['0']*20))
    raw=tmp_path/'maps.txt'
    with pytest.raises(inv.InventoryError):inv.inventory(123,ROLES,{'engine'},raw,process.parent)
    assert raw.read_bytes()==payload
    report=json.loads(Path(str(raw)+'.audit.json').read_text())
    assert not report['interpretation_pass'] and 'deleted executable' in report['error']
    assert report['raw_maps']['sha256']==__import__('hashlib').sha256(payload).hexdigest()
    with pytest.raises(FileExistsError):inv.inventory(123,ROLES,{'engine'},raw,process.parent)
    assert raw.read_bytes()==payload


def test_raw_retained_when_process_disappears_after_maps_read(tmp_path):
    process=tmp_path/'proc'/'123';process.mkdir(parents=True)
    (process/'maps').write_text(row('/x'))
    raw=tmp_path/'maps.txt'
    with pytest.raises(FileNotFoundError):inv.inventory(123,ROLES,{'engine'},raw,process.parent)
    assert raw.exists() and Path(str(raw)+'.audit.json').exists()


def test_real_unlinked_shm_and_current_interpreter_elf(tmp_path):
    """Real mmap/unlink, not just a mirror of parser-generated classification."""
    exe=Path('/proc/self/exe').resolve();roles={exe.name:'self_executable'}
    descriptor,name=tempfile.mkstemp(prefix='astra_inventory_test_',dir='/dev/shm')
    region=None
    try:
        os.ftruncate(descriptor,4096);region=mmap.mmap(descriptor,4096)
        region[:4]=b'DATA';os.unlink(name)
        result=inv.inventory(os.getpid(),roles,{'self_executable'},tmp_path/'actual.maps.txt')
        assert result['interpretation_pass']
        assert any(r['backing_path']==name for r in result['excluded_unlinked_shared_data'])
        actual=result['loaded_modules'][0]
        assert actual['sha256']==__import__('hashlib').sha256(exe.read_bytes()).hexdigest()
        assert actual['inode']==exe.stat().st_ino and actual['build_id']
    finally:
        if region is not None:region.close()
        os.close(descriptor)
        if Path(name).exists():os.unlink(name)


def test_mapped_inode_substitution_and_non_elf_role_fail(tmp_path):
    exe=Path('/proc/self/exe').resolve();stat=exe.stat()
    item={'device_major':os.major(stat.st_dev),'device_minor':os.minor(stat.st_dev),'inode':stat.st_ino+1,'backing_path':str(exe)}
    with pytest.raises(inv.InventoryError,match='device/inode'):inv.inspect_elf(exe,item)
    plain=tmp_path/'not-elf';plain.write_bytes(b'TEXT');stat=plain.stat()
    item.update(device_major=os.major(stat.st_dev),device_minor=os.minor(stat.st_dev),inode=stat.st_ino,backing_path=str(plain))
    with pytest.raises(inv.InventoryError,match='not an ELF'):inv.inspect_elf(plain,item)


def test_elf_lacks_build_id_is_rejected(tmp_path):
    import struct
    # Minimal 64-bit ELF header; external readelf is the independent parser.
    raw=b'\x7fELF'+bytes([2,1,1])+bytes(9)+struct.pack('<HHIQQQIHHHHHH',3,62,1,0,0,0,0,64,56,0,64,0,0)
    path=tmp_path/'empty.elf';path.write_bytes(raw);s=path.stat()
    item={'device_major':os.major(s.st_dev),'device_minor':os.minor(s.st_dev),'inode':s.st_ino,'backing_path':str(path)}
    with pytest.raises(inv.InventoryError,match='build-id'):inv.inspect_elf(path,item)


@pytest.mark.parametrize('after',[{'pid':321,'proc_start_ticks':'77'},{'pid':123,'proc_start_ticks':'78'},
  {'pid':123,'proc_start_ticks':77},{'pid':True,'proc_start_ticks':'77'},{}])
def test_pre_post_same_process_rejects_pid_reuse_missing_or_type_coercion(after):
    before={'pid':123,'proc_start_ticks':'77'}
    assert inv.same_process(before,before)
    assert not inv.same_process(before,after)
