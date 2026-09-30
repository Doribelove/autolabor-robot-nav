"""Exact failed-run and independent GLX maps replay, plus narrow-policy faults."""
import copy
import json
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import mapped_elf_inventory as base
import mapped_elf_inventory_v2 as v2
import passive_sweep_q04 as q


def line(address,path,perms='r-xp',size=8192,inode=1,device='00:01',offset=0):
    return f'{address:x}-{address+size:x} {perms} {offset:08x} {device} {inode} {path}'


def synthetic():
    lines=[line(0x10000,'/lib/libneeded.so',inode=10)]
    for i,item in enumerate(v2.GRAPHICS_BINDINGS.values()):lines.append(line(0x20000+i*0x10000,item['path'],inode=20+i))
    lines.extend([line(0x60000,v2.GRAPHICS_MEMFD+' (deleted)','r-xs',inode=1238),
      line(0x70000,v2.GRAPHICS_MEMFD+' (deleted)','rw-s',inode=1238)])
    return lines


def inspect(path,row):
    match=next((b for b in v2.GRAPHICS_BINDINGS.values() if b['path']==path),None)
    return {'path':path,'sha256':match['sha256'] if match else '0'*64,'build_id':'b'*40,
      'device_major':row['device_major'],'device_minor':row['device_minor'],'inode':row['inode'],'roles':[]}


def test_narrow_alias_kept_as_external_code_not_elf_or_memory_hash():
    result=v2.inventory_from_text('\n'.join(synthetic()),{'libneeded.so':'physics'},{'physics'},inspect)
    code=result['external_graphics_dynamic_code'][0]
    assert not code['static_ELF_verified'] and not code['executable_memory_bytes_read_or_hashed']
    assert len(code['mapping_rows'])==2 and len(code['render_driver_bindings'])==3
    assert {r for item in result['loaded_modules'] for r in item['roles']}=={'physics'}|set(v2.GRAPHICS_ROLES.values())
    assert len(result['executable_mappings'])==5


@pytest.mark.parametrize('mutation',['missing_rw','missing_rx','rwx','private_rx','different_inode','wrong_size',
  'wrong_offset','wrong_device','wrong_name','missing_driver','second_alias_group','undeleted'])
def test_alias_policy_boundary_rejections(mutation):
    rows=synthetic()
    if mutation=='missing_rw':rows.pop()
    elif mutation=='missing_rx':del rows[-2]
    elif mutation=='rwx':rows[-2]=rows[-2].replace('r-xs','rwxs')
    elif mutation=='private_rx':rows[-2]=rows[-2].replace('r-xs','r-xp')
    elif mutation=='different_inode':rows[-1]=rows[-1].replace(' 1238 ',' 1239 ')
    elif mutation=='wrong_size':rows[-1]=rows[-1].replace('70000-72000','70000-73000')
    elif mutation=='wrong_offset':rows[-1]=rows[-1].replace('00000000','00001000')
    elif mutation=='wrong_device':rows[-1]=rows[-1].replace('00:01','00:02')
    elif mutation=='wrong_name':rows[-2]=rows[-2].replace('/.glXXXXXX','/.other')
    elif mutation=='missing_driver':del rows[1]
    elif mutation=='second_alias_group':rows.append(line(0x80000,v2.GRAPHICS_MEMFD+' (deleted)','r-xs',inode=555))
    else:rows[-2]=rows[-2].replace(' (deleted)','')
    with pytest.raises(base.InventoryError):v2.inventory_from_text('\n'.join(rows),{'libneeded.so':'physics'},{'physics'},inspect)


@pytest.mark.parametrize('mutation',['driver_digest','driver_path','deleted_required','unrelated_deleted_exec','unknown_astra'])
def test_code_and_driver_identity_still_fail_closed(mutation):
    rows=synthetic()
    def altered(path,row):
        result=inspect(path,row)
        if 'glcore' in path:
            if mutation=='driver_digest':result['sha256']='f'*64
            if mutation=='driver_path':result['path']='/tmp/copied-driver'
        return result
    if mutation=='deleted_required':rows[0]+=' (deleted)'
    if mutation=='unrelated_deleted_exec':rows.append(line(0x80000,'/memfd:arbitrary (deleted)','r-xs'))
    if mutation=='unknown_astra':rows.append(line(0x80000,'/lib/libastra_unknown.so'))
    with pytest.raises(base.InventoryError):v2.inventory_from_text('\n'.join(rows),{'libneeded.so':'physics'},{'physics'},altered)


def test_actual_q03_failed_maps_replay_all_physics_roles_and_exact_graphics_files():
    path=q.STAGE/'runs/passive_sweep_q03_static/elf_pre.maps.txt'
    text=path.read_text();required=set(q.ELF_ROLES.values())-{'actor_binaries'}
    with pytest.raises(base.InventoryError):base.inventory_from_text(text,q.ELF_ROLES,required)
    result=v2.inventory_from_text(text,q.ELF_ROLES,required)
    assert len(result['external_graphics_dynamic_code'])==1
    assert len(result['loaded_modules'])==len(required)
    assert all(item['sha256']==q.sha(item['path']) for item in result['loaded_modules'])


def test_independent_actual_glx_before_after_replay_and_renderer_evidence():
    root=q.STAGE/'audits/mapped_graphics_probe_v1';report=json.loads((root/'report.json').read_text())
    assert report['success'] and report['GL']['vendor']=='NVIDIA Corporation'
    for label,expected in [('03_after_choose_FBConfig',0),('04_after_create_context',0),('05_after_make_current',1),('07_after_cleanup',0)]:
        text=(root/(label+'.maps.txt')).read_text()
        result=v2.inventory_from_text(text,{},set())
        assert len(result['external_graphics_dynamic_code'])==expected


def test_v2_failure_still_preserves_raw_before_graphics_policy(tmp_path):
    proc=tmp_path/'proc'/'123';proc.mkdir(parents=True)
    payload='\n'.join(synthetic()[:-1])
    (proc/'maps').write_text(payload);(proc/'stat').write_text('123 (fake) '+' '.join(['1']*20))
    raw=tmp_path/'raw.maps.txt'
    with pytest.raises(base.InventoryError):v2.inventory(123,{},set(),raw,proc.parent)
    assert raw.read_text()==payload
    audit=json.loads(Path(str(raw)+'.audit.json').read_text())
    assert not audit['interpretation_pass'] and 'exactly one RX' in audit['error']
