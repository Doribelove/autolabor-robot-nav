"""Read actual Linux mappings, preserving raw evidence before interpretation.

Only reviewed role libraries are hashed. Mapping permissions and backing-file
identity, not every absolute pathname, distinguish code from shared data.
No process memory or environment is read. This is trusted-run provenance, not
protection against malicious processes or a claim that mapped pages equal disk.
"""
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time


class InventoryError(ValueError):
    pass


def same_process(before,after):
    """PID plus kernel process start ticks prevent PID-reuse equivalence."""
    return (type(before.get('pid'))is int and before['pid']>0 and type(after.get('pid'))is int and
      before['pid']==after['pid'] and type(before.get('proc_start_ticks'))is str and
      before['proc_start_ticks'].isdigit() and int(before['proc_start_ticks'])>0 and
      before['proc_start_ticks']==after.get('proc_start_ticks'))


def _exclusive_bytes(path,payload):
    path=Path(path)
    with path.open('xb') as stream:
        stream.write(payload);stream.flush();os.fsync(stream.fileno())
    directory=os.open(str(path.parent),os.O_RDONLY|os.O_DIRECTORY)
    try:os.fsync(directory)
    finally:os.close(directory)


def parse_maps(text):
    """Parse every mapping, retaining spaces, original line and deleted suffix."""
    rows=[];last_end=0
    for number,line in enumerate(text.splitlines(),1):
        match=re.fullmatch(r'([0-9a-f]+)-([0-9a-f]+) ([r-][w-][x-][ps]) ([0-9a-f]+) ([0-9a-f]+):([0-9a-f]+) +([0-9]+)(?: +(.*))?',line)
        if not match:raise InventoryError('malformed maps row '+str(number))
        lo,hi,perms,offset,major,minor,inode,name=match.groups()
        start,end=int(lo,16),int(hi,16)
        if start>=end or start<last_end:raise InventoryError('overlapping/reversed maps row '+str(number))
        last_end=end;name=name or '';deleted=name.endswith(' (deleted)')
        backing=name[:-10] if deleted else name
        rows.append({'line_number':number,'raw_line':line,'start':start,'end':end,'permissions':perms,
          'offset':int(offset,16),'device_major':int(major,16),'device_minor':int(minor,16),'inode':int(inode),
          'mapped_path':name,'backing_path':backing,'deleted':deleted,'executable':'x' in perms})
    if not rows:raise InventoryError('empty process maps')
    return rows


def classify_mapping(row,roles):
    """Return role or explicit exclusion; deleted code always fails closed.

    Unlinked non-executable shm is classified as DATA by path+permissions, not
    asserted to be inspected ELF bytes. That narrow exclusion cannot satisfy
    any required role; a matching reviewed or unreviewed Astra name is checked
    first. Unknown deleted files remain failures, preserving diagnostic raw.
    """
    path=row['backing_path'];name=Path(path).name
    matched=[role for stem,role in roles.items() if name==stem or name.startswith(stem+'.')]
    if len(matched)>1:raise InventoryError('ambiguous ELF role: '+name)
    if row['deleted'] and (row['executable'] or matched or name.startswith('libastra_') or '.so' in name):
        raise InventoryError('deleted executable or role/ELF mapping: '+row['mapped_path'])
    if name.startswith('libastra_') and not matched:raise InventoryError('unreviewed custom Astra plugin mapped: '+name)
    if row['deleted']:
        shared_data=(path.startswith('/dev/shm/') or path.startswith('/run/shm/') or
                     path.startswith('/memfd:') or path.startswith('memfd:') or path.startswith('/SYSV'))
        if shared_data and not row['executable']:
            return {'kind':'excluded_unlinked_nonexecuting_shared_data','role':None,
              'basis':'maps pathname and non-executable permissions; backing bytes not asserted to be ELF or inspected'}
        raise InventoryError('unclassified deleted mapping: '+row['mapped_path'])
    if matched:
        if not path.startswith('/') or row['inode']<=0:raise InventoryError('role has no ordinary file backing: '+path)
        return {'kind':'reviewed_role_candidate','role':matched[0]}
    return {'kind':'other_executable_mapping' if row['executable'] else 'other_mapping','role':None}


def _identity(stat):
    return (stat.st_dev,stat.st_ino,stat.st_size,stat.st_mtime_ns,stat.st_ctime_ns)


def inspect_elf(path,row):
    """Hash and read BuildID from one fd, compare mapped device+inode first."""
    path=Path(path).resolve(strict=True)
    descriptor=os.open(str(path),os.O_RDONLY|os.O_CLOEXEC)
    try:
        before=os.fstat(descriptor)
        if (os.major(before.st_dev),os.minor(before.st_dev),before.st_ino)!=(row['device_major'],row['device_minor'],row['inode']):
            raise InventoryError('mapped device/inode differs from opened file: '+str(path))
        if os.read(descriptor,4)!=b'\x7fELF':raise InventoryError('reviewed role is not an ELF: '+str(path))
        os.lseek(descriptor,0,os.SEEK_SET);digest=hashlib.sha256()
        while True:
            block=os.read(descriptor,1024*1024)
            if not block:break
            digest.update(block)
        notes=subprocess.check_output(['readelf','-n','/proc/self/fd/'+str(descriptor)],pass_fds=(descriptor,),text=True,timeout=10)
        found=re.search(r'Build ID:\s*([0-9a-fA-F]+)',notes)
        if not found:raise InventoryError('mapped target ELF lacks build-id: '+str(path))
        if _identity(before)!=_identity(os.fstat(descriptor)):raise InventoryError('mapped file changed during inspection: '+str(path))
        return {'path':str(path),'sha256':digest.hexdigest(),'build_id':found.group(1),
          'mapped_path':row['backing_path'],'device_major':row['device_major'],'device_minor':row['device_minor'],
          'inode':row['inode'],'file_size':before.st_size,'roles':[]}
    finally:os.close(descriptor)


def inventory_from_text(text,roles,required_roles,inspector=inspect_elf):
    rows=parse_maps(text);selected={};excluded=[];executable=[];seen={};executable_roles=set()
    for row in rows:
        classification=classify_mapping(row,roles)
        if classification['kind']=='excluded_unlinked_nonexecuting_shared_data':
            excluded.append(dict(row,classification=classification));continue
        if row['executable']:executable.append(row)
        role=classification['role']
        if role is None:continue
        if row['executable']:executable_roles.add(role)
        key=(row['device_major'],row['device_minor'],row['inode'],row['backing_path'])
        if key not in seen:seen[key]=inspector(row['backing_path'],row)
        item=dict(seen[key],roles=[role])
        if role in selected and any(selected[role][k]!=item[k] for k in ('sha256','device_major','device_minor','inode')):
            raise InventoryError('multiple distinct mapped ELF identities for '+role)
        selected[role]=item
    missing=set(required_roles)-set(selected)
    if missing:raise InventoryError('mapped required ELF missing: '+str(sorted(missing)))
    if set(selected)-executable_roles:raise InventoryError('role ELF has no executable mapping: '+str(sorted(set(selected)-executable_roles)))
    return {'loaded_modules':list(selected.values()),'excluded_unlinked_shared_data':excluded,
      'executable_mappings':executable,'mapping_count':len(rows),
      'scope':'selected role ELF backing files; no process memory comparison; non-role executable maps retained in raw inventory'}


def inventory(pid,roles,required_roles,raw_maps_path,proc_root=Path('/proc')):
    """Persist exclusive/fsynced raw maps BEFORE parsing, role or hash checks.

    On interpretation failure `<raw>.audit.json` preserves the actual error and
    raw digest. A subsequent attempt must choose a new path, never overwrite.
    """
    if type(pid)is not int or pid<=0:raise InventoryError('positive integer PID required')
    process=Path(proc_root)/str(pid);raw_path=Path(raw_maps_path)
    stat_before=None;stat_error=None
    try:stat_before=(process/'stat').read_text().rsplit(')',1)[1].split()[19]
    except BaseException as error:stat_error=error
    payload=(process/'maps').read_bytes()
    _exclusive_bytes(raw_path,payload)
    record={'pid':pid,'raw_maps':{'path':str(raw_path.resolve()),'sha256':hashlib.sha256(payload).hexdigest()},
      'captured_wall_monotonic_ns':time.monotonic_ns(),'proc_start_ticks_before_capture':stat_before,'interpretation_pass':False}
    try:
        # Even a pre-capture stat error is raised only after raw maps are durable.
        # Parent independently checks launched ancestry; no adversarial proof.
        if stat_error is not None:raise stat_error
        text=payload.decode('utf-8',errors='strict')
        result=inventory_from_text(text,roles,required_roles)
        stat_after=(process/'stat').read_text().rsplit(')',1)[1].split()[19]
        if stat_before!=stat_after:raise InventoryError('process lifetime changed during inventory')
        record.update(interpretation_pass=True,proc_start_ticks=stat_after,**result)
        return dict(record,maps=text)
    except BaseException as error:
        record['error']=type(error).__name__+': '+str(error)
        raise
    finally:
        _exclusive_bytes(Path(str(raw_path)+'.audit.json'),(json.dumps(record,indent=2,allow_nan=False)+'\n').encode())
