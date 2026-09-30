"""Q04 provenance compatibility for locally reproduced NVIDIA GLX allocation.

The narrow deleted RX/RW alias is recorded as external dynamic graphics code,
never as a verified static ELF or a read/hash of anonymous executable memory.
All physics/ROS/custom ELF rules and durable raw-map evidence remain strict.
"""
import hashlib
import json
from pathlib import Path
import time

import mapped_elf_inventory as base
InventoryError=base.InventoryError
same_process=base.same_process
GRAPHICS_ROLES={'libnvidia-glcore.so':'graphics_nvidia_glcore',
 'libnvidia-gpucomp.so':'graphics_nvidia_gpucomp','libGLX_nvidia.so':'graphics_nvidia_glx'}
# Populated from actual mapped-file bindings verified by the local GLX probe;
# a driver update requires a NEW reviewed policy, not a permissive version glob.
GRAPHICS_BINDINGS={
 'graphics_nvidia_glcore':{'path':'/usr/lib/x86_64-linux-gnu/libnvidia-glcore.so.580.82.09','sha256':'655613a7f432d0174b01574fb880cdc69f410e9130f815233f0a6e3f87be57b2'},
 'graphics_nvidia_gpucomp':{'path':'/usr/lib/x86_64-linux-gnu/libnvidia-gpucomp.so.580.82.09','sha256':'f6dd0713e011013aa88006e063f7656a15babc3448d883437cbe55e236ff2754'},
 'graphics_nvidia_glx':{'path':'/usr/lib/x86_64-linux-gnu/libGLX_nvidia.so.580.82.09','sha256':'218f03a471bc0b1f4db0dbca699f3226776a7947804b56057aadb72c3bc15c52'}}
GRAPHICS_MEMFD='/memfd:/.glXXXXXX'
GRAPHICS_SIZE=8192


def graphics_aliases(rows):
    selected=[r for r in rows if r['backing_path']==GRAPHICS_MEMFD]
    if not selected:return []
    if len(selected)!=2:raise InventoryError('graphics alias requires exactly one RX and one RW mapping')
    if {r['permissions'] for r in selected}!={'r-xs','rw-s'}:raise InventoryError('graphics alias permissions differ')
    for r in selected:
        if not r['deleted'] or r['end']-r['start']!=GRAPHICS_SIZE or r['offset']!=0 or r['inode']<=0 or (r['device_major'],r['device_minor'])!=(0,1):
            raise InventoryError('graphics alias backing/size/offset differs')
    if len({(r['device_major'],r['device_minor'],r['inode']) for r in selected})!=1:raise InventoryError('graphics aliases do not share backing inode')
    return selected


def inventory_from_text(text,roles,required_roles,inspector=base.inspect_elf):
    rows=base.parse_maps(text);aliases=graphics_aliases(rows)
    # Classify only this exact observed alias group before the inherited strict
    # deleted-executable rejection. All other rows go through unchanged rules.
    omitted={r['line_number'] for r in aliases}
    retained='\n'.join(r['raw_line'] for r in rows if r['line_number'] not in omitted)
    combined=dict(roles)
    for stem,role in GRAPHICS_ROLES.items():
        if stem in combined and combined[stem]!=role:raise InventoryError('graphics role collision')
        combined[stem]=role
    required=set(required_roles)|(set(GRAPHICS_ROLES.values()) if aliases else set())
    result=base.inventory_from_text(retained,combined,required,inspector)
    driver_rows={r:entry for entry in result['loaded_modules'] for r in entry['roles'] if r in GRAPHICS_ROLES.values()}
    for role,entry in driver_rows.items():
        expected=GRAPHICS_BINDINGS[role]
        if entry['path']!=expected['path'] or entry['sha256']!=expected['sha256']:raise InventoryError('mapped graphics driver differs from exact policy: '+role)
    result['external_graphics_dynamic_code']=[{'classification':'observed_external_graphics_dynamic_code',
      'mapping_rows':aliases,'render_driver_bindings':list(driver_rows.values()),
      'attribution':'same exact alias pattern appeared after glXMakeContextCurrent and disappeared after context cleanup in local standalone probe',
      'static_ELF_verified':False,'executable_memory_bytes_read_or_hashed':False,
      'scope':'trusted local graphics stack; no arbitrary-memfd provenance or malicious-process proof'}] if aliases else []
    result['mapping_count']=len(rows)
    result['executable_mappings']=[r for r in rows if r['executable']]
    original_numbers={r['raw_line']:r['line_number'] for r in rows}
    for row in result['excluded_unlinked_shared_data']:row['line_number']=original_numbers[row['raw_line']]
    return result


def inventory(pid,roles,required_roles,raw_maps_path,proc_root=Path('/proc')):
    """Persist raw bytes before any interpretation, including graphics policy."""
    if type(pid)is not int or pid<=0:raise InventoryError('positive integer PID required')
    process=Path(proc_root)/str(pid);raw_path=Path(raw_maps_path);stat_before=None;stat_error=None
    try:stat_before=(process/'stat').read_text().rsplit(')',1)[1].split()[19]
    except BaseException as error:stat_error=error
    payload=(process/'maps').read_bytes();base._exclusive_bytes(raw_path,payload)
    record={'pid':pid,'raw_maps':{'path':str(raw_path.resolve()),'sha256':hashlib.sha256(payload).hexdigest()},
      'captured_wall_monotonic_ns':time.monotonic_ns(),'proc_start_ticks_before_capture':stat_before,'interpretation_pass':False,
      'inventory_schema':'astra_mapped_elf_inventory_v2'}
    try:
        if stat_error is not None:raise stat_error
        text=payload.decode('utf-8',errors='strict');result=inventory_from_text(text,roles,required_roles)
        stat_after=(process/'stat').read_text().rsplit(')',1)[1].split()[19]
        if stat_before!=stat_after:raise InventoryError('process lifetime changed during inventory')
        record.update(interpretation_pass=True,proc_start_ticks=stat_after,**result)
        return dict(record,maps=text)
    except BaseException as error:
        record['error']=type(error).__name__+': '+str(error);raise
    finally:base._exclusive_bytes(Path(str(raw_path)+'.audit.json'),(json.dumps(record,indent=2,allow_nan=False)+'\n').encode())
