#!/usr/bin/env python3
"""Actual filesystem fixtures for unreleased archive helper; no runtime evidence changed."""
import concurrent.futures,hashlib,importlib.util,json,multiprocessing,os,stat,tempfile,time
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1];P=S/'workspace/scripts/astra_gazebo/bound_archive.py'
spec=importlib.util.spec_from_file_location('archive_under_review',P);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
payload=b'Astra independent actual filesystem bytes\n'*2048;digest=hashlib.sha256(payload).hexdigest();rows=[]
def case(name,fn):
 p=Path(tempfile.mkdtemp(prefix=name+'_',dir=str(D/'fixtures')))
 try:out=fn(p);rows.append(dict(name=name,**out))
 except BaseException as e:rows.append({'name':name,'fixture_error':type(e).__name__+': '+str(e)})
def setup(p):
 source=p/'source';source.write_bytes(payload);run=p/'run';run.mkdir();store=p/'store';return source,run,store,{'path':str(source),'sha256':digest}
def concurrent_case(p):
 source,run,store,item=setup(p);runs=[p/('run'+str(i))for i in range(8)]
 for r in runs:r.mkdir()
 with concurrent.futures.ThreadPoolExecutor(max_workers=8)as pool:out=list(pool.map(lambda r:m.archive(r,item,store),runs))
 paths=[Path(x['path'])for x in out];same=len({(x.stat().st_dev,x.stat().st_ino)for x in paths+[store/digest]})==1
 return {'pass':same and paths[0].stat().st_ino!=source.stat().st_ino and paths[0].stat().st_nlink==9 and not list(store.glob('.pending-*')),'shared_inode':paths[0].stat().st_ino,'source_inode':source.stat().st_ino,'archive_nlink':paths[0].stat().st_nlink,'single_physical_archive_blocks':(store/digest).stat().st_blocks,'sum_unique_inode_blocks':sum(x.stat().st_blocks for x in {x.stat().st_ino:x for x in paths+[store/digest]}.values()),'eight_logical_run_bytes':sum(x.stat().st_size for x in paths)}
def preexisting(p,kind):
 source,run,store,item=setup(p);store.mkdir();target=store/digest
 if kind=='source_hardlink':os.link(source,target)
 elif kind=='writable_copy':target.write_bytes(payload)
 elif kind=='readonly_copy':target.write_bytes(payload);target.chmod(0o444)
 else:raise ValueError(kind)
 out=m.archive(run,item,store);local=Path(out['path']);same=local.stat().st_ino==source.stat().st_ino;writebits=local.stat().st_mode&0o222
 if same:source.write_bytes(b'changed source')
 return {'accepted':True,'source_same_inode':same,'archive_write_bits':writebits,'post_source_edit_archive_hash_still_exact':hashlib.sha256(local.read_bytes()).hexdigest()==digest,'pass':not same and writebits==0}
def localcopy(p):
 source,run,store,item=setup(p);directory=run/'bound_blobs';directory.mkdir();local=directory/digest;local.write_bytes(payload);local.chmod(0o444);inode=local.stat().st_ino
 out=m.archive(run,item,store)
 return {'accepted':True,'existing_not_replaced':local.stat().st_ino==inode,'same_store_inode':local.stat().st_ino==(store/digest).stat().st_ino,'pass':local.stat().st_ino==(store/digest).stat().st_ino}
def symlink_source(p):
 source,run,store,item=setup(p);link=p/'source_alias';link.symlink_to(source);item['path']=str(link);out=m.archive(run,item,store);local=Path(out['path']);return {'pass':local.read_bytes()==payload and local.stat().st_ino!=source.stat().st_ino,'source_alias_accepted':True}
def symlink_archive(p,where):
 source,run,store,item=setup(p);directory=store if where=='store'else run/'bound_blobs';directory.mkdir();link=directory/digest;link.symlink_to(source)
 try:m.archive(run,item,store);rejected=False
 except (OSError,ValueError):rejected=True
 return {'pass':rejected and source.read_bytes()==payload and link.is_symlink(),'rejected':rejected}
def fifo(p):
 source,run,store,item=setup(p);source.unlink();os.mkfifo(source)
 process=multiprocessing.Process(target=m.archive,args=(run,item,store));process.start();process.join(.5);blocked=process.is_alive()
 if blocked:process.terminate();process.join(2)
 return {'pass':not blocked,'blocked_on_nonregular_input':blocked,'fixture_process_terminated':blocked,'runtime_processes_touched':False}
(D/'fixtures').mkdir(exist_ok=True)
case('concurrent_eight_runs',concurrent_case)
for k in ('source_hardlink','writable_copy','readonly_copy'):case('existing_'+k,lambda p,k=k:preexisting(p,k))
case('existing_local_distinct_copy',localcopy);case('source_symlink',symlink_source)
for w in ('store','local'):case('archive_symlink_'+w,lambda p,w=w:symlink_archive(p,w))
case('nonregular_fifo',fifo)
out={'reviewed_source':{'path':str(P),'sha256':hashlib.sha256(P.read_bytes()).hexdigest()},'scope':'Real files/inodes/blocks and short child fixture, no ROS or runtime process operation','rows':rows,'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
with (D/'actual_filesystem_findings.json').open('x')as f:json.dump(out,f,indent=2);f.write('\n')
print(json.dumps(out,indent=2))
