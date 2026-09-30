"""Fixed two-case/10-pair M12 offline experiment; no ROS, actor or learning."""
from pathlib import Path
import copy
import cProfile
import dataclasses
import hashlib
import importlib.util
import json
import math
import os
import platform
import pstats
import site
import sys
import time

A=Path(__file__).resolve().parent;S=A.parents[1];W=S/'workspace';P=W/'scripts/astra_gazebo'
sys.path[:0]=[str(P),str(P/'tests')]
import numpy as np
import scipy
import shapely
import yaml
import pytest
import sensor_tracks_ingestion_m08 as old
import sensor_tracks_ingestion_m12 as candidate

def bind(path):
    path=Path(path).resolve();data=path.read_bytes()
    return dict(path=str(path),sha256=hashlib.sha256(data).hexdigest(),bytes=len(data))

def write(name,data):
    path=A/name
    if path.exists():raise ValueError('refuse to overwrite '+str(path))
    path.write_text(json.dumps(data,sort_keys=True,indent=2,allow_nan=False)+'\n')

protocol=json.loads((A/'protocol.json').read_text())
frozen_path=S/'audits/route_tracks_runtime_m08/benchmark_frozen.json'
frozen=json.loads(frozen_path.read_text())
protected=list(frozen['sources'])+[protocol['source_candidate'],protocol['protected_source']]
for row in protected:assert bind(row['path'])['sha256']==row['sha256'],row
assert scipy.__version__=='1.10.1' and np.__version__=='1.24.4'
assert sys.flags.no_user_site and all(os.environ.get(n)=='1' for n in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS'))
spec=importlib.util.spec_from_file_location('frozen_m08_benchmark_m12_reference',S/'audits/route_tracks_runtime_m08/benchmark.py')
b=importlib.util.module_from_spec(spec);sys.modules[spec.name]=b;spec.loader.exec_module(b)
assert 'torch' not in sys.modules
write('premeasurement_bindings.json',dict(protocol=bind(A/'protocol.json'),harness=bind(Path(__file__)),
      candidate=bind(candidate.__file__),protected=[bind(r['path']) for r in protected],old_globals_unchanged=old.IngestionSensorTracks.accept.__globals__['_clusters'] is old._clusters))

grid,pgm,metadata=b.make_grid(size=400,obstacles=[]);route=b.CommonRoute([(2.,7.),(12.,7.)]);pose=(3.,7.,0.)
sparse=np.array([(x*.4+1,y*.4+1) for x in range(64) for y in range(32)])
theta=np.linspace(-np.pi,np.pi,1440,endpoint=False);dense=np.array([pose[0]+3*np.cos(theta),pose[1]+3*np.sin(theta)]).T
cases=[('combined1440_sparse_reobserved_and2048_retained',sparse[:1440],np.array_split(sparse,2)),('maximum1440_dense_new_scan',dense,())]
(A/'synthetic_map.pgm').write_bytes(pgm);(A/'synthetic_map.yaml').write_bytes(metadata)
write('fixtures.json',dict(source_benchmark=bind(b.__file__),urdf=bind(b.URDF),T=b.T,pose=pose,route=list(route.points),
    synthetic_model_ranges=8.,synthetic_history_shape=[6,72],source_history_is_real=False,
    cases=[dict(name=n,points_shape=list(p.shape),points_sha256=hashlib.sha256(p.tobytes()).hexdigest(),
                retained_shapes=[list(c.shape) for c in r],retained_sha256=[hashlib.sha256(c.tobytes()).hexdigest() for c in r]) for n,p,r in cases]))

def canonical(value):
    if isinstance(value,np.ndarray):return dict(ndarray_dtype=value.dtype.str,shape=list(value.shape),bytes_sha256=hashlib.sha256(value.tobytes()).hexdigest())
    if isinstance(value,np.generic):return dict(numpy_scalar_dtype=value.dtype.str,value=canonical(value.item()))
    if isinstance(value,dict):return {k:canonical(v) for k,v in value.items()}
    if isinstance(value,(tuple,list)):return [canonical(v) for v in value]
    if isinstance(value,float) and not math.isfinite(value):return {'nonfinite_float':repr(value)}
    return value

def sha(value):return hashlib.sha256(json.dumps(canonical(value),sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()

def semantics(core,built,selection,command):
    snapshot=dataclasses.asdict(core.tracker.snapshot)
    snapshot['statistics']=[(k,v) for k,v in snapshot['statistics'] if k!='processing_ns']
    fields=dict(snapshot=snapshot,observation=built['observation'],previews=built['previews'],backups=built['backups'],
        backup_clearances=built['backup_clearances'],preview_clearances=built['preview_clearances'],prediction_audit=built['prediction_audit'],
        mask=built['mask'],selection=selection,command=command,state_source_ns=built['state_source_ns'],scan_source_ns=built['scan_source_ns'])
    return {k:sha(v) for k,v in fields.items()}

def trial(kind,points,retained):
    # Explicit per-instance offline injection. Never replace a frozen module global.
    core=b.RouteTracksRuntime(grid,route,b.URDF.read_text())
    if kind=='m12':core.tracker=candidate.IngestionSensorTracksM12(grid,core.tracker.config)
    core.identity=copy.deepcopy(core.identity)
    core.identity['offline_ingestion_implementation']=dict(name=kind,source=bind(candidate.__file__ if kind=='m12' else old.__file__))
    core.semantics_sha256=sha(core.identity)
    for index,chunk in enumerate(retained):b.accept(core,b.frame(b.T-200_000_000+index*100_000_000,pose,chunk),pose)
    f=b.frame(b.T,pose,points);actual=b.state(b.T,pose)
    start=time.perf_counter_ns();b.accept(core,f,pose);accepted=time.perf_counter_ns()
    built=core.build(actual,f,np.full((6,72),8.),1.,b.T);built_at=time.perf_counter_ns()
    selection=core.select(built);command=core.command(actual,f,selection[0],b.T);end=time.perf_counter_ns()
    result=dict(timing_ms=dict(accept=(accepted-start)/1e6,build=(built_at-accepted)/1e6,select_and_recheck=(end-built_at)/1e6,whole=(end-start)/1e6),
        field_sha256=semantics(core,built,selection,command),mask=built['mask'],selection=selection,command=command,
        backup_clearances=built['backup_clearances'],preview_clearances=built['preview_clearances'],
        query_count=built['query_count'],support_count=built['prediction_audit']['dynamic_count'],
        footprint_components=core.footprint.component_count,footprint_planes=len(core.footprint.vertices),
        evaluated_support_count=built['prediction_audit']['evaluated_support_count'])
    # Return only compact receipts. Complete old/new intermediate values were hashed
    # after the timed region without changing either computation.
    return json.loads(json.dumps(result,allow_nan=False,default=lambda x:x.item() if isinstance(x,np.generic) else x))

results=[];all_equal=True
for name,points,retained in cases:
    raw=[];first=None
    for index in range(10):
        order=['m08','m12'] if index%2==0 else ['m12','m08'];pair={}
        load_before=list(os.getloadavg())
        for kind in order:pair[kind]=trial(kind,points,retained)
        exact=pair['m08']['field_sha256']==pair['m12']['field_sha256']
        if first is None:first=pair['m08']['field_sha256']
        deterministic=all(row['field_sha256']==first for row in pair.values())
        all_equal=all_equal and exact and deterministic
        row=dict(pair_index=index,order=order,outputs=pair,exact_semantic_parity=exact,deterministic_across_pairs=deterministic,load_average_before=load_before)
        raw.append(row)
        with (A/'paired_trials.jsonl').open('a') as f:f.write(json.dumps(dict(case=name,**row),sort_keys=True,allow_nan=False)+'\n');f.flush();os.fsync(f.fileno())
        print(name,index,order,{k:pair[k]['timing_ms']['whole'] for k in order},'parity',exact,flush=True)
        if not exact or not deterministic:raise AssertionError('semantic mismatch; all completed timing rows retained')
    times={kind:{phase:b.summary([r['outputs'][kind]['timing_ms'][phase] for r in raw]) for phase in ('accept','build','select_and_recheck','whole')} for kind in ('m08','m12')}
    fractions=[(r['outputs']['m08']['timing_ms']['whole']-r['outputs']['m12']['timing_ms']['whole'])/r['outputs']['m08']['timing_ms']['whole'] for r in raw]
    results.append(dict(name=name,paired_trials=raw,timing=times,paired_relative_reductions=fractions,
        paired_median_relative_reduction=float(np.median(fractions)),ratio_of_medians_relative_reduction=1-times['m12']['whole']['median_ms']/times['m08']['whole']['median_ms'],
        all_m12_observed_whole_under50ms=all(r['outputs']['m12']['timing_ms']['whole']<=50. for r in raw),
        m12_over50ms_count=sum(r['outputs']['m12']['timing_ms']['whole']>50. for r in raw)))
    write('paired_'+('sparse' if len(retained) else 'dense')+'.json',results[-1])

write('comparison.json',dict(schema='astra_m12_paired_offline_comparison_v1',cases=results,semantic_parity=all_equal,
    factor_gate=dict(primary_ge5percent=results[0]['paired_median_relative_reduction']>=.05,
                    dense_regression_le5percent=results[1]['paired_median_relative_reduction']>=-.05),
    all_m12_observed_whole_under50ms=all(r['all_m12_observed_whole_under50ms'] for r in results),
    live_qualification=False,WCET_qualified=False,includes_actor=False,includes_ros_tf_callbacks=False,
    original_historical_negative_preserved=True,original_benchmark_report=bind(frozen_path)))

# Separate retrieval mechanism profile; excluded from paired measurements.
mechanism=[]
for name,points,_ in cases:
    for kind,func in [('m08',old._clusters),('m12',candidate._clusters)]:
        profiler=cProfile.Profile();out=profiler.runcall(func,points,.3);stats=pstats.Stats(profiler)
        calls=[dict(file=f,line=l,function=n,primitive_calls=v[0],total_calls=v[1],self_ms=v[2]*1000,cumulative_ms=v[3]*1000)
               for (f,l,n),v in stats.stats.items() if 'query_ball_point' in n or '_LazyNeighbors32' in n or (f==candidate.__file__ and n=='__call__')]
        mechanism.append(dict(case=name,implementation=kind,profiled_only=True,cluster_count=len(out),calls=calls))
write('retrieval_mechanism_profile.json',dict(rows=mechanism,not_deadline_timing=True))

raw_maps=Path('/proc/self/maps').read_text();(A/'process.maps.txt').write_text(raw_maps)
paths=sorted({line.split(None,5)[5] for line in raw_maps.splitlines() if len(line.split(None,5))==6 and line.split(None,5)[5].startswith('/') and '.so' in line.split(None,5)[5]})
modules=[]
for name,module in sorted(sys.modules.items()):
    path=getattr(module,'__file__',None)
    if path and Path(path).is_file() and (str(P) in str(Path(path).resolve()) or str(S/'audits/route_tracks_runtime_m08') in str(Path(path).resolve())):
        modules.append(dict(module=name,**bind(path)))
environment=dict(executable=sys.executable,interpreter=bind(sys.executable),python=sys.version,platform=platform.platform(),
    user_site_enabled=site.ENABLE_USER_SITE,sys_path=sys.path,
    selected_env={k:os.environ.get(k) for k in ('PYTHONNOUSERSITE','PYTHONDONTWRITEBYTECODE','OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS')},
    packages=[dict(name=n,version=m.__version__,**bind(m.__file__)) for n,m in [('numpy',np),('scipy',scipy),('shapely',shapely),('yaml',yaml),('pytest',pytest)]],
    mapped_libraries=[bind(p) for p in paths if Path(p).is_file()],source_modules=modules,
    cpu_models=sorted({line.split(':',1)[1].strip() for line in Path('/proc/cpuinfo').read_text().splitlines() if line.startswith('model name')}),
    affinity=sorted(os.sched_getaffinity(0)),torch_imported='torch' in sys.modules,gpu_used=False)
assert not environment['torch_imported']
write('environment.json',environment)
for row in protected:assert bind(row['path'])['sha256']==row['sha256'],row
write('postmeasurement_bindings.json',dict(all_protected_match=True,resources=[bind(r['path']) for r in protected],old_globals_unchanged=old.IngestionSensorTracks.accept.__globals__['_clusters'] is old._clusters))
print('completed two cases / 20 pairs / 40 full-core trials',flush=True)
