"""One frozen M08 negative case only, plus separate diagnostic CPU profiling."""
from pathlib import Path
import cProfile
import dataclasses
import hashlib
import importlib.util
import json
import os
import platform
import pstats
import sys
import time

A = Path(__file__).resolve().parent
S = A.parents[1]
W = S / 'workspace'
B = S / 'audits/route_tracks_runtime_m08'


def bind(path):
    path = Path(path).resolve()
    return dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def write(name, value):
    (A / name).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


expected = json.loads((B / 'benchmark_frozen.json').read_text())
original = next(c for c in expected['cases'] if c['name'] == 'combined1440_sparse_reobserved_and2048_retained')
for row in expected['sources']:
    assert bind(row['path'])['sha256'] == row['sha256'], row
spec = importlib.util.spec_from_file_location('frozen_m08_benchmark', B / 'benchmark.py')
b = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = b
spec.loader.exec_module(b)
np = b.np
import scipy
import scipy.spatial
import shapely
import yaml
import pytest

# Exactly the retained negative fixture from the frozen benchmark's main().
grid, pgm, metadata = b.make_grid(size=400, obstacles=[])
route = b.CommonRoute([(2., 7.), (12., 7.)])
pose = (3., 7., 0.)
sparse = np.array([(x * .4 + 1, y * .4 + 1) for x in range(64) for y in range(32)])
retained = np.array_split(sparse, 2)
points = sparse[:1440]
(A / 'synthetic_map.pgm').write_bytes(pgm)
(A / 'synthetic_map.yaml').write_bytes(metadata)
write('fixture.json', dict(name=original['name'], route=list(route.points), pose=pose,
                          sparse_formula='[(x*.4+1,y*.4+1) for x in range(64) for y in range(32)]',
                          retained_chunks=[len(x) for x in retained], current_returns=len(points),
                          source_ns=b.T, retained_source_ns=[b.T-200_000_000, b.T-100_000_000],
                          history='synthetic np.full((6,72),8.); not six real scans',
                          setup_excluded_from_timing='runtime/URDF construction and two prior accept calls',
                          fixture_arrays_sha256=hashlib.sha256(sparse.tobytes()).hexdigest()))

if '--profile-only' not in sys.argv:
    print('Running only the frozen combined sparse case: 10 trials, no ROS/actor.', flush=True)
    start = time.perf_counter_ns()
    result = b.bench(grid, route, pose, points, 10, retained)
    write('reproduction.json', dict(schema='astra_m08_compatible_environment_single_case_replay_v1',
                                   original=original, current=result,
                                   harness_wall_s=(time.perf_counter_ns()-start)/1e9,
                                   historical_result_modified=False, live_qualification=False,
                                   includes_ros_tf_callback_scheduling=False, includes_actor=False))
    print(json.dumps(result['timing']), flush=True)
else:
    assert (A / 'reproduction.json').is_file()
    print('Profile-only follow-up; original 10-trial result is preserved.', flush=True)


def ready():
    core = b.RouteTracksRuntime(grid, route, b.URDF.read_text())
    for i, chunk in enumerate(retained):
        b.accept(core, b.frame(b.T-200_000_000+i*100_000_000, pose, chunk), pose)
    return core, b.frame(b.T, pose, points), b.state(b.T, pose)


def digest_semantics(core, built, command):
    snapshot = dataclasses.asdict(core.tracker.snapshot)
    snapshot['statistics'] = [(k, v) for k, v in snapshot['statistics'] if k != 'processing_ns']
    data = dict(snapshot=snapshot,
                observation={k: v.tolist() for k, v in built['observation'].items()},
                previews=built['previews'], backups=built['backups'],
                backup_clearances=built['backup_clearances'], preview_clearances=built['preview_clearances'],
                prediction_audit=built['prediction_audit'], mask=built['mask'], command=command)
    def scalar(value):
        if isinstance(value, np.generic):
            return value.item()
        raise TypeError('unsupported audit scalar: ' + type(value).__name__)
    return hashlib.sha256(json.dumps(data, sort_keys=True, separators=(',', ':'), allow_nan=False, default=scalar).encode()).hexdigest()


def collect(profile):
    core, frame, actual = ready()
    records = {}
    def call(name, function):
        if not profile:
            return function()
        profiler = cProfile.Profile()
        value = profiler.runcall(function)
        profiler.dump_stats(str(A / (name + '.prof')))
        stats = pstats.Stats(profiler)
        entries = []
        for (file, line, function), (primitive, total, self_s, cumulative_s, callers) in stats.stats.items():
            entries.append(dict(file=file, line=line, function=function,
                                primitive_calls=primitive, total_calls=total,
                                self_ms=self_s*1000, cumulative_ms=cumulative_s*1000))
        entries.sort(key=lambda x: x['cumulative_ms'], reverse=True)
        records[name] = dict(profiled_total_ms=stats.total_tt*1000, calls=stats.total_calls,
                             entries=entries, timing_is_instrumented=True)
        return value
    call('accept', lambda: b.accept(core, frame, pose))
    built = call('build', lambda: core.build(actual, frame, np.full((6, 72), 8.), 1., b.T))
    chosen = call('select', lambda: core.select(built))[0]
    command = call('command', lambda: core.command(actual, frame, chosen, b.T))
    fingerprint = digest_semantics(core, built, command)
    return records, fingerprint, dict(query_count=built['query_count'],
                                     support_count=built['prediction_audit']['dynamic_count'],
                                     evaluated_support_count=built['prediction_audit']['evaluated_support_count'],
                                     chosen=chosen, mask=built['mask'], command=command,
                                     footprint_components=core.footprint.component_count,
                                     footprint_planes=len(core.footprint.vertices))


print('Separate one-call-per-stage cProfile, not a comparable deadline timing.', flush=True)
_, unprofiled_hash, semantics = collect(False)
profiles, profiled_hash, profiled_semantics = collect(True)
assert unprofiled_hash == profiled_hash and semantics == profiled_semantics
write('profile.json', dict(stages=profiles, unprofiled_semantics_sha256=unprofiled_hash,
                          profiled_semantics_sha256=profiled_hash,
                          semantic_parity=True, semantics=semantics,
                          warning='inclusive cumulative times overlap; cProfile overhead is not an uninstrumented latency result'))

# Read-only actual interpreter/import/loaded-native evidence; no credential env.
raw_maps = Path('/proc/self/maps').read_text()
(A / 'process.maps.txt').write_text(raw_maps)
mapped = sorted({line.split(None, 5)[5] for line in raw_maps.splitlines()
                 if len(line.split(None, 5)) == 6 and line.split(None, 5)[5].startswith('/')
                 and '.so' in line.split(None, 5)[5]})
native = [bind(path) for path in mapped if Path(path).is_file()]
modules = []
for name, module in sorted(sys.modules.items()):
    path = getattr(module, '__file__', None)
    if path and Path(path).is_file() and (str(W / 'scripts/astra_gazebo') in str(Path(path).resolve()) or str(B) in str(Path(path).resolve())):
        modules.append(dict(module=name, **bind(path)))
cpu_models = sorted({line.split(':', 1)[1].strip() for line in Path('/proc/cpuinfo').read_text().splitlines() if line.startswith('model name')})
environment = dict(interpreter=bind(sys.executable), executable=sys.executable,
                   python=sys.version, platform=platform.platform(),
                   user_site_disabled=bool(sys.flags.no_user_site), sys_path=sys.path,
                   declared_environment={k: os.environ.get(k) for k in ('PYTHONNOUSERSITE', 'PYTHONDONTWRITEBYTECODE', 'OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS')},
                   packages=[dict(name=name, version=module.__version__, **bind(module.__file__))
                             for name, module in [('numpy', np), ('scipy', scipy), ('shapely', shapely), ('yaml', yaml), ('pytest', pytest)]],
                   cpu_models=cpu_models, affinity=sorted(os.sched_getaffinity(0)),
                   load_average=list(os.getloadavg()), source_modules=modules, actual_mapped_native_libraries=native,
                   torch_imported='torch' in sys.modules, gpu_used=False, simulation_started=False,
                   benchmark_case_count=1, frozen_case_trials=10,
                   excluded='ROS callbacks/TF/warmup/scheduling, actor inference, live load, hard WCET proof')
assert not environment['torch_imported']
write('environment.json', environment)
for row in expected['sources']:
    assert bind(row['path'])['sha256'] == row['sha256'], row
write('protected_bindings.json', dict(before_and_after_match=True, resources=expected['sources'],
                                     harness=bind(Path(__file__)), original_benchmark=bind(B/'benchmark_frozen.json')))
print(json.dumps(dict(completed=True, semantics=semantics, native_libraries=len(native), source_modules=len(modules))), flush=True)
