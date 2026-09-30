"""Read-only exploratory profiler for frozen M08 candidate build; no timing claim."""
import cProfile
import hashlib
import importlib.util
import json
import pstats
from pathlib import Path
import sys

import numpy as np

A = Path(__file__).resolve().parent
S = A.parents[1]
P = S / 'workspace/scripts/astra_gazebo'
sys.path[:0] = [str(P), str(P/'tests')]
spec = importlib.util.spec_from_file_location('frozen_m08_benchmark_for_m13',
                                               S/'audits/route_tracks_runtime_m08/benchmark.py')
b = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = b
spec.loader.exec_module(b)


def bind(path):
    return {'path': str(path.resolve()), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def profile(name, points, retained):
    grid, _, _ = b.make_grid(size=400, obstacles=[])
    route = b.CommonRoute([(2., 7.), (12., 7.)])
    pose = (3., 7., 0.)
    core = b.RouteTracksRuntime(grid, route, b.URDF.read_text())
    for index, chunk in enumerate(retained):
        b.accept(core, b.frame(b.T-200_000_000+index*100_000_000, pose, chunk), pose)
    frame = b.frame(b.T, pose, points)
    b.accept(core, frame, pose)
    actual = b.state(b.T, pose)
    profiler = cProfile.Profile()
    profiler.enable()
    built = core.build(actual, frame, np.full((6, 72), 8.), 1., b.T)
    profiler.disable()
    profile_path = A / (name + '.pstats')
    profiler.dump_stats(str(profile_path))
    entries = []
    for (file, line, function), (primitive, count, self_s, cumulative_s, _) in pstats.Stats(profiler).stats.items():
        entries.append({'file': file, 'line': line, 'function': function,
                        'primitive_calls': primitive, 'total_calls': count,
                        'self_ms_profiled': self_s*1000,
                        'cumulative_ms_profiled': cumulative_s*1000})
    entries.sort(key=lambda x: x['cumulative_ms_profiled'], reverse=True)
    return {'name': name, 'point_count': len(points), 'retained_count': sum(len(x) for x in retained),
            'query_count': built['query_count'], 'mask': built['mask'], 'profile': bind(profile_path),
            'top_functions': entries[:80]}


def main():
    sparse = np.array([(x*.4+1, y*.4+1) for x in range(64) for y in range(32)])
    theta = np.linspace(-np.pi, np.pi, 1440, endpoint=False)
    dense = np.array([3.+3*np.cos(theta), 7.+3*np.sin(theta)]).T
    cases = [profile('sparse1440_retained2048', sparse[:1440], np.array_split(sparse, 2)),
             profile('dense1440', dense, ())]
    result = {'schema': 'astra_m13_read_only_build_profile_v1',
              'scope': 'one synthetic build per case under cProfile; relative hotspots only; no deadline or population inference',
              'no_ROS_Gazebo_training_or_actor': True,
              'sources': [bind(Path(__file__)), bind(Path(b.__file__)), bind(P/'route_candidate_core_m08.py'),
                          bind(P/'sensor_tracks_ingestion_m08.py'), bind(P/'sensor_tracks_query_m08.py')],
              'cases': cases}
    output = A/'profile_report.json'
    with output.open('x') as f:
        json.dump(result, f, indent=2, allow_nan=False)
        f.write('\n')
    for case in cases:
        print(case['name'], case['point_count'], case['retained_count'])
        for row in case['top_functions'][:12]:
            print(' ', round(row['cumulative_ms_profiled'], 3), row['function'], Path(row['file']).name, row['line'])


if __name__ == '__main__':
    main()
