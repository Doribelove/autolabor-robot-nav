"""Read-only, zero-authority preview of N06 release resource bindings.

The preview deliberately omits the pending runtime closure and independent
source review. It is not a release and cannot reserve seeds or start ROS.
"""

import hashlib
import json
from pathlib import Path


STAGE = Path(__file__).resolve().parents[1]


def read(name):
    return json.loads((STAGE / name).read_text())


def binding(path):
    path = Path(path)
    if not path.is_absolute() or path.resolve(strict=True) != path or not path.is_file():
        raise ValueError('noncanonical/nonregular resource: ' + str(path))
    return {'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def main():
    prepared = read('contracts/navigation_n06_prepared_r01/prepared.json')
    preparation = read('audits/navigation_n06_preparation/source_evidence_manifest.json')
    manager = read('audits/navigation_n06_manager_r02/source_evidence_manifest.json')
    conversions = read('contracts/navigation_n06_conversion_r01/bundle.json')
    sources = {}
    resources = {}

    def add(item, origin):
        if not isinstance(item, dict) or 'path' not in item or 'sha256' not in item:
            raise ValueError('invalid bound item in ' + origin)
        actual = binding(item['path'])
        if item['sha256'] != actual['sha256']:
            raise ValueError('changed bound bytes in ' + origin + ': ' + item['path'])
        old = resources.get(actual['path'])
        if old is not None and old != actual['sha256']:
            raise ValueError('contradictory hash: ' + actual['path'])
        resources[actual['path']] = actual['sha256']
        sources.setdefault(actual['path'], set()).add(origin)

    for category in ('owned_sources', 'root_owned_and_frozen_runtime_nodes',
                     'prepared_inputs', 'author_audits_including_failures',
                     'read_only_dependencies'):
        for item in preparation[category]:
            add(item, 'preparation.' + category)
    for category in ('owned_sources', 'author_evidence', 'read_only_dependencies'):
        for item in manager[category]:
            add(item, 'manager_r02.' + category)
    for unit in prepared['units']:
        for category in ('contract', 'launch', 'gate_expectation', 'robot_description', 'world'):
            add(unit[category], 'prepared.' + category)
    for unit in conversions['units']:
        for category in ('input_urdf', 'conversion_manifest', 'converted_sdf'):
            add(unit[category], 'conversion.' + category)
        for item in unit['conversion_resources']:
            add(item, 'conversion.resources')
    for name in ('contracts/navigation_n06_prepared_r01/prepared.json',
                 'contracts/navigation_n06_prepared_r01/schedule.json',
                 'audits/navigation_n06_preparation/source_evidence_manifest.json',
                 'audits/navigation_n06_manager_r02/source_evidence_manifest.json',
                 'contracts/navigation_n06_conversion_r01/bundle.json',
                 'audits/navigation_n06_conversion_independent/review.json',
                 'audits/navigation_n06_preparation_independent/review.json',
                 'audits/development_geometry_d04_independent/review.json'):
        path = STAGE / name
        if path.exists():
            add(binding(path), 'top.' + name)

    result = {
        'schema': 'astra_navigation_n06_resource_roster_preview_v1',
        'status': 'partial_read_only_no_release_or_authority',
        'resource_count': len(resources),
        'source_config_count': sum(Path(p).suffix in ('.py', '.launch', '.xacro', '.cpp', '.h', '.yaml', '.xml') for p in resources),
        'runtime_closure_included': False,
        'independent_source_review_included': False,
        'release_created': False,
        'resources': [{'path': p, 'sha256': h, 'origins': sorted(sources[p])}
                      for p, h in sorted(resources.items())],
    }
    output = STAGE / 'audits/navigation_n06_release_roster_preview_r01.json'
    if output.exists():
        raise FileExistsError('preserve first preview bytes')
    output.write_text(json.dumps(result, indent=2) + '\n')
    print(output, len(resources), result['source_config_count'])


if __name__ == '__main__':
    main()
