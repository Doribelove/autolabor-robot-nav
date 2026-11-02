"""Locate map assets after a recording has been moved to another machine."""
import json
from pathlib import Path


def recorded_map_directory(recording):
    recording = Path(recording)
    bundled = recording / 'assets' / 'static_map'
    if (bundled / 'manifest.yaml').is_file():
        return bundled
    session = json.loads((recording / 'session.json').read_text())
    original = session.get('local_map_directory')
    if original and (Path(original) / 'manifest.yaml').is_file():
        return Path(original)
    raise FileNotFoundError(
        'Recording map is missing: copy the complete map set into %s' % bundled)
