"""Versioned backend-owned tuning, committed before runtime acknowledgment."""
import json
import math
import os
from pathlib import Path
import tempfile
import warnings


def default_state_file(name):
    ros_home = Path(os.environ.get('ROS_HOME', str(Path.home() / '.ros')))
    return str(ros_home / 'autolabor_navigation' / name)


def validate_values(values, defaults, minimum, maximum):
    if set(values) != set(defaults):
        raise ValueError('saved tuning fields do not match this schema')
    result = {}
    for key, initial in defaults.items():
        value = values[key]
        if type(initial) is int:
            if type(value) is not int:
                raise ValueError('saved tuning requires integer: ' + key)
        elif type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError('saved tuning requires finite number: ' + key)
        if not minimum[key] <= value <= maximum[key]:
            raise ValueError('saved tuning out of range: ' + key)
        result[key] = value
    return result


def load_values(path, defaults, minimum, maximum):
    target = Path(path)
    if not target.exists():
        return None
    with target.open(encoding='utf-8') as stream:
        data = json.load(stream)
    if set(data) != {'version', 'values'} or data['version'] != 1:
        raise ValueError('unsupported tuning store schema: ' + str(target))
    return validate_values(data['values'], defaults, minimum, maximum)


def save_values(path, values):
    target = Path(path)
    if not target.is_absolute():
        raise ValueError('tuning state path must be absolute')
    target.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    descriptor, staged = tempfile.mkstemp(prefix=target.name + '.', dir=str(target.parent))
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
            json.dump({'version': 1, 'values': values}, stream, allow_nan=False, indent=2)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(staged, str(target))
        # Rename is the commit point; a directory-sync warning must not claim
        # that the already committed values were rejected.
        try:
            directory = os.open(str(target.parent), os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        except OSError as error:
            warnings.warn('tuning committed, directory sync failed: ' + str(error))
    finally:
        if os.path.exists(staged):
            os.unlink(staged)
