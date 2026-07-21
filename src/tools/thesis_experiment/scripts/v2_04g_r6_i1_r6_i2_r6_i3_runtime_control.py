#!/usr/bin/env python3
"""Run the frozen R6 transaction control with an R6-I3 identity."""

import hashlib
import os
from pathlib import Path
import stat
import sys
import types


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-R6-I3"
SOURCE_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_runtime_control.py"
)
SOURCE_SHA256 = (
    "b927578e21ba2318571c0e2b4fb1925470648d0ffd40e0ce39346189d83e6fa1"
)
MAX_SOURCE_BYTES = 4 * 1024 * 1024


class R6I3WrapperError(RuntimeError):
    """Raised when a frozen child implementation cannot be trusted."""


def _read_frozen_source_once():
    root = WORKSPACE
    if not root.is_absolute() or root != root.resolve() or root.is_symlink():
        raise R6I3WrapperError("workspace root is not canonical")
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    directory_flags = (
        flags
        | getattr(os, "O_DIRECTORY", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    descriptors = []
    try:
        current = os.open(str(root), directory_flags)
        descriptors.append(current)
        for component in SOURCE_RELATIVE.parts[:-1]:
            current = os.open(component, directory_flags, dir_fd=current)
            descriptors.append(current)
        source_fd = os.open(
            SOURCE_RELATIVE.parts[-1],
            flags | getattr(os, "O_NOFOLLOW", 0),
            dir_fd=current,
        )
        descriptors.append(source_fd)
        before = os.fstat(source_fd)
        if not stat.S_ISREG(before.st_mode) or before.st_size > MAX_SOURCE_BYTES:
            raise R6I3WrapperError("frozen runtime-control source is unsafe")
        chunks = []
        remaining = MAX_SOURCE_BYTES + 1
        while remaining:
            chunk = os.read(source_fd, min(1024 * 1024, remaining))
            if not chunk:
                break
            chunks.append(chunk)
            remaining -= len(chunk)
        payload = b"".join(chunks)
        after = os.fstat(source_fd)
        fields = (
            "st_dev", "st_ino", "st_mode", "st_size", "st_mtime_ns",
            "st_ctime_ns",
        )
        if not (
            len(payload) == before.st_size
            and all(
                getattr(before, field) == getattr(after, field)
                for field in fields
            )
        ):
            raise R6I3WrapperError("frozen runtime control changed during read")
        if hashlib.sha256(payload).hexdigest() != SOURCE_SHA256:
            raise R6I3WrapperError("frozen runtime-control SHA256 drifted")
        return payload
    except OSError as exc:
        raise R6I3WrapperError(
            "cannot safely open frozen runtime control: {}".format(exc)
        ) from exc
    finally:
        for descriptor in reversed(descriptors):
            try:
                os.close(descriptor)
            except OSError:
                pass


def _load_frozen_control():
    payload = _read_frozen_source_once()
    name = "v2_04g_r6_i1_runtime_control_frozen_for_r6_i3"
    module = types.ModuleType(name)
    module.__file__ = str(WORKSPACE / SOURCE_RELATIVE)
    module.__package__ = ""
    sys.modules[name] = module
    exec(compile(payload, module.__file__, "exec"), module.__dict__)
    module.STAGE = STAGE
    return module


def main():
    return _load_frozen_control().main()


if __name__ == "__main__":
    raise SystemExit(main())
