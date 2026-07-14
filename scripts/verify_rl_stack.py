#!/usr/bin/env python3
"""Verify the frozen T09 RL stack, origins and installed RECORD hashes."""

import argparse
import hashlib
from importlib.metadata import distribution
import json
from pathlib import Path
import platform
import sys

import yaml


WORKSPACE = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--lock", default=str(WORKSPACE / "requirements/thesis-rl-lock.yaml"))
    parser.add_argument("--output")
    args = parser.parse_args()
    lock = yaml.safe_load(Path(args.lock).read_text(encoding="utf-8"))
    errors = []
    if platform.python_version() != str(lock["python"]):
        errors.append("python_version")
    package_report = {}
    for name, expected in lock["packages"].items():
        dist = distribution(name)
        record = next(path for path in dist.files if str(path).endswith(".dist-info/RECORD"))
        record_path = Path(dist.locate_file(record))
        digest = hashlib.sha256(record_path.read_bytes()).hexdigest()
        package_report[name] = {"version": dist.version, "record_sha256": digest,
                                "record_path": str(record_path)}
        if dist.version != str(expected["version"]):
            errors.append("{}:version".format(name))
        if digest != expected["record_sha256"]:
            errors.append("{}:record_sha256".format(name))
    import gymnasium
    import numpy
    import stable_baselines3
    import torch
    from stable_baselines3 import SAC
    origins = {name: module.__file__ for name, module in (
        ("torch", torch), ("gymnasium", gymnasium),
        ("stable_baselines3", stable_baselines3), ("numpy", numpy),
    )}
    for name, origin in origins.items():
        if not str(origin).startswith(str(WORKSPACE / ".venv")):
            errors.append("{}:origin".format(name))
        if any(root in str(origin) for root in lock["forbidden_import_roots"]):
            errors.append("{}:forbidden_origin".format(name))
    if torch.cuda.is_available() or lock["cuda_required"] is not False:
        errors.append("cuda_boundary")
    report = {
        "schema_version": "1.0", "status": "valid" if not errors else "invalid",
        "python": platform.python_version(), "cuda_available": torch.cuda.is_available(),
        "sac_class": "{}.{}".format(SAC.__module__, SAC.__name__),
        "origins": origins, "packages": package_report, "errors": errors,
    }
    text = yaml.safe_dump(report, sort_keys=False)
    if args.output:
        destination = Path(args.output)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(text, encoding="utf-8")
    print(json.dumps({"status": report["status"], "errors": errors}, sort_keys=True))
    return 0 if not errors else 1


if __name__ == "__main__":
    sys.exit(main())
