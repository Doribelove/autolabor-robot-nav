"""Prepare an isolated, hash-locked BEV environment without accessing devices."""
import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import venv

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
REQUIREMENTS = HERE / "requirements-bev.txt"


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + ".tmp")
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")
    temp.replace(path)


def requirements():
    return dict(line.split("==") for line in REQUIREMENTS.read_text().splitlines()
                if line and not line.startswith("#"))


def download(directory):
    directory = Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    manifest = {}
    for version, abi in (("3.8", "cp38"), ("3.11", "cp311")):
        target = directory / ("linux-aarch64-" + abi)
        target.mkdir(exist_ok=True)
        subprocess.run([
            sys.executable, "-m", "pip", "--isolated", "--disable-pip-version-check",
            "--cache-dir", str(directory / ".download-cache"),
            "download", "--index-url", "https://pypi.org/simple", "--only-binary=:all:",
            "--platform", "manylinux2014_aarch64", "--platform", "manylinux_2_17_aarch64",
            "--implementation", "cp", "--python-version", version,
            "--abi", abi, "--abi", "abi3", "--abi", "none",
            "--progress-bar", "off", "--dest", str(target), "-r", str(REQUIREMENTS),
        ], check=True)
        wheels = sorted(target.glob("*.whl"))
        lines = []
        for name, pinned in requirements().items():
            normalized = name.lower().replace("-", "_")
            found = [w for w in wheels if w.name.lower().split("-")[:2] == [normalized, pinned]]
            if len(found) != 1:
                raise ValueError("Need one wheel for " + name)
            wheel = found[0]
            checksum = digest(wheel)
            lines.append("%s==%s --hash=sha256:%s" % (name, pinned, checksum))
            manifest[str(wheel.relative_to(directory))] = dict(
                bytes=wheel.stat().st_size, sha256=checksum)
        lock = target / "requirements.lock"
        lock.write_text("\n".join(lines) + "\n")
        manifest[str(lock.relative_to(directory))] = dict(bytes=lock.stat().st_size, sha256=digest(lock))
    write_json(directory / "MANIFEST.json", dict(
        schema="j6m.surround.environment.v1", source="https://pypi.org/simple",
        requirements_sha256=digest(REQUIREMENTS), files=manifest))
    print(directory)


def setup(directory, wheelhouse):
    directory, wheelhouse = Path(directory).resolve(), Path(wheelhouse).resolve()
    if platform.machine() not in ("aarch64", "arm64") or sys.version_info[:2] not in ((3, 8), (3, 11)):
        raise ValueError("This wheelhouse supports Linux aarch64 CPython 3.8 / 3.11")
    target = wheelhouse / ("linux-aarch64-cp%d%d" % sys.version_info[:2])
    manifest = json.loads((wheelhouse / "MANIFEST.json").read_text())
    if manifest["requirements_sha256"] != digest(REQUIREMENTS):
        raise ValueError("Wheelhouse belongs to a different dependency lock")
    for name, spec in manifest["files"].items():
        path = (wheelhouse / name).resolve()
        if wheelhouse not in path.parents or not path.is_file() or digest(path) != spec["sha256"]:
            raise ValueError("Wheelhouse integrity failure: " + name)
    if directory.exists():
        raise ValueError("Environment already exists; use check or choose a new directory")
    # No apt, sudo, ensurepip, shared-site packages or modifications to system Python.
    venv.EnvBuilder(with_pip=False, system_site_packages=False).create(directory)
    python = directory / "bin/python"
    pip_wheel = next(target.glob("pip-*.whl"))
    env = os.environ.copy()
    env.update(PYTHONNOUSERSITE="1", PYTHONPATH="", PIP_CONFIG_FILE=os.devnull,
               OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
    bootstrap = "import runpy,sys;sys.path.insert(0,sys.argv.pop(1));runpy.run_module('pip',run_name='__main__')"
    subprocess.run([
        str(python), "-s", "-c", bootstrap, str(pip_wheel),
        "--isolated", "--disable-pip-version-check", "--no-cache-dir", "install", "--no-index",
        "--find-links", str(target), "--require-hashes", "-r", str(target / "requirements.lock"),
    ], check=True, env=env)
    subprocess.run([str(python), "-s", "-m", "pip", "check"], check=True, env=env)
    result = subprocess.run([str(python), "-s", str(__file__), "check"],
                            check=True, env=env, text=True, stdout=subprocess.PIPE)
    write_json(directory / "environment.json", json.loads(result.stdout))
    print(directory)


def check():
    import cv2
    import numpy as np
    import PIL
    expected = requirements()
    packages = {k: importlib.metadata.version(k) for k in expected}
    isolated = sys.prefix != sys.base_prefix
    private_modules = all(Path(sys.prefix).resolve() in Path(m.__file__).resolve().parents
                          for m in (cv2, np, PIL))
    a = np.arange(48, dtype=np.uint8).reshape(4, 4, 3)
    y, x = np.indices((4, 4), dtype=np.float32)
    remap_ok = np.array_equal(a, cv2.remap(a, x, y, cv2.INTER_LINEAR))
    encoded, png = cv2.imencode(".png", a)
    png_ok = encoded and np.array_equal(a, cv2.imdecode(png, cv2.IMREAD_COLOR))
    ready = isolated and private_modules and packages == expected and remap_ok and png_ok
    result = dict(schema="j6m.surround.environment.check.v1", software_ready=bool(ready),
                  python=sys.version, executable=sys.executable, architecture=platform.machine(),
                  prefix=sys.prefix, isolated=isolated, private_core_modules=private_modules,
                  packages=packages, opencv_remap=bool(remap_ok), png_roundtrip=bool(png_ok),
                  camera_opened=False, bpu_opened=False, hardware_verified=False)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if ready else 2


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="command", required=True)
    d = sub.add_parser("download")
    d.add_argument("--wheelhouse", default=str(ROOT / "runtime/surround_wheels_20261020"))
    s = sub.add_parser("setup")
    s.add_argument("--venv", default=str(ROOT / "runtime/surround_env"))
    s.add_argument("--wheelhouse", default=str(ROOT / "runtime/surround_wheels_20261020"))
    sub.add_parser("check")
    a = p.parse_args()
    if a.command == "download":
        download(a.wheelhouse)
    elif a.command == "setup":
        setup(a.venv, a.wheelhouse)
    else:
        return check()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
