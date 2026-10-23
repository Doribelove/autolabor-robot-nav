"""Create a local reviewable candidate bundle; never copies to or starts a board."""
import argparse
import json
from pathlib import Path
import shutil
import tarfile

from .contracts import atomic_json, sha256
from .inference import ROOT


def build(output, root=ROOT, wheelhouse=None, capture_library=None):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    package = Path("distribution/j6m_bpu_navigation")
    sources = []
    for base in (package / "surround", Path("src/application/autolabor_surround_vision"),
                 Path("src/application/autolabor_fod_msgs")):
        sources += [p.relative_to(root) for p in (root / base).rglob("*")
                    if p.is_file() and "__pycache__" not in p.parts and p.suffix != ".pyc"]
    for name in ("board_preprocess.py", "native_tensor.py", "production_io.py", "production_ptq.py"):
        sources.append(package / "tools" / name)
    sources += [package / "production_models.json", package / "native/tensor_executor.cpp",
                Path("src/application/autolabor_fod_vision/src/autolabor_fod_vision/two_stage.py"),
                Path("src/application/autolabor_fod_vision/src/autolabor_fod_vision/__init__.py")]
    config = json.loads((root / package / "surround/config/surround.json").read_text())
    for kind in ("detector_zed", "classifier"):
        src = root / config["inference"][kind]["path"]
        if sha256(src) != config["inference"][kind]["sha256"]:
            raise ValueError("HBM identity mismatch")
        dest = Path("models") / (kind + ".hbm")
        (output / dest).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, output / dest)
        config["inference"][kind]["path"] = str(dest)
    for relative in sorted(set(sources)):
        src, dest = root / relative, output / relative
        if src.is_symlink() or not src.is_file():
            raise ValueError("Bundle source must be a regular file: " + str(relative))
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dest)
    native = (Path(capture_library).resolve() if capture_library else
              root / config["capture"]["library"])
    config["capture"]["library"] = "lib/libj6m_surround_capture.so"
    if native.is_file():
        (output / "lib").mkdir()
        shutil.copy2(native, output / "lib" / native.name)
    # Include the reviewed PYM/GDC candidate JSON, not an absolute path back to
    # the development host's vendor tree. LUTs and hardware approval stay pending.
    for key, filename in (("vpm_config", "vpm_config.json"), ("camera_config", "hb_j6dev.json")):
        src = root / config["capture"][key]
        dest = Path("camera_config") / filename
        (output / dest).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, output / dest)
        config["capture"][key] = str(dest)
    if wheelhouse:
        wheelhouse = Path(wheelhouse).resolve()
        wheel_manifest = json.loads((wheelhouse / "MANIFEST.json").read_text())
        destination = output / "runtime/surround_wheels_20261020"
        for relative, spec in wheel_manifest["files"].items():
            src = (wheelhouse / relative).resolve()
            if wheelhouse not in src.parents or sha256(src) != spec["sha256"]:
                raise ValueError("Wheelhouse integrity failure: " + relative)
            dest = destination / relative
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dest)
        shutil.copy2(wheelhouse / "MANIFEST.json", destination / "MANIFEST.json")
    # The native executor and SDK shared objects are deliberately selected only
    # after verifying the new image's ABI. Keep the configuration pending.
    atomic_json(output / package / "surround/config/surround.json", config)
    manifest = {}
    for p in output.rglob("*"):
        if p.is_file():
            manifest[str(p.relative_to(output))] = dict(bytes=p.stat().st_size, sha256=sha256(p))
    atomic_json(output / "MANIFEST.json", dict(
        schema="j6m.surround.bundle.v1", production_ready=False, motion_eligible=False,
        deployed=False, files=manifest, vendor_images_included=False,
        offline_python_wheels_included=bool(wheelhouse),
        note="Local candidate only. Rebuild capture with image-matched headers; verify BPU ABI before use."))
    archive = output.with_name(output.name + ".tar.gz")
    if archive.exists():
        raise ValueError("Archive already exists; choose a new version")
    with tarfile.open(archive, "x:gz") as tar:
        tar.add(output, arcname=output.name)
    atomic_json(archive.with_name(archive.name + ".sha256.json"),
                dict(file=archive.name, bytes=archive.stat().st_size, sha256=sha256(archive)))
    print(archive)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", required=True)
    p.add_argument("--wheelhouse")
    p.add_argument("--capture-library")
    a = p.parse_args()
    build(a.output, wheelhouse=a.wheelhouse, capture_library=a.capture_library)
