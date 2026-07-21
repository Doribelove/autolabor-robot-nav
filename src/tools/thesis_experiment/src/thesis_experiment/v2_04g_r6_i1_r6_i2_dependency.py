"""Canonical external dependency closure for the R6-I2 repair review.

This module is deliberately offline-only.  It resolves the Python and ROS
runtime names discovered from the I2 review surface to regular files, records
their canonical absolute paths and SHA256 values, and fails closed when any
binding cannot be resolved.
"""

import hashlib
import importlib.machinery
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess

from thesis_experiment import v2_04g_r6_i1_dependency as i1_dependency


STAGE = "V2-04G-R6-I2"
CONTRACT = (
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_r6_i2_bootstrap_integrity_repair_contract.yaml"
)
I1_CLOSURE = (
    "artifacts/v2/integration/v2_04g_r6_i1/"
    "execution_dependency_closure.yaml"
)
COMPILED_SCENE_INDEX = (
    "artifacts/v2/integration/v2_04g_r6_i1/compiled_scenes/"
    "compiled_scene_index.yaml"
)
ENTRYPOINTS = (
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_repair_harness.py",
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_reviewer.py",
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_assessor.py",
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_dependency_generator.py",
)
MANDATORY_I2_INPUTS = (
    CONTRACT,
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i2_stage_transition.yaml",
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i2_repair_preregistration.yaml",
    "src/simulation/m2_gazebo/launch/"
    "m2_v2_04g_r6_i2_execution_integration.launch",
    "src/simulation/m2_gazebo/launch/"
    "m2_v2_04g_r6_i2_spawn_m2.launch",
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_r6_i2_bootstrap.py",
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_r6_i2_authorization.py",
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_r6_i2_dependency.py",
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
    "test_v2_04g_r6_i2_bootstrap.py",
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
    "test_v2_04g_r6_i2_authorization_assessment.py",
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
    "test_v2_04g_r6_i2_dependency.py",
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
    "test_v2_04g_r6_i2_repair_harness.py",
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
    "test_v2_04g_r6_i2_review.py",
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i2_repair_review/"
    "v2_04g_r6_i2_authorization_assessment_review.yaml",
)
DECLARED_EXTERNAL_PYTHON_BINDINGS = ()
DECLARED_EXTERNAL_RUNTIME_BINDINGS = (
    "package-executable:xacro:xacro",
)

GENERATED_MESSAGE_PREFIX = "generated-ros-message:"
FIND_BINDING = re.compile(
    r"^\$\(find\s+([A-Za-z0-9_]+)\)/(.+)$"
)
NODE_BINDING = re.compile(
    r"^(?:node|workspace-built-node):"
    r"([A-Za-z0-9_]+):([^:\s]+)$"
)
PACKAGE_EXECUTABLE_BINDING = re.compile(
    r"^package-executable:([A-Za-z0-9_]+):([^:\s]+)$"
)
EXCLUDED_PACKAGE_PARTS = {"__pycache__"}
EXCLUDED_PACKAGE_SUFFIXES = {".pyc", ".pyo"}


class R6I2DependencyError(ValueError):
    """Raised when the canonical I2 dependency closure is incomplete."""


def _load_yaml_mapping(path, label):
    _, payload = _read_regular_file_once(path)
    try:
        document = i1_dependency.yaml.safe_load(payload.decode("utf-8"))
    except (UnicodeDecodeError, i1_dependency.yaml.YAMLError) as exc:
        raise R6I2DependencyError(
            "cannot parse {}: {}".format(label, exc)
        ) from exc
    if not isinstance(document, dict):
        raise R6I2DependencyError("{} must be a mapping".format(label))
    return document


def _contract_resource_paths(workspace):
    root = Path(workspace).resolve()
    document = _load_yaml_mapping(root / CONTRACT, "I2 contract")
    if document.get("stage") != STAGE:
        raise R6I2DependencyError("I2 contract stage drifted")
    resources = document.get("resources")
    if not isinstance(resources, dict) or not resources:
        raise R6I2DependencyError("I2 contract resources are missing")
    paths = []
    for resource_id, row in resources.items():
        if (
            not isinstance(row, dict)
            or set(row) != {"path", "sha256"}
            or not isinstance(row["path"], str)
            or not (
                isinstance(row["sha256"], str)
                or row["sha256"] == 0
            )
        ):
            raise R6I2DependencyError(
                "I2 contract resource schema drifted: {}".format(
                    resource_id
                )
            )
        relative = Path(row["path"])
        if (
            relative.is_absolute()
            or not relative.parts
            or any(part in {"", ".", ".."} for part in relative.parts)
        ):
            raise R6I2DependencyError(
                "unsafe I2 contract resource path: {}".format(row["path"])
            )
        paths.append(relative.as_posix())
    if len(paths) != len(set(paths)):
        raise R6I2DependencyError(
            "I2 contract resource paths are not unique"
        )
    return tuple(paths)


def _inherited_python_names(workspace):
    root = Path(workspace).resolve()
    document = _load_yaml_mapping(
        root / I1_CLOSURE, "frozen I1 dependency closure"
    )
    names = document.get("external_python_modules")
    if (
        not isinstance(names, list)
        or names != sorted(set(names))
        or len(names) != 39
        or any(not isinstance(name, str) or not name for name in names)
    ):
        raise R6I2DependencyError(
            "frozen I1 external Python binding set drifted"
        )
    return tuple(names)


def _read_regular_file_once(path):
    """Read one canonical regular file through one no-follow descriptor."""

    requested = Path(path)
    try:
        canonical = Path(os.path.realpath(str(requested))).absolute()
    except OSError as exc:
        raise R6I2DependencyError(
            "cannot canonicalize dependency {}: {}".format(requested, exc)
        ) from exc
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(str(canonical), flags)
    except OSError as exc:
        raise R6I2DependencyError(
            "cannot open canonical dependency {}: {}".format(
                canonical, exc
            )
        ) from exc
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode):
            raise R6I2DependencyError(
                "dependency is not a regular file: {}".format(canonical)
            )
        chunks = []
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            chunks.append(chunk)
        after = os.fstat(descriptor)
        identity_before = (
            before.st_dev,
            before.st_ino,
            before.st_size,
            before.st_mtime_ns,
        )
        identity_after = (
            after.st_dev,
            after.st_ino,
            after.st_size,
            after.st_mtime_ns,
        )
        if identity_before != identity_after:
            raise R6I2DependencyError(
                "dependency changed during single-open read: {}".format(
                    canonical
                )
            )
        descriptor_path = Path(
            os.path.realpath("/proc/self/fd/{}".format(descriptor))
        )
        if descriptor_path != canonical:
            raise R6I2DependencyError(
                "canonical dependency identity drifted: {}".format(
                    canonical
                )
            )
        payload = b"".join(chunks)
        if len(payload) != before.st_size:
            raise R6I2DependencyError(
                "dependency size drifted during read: {}".format(canonical)
            )
        return canonical, payload
    finally:
        os.close(descriptor)


def canonical_file_record(path):
    """Return the exact canonical path, size, and SHA256 for one file."""

    canonical, payload = _read_regular_file_once(path)
    return {
        "canonical_path": canonical.as_posix(),
        "sha256": hashlib.sha256(payload).hexdigest(),
        "size_bytes": len(payload),
    }


def _record_map(paths):
    records = {}
    for path in paths:
        record = canonical_file_record(path)
        records[record["canonical_path"]] = record
    return records


def _package_files(locations, origin=None):
    candidates = set()
    if origin and origin not in {"built-in", "frozen"}:
        candidate = Path(origin)
        if candidate.is_file():
            candidates.add(candidate)
    for location in locations:
        root = Path(location)
        if not root.is_dir():
            raise R6I2DependencyError(
                "Python package location is missing: {}".format(root)
            )
        for candidate in root.rglob("*"):
            if any(
                part in EXCLUDED_PACKAGE_PARTS
                for part in candidate.parts
            ):
                continue
            if candidate.suffix in EXCLUDED_PACKAGE_SUFFIXES:
                continue
            if candidate.is_file():
                candidates.add(candidate)
    if not candidates:
        raise R6I2DependencyError("Python package contains no bindable files")
    return candidates


def _generated_message_files(workspace, module_name):
    parts = module_name.split(".")
    if len(parts) != 2 or parts[1] != "msg":
        raise R6I2DependencyError(
            "invalid generated ROS message binding: {}".format(module_name)
        )
    package_name = parts[0]
    roots = []
    devel_root = Path(workspace) / "devel"
    for dist_packages in sorted(
        devel_root.glob("lib/python*/dist-packages")
    ):
        package_root = dist_packages / package_name
        if (package_root / "msg").is_dir():
            roots.append(package_root)
    if len(roots) != 1:
        raise R6I2DependencyError(
            "generated ROS message package must resolve exactly once: "
            "{} -> {}".format(module_name, roots)
        )
    root = roots[0]
    candidates = set()
    root_init = root / "__init__.py"
    if root_init.is_file():
        candidates.add(root_init)
    candidates.update(
        candidate
        for candidate in (root / "msg").rglob("*")
        if candidate.is_file()
        and "__pycache__" not in candidate.parts
        and candidate.suffix not in EXCLUDED_PACKAGE_SUFFIXES
    )
    if not candidates:
        raise R6I2DependencyError(
            "generated ROS message binding has no files: {}".format(
                module_name
            )
        )
    return root, candidates


def resolve_python_binding(workspace, binding, interpreter_record=None):
    """Resolve one external Python binding without importing its code."""

    interpreter = interpreter_record or canonical_file_record(
        os.sys.executable
    )
    if binding.startswith(GENERATED_MESSAGE_PREFIX):
        module_name = binding[len(GENERATED_MESSAGE_PREFIX):]
        root, files = _generated_message_files(workspace, module_name)
        records = _record_map(files)
        return {
            "binding": binding,
            "resolution_kind": "generated_ros_message_package",
            "module_origin": (root / "msg").resolve().as_posix(),
            "canonical_paths": sorted(records),
        }, records

    try:
        specification = importlib.util.find_spec(binding)
    except (ImportError, AttributeError, ValueError) as exc:
        raise R6I2DependencyError(
            "cannot resolve external Python binding {}: {}".format(
                binding, exc
            )
        ) from exc
    if specification is None:
        # Integration review may run without sourcing the ROS overlay. Resolve
        # the installed and generated roots explicitly, then bind the exact
        # files selected from those roots.
        search_roots = []
        for candidate in (
            Path(workspace) / "devel/lib/python3/dist-packages",
            Path("/opt/ros/noetic/lib/python3/dist-packages"),
            Path("/usr/lib/python3/dist-packages"),
        ):
            if candidate.is_dir():
                search_roots.append(str(candidate.resolve()))
        specification = importlib.machinery.PathFinder.find_spec(
            binding, search_roots
        )
    if specification is None:
        if binding != "sys":
            raise R6I2DependencyError(
                "external Python binding is unresolved: {}".format(binding)
            )
        return {
            "binding": binding,
            "resolution_kind": "interpreter_builtin",
            "module_origin": "built-in",
            "canonical_paths": [interpreter["canonical_path"]],
        }, {interpreter["canonical_path"]: interpreter}

    origin = specification.origin
    locations = list(specification.submodule_search_locations or [])
    # CPython exposes ``sys`` through BuiltinImporter with a ``None`` origin.
    if binding == "sys" and origin is None:
        return {
            "binding": binding,
            "resolution_kind": "interpreter_builtin",
            "module_origin": "built-in",
            "canonical_paths": [interpreter["canonical_path"]],
        }, {interpreter["canonical_path"]: interpreter}
    if origin in {"built-in", "frozen"}:
        return {
            "binding": binding,
            "resolution_kind": "interpreter_{}".format(origin),
            "module_origin": origin,
            "canonical_paths": [interpreter["canonical_path"]],
        }, {interpreter["canonical_path"]: interpreter}

    files = _package_files(locations, origin)
    records = _record_map(files)
    resolution_kind = (
        "python_package_tree" if locations else "python_module_file"
    )
    canonical_origin = (
        canonical_file_record(origin)["canonical_path"]
        if origin and Path(origin).is_file()
        else str(origin)
    )
    return {
        "binding": binding,
        "resolution_kind": resolution_kind,
        "module_origin": canonical_origin,
        "canonical_paths": sorted(records),
    }, records


def _rospack_find(package_name):
    executable = shutil.which("rospack")
    if not executable:
        raise R6I2DependencyError("rospack executable is unavailable")
    try:
        result = subprocess.run(
            [executable, "find", package_name],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            encoding="utf-8",
            timeout=10,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise R6I2DependencyError(
            "cannot resolve ROS package {}: {}".format(package_name, exc)
        ) from exc
    lines = [line.strip() for line in result.stdout.splitlines() if line.strip()]
    if len(lines) != 1:
        raise R6I2DependencyError(
            "ROS package must resolve exactly once: {} -> {}".format(
                package_name, lines
            )
        )
    package_root = Path(lines[0])
    if not package_root.is_dir():
        raise R6I2DependencyError(
            "ROS package root is missing: {}".format(package_root)
        )
    return package_root.resolve(), canonical_file_record(executable)


def _runtime_node_candidates(workspace, package_root, package_name, executable):
    candidates = [
        Path(workspace) / "devel/lib" / package_name / executable,
        package_root / executable,
        package_root / "scripts" / executable,
        package_root / "nodes" / executable,
    ]
    if package_root.parent.name == "share":
        prefix = package_root.parent.parent
        candidates.append(prefix / "lib" / package_name / executable)
    return [candidate for candidate in candidates if candidate.is_file()]


def resolve_runtime_binding(workspace, binding):
    """Resolve one ROS launch substitution or node name to exact files."""

    find_match = FIND_BINDING.fullmatch(binding)
    node_match = NODE_BINDING.fullmatch(binding)
    package_executable_match = PACKAGE_EXECUTABLE_BINDING.fullmatch(binding)
    if find_match:
        package_name, relative = find_match.groups()
        package_root, resolver_record = _rospack_find(package_name)
        target = package_root / relative
        if not target.is_file():
            raise R6I2DependencyError(
                "ROS launch substitution is unresolved: {} -> {}".format(
                    binding, target
                )
            )
        kind = "roslaunch_find_file"
    elif node_match or package_executable_match:
        match = node_match or package_executable_match
        package_name, executable = match.groups()
        package_root, resolver_record = _rospack_find(package_name)
        candidates = _runtime_node_candidates(
            workspace,
            package_root,
            package_name,
            executable,
        )
        canonical_candidates = {
            Path(os.path.realpath(str(candidate))).absolute()
            for candidate in candidates
        }
        if len(canonical_candidates) != 1:
            raise R6I2DependencyError(
                "ROS node binding must resolve exactly once: {} -> {}".format(
                    binding, sorted(str(path) for path in canonical_candidates)
                )
            )
        target = next(iter(canonical_candidates))
        kind = (
            "ros_node_executable"
            if node_match
            else "ros_package_executable"
        )
    else:
        raise R6I2DependencyError(
            "unsupported external runtime binding: {}".format(binding)
        )

    paths = [target]
    manifest = package_root / "package.xml"
    if not manifest.is_file():
        raise R6I2DependencyError(
            "external ROS package manifest is missing: {}".format(
                package_root
            )
        )
    paths.append(manifest)
    records = _record_map(paths)
    records[resolver_record["canonical_path"]] = resolver_record
    target_record = canonical_file_record(target)
    return {
        "binding": binding,
        "resolution_kind": kind,
        "package": package_name,
        "package_root": package_root.as_posix(),
        "target_canonical_path": target_record["canonical_path"],
        "canonical_paths": sorted(records),
    }, records


def build_external_dependency_closure(
    workspace,
    python_bindings,
    runtime_bindings,
):
    """Close every named external binding to canonical file records."""

    workspace = Path(workspace).resolve()
    interpreter = canonical_file_record(os.sys.executable)
    python_rows = []
    runtime_rows = []
    records = {interpreter["canonical_path"]: interpreter}

    for binding in sorted(set(python_bindings)):
        row, resolved = resolve_python_binding(
            workspace, binding, interpreter_record=interpreter
        )
        python_rows.append(row)
        records.update(resolved)
    for binding in sorted(set(runtime_bindings)):
        row, resolved = resolve_runtime_binding(workspace, binding)
        runtime_rows.append(row)
        records.update(resolved)

    result = {
        "python_interpreter": interpreter,
        "python_bindings": python_rows,
        "runtime_bindings": runtime_rows,
        "files": [records[path] for path in sorted(records)],
        "unresolved": [],
    }
    result["closure_sha256"] = hashlib.sha256(
        json.dumps(
            result,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
    ).hexdigest()
    return result


def _discover_local_closure(workspace, entrypoints, mandatory_inputs):
    root = Path(workspace).resolve()
    packages = i1_dependency._package_index(root)
    module_roots = i1_dependency._module_roots(packages)
    declared = tuple(entrypoints) + tuple(mandatory_inputs)
    if not entrypoints:
        raise R6I2DependencyError("I2 closure has no entrypoint")
    if any(
        "authorization" in Path(path).name.lower()
        and "r6_i2" in Path(path).name.lower()
        and Path(path).suffix in {".yaml", ".yml"}
        and tuple(Path(path).parts[:2]) == ("experiments", "manifests")
        for path in declared
    ):
        raise R6I2DependencyError(
            "I2 integration closure must not contain an authorization"
        )

    initial = set()
    for relative in declared:
        path = (root / relative).resolve()
        try:
            path.relative_to(root)
        except ValueError as exc:
            raise R6I2DependencyError(
                "I2 local dependency leaves workspace: {}".format(path)
            ) from exc
        if not path.is_file():
            raise R6I2DependencyError(
                "mandatory I2 dependency is missing: {}".format(relative)
            )
        initial.add(path)

    compiled_index = (root / COMPILED_SCENE_INDEX).resolve()
    compiled_children = set()
    if compiled_index in initial:
        compiled_children = i1_dependency._scene_children(root)
        if len(compiled_children) != 14:
            raise R6I2DependencyError(
                "compiled scene child count drifted: {}".format(
                    len(compiled_children)
                )
            )
        initial.update(compiled_children)

    files = set()
    edges = {
        (compiled_index, child, "compiled_scene_child")
        for child in compiled_children
    }
    external_python = set()
    external_runtime = set()
    pending = list(initial)
    while pending:
        source = pending.pop()
        if source in files:
            continue
        files.add(source)
        targets = set()
        if source.suffix == ".py":
            discovered, external = i1_dependency._discover_python(
                source, module_roots
            )
            targets.update(discovered)
            external_python.update(external)
        if source.suffix in {".launch", ".xml", ".xacro"}:
            discovered, external = i1_dependency._discover_launch(
                source, packages
            )
            targets.update(discovered)
            external_runtime.update(external)
        for target in targets:
            if target.suffix == ".py" and source.suffix == ".py":
                kind = "python_import"
            else:
                kind = "protocol_input"
            edges.add((source, target, kind))
            if target not in files:
                pending.append(target)

    primary = (root / entrypoints[0]).resolve()
    for target in files:
        if target != primary and not any(edge[1] == target for edge in edges):
            edges.add((primary, target, "declared_protocol_input"))

    local_records = []
    for path in sorted(
        files, key=lambda item: item.relative_to(root).as_posix()
    ):
        record = canonical_file_record(path)
        local_records.append({
            "path": path.relative_to(root).as_posix(),
            "sha256": record["sha256"],
            "size_bytes": record["size_bytes"],
        })
    edge_rows = [
        {
            "from": source.relative_to(root).as_posix(),
            "to": target.relative_to(root).as_posix(),
            "kind": kind,
        }
        for source, target, kind in sorted(
            edges,
            key=lambda row: (
                row[0].relative_to(root).as_posix(),
                row[1].relative_to(root).as_posix(),
                row[2],
            ),
        )
    ]
    return {
        "entrypoints": list(entrypoints),
        "files": local_records,
        "edges": edge_rows,
        "external_python_names": sorted(external_python),
        "external_runtime_names": sorted(external_runtime),
        "required_paths": [row["path"] for row in local_records],
    }


def build_dependency_closure(
    workspace,
    entrypoints=None,
    mandatory_inputs=None,
):
    """Build the complete local plus canonical-external I2 closure."""

    selected_entrypoints = (
        tuple(ENTRYPOINTS) if entrypoints is None else tuple(entrypoints)
    )
    selected_inputs = (
        tuple(
            dict.fromkeys(
                MANDATORY_I2_INPUTS
                + _contract_resource_paths(workspace)
            )
        )
        if mandatory_inputs is None
        else tuple(mandatory_inputs)
    )
    local = _discover_local_closure(
        workspace, selected_entrypoints, selected_inputs
    )
    inherited_python = set(_inherited_python_names(workspace))
    local["external_python_names"] = sorted(
        set(local["external_python_names"]) | inherited_python
    )
    inherited_coverage = (
        inherited_python & set(local["external_python_names"])
    )
    if len(inherited_coverage) != 39:
        raise R6I2DependencyError(
            "I1 Python binding coverage is incomplete"
        )
    discovered_runtime = set(local["external_runtime_names"])
    expected_runtime = {
        "$(find gazebo_ros)/launch/empty_world.launch",
        "node:gazebo_ros:spawn_model",
        "node:move_base:move_base",
        "node:robot_state_publisher:robot_state_publisher",
    }
    if discovered_runtime != expected_runtime:
        raise R6I2DependencyError(
            "I2 discovered runtime binding set drifted: {}".format(
                sorted(discovered_runtime)
            )
        )
    external = build_external_dependency_closure(
        workspace,
        (
            set(local["external_python_names"])
            | set(DECLARED_EXTERNAL_PYTHON_BINDINGS)
        ),
        (
            set(local["external_runtime_names"])
            | set(DECLARED_EXTERNAL_RUNTIME_BINDINGS)
        ),
    )
    document = {
        "schema_version": "3.0",
        "stage": STAGE,
        "review_scope": "offline_execution_integration_repair_only",
        "execution_authorized": False,
        "seed_or_evidence_units_allocated": 0,
        "seed_or_evidence_units_consumed": 0,
        "authorization_resources": [],
        "generator": (
            "thesis_experiment.v2_04g_r6_i1_r6_i2_dependency."
            "build_dependency_closure"
        ),
        "local": local,
        "external": external,
        "unresolved": [],
    }
    document["closure_sha256"] = hashlib.sha256(
        json.dumps(
            document,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
    ).hexdigest()
    return document


def verify_external_files(document):
    """Re-hash all external records and reject schema/path drift."""

    if not isinstance(document, dict):
        raise R6I2DependencyError("external closure must be a mapping")
    expected_keys = {
        "python_interpreter",
        "python_bindings",
        "runtime_bindings",
        "files",
        "unresolved",
        "closure_sha256",
    }
    if set(document) != expected_keys:
        raise R6I2DependencyError("external closure schema drifted")
    if document["unresolved"] != []:
        raise R6I2DependencyError("external closure contains unresolved items")
    seen = set()
    for record in document["files"]:
        if set(record) != {"canonical_path", "sha256", "size_bytes"}:
            raise R6I2DependencyError("external file record schema drifted")
        path = Path(record["canonical_path"])
        if not path.is_absolute() or path != path.resolve():
            raise R6I2DependencyError(
                "external path is not canonical: {}".format(path)
            )
        if path.as_posix() in seen:
            raise R6I2DependencyError(
                "duplicate external path: {}".format(path)
            )
        seen.add(path.as_posix())
        if canonical_file_record(path) != record:
            raise R6I2DependencyError(
                "external dependency drifted: {}".format(path)
            )
    available = seen
    interpreter_path = document["python_interpreter"]["canonical_path"]
    if interpreter_path not in available:
        raise R6I2DependencyError("Python interpreter record is unbound")
    for section in ("python_bindings", "runtime_bindings"):
        bindings = document[section]
        names = [row["binding"] for row in bindings]
        if names != sorted(set(names)):
            raise R6I2DependencyError(
                "{} are not unique and sorted".format(section)
            )
        for row in bindings:
            paths = row.get("canonical_paths")
            if not paths or paths != sorted(set(paths)):
                raise R6I2DependencyError(
                    "binding paths are incomplete: {}".format(row["binding"])
                )
            if not set(paths).issubset(available):
                raise R6I2DependencyError(
                    "binding references an unrecorded file: {}".format(
                        row["binding"]
                    )
                )
    payload = {
        key: value
        for key, value in document.items()
        if key != "closure_sha256"
    }
    digest = hashlib.sha256(
        json.dumps(
            payload,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
    ).hexdigest()
    if digest != document["closure_sha256"]:
        raise R6I2DependencyError("external closure digest drifted")
    return {
        "external_file_count": len(seen),
        "python_binding_count": len(document["python_bindings"]),
        "runtime_binding_count": len(document["runtime_bindings"]),
        "closure_sha256": digest,
    }
