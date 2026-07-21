"""Mechanical dependency discovery for the R6-I1 execution entrypoints."""

import ast
import hashlib
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml


ENTRYPOINTS = (
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_bounded_validation.py",
    "src/tools/thesis_experiment/scripts/assess_v2_04g_r6_i1.py",
)
MANDATORY_EXECUTION_INPUTS = (
    "config/thesis_experiments/v2/v2_04g_r6_i1_execution_integration_contract.yaml",
    "experiments/manifests/v2/integration/v2_04g_r6_i1_execution_preregistration.yaml",
    "experiments/manifests/v2/integration/v2_04g_r6_i1_scene_derivation.yaml",
    "experiments/manifests/v2/integration/v2_04g_r6_i1_stage_transition.yaml",
    "artifacts/v2/integration/v2_04g_r6_i1/v2_04g_r6_i1_scenes.yaml",
    "artifacts/v2/integration/v2_04g_r6_i1/compiled_scenes/compiled_scene_index.yaml",
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_activation_probe_listener.py",
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_mechanism_episode.py",
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_runtime_control.py",
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_candidate_materializer.py",
    "src/tools/thesis_experiment/scripts/generate_v2_04g_r6_i1_dependency_closure.py",
    "src/tools/thesis_experiment/src/thesis_experiment/v2_04g_r6_i1_dependency.py",
    "src/tools/thesis_experiment/src/thesis_experiment/v2_04g_r6_i1_execution_integrity.py",
    "src/tools/thesis_experiment/src/thesis_experiment/v2_04g_r6_integrity.py",
    "src/application/teb_mode_manager/scripts/r6_rule_context_supervisor_node.py",
    "src/application/teb_mode_manager/scripts/v2_04g_r6_typed_anchor_transaction_node.py",
    "src/application/teb_mode_manager/src/teb_mode_manager/r6_execution_integration.py",
    "src/application/teb_mode_manager/src/teb_mode_manager/r6_relative_ttc_supervisor.py",
    "src/application/teb_mode_manager/launch/v2_04g_r6_rule_supervisor.launch",
    "src/application/teb_mode_manager/launch/v2_04g_r6_simulation_typed_anchor.launch",
    "src/simulation/m2_gazebo/launch/m2_v2_04g_r6_execution_integration.launch",
)
RUNTIME_CONFIG_ROOT = (
    "artifacts/v2/integration/v2_04g_r6_i1/runtime_candidate_configs"
)
COMPILED_SCENE_ROOT = (
    "artifacts/v2/integration/v2_04g_r6_i1/compiled_scenes"
)
FIND_PATTERN = re.compile(r"\$\(find\s+([A-Za-z0-9_]+)\)/([^\"'<>\s]+)")
PYTHON_FILE_PATTERN = re.compile(
    r"(?:with_name|with_suffix)\(\s*[\"']([^\"']+\\.py)[\"']\s*\)"
)


class R6DependencyError(ValueError):
    """Raised when a local execution dependency cannot be resolved."""


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _relative(root, path):
    resolved = Path(path).resolve()
    try:
        return resolved.relative_to(root).as_posix()
    except ValueError as exc:
        raise R6DependencyError(
            "dependency leaves workspace: {}".format(resolved)
        ) from exc


def _package_index(root):
    result = {}
    for package_xml in root.rglob("package.xml"):
        if any(
            part in {".git", "build", "devel", "install", "log"}
            for part in package_xml.parts
        ):
            continue
        try:
            document = ET.parse(str(package_xml)).getroot()
        except (ET.ParseError, OSError):
            continue
        name = document.findtext("name")
        if name and name not in result:
            result[name] = package_xml.parent.resolve()
    return result


def _module_roots(packages):
    result = {}
    for name, package in packages.items():
        candidate = package / "src"
        if (candidate / name).exists():
            result[name] = candidate
    return result


def _resolve_python_module(module, module_roots):
    if not module:
        return None
    top = module.split(".")[0]
    root = module_roots.get(top)
    if root is None:
        return None
    path = root.joinpath(*module.split("."))
    if path.with_suffix(".py").is_file():
        return path.with_suffix(".py").resolve()
    if (path / "__init__.py").is_file():
        return (path / "__init__.py").resolve()
    return None


def _discover_python(path, module_roots):
    try:
        text = path.read_text(encoding="utf-8")
        tree = ast.parse(text, filename=str(path))
    except (OSError, UnicodeDecodeError, SyntaxError) as exc:
        raise R6DependencyError(
            "cannot parse Python dependency {}: {}".format(path, exc)
        ) from exc
    targets = set()
    external = set()
    for node in ast.walk(tree):
        modules = []
        if isinstance(node, ast.Import):
            modules.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.level:
                # Local relative imports are resolved from the nearest package
                # source root using their absolute package path below.
                for root_name, source_root in module_roots.items():
                    try:
                        relative_parent = path.parent.relative_to(source_root)
                    except ValueError:
                        continue
                    package_parts = list(relative_parent.parts)
                    trim = max(0, node.level - 1)
                    if trim:
                        package_parts = package_parts[:-trim]
                    suffix = node.module.split(".") if node.module else []
                    modules.append(
                        ".".join(package_parts + suffix)
                    )
                    break
            elif node.module:
                modules.append(node.module)
        for module in modules:
            resolved = _resolve_python_module(module, module_roots)
            if resolved is not None:
                targets.add(resolved)
            elif (
                module.endswith(".msg")
                and module.split(".")[0] in module_roots
            ):
                package = module_roots[module.split(".")[0]].parent
                message_files = set((package / "msg").glob("*.msg"))
                if not message_files:
                    raise R6DependencyError(
                        "generated ROS message import has no source: {}".format(
                            module
                        )
                    )
                targets.update(path.resolve() for path in message_files)
                for metadata in ("package.xml", "CMakeLists.txt"):
                    candidate = package / metadata
                    if candidate.is_file():
                        targets.add(candidate.resolve())
                external.add("generated-ros-message:" + module)
            elif module.split(".")[0] in module_roots:
                raise R6DependencyError(
                    "unresolved local Python import {} from {}".format(
                        module, path
                    )
                )
            else:
                external.add(module.split(".")[0])
    for match in PYTHON_FILE_PATTERN.finditer(text):
        candidate = path.parent / match.group(1)
        if candidate.is_file():
            targets.add(candidate.resolve())
        else:
            raise R6DependencyError(
                "unresolved dynamic Python dependency {} from {}".format(
                    match.group(1), path
                )
            )
    return targets, external


def _discover_text_references(path, packages):
    try:
        text = path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError) as exc:
        raise R6DependencyError(
            "cannot inspect text dependency {}: {}".format(path, exc)
        ) from exc
    targets = set()
    external = set()
    for package_name, relative in FIND_PATTERN.findall(text):
        package = packages.get(package_name)
        if package is None:
            external.add("$(find {})/{}".format(package_name, relative))
            continue
        candidate = (package / relative).resolve()
        if candidate.is_file():
            targets.add(candidate)
        else:
            raise R6DependencyError(
                "unresolved $(find {}) dependency {} from {}".format(
                    package_name, relative, path
                )
            )
    return targets, external


def _discover_launch(path, packages):
    targets, external = _discover_text_references(path, packages)
    try:
        launch = ET.parse(str(path)).getroot()
    except (ET.ParseError, OSError) as exc:
        raise R6DependencyError(
            "cannot parse launch {}: {}".format(path, exc)
        ) from exc
    local_packages = set()
    for node in launch.iter("node"):
        package_name = node.attrib.get("pkg", "")
        executable = node.attrib.get("type", "")
        package = packages.get(package_name)
        if package is None:
            external.add("node:{}:{}".format(package_name, executable))
            continue
        local_packages.add(package_name)
        candidates = (
            package / "scripts" / executable,
            package / "nodes" / executable,
            package / executable,
        )
        script = next((item for item in candidates if item.is_file()), None)
        if script is not None:
            targets.add(script.resolve())
        else:
            external.add(
                "workspace-built-node:{}:{}".format(
                    package_name, executable
                )
            )
    for package_name in local_packages:
        package = packages[package_name]
        for metadata in ("package.xml", "CMakeLists.txt"):
            candidate = package / metadata
            if candidate.is_file():
                targets.add(candidate.resolve())
    return targets, external


def _scene_children(root):
    index_path = root / COMPILED_SCENE_ROOT / "compiled_scene_index.yaml"
    try:
        index = yaml.safe_load(index_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, yaml.YAMLError) as exc:
        raise R6DependencyError(
            "cannot load compiled scene index: {}".format(exc)
        ) from exc
    children = set()
    for row in index.get("files", []):
        if not isinstance(row, dict) or set(row) != {"path", "sha256"}:
            raise R6DependencyError("compiled scene child schema drifted")
        path = (root / row["path"]).resolve()
        if not path.is_file() or _sha256(path) != row["sha256"]:
            raise R6DependencyError(
                "compiled scene child drifted: {}".format(row["path"])
            )
        children.add(path)
    return children


def build_dependency_closure(workspace):
    """Discover all local orchestration/config/scene dependencies."""

    root = Path(workspace).resolve()
    packages = _package_index(root)
    module_roots = _module_roots(packages)
    initial = set()
    for relative in ENTRYPOINTS + MANDATORY_EXECUTION_INPUTS:
        path = (root / relative).resolve()
        if not path.is_file():
            raise R6DependencyError(
                "mandatory dependency is missing: {}".format(relative)
            )
        initial.add(path)
    runtime_root = root / RUNTIME_CONFIG_ROOT
    runtime_files = set(runtime_root.glob("*/*.yaml"))
    if len(runtime_files) != 6:
        raise R6DependencyError("runtime config closure must contain six files")
    initial.update(path.resolve() for path in runtime_files)
    initial.update(_scene_children(root))

    files = set()
    edges = set()
    pending = list(initial)
    external_python = set()
    external_runtime = set()
    while pending:
        source = pending.pop()
        if source in files:
            continue
        files.add(source)
        targets = set()
        if source.suffix == ".py":
            discovered, external = _discover_python(source, module_roots)
            targets.update(discovered)
            external_python.update(external)
        if source.suffix in {".launch", ".xml", ".xacro"}:
            discovered, external = _discover_launch(source, packages)
            targets.update(discovered)
            external_runtime.update(external)
        elif source.suffix in {".yaml", ".yml", ".cfg", ".conf"}:
            discovered, external = _discover_text_references(
                source, packages
            )
            targets.update(discovered)
            external_runtime.update(external)
        for target in targets:
            edges.add((source, target, "python_import" if source.suffix == ".py"
                       and target.suffix == ".py" else "future_protocol_input"))
            if target not in files:
                pending.append(target)

    entrypoint_paths = [(root / value).resolve() for value in ENTRYPOINTS]
    primary = entrypoint_paths[0]
    # Mandatory data/config roots are deliberately attached to the real batch
    # entrypoint.  The discovered Python/launch edges then expand transitively.
    for target in files:
        if target != primary and not any(edge[1] == target for edge in edges):
            edges.add((primary, target, "future_protocol_input"))

    records = [
        {"path": _relative(root, path), "sha256": _sha256(path)}
        for path in sorted(files, key=lambda item: _relative(root, item))
    ]
    edge_rows = [
        {
            "from": _relative(root, source),
            "to": _relative(root, target),
            "kind": kind,
        }
        for source, target, kind in sorted(
            edges,
            key=lambda row: (
                _relative(root, row[0]),
                _relative(root, row[1]),
                row[2],
            ),
        )
    ]
    entrypoints = list(ENTRYPOINTS)
    payload = "".join(
        "{} {}\n".format(row["path"], row["sha256"])
        for row in records
    )
    payload += "".join(
        "entrypoint {}\n".format(path) for path in sorted(entrypoints)
    )
    payload += "".join(
        "{}\0{}\0{}\n".format(
            row["from"], row["to"], row["kind"]
        )
        for row in edge_rows
    )
    return {
        "schema_version": "2.0",
        "stage": "V2-04G-R6-I1",
        "generator": (
            "thesis_experiment.v2_04g_r6_i1_dependency."
            "build_dependency_closure"
        ),
        "entrypoints": entrypoints,
        "files": records,
        "edges": edge_rows,
        "unresolved": [],
        "external_python_modules": sorted(external_python),
        "external_runtime_bindings": sorted(external_runtime),
        "required_paths": [row["path"] for row in records],
        "closure_sha256": hashlib.sha256(
            payload.encode("utf-8")
        ).hexdigest(),
    }
