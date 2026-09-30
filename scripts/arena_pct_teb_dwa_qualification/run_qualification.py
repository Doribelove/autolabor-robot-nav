#!/usr/bin/env python3
"""Run the bounded Arena qualification matrix without touching Arena sources."""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import yaml

from build_arena_episode import (
    EXPECTED_SOURCE_WORLD_SHA256,
    MAP_YAML,
    ROOT,
    SOURCE_WORLD,
    build,
    sha256,
)


METHODS = ("pct_scope_rl", "analytic_prior", "actual_teb", "dwa")
CHECKPOINT = ROOT / (
    "artifacts/gazebo_pct_teb_dwa_benchmark/development/training/"
    "pct_benchmark_r1_seed9701/actor_final.pt"
)
TEB_CONFIG = ROOT / "src/simulation/m2_gazebo/config/benchmark_teb_v0.yaml"
DWA_CONFIG = ROOT / "src/simulation/m2_gazebo/config/benchmark_dwa_v0.yaml"
CONTRACT = ROOT / "config/thesis_experiments/arena/arena_pct_q1_contract.yaml"
DEFAULT_OUTPUT = ROOT / "artifacts/arena_pct_teb_dwa_qualification"
LAUNCH = "m2_arena_pct_teb_dwa_q1_episode.launch"

QUALIFICATION_SCHEDULE = (
    {
        "seed": 20310813,
        "scene": "hospital_static",
        "order": ("pct_scope_rl", "actual_teb", "analytic_prior", "dwa"),
    },
    {
        "seed": 20310814,
        "scene": "hospital_single_crossing",
        "order": ("actual_teb", "analytic_prior", "dwa", "pct_scope_rl"),
    },
    {
        "seed": 20310815,
        "scene": "hospital_single_crossing",
        "order": ("analytic_prior", "dwa", "pct_scope_rl", "actual_teb"),
    },
)


SOURCE_BINDINGS = (
    CONTRACT,
    Path(__file__).resolve(),
    Path(__file__).resolve().parent / "build_arena_episode.py",
    Path(__file__).resolve().parent / "analyze_qualification.py",
    Path(__file__).resolve().parent / "review_qualification_release.py",
    ROOT / "src/simulation/m2_gazebo/package.xml",
    ROOT / "src/application/v5_e2e_scope_rl/package.xml",
    ROOT / "src/application/v3_scope_planner/package.xml",
    ROOT
    / "src/application/v5_e2e_scope_rl/scripts/arena_pct_teb_dwa_qualification_episode.py",
    ROOT
    / "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/arena_benchmark_runtime.py",
    ROOT
    / "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/benchmark_runtime.py",
    ROOT
    / "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/runtime_adapter.py",
    ROOT
    / "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/asymmetric_ppo.py",
    ROOT
    / "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/sensor_env.py",
    ROOT
    / "src/application/v3_scope_planner/src/v3_scope_planner/execution_core.py",
    ROOT / "src/application/v3_scope_planner/src/v3_scope_planner/__init__.py",
    ROOT / "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/__init__.py",
    ROOT
    / "src/application/v5_e2e_scope_rl/scripts/pct_teb_dwa_benchmark_episode.py",
    ROOT
    / "src/application/v5_e2e_scope_rl/scripts/pct_teb_dwa_benchmark_r2_episode.py",
    ROOT / "src/simulation/m2_gazebo/launch/m2_arena_pct_teb_dwa_q1_episode.launch",
    ROOT / "src/simulation/m2_gazebo/launch/spawn_m2.launch",
    ROOT / "src/simulation/m2_gazebo/urdf/m2.urdf.xacro",
    ROOT / "src/simulation/m2_gazebo/urdf/m2_base.xacro",
    ROOT / "src/simulation/m2_gazebo/urdf/m2_sensors.xacro",
    ROOT / "src/simulation/m2_gazebo/urdf/m2_gazebo.xacro",
    ROOT / "src/simulation/m2_gazebo/src/m2_ackermann_plugin.cpp",
    ROOT / "src/simulation/m2_gazebo/include/m2_gazebo/ackermann_kinematics.h",
    ROOT / "src/simulation/m2_gazebo/include/m2_gazebo/actuator_dynamics.h",
    ROOT / "src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp",
    ROOT / "src/simulation/m2_gazebo/include/m2_gazebo/v3_reactive_actor.h",
    ROOT / "devel/lib/libm2_ackermann_plugin.so",
    ROOT / "devel/lib/libv3_reactive_actor_plugin.so",
    ROOT / "devel/lib/libteb_local_planner.so",
    ROOT
    / "src/navigation_arena/forks/navigation/local_planner/teb/teb_local_planner_plugin.xml",
    ROOT / "src/navigation_arena/forks/navigation/local_planner/teb/package.xml",
    ROOT / "devel/lib/v5_e2e_scope_rl/arena_pct_teb_dwa_qualification_episode.py",
    ROOT / "devel/lib/python3/dist-packages/v5_e2e_scope_rl/__init__.py",
    ROOT / "devel/lib/python3/dist-packages/v3_scope_planner/__init__.py",
    ROOT / "src/simulation/m2_gazebo/config/benchmark_move_base.yaml",
    ROOT / "src/simulation/m2_gazebo/config/costmap_common.yaml",
    ROOT / "src/simulation/m2_gazebo/config/arena_global_costmap.yaml",
    ROOT / "src/simulation/m2_gazebo/config/local_costmap.yaml",
    TEB_CONFIG,
    DWA_CONFIG,
    CHECKPOINT,
    SOURCE_WORLD,
    MAP_YAML,
    ROOT / "src/navigation_arena/arena-rosnav-3D/simulator_setup/maps/hospital/map.pgm",
)


SYSTEM_BINDINGS = (
    Path("/usr/bin/env"),
    Path("/bin/sh"),
    Path("/usr/bin/python3"),
    Path("/usr/bin/python3.8"),
    Path("/usr/bin/ss"),
    Path("/usr/bin/pkg-config"),
    Path("/usr/bin/gzserver"),
    Path("/opt/ros/noetic/bin/roslaunch"),
    Path("/opt/ros/noetic/bin/rosmaster"),
    Path("/opt/ros/noetic/bin/rospack"),
    Path("/opt/ros/noetic/bin/catkin_find"),
    Path("/opt/ros/noetic/bin/xacro"),
    Path("/opt/ros/noetic/lib/xacro/xacro"),
    Path("/opt/ros/noetic/lib/rosout/rosout"),
    Path("/opt/ros/noetic/lib/gazebo_ros/gzserver"),
    Path("/opt/ros/noetic/lib/gazebo_ros/libcommon.sh"),
    Path("/opt/ros/noetic/lib/gazebo_ros/spawn_model"),
    Path("/opt/ros/noetic/lib/libgazebo_ros_api_plugin.so"),
    Path("/opt/ros/noetic/lib/libgazebo_ros_paths_plugin.so"),
    Path("/opt/ros/noetic/lib/libgazebo_ros_laser.so"),
    Path("/opt/ros/noetic/lib/libgazebo_ros_bumper.so"),
    Path("/opt/ros/noetic/lib/libgazebo_ros_imu_sensor.so"),
    Path("/opt/ros/noetic/lib/libdwa_local_planner.so"),
    Path("/opt/ros/noetic/lib/libnavfn.so"),
    Path("/opt/ros/noetic/lib/libcostmap_2d.so"),
    Path("/opt/ros/noetic/lib/liblayers.so"),
    Path("/opt/ros/noetic/lib/move_base/move_base"),
    Path("/opt/ros/noetic/lib/map_server/map_server"),
    Path("/opt/ros/noetic/lib/robot_state_publisher/robot_state_publisher"),
    Path("/opt/ros/noetic/lib/tf2_ros/static_transform_publisher"),
    Path("/opt/ros/noetic/lib/topic_tools/relay"),
    Path("/opt/ros/noetic/share/gazebo_ros/launch/empty_world.launch"),
    Path("/opt/ros/noetic/share/gazebo_ros/package.xml"),
    Path("/opt/ros/noetic/share/gazebo_plugins/package.xml"),
    Path("/opt/ros/noetic/share/xacro/package.xml"),
    Path("/opt/ros/noetic/share/teb_local_planner/teb_local_planner_plugin.xml"),
    Path("/opt/ros/noetic/share/dwa_local_planner/blp_plugin.xml"),
    Path("/opt/ros/noetic/share/dwa_local_planner/package.xml"),
    Path("/opt/ros/noetic/share/navfn/bgp_plugin.xml"),
    Path("/opt/ros/noetic/share/navfn/package.xml"),
    Path("/opt/ros/noetic/share/costmap_2d/costmap_plugins.xml"),
    Path("/opt/ros/noetic/share/costmap_2d/package.xml"),
    Path("/usr/share/gazebo-11/models/sun/model.config"),
    Path("/usr/share/gazebo-11/models/sun/model.sdf"),
    Path("/usr/share/gazebo-11/models/ground_plane/model.config"),
    Path("/usr/share/gazebo-11/models/ground_plane/model.sdf"),
    Path("/usr/share/gazebo/setup.sh"),
    ROOT / ".venv/bin/python",
    ROOT / ".venv/bin/python3",
    ROOT / ".venv/bin/python3.8",
    ROOT / ".venv/pyvenv.cfg",
)


def process_group_is_alive(process_group):
    try:
        os.killpg(int(process_group), 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _process_state_and_starttime(pid):
    """Read Linux /proc starttime without exposing process environment."""
    payload = (Path("/proc") / str(int(pid)) / "stat").read_text(
        encoding="utf-8", errors="replace"
    )
    closing = payload.rfind(")")
    if closing < 0:
        raise RuntimeError("PROC_STAT_COMM_TERMINATOR_ABSENT")
    fields_after_comm = payload[closing + 2 :].split()
    if len(fields_after_comm) <= 19:
        raise RuntimeError("PROC_STAT_STARTTIME_ABSENT")
    return str(fields_after_comm[0]), str(fields_after_comm[19])


def _process_starttime(pid):
    return _process_state_and_starttime(pid)[1]


def episode_processes(ros_uri, gazebo_uri, allowed_scan_errors=()):
    """Find same-UID processes carrying either exact episode URI.

    Only PID, starttime, and the matched variable names are returned.  Values
    from /proc/*/environ are never persisted or printed.
    """
    expected = {
        b"ROS_MASTER_URI=" + str(ros_uri).encode("utf-8"): "ROS_MASTER_URI",
        b"GAZEBO_MASTER_URI=" + str(gazebo_uri).encode("utf-8"): (
            "GAZEBO_MASTER_URI"
        ),
    }
    records = []
    errors = []
    process_root = Path("/proc")
    own_pid = os.getpid()
    own_uid = os.getuid()
    for entry in sorted(
        (item for item in process_root.iterdir() if item.name.isdigit()),
        key=lambda item: int(item.name),
    ):
        pid = int(entry.name)
        if pid == own_pid:
            continue
        starttime = None
        try:
            if entry.stat().st_uid != own_uid:
                continue
            process_state, starttime = _process_state_and_starttime(pid)
            if process_state == "Z":
                continue
            entries = set((entry / "environ").read_bytes().split(b"\0"))
            matched = sorted(name for item, name in expected.items() if item in entries)
            if not matched:
                continue
            if _process_starttime(pid) != starttime:
                continue
            records.append(
                {
                    "pid": pid,
                    "starttime": starttime,
                    "matched_keys": matched,
                }
            )
        except (FileNotFoundError, ProcessLookupError):
            # Normal /proc race: the process exited while it was inspected.
            continue
        except (PermissionError, OSError) as error:
            errors.append(
                {
                    "pid": pid,
                    "starttime": starttime,
                    "error": type(error).__name__,
                    "errno": getattr(error, "errno", None),
                }
            )
    allowed = {
        (int(row["pid"]), str(row.get("starttime")), str(row["error"]))
        for row in allowed_scan_errors
    }
    unexpected = [
        row
        for row in errors
        if (int(row["pid"]), str(row.get("starttime")), str(row["error"]))
        not in allowed
    ]
    return {
        "processes": records,
        "scan_complete": not unexpected,
        "scan_errors": errors,
        "unexpected_scan_errors": unexpected,
    }


def _signal_episode_process(record, ros_uri, gazebo_uri, signal_value):
    pid = int(record["pid"])
    try:
        audit = episode_processes(ros_uri, gazebo_uri)
    except Exception:
        return False
    current = next(
        (row for row in audit["processes"] if int(row["pid"]) == pid), None
    )
    if current is None or str(current["starttime"]) != str(record["starttime"]):
        return False
    try:
        os.kill(pid, signal_value)
    except ProcessLookupError:
        return False
    return True


def port_probe(ports):
    try:
        completed = subprocess.run(
            ["/usr/bin/ss", "-ltn"],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError) as error:
        return {
            "probe_complete": False,
            "active_ports": [],
            "error": type(error).__name__,
        }
    active = []
    for port in ports:
        suffix = ":{}".format(int(port))
        if any(
            len(line.split()) >= 4 and line.split()[3].endswith(suffix)
            for line in completed.stdout.splitlines()[1:]
        ):
            active.append(int(port))
    return {"probe_complete": True, "active_ports": active, "error": None}


def stop(
    process,
    process_group,
    ros_uri,
    gazebo_uri,
    ros_port,
    gazebo_port,
    allowed_scan_errors=(),
):
    return_code = process.poll()
    signal_rounds = []
    clean_streak = 0
    for signal_value, deadline_s in (
        (signal.SIGINT, 15.0),
        (signal.SIGTERM, 5.0),
        (signal.SIGKILL, 2.0),
    ):
        group_signaled = False
        if process_group_is_alive(process_group):
            try:
                os.killpg(process_group, signal_value)
                group_signaled = True
            except ProcessLookupError:
                pass
        signaled_identities = set()
        signaled_rows = []
        deadline = time.monotonic() + deadline_s
        while time.monotonic() < deadline:
            process.poll()
            uri_audit = episode_processes(
                ros_uri, gazebo_uri, allowed_scan_errors
            )
            for row in uri_audit["processes"]:
                identity = (int(row["pid"]), str(row["starttime"]))
                if identity in signaled_identities:
                    continue
                signaled_identities.add(identity)
                if _signal_episode_process(
                    row, ros_uri, gazebo_uri, signal_value
                ):
                    signaled_rows.append(
                        {
                            "pid": identity[0],
                            "starttime": identity[1],
                            "matched_keys": list(row["matched_keys"]),
                        }
                    )
            ports = port_probe((ros_port, gazebo_port))
            clean = bool(
                not process_group_is_alive(process_group)
                and uri_audit["scan_complete"]
                and not uri_audit["processes"]
                and ports["probe_complete"]
                and not ports["active_ports"]
            )
            clean_streak = clean_streak + 1 if clean else 0
            if clean_streak >= 3:
                break
            time.sleep(0.10)
        signal_rounds.append(
            {
                "signal": int(signal_value),
                "group_signaled": group_signaled,
                "uri_processes_signaled": signaled_rows,
            }
        )
        if clean_streak >= 3:
            break
    final_uri = episode_processes(ros_uri, gazebo_uri, allowed_scan_errors)
    final_ports = port_probe((ros_port, gazebo_port))
    final_return_code = process.poll()
    process_group_clear = not process_group_is_alive(process_group)
    uri_clear = not final_uri["processes"]
    ports_clear = not final_ports["active_ports"]
    cleanup_complete = bool(
        process_group_clear
        and final_uri["scan_complete"]
        and uri_clear
        and final_ports["probe_complete"]
        and ports_clear
    )
    return {
        "process_group_id": int(process_group),
        "process_group_clear": process_group_clear,
        "uri_processes_clear": uri_clear,
        "uri_processes": final_uri["processes"],
        "uri_scan_complete": final_uri["scan_complete"],
        "uri_scan_errors": final_uri["scan_errors"],
        "unexpected_uri_scan_errors": final_uri["unexpected_scan_errors"],
        "ports_clear": ports_clear,
        "active_ports": final_ports["active_ports"],
        "port_probe_complete": final_ports["probe_complete"],
        "port_probe_error": final_ports["error"],
        "signal_rounds": signal_rounds,
        "cleanup_complete": cleanup_complete,
        "return_code": (
            final_return_code if final_return_code is not None else return_code
        ),
    }


def port_is_listening(port):
    probe = port_probe((port,))
    if not probe["probe_complete"]:
        raise RuntimeError("EPISODE_PORT_PROBE_FAILED:" + str(probe["error"]))
    return int(port) in probe["active_ports"]


def require_isolation(ros_port, gazebo_port):
    ros_uri = "http://127.0.0.1:{}".format(int(ros_port))
    gazebo_uri = "http://127.0.0.1:{}".format(int(gazebo_port))
    ports = port_probe((ros_port, gazebo_port))
    first_scan = episode_processes(ros_uri, gazebo_uri)
    time.sleep(0.05)
    second_scan = episode_processes(ros_uri, gazebo_uri)
    first_errors = {
        (int(row["pid"]), str(row.get("starttime")), str(row["error"]))
        for row in first_scan["scan_errors"]
    }
    second_errors = {
        (int(row["pid"]), str(row.get("starttime")), str(row["error"]))
        for row in second_scan["scan_errors"]
    }
    stable_unreadable_baseline = first_errors == second_errors
    processes = episode_processes(
        ros_uri, gazebo_uri, second_scan["scan_errors"]
    )
    if (
        not ports["probe_complete"]
        or ports["active_ports"]
        or not stable_unreadable_baseline
        or not processes["scan_complete"]
        or processes["processes"]
    ):
        raise RuntimeError(
            "ARENA_EPISODE_PRE_ISOLATION_FAILED:ports={}:pids={}:scan_complete={}:port_probe_complete={}".format(
                ports["active_ports"],
                [row["pid"] for row in processes["processes"]],
                processes["scan_complete"],
                ports["probe_complete"],
            )
        )
    return {
        "ros_uri": ros_uri,
        "gazebo_uri": gazebo_uri,
        "uri_scan_complete": True,
        "port_probe_complete": True,
        "allowed_uri_scan_errors": second_scan["scan_errors"],
    }


def runtime_environment(runtime_root, ros_port, gazebo_port):
    # Whitelist the ROS/Gazebo execution contract. roslaunch writes its child
    # environment to ROS_LOG_DIR, so no parent path or credential is inherited.
    arena_world_root = SOURCE_WORLD.parents[1]
    arena_model_root = arena_world_root / "models"
    runtime_home = (runtime_root / "home").resolve()
    runtime_home.mkdir(parents=True, exist_ok=True)
    (runtime_root / "roslog").mkdir(parents=True, exist_ok=True)
    (runtime_root / "ros_home").mkdir(parents=True, exist_ok=True)
    return {
        "PATH": os.pathsep.join(
            (
                str((ROOT / ".venv/bin").resolve()),
                "/opt/ros/noetic/bin",
                "/usr/local/sbin",
                "/usr/local/bin",
                "/usr/sbin",
                "/usr/bin",
                "/sbin",
                "/bin",
            )
        ),
        "HOME": str(runtime_home),
        "USER": "robot",
        "LOGNAME": "robot",
        "SHELL": "/bin/bash",
        "LANG": "C.UTF-8",
        "LC_ALL": "C.UTF-8",
        "PYTHONNOUSERSITE": "1",
        "PYTHONHASHSEED": "0",
        "PYTHONPATH": os.pathsep.join(
            (
                str((ROOT / "devel/lib/python3/dist-packages").resolve()),
                "/opt/ros/noetic/lib/python3/dist-packages",
            )
        ),
        "ROS_ROOT": "/opt/ros/noetic/share/ros",
        "ROS_DISTRO": "noetic",
        "ROS_VERSION": "1",
        "ROS_PYTHON_VERSION": "3",
        "ROS_ETC_DIR": "/etc/ros",
        "ROS_PACKAGE_PATH": os.pathsep.join(
            (str((ROOT / "src").resolve()), "/opt/ros/noetic/share")
        ),
        "CMAKE_PREFIX_PATH": os.pathsep.join(
            (str((ROOT / "devel").resolve()), "/opt/ros/noetic")
        ),
        "LD_LIBRARY_PATH": os.pathsep.join(
            (
                str((ROOT / "devel/lib").resolve()),
                "/opt/ros/noetic/lib",
                "/opt/ros/noetic/lib/x86_64-linux-gnu",
                "/usr/local/cuda-11.x/lib64",
            )
        ),
        "PKG_CONFIG_PATH": os.pathsep.join(
            (
                str((ROOT / "devel/lib/pkgconfig").resolve()),
                "/opt/ros/noetic/lib/pkgconfig",
                "/opt/ros/noetic/lib/x86_64-linux-gnu/pkgconfig",
            )
        ),
        "THESIS_WS": str(ROOT),
        "ROS_HOME": str((runtime_root / "ros_home").resolve()),
        "ROS_LOG_DIR": str((runtime_root / "roslog").resolve()),
        "ROS_MASTER_URI": "http://127.0.0.1:{}".format(ros_port),
        "ROS_HOSTNAME": "127.0.0.1",
        "GAZEBO_MASTER_URI": "http://127.0.0.1:{}".format(gazebo_port),
        "GAZEBO_MODEL_DATABASE_URI": "",
        "GAZEBO_MODEL_PATH": os.pathsep.join(
            (str(arena_model_root), "/usr/share/gazebo-11/models")
        ),
        "GAZEBO_RESOURCE_PATH": os.pathsep.join(
            (str(arena_world_root), "/usr/share/gazebo-11")
        ),
        "GAZEBO_PLUGIN_PATH": os.pathsep.join(
            (str((ROOT / "devel/lib").resolve()), "/opt/ros/noetic/lib")
        ),
    }


def resource_record(path):
    absolute = Path(path).absolute()
    try:
        value = str(absolute.relative_to(ROOT))
        scope = "workspace_relative"
    except ValueError:
        value = str(absolute)
        scope = "absolute_system"
    record = {
        "scope": scope,
        "path": value,
        "size_bytes": absolute.stat().st_size,
        "sha256": sha256(absolute),
    }
    if absolute.is_symlink():
        record["symlink_target"] = os.readlink(str(absolute))
    return record


def resolve_resource_record(row):
    if row.get("scope") == "workspace_relative":
        return (ROOT / str(row["path"])).absolute()
    if row.get("scope") == "absolute_system":
        return Path(str(row["path"])).absolute()
    raise RuntimeError("Q1_RELEASE_RESOURCE_SCOPE_INVALID:" + str(row))


def release_resource_rows(release_path):
    release = yaml.safe_load(Path(release_path).read_text(encoding="utf-8"))
    rows = []
    for name in ("resources", "system_resources", "hospital_assets"):
        rows.extend(list(release.get(name, [])))
    return rows


def read_yaml_with_sha256(path):
    payload = Path(path).read_bytes()
    observed_sha256 = hashlib.sha256(payload).hexdigest()
    return yaml.safe_load(payload), observed_sha256


def read_json_with_sha256(path):
    payload = Path(path).read_bytes()
    observed_sha256 = hashlib.sha256(payload).hexdigest()
    return json.loads(payload.decode("utf-8")), observed_sha256


def verify_release_resources(release_path, expected_release_sha256=None):
    release, observed_release_sha256 = read_yaml_with_sha256(release_path)
    if (
        expected_release_sha256 is not None
        and observed_release_sha256 != str(expected_release_sha256)
    ):
        raise RuntimeError(
            "Q1_RELEASE_SHA_MISMATCH:" + observed_release_sha256
        )
    expected_categories = {
        "resources": [resource_record(path) for path in SOURCE_BINDINGS],
        "system_resources": [resource_record(path) for path in SYSTEM_BINDINGS],
        "hospital_assets": hospital_asset_manifest(),
    }
    failures = []
    seen = set()
    for category, expected_rows in expected_categories.items():
        observed_rows = list(release.get(category, []))
        if observed_rows != expected_rows:
            failures.append(category + ":exact_roster_mismatch")
        for row in observed_rows:
            path = resolve_resource_record(row)
            identifier = (str(row.get("scope")), str(path))
            if identifier in seen:
                failures.append(str(path) + ":duplicate")
                continue
            seen.add(identifier)
            if not path.is_file():
                failures.append(str(path) + ":absent")
                continue
            if "symlink_target" in row and (
                not path.is_symlink()
                or os.readlink(str(path)) != str(row["symlink_target"])
            ):
                failures.append(str(path) + ":symlink_target_mismatch")
            observed = sha256(path)
            if observed != str(row["sha256"]):
                failures.append(str(path) + ":hash_mismatch:" + observed)
            if int(path.stat().st_size) != int(row["size_bytes"]):
                failures.append(str(path) + ":size_mismatch")
    if sha256(SOURCE_WORLD) != EXPECTED_SOURCE_WORLD_SHA256:
        failures.append(str(SOURCE_WORLD) + ":frozen_source_hash_mismatch")
    if failures:
        raise RuntimeError("Q1_RELEASE_REHASH_FAILED:" + ",".join(failures))
    return release, observed_release_sha256


def hospital_asset_manifest():
    hospital_root = SOURCE_WORLD.parents[1]
    roots = (hospital_root / "models", hospital_root / "photos")
    return [
        resource_record(path)
        for root in roots
        for path in sorted(item for item in root.rglob("*") if item.is_file())
    ]


def release_payload(stage):
    missing = [
        str(path)
        for path in tuple(SOURCE_BINDINGS) + tuple(SYSTEM_BINDINGS)
        if not Path(path).is_file()
    ]
    if missing:
        raise RuntimeError("RELEASE_RESOURCE_ABSENT:" + ",".join(missing))
    resources = [resource_record(path) for path in SOURCE_BINDINGS]
    system_resources = [resource_record(path) for path in SYSTEM_BINDINGS]
    hospital_assets = hospital_asset_manifest()
    return {
        "schema_version": "1.0",
        "stage": stage,
        "simulation_only": True,
        "execution_authorized_by_current_user_instruction": True,
        "formal_result": False,
        "qualification_only": True,
        "candidate_variant": "r2_candidate_v060",
        "runtime_versions": {
            "ros_distribution": "noetic",
            "gazebo": "11.15.1",
            "gazebo_ros": "2.9.3",
            "workspace_teb_local_planner": "0.8.4",
            "dwa_local_planner": "1.17.3",
            "python": "3.8",
            "python_packages": {
                name: importlib.metadata.version(name)
                for name in (
                    "numpy",
                    "torch",
                    "scipy",
                    "Pillow",
                    "PyYAML",
                    "gymnasium",
                    "matplotlib",
                )
            },
        },
        "schedule": [
            {
                "seed": row["seed"],
                "scene": row["scene"],
                "method_order": list(row["order"]),
            }
            for row in QUALIFICATION_SCHEDULE
        ],
        "resources": resources,
        "system_resources": system_resources,
        "hospital_assets": hospital_assets,
        "resource_roster_counts": {
            "workspace_execution_resources": len(resources),
            "selected_system_runtime_resources": len(system_resources),
            "hospital_assets": len(hospital_assets),
            "total": len(resources) + len(system_resources) + len(hospital_assets),
        },
        "resource_roster_scope": (
            "exact_selected_execution_roster_not_full_transitive_os_or_python_closure"
        ),
        "runtime_environment_policy": (
            "explicit_whitelist_no_parent_paths_no_user_site_ephemeral_home"
        ),
        "execution_protocol": "two_phase_prepare_review_then_execute",
    }


def write_release(output_root, stage):
    release_path = output_root / "q1_execution_release.yaml"
    if release_path.exists():
        raise RuntimeError("Q1_RELEASE_ALREADY_EXISTS:" + str(release_path))
    output_root.mkdir(parents=True, exist_ok=True)
    payload = release_payload(stage)
    temporary = release_path.with_suffix(".yaml.tmp")
    temporary.write_text(
        yaml.safe_dump(payload, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    os.replace(str(temporary), str(release_path))
    verify_release_resources(release_path)
    return release_path


def write_atomic_json(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    os.replace(str(temporary), str(path))


def raw_evidence_records(output_root):
    output_root = Path(output_root).resolve()
    q1_root = output_root / "q1"
    fixed = (
        output_root / "q1_execution_release.yaml",
        output_root / "q1_release_review.json",
    )
    candidates = list(fixed)
    if q1_root.is_dir():
        candidates.extend(q1_root.rglob("*"))
    records = []
    for path in sorted(set(Path(item) for item in candidates)):
        relative = path.relative_to(output_root)
        if relative.parts[:2] == ("q1", "analysis"):
            continue
        if path.is_symlink():
            target_text = os.readlink(str(path))
            resolved_target = (path.parent / target_text).resolve()
            try:
                resolved_target.relative_to(output_root)
            except ValueError as error:
                raise RuntimeError(
                    "RAW_EVIDENCE_SYMLINK_TARGET_OUTSIDE_ROOT:" + str(path)
                ) from error
            records.append(
                {
                    "path": str(relative),
                    "type": "symlink",
                    "size_bytes": int(path.lstat().st_size),
                    "link_target": target_text,
                }
            )
        elif path.is_file():
            records.append(
                {
                    "path": str(relative),
                    "type": "file",
                    "size_bytes": int(path.stat().st_size),
                    "sha256": sha256(path),
                }
            )
    return records


def write_raw_evidence_closure(
    output_root,
    release_sha256,
    release_review_sha256,
    terminal_journal_sha256,
    matrix_receipt_sha256,
    identities,
):
    output_root = Path(output_root).resolve()
    manifest_path = output_root / "q1_raw_evidence_manifest.json"
    seal_path = output_root / "q1_raw_evidence_seal.json"
    if manifest_path.exists() or seal_path.exists():
        raise RuntimeError("Q1_RAW_EVIDENCE_CLOSURE_ALREADY_EXISTS")
    records = raw_evidence_records(output_root)
    manifest = {
        "schema_version": "1.0",
        "stage": "ARENA-PCT-Q1",
        "release_sha256": str(release_sha256),
        "release_review_sha256": str(release_review_sha256),
        "terminal_journal_sha256": str(terminal_journal_sha256),
        "matrix_receipt_sha256": str(matrix_receipt_sha256),
        "path_set_policy": (
            "release_review_and_q1_raw_tree_excluding_q1_analysis_and_closure_files"
        ),
        "files": records,
    }
    write_atomic_json(manifest_path, manifest)
    manifest_sha256 = sha256(manifest_path)
    seal = {
        "schema_version": "1.0",
        "stage": "ARENA-PCT-Q1-RAW-EVIDENCE-SEAL",
        "terminal": True,
        "release_sha256": str(release_sha256),
        "release_review_sha256": str(release_review_sha256),
        "terminal_journal_sha256": str(terminal_journal_sha256),
        "matrix_receipt_sha256": str(matrix_receipt_sha256),
        "raw_evidence_manifest_sha256": manifest_sha256,
        "raw_file_count": len(records),
        "episode_identities": list(identities),
    }
    write_atomic_json(seal_path, seal)
    return manifest_path, seal_path


def write_q1_journal(
    path,
    release_sha256,
    release_review_sha256,
    episodes,
    terminal=False,
    in_progress=None,
    aborted=False,
    abort_reason=None,
):
    write_atomic_json(
        path,
        {
            "schema_version": "1.0",
            "stage": "ARENA-PCT-Q1",
            "attempt_policy": "attempt_1_only_no_retry_no_replacement",
            "terminal": bool(terminal),
            "aborted": bool(aborted),
            "abort_reason": abort_reason,
            "in_progress": in_progress,
            "episodes": list(episodes),
            "source_world_sha256_after": sha256(SOURCE_WORLD),
            "release_sha256": str(release_sha256),
            "release_review_sha256": str(release_review_sha256),
        },
    )


def execute_episode(
    *,
    block_root,
    world_path,
    manifest_path,
    scene,
    seed,
    method,
    stage,
    port,
    release_path=None,
    release_sha256=None,
    release_review_sha256=None,
):
    if release_path is not None:
        verify_release_resources(
            release_path, expected_release_sha256=release_sha256
        )
    isolation = require_isolation(port, port + 1)
    episode_root = block_root / method
    episode_root.mkdir(parents=True, exist_ok=False)
    report_path = episode_root / "episode_report.yaml"
    log_path = episode_root / "roslaunch.log"
    runtime_root = episode_root / "runtime"
    environment = runtime_environment(runtime_root, port, port + 1)
    forbidden_child_names = [
        name
        for name in environment
        if any(
            token in name.upper()
            for token in ("KEY", "TOKEN", "SECRET", "PASSWORD", "CREDENTIAL")
        )
    ]
    if forbidden_child_names:
        raise RuntimeError(
            "CREDENTIAL_LIKE_CHILD_ENVIRONMENT_NAMES:"
            + ",".join(sorted(forbidden_child_names))
        )
    command = [
        "roslaunch",
        "m2_gazebo",
        LAUNCH,
        "world:={}".format(world_path.resolve()),
        "method:={}".format(method),
        "scene_manifest:={}".format(manifest_path.resolve()),
        "map_yaml:={}".format(MAP_YAML.resolve()),
        "scene_id:={}".format(scene),
        "seed:={}".format(seed),
        "stage:={}".format(stage),
        "report_path:={}".format(report_path.resolve()),
        "actor_checkpoint_path:={}".format(
            CHECKPOINT.resolve() if method == "pct_scope_rl" else ""
        ),
        "teb_config:={}".format(TEB_CONFIG.resolve()),
        "dwa_config:={}".format(DWA_CONFIG.resolve()),
        "gui:=false",
        "release_sha256:={}".format(release_sha256 or ""),
        "release_review_sha256:={}".format(release_review_sha256 or ""),
    ]
    binding = {
        "schema_version": "1.0",
        "stage": stage,
        "scene_id": scene,
        "seed": int(seed),
        "method": method,
        "attempt": 1,
        "ros_port": int(port),
        "gazebo_port": int(port + 1),
        "ros_master_uri": isolation["ros_uri"],
        "gazebo_master_uri": isolation["gazebo_uri"],
        "world_sha256": sha256(world_path),
        "scene_manifest_sha256": sha256(manifest_path),
        "release_path": str(release_path.relative_to(ROOT)) if release_path else None,
        "release_sha256": sha256(release_path) if release_path else None,
        "release_review_sha256": release_review_sha256,
        "checkpoint_sha256": sha256(CHECKPOINT) if method == "pct_scope_rl" else None,
        "planner_config_sha256": (
            sha256(TEB_CONFIG)
            if method == "actual_teb"
            else sha256(DWA_CONFIG) if method == "dwa" else None
        ),
        "command": command,
        "child_environment_names": sorted(environment),
        "child_environment": dict(sorted(environment.items())),
        "credential_like_child_environment_names": forbidden_child_names,
    }
    binding_path = episode_root / "preflight_binding.json"
    binding_path.write_text(
        json.dumps(binding, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    started = time.monotonic()
    timed_out = False
    return_code = None
    cleanup = {
        "process_group_id": None,
        "process_group_clear": False,
        "uri_processes_clear": False,
        "uri_processes": [],
        "uri_scan_complete": False,
        "uri_scan_errors": [],
        "unexpected_uri_scan_errors": [],
        "ports_clear": False,
        "active_ports": [],
        "port_probe_complete": False,
        "port_probe_error": None,
        "signal_rounds": [],
        "cleanup_complete": False,
        "return_code": None,
    }
    with log_path.open("wb") as log:
        process = subprocess.Popen(
            command,
            cwd=str(ROOT),
            env=environment,
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        process_group = process.pid
        try:
            return_code = process.wait(timeout=240.0)
        except subprocess.TimeoutExpired:
            timed_out = True
        finally:
            cleanup = stop(
                process,
                process_group,
                isolation["ros_uri"],
                isolation["gazebo_uri"],
                port,
                port + 1,
                isolation["allowed_uri_scan_errors"],
            )
            if return_code is None:
                return_code = cleanup["return_code"]
    receipt = {
        "schema_version": "1.0",
        "stage": stage,
        "scene_id": scene,
        "seed": int(seed),
        "method": method,
        "attempt": 1,
        "return_code": return_code,
        "wall_elapsed_s": time.monotonic() - started,
        "runner_timeout": timed_out,
        "process_group_id": cleanup["process_group_id"],
        "post_process_group_clear": cleanup["process_group_clear"],
        "ros_port": int(port),
        "gazebo_port": int(port + 1),
        "post_uri_processes_clear": cleanup["uri_processes_clear"],
        "post_uri_processes": cleanup["uri_processes"],
        "post_uri_scan_complete": cleanup["uri_scan_complete"],
        "post_uri_scan_errors": cleanup["uri_scan_errors"],
        "post_unexpected_uri_scan_errors": cleanup[
            "unexpected_uri_scan_errors"
        ],
        "pre_isolation_unreadable_processes": isolation[
            "allowed_uri_scan_errors"
        ],
        "post_ports_clear": cleanup["ports_clear"],
        "post_active_ports": cleanup["active_ports"],
        "post_port_probe_complete": cleanup["port_probe_complete"],
        "post_port_probe_error": cleanup["port_probe_error"],
        "cleanup_signal_rounds": cleanup["signal_rounds"],
        "cleanup_complete": cleanup["cleanup_complete"],
        "report_present": report_path.is_file(),
        "report_sha256": sha256(report_path) if report_path.is_file() else None,
        "roslaunch_log_sha256": sha256(log_path),
        "preflight_binding_sha256": sha256(binding_path),
        "source_world_sha256_after": sha256(SOURCE_WORLD),
        "release_sha256": release_sha256,
        "release_review_sha256": release_review_sha256,
    }
    receipt_path = episode_root / "receipt.json"
    receipt_path.write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    report = None
    if report_path.is_file():
        report = yaml.safe_load(report_path.read_text(encoding="utf-8"))
    if not cleanup["cleanup_complete"]:
        raise RuntimeError(
            "ARENA_EPISODE_POST_ISOLATION_FAILED:group={}:uri={}:scan={}:ports={}:probe={}".format(
                cleanup["process_group_clear"],
                cleanup["uri_processes"],
                cleanup["uri_scan_complete"],
                cleanup["active_ports"],
                cleanup["port_probe_complete"],
            )
        )
    return receipt, report


def run_development(args):
    if not 20310801 <= args.seed <= 20310812:
        raise RuntimeError("DEVELOPMENT_SEED_OUTSIDE_20310801_20310812")
    stage = "ARENA-PCT-DEVELOPMENT"
    block_root = args.output.resolve() / "development" / (
        "{}_{}".format(args.seed, args.scene)
    )
    world_path, manifest_path, _ = build(
        args.scene, args.seed, block_root / "snapshot", stage
    )
    receipt, report = execute_episode(
        block_root=block_root,
        world_path=world_path,
        manifest_path=manifest_path,
        scene=args.scene,
        seed=args.seed,
        method=args.method,
        stage=stage,
        port=args.port,
    )
    print(json.dumps(receipt, indent=2, sort_keys=True))
    if report:
        print(
            "goal={} integrity={} restricted_s={} clearance_m={}".format(
                report["result"]["goal_reached"],
                report["execution_integrity_pass"],
                report["result"]["restricted_elapsed_s"],
                report["result"]["minimum_conservative_clearance_m"],
            )
        )
    return 0 if report is not None else 2


def require_safe_output_root(output_root):
    output_root = Path(output_root).resolve()
    try:
        output_root.relative_to(ROOT.resolve())
    except ValueError as error:
        raise RuntimeError("Q1_OUTPUT_MUST_BE_INSIDE_THESIS_WORKSPACE") from error
    try:
        output_root.relative_to(
            (ROOT / "src/navigation_arena/arena-rosnav-3D").resolve()
        )
    except ValueError:
        return output_root
    raise RuntimeError("Q1_OUTPUT_INSIDE_ARENA_SOURCE_TREE_FORBIDDEN")


def prepare_qualification(args):
    output_root = require_safe_output_root(args.output)
    q1_root = output_root / "q1"
    review_path = output_root / "q1_release_review.json"
    if q1_root.exists() or review_path.exists():
        raise RuntimeError("Q1_PREPARE_TARGET_NOT_FRESH")
    release_path = write_release(output_root, "ARENA-PCT-Q1")
    print(
        json.dumps(
            {
                "release_path": str(release_path.relative_to(ROOT)),
                "release_sha256": sha256(release_path),
                "next_step": "run_review_qualification_release_before_execution",
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


def require_release_review(
    output_root,
    release_path,
    expected_release_sha256,
    expected_release_review_sha256,
):
    release, observed_release_sha256 = verify_release_resources(
        release_path, expected_release_sha256=expected_release_sha256
    )
    if observed_release_sha256 != str(expected_release_sha256):
        raise RuntimeError(
            "Q1_RELEASE_SHA_ARGUMENT_MISMATCH:" + observed_release_sha256
        )
    review_path = output_root / "q1_release_review.json"
    if not review_path.is_file():
        raise RuntimeError("Q1_RELEASE_REVIEW_ABSENT")
    review, observed_review_sha256 = read_json_with_sha256(review_path)
    if observed_review_sha256 != str(expected_release_review_sha256):
        raise RuntimeError(
            "Q1_RELEASE_REVIEW_SHA_ARGUMENT_MISMATCH:"
            + observed_review_sha256
        )
    expected_counts = {
        name: len(release.get(name, []))
        for name in ("resources", "system_resources", "hospital_assets")
    }
    expected_roster_sha256 = hashlib.sha256(
        json.dumps(
            {
                name: release.get(name, [])
                for name in ("resources", "system_resources", "hospital_assets")
            },
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
    ).hexdigest()
    if (
        review.get("status") != "arena_q1_release_preflight_pass"
        or review.get("release_sha256") != observed_release_sha256
        or not review.get("execution_ready")
        or review.get("stage") != "ARENA-PCT-Q1-RELEASE-REVIEW"
        or review.get("exact_release_payload_match") is not True
        or review.get("exact_resource_roster_match") is not True
        or review.get("resource_category_counts") != expected_counts
        or review.get("resource_roster_sha256") != expected_roster_sha256
        or review.get("resource_count") != sum(expected_counts.values())
        or review.get("source_world_hash_pass") is not True
        or review.get("q1_execution_absent_at_review") is not True
        or review.get("credential_like_child_environment_names") != []
        or review.get("external_workspace_path_leaks") != []
        or review.get("episode_identities") != 12
        or review.get("formal_result") is not False
        or review.get("qualification_only") is not True
        or review.get("release_path") != str(release_path.relative_to(ROOT))
        or review.get("reviewer_source_sha256")
        != sha256(
            Path(__file__).resolve().parent
            / "review_qualification_release.py"
        )
    ):
        raise RuntimeError("Q1_RELEASE_REVIEW_INVALID")
    return review_path, review, observed_review_sha256


def run_qualification(args):
    output_root = require_safe_output_root(args.output)
    q1_root = output_root / "q1"
    release_path = output_root / "q1_execution_release.yaml"
    if not release_path.is_file():
        raise RuntimeError("Q1_RELEASE_ABSENT_RUN_PREPARE_FIRST")
    if q1_root.exists():
        raise RuntimeError("Q1_EXECUTION_ROOT_ALREADY_EXISTS_NONRESUMABLE")
    review_path, _review, frozen_review_sha256 = require_release_review(
        output_root,
        release_path,
        args.release_sha256,
        args.release_review_sha256,
    )
    frozen_release_sha256 = str(args.release_sha256)
    verify_release_resources(
        release_path, expected_release_sha256=frozen_release_sha256
    )
    matrix = []
    journal_path = q1_root / "matrix_journal.json"
    write_q1_journal(
        journal_path,
        frozen_release_sha256,
        frozen_review_sha256,
        matrix,
        terminal=False,
        in_progress=None,
    )
    episode_index = 0
    for block in QUALIFICATION_SCHEDULE:
        block_root = q1_root / "blocks" / (
            "{}_{}".format(block["seed"], block["scene"])
        )
        world_path, manifest_path, _ = build(
            block["scene"],
            block["seed"],
            block_root / "snapshot",
            "ARENA-PCT-Q1",
            frozen_release_sha256,
            frozen_review_sha256,
        )
        for method in block["order"]:
            port = args.port + episode_index * 4
            episode_index += 1
            identity = {
                "stage": "ARENA-PCT-Q1",
                "scene_id": block["scene"],
                "seed": block["seed"],
                "method": method,
                "attempt": 1,
                "episode_index": episode_index,
                "ros_port": port,
                "gazebo_port": port + 1,
            }
            write_q1_journal(
                journal_path,
                frozen_release_sha256,
                frozen_review_sha256,
                matrix,
                terminal=False,
                in_progress=identity,
            )
            try:
                verify_release_resources(
                    release_path,
                    expected_release_sha256=frozen_release_sha256,
                )
                receipt, report = execute_episode(
                    block_root=block_root,
                    world_path=world_path,
                    manifest_path=manifest_path,
                    scene=block["scene"],
                    seed=block["seed"],
                    method=method,
                    stage="ARENA-PCT-Q1",
                    port=port,
                    release_path=release_path,
                    release_sha256=frozen_release_sha256,
                    release_review_sha256=frozen_review_sha256,
                )
                receipt_path = block_root / method / "receipt.json"
                matrix.append(
                    {
                        **receipt,
                        "receipt_path": str(receipt_path.relative_to(ROOT)),
                        "receipt_sha256": sha256(receipt_path),
                        "goal_reached": (
                            bool(report["result"]["goal_reached"]) if report else False
                        ),
                        "execution_integrity_pass": (
                            bool(report["execution_integrity_pass"]) if report else False
                        ),
                    }
                )
            except Exception as error:
                write_q1_journal(
                    journal_path,
                    frozen_release_sha256,
                    frozen_review_sha256,
                    matrix,
                    terminal=True,
                    in_progress=identity,
                    aborted=True,
                    abort_reason=type(error).__name__ + ":" + str(error),
                )
                raise
            status = matrix[-1]
            write_q1_journal(
                journal_path,
                frozen_release_sha256,
                frozen_review_sha256,
                matrix,
                terminal=False,
                in_progress=None,
            )
            print(
                "{}/12 seed={} scene={} method={} report={} goal={} integrity={}".format(
                    episode_index,
                    block["seed"],
                    block["scene"],
                    method,
                    status.get("report_present", False),
                    status["goal_reached"],
                    status["execution_integrity_pass"],
                ),
                flush=True,
            )
    verify_release_resources(
        release_path, expected_release_sha256=frozen_release_sha256
    )
    write_q1_journal(
        journal_path,
        frozen_release_sha256,
        frozen_review_sha256,
        matrix,
        terminal=True,
        in_progress=None,
    )
    matrix_path = q1_root / "matrix_receipt.json"
    write_atomic_json(
        matrix_path,
        {
            "schema_version": "1.0",
            "stage": "ARENA-PCT-Q1",
            "attempt_policy": "attempt_1_only_no_retry_no_replacement",
            "terminal": True,
            "episodes": matrix,
            "source_world_sha256_after": sha256(SOURCE_WORLD),
            "release_sha256": frozen_release_sha256,
            "release_review_path": str(review_path.relative_to(ROOT)),
            "release_review_sha256": frozen_review_sha256,
            "terminal_journal_path": str(journal_path.relative_to(ROOT)),
            "terminal_journal_sha256": sha256(journal_path),
        },
    )
    manifest_path, seal_path = write_raw_evidence_closure(
        output_root,
        frozen_release_sha256,
        frozen_review_sha256,
        sha256(journal_path),
        sha256(matrix_path),
        [
            {
                "seed": int(row["seed"]),
                "scene_id": str(row["scene_id"]),
                "method": str(row["method"]),
                "attempt": int(row["attempt"]),
            }
            for row in matrix
        ],
    )
    print(
        json.dumps(
            {
                "matrix_receipt_sha256": sha256(matrix_path),
                "raw_evidence_manifest_sha256": sha256(manifest_path),
                "raw_evidence_seal_sha256": sha256(seal_path),
            },
            indent=2,
            sort_keys=True,
        ),
        flush=True,
    )
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "stage",
        choices=("development", "prepare", "qualification"),
        help="bounded stage",
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--port", type=int, default=15400)
    parser.add_argument("--scene", choices=("hospital_static", "hospital_single_crossing"))
    parser.add_argument("--method", choices=METHODS)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--release-sha256")
    parser.add_argument("--release-review-sha256")
    args = parser.parse_args()
    if args.stage == "development":
        if args.scene is None or args.method is None or args.seed is None:
            parser.error("development requires --scene, --method and --seed")
        return run_development(args)
    if args.stage == "prepare":
        if any(value is not None for value in (args.scene, args.method, args.seed)):
            parser.error("prepare uses the frozen built-in schedule")
        if args.release_sha256 is not None:
            parser.error("prepare does not accept --release-sha256")
        if args.release_review_sha256 is not None:
            parser.error("prepare does not accept --release-review-sha256")
        return prepare_qualification(args)
    if any(value is not None for value in (args.scene, args.method, args.seed)):
        parser.error("qualification uses the frozen built-in schedule")
    if args.release_sha256 is None:
        parser.error("qualification requires --release-sha256")
    if args.release_review_sha256 is None:
        parser.error("qualification requires --release-review-sha256")
    return run_qualification(args)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
