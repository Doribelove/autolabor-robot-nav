#!/usr/bin/env python3
"""Run fresh V3-11 fair-execution smoke or qualification matrices."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
CONTRACT_PATH = (
    WORKSPACE
    / "config/thesis_experiments/v3/"
    "v3_11_a0_actual_teb_fair_baseline_contract.yaml"
)
PLANNER_PATH = (
    WORKSPACE
    / "artifacts/v3/planner/v3_08_a0/"
    "v3_08_a0_effective_planner_contract.yaml"
)
MODEL_PATH = (
    WORKSPACE
    / "artifacts/v3/prediction/v3_07_b0_r1/model/"
    "v3_07_b0_r1_spatial_candidate_predictor.yaml"
)


def sha256_bytes(payload):
    return hashlib.sha256(payload).hexdigest()


def read_bytes(path):
    with path.open("rb") as stream:
        return stream.read()


def yaml_bytes(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")


def atomic_yaml(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(yaml_bytes(value))
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def append_jsonl(path, value):
    payload = (
        json.dumps(
            value, sort_keys=True, separators=(",", ":"), allow_nan=False
        )
        + "\n"
    ).encode("utf-8")
    with path.open("ab", buffering=0) as stream:
        stream.write(payload)
        os.fsync(stream.fileno())


def pose_text(x_m, y_m, z_m, yaw_rad=0.0):
    return "{:.9f} {:.9f} {:.9f} 0 0 {:.9f}".format(
        float(x_m), float(y_m), float(z_m), float(yaw_rad)
    )


def add_box(world, name, x_m, y_m, length_m, width_m, height_m):
    model = ET.SubElement(world, "model", {"name": name})
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = pose_text(
        x_m, y_m, 0.5 * height_m
    )
    link = ET.SubElement(model, "link", {"name": "body"})
    for tag in ("collision", "visual"):
        item = ET.SubElement(link, tag, {"name": tag})
        geometry = ET.SubElement(item, "geometry")
        box = ET.SubElement(geometry, "box")
        ET.SubElement(box, "size").text = "{:.6f} {:.6f} {:.6f}".format(
            length_m, width_m, height_m
        )
    return model


def indent(element, level=0):
    indentation = "\n" + level * "  "
    child_indentation = "\n" + (level + 1) * "  "
    if len(element):
        if not element.text or not element.text.strip():
            element.text = child_indentation
        for child in element:
            indent(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = indentation
    if level and (not element.tail or not element.tail.strip()):
        element.tail = indentation


def deterministic_signed(seed, salt):
    payload = "{}:{}".format(int(seed), salt).encode("ascii")
    integer = int(hashlib.sha256(payload).hexdigest()[:8], 16)
    return 2.0 * (integer / float(0xFFFFFFFF)) - 1.0


def actor_parameters(seed):
    return {
        "start_coordinate_m": -(
            2.85 + ((int(seed) * 7) % 9) * 0.05
        ),
        "base_speed_mps": 0.90 + ((int(seed) * 3) % 5) * 0.05,
    }


def render_world(scene, actor_policy, seed):
    actor = actor_parameters(seed)
    angle_rad = math.radians(float(scene["actor_angle_deg"]))
    dx_m = math.cos(angle_rad)
    dy_m = math.sin(angle_rad)
    start_x = actor["start_coordinate_m"] * dx_m
    start_y = actor["start_coordinate_m"] * dy_m
    end_coordinate_m = 7.0
    end_x = end_coordinate_m * dx_m
    end_y = end_coordinate_m * dy_m

    sdf = ET.Element("sdf", {"version": "1.6"})
    world = ET.SubElement(sdf, "world", {"name": "v3_11_fair_world"})
    ET.SubElement(world, "gravity").text = "0 0 -9.81"
    physics = ET.SubElement(
        world, "physics", {"name": "v3_11_physics", "type": "ode"}
    )
    ET.SubElement(physics, "max_step_size").text = "0.002"
    ET.SubElement(physics, "real_time_update_rate").text = "500"
    light = ET.SubElement(
        world, "light", {"name": "sun", "type": "directional"}
    )
    ET.SubElement(light, "pose").text = "0 0 10 0 0 0"
    ET.SubElement(light, "diffuse").text = "0.8 0.8 0.8 1"
    ET.SubElement(light, "specular").text = "0.2 0.2 0.2 1"
    ET.SubElement(light, "direction").text = "-0.5 0.1 -0.9"

    ground = ET.SubElement(world, "model", {"name": "ground_plane"})
    ET.SubElement(ground, "static").text = "true"
    ground_link = ET.SubElement(ground, "link", {"name": "link"})
    collision = ET.SubElement(
        ground_link, "collision", {"name": "collision"}
    )
    geometry = ET.SubElement(collision, "geometry")
    plane = ET.SubElement(geometry, "plane")
    ET.SubElement(plane, "normal").text = "0 0 1"
    ET.SubElement(plane, "size").text = "40 30"

    half_width = scene.get("corridor_half_width_m")
    if half_width is not None:
        for side, y_m in (
            ("upper", float(half_width) + 0.10),
            ("lower", -float(half_width) - 0.10),
        ):
            add_box(
                world,
                "v3_11_{}_wall".format(side),
                0.5,
                y_m,
                13.0,
                0.20,
                1.0,
            )

    actor_model = add_box(
        world,
        "v3_11_reactive_actor",
        start_x,
        start_y,
        0.72,
        0.72,
        1.0,
    )
    ET.SubElement(actor_model, "pose").text = pose_text(
        start_x, start_y, 0.5, angle_rad
    )
    plugin = ET.SubElement(
        actor_model,
        "plugin",
        {
            "name": "v3_11_reactive_actor_controller",
            "filename": "libv3_reactive_actor_plugin.so",
        },
    )
    fields = {
        "ego_model": "autolabor_m2",
        "policy": actor_policy,
        "seed": int(seed),
        "start": "{:.9f} {:.9f}".format(start_x, start_y),
        "end": "{:.9f} {:.9f}".format(end_x, end_y),
        "conflict": "0 0",
        "base_speed": "{:.9f}".format(actor["base_speed_mps"]),
        "initial_speed": "{:.9f}".format(actor["base_speed_mps"]),
        "time_offset": "0.0",
        "start_delay": "5.0",
        "activation_ego_x": "-3.0",
        "ego_observation_period": "0.20",
        "ego_observation_hold": "0.30",
        "response_distance": "4.0",
        "response_time_margin": "1.35",
        "z": "0.5",
    }
    for key, value in fields.items():
        ET.SubElement(plugin, key).text = str(value)
    indent(sdf)
    return (
        '<?xml version="1.0"?>\n'
        + ET.tostring(sdf, encoding="unicode")
        + "\n",
        actor,
    )


def block_rows(contract, phase):
    matrix = contract["evaluation"][phase]
    methods = list(contract["methods"]["identifiers"])
    rows = []
    sequence = 0
    for block_index, seed in enumerate(
        range(int(matrix["seed_start"]), int(matrix["seed_end"]) + 1)
    ):
        stratum_index = block_index % len(matrix["strata"])
        stratum = matrix["strata"][stratum_index]
        method_order = methods if block_index % 2 == 0 else list(reversed(methods))
        for method in method_order:
            sequence += 1
            rows.append(
                {
                    "sequence": sequence,
                    "block_index": block_index + 1,
                    "seed": seed,
                    "scene_id": stratum["scene_id"],
                    "actor_policy": stratum["actor_policy"],
                    "method": method,
                }
            )
    return rows


def stop_process(process):
    if process.poll() is not None:
        return process.returncode
    try:
        os.killpg(process.pid, signal.SIGINT)
        return process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        return process.wait(timeout=5.0)


def run_episode(row, scene, root, lookahead_m, port_base):
    root.mkdir(parents=True, exist_ok=False)
    world_path = root / "world.sdf"
    report_path = root / "episode_report.yaml"
    log_path = root / "roslaunch.log"
    runtime_home = root / "runtime_home"
    runtime_home.mkdir()
    (runtime_home / "roslog").mkdir()
    world, actor = render_world(
        scene, row["actor_policy"], row["seed"]
    )
    world_path.write_text(world, encoding="utf-8")

    environment = dict(os.environ)
    environment.update(
        {
            "HOME": str(runtime_home.resolve()),
            "ROS_HOME": str((runtime_home / ".ros").resolve()),
            "ROS_LOG_DIR": str((runtime_home / "roslog").resolve()),
            "ROS_MASTER_URI": "http://127.0.0.1:{}".format(port_base),
            "GAZEBO_MASTER_URI": "http://127.0.0.1:{}".format(port_base + 1),
            "GAZEBO_MODEL_DATABASE_URI": "",
        }
    )
    command = [
        "roslaunch",
        "m2_gazebo",
        "m2_v3_11_fair_episode.launch",
        "world:={}".format(world_path.resolve()),
        "method:={}".format(row["method"]),
        "scene_id:={}".format(row["scene_id"]),
        "seed:={}".format(row["seed"]),
        "actor_policy:={}".format(row["actor_policy"]),
        "actor_angle_deg:={}".format(scene["actor_angle_deg"]),
        "report_path:={}".format(report_path.resolve()),
        "planner_contract_path:={}".format(PLANNER_PATH.resolve()),
        "predictor_model_path:={}".format(MODEL_PATH.resolve()),
        "lookahead_m:={:.6f}".format(float(lookahead_m)),
        "gui:=false",
    ]
    started = time.monotonic()
    with log_path.open("wb") as log_stream:
        process = subprocess.Popen(
            command,
            cwd=str(WORKSPACE),
            env=environment,
            stdin=subprocess.DEVNULL,
            stdout=log_stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        timed_out = False
        try:
            return_code = process.wait(timeout=150.0)
        except subprocess.TimeoutExpired:
            timed_out = True
            return_code = stop_process(process)
        else:
            stop_process(process)
    elapsed_wall_s = time.monotonic() - started
    if not report_path.is_file():
        tail = read_bytes(log_path)[-8000:].decode("utf-8", errors="replace")
        raise RuntimeError(
            "episode report absent rc={} timeout={}\n{}".format(
                return_code, timed_out, tail
            )
        )
    with report_path.open("r", encoding="utf-8") as stream:
        report = yaml.safe_load(stream)
    return {
        "row": row,
        "actor_generated_parameters": actor,
        "episode_report": report,
        "episode_report_sha256": sha256_bytes(read_bytes(report_path)),
        "world_sha256": sha256_bytes(read_bytes(world_path)),
        "roslaunch_log_sha256": sha256_bytes(read_bytes(log_path)),
        "roslaunch_return_code": return_code,
        "wall_elapsed_s": elapsed_wall_s,
        "timed_out": timed_out,
    }


def summarize(receipts, phase, contract_sha, lookahead_m):
    complete = [item["episode_report"] for item in receipts]
    method_summary = {}
    for method in ("v3_spatial_safe", "actual_teb"):
        rows = [item for item in complete if item["method"] == method]
        method_summary[method] = {
            "episodes": len(rows),
            "episode_passes": sum(bool(item["episode_pass"]) for item in rows),
            "goal_successes": sum(
                bool(item["result"]["goal_reached"]) for item in rows
            ),
            "mean_restricted_elapsed_s": (
                sum(
                    float(item["result"]["restricted_elapsed_s"])
                    for item in rows
                )
                / len(rows)
                if rows
                else None
            ),
            "minimum_clearance_m": (
                min(
                    float(
                        item["result"][
                            "minimum_conservative_clearance_m"
                        ]
                    )
                    for item in rows
                )
                if rows
                else None
            ),
            "contact_state_count": sum(
                int(item["result"]["contact_state_count"]) for item in rows
            ),
        }
    blocks = {}
    for receipt in receipts:
        row = receipt["row"]
        blocks.setdefault(row["block_index"], {})[row["method"]] = receipt[
            "episode_report"
        ]
    paired = []
    gate_pair_differences = []
    for block_index in sorted(blocks):
        block = blocks[block_index]
        if set(block) != {"v3_spatial_safe", "actual_teb"}:
            continue
        v3_time = float(
            block["v3_spatial_safe"]["result"]["restricted_elapsed_s"]
        )
        teb_time = float(
            block["actual_teb"]["result"]["restricted_elapsed_s"]
        )
        paired.append(
            {
                "block_index": block_index,
                "scene_id": block["v3_spatial_safe"]["scene_id"],
                "actor_policy": block["v3_spatial_safe"][
                    "actor_policy_evaluator_only"
                ],
                "v3_restricted_elapsed_s": v3_time,
                "actual_teb_restricted_elapsed_s": teb_time,
                "v3_minus_teb_s": v3_time - teb_time,
                "reduction_percent": 100.0 * (teb_time - v3_time) / teb_time,
            }
        )
        v3_actor = block["v3_spatial_safe"]["warmup"]["gate_actor_track"]
        teb_actor = block["actual_teb"]["warmup"]["gate_actor_track"]
        gate_pair_differences.append(
            {
                "block_index": block_index,
                "actor_coordinate_absolute_difference_m": abs(
                    float(v3_actor["coordinate_m"])
                    - float(teb_actor["coordinate_m"])
                ),
                "actor_speed_absolute_difference_mps": abs(
                    float(v3_actor["coordinate_speed_mps"])
                    - float(teb_actor["coordinate_speed_mps"])
                ),
            }
        )
    if paired:
        mean_v3 = sum(
            item["v3_restricted_elapsed_s"] for item in paired
        ) / len(paired)
        mean_teb = sum(
            item["actual_teb_restricted_elapsed_s"] for item in paired
        ) / len(paired)
        reduction = 100.0 * (mean_teb - mean_v3) / mean_teb
    else:
        reduction = None
    return {
        "schema_version": "1.0",
        "stage": "V3-11-{}".format(phase.upper()),
        "phase": phase,
        "status": "execution_complete",
        "simulation_only": True,
        "contract_sha256": contract_sha,
        "lookahead_m": float(lookahead_m),
        "training_steps": 0,
        "episode_count": len(receipts),
        "method_summary": method_summary,
        "paired_blocks": paired,
        "gate_pair_differences": gate_pair_differences,
        "maximum_gate_actor_coordinate_difference_m": (
            max(
                item["actor_coordinate_absolute_difference_m"]
                for item in gate_pair_differences
            )
            if gate_pair_differences
            else None
        ),
        "maximum_gate_actor_speed_difference_mps": (
            max(
                item["actor_speed_absolute_difference_mps"]
                for item in gate_pair_differences
            )
            if gate_pair_differences
            else None
        ),
        "paired_mean_time_reduction_percent": reduction,
        "all_gate_physical_valid": all(
            item["gate_physical_valid"] for item in complete
        ),
        "all_v3_spatial_selected": all(
            item["v3_plan"]["selected_candidate_id"]
            == "spatial_left_v120"
            for item in complete
            if item["method"] == "v3_spatial_safe"
        ),
        "all_actor_modes_match_evaluator_policy": all(
            item["warmup"]["actor_mode_matches_evaluator_policy"]
            for item in complete
        ),
        "all_zero_contact": all(
            item["result"]["contact_state_count"] == 0 for item in complete
        ),
        "all_nonnegative_conservative_clearance": all(
            item["result"]["minimum_conservative_clearance_m"] >= 0.0
            for item in complete
        ),
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--phase", choices=("smoke", "qualification"), required=True
    )
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--lookahead-m", type=float, default=0.70)
    parser.add_argument("--execute", action="store_true", required=True)
    arguments = parser.parse_args()
    output_root = Path(arguments.output_root).resolve()
    if output_root.exists():
        raise RuntimeError("output root exists; resume/retry is forbidden")
    output_root.mkdir(parents=True)
    contract_bytes = read_bytes(CONTRACT_PATH)
    contract = yaml.safe_load(contract_bytes)
    contract_sha = sha256_bytes(contract_bytes)
    if contract["authority"]["integration_and_qualification_authorized"] is not True:
        raise RuntimeError("contract does not authorize qualification execution")
    if contract["authority"]["training_authorized"] is not False:
        raise RuntimeError("training boundary differs")
    for resource in contract["qualification_dependency_freeze"]:
        path = WORKSPACE / resource["path"]
        if not path.is_file():
            raise RuntimeError(
                "frozen dependency absent: {}".format(resource["path"])
            )
        observed = sha256_bytes(read_bytes(path))
        if observed != resource["sha256"]:
            raise RuntimeError(
                "frozen dependency changed: {}".format(resource["path"])
            )
    scenes = {
        item["scene_id"]: item for item in contract["scene_population"]
    }
    rows = block_rows(contract, arguments.phase)
    atomic_yaml(
        output_root / "exact_schedule.yaml",
        {
            "phase": arguments.phase,
            "row_count": len(rows),
            "rows": rows,
        },
    )
    journal = output_root / "attempt_journal.jsonl"
    journal.touch(exist_ok=False)
    append_jsonl(
        journal,
        {
            "event": "execution_opened",
            "phase": arguments.phase,
            "contract_sha256": contract_sha,
            "authorized_units": len(rows),
            "lookahead_m": arguments.lookahead_m,
        },
    )
    receipts = []
    phase_offset = 100 if arguments.phase == "smoke" else 1000
    for row in rows:
        episode_root = output_root / "episodes" / (
            "{:03d}_{}_{}_{}_{}".format(
                row["sequence"],
                row["seed"],
                row["scene_id"],
                row["actor_policy"],
                row["method"],
            )
        )
        append_jsonl(
            journal,
            {
                "event": "unit_consumed",
                "sequence": row["sequence"],
                "row": row,
                "attempt": 1,
            },
        )
        try:
            receipt = run_episode(
                row,
                scenes[row["scene_id"]],
                episode_root,
                arguments.lookahead_m,
                22000 + phase_offset + 2 * row["sequence"],
            )
        except Exception as error:
            append_jsonl(
                journal,
                {
                    "event": "terminal_failure",
                    "sequence": row["sequence"],
                    "row": row,
                    "error": "{}:{}".format(type(error).__name__, error),
                    "consumed_units": row["sequence"],
                    "remaining_units_forfeited": len(rows)
                    - row["sequence"],
                },
            )
            raise
        receipt_path = episode_root / "matrix_receipt.yaml"
        atomic_yaml(receipt_path, receipt)
        receipts.append(receipt)
        report = receipt["episode_report"]
        append_jsonl(
            journal,
            {
                "event": "unit_complete",
                "sequence": row["sequence"],
                "row": row,
                "matrix_receipt_sha256": sha256_bytes(
                    read_bytes(receipt_path)
                ),
                "episode_pass": report["episode_pass"],
                "goal_reached": report["result"]["goal_reached"],
                "restricted_elapsed_s": report["result"][
                    "restricted_elapsed_s"
                ],
                "minimum_conservative_clearance_m": report["result"][
                    "minimum_conservative_clearance_m"
                ],
            },
        )
        print(
            "[V3-11-{}] {}/{} {} {} pass={} goal={} t={:.3f}s c={:.3f}m".format(
                arguments.phase.upper(),
                row["sequence"],
                len(rows),
                row["scene_id"],
                row["method"],
                report["episode_pass"],
                report["result"]["goal_reached"],
                report["result"]["restricted_elapsed_s"],
                report["result"]["minimum_conservative_clearance_m"],
            ),
            flush=True,
        )
    summary = summarize(
        receipts, arguments.phase, contract_sha, arguments.lookahead_m
    )
    atomic_yaml(output_root / "matrix_summary.yaml", summary)
    append_jsonl(
        journal,
        {
            "event": "execution_complete",
            "authorized_units": len(rows),
            "consumed_units": len(rows),
            "completed_units": len(rows),
            "forfeited_units": 0,
            "summary_sha256": sha256_bytes(
                read_bytes(output_root / "matrix_summary.yaml")
            ),
        },
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
