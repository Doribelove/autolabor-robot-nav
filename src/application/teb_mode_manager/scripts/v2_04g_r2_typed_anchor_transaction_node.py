#!/usr/bin/env python3
"""V2-04G-R2 mechanism stage over the frozen R1 bounded-join node."""

import importlib.util
from pathlib import Path

import rospy
import yaml

from teb_mode_manager.action_pipeline import ActionPipelineError
from teb_mode_manager.idempotent_typed_teb_transaction import (
    IdempotentTypedTebTransactionBackend,
)


R1_NODE = Path(__file__).with_name("v2_04g_r1_typed_anchor_transaction_node.py")
_SPEC = importlib.util.spec_from_file_location("v2_04g_r1_frozen_node", R1_NODE)
_R1 = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_R1)


def load_r2_mechanism_config(path):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    required = {
        "schema_version", "architecture_generation", "stage", "profile_id",
        "status", "simulation_only", "runtime_ready", "training_allowed",
        "real_vehicle_use_forbidden", "static_topology", "corridor_centerline",
        "maneuver", "dynamic_release", "policy_boundary",
    }
    if not isinstance(data, dict) or set(data) != required:
        raise ValueError("V2-04G-R2 mechanism config keys drifted")
    if not (
        str(data["schema_version"]) == "2.0"
        and data["architecture_generation"] == "v2"
        and data["stage"] == "V2-04G-R2"
        and data["status"] == "calibration_candidate"
        and data["simulation_only"] is True
        and data["runtime_ready"] is False
        and data["training_allowed"] is False
        and data["real_vehicle_use_forbidden"] is True
    ):
        raise ValueError("V2-04G-R2 mechanism safety boundary drifted")
    if data["policy_boundary"] != {
        "runtime_scene_labels_allowed": False,
        "runtime_manifest_access": False,
        "published_velocity_commands": False,
        "learned_policy_loaded": False,
    }:
        raise ValueError("V2-04G-R2 mechanism policy boundary drifted")
    return data


# The R1 class resolves this loader from its own module globals. Replacing only
# the stage validator keeps its bounded join and typed transaction code exact.
_R1.load_r1_mechanism_config = load_r2_mechanism_config
# Keep the R1 context cache and join decision byte-identical. R2 changes only
# the execution side after the join: repeated writes of an already acknowledged
# typed profile are coalesced while an activated transaction trace is retained.
_R1.TypedTebTransactionBackend = IdempotentTypedTebTransactionBackend
SimulationTypedAnchorTransactionNode = _R1.SimulationTypedAnchorTransactionNode


def main():
    rospy.init_node("v2_04g_r2_typed_anchor_transaction")
    try:
        SimulationTypedAnchorTransactionNode()
    except (ActionPipelineError, RuntimeError, ValueError) as exc:
        rospy.logfatal("V2-04G-R2 simulation typed transaction denied: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
