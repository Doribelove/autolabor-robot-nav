"""Calibration-only deterministic mechanism layer for V2-04G.

The layer never publishes velocity commands and never reads scene labels.  It
keeps episode-local topology preference state, applies corridor-centerline
feedback, selects a bounded forward/reverse maneuver profile, and produces
mode-conditioned residuals for the existing feasible decoder.
"""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Dict, Mapping

import yaml


@dataclass(frozen=True)
class MechanismSnapshot:
    front_clearance_m: float
    rear_clearance_m: float
    left_clearance_m: float
    right_clearance_m: float
    corridor_center_offset_m: float
    signed_heading_error_rad: float
    rear_covered: bool


@dataclass(frozen=True)
class MechanismCommand:
    residuals: Dict[str, float]
    topology_preference: str
    topology_locked: bool
    corridor_centerline_active: bool
    maneuver_reverse: bool
    reason: str


def load_mechanism_config(path):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    required = {
        "schema_version", "architecture_generation", "stage", "profile_id",
        "status", "simulation_only", "runtime_ready", "training_allowed",
        "real_vehicle_use_forbidden", "static_topology", "corridor_centerline",
        "maneuver", "dynamic_release", "policy_boundary",
    }
    if not isinstance(data, dict) or set(data) != required:
        raise ValueError("V2-04G mechanism config keys drifted")
    if not (
        str(data["schema_version"]) == "2.0"
        and data["architecture_generation"] == "v2"
        and data["stage"] == "V2-04G"
        and data["status"] in (
            "calibration_candidate",
            "frozen_after_calibration_for_held_out_validation",
        )
        and data["simulation_only"] is True
        and data["runtime_ready"] is False
        and data["training_allowed"] is False
        and data["real_vehicle_use_forbidden"] is True
    ):
        raise ValueError("V2-04G mechanism safety boundary drifted")
    boundary = data["policy_boundary"]
    if boundary != {
        "runtime_scene_labels_allowed": False,
        "runtime_manifest_access": False,
        "published_velocity_commands": False,
        "learned_policy_loaded": False,
    }:
        raise ValueError("V2-04G mechanism policy boundary drifted")
    return data


class RuleMechanismController:
    def __init__(self, config: Mapping):
        self.config = config
        self.topology_preference = "NONE"
        self.topology_locked = False
        self.topology_switch_count = 0
        self.last_mode = "BALANCED"

    def reset(self):
        self.__init__(self.config)

    @staticmethod
    def _bounded_residuals(values):
        result = {}
        for name, value in values.items():
            number = float(value)
            if not math.isfinite(number) or number < -1.0 or number > 1.0:
                raise ValueError("mechanism residual {} is outside [-1, 1]".format(name))
            result[name] = number
        return result

    def update(self, geometry_mode, dynamic_overlay, snapshot):
        reasons = []
        residuals = {}
        corridor_active = False
        maneuver_reverse = False

        static = self.config["static_topology"]
        if geometry_mode == "STATIC_DENSE":
            if snapshot.front_clearance_m < float(static["unlock_front_clearance_m"]):
                self.topology_locked = False
                reasons.append("static_topology_unlocked_front_infeasible")
            if not self.topology_locked:
                preference = (
                    "LEFT" if snapshot.left_clearance_m >= snapshot.right_clearance_m else "RIGHT"
                )
                if self.topology_preference not in ("NONE", preference):
                    self.topology_switch_count += 1
                self.topology_preference = preference
                self.topology_locked = True
                reasons.append("static_topology_locked_{}".format(preference.lower()))
            residuals.update(static["residuals"])
        elif self.last_mode == "STATIC_DENSE":
            self.topology_locked = False
            self.topology_preference = "NONE"
            reasons.append("static_topology_released_on_mode_exit")

        corridor = self.config["corridor_centerline"]
        if geometry_mode == "CORRIDOR":
            corridor_active = True
            residuals.update(corridor["centered_residuals"])
            if abs(snapshot.corridor_center_offset_m) >= float(corridor["correction_offset_m"]):
                residuals.update(corridor["correction_residuals"])
                reasons.append("corridor_centerline_correction")
            else:
                reasons.append("corridor_centerline_tracking")

        maneuver = self.config["maneuver"]
        if geometry_mode == "MANEUVER":
            maneuver_reverse = bool(
                snapshot.rear_covered
                and snapshot.rear_clearance_m >= float(maneuver["reverse_rear_clearance_min_m"])
                and snapshot.front_clearance_m <= float(maneuver["reverse_front_clearance_max_m"])
                and abs(snapshot.signed_heading_error_rad)
                >= float(maneuver["reverse_heading_error_min_rad"])
            )
            residuals.update(
                maneuver["reverse_residuals"] if maneuver_reverse
                else maneuver["forward_residuals"]
            )
            reasons.append(
                "maneuver_reverse_segment" if maneuver_reverse
                else "maneuver_forward_segment"
            )

        if dynamic_overlay != "NONE":
            reasons.append("dynamic_overlay_active_{}".format(dynamic_overlay.lower()))
        self.last_mode = geometry_mode
        return MechanismCommand(
            residuals=self._bounded_residuals(residuals),
            topology_preference=self.topology_preference,
            topology_locked=self.topology_locked,
            corridor_centerline_active=corridor_active,
            maneuver_reverse=maneuver_reverse,
            reason=";".join(reasons) if reasons else "balanced_no_special_mechanism",
        )
