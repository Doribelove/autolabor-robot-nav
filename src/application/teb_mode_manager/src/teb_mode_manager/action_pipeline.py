"""Typed V2-04 Anchor Bank, feasible decoder and execution-aware transaction loop.

This module has no ROS dependency.  Its default executor remains deterministic
shadow execution.  The separately gated V2-04B Gazebo-only typed executor lives
in :mod:`teb_mode_manager.typed_teb_transaction`; importing this module never
enables parameter writes.
"""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import yaml


GEOMETRY_MODES = ("BALANCED", "CRUISE", "STATIC_DENSE", "CORRIDOR", "MANEUVER")
DYNAMIC_OVERLAYS = ("NONE", "CROSSING", "HEAD_ON", "FOLLOW", "OVERTAKE_OR_YIELD")
PARAMETER_TYPES = ("double", "int", "bool")

PROJECTION_PHYSICAL_BOUND = 1 << 0
PROJECTION_ACKERMANN = 1 << 1
PROJECTION_INFLATION_GAP = 1 << 2
PROJECTION_DYNAMIC_INFLATION_GAP = 1 << 3
PROJECTION_POSITIVE_WEIGHT = 1 << 4
PROJECTION_SPEED_ENVELOPE = 1 << 5


class ActionPipelineError(ValueError):
    """Fail-closed configuration, decoding or transaction error."""


def _finite_double(value: Any, context: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ActionPipelineError("{} must be a double".format(context))
    number = float(value)
    if not math.isfinite(number):
        raise ActionPipelineError("{} must be finite".format(context))
    return number


def _exact_keys(value: Mapping[str, Any], expected: Sequence[str], context: str) -> None:
    if not isinstance(value, Mapping):
        raise ActionPipelineError("{} must be a mapping".format(context))
    missing = sorted(set(expected) - set(value))
    extra = sorted(set(value) - set(expected))
    if missing or extra:
        raise ActionPipelineError(
            "{} keys differ; missing={}, extra={}".format(context, missing, extra)
        )


@dataclass(frozen=True)
class ParameterDefinition:
    name: str
    parameter_type: str
    lifecycle: str
    lower: Optional[float]
    upper: Optional[float]
    max_rate_per_s: Optional[float]
    residual_fraction: float

    @property
    def continuous(self) -> bool:
        return self.parameter_type == "double"

    def validate_value(self, value: Any, context: str) -> Any:
        if self.parameter_type == "bool":
            if not isinstance(value, bool):
                raise ActionPipelineError("{} must be bool".format(context))
            return value
        if self.parameter_type == "int":
            if isinstance(value, bool) or not isinstance(value, int):
                raise ActionPipelineError("{} must be int".format(context))
            number = value
        else:
            number = _finite_double(value, context)
        if self.lower is None or self.upper is None:
            raise ActionPipelineError("{} numeric bounds are missing".format(context))
        if number < self.lower or number > self.upper:
            raise ActionPipelineError(
                "{}={} outside [{}, {}]".format(context, number, self.lower, self.upper)
            )
        return float(number) if self.parameter_type == "double" else int(number)


@dataclass(frozen=True)
class TypedProfile:
    anchor_id: str
    profile_id: str
    values: Dict[str, Any]


@dataclass(frozen=True)
class OverlayDefinition:
    overlay_id: str
    profile_suffix: str
    scale: Dict[str, float]
    offset: Dict[str, float]


@dataclass(frozen=True)
class DecodeResult:
    anchor_id: str
    profile_id: str
    commanded: TypedProfile
    feasible: TypedProfile
    projection_reason_mask: int

    @property
    def projected(self) -> bool:
        return self.projection_reason_mask != 0


@dataclass(frozen=True)
class ExecutionReceipt:
    requested: TypedProfile
    acknowledgement: TypedProfile
    readback: TypedProfile
    executed: TypedProfile
    t_request_s: float
    t_ack_s: float
    t_readback_s: float
    t_active_s: float


@dataclass(frozen=True)
class ParameterTransactionTrace:
    world_model_seq: int
    mode_seq: int
    config_seq: int
    geometry_mode: str
    dynamic_overlay: str
    transition_state: str
    anchor_id: str
    profile_id: str
    execution_backend: str
    parameter_names: Tuple[str, ...]
    parameter_types: Tuple[str, ...]
    commanded: Tuple[float, ...]
    feasible: Tuple[float, ...]
    safe: Tuple[float, ...]
    executed: Tuple[float, ...]
    projection_reason_mask: int
    safety_reason_mask: int
    t_request_s: float
    t_ack_s: float
    t_readback_s: float
    t_active_s: float
    activated: bool
    slow_profile_committed: bool
    training_used: bool
    valid: bool
    fault_reason: str

    def stage_mapping(self, stage: str) -> Dict[str, float]:
        if stage not in ("commanded", "feasible", "safe", "executed"):
            raise ActionPipelineError("unknown action stage {}".format(stage))
        return dict(zip(self.parameter_names, getattr(self, stage)))

    def typed_stage_mapping(self, stage: str) -> Dict[str, Any]:
        """Reconstruct exact dynamic-reconfigure types from a numeric ROS trace."""

        numeric = self.stage_mapping(stage)
        typed = {}
        for name, parameter_type in zip(self.parameter_names, self.parameter_types):
            value = numeric[name]
            if not math.isfinite(value):
                raise ActionPipelineError("{}:{} is non-finite".format(stage, name))
            if parameter_type == "double":
                typed[name] = float(value)
            elif parameter_type == "int":
                if not float(value).is_integer():
                    raise ActionPipelineError("{}:{} is not integral".format(stage, name))
                typed[name] = int(value)
            elif parameter_type == "bool":
                if value not in (0.0, 1.0):
                    raise ActionPipelineError("{}:{} is not boolean".format(stage, name))
                typed[name] = bool(value)
            else:
                raise ActionPipelineError("{}:{} type is unknown".format(stage, name))
        return typed


class AnchorBank:
    """Strict typed profile store with factorized dynamic overlay templates."""

    ROOT_KEYS = (
        "schema_version", "architecture_generation", "bank_id", "status",
        "simulation_only", "runtime_ready", "training_allowed",
        "parameter_write_enabled", "real_vehicle_use_forbidden", "source_provenance",
        "vehicle", "parameters", "mode_anchor_map", "anchors", "overlays",
        "transaction", "policy_boundary",
    )

    def __init__(self, data: Mapping[str, Any]):
        _exact_keys(data, self.ROOT_KEYS, "anchor_bank")
        if not (
            str(data["schema_version"]) == "2.0"
            and data["architecture_generation"] == "v2"
            and data["status"] in (
                "uncalibrated_simulation_candidate", "calibrated_simulation_frozen",
            )
            and data["simulation_only"] is True
            and data["runtime_ready"] is False
            and data["training_allowed"] is False
            and data["parameter_write_enabled"] is False
            and data["real_vehicle_use_forbidden"] is True
        ):
            raise ActionPipelineError("anchor bank safety boundary drifted")
        provenance = data["source_provenance"]
        _exact_keys(
            provenance,
            ("baseline", "mode_deltas", "formal_test_scenes_used"),
            "source_provenance",
        )
        if provenance["formal_test_scenes_used"] is not False:
            raise ActionPipelineError("formal test scenes cannot select anchors")
        vehicle = data["vehicle"]
        _exact_keys(
            vehicle,
            (
                "minimum_turning_radius_m", "minimum_inflation_gap_m",
                "minimum_dynamic_inflation_gap_m",
            ),
            "vehicle",
        )
        self.minimum_turning_radius_m = _finite_double(
            vehicle["minimum_turning_radius_m"], "minimum_turning_radius_m"
        )
        self.minimum_inflation_gap_m = _finite_double(
            vehicle["minimum_inflation_gap_m"], "minimum_inflation_gap_m"
        )
        self.minimum_dynamic_inflation_gap_m = _finite_double(
            vehicle["minimum_dynamic_inflation_gap_m"],
            "minimum_dynamic_inflation_gap_m",
        )
        if min(
            self.minimum_turning_radius_m,
            self.minimum_inflation_gap_m,
            self.minimum_dynamic_inflation_gap_m,
        ) <= 0.0:
            raise ActionPipelineError("vehicle constraints must be positive")
        self.definitions = self._load_definitions(data["parameters"])
        self.parameter_names = tuple(self.definitions)
        self.parameter_types = tuple(
            self.definitions[name].parameter_type for name in self.parameter_names
        )
        self.anchors = self._load_anchors(data["anchors"])
        mode_map = data["mode_anchor_map"]
        _exact_keys(mode_map, GEOMETRY_MODES, "mode_anchor_map")
        if any(anchor_id not in self.anchors for anchor_id in mode_map.values()):
            raise ActionPipelineError("mode_anchor_map references an unknown anchor")
        self.mode_anchor_map = dict(mode_map)
        self.overlays = self._load_overlays(data["overlays"])
        self.transaction = dict(data["transaction"])
        self.policy_boundary = dict(data["policy_boundary"])
        self._validate_transaction_boundary()
        self._validate_profiles_intrinsically_feasible()

    @classmethod
    def from_file(cls, path: Any) -> "AnchorBank":
        source = Path(path)
        try:
            data = yaml.safe_load(source.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError) as exc:
            raise ActionPipelineError("cannot load anchor bank {}: {}".format(source, exc))
        if not isinstance(data, dict):
            raise ActionPipelineError("anchor bank root must be a mapping")
        return cls(data)

    def _load_definitions(self, rows: Any) -> Dict[str, ParameterDefinition]:
        if not isinstance(rows, list) or not rows:
            raise ActionPipelineError("parameters must be a non-empty list")
        definitions = {}
        for index, row in enumerate(rows):
            _exact_keys(
                row,
                (
                    "name", "type", "lifecycle", "bounds", "max_rate_per_s",
                    "residual_fraction",
                ),
                "parameters[{}]".format(index),
            )
            name = row["name"]
            parameter_type = row["type"]
            if not isinstance(name, str) or not name or name in definitions:
                raise ActionPipelineError("parameter names must be unique non-empty strings")
            if parameter_type not in PARAMETER_TYPES:
                raise ActionPipelineError("{} has unsupported type".format(name))
            lifecycle = row["lifecycle"]
            if lifecycle not in ("fast_continuous", "slow_mode_profile"):
                raise ActionPipelineError("{} lifecycle is invalid".format(name))
            bounds = row["bounds"]
            if parameter_type == "bool":
                if bounds is not None or row["max_rate_per_s"] is not None:
                    raise ActionPipelineError("bool {} cannot define numeric limits".format(name))
                lower = upper = rate = None
            else:
                if not isinstance(bounds, list) or len(bounds) != 2:
                    raise ActionPipelineError("{} bounds must be a pair".format(name))
                lower = _finite_double(bounds[0], name + ".lower")
                upper = _finite_double(bounds[1], name + ".upper")
                if lower > upper:
                    raise ActionPipelineError("{} bounds are reversed".format(name))
                if parameter_type == "int":
                    if not float(lower).is_integer() or not float(upper).is_integer():
                        raise ActionPipelineError("{} int bounds must be integral".format(name))
                    if row["max_rate_per_s"] is not None:
                        raise ActionPipelineError("int {} must commit atomically".format(name))
                    rate = None
                else:
                    rate = _finite_double(row["max_rate_per_s"], name + ".max_rate_per_s")
                    if rate <= 0.0:
                        raise ActionPipelineError("{} rate must be positive".format(name))
            residual = _finite_double(row["residual_fraction"], name + ".residual_fraction")
            if residual < 0.0 or residual > 1.0:
                raise ActionPipelineError("{} residual fraction is invalid".format(name))
            if lifecycle != "fast_continuous" and residual != 0.0:
                raise ActionPipelineError("slow profile parameters cannot receive residuals")
            definitions[name] = ParameterDefinition(
                name, parameter_type, lifecycle, lower, upper, rate, residual
            )
        return definitions

    def _load_anchors(self, rows: Any) -> Dict[str, TypedProfile]:
        if not isinstance(rows, Mapping):
            raise ActionPipelineError("anchors must be a mapping")
        anchors = {}
        for anchor_id, row in rows.items():
            _exact_keys(
                row,
                ("geometry_mode", "maneuver_direction", "profile_id", "values"),
                "anchors.{}".format(anchor_id),
            )
            if row["geometry_mode"] not in GEOMETRY_MODES:
                raise ActionPipelineError("{} geometry mode is invalid".format(anchor_id))
            if row["maneuver_direction"] not in ("FORWARD", "REVERSE"):
                raise ActionPipelineError("{} direction is invalid".format(anchor_id))
            values = self.validate_values(row["values"], "anchors.{}.values".format(anchor_id))
            anchors[anchor_id] = TypedProfile(anchor_id, str(row["profile_id"]), values)
        return anchors

    def _load_overlays(self, rows: Any) -> Dict[str, OverlayDefinition]:
        _exact_keys(rows, DYNAMIC_OVERLAYS, "overlays")
        overlays = {}
        for overlay_id in DYNAMIC_OVERLAYS:
            row = rows[overlay_id]
            _exact_keys(row, ("profile_suffix", "scale", "offset"), "overlays." + overlay_id)
            scale = self._overlay_terms(row["scale"], "scale", overlay_id)
            offset = self._overlay_terms(row["offset"], "offset", overlay_id)
            overlays[overlay_id] = OverlayDefinition(
                overlay_id, str(row["profile_suffix"]), scale, offset
            )
        return overlays

    def _overlay_terms(self, values: Any, operation: str, overlay_id: str) -> Dict[str, float]:
        if not isinstance(values, Mapping):
            raise ActionPipelineError("overlay terms must be a mapping")
        result = {}
        for name, value in values.items():
            if name not in self.definitions:
                raise ActionPipelineError("overlay references unknown parameter {}".format(name))
            definition = self.definitions[name]
            if definition.lifecycle != "fast_continuous" or not definition.continuous:
                raise ActionPipelineError("overlay cannot modify typed slow parameter {}".format(name))
            number = _finite_double(value, "overlays.{}.{}.{}".format(overlay_id, operation, name))
            if operation == "scale" and number <= 0.0:
                raise ActionPipelineError("overlay scale must be positive")
            result[name] = number
        return result

    def _validate_transaction_boundary(self) -> None:
        tx = self.transaction
        _exact_keys(
            tx,
            (
                "decision_frequency_hz", "equality_tolerance",
                "continuous_convergence_tolerance", "transition_origin", "discrete_commit",
                "backend", "ack_delay_s", "readback_delay_s", "activation_delay_s",
                "dynamic_reconfigure_enabled", "restore_previous_on_failure",
            ),
            "transaction",
        )
        if tx["transition_origin"] != "previous_executed":
            raise ActionPipelineError("transition origin must be previous_executed")
        if tx["discrete_commit"] != "after_continuous_convergence":
            raise ActionPipelineError("typed discrete commit contract drifted")
        if tx["backend"] != "deterministic_shadow":
            raise ActionPipelineError("V2-04 only permits deterministic shadow execution")
        if tx["dynamic_reconfigure_enabled"] is not False:
            raise ActionPipelineError("V2-04 dynamic_reconfigure must remain disabled")
        if tx["restore_previous_on_failure"] is not True:
            raise ActionPipelineError("transaction rollback must remain enabled")
        boundary = self.policy_boundary
        _exact_keys(
            boundary,
            (
                "policy_source", "learned_policy_loaded", "runtime_scene_labels_allowed",
                "runtime_manifest_access", "forbidden_runtime_topics",
                "published_velocity_commands",
            ),
            "policy_boundary",
        )
        if boundary["policy_source"] != "rule_supervisor_zero_residual":
            raise ActionPipelineError("V2-04 policy must be rule zero-residual")
        for name in (
            "learned_policy_loaded", "runtime_scene_labels_allowed", "runtime_manifest_access",
            "published_velocity_commands",
        ):
            if boundary[name] is not False:
                raise ActionPipelineError("policy boundary {} drifted".format(name))
        forbidden = set(boundary["forbidden_runtime_topics"])
        if not {"/gazebo/model_states", "/pedsim_simulator/simulated_agents"}.issubset(forbidden):
            raise ActionPipelineError("runtime truth topics must remain forbidden")

    def validate_values(self, values: Any, context: str) -> Dict[str, Any]:
        _exact_keys(values, self.parameter_names, context)
        return {
            name: self.definitions[name].validate_value(values[name], context + "." + name)
            for name in self.parameter_names
        }

    def anchor_for_mode(self, geometry_mode: str, maneuver_reverse: bool = False) -> TypedProfile:
        if geometry_mode not in GEOMETRY_MODES:
            raise ActionPipelineError("unknown geometry mode {}".format(geometry_mode))
        anchor_id = self.mode_anchor_map[geometry_mode]
        if geometry_mode == "MANEUVER" and maneuver_reverse:
            anchor_id = "anchor_maneuver_reverse"
        return self.anchors[anchor_id]

    def _validate_profiles_intrinsically_feasible(self) -> None:
        decoder = FeasibleActionDecoder(self)
        for mode in GEOMETRY_MODES:
            anchor_ids = [self.mode_anchor_map[mode]]
            if mode == "MANEUVER":
                anchor_ids.append("anchor_maneuver_reverse")
            for anchor_id in anchor_ids:
                for overlay in DYNAMIC_OVERLAYS:
                    result = decoder.decode_anchor(anchor_id, overlay)
                    if result.projected:
                        raise ActionPipelineError(
                            "{}+{} requires normal-path projection mask {}".format(
                                anchor_id, overlay, result.projection_reason_mask
                            )
                        )


class FeasibleActionDecoder:
    """Decode bounded semantic residuals with Ackermann/distance constraints inside."""

    WEIGHT_NAMES = (
        "weight_obstacle", "weight_viapoint", "weight_optimaltime",
        "weight_dynamic_obstacle", "weight_dynamic_obstacle_inflation",
        "weight_velocity_obstacle_ratio",
    )

    def __init__(self, bank: AnchorBank):
        self.bank = bank

    def decode(
        self,
        geometry_mode: str,
        dynamic_overlay: str,
        residuals: Optional[Mapping[str, Any]] = None,
        speed_envelope_mps: Optional[float] = None,
        maneuver_reverse: bool = False,
    ) -> DecodeResult:
        anchor = self.bank.anchor_for_mode(geometry_mode, maneuver_reverse=maneuver_reverse)
        return self.decode_anchor(
            anchor.anchor_id, dynamic_overlay, residuals=residuals,
            speed_envelope_mps=speed_envelope_mps,
        )

    def decode_anchor(
        self,
        anchor_id: str,
        dynamic_overlay: str,
        residuals: Optional[Mapping[str, Any]] = None,
        speed_envelope_mps: Optional[float] = None,
    ) -> DecodeResult:
        if anchor_id not in self.bank.anchors:
            raise ActionPipelineError("unknown anchor {}".format(anchor_id))
        if dynamic_overlay not in self.bank.overlays:
            raise ActionPipelineError("unknown overlay {}".format(dynamic_overlay))
        anchor = self.bank.anchors[anchor_id]
        overlay = self.bank.overlays[dynamic_overlay]
        values = dict(anchor.values)
        for name, factor in overlay.scale.items():
            values[name] *= factor
        for name, offset in overlay.offset.items():
            values[name] += offset
        residual_values = {} if residuals is None else dict(residuals)
        invalid_names = sorted(
            name for name in residual_values
            if name not in self.bank.definitions
            or self.bank.definitions[name].lifecycle != "fast_continuous"
        )
        if invalid_names:
            raise ActionPipelineError("residual contains unsupported parameters {}".format(invalid_names))
        for name, residual in residual_values.items():
            z = _finite_double(residual, "residual." + name)
            if z < -1.0 or z > 1.0:
                raise ActionPipelineError("residual.{} outside [-1, 1]".format(name))
            definition = self.bank.definitions[name]
            fraction = definition.residual_fraction
            if name in self.WEIGHT_NAMES:
                values[name] *= math.exp(math.log1p(fraction) * z)
            elif z >= 0.0:
                values[name] += z * fraction * (definition.upper - values[name])
            else:
                values[name] += z * fraction * (values[name] - definition.lower)
        commanded_values = dict(values)
        profile_id = "{}+{}".format(anchor.profile_id, overlay.profile_suffix)
        commanded = TypedProfile(anchor_id, profile_id, commanded_values)
        feasible_values, reason_mask = self._intrinsic_feasible(
            values, speed_envelope_mps=speed_envelope_mps
        )
        feasible = TypedProfile(anchor_id, profile_id, feasible_values)
        return DecodeResult(anchor_id, profile_id, commanded, feasible, reason_mask)

    def _intrinsic_feasible(
        self, values: Mapping[str, Any], speed_envelope_mps: Optional[float]
    ) -> Tuple[Dict[str, Any], int]:
        feasible = dict(values)
        mask = 0
        for name, definition in self.bank.definitions.items():
            if definition.parameter_type == "bool":
                if not isinstance(feasible[name], bool):
                    raise ActionPipelineError("{} lost bool type".format(name))
                continue
            if definition.parameter_type == "int":
                if isinstance(feasible[name], bool) or not isinstance(feasible[name], int):
                    raise ActionPipelineError("{} lost int type".format(name))
                number = feasible[name]
            else:
                number = _finite_double(feasible[name], "decoded." + name)
            bounded = min(definition.upper, max(definition.lower, number))
            if bounded != number:
                mask |= PROJECTION_PHYSICAL_BOUND
            feasible[name] = float(bounded) if definition.parameter_type == "double" else int(bounded)
        if speed_envelope_mps is not None:
            envelope = _finite_double(speed_envelope_mps, "speed_envelope_mps")
            if envelope <= 0.0:
                raise ActionPipelineError("speed envelope must be positive")
            if feasible["max_vel_x"] > envelope:
                feasible["max_vel_x"] = max(
                    self.bank.definitions["max_vel_x"].lower, envelope
                )
                mask |= PROJECTION_SPEED_ENVELOPE
        yaw_limit = feasible["max_vel_x"] / self.bank.minimum_turning_radius_m
        if feasible["max_vel_theta"] > yaw_limit:
            # This is the decoder's native rho -> yaw mapping, not a terminal
            # projection.  The audit mask remains clear unless a later audit
            # must repair the already-decoded action.
            feasible["max_vel_theta"] = yaw_limit
        inflation_min = feasible["min_obstacle_dist"] + self.bank.minimum_inflation_gap_m
        if feasible["inflation_dist"] < inflation_min:
            # Inflation is represented as clearance plus a positive gap.
            feasible["inflation_dist"] = inflation_min
        dynamic_min = (
            feasible["min_obstacle_dist"] + self.bank.minimum_dynamic_inflation_gap_m
        )
        if feasible["dynamic_obstacle_inflation_dist"] < dynamic_min:
            feasible["dynamic_obstacle_inflation_dist"] = dynamic_min
        for name in self.WEIGHT_NAMES:
            if feasible[name] <= 0.0:
                feasible[name] = max(self.bank.definitions[name].lower, 1.0e-9)
                mask |= PROJECTION_POSITIVE_WEIGHT
        feasible = self.bank.validate_values(feasible, "feasible")
        return feasible, mask


class DeterministicShadowBackend:
    """Atomic simulation-only executor with injectable transaction faults."""

    backend_id = "deterministic_shadow"

    def __init__(self, bank: AnchorBank, initial: TypedProfile):
        self.bank = bank
        self.current = TypedProfile(
            initial.anchor_id, initial.profile_id,
            bank.validate_values(initial.values, "shadow_initial"),
        )
        self._next_fault = ""

    def inject_next_fault(self, fault: str) -> None:
        if fault not in ("timeout", "ack_mismatch", "readback_mismatch"):
            raise ActionPipelineError("unsupported shadow fault {}".format(fault))
        self._next_fault = fault

    def apply(self, requested: TypedProfile, now_s: float) -> ExecutionReceipt:
        values = self.bank.validate_values(requested.values, "shadow_request")
        fault = self._next_fault
        self._next_fault = ""
        if fault == "timeout":
            raise ActionPipelineError("shadow_request_timeout")
        tx = self.bank.transaction
        t_request = _finite_double(now_s, "transaction.now_s")
        acknowledgement = TypedProfile(requested.anchor_id, requested.profile_id, dict(values))
        if fault == "ack_mismatch":
            raise ActionPipelineError("shadow_ack_mismatch")
        t_ack = t_request + float(tx["ack_delay_s"])
        readback = TypedProfile(requested.anchor_id, requested.profile_id, dict(values))
        if fault == "readback_mismatch":
            raise ActionPipelineError("shadow_readback_mismatch")
        t_readback = t_ack + float(tx["readback_delay_s"])
        executed = TypedProfile(requested.anchor_id, requested.profile_id, dict(values))
        t_active = t_readback + float(tx["activation_delay_s"])
        self.current = executed
        return ExecutionReceipt(
            requested, acknowledgement, readback, executed,
            t_request, t_ack, t_readback, t_active,
        )


class RuleAnchorTransactionLoop:
    """No-training rule/profile loop driven only by ContextState semantics."""

    def __init__(
        self,
        bank: AnchorBank,
        backend: Optional[Any] = None,
    ):
        self.bank = bank
        self.decoder = FeasibleActionDecoder(bank)
        initial = bank.anchors["anchor_balanced"]
        self.backend = backend or DeterministicShadowBackend(bank, initial)
        self.executed = self.backend.current
        self.last_update_s: Optional[float] = None
        self.config_seq = 0
        self.frequency_hz = _finite_double(
            bank.transaction["decision_frequency_hz"], "decision_frequency_hz"
        )
        self.convergence_tolerance = _finite_double(
            bank.transaction["continuous_convergence_tolerance"],
            "continuous_convergence_tolerance",
        )

    def update(
        self,
        now_s: float,
        world_model_seq: int,
        mode_seq: int,
        geometry_mode: str,
        dynamic_overlay: str,
        transition_state: str,
        context_valid: bool,
        maneuver_reverse: bool = False,
    ) -> ParameterTransactionTrace:
        stamp = _finite_double(now_s, "now_s")
        if self.last_update_s is not None and stamp < self.last_update_s:
            raise ActionPipelineError("transaction time moved backwards")
        dt = 1.0 / self.frequency_hz if self.last_update_s is None else stamp - self.last_update_s
        if dt <= 0.0:
            raise ActionPipelineError("transaction dt must be positive")
        self.last_update_s = stamp
        self.config_seq += 1
        if not context_valid or transition_state == "FAULTED":
            current = self.executed
            vector = self.numeric_vector(current)
            return self._trace(
                world_model_seq, mode_seq, geometry_mode, dynamic_overlay,
                transition_state, current.anchor_id, current.profile_id,
                vector, vector, vector, vector, 0, 0,
                stamp, stamp, stamp, stamp, False, False, False,
                "invalid_or_faulted_context_hold_previous_executed",
            )
        decoded = self.decoder.decode(
            geometry_mode, dynamic_overlay, residuals={}, maneuver_reverse=maneuver_reverse
        )
        safe = decoded.feasible
        candidate_values, slow_committed = self._rate_limited_candidate(safe.values, dt)
        requested = TypedProfile(decoded.anchor_id, decoded.profile_id, candidate_values)
        previous = self.executed
        try:
            receipt = self.backend.apply(requested, stamp)
            self.executed = receipt.executed
        except ActionPipelineError as exc:
            self.executed = previous
            return self._trace(
                world_model_seq, mode_seq, geometry_mode, dynamic_overlay,
                transition_state, decoded.anchor_id, decoded.profile_id,
                self.numeric_vector(decoded.commanded), self.numeric_vector(decoded.feasible),
                self.numeric_vector(safe), self.numeric_vector(previous),
                decoded.projection_reason_mask, 0,
                stamp, stamp, stamp, stamp, False, False, False, str(exc),
            )
        return self._trace(
            world_model_seq, mode_seq, geometry_mode, dynamic_overlay,
            transition_state, decoded.anchor_id, decoded.profile_id,
            self.numeric_vector(decoded.commanded), self.numeric_vector(decoded.feasible),
            self.numeric_vector(safe), self.numeric_vector(receipt.executed),
            decoded.projection_reason_mask, 0,
            receipt.t_request_s, receipt.t_ack_s, receipt.t_readback_s, receipt.t_active_s,
            True, slow_committed, True, "",
        )

    def _rate_limited_candidate(
        self, target: Mapping[str, Any], dt: float
    ) -> Tuple[Dict[str, Any], bool]:
        current = self.executed.values
        result = dict(current)
        continuous_converged = True
        for name, definition in self.bank.definitions.items():
            if not definition.continuous:
                continue
            maximum_delta = definition.max_rate_per_s * dt
            delta = float(target[name]) - float(current[name])
            step = min(maximum_delta, max(-maximum_delta, delta))
            result[name] = float(current[name]) + step
            if abs(float(target[name]) - result[name]) > self.convergence_tolerance:
                continuous_converged = False
        discrete_changed = False
        if continuous_converged:
            for name, definition in self.bank.definitions.items():
                if not definition.continuous:
                    discrete_changed = discrete_changed or target[name] != current[name]
                    result[name] = target[name]
        return (
            self.bank.validate_values(result, "transaction_candidate"),
            continuous_converged and discrete_changed,
        )

    def numeric_vector(self, profile: TypedProfile) -> Tuple[float, ...]:
        vector = []
        for name in self.bank.parameter_names:
            value = profile.values[name]
            if isinstance(value, bool):
                vector.append(1.0 if value else 0.0)
            else:
                vector.append(float(value))
        return tuple(vector)

    def _trace(
        self, world_model_seq: int, mode_seq: int, geometry_mode: str,
        dynamic_overlay: str, transition_state: str, anchor_id: str, profile_id: str,
        commanded: Tuple[float, ...], feasible: Tuple[float, ...],
        safe: Tuple[float, ...], executed: Tuple[float, ...],
        projection_mask: int, safety_mask: int, t_request: float, t_ack: float,
        t_readback: float, t_active: float, activated: bool, slow_committed: bool,
        valid: bool, fault_reason: str,
    ) -> ParameterTransactionTrace:
        return ParameterTransactionTrace(
            int(world_model_seq), int(mode_seq), self.config_seq,
            geometry_mode, dynamic_overlay, transition_state, anchor_id, profile_id,
            self.backend.backend_id, self.bank.parameter_names, self.bank.parameter_types,
            commanded, feasible, safe, executed, int(projection_mask), int(safety_mask),
            t_request, t_ack, t_readback, t_active, activated, slow_committed,
            False, valid, fault_reason,
        )
