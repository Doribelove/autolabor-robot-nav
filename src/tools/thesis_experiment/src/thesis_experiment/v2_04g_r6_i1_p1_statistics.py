"""Deterministic, offline-only confirmatory statistics for V2-04G-P1.

The evaluator is deliberately independent of ROS and of the execution runner.  It
accepts the frozen schedule plus one terminal record per scheduled identity and
fails closed on any missing, duplicate, replaced, or structurally incomplete
episode.  Failed episodes stay in the intention-to-treat matrix through the
restricted-completion-time rule.
"""

from __future__ import annotations

import hashlib
import itertools
import json
import math
import re
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple


CONTROL = "r6_semantics_legacy_control"
TREATMENT = "r6_semantics_circle_contact"
REFERENCE = "fixed_teb_reference"
METHODS = (CONTROL, TREATMENT, REFERENCE)
ROLES = (
    "single_true_conflict",
    "multi_true_conflict",
    "time_separated_semantic_clear",
)
CONFLICT_ROLES = frozenset(ROLES[:2])
CLEAR_ROLE = ROLES[2]
EXPECTED_EPISODES = 270
EXPECTED_BLOCKS = 90
ALPHA = 0.05
BOOTSTRAP_RESAMPLES = 10000
BOOTSTRAP_SEED = 20260721
RANDOMIZATION_RESAMPLES = 100000
RANDOMIZATION_SEED = 20260722
TERMINAL_STATUSES = frozenset(
    {"completed", "timeout", "collision", "planner_failure", "process_failure"}
)
HEX_SHA256 = re.compile(r"^[0-9a-f]{64}$")


class P1StatisticsError(ValueError):
    """Raised when the frozen schedule or terminal episode matrix is invalid."""


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise P1StatisticsError(message)


def _finite(value: Any, context: str, *, positive: bool = False) -> float:
    _require(type(value) in (int, float), "{} must be numeric".format(context))
    result = float(value)
    _require(math.isfinite(result), "{} must be finite".format(context))
    if positive:
        _require(result > 0.0, "{} must be positive".format(context))
    return result


def _identity(row: Mapping[str, Any]) -> Tuple[int, str, str, int, int]:
    return (
        row["sequence"],
        row["method_id"],
        row["scene_id"],
        row["seed"],
        row["attempt"],
    )


def _counter_integer(seed: int, namespace: str, counter: int) -> int:
    """Version-independent deterministic counter generator based on SHA-256."""

    payload = "p1-r1:{}:{}:{}".format(seed, namespace, counter).encode("utf-8")
    return int.from_bytes(hashlib.sha256(payload).digest(), "big")


def validate_schedule(schedule: Mapping[str, Any]) -> List[Dict[str, Any]]:
    """Return normalized episode rows after checking all balance invariants."""

    _require(type(schedule) is dict, "schedule must be a mapping")
    required = {
        "schema_version", "stage", "status", "simulation_only", "formal_result",
        "runtime_ready", "execution_ready", "execution_authorized",
        "evidence_budget_authorized", "training_budget_steps",
        "real_vehicle_use_forbidden", "schedule_id", "construction", "methods",
        "scene_roles", "episode_count", "block_count", "episodes",
        "canonical_episode_digest",
    }
    _require(set(schedule) == required, "schedule top-level schema differs")
    _require(schedule["stage"] == "V2-04G-P1-R1", "schedule stage differs")
    _require(schedule["status"] == "offline_schedule_frozen_not_authorized", "schedule status differs")
    _require(schedule["simulation_only"] is True, "schedule must be simulation-only")
    _require(schedule["real_vehicle_use_forbidden"] is True, "real vehicle must be forbidden")
    for key in ("formal_result", "runtime_ready", "execution_ready", "execution_authorized"):
        _require(schedule[key] is False, "{} must be false".format(key))
    _require(schedule["evidence_budget_authorized"] == 0, "authorized evidence budget must be zero")
    _require(schedule["training_budget_steps"] == 0, "training budget must be zero")
    _require(schedule["methods"] == list(METHODS), "method roster differs")
    _require(schedule["scene_roles"] == list(ROLES), "role roster differs")
    expected_construction = {
        "algorithm": "six_permutation_three_role_wave_balance_v1",
        "waves": 30,
        "role_permutation_cycle": [list(item) for item in itertools.permutations(ROLES)],
        "role_permutation_index": "zero_based_wave_mod_6",
        "method_permutation_cycle": [list(item) for item in itertools.permutations(METHODS)],
        "method_permutation_index": "(zero_based_wave_plus_2_times_canonical_role_index)_mod_6",
        "block_rows_contiguous": True,
        "retry_resume_replacement_allowed": False,
    }
    _require(schedule["construction"] == expected_construction, "schedule construction differs")
    _require(schedule["episode_count"] == EXPECTED_EPISODES, "episode count differs")
    _require(schedule["block_count"] == EXPECTED_BLOCKS, "block count differs")
    episodes = schedule["episodes"]
    _require(type(episodes) is list and len(episodes) == EXPECTED_EPISODES, "episode roster differs")
    _require(canonical_sha256(episodes) == schedule["canonical_episode_digest"], "episode digest differs")

    expected_keys = {
        "sequence", "wave", "role_order_position", "block_id",
        "within_block_position", "scene_role", "seed", "scene_id",
        "instance", "world", "method_id", "expected_ttc_status", "attempt",
    }
    identities = set()
    blocks: Dict[str, List[Mapping[str, Any]]] = {}
    method_role_position = {}
    role_order_position = {}
    for index, raw in enumerate(episodes, 1):
        _require(type(raw) is dict and set(raw) == expected_keys, "schedule row schema differs")
        _require(raw["sequence"] == index, "schedule sequence is not contiguous")
        _require(type(raw["wave"]) is int and 1 <= raw["wave"] <= 30, "wave differs")
        _require(type(raw["role_order_position"]) is int and 1 <= raw["role_order_position"] <= 3, "role order differs")
        _require(type(raw["within_block_position"]) is int and 1 <= raw["within_block_position"] <= 3, "within-block position differs")
        _require(raw["scene_role"] in ROLES, "unknown scene role")
        _require(raw["method_id"] in METHODS, "unknown method")
        _require(raw["attempt"] == 1, "only attempt=1 is allowed")
        _require(type(raw["seed"]) is int and 5201 <= raw["seed"] <= 5290, "seed outside evidence block")
        role_seed_ranges = {
            ROLES[0]: range(5201, 5231),
            ROLES[1]: range(5231, 5261),
            ROLES[2]: range(5261, 5291),
        }
        _require(raw["seed"] in role_seed_ranges[raw["scene_role"]], "seed/role mapping differs")
        expected_status = (
            "OBSERVED_CONFLICT" if raw["scene_role"] in CONFLICT_ROLES
            else "NO_CONFLICT_IN_HORIZON"
        )
        _require(raw["expected_ttc_status"] == expected_status, "expected TTC status differs")
        for resource in (raw["instance"], raw["world"]):
            _require(type(resource) is dict and set(resource) == {"path", "sha256"}, "resource binding differs")
            _require(type(resource["path"]) is str and type(resource["sha256"]) is str, "resource binding type differs")
            _require(HEX_SHA256.fullmatch(resource["sha256"]) is not None, "resource SHA differs")
            resource_path = resource["path"]
            _require(
                not resource_path.startswith("/")
                and ".." not in resource_path.split("/")
                and resource_path.startswith(
                    "artifacts/v2/integration/v2_04g_r6_i1/"
                    "p1_r1_offline_readiness/compiled_scenes/"
                ),
                "resource path differs",
            )
        ident = _identity(raw)
        _require(ident not in identities, "duplicate schedule identity")
        identities.add(ident)
        blocks.setdefault(raw["block_id"], []).append(raw)
        key = (raw["scene_role"], raw["method_id"], raw["within_block_position"])
        method_role_position[key] = method_role_position.get(key, 0) + 1
        rkey = (raw["scene_role"], raw["role_order_position"])
        role_order_position[rkey] = role_order_position.get(rkey, 0) + 1

    _require(len(blocks) == EXPECTED_BLOCKS, "paired block count differs")
    _require(
        {next(iter({row["seed"] for row in rows})) for rows in blocks.values()}
        == set(range(5201, 5291)),
        "evidence seed set differs",
    )
    for rows in blocks.values():
        _require(len(rows) == 3, "a paired block does not contain three methods")
        _require({row["method_id"] for row in rows} == set(METHODS), "paired block method roster differs")
        common = {
            (
                row["seed"], row["scene_id"], row["scene_role"], row["wave"],
                row["role_order_position"], row["expected_ttc_status"],
            )
            for row in rows
        }
        _require(len(common) == 1, "methods do not share one scene instance")
        _require(len({canonical_sha256(row["instance"]) for row in rows}) == 1, "instance binding differs within block")
        _require(len({canonical_sha256(row["world"]) for row in rows}) == 1, "world binding differs within block")
        _require({row["within_block_position"] for row in rows} == {1, 2, 3}, "within-block positions differ")
        sequences = sorted(row["sequence"] for row in rows)
        _require(sequences == list(range(sequences[0], sequences[0] + 3)), "block rows are not contiguous")
    for role in ROLES:
        role_rows = [row for row in episodes if row["scene_role"] == role]
        _require(len(role_rows) == 90, "role episode count differs")
        _require(len({row["seed"] for row in role_rows}) == 30, "role seed count differs")
        for method in METHODS:
            _require(sum(row["method_id"] == method for row in role_rows) == 30, "method/role balance differs")
            for position in (1, 2, 3):
                _require(method_role_position[(role, method, position)] == 10, "within-block position balance differs")
        for position in (1, 2, 3):
            _require(role_order_position[(role, position)] == 30, "role-order episode balance differs")
        permutations = {}
        for rows in blocks.values():
            if rows[0]["scene_role"] != role:
                continue
            order = tuple(
                row["method_id"]
                for row in sorted(rows, key=lambda item: item["within_block_position"])
            )
            permutations[order] = permutations.get(order, 0) + 1
        _require(len(permutations) == 6 and set(permutations.values()) == {5}, "method permutation balance differs")
    for wave in range(1, 31):
        wave_blocks = {
            row["block_id"]: row for row in episodes if row["wave"] == wave
        }
        _require(len(wave_blocks) == 3, "wave block count differs")
        _require({row["scene_role"] for row in wave_blocks.values()} == set(ROLES), "wave role roster differs")
        _require({row["role_order_position"] for row in wave_blocks.values()} == {1, 2, 3}, "wave role positions differ")
    return [dict(row) for row in episodes]


def clopper_pearson_upper(events: int, trials: int, alpha: float = ALPHA) -> float:
    """One-sided exact binomial upper confidence bound (stdlib-only)."""

    _require(type(events) is int and type(trials) is int, "counts must be integers")
    _require(0 <= events <= trials and trials > 0, "invalid binomial counts")
    _require(0.0 < alpha < 1.0, "invalid alpha")
    if events == trials:
        return 1.0

    def cdf(probability: float) -> float:
        return sum(
            math.comb(trials, index)
            * probability ** index
            * (1.0 - probability) ** (trials - index)
            for index in range(events + 1)
        )

    low, high = 0.0, 1.0
    for _ in range(100):
        middle = (low + high) / 2.0
        if cdf(middle) > alpha:
            low = middle
        else:
            high = middle
    return (low + high) / 2.0


def _quantile(values: Sequence[float], probability: float) -> float:
    ordered = sorted(values)
    # Frozen nearest-rank rule: ceil(p * B) - 1.
    index = int(math.ceil(probability * len(ordered))) - 1
    index = min(max(index, 0), len(ordered) - 1)
    return ordered[index]


def _bootstrap_means(
    values: Sequence[float], *, resamples: int, seed: int, strata: Sequence[str] = ()
) -> List[float]:
    _require(values, "bootstrap requires observations")
    if strata:
        _require(len(strata) == len(values), "bootstrap strata length differs")
        groups = []
        for label in sorted(set(strata)):
            groups.append([value for value, group in zip(values, strata) if group == label])
    else:
        groups = [list(values)]
    output = []
    for replicate in range(resamples):
        sample = []
        for group_index, group in enumerate(groups):
            for draw_index in range(len(group)):
                selected = _counter_integer(
                    seed,
                    "bootstrap:{}:{}".format(replicate, group_index),
                    draw_index,
                ) % len(group)
                sample.append(group[selected])
        output.append(sum(sample) / len(sample))
    return output


def _paired_randomization_p(
    differences: Sequence[float], *, resamples: int, seed: int
) -> float:
    _require(differences, "paired randomization test requires observations")
    observed = sum(differences) / len(differences)
    exceed = 0
    for replicate in range(resamples):
        candidate = sum(
            value
            if _counter_integer(seed, "signflip:{}".format(replicate), index) & 1
            else -value
            for index, value in enumerate(differences)
        )
        candidate /= len(differences)
        if candidate >= observed - 1e-15:
            exceed += 1
    return (exceed + 1.0) / (resamples + 1.0)


def _overlay_fraction(intervals: Any, duration: Any, context: str) -> float:
    duration_s = _finite(duration, context + ".observation_duration_s", positive=True)
    _require(type(intervals) is list and intervals, context + ".overlay_intervals must be non-empty")
    cursor = 0.0
    non_none = 0.0
    for index, interval in enumerate(intervals):
        _require(type(interval) is dict and set(interval) == {"start_s", "end_s", "state"}, "overlay interval schema differs")
        start = _finite(interval["start_s"], "overlay start")
        end = _finite(interval["end_s"], "overlay end")
        _require(abs(start - cursor) <= 1e-9 and end > start, "overlay intervals must exactly tile time")
        _require(
            interval["state"] in {
                "NONE", "LEGACY_DYNAMIC", "CIRCLE_CONTACT_DYNAMIC",
                "CAUTION", "EMERGENCY",
            },
            "overlay state is outside the frozen instrumentation vocabulary",
        )
        if interval["state"] != "NONE":
            non_none += end - start
        cursor = end
    _require(abs(cursor - duration_s) <= 1e-9, "overlay trace does not cover observation duration")
    return non_none / duration_s


def _normalize_episode(raw: Mapping[str, Any], scheduled: Mapping[str, Any]) -> Dict[str, Any]:
    required = {
        "sequence", "method_id", "scene_id", "seed", "attempt", "block_id",
        "scene_role", "consumed", "terminal_status", "success", "collision",
        "navigation_time_s", "timeout_s", "minimum_footprint_clearance_m",
        "observed_ttc_status", "algorithmic_emergency", "interface_failure",
        "observation_duration_s", "overlay_intervals", "raw_resources_complete",
        "integrity_failures",
    }
    _require(type(raw) is dict and set(raw) == required, "terminal episode schema differs")
    for key in ("sequence", "method_id", "scene_id", "seed", "attempt", "block_id", "scene_role"):
        _require(raw[key] == scheduled[key], "episode identity differs: {}".format(key))
    _require(raw["consumed"] is True, "terminal assessment requires every identity consumed")
    _require(raw["terminal_status"] in TERMINAL_STATUSES, "unknown terminal status")
    _require(type(raw["success"]) is bool and type(raw["collision"]) is bool, "success/collision must be bool")
    _require(type(raw["algorithmic_emergency"]) is bool, "algorithmic emergency flag must be bool")
    _require(type(raw["interface_failure"]) is bool, "interface failure flag must be bool")
    _require(raw["interface_failure"] is False, "interface failure invalidates integrity")
    _require(raw["terminal_status"] != "process_failure", "process failure invalidates integrity")
    _require(raw["raw_resources_complete"] is True, "raw resources incomplete")
    _require(raw["integrity_failures"] == [], "episode integrity failure present")
    timeout = _finite(raw["timeout_s"], "timeout_s", positive=True)
    _require(abs(timeout - 80.0) <= 1e-9, "scene timeout differs from frozen 80 s")
    clearance = _finite(raw["minimum_footprint_clearance_m"], "minimum clearance")
    _require(raw["observed_ttc_status"] in {"OBSERVED_CONFLICT", "NO_CONFLICT_IN_HORIZON"}, "unknown TTC status")
    if raw["success"]:
        _require(raw["terminal_status"] == "completed", "successful episode must be completed")
        _require(raw["collision"] is False, "successful episode cannot collide")
        _require(raw["algorithmic_emergency"] is False, "successful episode cannot have algorithmic emergency")
        navigation = _finite(raw["navigation_time_s"], "navigation_time_s", positive=True)
        _require(navigation <= timeout, "successful navigation exceeds timeout")
        restricted = navigation
    else:
        if raw["navigation_time_s"] is not None:
            _finite(raw["navigation_time_s"], "failed navigation_time_s", positive=True)
        restricted = timeout
    overlay = _overlay_fraction(raw["overlay_intervals"], raw["observation_duration_s"], "episode")
    result = dict(raw)
    result["restricted_completion_time_s"] = restricted
    result["non_none_overlay_time_fraction"] = overlay
    result["minimum_footprint_clearance_m"] = clearance
    result["ttc_status_match"] = raw["observed_ttc_status"] == scheduled["expected_ttc_status"]
    return result


def _role_counts(rows: Sequence[Mapping[str, Any]], predicate) -> Dict[str, int]:
    return {role: sum(1 for row in rows if row["scene_role"] == role and predicate(row)) for role in ROLES}


def evaluate(schedule: Mapping[str, Any], episodes: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    """Evaluate the complete future P1 terminal matrix with four fail-closed gates."""

    scheduled = validate_schedule(schedule)
    _require(type(episodes) is list and len(episodes) == EXPECTED_EPISODES, "terminal matrix must contain 270 rows")
    scheduled_by_identity = {_identity(row): row for row in scheduled}
    observed_by_identity = {}
    for raw in episodes:
        _require(type(raw) is dict, "episode row must be a mapping")
        ident = _identity(raw)
        _require(ident in scheduled_by_identity, "extra or replacement identity present")
        _require(ident not in observed_by_identity, "duplicate terminal identity")
        observed_by_identity[ident] = _normalize_episode(raw, scheduled_by_identity[ident])
    _require(set(observed_by_identity) == set(scheduled_by_identity), "missing terminal identity")
    rows = [observed_by_identity[_identity(item)] for item in scheduled]

    block_methods: Dict[str, Dict[str, Mapping[str, Any]]] = {}
    for row in rows:
        block_methods.setdefault(row["block_id"], {})[row["method_id"]] = row
    pairs = [(values[CONTROL], values[TREATMENT]) for _, values in sorted(block_methods.items())]
    clear_pairs = [(control, treatment) for control, treatment in pairs if control["scene_role"] == CLEAR_ROLE]
    conflict_pairs = [(control, treatment) for control, treatment in pairs if control["scene_role"] in CONFLICT_ROLES]
    _require(len(pairs) == 90 and len(clear_pairs) == 30 and len(conflict_pairs) == 60, "pair populations differ")

    controls = [pair[0] for pair in pairs]
    treatments = [pair[1] for pair in pairs]
    success_harm = sum(c["success"] and not t["success"] for c, t in pairs)
    success_benefit = sum(t["success"] and not c["success"] for c, t in pairs)
    collision_harm = sum(t["collision"] and not c["collision"] for c, t in pairs)
    collision_benefit = sum(c["collision"] and not t["collision"] for c, t in pairs)
    success_harm_upper = clopper_pearson_upper(success_harm, len(pairs))
    collision_harm_upper = clopper_pearson_upper(collision_harm, len(pairs))
    success_role_delta = {
        role: sum(t["success"] for c, t in pairs if c["scene_role"] == role)
        - sum(c["success"] for c, t in pairs if c["scene_role"] == role)
        for role in ROLES
    }
    collision_role_delta = {
        role: sum(t["collision"] for c, t in pairs if c["scene_role"] == role)
        - sum(c["collision"] for c, t in pairs if c["scene_role"] == role)
        for role in ROLES
    }
    emergency_harm = sum(
        t["algorithmic_emergency"] and not c["algorithmic_emergency"]
        for c, t in pairs
    )
    emergency_harm_upper = clopper_pearson_upper(emergency_harm, len(pairs))
    emergency_role_delta = {
        role: sum(t["algorithmic_emergency"] for c, t in pairs if c["scene_role"] == role)
        - sum(c["algorithmic_emergency"] for c, t in pairs if c["scene_role"] == role)
        for role in ROLES
    }
    success_pass = (
        success_harm_upper <= 0.05
        and sum(t["success"] for t in treatments) >= sum(c["success"] for c in controls)
        and all(value >= 0 for value in success_role_delta.values())
    )
    collision_pass = (
        collision_harm_upper <= 0.05
        and sum(t["collision"] for t in treatments) <= sum(c["collision"] for c in controls)
        and all(value <= 0 for value in collision_role_delta.values())
    )
    emergency_pass = (
        emergency_harm_upper <= 0.05
        and sum(t["algorithmic_emergency"] for t in treatments)
        <= sum(c["algorithmic_emergency"] for c in controls)
        and all(value <= 0 for value in emergency_role_delta.values())
    )

    clearance_differences = [
        t["minimum_footprint_clearance_m"] - c["minimum_footprint_clearance_m"]
        for c, t in pairs
    ]
    clearance_strata = [c["scene_role"] for c, _ in pairs]
    clearance_point = sum(clearance_differences) / len(clearance_differences)
    clearance_bootstrap = _bootstrap_means(
        clearance_differences,
        resamples=BOOTSTRAP_RESAMPLES,
        seed=BOOTSTRAP_SEED + 1,
        strata=clearance_strata,
    )
    clearance_lower = _quantile(clearance_bootstrap, ALPHA)
    clearance_role_point = {
        role: sum(
            value for value, stratum in zip(clearance_differences, clearance_strata)
            if stratum == role
        ) / 30.0
        for role in ROLES
    }
    clearance_pass = clearance_lower >= -0.05 and all(value >= -0.05 for value in clearance_role_point.values())

    control_match = sum(c["ttc_status_match"] for c, _ in conflict_pairs) / 60.0
    treatment_match = sum(t["ttc_status_match"] for _, t in conflict_pairs) / 60.0
    ttc_pass = treatment_match >= 0.95 and treatment_match >= control_match

    log_differences = [
        math.log(c["restricted_completion_time_s"])
        - math.log(t["restricted_completion_time_s"])
        for c, t in clear_pairs
    ]
    log_point = sum(log_differences) / len(log_differences)
    time_reduction = 1.0 - math.exp(-log_point)
    efficiency_bootstrap_log = _bootstrap_means(
        log_differences,
        resamples=BOOTSTRAP_RESAMPLES,
        seed=BOOTSTRAP_SEED + 2,
    )
    efficiency_bootstrap_effect = [1.0 - math.exp(-value) for value in efficiency_bootstrap_log]
    efficiency_lower = _quantile(efficiency_bootstrap_effect, ALPHA)
    efficiency_p = _paired_randomization_p(
        log_differences,
        resamples=RANDOMIZATION_RESAMPLES,
        seed=RANDOMIZATION_SEED,
    )
    efficiency_adjusted_p = efficiency_p
    efficiency_pass = time_reduction >= 0.05 and efficiency_lower > 0.0 and efficiency_adjusted_p <= ALPHA

    control_overlay = [c["non_none_overlay_time_fraction"] for c, _ in clear_pairs]
    treatment_overlay = [t["non_none_overlay_time_fraction"] for _, t in clear_pairs]
    overlay_differences = [c - t for c, t in zip(control_overlay, treatment_overlay)]
    control_overlay_mean = sum(control_overlay) / len(control_overlay)
    treatment_overlay_mean = sum(treatment_overlay) / len(treatment_overlay)
    mechanism_defined = control_overlay_mean > 0.0
    mechanism_reduction = (
        1.0 - treatment_overlay_mean / control_overlay_mean
        if mechanism_defined else None
    )
    mechanism_bootstrap_abs = _bootstrap_means(
        overlay_differences,
        resamples=BOOTSTRAP_RESAMPLES,
        seed=BOOTSTRAP_SEED + 3,
    )
    mechanism_abs_lower = _quantile(mechanism_bootstrap_abs, ALPHA)
    mechanism_p = _paired_randomization_p(
        overlay_differences,
        resamples=RANDOMIZATION_RESAMPLES,
        seed=RANDOMIZATION_SEED + 1,
    )
    mechanism_adjusted_p = max(efficiency_p, mechanism_p)
    mechanism_pass = (
        mechanism_defined
        and mechanism_reduction is not None
        and mechanism_reduction >= 0.30
        and mechanism_abs_lower > 0.0
        and mechanism_adjusted_p <= ALPHA
    )

    integrity_pass = True
    safety_pass = success_pass and collision_pass and clearance_pass and ttc_pass and emergency_pass
    four_gates = {
        "integrity": integrity_pass,
        "safety_and_task_noninferiority": safety_pass,
        "efficiency_superiority": efficiency_pass,
        "mechanism_reduction": mechanism_pass,
    }
    all_pass = all(four_gates.values())
    return {
        "schema_version": "1.0",
        "stage": "V2-04G-P1",
        "assessment_status": (
            "all_four_confirmatory_gates_pass"
            if all_pass else "performance_improvement_not_demonstrated"
        ),
        "formal_study_complete": True,
        "formal_result": True,
        "performance_improvement_demonstrated": all_pass,
        "claim_authorized": all_pass,
        "runtime_ready": False,
        "simulation_only": True,
        "claim_scope": (
            "three_frozen_dynamic_templates_under_fresh_simulator_stochastic_seeds_5201_5290"
            if all_pass else None
        ),
        "matrix": {
            "scheduled": 270,
            "consumed_and_retained": 270,
            "confirmatory_pairs": 90,
            "semantic_clear_pairs": 30,
            "true_conflict_pairs": 60,
            "episode_digest": canonical_sha256(episodes),
        },
        "four_gate_decision": four_gates,
        "safety_and_task": {
            "pass": safety_pass,
            "success": {
                "pass": success_pass,
                "harmful_discordances": success_harm,
                "beneficial_discordances": success_benefit,
                "one_sided_exact_harm_rate_upper": success_harm_upper,
                "conservative_risk_difference_lower": -success_harm_upper,
                "role_treatment_minus_control_count": success_role_delta,
            },
            "collision": {
                "pass": collision_pass,
                "harmful_discordances": collision_harm,
                "beneficial_discordances": collision_benefit,
                "one_sided_exact_harm_rate_upper": collision_harm_upper,
                "conservative_risk_difference_upper": collision_harm_upper,
                "role_treatment_minus_control_count": collision_role_delta,
            },
            "clearance": {
                "pass": clearance_pass,
                "paired_mean_difference_m": clearance_point,
                "one_sided_95_percent_lower_m": clearance_lower,
                "role_paired_mean_difference_m": clearance_role_point,
            },
            "true_conflict_semantics": {
                "pass": ttc_pass,
                "control_match_fraction": control_match,
                "treatment_match_fraction": treatment_match,
            },
            "algorithmic_emergency": {
                "pass": emergency_pass,
                "harmful_discordances": emergency_harm,
                "one_sided_exact_harm_rate_upper": emergency_harm_upper,
                "role_treatment_minus_control_count": emergency_role_delta,
            },
        },
        "efficiency": {
            "pass": efficiency_pass,
            "restricted_completion_time_rule": "navigation_time_only_for_clean_success_else_80_second_timeout",
            "geometric_time_reduction_fraction": time_reduction,
            "one_sided_95_percent_lower_fraction": efficiency_lower,
            "raw_p_value": efficiency_p,
            "fixed_sequence_adjusted_p_value": efficiency_adjusted_p,
        },
        "mechanism": {
            "pass": mechanism_pass,
            "metric": "timestamped_non_none_overlay_time_fraction",
            "trace_coverage_required": True,
            "control_mean_fraction": control_overlay_mean,
            "treatment_mean_fraction": treatment_overlay_mean,
            "relative_reduction_fraction": mechanism_reduction,
            "paired_absolute_decrease_one_sided_95_percent_lower": mechanism_abs_lower,
            "raw_p_value": mechanism_p,
            "fixed_sequence_adjusted_p_value": mechanism_adjusted_p,
            "zero_control_mean_policy": "gate_fails_as_relative_reduction_is_undefined",
        },
        "methodology": {
            "bootstrap_resamples": BOOTSTRAP_RESAMPLES,
            "bootstrap_seed": BOOTSTRAP_SEED,
            "paired_randomization_resamples": RANDOMIZATION_RESAMPLES,
            "paired_randomization_seed": RANDOMIZATION_SEED,
            "deterministic_rng": "sha256_counter_p1_r1_v1",
            "bootstrap_quantile": "nearest_rank_ceil_p_times_B_minus_one",
            "binary_noninferiority": "one_sided_clopper_pearson_upper_on_harmful_discordance_rate_conservative_for_paired_risk_difference",
            "fixed_sequence_adjustment": "efficiency_p_adj=p_efficiency; mechanism_p_adj=max(p_efficiency,p_mechanism)",
        },
        "claim_wording_if_pass": (
            "On the preregistered frozen 90-block simulation distribution, circle-contact conflict semantics improved semantically clear restricted completion time relative to legacy semantics while all preregistered safety, task, and mechanism gates passed."
            if all_pass else None
        ),
        "forbidden_claims": [
            "whole-system superiority", "learning gain", "cross-family generalization",
            "real-vehicle performance or safety", "deployment readiness",
        ],
    }


def build_assessment(
    schedule: Mapping[str, Any], episodes: Sequence[Mapping[str, Any]]
) -> Dict[str, Any]:
    """Return a persisted fail report instead of dropping an invalid matrix."""

    try:
        return evaluate(schedule, episodes)
    except (P1StatisticsError, KeyError, TypeError, ValueError) as exc:
        try:
            ledger_digest = canonical_sha256(episodes)
        except (TypeError, ValueError):
            ledger_digest = None
        return {
            "schema_version": "1.0",
            "stage": "V2-04G-P1",
            "assessment_status": "integrity_failure_performance_claim_forbidden",
            "formal_study_complete": False,
            "formal_result": False,
            "performance_improvement_demonstrated": False,
            "claim_authorized": False,
            "runtime_ready": False,
            "simulation_only": True,
            "claim_scope": None,
            "matrix": {
                "scheduled": 270,
                "received": len(episodes) if isinstance(episodes, list) else None,
                "terminal_ledger_digest": ledger_digest,
                "successful_only_analysis_performed": False,
                "failed_or_missing_rows_deleted": False,
            },
            "four_gate_decision": {
                "integrity": False,
                "safety_and_task_noninferiority": False,
                "efficiency_superiority": False,
                "mechanism_reduction": False,
            },
            "integrity_failures": [str(exc)],
            "failure_policy": (
                "persist_terminal_ledger_stop_stage_no_retry_no_replacement_"
                "no_confirmatory_claim"
            ),
            "claim_wording_if_pass": None,
        }
