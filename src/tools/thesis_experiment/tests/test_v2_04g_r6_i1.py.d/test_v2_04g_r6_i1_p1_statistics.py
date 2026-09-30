import copy
import importlib.util
from pathlib import Path

import pytest
import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
SOURCE = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_p1_statistics.py"
)
SCHEDULE = (
    WORKSPACE
    / "artifacts/v2/integration/v2_04g_r6_i1/p1_r1_offline_readiness/"
    "v2_04g_p1_r1_exact_schedule.yaml"
)


def _module():
    spec = importlib.util.spec_from_file_location("p1_statistics_under_test", SOURCE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


P1 = _module()


def _schedule():
    return yaml.safe_load(SCHEDULE.read_text(encoding="utf-8"))


def _trace(fraction):
    if fraction == 0.0:
        return [{"start_s": 0.0, "end_s": 10.0, "state": "NONE"}]
    return [
        {"start_s": 0.0, "end_s": 10.0 * fraction, "state": "LEGACY_DYNAMIC"},
        {"start_s": 10.0 * fraction, "end_s": 10.0, "state": "NONE"},
    ]


def _episodes(schedule=None):
    schedule = schedule or _schedule()
    rows = []
    for planned in schedule["episodes"]:
        clear = planned["scene_role"] == P1.CLEAR_ROLE
        method = planned["method_id"]
        if clear:
            time_s = {
                P1.CONTROL: 50.0,
                P1.TREATMENT: 45.0,
                P1.REFERENCE: 48.0,
            }[method]
            overlay = {
                P1.CONTROL: 0.6,
                P1.TREATMENT: 0.3,
                P1.REFERENCE: 0.4,
            }[method]
        else:
            time_s, overlay = 50.0, 0.5
        rows.append({
            "sequence": planned["sequence"],
            "method_id": method,
            "scene_id": planned["scene_id"],
            "seed": planned["seed"],
            "attempt": 1,
            "block_id": planned["block_id"],
            "scene_role": planned["scene_role"],
            "consumed": True,
            "terminal_status": "completed",
            "success": True,
            "collision": False,
            "navigation_time_s": time_s,
            "timeout_s": 80.0,
            "minimum_footprint_clearance_m": 0.5,
            "observed_ttc_status": planned["expected_ttc_status"],
            "algorithmic_emergency": False,
            "interface_failure": False,
            "observation_duration_s": 10.0,
            "overlay_intervals": _trace(overlay),
            "raw_resources_complete": True,
            "integrity_failures": [],
        })
    return rows


def _matching(rows, *, method=None, role=None):
    return [
        row for row in rows
        if (method is None or row["method_id"] == method)
        and (role is None or row["scene_role"] == role)
    ]


def test_frozen_schedule_and_exact_binomial_boundaries():
    schedule = _schedule()
    normalized = P1.validate_schedule(schedule)
    assert len(normalized) == 270
    assert P1.clopper_pearson_upper(0, 90) == pytest.approx(0.032738, abs=1e-6)
    assert P1.clopper_pearson_upper(1, 90) == pytest.approx(0.051626, abs=1e-6)
    assert P1.clopper_pearson_upper(0, 60) == pytest.approx(0.048703, abs=1e-6)


def test_all_four_gates_pass_and_replay_is_exact():
    schedule, episodes = _schedule(), _episodes()
    first = P1.build_assessment(schedule, episodes)
    second = P1.build_assessment(schedule, episodes)
    assert first == second
    assert first["assessment_status"] == "all_four_confirmatory_gates_pass"
    assert first["four_gate_decision"] == {
        "integrity": True,
        "safety_and_task_noninferiority": True,
        "efficiency_superiority": True,
        "mechanism_reduction": True,
    }
    assert first["formal_study_complete"] is True
    assert first["performance_improvement_demonstrated"] is True
    assert first["claim_authorized"] is True


def test_missing_duplicate_and_interface_failure_persist_integrity_failure():
    schedule, episodes = _schedule(), _episodes()
    for broken in (
        episodes[:-1],
        episodes[:-1] + [copy.deepcopy(episodes[0])],
    ):
        report = P1.build_assessment(schedule, broken)
        assert report["four_gate_decision"]["integrity"] is False
        assert report["claim_authorized"] is False
        assert report["matrix"]["successful_only_analysis_performed"] is False
    interface = copy.deepcopy(episodes)
    interface[0]["interface_failure"] = True
    assert P1.build_assessment(schedule, interface)["assessment_status"].startswith("integrity_failure")
    process = copy.deepcopy(episodes)
    process[0]["success"] = False
    process[0]["terminal_status"] = "process_failure"
    process[0]["navigation_time_s"] = None
    assert P1.build_assessment(schedule, process)["four_gate_decision"]["integrity"] is False


def test_timeout_collision_and_planner_failure_are_retained_itt_outcomes():
    schedule, episodes = _schedule(), _episodes()
    target = _matching(episodes, method=P1.TREATMENT, role=P1.CLEAR_ROLE)[0]
    target.update(success=False, terminal_status="timeout", navigation_time_s=None)
    report = P1.build_assessment(schedule, episodes)
    assert report["formal_study_complete"] is True
    assert report["matrix"]["consumed_and_retained"] == 270
    assert report["performance_improvement_demonstrated"] is False

    planner = _episodes()
    target = _matching(planner, method=P1.TREATMENT, role=P1.CLEAR_ROLE)[0]
    target.update(success=False, terminal_status="planner_failure", navigation_time_s=12.0)
    assert P1.build_assessment(schedule, planner)["formal_study_complete"] is True

    collision = _episodes()
    target = _matching(collision, method=P1.TREATMENT)[0]
    target.update(success=False, collision=True, terminal_status="collision", navigation_time_s=12.0)
    report = P1.build_assessment(schedule, collision)
    assert report["safety_and_task"]["collision"]["pass"] is False
    assert report["claim_authorized"] is False


def test_each_confirmatory_gate_can_independently_veto_claim():
    schedule = _schedule()

    efficiency = _episodes()
    for row in _matching(efficiency, method=P1.TREATMENT, role=P1.CLEAR_ROLE):
        row["navigation_time_s"] = 50.0
    report = P1.build_assessment(schedule, efficiency)
    assert report["four_gate_decision"]["efficiency_superiority"] is False
    assert report["formal_study_complete"] is True
    assert report["claim_authorized"] is False

    mechanism = _episodes()
    for row in _matching(mechanism, method=P1.TREATMENT, role=P1.CLEAR_ROLE):
        row["overlay_intervals"] = _trace(0.6)
    report = P1.build_assessment(schedule, mechanism)
    assert report["four_gate_decision"]["mechanism_reduction"] is False
    assert report["claim_authorized"] is False

    clearance = _episodes()
    for row in _matching(clearance, method=P1.TREATMENT):
        row["minimum_footprint_clearance_m"] = 0.44
    report = P1.build_assessment(schedule, clearance)
    assert report["safety_and_task"]["clearance"]["pass"] is False

    ttc = _episodes()
    for row in _matching(ttc, method=P1.TREATMENT)[:4]:
        if row["scene_role"] in P1.CONFLICT_ROLES:
            row["observed_ttc_status"] = "NO_CONFLICT_IN_HORIZON"
    # Select four conflict rows even if schedule prefix included fewer above.
    changed = sum(
        row["observed_ttc_status"] == "NO_CONFLICT_IN_HORIZON"
        and row["scene_role"] in P1.CONFLICT_ROLES
        for row in _matching(ttc, method=P1.TREATMENT)
    )
    if changed < 4:
        for row in _matching(ttc, method=P1.TREATMENT):
            if row["scene_role"] in P1.CONFLICT_ROLES and row["observed_ttc_status"] != "NO_CONFLICT_IN_HORIZON":
                row["observed_ttc_status"] = "NO_CONFLICT_IN_HORIZON"
                changed += 1
                if changed == 4:
                    break
    report = P1.build_assessment(schedule, ttc)
    assert report["safety_and_task"]["true_conflict_semantics"]["pass"] is False


def test_zero_control_mechanism_denominator_and_overlay_trace_errors_fail():
    schedule = _schedule()
    zero = _episodes()
    for row in _matching(zero, role=P1.CLEAR_ROLE):
        row["overlay_intervals"] = _trace(0.0)
    report = P1.build_assessment(schedule, zero)
    assert report["mechanism"]["relative_reduction_fraction"] is None
    assert report["four_gate_decision"]["mechanism_reduction"] is False

    gap = _episodes()
    gap[0]["overlay_intervals"][0]["start_s"] = 0.1
    assert P1.build_assessment(schedule, gap)["four_gate_decision"]["integrity"] is False
    unknown = _episodes()
    unknown[0]["overlay_intervals"][0]["state"] = "UNREGISTERED_STATE"
    assert P1.build_assessment(schedule, unknown)["four_gate_decision"]["integrity"] is False


def test_schedule_rejects_wrong_role_seed_attempt_and_wave_drift():
    original = _schedule()
    mutations = []
    wrong_seed = copy.deepcopy(original)
    wrong_seed["episodes"][0]["seed"] = 5261
    mutations.append(wrong_seed)
    attempt = copy.deepcopy(original)
    attempt["episodes"][0]["attempt"] = 2
    mutations.append(attempt)
    wave = copy.deepcopy(original)
    wave["episodes"][0]["wave"] = 2
    mutations.append(wave)
    for schedule in mutations:
        schedule["canonical_episode_digest"] = P1.canonical_sha256(schedule["episodes"])
        with pytest.raises(P1.P1StatisticsError):
            P1.validate_schedule(schedule)

