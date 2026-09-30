import importlib.util
from pathlib import Path

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
BUILDER_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_release_builder.py"
)
RUNNER_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_exact_runner.py"
)
ASSESSOR_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_assess.py"
)


def _module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


BUILDER = _module("p2_c1_builder_under_test", BUILDER_PATH)
RUNNER = _module("p2_c1_runner_under_test", RUNNER_PATH)
ASSESSOR = _module("p2_c1_assessor_under_test", ASSESSOR_PATH)


def _yaml(files, relative):
    return yaml.safe_load(files[relative].decode("utf-8"))


def test_c1_reorders_only_execution_and_preserves_every_q1_source_row():
    files = BUILDER.build_bundle(WORKSPACE)
    schedule = _yaml(files, BUILDER.SCHEDULE_RELATIVE)
    assert len(schedule["rows"]) == 342
    assert all(
        row["row_type"] == "qualification_probe"
        for row in schedule["rows"][:12]
    )
    assert all(
        row["row_type"] == "calibration_navigation"
        for row in schedule["rows"][12:]
    )
    assert {row["q1_source_sequence"] for row in schedule["rows"]} == set(
        range(1, 343)
    )
    assert schedule["behavioral_factor_count_changed"] == 0
    assert schedule["held_out_test_row_count"] == 0
    assert not set(range(5401, 5491)) & {
        row["seed"] for row in schedule["rows"]
    }


def test_c1_command_plan_repairs_semantic_bindings_and_direct_bootstrap():
    files = BUILDER.build_bundle(WORKSPACE)
    plan = _yaml(files, BUILDER.PLAN_RELATIVE)
    semantic = [
        row
        for row in plan["rows"]
        if row["semantic_transaction_launch_argv"] is not None
    ]
    assert len(semantic) == 154
    for row in semantic:
        base = row["base_launch_argv"]
        assert sum(
            item.startswith("rule_supervisor_config:=") for item in base
        ) == 1
        assert sum(
            item.startswith("rule_supervisor_config_sha256:=")
            for item in base
        ) == 1
        assert "publish_teb_obstacles:=true" in base
    direct = [
        row
        for row in plan["rows"][:12]
        if "start_move_base:=false" in row["base_launch_argv"]
    ]
    assert len(direct) == 8
    assert all(
        row["bootstrap_mode"] == "direct_probe_positive_clock_only"
        for row in direct
    )


def test_c1_dependency_closure_is_closed_and_authority_is_finite():
    files = BUILDER.build_bundle(WORKSPACE)
    closure = _yaml(files, BUILDER.CLOSURE_RELATIVE)
    report = _yaml(files, BUILDER.REPORT_RELATIVE)
    assert closure["unresolved"] == []
    assert closure["local_resource_count"] == len(closure["local_resources"])
    assert closure["external_resource_count"] == len(
        closure["external_resources"]
    )
    assert closure["authorized_evidence_units_max"] == 342
    assert report["release_present"] is False
    assert report["consumed_evidence_units"] == 0
    assert report["held_out_test_rows"] == 0


def test_c1_monotone_qualification_gate_skips_failed_speed_and_higher_only():
    qualification = {
        "row_type": "qualification_probe",
        "candidate_speed_mps": 1.35,
    }
    lower = dict(qualification, candidate_speed_mps=1.30)
    high_circle = {
        "row_type": "calibration_navigation",
        "method_id": "circle_contact_v140",
    }
    low_fixed = {
        "row_type": "calibration_navigation",
        "method_id": "fixed_teb_v130",
    }
    tuned = {
        "row_type": "calibration_navigation",
        "method_id": "TEB-Tuned-V1",
    }
    assert RUNNER._should_skip(qualification, 1.35) is False
    assert RUNNER._should_skip(lower, 1.35) is False
    assert RUNNER._should_skip(high_circle, 1.35) is True
    assert RUNNER._should_skip(low_fixed, 1.35) is False
    assert RUNNER._should_skip(tuned, 1.35) is False


def test_c1_tie_rules_prefer_lower_speed_and_then_tuned_baseline():
    candidates = [
        {
            "method_id": "circle_contact_v130",
            "semantic_clear_geometric_restricted_time_s": 10.0,
        },
        {
            "method_id": "circle_contact_v125",
            "semantic_clear_geometric_restricted_time_s": 10.05,
        },
    ]
    selected = ASSESSOR._pick_within_one_percent(
        candidates, "semantic_clear_geometric_restricted_time_s"
    )
    assert selected["method_id"] == "circle_contact_v125"
    fair = [
        {
            "method_id": "TEB-Tuned-V1",
            "equal_role_geometric_restricted_time_s": 10.05,
        },
        {
            "method_id": "fixed_teb_v140",
            "equal_role_geometric_restricted_time_s": 10.0,
        },
    ]
    selected_fair = ASSESSOR._pick_within_one_percent(
        fair,
        "equal_role_geometric_restricted_time_s",
        prefer_tuned=True,
    )
    assert selected_fair["method_id"] == "TEB-Tuned-V1"
