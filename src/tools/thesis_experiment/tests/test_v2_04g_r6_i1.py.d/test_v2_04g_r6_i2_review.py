import copy
import hashlib
import importlib.util
import json
from pathlib import Path

import pytest
import yaml


WORKSPACE = Path(__file__).resolve().parents[5]
SCRIPT = (
    WORKSPACE
    / "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_reviewer.py"
)
CLOSURE = (
    WORKSPACE
    / "artifacts/v2/integration/v2_04g_r6_i1/"
    "r6_i2_repair_review/execution_dependency_closure.yaml"
)


def _module():
    specification = importlib.util.spec_from_file_location(
        "v2_04g_r6_i2_reviewer_test", SCRIPT
    )
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def _document(module, relative):
    return module._snapshot(
        WORKSPACE, relative, yaml_document=True
    )["document"]


def _contract_with_text_placeholders(module):
    document = _document(module, module.CONTRACT_RELATIVE)
    for row in document["resources"].values():
        if row["sha256"] == 0:
            row["sha256"] = "0" * 64
    return document


def test_contract_is_closed_type_sensitive_and_non_authorizing():
    module = _module()
    contract = _contract_with_text_placeholders(module)

    module._verify_contract(contract)
    assert contract["stage"] == "V2-04G-R6-I2"
    assert contract["execution_authorized"] is False
    assert (
        contract["independent_stage_boundary"]["current_stage_seed_values"]
        == []
    )
    assert (
        contract["independent_stage_boundary"][
            "current_stage_execution_schedule"
        ]
        == []
    )
    assert (
        contract["independent_stage_boundary"][
            "current_stage_evidence_budget_authorized"
        ]
        == 0
    )

    extra = copy.deepcopy(contract)
    extra["unexpected"] = False
    with pytest.raises(module.R6I2ReviewError, match="contract keys drifted"):
        module._verify_contract(extra)

    type_confused = copy.deepcopy(contract)
    type_confused["execution_authorized"] = 0
    with pytest.raises(module.R6I2ReviewError, match="contract boundary"):
        module._verify_contract(type_confused)


def test_transition_and_preregistration_preserve_empty_execution_state():
    module = _module()
    transition = _document(module, module.TRANSITION_RELATIVE)
    preregistration = _document(module, module.PREREGISTRATION_RELATIVE)

    module._verify_transition(transition)
    module._verify_preregistration(preregistration)
    assert transition["seed_values"] == preregistration["seed_values"] == []
    assert preregistration["execution_schedule"] == []
    assert transition["evidence_budget_authorized"] == 0
    assert preregistration["evidence_budget_authorized"] == 0

    mutated = copy.deepcopy(preregistration)
    mutated["seed_values"] = [9001]
    with pytest.raises(
        module.R6I2ReviewError, match="preregistration boundary"
    ):
        module._verify_preregistration(mutated)


def test_frozen_factor_profiles_differ_at_one_leaf_and_seven_thresholds_hold():
    module = _module()
    contract = _contract_with_text_placeholders(module)
    preregistration = _document(module, module.PREREGISTRATION_RELATIVE)

    review = module._verify_semantic_boundary(
        WORKSPACE, preregistration, contract
    )

    assert review["leaf_difference_count"] == 1
    assert review["only_leaf_difference"] == (
        "dynamic.conflict_estimator_id"
    )
    assert review["threshold_count"] == 7
    assert review["compiled_scene_index_sha256"] == (
        "1f1cdde389dc98687142ca8d8c47c03bc8391b003d9103bde05c0e41cfddc4a0"
    )
    assert review["evaluator_sha256"] == (
        "55ad5a0abd2d6a8fe41ed9942a512405e42d7be6c41eff779df8a7335496e681"
    )


def test_launch_review_is_xml_only_paused_and_requires_explicit_xacro():
    module = _module()
    main_payload = module._snapshot(
        WORKSPACE, module.MAIN_LAUNCH_RELATIVE
    )["payload"]
    spawn_payload = module._snapshot(
        WORKSPACE, module.SPAWN_LAUNCH_RELATIVE
    )["payload"]

    review = module._verify_launch_xml(main_payload, spawn_payload)

    assert review["xml_parse_only"] is True
    assert review["paused_default"] is True
    assert review["permissive_gate_defaults"] is False
    assert review["xacro_executable_required"] is True
    assert review["process_started"] is False

    unsafe = main_payload.replace(
        b'name="start_typed_transaction" default="false"',
        b'name="start_typed_transaction" default="true"',
    )
    with pytest.raises(
        module.R6I2ReviewError, match="start_typed_transaction"
    ):
        module._verify_launch_xml(unsafe, spawn_payload)


def test_i1_hashes_and_r5_tree_are_unchanged():
    module = _module()

    frozen = module._verify_i1_frozen(WORKSPACE)
    r5 = module._tree_snapshot(WORKSPACE, module.R5_ROOT_RELATIVE)

    assert len(frozen) == 6
    assert r5 == {
        "file_count": 68,
        "tree_sha256": (
            "ecb1f33093dee469008c2ad2d783b3e8ffd1c0739db7903b5df273717e270984"
        ),
    }


def test_component_review_is_closed_and_records_zero_side_effects():
    module = _module()
    component = _document(module, module.COMPONENT_REVIEW_RELATIVE)

    review = module._verify_component_review(component)

    assert review["pass"] is True
    assert review["execution_authorized"] is False
    assert review["seed_values"] == []
    assert review["evidence_budget_units"] == 0
    assert review["ros_or_gazebo_started"] is False

    mutated = copy.deepcopy(component)
    mutated["real_authorization_created"] = 0
    with pytest.raises(module.R6I2ReviewError, match="component review boundary"):
        module._verify_component_review(mutated)


def test_strict_single_open_yaml_rejects_duplicate_keys(tmp_path):
    module = _module()
    workspace = tmp_path.resolve()
    source = workspace / "duplicate.yaml"
    source.write_text("stage: one\nstage: two\n", encoding="utf-8")

    with pytest.raises(module.R6I2ReviewError, match="duplicate YAML key"):
        module._snapshot(workspace, "duplicate.yaml", yaml_document=True)


def test_authorization_and_execution_evidence_absence_fails_closed(tmp_path):
    module = _module()
    workspace = tmp_path.resolve()
    manifest_root = workspace / "experiments/manifests/v2/integration"
    manifest_root.mkdir(parents=True)

    assert module._verify_no_execution_evidence(workspace)["pass"] is True

    authorization = (
        manifest_root / "v2_04g_r6_i2_execution_authorization.yaml"
    )
    authorization.write_text(
        "execution_authorized: true\n", encoding="utf-8"
    )
    with pytest.raises(
        module.R6I2ReviewError, match="execution authorization exists"
    ):
        module._verify_no_execution_evidence(workspace)


def test_reviewer_source_has_no_ros_or_process_launcher_import():
    source = SCRIPT.read_text(encoding="utf-8")

    assert "import rospy" not in source
    assert "import roslaunch" not in source
    assert "import subprocess" not in source
    assert "from subprocess" not in source


def test_review_report_writer_is_atomic_and_value_only(tmp_path):
    module = _module()
    output = tmp_path / "review.yaml"
    value = {
        "stage": "V2-04G-R6-I2",
        "execution_authorized": False,
        "seed_values": [],
        "evidence_budget_authorized": 0,
    }

    module._atomic_yaml(output, value)

    assert yaml.safe_load(output.read_text(encoding="utf-8")) == value
    assert not list(tmp_path.glob("review.yaml.tmp.*"))


@pytest.mark.skipif(
    not CLOSURE.is_file(),
    reason="canonical closure is frozen after all I2 sources stabilize",
)
def test_canonical_closure_rehashes_and_covers_all_39_i1_python_names():
    module = _module()
    closure = _document(module, module.CLOSURE_RELATIVE)

    review = module._verify_dependency_closure(WORKSPACE, closure)

    assert review["inherited_python_binding_count"] == 39
    assert review["inherited_python_binding_coverage_count"] == 39
    assert review["external_python_binding_count"] >= 39
    assert review["external_runtime_binding_count"] == 5
    assert review["unresolved_count"] == 0
    assert review["xacro_target_canonical_path"] == (
        "/opt/ros/noetic/lib/xacro/xacro"
    )
    assert review["process_started"] is False

    mutated = copy.deepcopy(closure)
    mutated["seed_or_evidence_units_consumed"] = 1
    payload = {
        key: value
        for key, value in mutated.items()
        if key != "closure_sha256"
    }
    mutated["closure_sha256"] = hashlib.sha256(
        json.dumps(
            payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True
        ).encode("utf-8")
    ).hexdigest()
    with pytest.raises(module.R6I2ReviewError, match="closure boundary"):
        module._verify_dependency_closure(WORKSPACE, mutated)


@pytest.mark.skipif(
    not CLOSURE.is_file(),
    reason="full review becomes runnable after closure and hashes freeze",
)
def test_full_review_is_deterministic_and_has_zero_side_effects():
    module = _module()
    contract = _document(module, module.CONTRACT_RELATIVE)
    if any(
        not isinstance(row["sha256"], str)
        for row in contract["resources"].values()
    ):
        pytest.skip("new-source contract hashes are not frozen yet")

    first = module.build_review(WORKSPACE)
    second = module.build_review(WORKSPACE)

    assert first == second
    assert first["status"] == (
        "repair_integration_review_pass_execution_not_authorized"
    )
    assert first["execution_authorized"] is False
    assert first["side_effects"] == {
        "authorization_created": False,
        "seed_values": [],
        "execution_schedule": [],
        "seed_or_evidence_units_allocated": 0,
        "seed_or_evidence_units_consumed": 0,
        "ros_started": False,
        "gazebo_started": False,
        "move_base_started": False,
        "training_started": False,
        "real_vehicle_used": False,
        "real_vehicle_teb_written": False,
    }
