import hashlib
from pathlib import Path

import yaml

from thesis_experiment.v2_scene import load_v2_scene_manifest


WORKSPACE = Path(__file__).resolve().parents[4]
CONTRACT = WORKSPACE / "config/thesis_experiments/v2/v2_04d_paired_validation_contract.yaml"
PREREGISTRATION = (
    WORKSPACE / "experiments/manifests/v2/validation/v2_04d_preregistration.yaml"
)


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def test_v2_04d_contract_is_frozen_simulation_only_and_resources_match():
    contract = yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))
    assert contract["status"] == "preregistered_simulation_only"
    assert contract["simulation_only"] is True
    assert contract["runtime_ready"] is False
    assert contract["training_allowed"] is False
    assert contract["real_vehicle_use_forbidden"] is True
    assert contract["methods"]["order"] == [
        "fixed_teb", "balanced_anchor", "rule_multi_anchor"
    ]
    assert contract["pairing"]["planned_navigation_episode_count"] == 30
    assert contract["stage_2_comparison"][
        "authorized_only_after_all_stage_1_hard_gates"
    ] is True
    for resource in contract["frozen_inputs"].values():
        assert _sha256(WORKSPACE / resource["path"]) == resource["sha256"]


def test_v2_04d_preregistration_and_scene_split_are_label_closed():
    prereg = yaml.safe_load(PREREGISTRATION.read_text(encoding="utf-8"))
    assert prereg["status"] == "frozen_before_navigation"
    assert prereg["training_started"] is False
    assert prereg["claims"]["runtime_scene_labels_available"] is False
    assert prereg["budget"] == {
        "method_count": 3,
        "validation_scene_count": 10,
        "planned_navigation_episode_count": 30,
        "attempts_per_interface_failure": 2,
        "early_stopping_allowed": False,
    }
    scenes = load_v2_scene_manifest(
        WORKSPACE / prereg["resources"]["validation_scenes"]["path"], WORKSPACE
    )
    assert len(scenes["scenes"]) == 10
    assert {row["split"] for row in scenes["scenes"]} == {"validation"}
    assert {row["family"] for row in scenes["scenes"]} == {
        "CRUISE", "DYNAMIC", "STATIC_DENSE", "CORRIDOR", "MANEUVER"
    }
    assert len({row["seed"] for row in scenes["scenes"]}) == 10


def test_v2_04d_runtime_comparators_do_not_receive_scene_labels():
    launch = (WORKSPACE /
        "src/simulation/m2_gazebo/launch/m2_v2_04d_paired_validation.launch"
    ).read_text(encoding="utf-8")
    supervisor = (WORKSPACE /
        "src/application/teb_mode_manager/config/v2_04d_rule_supervisor_frozen.yaml"
    ).read_text(encoding="utf-8")
    transaction = (WORKSPACE /
        "src/application/teb_mode_manager/scripts/simulation_typed_anchor_transaction_node.py"
    ).read_text(encoding="utf-8")
    assert "manifest" not in transaction.lower()
    assert "scene" not in transaction.lower()
    assert "runtime_manifest_access: false" in supervisor
    assert "runtime_scene_labels_allowed: false" in supervisor
    assert "force_geometry_balanced" in launch
    assert "v2_04c_anchor_bank_frozen.yaml" in launch


def test_completed_v2_04d_proves_non_degradation_but_not_performance_effectiveness():
    path = WORKSPACE / "artifacts/v2/validation/v2_04d/v2_04d_paired_assessment.yaml"
    if not path.is_file():
        return
    report = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert report["evidence"]["valid_paired_episode_count"] == 30
    assert report["stage_1_hard_gates"]["all_stage_1_hard_gates_pass"] is True
    assert report["decision"]["success_non_degradation_proven"] is True
    assert report["decision"]["performance_effectiveness_proven"] is False
    assert report["decision"]["enter_v2_05_authorized"] is False
    assert report["decision"]["sac_training_authorized"] is False
    assert report["decision"]["real_vehicle_authorized"] is False
    assert report["method_summaries"]["fixed_teb"]["success_count"] == 10
    assert report["method_summaries"]["balanced_anchor"][
        "distinct_active_anchor_ids"
    ] == ["anchor_balanced"]
    assert report["method_summaries"]["rule_multi_anchor"][
        "distinct_active_anchor_id_count"
    ] >= 3
    assert report["ttc_evidence_quality"]["tracker_invalid_count"] == 0
    assert report["ttc_evidence_quality"]["cross_method_ttc_coverage_pass"] is False
