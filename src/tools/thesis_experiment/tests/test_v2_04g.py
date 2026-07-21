import hashlib
import importlib.util
from pathlib import Path
import tempfile

import yaml

from teb_mode_manager import AnchorBank, load_mechanism_config
from thesis_experiment.v2_scene import load_v2_scene_manifest


WORKSPACE = Path(__file__).resolve().parents[4]
PREREG = (
    WORKSPACE
    / "experiments/manifests/v2/calibration/v2_04g_preregistration.yaml"
)
CONTRACT = (
    WORKSPACE
    / "config/thesis_experiments/v2/v2_04g_mechanism_repair_contract.yaml"
)
CANDIDATES = (
    WORKSPACE
    / "experiments/manifests/v2/calibration/v2_04g_mechanism_candidates.yaml"
)


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _batch_module():
    path = (
        WORKSPACE
        / "src/tools/thesis_experiment/scripts/v2_04g_calibration_batch.py"
    )
    spec = importlib.util.spec_from_file_location("v2_04g_batch_for_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_v2_04g_preregistration_is_frozen_calibration_only():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    assert prereg["stage"] == "V2-04G"
    assert prereg["status"] == "frozen_before_first_navigation_episode"
    assert prereg["split"] == "calibration"
    assert prereg["runtime_ready"] is False
    assert prereg["training_allowed"] is False
    assert prereg["real_vehicle_use_forbidden"] is True
    assert prereg["budget"] == {
        "fixed_reference_episode_count": 15,
        "candidate_count": 3,
        "episode_count_per_candidate": 15,
        "planned_navigation_episode_count": 60,
        "tracker_invalid_component_probe_count": 3,
        "total_evidence_unit_budget": 63,
        "attempts_per_navigation_episode_max": 2,
        "budget_expansion_forbidden": True,
    }
    assert prereg["candidate_ids"] == [
        "g0_frozen_control", "g1_mechanism_balanced", "g2_mechanism_aggressive"
    ]
    for resource in prereg["resources"].values():
        assert _sha256(WORKSPACE / resource["path"]) == resource["sha256"]


def test_v2_04g_seed_firewall_and_five_family_scene_balance():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    firewall = prereg["seed_firewall"]
    calibration = set(firewall["calibration_seeds"])
    forbidden = set(firewall["forbidden_validation_seeds"])
    reserved = set(firewall["reserved_future_held_out_seeds"])
    assert calibration == set(range(4901, 4916))
    assert calibration.isdisjoint(forbidden)
    assert calibration.isdisjoint(reserved)
    assert reserved == set(range(5001, 5011))
    manifest = load_v2_scene_manifest(
        WORKSPACE / prereg["resources"]["compiled_source_manifest"]["path"],
        WORKSPACE,
    )
    assert len(manifest["scenes"]) == 15
    assert {scene["split"] for scene in manifest["scenes"]} == {"calibration"}
    by_family = {}
    for scene in manifest["scenes"]:
        by_family[scene["family"]] = by_family.get(scene["family"], 0) + 1
    assert by_family == {
        "CRUISE": 3, "DYNAMIC": 3, "STATIC_DENSE": 3,
        "CORRIDOR": 3, "MANEUVER": 3,
    }


def test_v2_04g_contract_is_fail_closed_and_blocks_training():
    contract = yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))
    assert contract["scope"]["planned_navigation_episode_count"] == 60
    assert contract["scope"]["total_evidence_unit_budget"] == 63
    assert contract["ttc_semantics"]["statuses"] == [
        "OBSERVED_CONFLICT", "NO_CONFLICT_IN_HORIZON", "TRACKER_INVALID"
    ]
    assert contract["hard_gates"]["candidate_safety"][
        "success_count_not_below_fixed"
    ] is True
    assert contract["hard_gates"]["candidate_safety"][
        "minimum_clearance_m_min_per_successful_episode"
    ] == 0.25
    post = contract["post_calibration_gate"]
    assert post["freeze_only_if_all_hard_gates_pass"] is True
    assert post["held_out_validation_generated_only_after_freeze"] is True
    assert post["v2_05_authorized"] is False
    assert post["sac_training_authorized"] is False
    assert post["real_vehicle_authorized"] is False


def test_v2_04g_materialized_candidates_and_schedule_are_strict():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    batch = _batch_module()
    with tempfile.TemporaryDirectory() as directory:
        runtime = batch._materialize_candidates(CANDIDATES, directory, "V2-04G")
        assert set(runtime) == set(prereg["candidate_ids"])
        for candidate_id, paths in runtime.items():
            AnchorBank.from_file(paths["anchor_bank"])
            if candidate_id == "g0_frozen_control":
                assert paths["mechanism"] == ""
            else:
                config = load_mechanism_config(paths["mechanism"])
                assert config["profile_id"].endswith(candidate_id + "_mechanism")
        instances = batch._load_instances(
            WORKSPACE / "artifacts/v2/calibration/v2_04g/compiled_scenes"
        )
        schedule = batch.build_schedule(prereg, instances, runtime)
        assert len(schedule) == 60
        assert [row["sequence"] for row in schedule] == list(range(1, 61))
        assert sum(row["method"] == "fixed_teb" for row in schedule) == 15
        assert sum(row["method"] == "rule_multi_anchor" for row in schedule) == 45


def test_v2_04g_runtime_policy_interfaces_do_not_expose_labels_or_truth():
    launch = (WORKSPACE /
        "src/simulation/m2_gazebo/launch/m2_v2_04g_mechanism_calibration.launch"
    ).read_text(encoding="utf-8")
    mechanism = (WORKSPACE /
        "src/application/teb_mode_manager/src/teb_mode_manager/mechanism_controller.py"
    ).read_text(encoding="utf-8")
    transaction = (WORKSPACE /
        "src/application/teb_mode_manager/scripts/v2_04g_typed_anchor_transaction_node.py"
    ).read_text(encoding="utf-8")
    assert "scene_family" not in launch
    assert "ModelStates" not in transaction
    assert "gazebo" not in mechanism.lower()
    assert "scene_id" not in mechanism
