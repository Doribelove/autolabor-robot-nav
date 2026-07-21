import hashlib
import importlib.util
from pathlib import Path
import tempfile

import yaml

from teb_mode_manager import AnchorBank, RuleMechanismController
from thesis_experiment.v2_scene import load_v2_scene_manifest


WORKSPACE = Path(__file__).resolve().parents[4]
PREREG = WORKSPACE / "experiments/manifests/v2/calibration/v2_04g_r3_preregistration.yaml"
CONTRACT = WORKSPACE / "config/thesis_experiments/v2/v2_04g_r3_full_calibration_contract.yaml"
CANDIDATES = WORKSPACE / "experiments/manifests/v2/calibration/v2_04g_r2_mechanism_candidates.yaml"


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _module(name, relative_path):
    path = WORKSPACE / relative_path
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_r3_resources_and_readiness_boundary_are_byte_exact():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    assert prereg["stage"] == "V2-04G-R3"
    assert prereg["split"] == "calibration"
    assert prereg["runtime_ready"] is False
    assert prereg["training_allowed"] is False
    assert prereg["real_vehicle_use_forbidden"] is True
    for group in ("resources", "frozen_readiness_boundary"):
        for resource in prereg[group].values():
            assert _sha256(WORKSPACE / resource["path"]) == resource["sha256"]


def test_r3_changes_only_fresh_evidence_and_freezes_join_r2_and_taxonomy():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    factor = prereg["single_changed_factor"]
    assert factor == {
        "name": "fresh_readiness_and_navigation_evidence",
        "join_changed": False,
        "r2_candidate_values_changed": False,
        "typed_transaction_runtime_changed": False,
        "readiness_taxonomy_changed": False,
        "evaluator_changed": False,
    }
    contract = yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))
    assert contract["scope"]["changed_factor"] == "evidence_seeds_only"
    assert contract["readiness_taxonomy"]["required_consecutive_stable_count"] == 10
    assert contract["readiness_taxonomy"][
        "maximum_backend_transaction_fault_count"] == 0
    assert contract["readiness_taxonomy"][
        "maximum_unknown_transaction_fault_count"] == 0


def test_r3_seed_firewall_is_fresh_disjoint_and_reserves_held_out():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    firewall = prereg["seed_firewall"]
    readiness = set(firewall["readiness_probe_only_seeds"])
    navigation = set(firewall["navigation_calibration_seeds"])
    prior = set(firewall["all_prior_v2_04g_calibration_and_probe_seeds_forbidden"])
    validation = set(firewall["previous_validation_seeds_forbidden"])
    held_out = set(firewall["reserved_future_held_out_seeds"])
    assert readiness == set(range(4991, 4997))
    assert navigation == set(range(5021, 5036))
    assert held_out == set(range(5001, 5011))
    assert readiness.isdisjoint(navigation | prior | validation | held_out)
    assert navigation.isdisjoint(prior | validation | held_out)
    assert firewall["fresh_r3_data_used_before_preregistration"] is False


def test_r3_scene_schedule_and_frozen_r2_candidates_are_calibration_only():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    manifest = load_v2_scene_manifest(
        WORKSPACE / prereg["resources"]["compiled_source_manifest"]["path"],
        WORKSPACE,
    )
    assert len(manifest["scenes"]) == 15
    assert {scene["split"] for scene in manifest["scenes"]} == {"calibration"}
    assert {scene["seed"] for scene in manifest["scenes"]} == set(range(5021, 5036))
    batch = _module(
        "v2_04g_r3_batch_for_test",
        "src/tools/thesis_experiment/scripts/v2_04g_r3_calibration_batch.py",
    )
    with tempfile.TemporaryDirectory() as directory:
        runtime = batch._R2.materialize_candidates(CANDIDATES, directory)
        assert list(runtime) == prereg["candidate_ids"]
        for paths in runtime.values():
            AnchorBank.from_file(paths["anchor_bank"])
            mechanism = yaml.safe_load(Path(paths["mechanism"]).read_text(
                encoding="utf-8"))
            assert mechanism["stage"] == "V2-04G-R2"
            RuleMechanismController(mechanism)
        instances = batch._BASE._load_instances(
            WORKSPACE / "artifacts/v2/calibration/v2_04g_r3/compiled_scenes")
        schedule = batch.build_schedule(prereg, instances, runtime)
        assert len(schedule) == 60
        assert [row["sequence"] for row in schedule] == list(range(1, 61))
        assert sum(row["method"] == "fixed_teb" for row in schedule) == 15
        assert sum(row["method"] == "rule_multi_anchor" for row in schedule) == 45
        assert {row["stage"] for row in schedule} == {"V2-04G-R3"}


def test_r3_order_budget_and_closed_post_stage_boundaries():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    assert prereg["budget"] == {
        "activation_readiness_probe_count": 6,
        "attempts_per_readiness_probe_max": 1,
        "ttc_component_probe_count": 3,
        "fixed_reference_episode_count": 15,
        "candidate_count": 3,
        "episode_count_per_candidate": 15,
        "planned_navigation_episode_count": 60,
        "attempts_per_navigation_episode_max": 2,
        "total_evidence_unit_budget": 69,
        "budget_expansion_forbidden": True,
    }
    assert prereg["activation_readiness_probe"][
        "required_before_ttc_and_navigation"] is True
    assert prereg["ttc_component_probe"][
        "required_after_readiness_and_before_navigation"] is True
    assert prereg["post_stage_boundaries"] == {
        "use_4991_4996_for_navigation_or_validation": False,
        "use_5021_5035_for_future_validation": False,
        "generate_future_validation_before_freeze": False,
        "v2_05_authorized": False,
        "sac_training_authorized": False,
        "real_vehicle_authorized": False,
    }


def test_r3_runtime_interfaces_still_do_not_expose_scene_truth():
    launch = (WORKSPACE /
        "src/simulation/m2_gazebo/launch/m2_v2_04g_r2_mechanism_calibration.launch"
    ).read_text(encoding="utf-8")
    transaction = (WORKSPACE /
        "src/application/teb_mode_manager/scripts/v2_04g_r2_typed_anchor_transaction_node.py"
    ).read_text(encoding="utf-8")
    assert "scene_family" not in launch
    assert "scene_id" not in transaction
    assert "ModelStates" not in transaction
    assert '"runtime_manifest_access": False' in transaction
