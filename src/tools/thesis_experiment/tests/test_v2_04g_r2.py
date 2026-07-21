import hashlib
import importlib.util
from pathlib import Path
import tempfile

import yaml

from teb_mode_manager import AnchorBank, RuleMechanismController
from thesis_experiment.v2_scene import load_v2_scene_manifest


WORKSPACE = Path(__file__).resolve().parents[4]
PREREG = WORKSPACE / "experiments/manifests/v2/calibration/v2_04g_r2_preregistration.yaml"
CONTRACT = WORKSPACE / "config/thesis_experiments/v2/v2_04g_r2_mechanism_repair_contract.yaml"
CANDIDATES = WORKSPACE / "experiments/manifests/v2/calibration/v2_04g_r2_mechanism_candidates.yaml"


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _batch_module():
    path = WORKSPACE / "src/tools/thesis_experiment/scripts/v2_04g_r2_calibration_batch.py"
    spec = importlib.util.spec_from_file_location("v2_04g_r2_batch_for_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_r2_preregistration_resources_are_exact_and_boundaries_are_closed():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    assert prereg["stage"] == "V2-04G-R2"
    assert prereg["split"] == "calibration"
    assert prereg["status"] == "preregistered_before_activation_probe"
    assert prereg["runtime_ready"] is False
    assert prereg["training_allowed"] is False
    assert prereg["real_vehicle_use_forbidden"] is True
    for resource in prereg["resources"].values():
        assert _sha256(WORKSPACE / resource["path"]) == resource["sha256"]
    assert prereg["post_stage_boundaries"] == {
        "generate_future_validation_before_freeze": False,
        "use_4951_4965_for_future_validation": False,
        "use_4971_4976_for_navigation_or_validation": False,
        "v2_05_authorized": False,
        "sac_training_authorized": False,
        "real_vehicle_authorized": False,
    }


def test_r2_join_source_and_numeric_limits_are_byte_frozen_from_r1():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    frozen = prereg["frozen_infrastructure"]
    assert _sha256(WORKSPACE /
        "src/application/teb_mode_manager/src/teb_mode_manager/bounded_context_join.py"
    ) == frozen["bounded_join_source_sha256"]
    assert _sha256(WORKSPACE /
        "src/application/teb_mode_manager/scripts/v2_04g_r1_typed_anchor_transaction_node.py"
    ) == frozen["r1_typed_join_node_sha256"]
    assert frozen["join_numeric_limits"] == {
        "maximum_entries": 32, "maximum_arrival_age_s": 1.0,
        "maximum_sequence_delta": 2, "maximum_timestamp_delta_s": 0.45,
    }
    assert frozen["join_future_sequence_allowed"] is False
    assert frozen["join_future_timestamp_allowed"] is False


def test_r2_seed_firewall_is_fresh_disjoint_and_reserves_validation():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    firewall = prereg["seed_firewall"]
    navigation = set(firewall["navigation_calibration_seeds"])
    activation = set(firewall["activation_probe_only_seeds"])
    prior = set(firewall["previous_v2_04g_calibration_seeds_forbidden"])
    prior_r1 = set(firewall["previous_v2_04g_r1_calibration_seeds_forbidden"])
    validation = set(firewall["previous_validation_seeds_forbidden"])
    held_out = set(firewall["reserved_future_held_out_seeds"])
    assert navigation == set(range(4951, 4966))
    assert activation == set(range(4971, 4977))
    assert navigation.isdisjoint(activation | prior | prior_r1 | validation | held_out)
    assert activation.isdisjoint(prior | prior_r1 | validation | held_out)
    assert firewall["fresh_r2_navigation_data_used_before_preregistration"] is False
    assert firewall["validation_data_used_for_candidate_ranking"] is False


def test_r2_scene_manifest_schedule_and_candidates_are_calibration_only():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    manifest = load_v2_scene_manifest(
        WORKSPACE / prereg["resources"]["compiled_source_manifest"]["path"], WORKSPACE
    )
    assert len(manifest["scenes"]) == 15
    assert {scene["split"] for scene in manifest["scenes"]} == {"calibration"}
    assert {scene["seed"] for scene in manifest["scenes"]} == set(range(4951, 4966))
    batch = _batch_module()
    with tempfile.TemporaryDirectory() as directory:
        runtime = batch.materialize_candidates(CANDIDATES, directory)
        assert list(runtime) == prereg["candidate_ids"]
        for paths in runtime.values():
            AnchorBank.from_file(paths["anchor_bank"])
            mechanism = yaml.safe_load(Path(paths["mechanism"]).read_text(encoding="utf-8"))
            assert mechanism["stage"] == "V2-04G-R2"
            RuleMechanismController(mechanism)
        instances = batch._R1._load_instances(
            WORKSPACE / "artifacts/v2/calibration/v2_04g_r2/compiled_scenes"
        )
        schedule = batch.build_schedule(prereg, instances, runtime)
        assert len(schedule) == 60
        assert [row["sequence"] for row in schedule] == list(range(1, 61))
        assert sum(row["method"] == "fixed_teb" for row in schedule) == 15
        assert sum(row["method"] == "rule_multi_anchor" for row in schedule) == 45


def test_r2_contract_prioritizes_reverse_and_three_target_families():
    contract = yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))
    assert contract["scope"]["total_evidence_unit_budget"] == 69
    assert contract["idempotent_transaction_semantics"][
        "coalesce_only_if_all_20_typed_values_equal_last_acknowledged_readback"
    ] is True
    assert contract["hard_gates"]["mechanism_activation"] == {
        "maneuver_reverse_episode_count_min": 2,
        "maneuver_reverse_sample_count_min": 3,
    }
    target = contract["hard_gates"]["priority_family_efficiency"]
    assert target["families"] == ["STATIC_DENSE", "CORRIDOR", "MANEUVER"]
    assert target["median_navigation_time_regression_percent_max"] == 15.0
    assert contract["post_calibration_gate"]["sac_training_authorized"] is False
    assert contract["post_calibration_gate"]["real_vehicle_authorized"] is False


def test_r2_runtime_interfaces_do_not_expose_scene_labels_or_truth():
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
