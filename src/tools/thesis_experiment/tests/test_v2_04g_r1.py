import copy
import hashlib
import importlib.util
from pathlib import Path
import tempfile

import yaml

from teb_mode_manager import AnchorBank, RuleMechanismController
from thesis_experiment.v2_scene import load_v2_scene_manifest


WORKSPACE = Path(__file__).resolve().parents[4]
PREREG = WORKSPACE / "experiments/manifests/v2/calibration/v2_04g_r1_preregistration.yaml"
CONTRACT = WORKSPACE / "config/thesis_experiments/v2/v2_04g_r1_interface_repair_contract.yaml"
CANDIDATES = WORKSPACE / "experiments/manifests/v2/calibration/v2_04g_r1_mechanism_candidates.yaml"
SOURCE_CANDIDATES = WORKSPACE / "experiments/manifests/v2/calibration/v2_04g_mechanism_candidates.yaml"


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _batch_module():
    path = WORKSPACE / "src/tools/thesis_experiment/scripts/v2_04g_r1_calibration_batch.py"
    spec = importlib.util.spec_from_file_location("v2_04g_r1_batch_for_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_r1_preregistration_resources_and_frozen_g_stop_boundary_are_exact():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    assert prereg["stage"] == "V2-04G-R1"
    assert prereg["split"] == "calibration"
    assert prereg["status"] == "preregistered_before_runtime_probe"
    assert prereg["runtime_ready"] is False
    assert prereg["training_allowed"] is False
    assert prereg["real_vehicle_use_forbidden"] is True
    for resource in prereg["resources"].values():
        assert _sha256(WORKSPACE / resource["path"]) == resource["sha256"]
    frozen = prereg["frozen_v2_04g_stop_boundary"]
    assert frozen["reused_as_v2_04g_r1_navigation_evidence"] is False
    for name in ("progress", "stop_report", "ttc_probe"):
        assert _sha256(WORKSPACE / frozen[name]["path"]) == frozen[name]["sha256"]


def test_r1_changes_only_join_factor_and_candidate_numbers_are_identical():
    contract = yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))
    assert contract["scope"]["single_changed_factor"] == "asynchronous_geometry_context_join"
    assert contract["bounded_context_join"] == {
        "geometry_cache_maximum_entries": 32,
        "geometry_cache_maximum_arrival_age_s": 1.0,
        "maximum_sequence_delta": 2,
        "maximum_timestamp_delta_s": 0.45,
        "future_sequence_allowed": False,
        "future_source_timestamp_allowed": False,
        "preference_order": [
            "smallest nonnegative sequence delta",
            "smallest nonnegative source timestamp delta",
            "newest sequence",
        ],
        "accepted_reasons": ["EXACT_SEQUENCE_JOIN", "BOUNDED_SEQUENCE_TIME_JOIN"],
        "no_bounded_match_behavior": "invalidate_context_and_hold_fail_closed",
        "simulation_clock_rollback_behavior": "clear_cache",
    }
    source = yaml.safe_load(SOURCE_CANDIDATES.read_text(encoding="utf-8"))
    target = yaml.safe_load(CANDIDATES.read_text(encoding="utf-8"))
    assert [row["candidate_id"] for row in source["candidates"]] == [
        row["candidate_id"] for row in target["candidates"]
    ]
    for old, new in zip(source["candidates"], target["candidates"]):
        assert old["supervisor_patch"] == new["supervisor_patch"]
        assert old["anchor_patch"] == new["anchor_patch"]
        old_mechanism = copy.deepcopy(old["mechanism"])
        new_mechanism = copy.deepcopy(new["mechanism"])
        if old_mechanism is not None:
            old_mechanism["stage"] = new_mechanism["stage"]
        assert old_mechanism == new_mechanism


def test_r1_seed_firewall_and_activation_probe_are_disjoint_and_fixed():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    firewall = prereg["seed_firewall"]
    navigation = set(firewall["navigation_calibration_seeds"])
    activation = set(firewall["activation_probe_only_seeds"])
    prior = set(firewall["previous_v2_04g_calibration_seeds_forbidden"])
    validation = set(firewall["previous_validation_seeds_forbidden"])
    held_out = set(firewall["reserved_future_held_out_seeds"])
    assert navigation == set(range(4921, 4936))
    assert activation == set(range(4941, 4947))
    assert navigation.isdisjoint(activation | prior | validation | held_out)
    assert activation.isdisjoint(prior | validation | held_out)
    probe = prereg["activation_readiness_probe"]
    assert probe["required_before_navigation"] is True
    assert probe["planned_probe_count"] == 6
    assert [row["seed"] for row in probe["schedule"]] == list(range(4941, 4947))
    assert [row["profile_id"] for row in probe["schedule"]] == (
        ["g1_mechanism_balanced"] * 3 + ["g2_mechanism_aggressive"] * 3
    )


def test_r1_scene_manifest_and_schedule_are_calibration_only():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    manifest = load_v2_scene_manifest(
        WORKSPACE / prereg["resources"]["compiled_source_manifest"]["path"], WORKSPACE
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
    batch = _batch_module()
    with tempfile.TemporaryDirectory() as directory:
        runtime = batch._materialize_candidates(CANDIDATES, directory, "V2-04G-R1")
        assert set(runtime) == set(prereg["candidate_ids"])
        for candidate_id, paths in runtime.items():
            AnchorBank.from_file(paths["anchor_bank"])
            if candidate_id == "g0_frozen_control":
                assert paths["mechanism"] == ""
            else:
                mechanism = yaml.safe_load(Path(paths["mechanism"]).read_text(encoding="utf-8"))
                assert mechanism["stage"] == "V2-04G-R1"
                RuleMechanismController(mechanism)
        instances = batch._load_instances(
            WORKSPACE / "artifacts/v2/calibration/v2_04g_r1/compiled_scenes"
        )
        schedule = batch.build_schedule(prereg, instances, runtime)
        assert len(schedule) == 60
        assert [row["sequence"] for row in schedule] == list(range(1, 61))
        assert sum(row["method"] == "fixed_teb" for row in schedule) == 15
        assert sum(row["method"] == "rule_multi_anchor" for row in schedule) == 45


def test_r1_contract_remains_fail_closed_and_blocks_later_stages():
    contract = yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))
    assert contract["scope"]["total_evidence_unit_budget"] == 69
    assert contract["activation_readiness_probe"]["all_repeats_required"] is True
    assert contract["hard_gates"]["bounded_context_join"][
        "minimum_valid_fraction_per_episode"
    ] == 0.90
    post = contract["post_calibration_gate"]
    assert post["freeze_only_if_all_hard_gates_pass"] is True
    assert post["held_out_validation_generated_only_after_freeze"] is True
    assert post["v2_05_authorized"] is False
    assert post["sac_training_authorized"] is False
    assert post["real_vehicle_authorized"] is False


def test_r1_runtime_interfaces_do_not_expose_labels_or_gazebo_truth():
    launch = (WORKSPACE /
        "src/simulation/m2_gazebo/launch/m2_v2_04g_r1_mechanism_calibration.launch"
    ).read_text(encoding="utf-8")
    transaction = (WORKSPACE /
        "src/application/teb_mode_manager/scripts/v2_04g_r1_typed_anchor_transaction_node.py"
    ).read_text(encoding="utf-8")
    join_source = (WORKSPACE /
        "src/application/teb_mode_manager/src/teb_mode_manager/bounded_context_join.py"
    ).read_text(encoding="utf-8")
    assert "scene_family" not in launch
    assert "scene_id" not in transaction
    assert "ModelStates" not in transaction
    assert "gazebo" not in join_source.lower()
