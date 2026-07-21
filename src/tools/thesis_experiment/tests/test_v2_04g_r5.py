import hashlib
import importlib.util
from pathlib import Path
import shutil
import tempfile

import pytest
import yaml


WORKSPACE = Path(__file__).resolve().parents[4]
PREREG = (
    WORKSPACE
    / "experiments/manifests/v2/calibration/v2_04g_r5_preregistration.yaml"
)
BANK = (
    WORKSPACE
    / "experiments/manifests/v2/calibration/v2_04g_r5_ttc_timing_candidates.yaml"
)
CONTRACT = (
    "config/thesis_experiments/v2/v2_04g_r5_ttc_robustness_contract.yaml"
)
NAVIGATION_DERIVATION = (
    "experiments/manifests/v2/calibration/v2_04g_r5_scene_derivation.yaml"
)
READINESS_DERIVATION = (
    "experiments/manifests/v2/calibration/"
    "v2_04g_r5_ttc_readiness_scene_derivation.yaml"
)
NAVIGATION_MANIFEST = (
    "artifacts/v2/calibration/v2_04g_r5/v2_04g_r5_calibration_scenes.yaml"
)


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _module(name, relative_path):
    path = WORKSPACE / relative_path
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _walk_resource_paths(value):
    if isinstance(value, dict):
        if (
            isinstance(value.get("path"), str)
            and isinstance(value.get("sha256"), str)
        ):
            yield value["path"]
        for child in value.values():
            yield from _walk_resource_paths(child)
    elif isinstance(value, list):
        for child in value:
            yield from _walk_resource_paths(child)


def _copy_validation_workspace(destination):
    """Copy the preregistered resource closure into a small isolated workspace."""
    destination = Path(destination)
    prereg_relative = PREREG.relative_to(WORKSPACE)
    pending = [str(prereg_relative)]
    copied = set()
    while pending:
        relative = pending.pop()
        if relative in copied:
            continue
        source = WORKSPACE / relative
        assert source.is_file(), relative
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(str(source), str(target))
        copied.add(relative)
        if target.suffix not in (".yaml", ".yml"):
            continue
        document = yaml.safe_load(target.read_text(encoding="utf-8"))
        for nested in _walk_resource_paths(document):
            if not Path(nested).is_absolute() and nested not in copied:
                pending.append(nested)
    return destination / prereg_relative


def _rewrite_yaml(path, document):
    Path(path).write_text(
        yaml.safe_dump(document, sort_keys=False), encoding="utf-8"
    )


def _replace_declared_hash(value, resource_path, digest):
    matches = 0
    if isinstance(value, dict):
        if value.get("path") == resource_path and "sha256" in value:
            value["sha256"] = digest
            matches += 1
        for child in value.values():
            matches += _replace_declared_hash(child, resource_path, digest)
    elif isinstance(value, list):
        for child in value:
            matches += _replace_declared_hash(child, resource_path, digest)
    return matches


def _mutate_resource(workspace, preregistration, relative_path, mutation):
    target = Path(workspace) / relative_path
    document = yaml.safe_load(target.read_text(encoding="utf-8"))
    mutation(document)
    _rewrite_yaml(target, document)
    prereg = yaml.safe_load(Path(preregistration).read_text(encoding="utf-8"))
    assert _replace_declared_hash(prereg, relative_path, _sha256(target)) == 1
    _rewrite_yaml(preregistration, prereg)


def _mutate_preregistration(preregistration, mutation):
    document = yaml.safe_load(
        Path(preregistration).read_text(encoding="utf-8")
    )
    mutation(document)
    _rewrite_yaml(preregistration, document)


def _assert_validation_rejects(workspace, preregistration):
    validator = _module(
        "validate_v2_04g_r5_mutation",
        "src/tools/thesis_experiment/scripts/validate_v2_04g_r5.py",
    )
    audit = (
        Path(workspace)
        / "artifacts/v2/calibration/v2_04g_r5/rejected-audit.yaml"
    )
    with pytest.raises(ValueError):
        validator.validate(workspace, preregistration, audit)
    assert not audit.exists()


def _semantic_diff(left, right, prefix=()):
    if isinstance(left, dict) and isinstance(right, dict):
        keys = set(left) | set(right)
        output = []
        for key in sorted(keys):
            if key not in left or key not in right:
                output.append(prefix + (key,))
            else:
                output.extend(_semantic_diff(left[key], right[key], prefix + (key,)))
        return output
    if isinstance(left, list) and isinstance(right, list):
        if len(left) != len(right):
            return [prefix]
        output = []
        for index, (left_item, right_item) in enumerate(zip(left, right)):
            output.extend(
                _semantic_diff(left_item, right_item, prefix + (str(index),))
            )
        return output
    return [] if left == right else [prefix]


def test_r5_materializer_has_exactly_one_behavioral_diff():
    materializer = _module(
        "v2_04g_r5_materializer_test",
        "src/tools/thesis_experiment/scripts/v2_04g_r5_candidate_materializer.py",
    )
    bank = yaml.safe_load(BANK.read_text(encoding="utf-8"))
    frozen = bank["frozen_m030_input"]
    base_supervisor = yaml.safe_load(
        (WORKSPACE / frozen["supervisor"]["path"]).read_text(encoding="utf-8")
    )
    base_anchor = yaml.safe_load(
        (WORKSPACE / frozen["anchor_bank"]["path"]).read_text(encoding="utf-8")
    )
    base_mechanism = yaml.safe_load(
        (WORKSPACE / frozen["mechanism"]["path"]).read_text(encoding="utf-8")
    )
    with tempfile.TemporaryDirectory() as directory:
        runtime = materializer.materialize_candidates(BANK, directory)
        for row in bank["candidates"]:
            paths = runtime[row["candidate_id"]]
            supervisor = yaml.safe_load(
                Path(paths["supervisor"]).read_text(encoding="utf-8")
            )
            anchor = yaml.safe_load(
                Path(paths["anchor_bank"]).read_text(encoding="utf-8")
            )
            mechanism = yaml.safe_load(
                Path(paths["mechanism"]).read_text(encoding="utf-8")
            )
            supervisor["profile_id"] = base_supervisor["profile_id"]
            anchor["bank_id"] = base_anchor["bank_id"]
            mechanism["profile_id"] = base_mechanism["profile_id"]
            expected_diff = (
                []
                if row["predicted_ttc_max_s"] == 5.0
                else [("dynamic", "predicted_ttc_max_s")]
            )
            assert _semantic_diff(base_supervisor, supervisor) == expected_diff
            assert supervisor["dynamic"]["predicted_ttc_max_s"] == row[
                "predicted_ttc_max_s"
            ]
            assert anchor == base_anchor
            assert mechanism == base_mechanism
            for anchor_id in (
                "anchor_maneuver_forward",
                "anchor_maneuver_reverse",
            ):
                values = anchor["anchors"][anchor_id]["values"]
                assert values["min_obstacle_dist"] == 0.30
                assert values["inflation_dist"] == 0.52


def test_r5_validator_accepts_only_the_no_ros_dry_run_design(tmp_path):
    validator = _module(
        "validate_v2_04g_r5_positive",
        "src/tools/thesis_experiment/scripts/validate_v2_04g_r5.py",
    )
    workspace = tmp_path / "workspace"
    preregistration = _copy_validation_workspace(workspace)
    output = (
        workspace
        / "artifacts/v2/calibration/v2_04g_r5/test-audit.yaml"
    )
    audit = validator.validate(workspace, preregistration, output)
    assert output.is_file()
    assert audit["stage"] == "V2-04G-R5"
    assert audit["status"] == "dry_run_audit_pass"
    assert audit["single_changed_factor"]["runtime_field"] == (
        "supervisor.dynamic.predicted_ttc_max_s"
    )
    assert audit["budget"]["total_evidence_unit_budget"] == 69
    assert audit["readiness_plan"]["schedule_identity_count"] == 6
    assert audit["navigation_plan"]["schedule_episode_count"] == 60
    assert audit["side_effects"] == {
        "ros_started": False,
        "gazebo_started": False,
        "navigation_episodes_started": 0,
        "ttc_probe_episodes_started": 0,
        "sac_training_started": False,
        "runtime_configs_persisted": 0,
        "progress_files_persisted": 0,
        "only_persistent_write_is_this_audit": True,
    }
    assert all(
        value is False
        for value in audit["authorization_after_audit"].values()
    )


def test_r5_validator_rejects_factor_mutation(tmp_path):
    workspace = tmp_path / "workspace"
    preregistration = _copy_validation_workspace(workspace)

    def mutate(document):
        document["single_changed_factor"]["candidate_values_s"] = [
            5.0,
            4.6,
            4.0,
        ]

    _mutate_preregistration(preregistration, mutate)
    _assert_validation_rejects(workspace, preregistration)


def test_r5_validator_rejects_weakened_readiness_fraction(tmp_path):
    workspace = tmp_path / "workspace"
    preregistration = _copy_validation_workspace(workspace)

    def mutate(document):
        document["ttc_activation_coverage_readiness"][
            "minimum_transaction_valid_fraction"
        ] = 0.50

    _mutate_preregistration(preregistration, mutate)
    _assert_validation_rejects(workspace, preregistration)


def test_r5_validator_rejects_contract_budget_drift(tmp_path):
    workspace = tmp_path / "workspace"
    preregistration = _copy_validation_workspace(workspace)

    def mutate(document):
        document["scope"]["total_evidence_unit_budget"] = 70

    _mutate_resource(workspace, preregistration, CONTRACT, mutate)
    _assert_validation_rejects(workspace, preregistration)


def test_r5_validator_rejects_contract_ttc_coverage_weakening(tmp_path):
    workspace = tmp_path / "workspace"
    preregistration = _copy_validation_workspace(workspace)

    def mutate(document):
        document["ttc_coverage_gate"][
            "full_navigation_expected_observed_conflict_scene_count_per_method"
        ] = 1

    _mutate_resource(workspace, preregistration, CONTRACT, mutate)
    _assert_validation_rejects(workspace, preregistration)


def test_r5_validator_rejects_coordinated_hard_gate_weakening(tmp_path):
    workspace = tmp_path / "workspace"
    preregistration = _copy_validation_workspace(workspace)

    def mutate_contract(document):
        document["hard_gates"]["candidate_safety"]["collision_count_max"] = 1

    def mutate_preregistration(document):
        document["hard_gates"]["candidate_safety"]["collision_count_max"] = 1

    _mutate_resource(
        workspace, preregistration, CONTRACT, mutate_contract
    )
    _mutate_preregistration(preregistration, mutate_preregistration)
    _assert_validation_rejects(workspace, preregistration)


def test_r5_validator_rejects_coordinated_historical_seed_deletion(tmp_path):
    workspace = tmp_path / "workspace"
    preregistration = _copy_validation_workspace(workspace)

    def mutate_derivation(document):
        document["forbidden_seed_sets"][
            "all_prior_v2_04g_calibration_and_probe"
        ].remove(5105)

    def mutate_preregistration(document):
        document["seed_firewall"][
            "all_prior_v2_04g_calibration_and_probe_seeds_forbidden"
        ].remove(5105)

    _mutate_resource(
        workspace, preregistration, NAVIGATION_DERIVATION, mutate_derivation
    )
    _mutate_resource(
        workspace, preregistration, READINESS_DERIVATION, mutate_derivation
    )
    _mutate_preregistration(preregistration, mutate_preregistration)
    _assert_validation_rejects(workspace, preregistration)


def test_r5_validator_rejects_heldout_seed_overlap(tmp_path):
    workspace = tmp_path / "workspace"
    preregistration = _copy_validation_workspace(workspace)

    def mutate(document):
        document["seed_firewall"]["navigation_calibration_seeds"][0] = 5001

    _mutate_preregistration(preregistration, mutate)
    _assert_validation_rejects(workspace, preregistration)


@pytest.mark.parametrize(
    "mutation",
    [
        lambda document: document["budget"].__setitem__(
            "total_evidence_unit_budget", 70
        ),
        lambda document: document["navigation_schedule"].__setitem__(
            "attempts_per_identity_max", 2
        ),
        lambda document: document["budget"].__setitem__(
            "terminal_failure_identity_retry_forbidden", False
        ),
        lambda document: document["budget"].__setitem__(
            "resume_after_any_terminal_failure_forbidden", False
        ),
    ],
    ids=[
        "budget-expansion",
        "second-attempt",
        "terminal-retry",
        "resume-after-terminal-failure",
    ],
)
def test_r5_validator_rejects_budget_or_retry_mutation(tmp_path, mutation):
    workspace = tmp_path / "workspace"
    preregistration = _copy_validation_workspace(workspace)
    _mutate_preregistration(preregistration, mutation)
    _assert_validation_rejects(workspace, preregistration)


@pytest.mark.parametrize("mutation_kind", ["hash", "value"])
def test_r5_validator_rejects_frozen_m030_mutation(tmp_path, mutation_kind):
    workspace = tmp_path / "workspace"
    preregistration = _copy_validation_workspace(workspace)
    prereg = yaml.safe_load(preregistration.read_text(encoding="utf-8"))
    bank_path = prereg["resources"]["candidate_bank"]["path"]

    def mutate(document):
        if mutation_kind == "hash":
            document["frozen_m030_input"]["supervisor"]["sha256"] = "0" * 64
        else:
            document["frozen_m030_input"][
                "maneuver_min_obstacle_dist_m"
            ] = 0.31

    _mutate_resource(
        workspace, preregistration, bank_path, mutate
    )
    _assert_validation_rejects(workspace, preregistration)


def test_r5_validator_rejects_coordinated_frozen_runtime_drift(tmp_path):
    workspace = tmp_path / "workspace"
    preregistration = _copy_validation_workspace(workspace)
    prereg = yaml.safe_load(preregistration.read_text(encoding="utf-8"))
    supervisor_path = prereg["resources"]["frozen_m030_supervisor"]["path"]
    bank_path = prereg["resources"]["candidate_bank"]["path"]

    def mutate_supervisor(document):
        document["profile_id"] = "coordinated-drift-that-must-not-refreeze"

    _mutate_resource(
        workspace, preregistration, supervisor_path, mutate_supervisor
    )
    changed_digest = _sha256(workspace / supervisor_path)

    def mutate_bank(document):
        document["frozen_m030_input"]["supervisor"]["sha256"] = changed_digest

    _mutate_resource(workspace, preregistration, bank_path, mutate_bank)
    _assert_validation_rejects(workspace, preregistration)


def test_r5_validator_rejects_candidate_with_a_second_behavioral_diff(tmp_path):
    workspace = tmp_path / "workspace"
    preregistration = _copy_validation_workspace(workspace)
    prereg = yaml.safe_load(preregistration.read_text(encoding="utf-8"))
    bank_path = prereg["resources"]["candidate_bank"]["path"]

    def mutate(document):
        document["candidates"][1]["overlay_release_confirmation_s"] = 0.35

    _mutate_resource(
        workspace, preregistration, bank_path, mutate
    )
    _assert_validation_rejects(workspace, preregistration)


def test_r5_validator_rejects_execution_authorization_flip(tmp_path):
    workspace = tmp_path / "workspace"
    preregistration = _copy_validation_workspace(workspace)

    def mutate(document):
        document["current_design_authorization"][
            "bounded_gazebo_calibration_execution"
        ] = True

    _mutate_preregistration(preregistration, mutate)
    _assert_validation_rejects(workspace, preregistration)


@pytest.mark.parametrize(
    "resource_name",
    ["dry_run_validator", "dry_run_validator_tests"],
)
def test_r5_validator_requires_its_audit_resource_closure(
    tmp_path, resource_name
):
    workspace = tmp_path / "workspace"
    preregistration = _copy_validation_workspace(workspace)

    def mutate(document):
        document["resources"].pop(resource_name)

    _mutate_preregistration(preregistration, mutate)
    _assert_validation_rejects(workspace, preregistration)


def test_r5_validator_rejects_navigation_scene_distribution_drift(tmp_path):
    workspace = tmp_path / "workspace"
    preregistration = _copy_validation_workspace(workspace)

    def mutate(document):
        document["scenes"][0]["family"] = "DYNAMIC"

    _mutate_resource(
        workspace,
        preregistration,
        NAVIGATION_MANIFEST,
        mutate,
    )
    _assert_validation_rejects(workspace, preregistration)


def test_r5_validator_rejects_frozen_scene_geometry_drift(tmp_path):
    workspace = tmp_path / "workspace"
    preregistration = _copy_validation_workspace(workspace)

    def mutate(document):
        document["scenes"][0]["static_obstacles"][0]["pose"]["x_m"] += 0.1

    _mutate_resource(
        workspace,
        preregistration,
        NAVIGATION_MANIFEST,
        mutate,
    )
    _assert_validation_rejects(workspace, preregistration)


def test_r5_validator_rejects_compile_support_scene_schedule_leak(tmp_path):
    workspace = tmp_path / "workspace"
    preregistration = _copy_validation_workspace(workspace)

    def mutate(document):
        support_scene = document["readiness_compile_support_boundary"][
            "scene_ids"
        ][0]
        row = document["ttc_activation_coverage_readiness"]["schedule"][0]
        row.update(
            {
                "identity": "r5-readiness-r5_ttc_h450-s5114",
                "scene_id": support_scene,
                "seed": 5114,
                "expected_status": "NO_CONFLICT_IN_HORIZON",
            }
        )

    _mutate_preregistration(preregistration, mutate)
    _assert_validation_rejects(workspace, preregistration)
