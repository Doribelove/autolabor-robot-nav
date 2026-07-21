import copy
import hashlib
import importlib.util
import math
from pathlib import Path

import pytest
import yaml


WORKSPACE = Path(__file__).resolve().parents[4]
SCRIPT = (
    WORKSPACE
    / "src/tools/thesis_experiment/scripts/review_v2_04g_r6.py"
)
CONTRACT = (
    WORKSPACE
    / "config/thesis_experiments/v2/"
    "v2_04g_r6_semantic_alignment_design_contract.yaml"
)
PREREGISTRATION = (
    WORKSPACE
    / "experiments/manifests/v2/preregistrations/"
    "v2_04g_r6_semantic_alignment_preregistration.yaml"
)
REPORT = (
    WORKSPACE
    / "artifacts/v2/design_review/v2_04g_r6/"
    "v2_04g_r6_design_review.yaml"
)


def _module():
    spec = importlib.util.spec_from_file_location(
        "review_v2_04g_r6_test", SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _identity(module):
    return {
        "stage": module.STAGE,
        "profile_id": "r6_semantics_circle_contact",
        "scene_id": "v2-04g-r5-readiness-dynamic-conflict-s5111",
        "seed": 0,
        "attempt": 1,
    }


def _materialized_scene(module, integrity, target):
    d1_contract = module._load_yaml(
        WORKSPACE
        / "config/thesis_experiments/v2/"
        "v2_04g_ttc_d1_offline_diagnosis_contract.yaml"
    )
    index = d1_contract["frozen_inputs"]["readiness_compiled_index"]
    lease = integrity.acquire_compiled_scene_lease(
        WORKSPACE,
        index["path"],
        index["sha256"],
        "v2-04g-r5-readiness-dynamic-conflict-s5111",
    )
    return integrity.materialize_scene_snapshot(lease, target)


def _success_resources(root, identity, startup_profile):
    artifact = root / "artifacts/attempt"
    artifact.mkdir(parents=True)
    trace = b"stamp,x\n0.0,0.0\n"
    trace_hash = hashlib.sha256(trace).hexdigest()
    activation = dict(identity)
    activation.update({"tracker_message_count": 20, "context_message_count": 20})
    evaluation = dict(activation)
    evaluation["raw_trace_sha256"] = trace_hash
    clearance = dict(identity)
    clearance["contact_count"] = 0
    startup_hash = hashlib.sha256(startup_profile).hexdigest()
    teardown = dict(identity)
    teardown.update({
        "restore_requested_while_backend_alive": True,
        "transaction_acknowledged": True,
        "transaction_readback_match": True,
        "independent_readback_match": True,
        "startup_profile_sha256": startup_hash,
        "transaction_readback_sha256": startup_hash,
        "independent_readback_sha256": startup_hash,
    })
    payloads = {
        "activation": yaml.safe_dump(activation).encode("utf-8"),
        "evaluation": yaml.safe_dump(evaluation).encode("utf-8"),
        "trace": trace,
        "clearance": yaml.safe_dump(clearance).encode("utf-8"),
        "process_log": b"offline fixture\n",
        "teardown_receipt": yaml.safe_dump(teardown).encode("utf-8"),
    }
    resources = {}
    for label, payload in payloads.items():
        path = artifact / label
        path.write_bytes(payload)
        resources[label] = {
            "path": str(path.relative_to(root)),
            "sha256": hashlib.sha256(payload).hexdigest(),
        }
    return resources, teardown


def _not_produced_terminal_binding(
    root, identity, integrity, phase="attempt_started"
):
    artifact = root / "artifacts/terminal"
    artifact.mkdir(parents=True)
    resources = {
        label: {
            "status": "not_produced",
            "phase": phase,
            "reason": "synthetic_interrupt",
        }
        for label in integrity.RAW_EVIDENCE_LABELS
    }
    return integrity.bind_terminal_attempt_evidence(
        root, "artifacts/terminal", identity, resources
    )


def _produced_terminal_binding(
    root, identity, integrity, resources, artifact_root="artifacts/attempt"
):
    declarations = {
        label: {
            "status": "produced",
            "path": row["path"],
            "sha256": row["sha256"],
        }
        for label, row in resources.items()
    }
    return integrity.bind_terminal_attempt_evidence(
        root, artifact_root, identity, declarations
    )


def _review_closure_manifest(preregistration):
    resources = preregistration["resources"]
    contract = resources["contract"]["path"]
    semantic = resources["semantic_reference"]["path"]
    edges = []
    for label, row in resources.items():
        if row["path"] == contract:
            continue
        kind = "frozen_input"
        if label == "candidate_bank":
            kind = "candidate_specification"
        elif label == "semantic_reference":
            kind = "design_reference"
        elif label == "integrity_protocol":
            kind = "integrity_protocol"
        edges.append({"from": contract, "to": row["path"], "kind": kind})
    edges.extend([
        {
            "from": semantic,
            "to": resources["frozen_risk_evidence"]["path"],
            "kind": "python_import",
        },
        {
            "from": semantic,
            "to": resources["frozen_rule_supervisor"]["path"],
            "kind": "python_import",
        },
    ])
    return {
        "files": [dict(row) for row in resources.values()],
        "edges": edges,
        "entrypoints": [contract],
        "unresolved": [],
    }


def test_r6_contract_is_design_only_and_has_no_seed_or_execution_authority():
    module = _module()
    contract = module._load_yaml(CONTRACT)
    module._verify_contract(contract)

    assert contract["stage"] == "V2-04G-R6-DESIGN"
    assert contract["single_changed_factor"]["factor_count"] == 1
    assert contract["seed_and_budget_boundary"]["seed_values"] == []
    assert contract["seed_and_budget_boundary"]["evidence_budget_authorized"] == 0
    assert contract["authorization"]["create_execution_authorization"] is False
    assert contract["authorization"]["ros_or_gazebo_execution"] is False
    assert contract["authorization"]["seed_or_evidence_consumption"] is False


def test_r6_preregistration_has_exactly_one_atomic_categorical_factor():
    module = _module()
    preregistration = module._load_yaml(PREREGISTRATION)
    module._verify_preregistration(preregistration)

    factor = preregistration["single_changed_factor"]
    assert factor["name"] == "dynamic_conflict_estimator_semantics"
    assert factor["runtime_field"] == (
        "supervisor.dynamic.conflict_estimator_id"
    )
    assert factor["allowed_values"] == [
        "legacy_class_conditioned_geometry_v1",
        "shared_circle_envelope_first_contact_v1",
    ]
    assert factor["horizon_scan_included"] is False
    assert factor["horizon_values_1_5_or_1_0_included"] is False


def test_r6_in_memory_candidates_differ_only_by_estimator_selector():
    module = _module()
    control = module._in_memory_config(module.LEGACY_ESTIMATOR_ID)
    repair = copy.deepcopy(control)
    repair["dynamic"]["conflict_estimator_id"] = module.ALIGNED_ESTIMATOR_ID

    assert module._leaf_differences(control, repair) == [
        "dynamic.conflict_estimator_id"
    ]
    assert control["dynamic"]["predicted_ttc_max_s"] == 5.0
    assert repair["dynamic"]["predicted_ttc_max_s"] == 5.0
    assert control["dynamic"]["minimum_track_confidence"] == 0.45
    assert repair["dynamic"]["minimum_track_confidence"] == 0.45


def test_r6_aligned_semantics_match_frozen_evaluator_and_remove_false_crossing():
    module = _module()
    result = module._semantic_review(
        module._load_project_modules(WORKSPACE),
        module._load_yaml(WORKSPACE / module.CANDIDATE_BANK_RELATIVE),
    )
    fixtures = {row["fixture_id"]: row for row in result["fixtures"]}

    mismatch = fixtures[
        "legacy_centerline_crossing_without_circle_contact"
    ]
    assert mismatch["legacy_overlay"] == "CROSSING"
    assert mismatch["aligned_overlay"] == "NONE"
    assert mismatch["aligned_ttc_s"] is None
    assert result["finite_ttc_evaluator_parity_fixture_count"] == 2
    assert result["supervisor_candidate_wiring_fixture_count"] == 5
    assert result["multi_track_ordering_fixture_count"] == 3
    assert all(row["matches_frozen_evaluator"] for row in result["fixtures"])


def test_r6_footprint_radius_is_atomic_and_uses_evaluator_circumradius():
    module = _module()
    modules = module._load_project_modules(WORKSPACE)
    footprint = [
        (-0.275, -0.275),
        (-0.275, 0.275),
        (0.275, 0.275),
        (0.275, -0.275),
    ]

    assert modules["footprint_radius"](
        footprint, module.LEGACY_ESTIMATOR_ID
    ) == pytest.approx(0.275)
    assert modules["footprint_radius"](
        footprint, module.ALIGNED_ESTIMATOR_ID
    ) == pytest.approx(math.hypot(0.275, 0.275))
    assert modules["footprint_radius"](
        [], module.ALIGNED_ESTIMATOR_ID
    ) == pytest.approx(0.25)
    with pytest.raises(ValueError, match="footprint-bearing"):
        modules["runtime_track_from_footprint"](
            modules["RuntimeTrack"](
                1, "UNKNOWN", 1.0, 0.0, -1.0, 0.0, 0.25, 0.9
            ),
            module.ALIGNED_ESTIMATOR_ID,
        )


def test_r6_frozen_d1_input_is_identifiable_but_not_new_evidence():
    result = _module()._verify_d1_design_input(WORKSPACE)

    assert result["trace_row_count"] == 193
    assert result["legacy_proxy_non_none_count"] == 25
    assert result["legacy_proxy_crossing_count"] == 21
    assert result["legacy_proxy_overtake_or_yield_count"] == 4
    assert result["shared_circle_ttc_finite_count"] == 0
    assert result["expected_changed_rows_under_proposed_definition"] == 25
    assert result["evidence_units_consumed"] == 0
    assert result["seed5111_reconsumed"] is False


def test_r6_readiness_direct_count_protocol_rejects_aggregate_shortcuts():
    module = _module()
    integrity = module._load_project_modules(WORKSPACE)["integrity"]
    identity = {
        "stage": module.STAGE,
        "profile_id": "profile",
        "scene_id": "scene",
        "seed": 0,
        "attempt": 1,
    }
    activation = dict(identity)
    activation.update({"tracker_message_count": 20, "context_message_count": 20})

    result = integrity.validate_readiness_raw_evidence(
        identity, activation, dict(activation), 20
    )
    assert result["pass"] is True
    invalid = dict(activation)
    invalid["tracker_message_count"] = 19
    with pytest.raises(integrity.R6IntegrityError, match="below"):
        integrity.validate_readiness_raw_evidence(
            identity, invalid, dict(activation), 20
        )
    invalid = dict(activation)
    invalid["context_message_count"] = True
    with pytest.raises(integrity.R6IntegrityError, match="integer"):
        integrity.validate_readiness_raw_evidence(
            identity, invalid, dict(activation), 20
        )
    bool_identity = dict(activation)
    bool_identity["seed"] = False
    bool_identity["attempt"] = True
    with pytest.raises(integrity.R6IntegrityError, match="identity mismatch"):
        integrity.validate_readiness_raw_evidence(
            identity, bool_identity, dict(activation), 20
        )


def test_r6_compiled_scene_protocol_binds_children_and_rejects_snapshot_tamper(
    tmp_path,
):
    module = _module()
    integrity = module._load_project_modules(WORKSPACE)["integrity"]
    snapshot = _materialized_scene(
        module, integrity, tmp_path / "snapshot"
    )

    verification = integrity.revalidate_scene_snapshot(
        snapshot, "pre_spawn"
    )
    assert verification.as_document()["verification_phase"] == "pre_spawn"
    instance = Path(
        snapshot.as_document()["snapshot_instance"]["path"]
    )
    instance.chmod(0o600)
    instance.write_bytes(instance.read_bytes() + b"\n")
    with pytest.raises(integrity.R6IntegrityError, match="hash drifted"):
        integrity.revalidate_scene_snapshot(snapshot, "post_episode")
    with pytest.raises(integrity.R6IntegrityError, match="materialized"):
        integrity.revalidate_scene_snapshot(
            snapshot.as_document(), "post_episode"
        )


def test_r6_journal_terminalizes_sigint_and_seals_orphan_without_resume(tmp_path):
    module = _module()
    integrity = module._load_project_modules(WORKSPACE)["integrity"]
    identity = _identity(module)
    terminal_binding = _not_produced_terminal_binding(
        tmp_path, identity, integrity
    )
    interrupted_root = tmp_path / "journals_interrupted"

    with pytest.raises(KeyboardInterrupt):
        with integrity.AtomicAttemptJournal(
            interrupted_root, identity
        ) as journal:
            interrupted_path = journal.path
            journal.attach_terminal_evidence(terminal_binding)
            raise KeyboardInterrupt()
    interrupted = integrity.strict_yaml(interrupted_path)
    assert interrupted["status"] == "terminal_interrupted"
    assert interrupted["resume_forbidden"] is True
    assert interrupted["evidence_binding"][
        "terminal_raw_evidence_declared"
    ] is True

    orphan_root = tmp_path / "journals_orphan"
    orphan_path = integrity.canonical_attempt_state_path(
        orphan_root, identity
    )
    orphan_path.parent.mkdir(parents=True)
    orphan_path.write_text(
        yaml.safe_dump({
            "stage": module.STAGE,
            "identity": identity,
            "status": "attempt_started",
            "lifecycle_phase": "attempt_started",
            "active_identity": identity,
        }),
        encoding="utf-8",
    )
    sealed = integrity.seal_orphaned_attempt(
        orphan_root, identity, terminal_binding
    )
    assert sealed["status"] == "terminal_unclean_shutdown"
    assert sealed["active_identity"] is None
    assert sealed["resume_forbidden"] is True
    assert sealed["evidence_binding"][
        "terminal_raw_evidence_declared"
    ] is True
    with pytest.raises(integrity.R6IntegrityError, match="resume forbidden"):
        with integrity.AtomicAttemptJournal(interrupted_root, identity):
            pass


def test_r6_journal_rejects_concurrent_same_identity_and_early_completion(
    tmp_path,
):
    module = _module()
    integrity = module._load_project_modules(WORKSPACE)["integrity"]
    identity = _identity(module)
    terminal = _not_produced_terminal_binding(
        tmp_path, identity, integrity
    )
    journal_root = tmp_path / "journals_active"

    with integrity.AtomicAttemptJournal(journal_root, identity) as journal:
        journal_path = journal.path
        journal.attach_terminal_evidence(terminal)
        assert (
            integrity.AtomicAttemptJournal(journal_root, identity).path
            == journal_path
        )
        with pytest.raises(integrity.R6IntegrityError, match="before execution"):
            journal.capture_startup_profile(b"stale terminal bundle\n")
        assert integrity.strict_yaml(journal_path)["active_identity"] == identity
        with pytest.raises(integrity.R6IntegrityError, match="already active"):
            with integrity.AtomicAttemptJournal(journal_root, identity):
                pass
        with pytest.raises(
            integrity.R6IntegrityError, match="terminal failure evidence"
        ):
            journal.complete({"unvalidated": True})
    assert integrity.strict_yaml(journal_path)["status"] == "terminal_incomplete"

    with integrity.AtomicAttemptJournal(
        tmp_path / "journals_phase_mismatch", identity
    ) as phase_journal:
        phase_journal.capture_startup_profile(b"startup\n")
        with pytest.raises(integrity.R6IntegrityError, match="phase"):
            phase_journal.attach_terminal_evidence(terminal)


def test_r6_terminal_bundle_requires_explicit_six_item_inventory(tmp_path):
    module = _module()
    integrity = module._load_project_modules(WORKSPACE)["integrity"]
    identity = _identity(module)
    artifact = tmp_path / "artifacts/terminal"
    artifact.mkdir(parents=True)
    resources = {
        label: {
            "status": "not_produced",
            "phase": "attempt_started",
            "reason": "synthetic_interrupt",
        }
        for label in integrity.RAW_EVIDENCE_LABELS
    }
    missing = dict(resources)
    missing.pop("evaluation")
    with pytest.raises(integrity.R6IntegrityError, match="resource set"):
        integrity.bind_terminal_attempt_evidence(
            tmp_path, "artifacts/terminal", identity, missing
        )
    (artifact / "undeclared.log").write_text("drift", encoding="utf-8")
    with pytest.raises(integrity.R6IntegrityError, match="undeclared"):
        integrity.bind_terminal_attempt_evidence(
            tmp_path, "artifacts/terminal", identity, resources
        )
    (artifact / "undeclared.log").unlink()
    invalid_phase = copy.deepcopy(resources)
    invalid_phase["activation"]["phase"] = "utterly_impossible_phase"
    with pytest.raises(integrity.R6IntegrityError, match="not-produced"):
        integrity.bind_terminal_attempt_evidence(
            tmp_path, "artifacts/terminal", identity, invalid_phase
        )
    post_episode_not_produced = copy.deepcopy(resources)
    for row in post_episode_not_produced.values():
        row["phase"] = "post_episode_scene_verified"
    with pytest.raises(integrity.R6IntegrityError, match="post-episode"):
        integrity.bind_terminal_attempt_evidence(
            tmp_path,
            "artifacts/terminal",
            identity,
            post_episode_not_produced,
        )


def test_r6_raw_evidence_binding_requires_all_six_direct_resources(tmp_path):
    module = _module()
    integrity = module._load_project_modules(WORKSPACE)["integrity"]
    identity = _identity(module)
    startup_profile = b"startup profile fixture\n"
    snapshot = _materialized_scene(
        module, integrity, tmp_path / "snapshot"
    )
    resources, _ = _success_resources(
        tmp_path, identity, startup_profile
    )
    journal_root = tmp_path / "journals_complete"
    with integrity.AtomicAttemptJournal(journal_root, identity) as journal:
        journal_path = journal.path
        startup = journal.capture_startup_profile(startup_profile)
        journal.bind_scene_snapshot(snapshot)
        journal.mark_execution_started()
        post = journal.verify_post_episode_scene()
        bound = integrity.bind_attempt_raw_evidence(
            tmp_path,
            "artifacts/attempt",
            identity,
            resources,
            20,
            startup,
            post,
        )
        document = bound.as_document()
        assert document["raw_evidence_bound"] is True
        assert set(document["resources"]) == set(
            integrity.RAW_EVIDENCE_LABELS
        )
        with pytest.raises(
            integrity.R6IntegrityError, match="validated evidence"
        ):
            journal.complete({"raw_evidence_bound": True})
        with pytest.raises(
            integrity.R6IntegrityError, match="launch-stop"
        ):
            journal.complete(bound)
        stop_gate = journal.authorize_launch_stop(
            bound.verified_teardown
        )
        assert stop_gate["identity"] == identity
        journal.complete(bound)
        with pytest.raises(
            integrity.R6IntegrityError, match="already complete"
        ):
            journal.complete(bound)
    assert integrity.strict_yaml(journal_path)["status"] == "evidence_complete"

    missing = dict(resources)
    missing.pop("trace")
    with pytest.raises(integrity.R6IntegrityError, match="resource set"):
        integrity.bind_attempt_raw_evidence(
            tmp_path,
            "artifacts/attempt",
            identity,
            missing,
            20,
            startup,
            post,
        )
    alias = copy.deepcopy(resources)
    alias["process_log"] = dict(alias["trace"])
    with pytest.raises(integrity.R6IntegrityError, match="alias"):
        integrity.bind_attempt_raw_evidence(
            tmp_path,
            "artifacts/attempt",
            identity,
            alias,
            20,
            startup,
            post,
        )


def test_r6_dependency_closure_rejects_missing_transitive_resource():
    module = _module()
    integrity = module._load_project_modules(WORKSPACE)["integrity"]
    preregistration = module._load_yaml(PREREGISTRATION)
    paths = [
        row["path"] for row in preregistration["resources"].values()
    ]
    manifest = _review_closure_manifest(preregistration)

    result = integrity.verify_dependency_closure(
        WORKSPACE, manifest, paths
    )
    assert result["file_count"] == len(paths)
    assert result["all_hashes_match"] is True
    manifest["files"] = manifest["files"][:-1]
    with pytest.raises(integrity.R6IntegrityError):
        integrity.verify_dependency_closure(WORKSPACE, manifest, paths)
    unreachable = _review_closure_manifest(preregistration)
    leaf = preregistration["resources"]["d1_handoff"]["path"]
    unreachable["edges"] = [
        edge for edge in unreachable["edges"] if edge["to"] != leaf
    ]
    with pytest.raises(integrity.R6IntegrityError, match="unreachable"):
        integrity.verify_dependency_closure(
            WORKSPACE, unreachable, paths
        )


def test_r6_teardown_protocol_requires_independent_startup_profile_readback(
    tmp_path,
):
    module = _module()
    integrity = module._load_project_modules(WORKSPACE)["integrity"]
    identity = _identity(module)
    startup_profile = b"startup profile fixture\n"
    expected = hashlib.sha256(startup_profile).hexdigest()
    receipt = dict(identity)
    receipt.update({
        "restore_requested_while_backend_alive": True,
        "transaction_acknowledged": True,
        "transaction_readback_match": True,
        "independent_readback_match": True,
        "startup_profile_sha256": expected,
        "transaction_readback_sha256": expected,
        "independent_readback_sha256": expected,
    })
    snapshot = _materialized_scene(
        module, integrity, tmp_path / "snapshot"
    )
    resources, _ = _success_resources(
        tmp_path, identity, startup_profile
    )
    invalid = dict(receipt)
    invalid["independent_readback_match"] = False
    invalid_payload = yaml.safe_dump(invalid).encode("utf-8")
    invalid_path = tmp_path / resources["teardown_receipt"]["path"]
    invalid_path.write_bytes(invalid_payload)
    resources["teardown_receipt"]["sha256"] = hashlib.sha256(
        invalid_payload
    ).hexdigest()
    terminal = _produced_terminal_binding(
        tmp_path, identity, integrity, resources
    )
    journal_root = tmp_path / "journals_teardown"
    with pytest.raises(
        integrity.R6TeardownFailure, match="independent"
    ):
        with integrity.AtomicAttemptJournal(
            journal_root, identity
        ) as journal:
            journal_path = journal.path
            startup = journal.capture_startup_profile(startup_profile)
            journal.bind_scene_snapshot(snapshot)
            journal.mark_execution_started()
            post = journal.verify_post_episode_scene()
            journal.attach_terminal_evidence(terminal)
            integrity.verify_teardown_restore(
                invalid, startup, post, identity
            )
    assert integrity.strict_yaml(journal_path)["status"] == (
        "terminal_teardown_failure"
    )


def test_r6_teardown_token_cannot_cross_attempt_or_inactive_journal(tmp_path):
    module = _module()
    integrity = module._load_project_modules(WORKSPACE)["integrity"]
    identity_a = _identity(module)
    identity_b = dict(identity_a)
    identity_b["attempt"] = 2
    startup_profile = b"startup profile fixture\n"
    resources_a, receipt_a = _success_resources(
        tmp_path / "a", identity_a, startup_profile
    )
    terminal_a = _produced_terminal_binding(
        tmp_path / "a", identity_a, integrity, resources_a
    )
    snapshot_a = _materialized_scene(
        module, integrity, tmp_path / "snapshot_a"
    )
    root_a = tmp_path / "journals_a"
    with integrity.AtomicAttemptJournal(root_a, identity_a) as journal_a:
        startup_a = journal_a.capture_startup_profile(startup_profile)
        journal_a.bind_scene_snapshot(snapshot_a)
        journal_a.mark_execution_started()
        post_a = journal_a.verify_post_episode_scene()
        journal_a.attach_terminal_evidence(terminal_a)
        token_a = integrity.verify_teardown_restore(
            receipt_a, startup_a, post_a, identity_a
        )
        bool_identity_receipt = dict(receipt_a)
        bool_identity_receipt["seed"] = False
        bool_identity_receipt["attempt"] = True
        with pytest.raises(
            integrity.R6TeardownFailure, match="identity mismatch"
        ):
            integrity.verify_teardown_restore(
                bool_identity_receipt,
                startup_a,
                post_a,
                identity_a,
            )
        assert journal_a.authorize_launch_stop(token_a)[
            "launch_stop_allowed"
        ] is True

    with pytest.raises(integrity.R6IntegrityError, match="active post-episode"):
        integrity.authorize_launch_stop(token_a, identity_a, journal_a)

    snapshot_b = _materialized_scene(
        module, integrity, tmp_path / "snapshot_b"
    )
    resources_b, _ = _success_resources(
        tmp_path / "b", identity_b, startup_profile
    )
    terminal_b = _produced_terminal_binding(
        tmp_path / "b", identity_b, integrity, resources_b
    )
    with integrity.AtomicAttemptJournal(
        tmp_path / "journals_b", identity_b
    ) as journal_b:
        journal_b.capture_startup_profile(startup_profile)
        journal_b.bind_scene_snapshot(snapshot_b)
        journal_b.mark_execution_started()
        journal_b.verify_post_episode_scene()
        journal_b.attach_terminal_evidence(terminal_b)
        with pytest.raises(integrity.R6IntegrityError, match="identity"):
            integrity.authorize_launch_stop(
                token_a, identity_b, journal_b
            )

    snapshot_c = _materialized_scene(
        module, integrity, tmp_path / "snapshot_c"
    )
    resources_c, _ = _success_resources(
        tmp_path / "c", identity_a, startup_profile
    )
    terminal_c = _produced_terminal_binding(
        tmp_path / "c", identity_a, integrity, resources_c
    )
    with integrity.AtomicAttemptJournal(
        tmp_path / "journals_c", identity_a
    ) as journal_c:
        journal_c.capture_startup_profile(startup_profile)
        journal_c.bind_scene_snapshot(snapshot_c)
        journal_c.mark_execution_started()
        journal_c.verify_post_episode_scene()
        journal_c.attach_terminal_evidence(terminal_c)
        with pytest.raises(
            integrity.R6IntegrityError, match="active attempt state"
        ):
            integrity.authorize_launch_stop(
                token_a, identity_a, journal_c
            )


def test_r6_machine_report_records_all_repairs_without_execution_claim():
    report = _module().build_report(WORKSPACE, CONTRACT)
    repairs = report["integrity_repair_review"]

    assert repairs["required_count"] == 6
    assert repairs["implemented_and_unit_verified_count"] == 6
    assert repairs["execution_validation_status"] == "NOT_RUN_NOT_AUTHORIZED"
    assert tuple(row["risk_id"] for row in repairs["repairs"]) == (
        _module().EXPECTED_RISK_IDS
    )
    assert all(
        row["design_fix_status"]
        == "OFFLINE_PROTOCOL_IMPLEMENTED_AND_UNIT_VERIFIED"
        for row in repairs["repairs"]
    )
    assert all(
        row["execution_validation_status"] == "NOT_RUN_NOT_AUTHORIZED"
        for row in repairs["repairs"]
    )


def test_r6_report_is_deterministic_and_matches_persisted_report():
    module = _module()
    first = module.build_report(WORKSPACE, CONTRACT)
    second = module.build_report(WORKSPACE, CONTRACT)
    persisted = yaml.safe_load(REPORT.read_text(encoding="utf-8"))

    assert first == second
    assert persisted == first
    assert first["review_result"] == "pass"
    assert first["execution_authorized"] is False
    assert first["side_effects"]["seeds_consumed"] == 0


def test_r6_rejects_duplicate_yaml_and_resource_hash_drift(tmp_path):
    module = _module()
    duplicate = tmp_path / "duplicate.yaml"
    duplicate.write_text("stage: R6\nstage: drift\n", encoding="utf-8")
    with pytest.raises(module.R6ReviewError, match="duplicate YAML key"):
        module._load_yaml(duplicate)

    preregistration = module._load_yaml(PREREGISTRATION)
    preregistration["resources"]["d1_report"]["sha256"] = "0" * 64
    with pytest.raises(module.R6ReviewError, match="d1_report"):
        module._verify_resources(WORKSPACE, preregistration)


def test_r6_verifies_every_d1_declared_dependency_and_rejects_drift():
    module = _module()
    d1_contract = module._load_yaml(
        WORKSPACE
        / "config/thesis_experiments/v2/"
        "v2_04g_ttc_d1_offline_diagnosis_contract.yaml"
    )
    closure = module._verify_d1_declared_closure(
        WORKSPACE, d1_contract
    )
    assert closure["total_declaration_count"] == 38
    assert closure["unique_file_count"] == 37
    assert closure["all_declared_hashes_match_single_open_reads"] is True

    drifted = copy.deepcopy(d1_contract)
    drifted["frozen_inputs"]["r5_contract"]["sha256"] = "0" * 64
    with pytest.raises(module.R6ReviewError, match="hash drifted"):
        module._verify_d1_declared_closure(WORKSPACE, drifted)

    conflicting = copy.deepcopy(d1_contract)
    source = conflicting["frozen_inputs"]["r5_contract"]
    conflicting["execution_dependency_chain"].append({
        "path": source["path"],
        "sha256": "0" * 64,
    })
    with pytest.raises(module.R6ReviewError, match="conflicting"):
        module._verify_d1_declared_closure(WORKSPACE, conflicting)


def test_r6_closed_safety_schemas_reject_added_authority_or_second_behavior():
    module = _module()
    contract = module._load_yaml(CONTRACT)
    contract["execution_authorized"] = True
    with pytest.raises(module.R6ReviewError, match="top-level"):
        module._verify_contract(contract)

    contract = module._load_yaml(CONTRACT)
    contract["authorization"]["execute_r6"] = True
    with pytest.raises(module.R6ReviewError, match="authorization"):
        module._verify_contract(contract)

    preregistration = module._load_yaml(PREREGISTRATION)
    preregistration["seed_and_budget_boundary"]["seed_schedule"] = [6001]
    with pytest.raises(module.R6ReviewError, match="seed and budget"):
        module._verify_preregistration(preregistration)

    bank = module._load_yaml(WORKSPACE / module.CANDIDATE_BANK_RELATIVE)
    bank["candidates"][1]["horizon_s"] = 1.5
    with pytest.raises(module.R6ReviewError, match="candidate declarations"):
        module._verify_candidate_bank(bank)


def test_r6_supervisor_rejects_numeric_drift_and_copies_external_config():
    module = _module()
    modules = module._load_project_modules(WORKSPACE)
    config = module._in_memory_config(module.ALIGNED_ESTIMATOR_ID)
    supervisor = modules["R6RelativeTTCSupervisor"](config)
    config["dynamic"]["predicted_ttc_max_s"] = 1.0
    assert supervisor.horizon_s == 5.0

    drift = module._in_memory_config(module.ALIGNED_ESTIMATOR_ID)
    drift["dynamic"]["predicted_ttc_max_s"] = 1.5
    with pytest.raises(ValueError, match="frozen at 5.0"):
        modules["R6RelativeTTCSupervisor"](drift)


def test_r6_closed_artifact_allowlist_rejects_oddly_named_authorization(
    tmp_path,
):
    module = _module()
    authorization = tmp_path / "rogue_execution_authorization.md"
    authorization.write_text(
        yaml.safe_dump({
            "stage": "V2-04G-R6-DESIGN",
            "execution_authorized": True,
        }),
        encoding="utf-8",
    )
    with pytest.raises(module.R6ReviewError, match="allowlist"):
        module._forbidden_r6_artifacts(tmp_path)


def test_r6_rejects_noncanonical_report_output_without_writing():
    module = _module()
    forbidden = REPORT.parent / "noncanonical-output.yaml"
    assert not forbidden.exists()

    with pytest.raises(module.R6ReviewError, match="output path drifted"):
        module.review(WORKSPACE, CONTRACT, forbidden)
    assert not forbidden.exists()


def test_r6_review_preserves_frozen_r5_and_d1_trees():
    module = _module()
    r5_before = module._tree_snapshot(
        WORKSPACE, module.R5_ARTIFACT_RELATIVE
    )
    d1_before = module._tree_snapshot(
        WORKSPACE, module.D1_ARTIFACT_RELATIVE
    )
    module.build_report(WORKSPACE, CONTRACT)
    r5_after = module._tree_snapshot(
        WORKSPACE, module.R5_ARTIFACT_RELATIVE
    )
    d1_after = module._tree_snapshot(
        WORKSPACE, module.D1_ARTIFACT_RELATIVE
    )

    assert r5_before == r5_after
    assert d1_before == d1_after
    assert r5_before["file_count"] == 68
    assert r5_before["tree_sha256"] == module.EXPECTED_R5_TREE_SHA256
