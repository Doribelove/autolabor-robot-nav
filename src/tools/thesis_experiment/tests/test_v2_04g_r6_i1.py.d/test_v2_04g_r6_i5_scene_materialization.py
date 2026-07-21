"""Offline-only regression tests for the versioned R6-I5 scene boundary."""

from __future__ import annotations

import copy
import importlib.util
from pathlib import Path
import tempfile

import pytest


WORKSPACE = Path(__file__).resolve().parents[5]
MATERIALIZER_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_scene_materializer.py"
)
SPEC = importlib.util.spec_from_file_location(
    "r6_i5_scene_materializer_test_target", MATERIALIZER_PATH
)
assert SPEC is not None and SPEC.loader is not None
materializer = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(materializer)


def test_canonical_derivation_is_exactly_bound_and_scene_bundle_passes() -> None:
    assert materializer.EXPECTED_DERIVATION_SHA256 == (
        "b74f24e169f3ffbe98f0139fc01dd78c1d2a1f8d6040df130719680ce4350145"
    )
    if (WORKSPACE / materializer.TARGET_ROOT).exists():
        receipt = materializer.review_materialization(
            WORKSPACE, materializer.EXPECTED_DERIVATION_SHA256
        )
        assert receipt["pass"] is True
        assert receipt["compiled_child_count"] == 14
    else:
        bundle = materializer.prepare_bundle(
            WORKSPACE,
            materializer.EXPECTED_DERIVATION_SHA256,
            require_target_absent=True,
        )
        assert len(bundle["outputs"]) == 17
        assert len(bundle["compiled_execution_rows"]) == 3


def test_exact_type_sensitive_diff_is_only_fifteen_identity_paths() -> None:
    source = materializer._snapshot(
        WORKSPACE, materializer.SOURCE_MANIFEST, parse_yaml=True
    )["document"]
    target = materializer._derive_manifest(source)
    assert tuple(materializer._diff_paths(source, target)) == (
        materializer.EXPECTED_CHANGED_PATHS
    )
    target["scenes"][0]["timeout_s"] += 1.0
    assert "scenes[0].timeout_s" in materializer._diff_paths(source, target)


def test_scene_patch_is_forbidden_at_every_depth() -> None:
    document = materializer._snapshot(
        WORKSPACE, materializer.DERIVATION, parse_yaml=True
    )["document"]
    mutated = copy.deepcopy(document)
    mutated["scene_derivations"][0]["scene_patch"] = {"timeout_s": 1.0}
    with pytest.raises(
        materializer.R6I5SceneMaterializationError,
        match="scene_patch is forbidden",
    ):
        materializer._validate_derivation(mutated)


def test_single_open_loader_rejects_duplicate_keys_and_symlinks() -> None:
    with tempfile.TemporaryDirectory(prefix="r6_i5_scene_test_") as temporary:
        root = Path(temporary)
        (root / "duplicate.yaml").write_text("stage: I5\nstage: drift\n", encoding="utf-8")
        with pytest.raises(
            materializer.R6I5SceneMaterializationError,
            match="duplicate YAML key",
        ):
            materializer._snapshot(root, Path("duplicate.yaml"), parse_yaml=True)

        (root / "regular.yaml").write_text("stage: I5\n", encoding="utf-8")
        (root / "linked.yaml").symlink_to("regular.yaml")
        with pytest.raises(
            materializer.R6I5SceneMaterializationError,
            match="single-open/no-follow",
        ):
            materializer._snapshot(root, Path("linked.yaml"), parse_yaml=True)


def test_wrong_caller_hash_fails_before_materialization() -> None:
    with pytest.raises(
        materializer.R6I5SceneMaterializationError,
        match="caller derivation hash differs",
    ):
        materializer.prepare_bundle(
            WORKSPACE,
            "0" * 64,
            require_target_absent=not (WORKSPACE / materializer.TARGET_ROOT).exists(),
        )


def test_fresh_firewall_allows_only_exact_canonical_readiness_outputs() -> None:
    expected = frozenset(
        {
            materializer.TARGET_ROOT / "execution_dependency_closure.yaml",
            materializer.TARGET_ROOT
            / "v2_04g_r6_i5_execution_readiness_review.yaml",
        }
    )
    assert materializer.CANONICAL_READINESS_FIREWALL_ALLOWLIST == expected

    with tempfile.TemporaryDirectory(prefix="r6_i5_firewall_test_") as temporary:
        root = Path(temporary)
        for relative in expected:
            absolute = root / relative
            absolute.parent.mkdir(parents=True, exist_ok=True)
            absolute.write_text(
                "stage: V2-04G-R6-I5\nseed: 5161\n",
                encoding="utf-8",
            )

        receipt = materializer._verify_fresh_firewall(
            root, target_allowed=True
        )
        assert receipt["pass"] is True
        assert receipt["structured_yaml_files_scanned"] == 0
        assert receipt["authorized_identity_documents"] == []

        extra = materializer.TARGET_ROOT / "unexpected_readiness.yaml"
        (root / extra).write_text(
            "stage: V2-04G-R6-I5\nseed: 5161\n",
            encoding="utf-8",
        )
        with pytest.raises(
            materializer.R6I5SceneMaterializationError,
            match="fresh identity leaked into unauthorized YAML",
        ):
            materializer._verify_fresh_firewall(root, target_allowed=True)


def test_fresh_firewall_still_rejects_execution_release() -> None:
    with tempfile.TemporaryDirectory(prefix="r6_i5_release_firewall_test_") as temporary:
        root = Path(temporary)
        release = root / materializer.RELEASE
        release.parent.mkdir(parents=True, exist_ok=True)
        release.write_text("stage: V2-04G-R6-I5\n", encoding="utf-8")
        with pytest.raises(
            materializer.R6I5SceneMaterializationError,
            match="execution release must remain absent",
        ):
            materializer._verify_fresh_firewall(root, target_allowed=True)
