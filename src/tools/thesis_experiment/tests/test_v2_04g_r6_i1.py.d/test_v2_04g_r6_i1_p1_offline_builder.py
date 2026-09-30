import importlib.util
from pathlib import Path

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
BUILDER_SOURCE = (
    WORKSPACE / "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p1_r1_offline_builder.py"
)
OUTPUT = (
    WORKSPACE / "artifacts/v2/integration/v2_04g_r6_i1/"
    "p1_r1_offline_readiness"
)


def _module():
    spec = importlib.util.spec_from_file_location("p1_builder_under_test", BUILDER_SOURCE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_bundle_rebuild_is_byte_deterministic_and_persisted():
    builder = _module()
    first = builder.build_bundle(WORKSPACE)
    second = builder.build_bundle(WORKSPACE)
    assert first == second
    assert len(first) == 194
    for relative, payload in first.items():
        assert (WORKSPACE / relative).read_bytes() == payload


def test_scene_bundle_is_94_scenes_188_children_and_exact_seed_sets():
    manifest = yaml.safe_load((OUTPUT / "v2_04g_p1_r1_scenes.yaml").read_text())
    index = yaml.safe_load((OUTPUT / "compiled_scenes/compiled_scene_index.yaml").read_text())
    assert len(manifest["scenes"]) == 94
    assert len(index["files"]) == 188
    assert {scene["seed"] for scene in manifest["scenes"]} == set(range(5201, 5295))
    evidence = [scene for scene in manifest["scenes"] if 5201 <= scene["seed"] <= 5290]
    assert len(evidence) == 90
    assert {scene["scene_id"] for scene in evidence} == {
        "v2-04g-p1-s{}".format(seed) for seed in range(5201, 5291)
    }
    assert all(scene["split"] == "test_id" for scene in evidence)
    assert all(scene["randomization"] == {
        "position_jitter_m": 0.0,
        "yaw_jitter_rad": 0.0,
        "agent_time_jitter_s": 0.0,
    } for scene in evidence)


def test_schedule_has_exact_balance_and_support_seeds_are_excluded():
    schedule = yaml.safe_load((OUTPUT / "v2_04g_p1_r1_exact_schedule.yaml").read_text())
    assert schedule["episode_count"] == 270
    assert schedule["block_count"] == 90
    assert {row["seed"] for row in schedule["episodes"]} == set(range(5201, 5291))
    assert not ({5291, 5292, 5293, 5294} & {row["seed"] for row in schedule["episodes"]})
    assert [row["sequence"] for row in schedule["episodes"]] == list(range(1, 271))
    for block_id in {row["block_id"] for row in schedule["episodes"]}:
        rows = [row for row in schedule["episodes"] if row["block_id"] == block_id]
        assert len(rows) == 3
        assert max(row["sequence"] for row in rows) - min(row["sequence"] for row in rows) == 2
        assert {row["method_id"] for row in rows} == set(schedule["methods"])
        assert len({row["instance"]["sha256"] for row in rows}) == 1
        assert len({row["world"]["sha256"] for row in rows}) == 1


def test_stage_remains_zero_authority_and_no_execution_state_exists():
    assessment = yaml.safe_load((OUTPUT / "v2_04g_p1_r1_assessment_not_run.yaml").read_text())
    resource = yaml.safe_load((OUTPUT / "v2_04g_p1_r1_resource_time_budget.yaml").read_text())
    assert assessment["execution_authorized"] is False
    assert assessment["evidence_budget_authorized"] == 0
    assert assessment["evidence_units_consumed"] == 0
    assert resource["authorized_resource_budget"] == 0
    for forbidden in (
        OUTPUT / "execution_release.yaml",
        OUTPUT / "authorization.yaml",
        OUTPUT / "attempts",
        OUTPUT / "journals",
    ):
        assert not forbidden.exists()

