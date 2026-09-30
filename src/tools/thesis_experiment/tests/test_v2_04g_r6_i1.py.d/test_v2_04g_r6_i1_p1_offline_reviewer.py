import importlib.util
from pathlib import Path

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
SOURCE = (
    WORKSPACE / "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p1_r1_offline_reviewer.py"
)
REVIEW = (
    WORKSPACE / "artifacts/v2/integration/v2_04g_r6_i1/"
    "p1_r1_offline_readiness/v2_04g_p1_r1_offline_review.yaml"
)


def _module():
    spec = importlib.util.spec_from_file_location("p1_reviewer_under_test", SOURCE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_persisted_independent_review_is_exactly_reproducible():
    reviewer = _module()
    rebuilt = reviewer.build_review(WORKSPACE)
    persisted = yaml.safe_load(REVIEW.read_text(encoding="utf-8"))
    assert rebuilt == persisted
    assert rebuilt["review_result"] == "pass"
    assert rebuilt["offline_review_pass"] is True
    assert rebuilt["execution_ready"] is False
    assert rebuilt["execution_authorized"] is False
    assert rebuilt["authorized_evidence_units"] == 0
    assert rebuilt["consumed_evidence_units"] == 0


def test_review_separates_offline_pass_from_future_execution_authority():
    review = yaml.safe_load(REVIEW.read_text(encoding="utf-8"))
    assert review["execution_gap"] == {
        "p1_exact_runner_adapter_present": False,
        "p1_raw_replay_adapter_present": False,
        "requires_separate_future_implementation_release_and_review": True,
        "current_offline_pass_is_execution_authorization": False,
    }
    assert review["claim_state"]["assessment_run"] is False
    assert review["claim_state"]["performance_improvement_demonstrated"] is False
    assert review["claim_state"]["future_positive_claim_requires_all_four_gates"] is True

