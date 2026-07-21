import copy
import importlib.util
from pathlib import Path

import pytest
import yaml


WORKSPACE = Path(__file__).resolve().parents[5]
REVIEWER = WORKSPACE / (
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_r6_i6_"
    "result_interpretation_reviewer.py"
)


def _load_reviewer():
    spec = importlib.util.spec_from_file_location("r6_i6_reviewer", str(REVIEWER))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_i6_review_matches_persisted_machine_report():
    reviewer = _load_reviewer()
    built = reviewer.build_review(WORKSPACE)
    output = WORKSPACE / reviewer.OUTPUT_RELATIVE
    persisted = yaml.safe_load(output.read_text(encoding="utf-8"))
    assert built == persisted
    assert built["closure_pass"] is True
    assert built["i5_interpretation"]["performance_claim_supported"] is False
    assert built["future_performance_design"]["summary"]["training_budget_steps"] == 0


def test_i6_rejects_performance_design_that_claims_authorization():
    reviewer = _load_reviewer()
    design = yaml.safe_load(
        (WORKSPACE / reviewer.PERFORMANCE_DESIGN_RELATIVE).read_text(encoding="utf-8")
    )
    changed = copy.deepcopy(design)
    changed["execution_authorized"] = True
    with pytest.raises(reviewer.InterpretationReviewError):
        reviewer._validate_performance_design(changed)


def test_i6_rejects_missing_all_gate_failure_rule():
    reviewer = _load_reviewer()
    design = yaml.safe_load(
        (WORKSPACE / reviewer.PERFORMANCE_DESIGN_RELATIVE).read_text(encoding="utf-8")
    )
    changed = copy.deepcopy(design)
    changed["confirmatory_decision_rule"]["decision_if_any_condition_fails"] = (
        "select_best_seed"
    )
    with pytest.raises(reviewer.InterpretationReviewError):
        reviewer._validate_performance_design(changed)


def test_i6_semantic_counts_are_interpreted_without_performance_inference():
    reviewer = _load_reviewer()
    review = reviewer.build_review(WORKSPACE)
    observations = review["i5_replay_verification"]["semantic_observations"]
    assert observations[-2]["non_none_overlay_count"] == 18
    assert observations[-1]["non_none_overlay_count"] == 0
    assert review["i5_interpretation"]["integration_claim_supported"] is True
    assert review["i5_interpretation"]["performance_improvement_established"] is False
