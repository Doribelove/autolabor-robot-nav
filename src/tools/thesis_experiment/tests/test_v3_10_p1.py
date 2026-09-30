import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
BUILDER = W / "src/tools/thesis_experiment/scripts/build_v3_10_p1_paper_closure.py"
REVIEWER = W / "src/tools/thesis_experiment/scripts/review_v3_10_p1_paper_closure.py"


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_v3_10_p1_exports_preserve_formal_result_and_claim_boundary():
    builder = load(BUILDER, "v3_10_p1_builder_test")
    outputs = builder.compute_outputs()
    assert set(outputs) == {builder.SUMMARY, builder.POLICY, builder.BLOCK, builder.FIGURE, builder.CLAIMS}
    claims = builder.yaml.safe_load(outputs[builder.CLAIMS])
    assert claims["formal_result"] is True
    assert claims["performance_improvement_demonstrated"] is True
    assert claims["qualified_scope"]["fresh_blocks"] == 30
    assert claims["forbidden_extrapolations"]["actual_teb_superiority"] is False
    assert claims["forbidden_extrapolations"]["real_vehicle_ready"] is False


def test_v3_10_p1_independent_review_and_manifest_newlines():
    builder = load(BUILDER, "v3_10_p1_builder_review_test")
    reviewer = load(REVIEWER, "v3_10_p1_reviewer_test")
    value = reviewer.review_value(builder)
    assert value["status"] == "v3_10_p1_offline_paper_evidence_closure_pass"
    assert value["point_relative_reduction"] > 0.10
    assert value["one_sided_lower_relative_reduction"] > 0.05
    assert value["actual_teb_comparison"] is False
    payload = (W / builder.MANIFEST).read_bytes()
    assert b"\\n" not in payload
    assert payload.endswith(b"\n")
