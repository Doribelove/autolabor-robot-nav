import sys
from pathlib import Path
import yaml

W = Path("/home/robot/robot_ws_base_rl")
PACKAGE = W / "src/application/v3_scope_planner/src"
if str(PACKAGE) not in sys.path:
    sys.path.insert(0, str(PACKAGE))
from v3_scope_planner import spatial_core


def values():
    planner = yaml.safe_load(
        (W / "artifacts/v3/planner/v3_08_a0/"
         "v3_08_a0_effective_planner_contract.yaml").read_text()
    )
    model = yaml.safe_load(
        (W / "artifacts/v3/prediction/v3_07_b0_r1/model/"
         "v3_07_b0_r1_spatial_candidate_predictor.yaml").read_text()
    )
    return planner, model


def snapshot(speed, coordinate=-2.6):
    return {
        "scene_snapshot_id": "test", "world_model_seq": 1,
        "snapshot_age_s": 0.0, "predictor_latency_ms": 0.0,
        "verifier_latency_ms": 0.0,
        "ego": {"x_m": -2.6, "y_m": 0.0, "yaw_rad": 0.0, "speed_mps": 0.8},
        "actor": {
            "coordinate_m": coordinate, "speed_mps": speed,
            "path_angle_deg": 90.0, "monotone_coordinate_certified": True,
        },
        "health": {
            "localization_ok": True, "transform_ok": True,
            "actor_track_ok": True, "command_chain_ok": True,
        },
    }


def test_spatial_supported_mode_and_assertive_fail_closed():
    planner, model = values()
    cautious = spatial_core.plan(snapshot(0.15), model, planner)
    assert cautious["decision"]["selected_candidate_id"] == "spatial_left_v120"
    assertive = spatial_core.plan(snapshot(1.35), model, planner)
    assert assertive["decision"]["selected_candidate_id"] == "certified_stop_v000"


def test_cleared_actor_enables_straight_recovery():
    planner, model = values()
    item = snapshot(1.0, coordinate=1.5)
    item["ego"]["x_m"] = -2.2
    item["ego"]["speed_mps"] = 0.0
    result = spatial_core.plan(item, model, planner)
    assert result["decision"]["selected_candidate_id"] == "cleared_straight_v120"
