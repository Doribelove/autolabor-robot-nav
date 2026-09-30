import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
SCRIPT = W / (
    "src/tools/thesis_experiment/scripts/"
    "build_v3_06_d0_c6_failure_attribution.py"
)


def test_v3_06_d0_frozen_report_replays_exactly():
    spec = importlib.util.spec_from_file_location(
        "v3_06_d0_tested", SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    report = module.build()
    assert report["status"] == "offline_c6_failure_attribution_complete"
    assert report["source_population"]["decision_count"] == 614
    assert report["diagnosis"]["straight_creep_family_saturated"] is True
    assert (
        report["diagnosis"][
            "new_spatial_intervention_data_and_calibration_required"
        ]
        is True
    )
