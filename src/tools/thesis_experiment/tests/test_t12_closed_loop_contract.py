from pathlib import Path

import yaml


ROOT = Path("/home/robot/robot_ws_base_rl")


def test_t12_closed_loop_scene_contract_is_paired_and_no_training():
    state = yaml.safe_load((
        ROOT / "experiments/manifests/t12/closed_loop_scenes.yaml"
    ).read_text(encoding="utf-8"))
    assert state["training_forbidden"] is True
    assert state["simulation_only"] is True
    assert state["real_vehicle_use_forbidden"] is True
    scenes = state["scenes"]
    assert len(scenes) == 10
    assert len({scene["scene_id"] for scene in scenes}) == 10
    assert {scene["split"] for scene in scenes} == {"test_id", "test_ood"}
