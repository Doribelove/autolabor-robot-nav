import math
import sys
from pathlib import Path
import xml.etree.ElementTree as ET


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
import v3_13_multiactor_design as base
import v3_16_response_gated_design as gated


SCENE = {
    "scene_id": "fixture",
    "actor_paths": [
        {"conflict_x_m": 0.5, "conflict_y_m": 0.0, "angle_deg": 90.0},
        {"conflict_x_m": 3.0, "conflict_y_m": 0.0, "angle_deg": 75.0},
    ],
}


def plugins(text):
    root = ET.fromstring(text)
    return [x for x in root.findall(".//plugin") if x.attrib.get("filename") == "libv3_reactive_actor_plugin.so"]


def test_baseline_world_remains_ungated():
    text, specs = base.render_world(SCENE, ["hesitant", "cautious"], 7741)
    assert len(specs) == 2
    assert all(x.find("response_enable_time") is None for x in plugins(text))


def test_v3_16_adds_exact_single_response_gate():
    text, specs = gated.render_world(SCENE, ["hesitant", "cautious"], 7741)
    values = [float(x.find("response_enable_time").text) for x in plugins(text)]
    assert values == [12.55, 12.55]
    assert all(math.isclose(x["response_enable_sim_s"], 12.55) for x in specs)
    assert all(x["prebranch_response_disabled"] for x in specs)


def test_other_actor_parameters_are_byte_semantically_equal():
    _, baseline = base.render_world(SCENE, ["hesitant", "cautious"], 7741)
    _, treatment = gated.render_world(SCENE, ["hesitant", "cautious"], 7741)
    cleaned = [{k: v for k, v in item.items() if k not in ("response_enable_sim_s", "prebranch_response_disabled")} for item in treatment]
    assert cleaned == baseline

