#!/usr/bin/env python3
"""Explicit opt-in cloud plan checks. No ROS endpoints and no action execution."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import yaml
from sweeper_mcp.agent import AgentRunner
from sweeper_mcp.tools import TOOL_SPECS

ROOT = Path(__file__).resolve().parents[1]


class CatalogOnly:
    def __init__(self):
        path = ROOT / "global_maps/map_sets/map_20260829_221335_gimp_cleaned/coverage_regions/fused/regions.json"
        raw = json.loads(path.read_text())
        self.catalog = {"map_digest": raw["map_digest"], "regions": [
            {"id": r["id"], "name": r["name"]} for r in raw["regions"]]}

    def initialize(self):
        return {}

    def list_tools(self):
        return {"tools": TOOL_SPECS}

    def call_tool(self, name, arguments=None, **kwargs):
        if name != "list_saved_coverage_regions":
            raise AssertionError("No robot tool allowed in this test: " + name)
        return {"is_error": False, "text": json.dumps(self.catalog, ensure_ascii=False)}


class RecordingClient:
    def __init__(self, inner):
        self.inner, self.responses = inner, []

    @property
    def last_metrics(self):
        return self.inner.last_metrics

    def chat(self, *args, **kwargs):
        response = self.inner.chat(*args, **kwargs)
        self.responses.append(response)
        return response


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cloud", action="store_true", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    path = Path(args.output)
    if path.exists():
        parser.error("Refusing to overwrite existing evidence")
    config = yaml.safe_load((ROOT / "src/sweeper_mcp/config/sweeper_mcp.yaml").read_text())["agent"]
    mcp = CatalogOnly()
    runner = AgentRunner(mcp=mcp, api_key=config["api_key"],
                         base_url=config.get("base_url", "https://api.deepseek.com"),
                         model=config.get("model", "deepseek-v4-flash"), temperature=0,
                         cloud_authorization_checker=lambda: True)
    runner.llm = RecordingClient(runner.llm)
    names = {r["id"]: r["name"] for r in mcp.catalog["regions"]}
    front, side = "研究院大楼前大厅", "研究院大楼侧门厅"
    cases = [
        ("user_example", "请你到研究院前门去，在那里清扫一下", ["clean:" + front]),
        ("front_door_region", "请清扫研究院大楼前门区域", ["clean:" + front]),
        ("front_entrance_area", "到研究院大楼入口那块清扫一下", ["clean:" + front]),
        ("front_main_door", "把研究院大楼正门口那块地方扫一遍", ["clean:" + front]),
        ("front_side_colloquial", "去研究院大楼前边把地清扫一下", ["clean:" + front]),
        ("front_hall_colloquial", "研究院前面的大厅需要打扫", ["clean:" + front]),
        ("side_door_hall", "请清扫研究院大楼侧门的大厅", ["clean:" + side]),
        ("side_door_area", "把研究院大楼侧门区域清理一下", ["clean:" + side]),
        ("side_entrance_hall", "去研究院侧边入口大厅做区域清扫", ["clean:" + side]),
        ("area_arrival", "去A区，然后在那里清扫一下", ["clean:A"]),
        ("vague_front_navigation", "请你到研究院前门去，不清扫", None),
        ("explicit_point_then_clean", "先到地图坐标(1,2)，再清扫A区", ["map:1,2,0", "clean:A"]),
        ("clean_then_origin", "清扫A区，最后返回地图原点", ["clean:A", "map:0,0,0"]),
        ("plain_origin", "清扫A区以后回原点", ["clean:A", "map:0,0,0"]),
        ("visual_search", "看看周围有没有垃圾，有垃圾就清扫", ["start_spot_cleaning"]),
        ("visual_find_nearby", "找找附近有没有垃圾，发现了就清理", ["start_spot_cleaning"]),
        ("visual_check_around", "检查一下四周，有需要清扫的垃圾就处理掉", ["start_spot_cleaning"]),
        ("visual_explicit", "启动视觉伺服清扫周边垃圾", ["start_spot_cleaning"]),
        ("origin_then_visual", "返回地图原点，看看周围有没有垃圾，有垃圾就清扫",
         ["map:0,0,0", "start_spot_cleaning"]),
        ("zero_then_visual_variant", "先回到地图零点，再找一下附近垃圾，发现了就清理",
         ["map:0,0,0", "start_spot_cleaning"]),
        ("plain_origin_visual", "回原点看看有没有垃圾，有的话就清扫",
         ["map:0,0,0", "start_spot_cleaning"]),
        ("areas_origin_visual",
         "先清扫研究院大楼前门区域，再清扫研究院大楼侧门的大厅，最后返回地图原点，看看周围有没有垃圾，有垃圾就清扫",
         ["clean:" + front, "clean:" + side, "map:0,0,0", "start_spot_cleaning"]),
        ("natural_long_variant",
         "先把研究院正门口那块扫完，接着清扫研究院侧边入口大厅，做完回地图零点，找找周围的垃圾，发现了就处理",
         ["clean:" + front, "clean:" + side, "map:0,0,0", "start_spot_cleaning"]),
        ("two_areas", "去研究院前门清扫，然后去研究院侧门清扫", ["clean:" + front, "clean:" + side]),
        ("vague_relative", "往前一点", None),
        ("explicit_relative", "向前两米", ["relative:2,0,0"]),
        ("unknown_region", "到不存在的月球大厅在那里清扫一下", None),
        ("independent_waypoint", "先到地图坐标(1,2)，然后去研究院前门在那里清扫一下", ["map:1,2,0", "clean:" + front]),
    ]
    rows = []
    for case, text, expected in cases:
        runner.reset_context()
        runner.llm.responses = []
        row = {"case": case, "text": text, "expected": expected}
        start = time.monotonic()
        try:
            plan = runner.plan(text)  # deliberately never runner.run / execute
            actual = []
            for step in plan["steps"]:
                tool, values = step["tool"], step["arguments"]
                if tool == "start_coverage_cleaning":
                    actual.extend("clean:" + names.get(r, r) for r in values["regions"])
                elif tool == "navigate_map_pose":
                    actual.append("map:" + ",".join(format(values[k], "g") for k in ("x_m", "y_m", "yaw_deg")))
                elif tool == "navigate_relative":
                    actual.append("relative:" + ",".join(format(values[k], "g") for k in ("forward_m", "left_m", "delta_yaw_deg")))
                else:
                    actual.append(tool)
            row.update(plan=plan, actual=actual,
                       passed=(not actual and plan.get("needs_clarification") is True) if expected is None else actual == expected)
        except Exception as exc:
            row.update(error=str(exc), passed=False)
        row.update(seconds=time.monotonic() - start, responses=runner.llm.responses)
        rows.append(row)
        print(json.dumps(row, ensure_ascii=False), flush=True)
    evidence = {"scope": "cloud planning only; no ROS, no robot actions", "rows": rows,
                "passed": sum(r["passed"] for r in rows),
                "source_sha256": {f: hashlib.sha256((ROOT / f).read_bytes()).hexdigest() for f in [
                    "src/sweeper_mcp/src/sweeper_mcp/agent.py", "src/sweeper_mcp/src/sweeper_mcp/navigation_grounding.py",
                    "src/sweeper_mcp/src/sweeper_mcp/region_catalog.py",
                    "src/sweeper_mcp/src/sweeper_mcp/tools.py"]}}
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x") as stream:
        json.dump(evidence, stream, ensure_ascii=False, indent=2)
    if evidence["passed"] != len(rows):
        raise SystemExit("Some plan checks failed; evidence preserved")


if __name__ == "__main__":
    main()
