#!/usr/bin/env python3
"""Opt-in cloud checks for generic dynamic coverage-region language matching."""
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
    def __init__(self, extra):
        path = ROOT / "global_maps/map_sets/map_20260829_221335_gimp_cleaned/coverage_regions/fused/regions.json"
        raw = json.loads(path.read_text())
        self.catalog = {"map_digest": raw["map_digest"], "regions": [
            {"id": str(row["id"]), "name": str(row["name"])} for row in raw["regions"]] + list(extra)}

    @staticmethod
    def initialize():
        return {}

    @staticmethod
    def list_tools():
        return {"tools": TOOL_SPECS}

    def call_tool(self, name, arguments=None, **kwargs):
        del arguments, kwargs
        if name != "list_saved_coverage_regions":
            raise AssertionError("No robot tool allowed in this test: " + name)
        return {"is_error": False, "text": json.dumps(self.catalog, ensure_ascii=False)}


def plan_case(config, text, extra):
    mcp = CatalogOnly(extra)
    runner = AgentRunner(
        mcp=mcp, api_key=config["api_key"],
        base_url=config.get("base_url", "https://api.deepseek.com"),
        model=config.get("model", "deepseek-v4-flash"), temperature=0,
        cloud_authorization_checker=lambda: True)
    started = time.monotonic()
    try:
        plan = runner.plan(text)  # never run/execute
    finally:
        runner.close()
    return plan, time.monotonic() - started


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cloud", action="store_true", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    output = Path(args.output)
    if output.exists():
        parser.error("Refusing to overwrite existing evidence")
    config = yaml.safe_load((ROOT / "src/sweeper_mcp/config/sweeper_mcp.yaml").read_text())["agent"]
    rear = {"id": "synthetic-rear-hall", "name": "研究院后大厅"}
    rear_door = {"id": "synthetic-rear-door", "name": "研究院后门厅"}
    west = {"id": "synthetic-rd-west", "name": "研发楼西侧大厅"}
    north = {"id": "synthetic-innovation-north-hall", "name": "创新中心北大厅"}
    north_door = {"id": "synthetic-innovation-north-lobby", "name": "创新中心北门厅"}
    east_lobby = {"id": "synthetic-library-east-lobby", "name": "图书馆东侧门厅"}
    cases = [
        ("new_rear_colloquial", "到研究院后面清扫一下", [rear], "研究院后大厅"),
        ("new_rear_variant", "麻烦把研究院后边那块打扫一下", [rear], "研究院后大厅"),
        ("unrelated_new_name", "去研发楼西边扫一下", [west], "研发楼西侧大厅"),
        ("generic_door_to_unique_hall", "清扫创新中心北门区域", [north], "创新中心北大厅"),
        ("generic_door_hall_paraphrase", "到图书馆东边门的大厅清扫", [east_lobby], "图书馆东侧门厅"),
        ("generic_same_direction_candidates", "清扫创新中心北门区域", [north, north_door], None),
        ("ambiguous_rear", "到研究院后面清扫一下", [rear, rear_door], None),
        ("unknown_place", "去研究院楼顶清扫一下", [rear], None),
        ("region_is_not_waypoint", "到研究院后面去，不清扫", [rear], None),
    ]
    rows = []
    for name, text, extra, expected in cases:
        row = {"case": name, "text": text, "synthetic_regions": extra, "expected": expected}
        try:
            plan, seconds = plan_case(config, text, extra)
            actual = [region for step in plan["steps"]
                      if step["tool"] == "start_coverage_cleaning"
                      for region in step["arguments"]["regions"]]
            acceptable = ({expected} | {item["id"] for item in extra if item["name"] == expected}
                          if expected else set())
            passed = ((len(actual) == 1 and actual[0] in acceptable and
                       not plan.get("needs_clarification"))
                      if expected else (not plan["steps"] and plan.get("needs_clarification") is True))
            row.update(plan=plan, actual=actual, seconds=seconds, passed=passed)
        except Exception as exc:
            row.update(error=str(exc), passed=False)
        rows.append(row)
        print(json.dumps(row, ensure_ascii=False), flush=True)
    evidence = {
        "scope": "cloud plan only; synthetic catalogue additions; no ROS/MCP robot actions",
        "passed": sum(row["passed"] for row in rows), "total": len(rows), "rows": rows,
        "source_sha256": {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in [
            "src/sweeper_mcp/src/sweeper_mcp/agent.py",
            "src/sweeper_mcp/src/sweeper_mcp/region_catalog.py",
            "src/sweeper_mcp/src/sweeper_mcp/tools.py",
        ]},
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x") as stream:
        json.dump(evidence, stream, ensure_ascii=False, indent=2)
    if evidence["passed"] != evidence["total"]:
        raise SystemExit("Some dynamic-region checks failed; evidence preserved")


if __name__ == "__main__":
    main()
