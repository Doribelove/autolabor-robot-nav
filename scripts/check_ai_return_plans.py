#!/usr/bin/env python3
"""Real cloud, real name catalog, synthetic localized pose; NO ROS or robot writes.

Only two in-memory read-only MCP methods exist here. Every mutating call raises.
Credentials are read privately and never included in output. Results go to stdout.
"""
import json
import argparse
from pathlib import Path
import time
import yaml
from sweeper_mcp.agent import AgentRunner
from sweeper_mcp.navigation_grounding import matches_return_anchor
from sweeper_mcp.tools import TOOL_SPECS

ROOT = Path(__file__).resolve().parents[1]


class ReadOnlyMCP:
    def __init__(self):
        raw = json.loads((ROOT / "global_maps/map_sets/map_20260829_221335_gimp_cleaned/coverage_regions/fused/regions.json").read_text())
        self.catalog = {"map_digest": raw["map_digest"], "regions": [
            {"id": r["id"], "name": r["name"]} for r in raw["regions"]]}
        self.anchor = dict(frame_id="map", x_m=3.125, y_m=-4.25, yaw_deg=37.5,
                           map_digest=raw["map_digest"], localized=True, stamp=1000.0, age_sec=0.1)
        self.capture = True
        self.calls = []
    def initialize(self): return {}
    def list_tools(self): return {"tools": TOOL_SPECS}
    def call_tool(self, name, arguments=None, **kwargs):
        self.calls.append({"tool": name, "arguments": dict(arguments or {})})
        if name == "list_saved_coverage_regions": data = self.catalog
        elif name == "get_robot_status": data = {"map_pose": self.anchor if self.capture else None, "map_pose_error": "未定位"}
        else: raise AssertionError("Robot tool execution forbidden: " + name)
        return {"text": json.dumps(data, ensure_ascii=False), "is_error": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--extended", action="store_true", help="test additional natural return-point expressions")
    parser.add_argument("--minimum", action="store_true", help="repeat the operator's exact minimum acceptance instruction")
    args = parser.parse_args()
    config = yaml.safe_load((ROOT / "src/sweeper_mcp/config/sweeper_mcp.yaml").read_text())["agent"]
    mcp = ReadOnlyMCP()
    agent = AgentRunner(mcp=mcp, api_key=config["api_key"], base_url=config["base_url"],
                        model=config["model"], temperature=0, cloud_authorization_checker=lambda: True)
    responses = []
    cloud_chat = agent.llm.chat
    def traced_chat(*args, **kwargs):
        response = cloud_chat(*args, **kwargs)
        responses.append(response)
        return response
    agent.llm.chat = traced_chat
    cases = [
        ("先去研究院大楼前门区域清扫，再回到当前的位置，进行视觉清扫", "chain"),
        ("先清扫研究院前大厅，清扫完再回到现在的位置，然后启动视觉伺服", "chain"),
        ("请到研究院前门去在那里打扫，完成以后回到现在这个点，看看周围有没有垃圾，有就清扫", "chain"),
        ("先到研究院侧门厅清扫，之后返回这里，进行视觉清扫", "chain"),
        ("清扫研究院前大厅，再回到说话时的位置，最后进行视觉清扫", "chain"),
        ("先清扫研究院前大厅，再返回原位进行视觉伺服清扫", "chain"),
        ("先清扫研究院前大厅，再回到地图原点，最后进行视觉清扫", "origin"),
        ("请你到研究院前门去，在那里清扫一下", "coverage"),
        ("先去区域清扫，再回到现在的位置，进行视觉清扫", "clarify"),
        ("先清扫研究院前大厅，再回到我现在的位置进行视觉清扫", "clarify"),
        ("先清扫研究院前大厅，再回到刚才的位置进行视觉清扫", "clarify"),
    ]
    selected = cases + [(cases[0][0], "unlocalized")]
    if args.extended:
        selected = [
            ("记住机器人现在的位置，先去研究院前大厅清扫，完成后回来，再启动视觉清扫", "chain"),
            ("清扫完研究院前大厅以后，返回现在这儿，然后进行视觉清扫", "chain"),
            ("先清扫研究院前大厅，再回到机器人目前停着的位置，然后启动视觉伺服", "chain"),
            ("请先打扫研究院前门区域，之后回到这个位置，最后进行视觉清扫", "chain"),
            ("先去研究院前门区域清扫一下，完事回到原地，再进行视觉清扫", "chain"),
            ("先清扫研究院前大厅，再回到机器人现在的位置，进行视觉清扫", "chain"),
            ("先清扫研究院前大厅，再回到现在所在的位置，最后进行视觉清扫", "chain"),
            ("先清扫研究院前大厅，接着清扫研究院侧门厅，最后回到当前的位置进行视觉清扫", "multi_chain"),
        ]
    if args.minimum:
        selected = [("清扫A区后回到当前点进行视觉清扫", "chain")] * 5
    rows = []
    for text, expected in selected:
        agent.reset_context()
        mcp.capture = expected != "unlocalized"
        started = time.monotonic()
        responses.clear()
        mcp.calls.clear()
        try:
            result = agent.run(text, execute=False)
            plan = result["plan"]
            names = [s["tool"] for s in plan["steps"]]
            if expected in ("clarify", "unlocalized"): ok = result["state"] == "NEEDS_CLARIFICATION" and not names
            elif expected == "coverage": ok = names == ["start_coverage_cleaning"]
            else:
                if expected == "multi_chain":
                    tools_ok = names in (["start_coverage_cleaning", "navigate_map_pose", "start_spot_cleaning"],
                                         ["start_coverage_cleaning", "start_coverage_cleaning", "navigate_map_pose", "start_spot_cleaning"])
                    regions = [region for step in plan["steps"] if step["tool"] == "start_coverage_cleaning"
                               for region in step["arguments"]["regions"]]
                    lookup = {r["id"]: r["name"] for r in mcp.catalog["regions"]}
                    regions = [lookup.get(region, region) for region in regions]
                    ok = tools_ok and regions == ["研究院大楼前大厅", "研究院大楼侧门厅"]
                    ok = ok and matches_return_anchor(plan["steps"][-2], mcp.anchor)
                else:
                    ok = names == ["start_coverage_cleaning", "navigate_map_pose", "start_spot_cleaning"]
                if expected == "chain": ok = ok and matches_return_anchor(plan["steps"][1], mcp.anchor)
                elif expected == "origin" and ok:
                    ok = all(plan["steps"][1]["arguments"][k] == 0 for k in ("x_m", "y_m", "yaw_deg"))
            if args.minimum:
                region_lookup = {r["id"]: r["name"] for r in mcp.catalog["regions"]}
                regions = [region_lookup.get(region, region) for step in plan["steps"]
                           if step["tool"] == "start_coverage_cleaning" for region in step["arguments"]["regions"]]
                captures = [call for call in mcp.calls if call["tool"] == "get_robot_status"
                            and call["arguments"].get("capture_map_pose") is True]
                ok = ok and regions == ["A"] and len(captures) == 1
            row = dict(text=text, expected=expected, ok=ok, result=result)
        except Exception as exc: row = dict(text=text, expected=expected, ok=False, error=str(exc))
        row["elapsed_s"] = time.monotonic() - started
        row["cloud_responses"] = list(responses)
        row["read_calls"] = list(mcp.calls)
        rows.append(row)
        print(json.dumps(row, ensure_ascii=False), flush=True)
    print(json.dumps({"passed": sum(r["ok"] for r in rows), "total": len(rows), "plan_only": True,
                      "synthetic_pose": mcp.anchor}, ensure_ascii=False), flush=True)
    return 0 if all(r["ok"] for r in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
