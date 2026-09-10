#!/usr/bin/env python3
"""Explicit, plan-only cloud and acoustic regression checks for the voice sample."""
import argparse
import json
import wave
from pathlib import Path

from replay_smart_voice import CatalogMCP, PreviewRunner, ROOT, worker_module
import numpy as np
import webrtcvad
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", required=True)
    parser.add_argument("--wav", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    output = Path(args.output)
    if output.exists():
        raise SystemExit("Refusing to overwrite output")
    config = yaml.safe_load((ROOT / "src/sweeper_mcp/config/sweeper_mcp.yaml").read_text())["agent"]
    mcp = CatalogMCP()
    runner = PreviewRunner(mcp=mcp, api_key=config["api_key"],
                           base_url=config.get("base_url", "https://api.deepseek.com"),
                           model=config.get("model", "deepseek-v4-flash"), temperature=0.0,
                           cloud_authorization_checker=lambda: True)
    rows = []
    cases = [
        ("whole_reference", json.loads(Path(args.reference).read_text())["text"], True),
        ("clarification_question", "请帮我清扫一个区域。", True),
        ("short_region_answer", "研究院大楼前大厅", False),
        ("supported_sequence", "先清扫A区，再清扫B区，然后清扫研究院大楼前大厅和研究院大楼侧门厅，最后返回地图原点。", True),
    ]
    for name, text, reset in cases:
        if reset:
            runner.reset_context()
        try:
            result = runner.run(text, execute=False)
            rows.append(dict(case=name, text=text, result=result))
        except Exception as exc:
            rows.append(dict(case=name, text=text, error=str(exc)))
        print(json.dumps(rows[-1], ensure_ascii=False), flush=True)
    names = {r["id"]: r["name"] for r in mcp.catalog["regions"]}

    def expanded(row):
        actions = []
        for step in row.get("result", {}).get("plan", {}).get("steps", []):
            args = step["arguments"]
            if step["tool"] == "start_coverage_cleaning":
                actions.extend("clean:" + names.get(region, region) for region in args["regions"])
            elif step["tool"] == "navigate_map_pose":
                actions.append("map:" + str([args[k] for k in ("x_m", "y_m", "yaw_deg")]))
            else:
                actions.append(step["tool"])
        return actions
    checks = {
        "whole_reference_retains_unsupported_conditions": rows[0].get("result", {}).get("state") == "NEEDS_CLARIFICATION",
        "question_waits_for_region": rows[1].get("result", {}).get("state") == "NEEDS_CLARIFICATION",
        "short_answer_resolves_pending_request": expanded(rows[2]) == ["clean:研究院大楼前大厅"],
        "supported_regions_in_order": expanded(rows[3])[:4] == ["clean:A", "clean:B", "clean:研究院大楼前大厅", "clean:研究院大楼侧门厅"],
        "supported_returns_to_map_origin_last": bool(expanded(rows[3])) and expanded(rows[3])[-1] in ("map:[0, 0, 0]", "map:[0.0, 0.0, 0.0]"),
    }
    gate = worker_module.WhisperAsrWorker.__new__(worker_module.WhisperAsrWorker)
    gate._vad_factory = lambda: webrtcvad.Vad(2)
    with wave.open(args.wav, "rb") as stream:
        room = stream.readframes(4 * 16000)
    noise = np.random.default_rng(20260909).normal(0, 0.02, 16000 * 5)
    noise_pcm = np.clip(noise * 32768, -32768, 32767).astype("<i2").tobytes()
    acoustic = {name: gate._audio_rejection(pcm) for name, pcm in [
        ("digital_silence", b"\0" * 32000 * 5), ("recorded_room_first_4s", room),
        ("fixed_white_noise", noise_pcm)]}
    checks.update({"reject_" + name: bool(reason) for name, reason in acoustic.items()})
    checks["reject_prompt_hallucination"] = bool(gate._transcript_rejection(
        {}, "语音指令", 0.0165, "简体中文，无人清扫车语音指令"))
    checks["preserve_short_stop"] = not gate._transcript_rejection({}, "停止", 0.01)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x") as stream:
        json.dump({"rows": rows, "checks": checks, "acoustic": acoustic}, stream, ensure_ascii=False, indent=2)
    print("CHECKS " + json.dumps(checks, ensure_ascii=False), flush=True)
    if not all(checks.values()):
        raise SystemExit("Some checks failed; inspect saved results")


if __name__ == "__main__":
    main()
