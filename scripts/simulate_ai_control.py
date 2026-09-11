#!/usr/bin/env python3
"""Adversarial AI execution scenarios, using only in-memory robot interfaces.

Run inside optimized.sh and a network namespace containing only lo. No robot
nodes, microphone, cloud, publishers or service clients are created. The real
AgentRunner, registry, authorization callbacks and visual status adapter run.
Failures are reported, not turned into expected successes or production fixes.
"""
import copy
import argparse
import json
from pathlib import Path
import socket
import sys
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/sweeper_mcp/scripts"))
from test_asr_authorization import _bare_node
from test_return_anchor import ANCHOR, TEXT, sequence
from test_agent_planning import AgentRunner, FakeLLM, LocalMCP
from sweeper_mcp.mock_backend import MockBackend
from sweeper_mcp.ros_backend import ROSBackend
from sweeper_mcp.voice_turns import VoiceTurnBuffer
from autolabor_fod_control.visual_session import VisualSessionReceipt


class ScenarioBackend(MockBackend):
    def __init__(self, fault=""):
        super().__init__()
        self.fault = fault
        self.anchor = copy.deepcopy(ANCHOR)
        self.digest = ANCHOR["map_digest"]
        self.localized = True
        self.capture_count = 0
        self.trace = []
        self.polls = dict(coverage=0, navigation=0, visual=0)
        self.node = None
        self.stage = "idle"
        self.catalog = {"map_digest": self.digest, "regions": [
            {"id": "region-a", "name": "A区"}]}
        self.visual_adapter = ROSBackend()
        self.visual_adapter._ensure = lambda: (_ for _ in ()).throw(
            AssertionError("ROS initialization forbidden in simulation"))
        self.visual_adapter._snapshot = self.visual_snapshot
        self.visual_adapter._ai_visual_owned = True
        self.visual_receipt = VisualSessionReceipt()

    def mark(self, kind, **fields):
        self.trace.append(dict(kind=kind, **copy.deepcopy(fields)))

    def _record(self, name, arguments=None):
        self.mark("tool", tool=name, arguments=arguments or {})
        return super()._record(name, arguments)

    def get_robot_status(self, capture_map_pose=False):
        self.capture_count += int(capture_map_pose)
        self.mark("capture", pose=self.anchor)
        return self._result({"map_pose": copy.deepcopy(self.anchor)})

    def list_saved_coverage_regions(self):
        return self._result(self.catalog)

    def start_coverage_cleaning(self, regions, **kwargs):
        self.stage = "coverage"
        self.coverage_started_at = time.monotonic()
        self._record("start_coverage_cleaning", {"regions": regions})
        self.coverage = dict(state="SWEEPING", active=True, batch_active=True,
                             batch_id="simulation-batch", ai_owned=True)
        self.anchor.update(x_m=40.0, y_m=20.0, yaw_deg=-90.0)
        return self._result(self.coverage)

    def get_coverage_status(self):
        if self.stage == "coverage":
            self.node._status_tick(None)
            self.polls["coverage"] += 1
            count = self.polls["coverage"]
            if count == 1 and self.fault == "cancel_coverage":
                self.node._runner.cancel()
            elif count >= 3 and (self.fault != "qt_gap_5s" or
                                time.monotonic() - self.coverage_started_at >= 5.2):
                state = "FAILED" if self.fault == "coverage_failed" else "COMPLETED"
                self.coverage.update(state=state, active=False, batch_active=False)
                self.stage = "coverage_done"
                if self.fault == "map_changed": self.digest = "b" * 64
                if self.fault == "localization_lost": self.localized = False
            self.mark("coverage", **self.coverage)
        return self._result(dict(self.coverage, map_digest=self.digest,
                                 localized=self.localized))

    def navigate_map_pose(self, x_m, y_m, yaw_deg, expected_map_digest=""):
        assert self.coverage["state"] == "COMPLETED"
        assert not self.coverage["active"] and not self.coverage["batch_active"]
        already_navigated = any(name == "navigate_map_pose" for name, _ in self.calls)
        assert (x_m, y_m, yaw_deg) == ((0.0, 0.0, 0.0) if already_navigated else (3.125, -4.25, 37.5))
        assert expected_map_digest == ("" if already_navigated else self.digest)
        self._record("navigate_map_pose", dict(x_m=x_m, y_m=y_m, yaw_deg=yaw_deg))
        self.stage = "navigation"
        self.polls["navigation"] = 0
        self.nav = dict(state="active", goal_id="simulation-goal-%d" % (2 if already_navigated else 1))
        return self._result(self.nav)

    def get_navigation_status(self):
        if self.stage == "navigation":
            self.polls["navigation"] += 1
            if self.polls["navigation"] == 1 and self.fault == "revoke_control":
                self.node._set_authorization(SimpleNamespace(
                    session_token="session-token", gate="control", enabled=False))
            elif self.polls["navigation"] >= 3:
                self.nav["state"] = "aborted" if self.fault == "navigation_failed" else "succeeded"
                self.stage = "navigation_done"
            self.mark("navigation", **self.nav)
        return self._result(self.nav)

    def cancel_navigation(self):
        self.stage = "navigation_cancelled"
        self.nav["state"] = "preempted"
        self.mark("cancel_navigation", **self.nav)
        return self._result(dict(self.nav, canceled=True))

    def cancel_coverage(self):
        self.stage = "coverage_cancelled"
        self.coverage.update(state="CANCELED", active=False, batch_active=False)
        self.mark("cancel_coverage", **self.coverage)
        return self._result(self.coverage)

    def start_spot_cleaning(self):
        assert self.nav["state"] == "succeeded"
        self._record("start_spot_cleaning")
        self.stage = "visual"
        if self.fault == "cancel_visual_start":
            self.node._runner.cancel()  # before a new ID is known
        self.visual_receipt.begin()
        self.visual_adapter._ai_visual_session_id = self.visual_receipt.session_id
        self.visual_adapter._ai_visual_baseline = self.visual_receipt.sequence
        return self._result(dict(accepted=True, state="FOD_ACTIVE",
                                 visual_session_id=self.visual_receipt.session_id,
                                 completion_baseline=self.visual_receipt.sequence))

    def visual_snapshot(self, key, _max_age):
        count = self.polls["visual"]
        state, visual_state, active = "FOD_ACTIVE", "APPROACH", True
        reason = "synthetic visual tracking"
        if count >= 3:
            state, visual_state, active = "GPS_ACTIVE", "COMPLETE", False
            if self.fault == "visual_abort": state, visual_state = "FOD_ABORTED", "ABORT"
            elif self.fault == "visual_fault": state, visual_state = "FAULT_STOP", "ABORT"
            elif self.fault == "abort_then_mode_return":
                visual_state, reason = "ABORT", "operator returned to navigation after visual abort"
            elif self.fault == "mode_return_without_completion":
                visual_state, reason = "IDLE", "visual disabled without completion"
            elif self.fault == "mode_return_while_visual_active":
                visual_state, active = "APPROACH", True
            if visual_state == "COMPLETE": self.visual_receipt.complete()
            elif visual_state == "ABORT": self.visual_receipt.abort()
            elif not active: self.visual_receipt.disable()
        data = (dict(state=state, visual_state=visual_state, reason=reason)
                if key == "mode" else dict(state=visual_state, active=active,
                    ai_completion_protocol=1, visual_session=self.visual_receipt.status()))
        return SimpleNamespace(data=json.dumps(data))

    def get_visual_servo_status(self):
        self.polls["visual"] += 1
        if self.fault == "cancel_visual" and self.polls["visual"] == 1:
            self.node._runner.cancel()
        result = self.visual_adapter.get_visual_servo_status()
        self.mark("visual", **json.loads(result.text))
        if self.fault == "visual_tool_error" and self.polls["visual"] >= 3:
            return self._result(json.loads(result.text), error=True)
        return result

    def stop_spot_cleaning(self):
        self.mark("stop_visual")
        if not self.visual_receipt.session_id:
            return self._result(dict(stopped=False, reason="new session not known yet"), error=True)
        self.visual_receipt.disable()
        self.stage = "visual_cancelled"
        return self._result(dict(stopped=True))


class ReplayLLM(FakeLLM):
    def __init__(self, response):
        super().__init__({})
        self.response = response

    def chat(self, messages, tools=None, tool_choice=None):
        if tools:
            self.calls += 1
            return copy.deepcopy(self.response)
        return {"content": "模拟执行总结；没有实车控制。"}


def execution_case(fault, cloud_row=None, followup=False):
    backend = ScenarioBackend(fault)
    node = _bare_node()
    node._parse_authorized = node._control_authorized = node._task_active = True
    if fault == "qt_delay": node._last_heartbeat = time.monotonic() - 600.0
    events = []
    llm, text = FakeLLM({"steps": sequence()}), TEXT
    if followup:
        llm.document["steps"].append({"tool": "navigate_map_pose", "description": "导航到地图原点", "arguments": {
            "x_m": 0.0, "y_m": 0.0, "yaw_deg": 0.0}})
        text += "，最后导航到地图原点"
    if cloud_row:
        raw = json.loads((ROOT / "global_maps/map_sets/map_20260829_221335_gimp_cleaned/coverage_regions/fused/regions.json").read_text())
        backend.catalog = {"map_digest": raw["map_digest"], "regions": [
            {"id": region["id"], "name": region["name"]} for region in raw["regions"]]}
        backend.digest = backend.anchor["map_digest"] = raw["map_digest"]
        llm, text = ReplayLLM(cloud_row["cloud_responses"][-1]), cloud_row["text"]
    runner = AgentRunner(mcp=LocalMCP(backend), llm=llm,
                         authorization_checker=node._execution_authorized,
                         cloud_authorization_checker=node._cloud_authorized,
                         event_callback=lambda item: events.append(copy.deepcopy(item)),
                         poll_interval=0.1 if fault == "qt_gap_5s" else 0.002,
                         visual_wait_timeout=0.8 if fault == "qt_gap_5s" else 0.1)
    node._runner, backend.node = runner, node
    node._status_tick(None)
    try:
        result = runner.run(text, execute=True)
    except Exception as exc:
        result = dict(state="REJECTED", error=str(exc))
    names = [item[0] for item in backend.calls]
    expected = {
        "normal": ("SUCCEEDED", 3), "qt_delay": ("SUCCEEDED", 3), "qt_gap_5s": ("SUCCEEDED", 3),
        "coverage_failed": ("FAILED", 1), "navigation_failed": ("FAILED", 2),
        "map_changed": ("REJECTED", 1), "localization_lost": ("REJECTED", 1),
        "cancel_coverage": ("CANCELLED", 1), "revoke_control": ("CANCELLED", 2),
        "visual_abort": ("FAILED", 3), "visual_fault": ("FAILED", 3),
        "abort_then_mode_return": ("FAILED", 3),
        "mode_return_without_completion": ("FAILED", 3),
        "mode_return_while_visual_active": ("FAILED", 3),
        "visual_tool_error": ("FAILED", 3),
        "cancel_visual": ("CANCELLED", 3), "cancel_visual_start": ("CANCELLED", 3),
    }[fault]
    if followup:
        expected = ("SUCCEEDED", 4) if fault == "normal" else ("FAILED", 3)
    ok = result["state"] == expected[0] and len(names) == expected[1] and backend.capture_count == 1
    if fault in {"cancel_visual", "cancel_visual_start"}:
        ok = ok and backend.visual_receipt.result == "CANCELLED" and backend.stage == "visual_cancelled"
        if fault == "cancel_visual_start":
            ok = ok and sum(item["kind"] == "stop_visual" for item in backend.trace) == 2
    return dict(case=fault, ok=ok, expected_state=expected[0], actual_state=result["state"],
                mutations=names, capture_count=backend.capture_count, result=result,
                trace=backend.trace, events=events,
                qt_handshake_age_s=time.monotonic() - node._last_heartbeat,
                control_authorized=node._control_authorized)


def voice_case(pause, unfinished):
    buffer = VoiceTurnBuffer()
    suffix = "然后" if unfinished else ""
    buffer.add("先清扫A区" + suffix, 10.0, 10.0)
    early = buffer.poll(10.0 + pause)
    buffer.add("再回到现在的位置，进行视觉清扫", 11.0 + pause, 11.0 + pause)
    # The new bounded multi-action grace is 4s; preserve the same early-pause
    # assertions, but inspect the final dispatch after the configured grace.
    final = buffer.poll(15.1 + pause)
    # Assess intended single-utterance composition, not a fake ASR accuracy score.
    ok = early is None and final is not None and "先清扫A区" in final.get("text", "")
    return dict(case="voice_pause", pause_s=pause, unfinished_suffix=unfinished,
                ok=ok, early=early, final=final)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cloud-log", action="append", default=[])
    parser.add_argument("--only-cloud", action="store_true")
    args = parser.parse_args()
    # /sys is a read-only host bind mount; query the actual network namespace.
    interfaces = sorted(name for _, name in socket.if_nameindex())
    if interfaces != ["lo"]:
        raise SystemExit("Refusing to run: requires isolated network namespace with only lo")
    print(json.dumps(dict(isolated_interfaces=interfaces, robot_control=False,
                          mock_execution=True, real_agent=True), ensure_ascii=False), flush=True)
    rows = []
    faults = [] if args.only_cloud else [
        "normal", "qt_delay", "qt_gap_5s", "coverage_failed", "navigation_failed", "map_changed",
        "localization_lost", "cancel_coverage", "revoke_control", "visual_abort", "visual_fault",
        "abort_then_mode_return", "mode_return_without_completion", "mode_return_while_visual_active",
        "visual_tool_error", "cancel_visual", "cancel_visual_start"]
    for fault in faults:
        try: row = execution_case(fault)
        except Exception as exc: row = dict(case=fault, ok=False, fixture_error=repr(exc))
        rows.append(row)
        print(json.dumps(row, ensure_ascii=False), flush=True)
    for unfinished in (() if args.only_cloud else (False, True)):
        for pause in (0.6, 1.0, 1.4, 1.6, 2.5):
            row = voice_case(pause, unfinished)
            rows.append(row)
            print(json.dumps(row, ensure_ascii=False), flush=True)
    if not args.only_cloud:
        row = execution_case("mode_return_without_completion", followup=True)
        row["case"] = "unfinished_visual_must_block_fourth_navigation"
        rows.append(row)
        print(json.dumps(row, ensure_ascii=False), flush=True)
        row = execution_case("normal", followup=True)
        row["case"] = "completed_visual_allows_fourth_navigation"
        rows.append(row)
        print(json.dumps(row, ensure_ascii=False), flush=True)
    for cloud_log in args.cloud_log:
        for line in Path(cloud_log).read_text().splitlines():
            saved = json.loads(line)
            if saved.get("expected") != "chain" or not saved.get("ok"):
                continue
            row = execution_case("normal", saved)
            row.update(case="cloud_plan_replay", source=cloud_log, text=saved["text"])
            rows.append(row)
            print(json.dumps(row, ensure_ascii=False), flush=True)
    print(json.dumps(dict(passed=sum(r["ok"] for r in rows), total=len(rows),
                          failed=[{k: v for k, v in r.items() if k in (
                              "case", "pause_s", "unfinished_suffix", "actual_state", "fixture_error")}
                                  for r in rows if not r["ok"]]), ensure_ascii=False), flush=True)
    return 0 if all(row["ok"] for row in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
