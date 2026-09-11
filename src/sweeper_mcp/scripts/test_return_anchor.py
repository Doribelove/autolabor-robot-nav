#!/usr/bin/env python3
"""Offline request-time pose binding and ordered three-subsystem execution."""
import copy
import json
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from test_agent_planning import AgentRunner, AgentError, FakeLLM, LocalMCP, _step
from sweeper_mcp.mock_backend import MockBackend
from sweeper_mcp.navigation_grounding import (current_return_reference, valid_return_anchor,
                                             unresolved_reference_error)

ANCHOR = dict(frame_id="map", x_m=3.125, y_m=-4.25, yaw_deg=37.5,
              map_digest="a" * 64, localized=True, stamp=1000.0, age_sec=0.1)
TEXT = "先去A区清扫，再回到现在的位置，进行视觉清扫"


def sequence(anchor=ANCHOR):
    return [_step("start_coverage_cleaning", {"regions": ["A区"]}),
            _step("navigate_map_pose", {k: anchor[k] for k in ("x_m", "y_m", "yaw_deg")}),
            _step("start_spot_cleaning", {})]


class PositionBackend(MockBackend):
    def __init__(self, anchor=ANCHOR, fail_tool=""):
        super().__init__(delay=0.005, fail_tool=fail_tool)
        self.anchor = copy.deepcopy(anchor)
        self.capture_count = 0
        self.digest = "a" * 64
        self.dispatched = []

    def get_robot_status(self, capture_map_pose=False):
        if capture_map_pose:
            self.capture_count += 1
        return self._result({"map_pose": self.anchor, "map_pose_error": "未定位"})

    def list_saved_coverage_regions(self):
        data = json.loads(super().list_saved_coverage_regions().text)
        data["map_digest"] = self.digest
        return self._result(data)

    def get_coverage_status(self):
        return self._result(dict(self.coverage, map_digest=self.digest, localized=True))

    def navigate_map_pose(self, x_m, y_m, yaw_deg, expected_map_digest=""):
        assert self.coverage["state"] == "COMPLETED", "must await coverage terminal"
        assert expected_map_digest == self.digest
        self.dispatched.append((x_m, y_m, yaw_deg))
        return super().navigate_map_pose(x_m, y_m, yaw_deg, expected_map_digest)

    def start_spot_cleaning(self):
        assert self.nav["state"] == "succeeded", "must await the same navigation goal"
        return super().start_spot_cleaning()


def runner(document=None, backend=None):
    return AgentRunner(llm=FakeLLM(document or {"steps": sequence()}),
                       mcp=LocalMCP(backend or PositionBackend()),
                       cloud_authorization_checker=lambda: True,
                       authorization_checker=lambda: True, poll_interval=0.005)


def test_all_three_steps_execute_in_order_with_frozen_request_pose():
    backend = PositionBackend()
    agent = runner(backend=backend)
    original_chat = agent.llm.chat
    def chat(*args, **kwargs):
        backend.anchor["x_m"] = 90.0  # robot moves while cloud/coverage runs
        return original_chat(*args, **kwargs)
    agent.llm.chat = chat
    result = agent.run(TEXT, execute=True)
    assert result["state"] == "SUCCEEDED", result
    assert backend.capture_count == 1
    assert backend.dispatched == [(3.125, -4.25, 37.5)]
    assert [c[0] for c in backend.calls] == [s["tool"] for s in sequence()]


def test_preview_captures_but_never_moves():
    agent = runner()
    result = agent.run(TEXT, execute=False)
    assert result["state"] == "PREVIEW"
    assert agent.mcp.backend.capture_count == 1
    assert agent.mcp.backend.calls == []


def test_missing_pose_requires_clarification_and_runs_nothing():
    agent = runner({"steps": [], "needs_clarification": True, "answer": "请先定位"}, PositionBackend(None))
    assert agent.run(TEXT, execute=True)["state"] == "NEEDS_CLARIFICATION"
    assert agent.mcp.backend.calls == []


def test_no_snapshot_does_not_allow_partial_execution():
    agent = runner(backend=PositionBackend(None))
    try: agent.run(TEXT, execute=True)
    except AgentError: pass
    else: assert False
    assert agent.mcp.backend.calls == []


def test_omission_reorder_or_changed_return_coordinate_rejected_atomically():
    cases = [sequence()[:1], sequence()[1:], [sequence()[0], sequence()[2]],
             list(reversed(sequence())), [sequence()[0], sequence()[2], sequence()[1]]]
    changed = sequence()
    changed[1]["arguments"]["yaw_deg"] = 0
    cases.append(changed)
    for steps in cases:
        agent = runner({"steps": steps})
        try: agent.run(TEXT, execute=True)
        except AgentError: pass
        else: assert False, steps
        assert agent.mcp.backend.calls == []


def test_current_reference_paraphrases_and_operator_exclusion():
    for text in ["回到当前的位置", "返回现在这个点", "回到说话时的位置", "返回这里", "回原位"]:
        assert current_return_reference(text), text
    assert not current_return_reference("回到我现在的位置")
    assert not current_return_reference("回到刚才的位置")
    for text in ("返回现在这儿", "回到原地", "回到这个位置", "回到机器人目前停着的位置",
                 "记住机器人现在的位置，清扫A区后回来，进行视觉清扫"):
        assert current_return_reference(text), text
    assert not current_return_reference("记住我现在的位置，清扫后回来")


def test_available_snapshot_does_not_authorize_arbitrary_area_or_human_point():
    for text in ("到A区去", "回到我现在的位置", "回到刚才的位置",
                 "记住我现在的位置，清扫A区后回来，进行视觉清扫"):
        agent = runner()
        try: agent.run(text, execute=True)
        except AgentError: pass
        else: assert False, text
        assert agent.mcp.backend.capture_count == 1
        assert agent.mcp.backend.calls == []


def test_snapshot_is_attempted_without_a_return_keyword_but_not_exposed_as_a_goal():
    agent = runner({"steps": [sequence()[0]]})
    result = agent.run("清扫A区", execute=False)
    assert agent.mcp.backend.capture_count == 1
    assert "return_anchor" not in result["plan"]


def test_invalid_pose_evidence_is_never_trusted():
    for field, value in [("x_m", float("nan")), ("stamp", 0), ("age_sec", 2),
                         ("age_sec", -1), ("frame_id", "camera_init"),
                         ("localized", False), ("map_digest", ""), ("x_m", True)]:
        bad = dict(ANCHOR, **{field: value})
        assert not valid_return_anchor(bad), bad


def test_map_switch_during_cloud_prevents_even_first_step():
    agent = runner()
    chat = agent.llm.chat
    def switched(*args, **kwargs):
        agent.mcp.backend.digest = "b" * 64
        return chat(*args, **kwargs)
    agent.llm.chat = switched
    try: agent.run(TEXT, execute=True)
    except AgentError as exc: assert "地图" in str(exc)
    else: assert False
    assert agent.mcp.backend.calls == []


def test_clarification_preserves_original_pose_not_reply_time_pose():
    agent = runner({"steps": [], "needs_clarification": True, "answer": "清扫哪个区域？"})
    assert agent.run("先区域清扫，再回到现在的位置，进行视觉清扫")["state"] == "NEEDS_CLARIFICATION"
    agent.mcp.backend.anchor["x_m"] = 80
    agent.llm.document = {"steps": sequence()}
    result = agent.run("A区", execute=False)
    assert result["state"] == "PREVIEW"
    assert result["plan"]["return_anchor"]["x_m"] == 3.125
    assert agent.mcp.backend.capture_count == 1


def test_preview_history_cannot_supply_return_point_to_new_instruction():
    agent = runner()
    agent.run(TEXT, execute=False)
    agent.llm.document = {"steps": [sequence()[1]]}
    try: agent.run("到A区去", execute=True)
    except AgentError: pass
    else: assert False
    assert agent.mcp.backend.calls == []


def test_failed_navigation_does_not_start_visual():
    agent = runner(backend=PositionBackend(fail_tool="navigate_map_pose"))
    result = agent.run(TEXT, execute=True)
    assert result["state"] == "FAILED"
    assert "start_spot_cleaning" not in [c[0] for c in agent.mcp.backend.calls]
