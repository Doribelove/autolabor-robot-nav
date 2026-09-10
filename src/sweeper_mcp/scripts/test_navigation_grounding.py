#!/usr/bin/env python3
"""Offline regressions: regions are not points; no cloud/ROS/motion."""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from test_agent_planning import AgentError, FakeLLM, LocalMCP, _runner, _step
from sweeper_mcp.agent import AgentRunner, SYSTEM_PROMPT
from sweeper_mcp.navigation_grounding import (map_targets, relative_targets,
                                              unresolved_reference_error)


def map_step(x=8, y=0, yaw=0):
    return _step("navigate_map_pose", {"x_m": x, "y_m": y, "yaw_deg": yaw})


def assert_rejected(text, steps):
    runner = _runner({"steps": steps})
    try:
        runner.run(text, execute=True)
    except AgentError as exc:
        assert "点位依据" in str(exc), exc
    else:
        assert False, "ungrounded navigation must not execute"
    assert runner.mcp.backend.calls == []
    assert runner.llm.calls == 2  # a bounded planning-only correction retry


def test_region_cleaning_requires_no_point():
    runner = _runner({"steps": [_step("start_coverage_cleaning", {"regions": ["A区"]})]})
    result = runner.run("去A区，在那里清扫一下", execute=False)
    assert result["state"] == "PREVIEW"
    assert [s["tool"] for s in result["plan"]["steps"]] == ["start_coverage_cleaning"]
    assert runner.mcp.backend.calls == []


def test_region_navigation_then_cleaning_is_rejected_atomically():
    assert_rejected("请你到研究院前门去，在那里清扫一下", [map_step(),
                    _step("start_coverage_cleaning", {"regions": ["A区"]})])


def test_bad_late_navigation_prevents_even_first_cleaning():
    assert_rejected("先清扫A区，然后到前门去", [
        _step("start_coverage_cleaning", {"regions": ["A区"]}), map_step()])


def test_region_alone_must_not_become_a_point():
    assert_rejected("请到研究院前门去", [map_step()])


def test_hallucinated_relative_distance_is_rejected():
    assert_rejected("去前门清扫", [_step("navigate_relative", {
        "forward_m": 1, "left_m": 0, "delta_yaw_deg": 0})])


def test_vague_relative_move_is_rejected():
    assert_rejected("往前一点", [_step("navigate_relative", {
        "forward_m": 1, "left_m": 0, "delta_yaw_deg": 0})])


def test_explicit_map_point_then_cleaning_is_preserved():
    runner = _runner({"steps": [map_step(1, 2), _step("start_coverage_cleaning", {"regions": ["A区"]})]})
    result = runner.run("先到地图坐标(1,2)，再清扫A区", execute=False)
    assert [s["tool"] for s in result["plan"]["steps"]] == ["navigate_map_pose", "start_coverage_cleaning"]


def test_wrong_coordinates_and_yaw_are_rejected():
    assert_rejected("到地图坐标(1,2)", [map_step(8, 0)])
    assert_rejected("到地图坐标(1,2)", [map_step(1, 2, 90)])


def test_numbers_in_region_name_or_repeat_count_are_not_a_point():
    assert_rejected("清扫1号区域2遍", [map_step(1, 2)])


def test_origin_and_numeric_notations():
    assert map_targets("最后返回地图原点") == [(0, 0, 0)]
    assert map_targets("回到地图上面的原点") == [(0, 0, 0)]
    assert map_targets("先回地图零点") == [(0, 0, 0)]
    assert map_targets("回到原点") == [(0, 0, 0)]
    assert map_targets("回零点") == [(0, 0, 0)]
    assert map_targets("回到起点") == []
    assert map_targets("到地图坐标（负一点五，二），朝向90度") == [(-1.5, 2, 90)]
    assert map_targets("map x=3, y=-2, yaw_deg=45") == [(3, -2, 45)]
    assert map_targets("地图坐标八零") == []  # no X/Y separator; ask instead of guessing


def test_relative_explicit_offsets():
    assert relative_targets("向前两米，然后向右一点五米，左转九十度") == [(2, 0, 0), (0, -1.5, 90)]
    assert relative_targets("后退0.5米，再顺时针转30度") == [(-0.5, 0, 0), (0, 0, -30)]


def test_preview_history_does_not_authorise_reusing_coordinates():
    runner = _runner({"steps": [map_step(1, 2)]})
    runner.run("导航到地图坐标(1,2)", execute=False)
    try:
        runner.run("现在去研究院前门", execute=True)
    except AgentError:
        pass
    else:
        assert False
    assert runner.mcp.backend.calls == []


def test_unresolved_user_coordinate_can_be_completed_in_context():
    llm = FakeLLM({"steps": [], "needs_clarification": True, "answer": "点位已收到，清扫哪个区域？"})
    runner = AgentRunner(llm=llm, mcp=LocalMCP(), cloud_authorization_checker=lambda: True)
    runner.run("先去地图坐标(1,2)，然后清扫一个区域", execute=False)
    llm.document = {"steps": [map_step(1, 2), _step("start_coverage_cleaning", {"regions": ["A区"]})]}
    result = runner.run("A区", execute=False)
    assert result["state"] == "PREVIEW"


def test_prompt_distinguishes_region_arrival_from_independent_navigation():
    assert "不按动词数量机械拆分" in SYSTEM_PROMPT
    assert "在那里清扫" in SYSTEM_PROMPT
    assert "没有命名导航点目录" in SYSTEM_PROMPT


def test_partial_scientific_notation_is_not_a_coordinate():
    assert map_targets("去地图坐标(1,2e3)") == []
    assert map_targets("去地图坐标(1,2.3.4)") == []


def test_retry_can_replace_ungrounded_navigation_with_single_coverage():
    clean = _step("start_coverage_cleaning", {"regions": ["A区"]})
    class CorrectingLLM(FakeLLM):
        def chat(self, *args, **kwargs):
            if self.calls:
                self.document = {"steps": [clean]}
            return super().chat(*args, **kwargs)
    runner = AgentRunner(llm=CorrectingLLM({"steps": [map_step(), clean]}),
                         mcp=LocalMCP(), cloud_authorization_checker=lambda: True)
    result = runner.run("去A区在那里清扫", execute=False)
    assert [s["tool"] for s in result["plan"]["steps"]] == ["start_coverage_cleaning"]
    assert runner.llm.calls == 2
    assert runner.mcp.backend.calls == []


def test_unsaved_current_point_cannot_be_silently_omitted():
    bad = {"steps": [_step("start_spot_cleaning", {})]}
    assert "不得跳过" in unresolved_reference_error(
        bad, "回到现在那个点看看垃圾，有就清扫")
    clarification = {"steps": [], "needs_clarification": True,
                     "answer": "请提供该点坐标"}
    assert unresolved_reference_error(
        clarification, "回到说话时的位置") == ""
