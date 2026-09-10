#!/usr/bin/env python3
import json
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))
sys.path.insert(0, os.path.dirname(__file__))

from sweeper_mcp.agent import AgentError, AgentRunner, SYSTEM_PROMPT
from sweeper_mcp.mock_backend import MockBackend
from sweeper_mcp.region_catalog import (compact_catalog, coverage_region_error,
                                        reference_map)
from sweeper_mcp.tools import ToolResult
from test_agent_planning import FakeLLM, LocalMCP, _step


def catalog(*rows):
    return json.dumps({"regions": [{"id": item[0], "name": item[1]} for item in rows]},
                      ensure_ascii=False)


def test_dynamic_catalog_references_accept_only_exact_current_values():
    text = catalog(("rear-id", "研究院后大厅"), ("west-id", "研发楼西侧大厅"))
    references = reference_map(text)
    assert set(references) == {"rear-id", "研究院后大厅", "west-id", "研发楼西侧大厅"}
    valid = {"steps": [_step("start_coverage_cleaning", {"regions": ["研究院后大厅"]})]}
    approximate = {"steps": [_step("start_coverage_cleaning", {"regions": ["研究院后面"]})]}
    assert coverage_region_error(valid, references) == ""
    assert "精确名称" in coverage_region_error(approximate, references)


def test_duplicate_name_requires_unambiguous_uuid_and_duplicate_owner_is_rejected():
    references = reference_map(catalog(("rear-1", "后区"), ("rear-2", "后区")))
    assert "后区" not in references
    assert references["rear-1"] != references["rear-2"]
    duplicated = {"steps": [_step("start_coverage_cleaning", {
        "regions": ["rear-1", "rear-1"]})]}
    assert "重复" in coverage_region_error(duplicated, references)


def test_invalid_or_oversized_catalog_fails_closed():
    assert reference_map("not-json") == {}
    assert compact_catalog("[]", 6000) == '{"regions":[]}'
    assert compact_catalog(catalog(("id", "很长的区域名称" * 100)), 512) == ""
    plan = {"steps": [_step("start_coverage_cleaning", {"regions": ["任意区域"]})]}
    assert "目录不可用" in coverage_region_error(plan, {})


class DynamicCatalogBackend(MockBackend):
    def list_saved_coverage_regions(self):
        return ToolResult(catalog(("rear-id", "研究院后大厅")), False)


def test_agent_retries_cloud_paraphrase_but_accepts_exact_dynamic_catalog_name():
    clean = _step("start_coverage_cleaning", {"regions": ["研究院后大厅"]})

    class CorrectingLLM(FakeLLM):
        def chat(self, *args, **kwargs):
            if self.calls:
                self.document = {"steps": [clean]}
            return super().chat(*args, **kwargs)

    runner = AgentRunner(
        llm=CorrectingLLM({"steps": [_step(
            "start_coverage_cleaning", {"regions": ["研究院后面"]})]}),
        mcp=LocalMCP(DynamicCatalogBackend()),
        cloud_authorization_checker=lambda: True)
    result = runner.run("到研究院后面清扫一下", execute=False)
    assert result["state"] == "PREVIEW"
    assert result["plan"]["steps"][0]["arguments"]["regions"] == ["研究院后大厅"]
    assert runner.llm.calls == 2
    assert runner.mcp.backend.calls == []


def test_system_prompt_is_generic_and_dynamic_not_a_fixed_alias_list():
    assert "每次规划时动态查询" in SYSTEM_PROMPT
    assert "地点主体、方位及其自然同义表达" in SYSTEM_PROMPT
    assert "不是固定区域、固定短语或别名表" in SYSTEM_PROMPT
    assert "不能偏爱" in SYSTEM_PROMPT
    assert "地点主体和用户明确说出的方位是硬约束" in SYSTEM_PROMPT
