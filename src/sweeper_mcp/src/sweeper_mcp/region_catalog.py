# -*- coding: utf-8 -*-
"""Dynamic coverage-region catalogue parsing and fail-closed plan checks.

Natural-language matching deliberately belongs to the cloud planner.  This
module only proves that every selected region is an exact, unambiguous member
of the catalogue fetched for the current request.
"""
import json


def catalog_rows(catalog_text):
    try:
        document = json.loads(catalog_text)
    except (TypeError, ValueError):
        return []
    if not isinstance(document, dict) or not isinstance(document.get("regions", []), list):
        return []
    rows = []
    for index, item in enumerate(document.get("regions", [])):
        if not isinstance(item, dict):
            continue
        region_id = str(item.get("id", "")).strip()
        name = str(item.get("name", "")).strip()
        if not region_id or not name:
            continue
        rows.append({"id": region_id, "name": name, "owner": "%d:%s" % (index, region_id)})
    return rows


def reference_map(catalog_text):
    """Map each exact unambiguous name/UUID to one catalogue row identity."""
    owners = {}
    for row in catalog_rows(catalog_text):
        for reference in (row["id"], row["name"]):
            owners.setdefault(reference, set()).add(row["owner"])
    return {reference: next(iter(matches)) for reference, matches in owners.items()
            if len(matches) == 1}


def compact_catalog(catalog_text, limit):
    """Render the complete dynamic name/UUID catalogue or fail closed."""
    rows = [{"id": row["id"], "name": row["name"]} for row in catalog_rows(catalog_text)]
    rendered = json.dumps({"regions": rows}, ensure_ascii=False, separators=(",", ":"))
    if len(rendered) > int(limit):
        return ""
    return rendered


def coverage_region_error(plan, references):
    """Reject invented, ambiguous or duplicated coverage catalogue targets."""
    for step in plan.get("steps", []):
        if step.get("tool") != "start_coverage_cleaning":
            continue
        requested = step.get("arguments", {}).get("regions", [])
        if not references:
            return ("当前区域目录不可用或无法完整提供，禁止提交区域清扫。"
                    "请确认 Qt 已加载与当前地图匹配的已保存区域后重试。")
        selected = set()
        for value in requested:
            key = str(value).strip()
            owner = references.get(key)
            if owner is None:
                return ("区域清扫目标不是本轮动态目录中的精确名称或 UUID。"
                        "请根据用户口语重新匹配当前目录；唯一明确时输出目录原文，"
                        "有歧义或无合理候选时返回 needs_clarification=true、steps=[]。")
            if owner in selected:
                return "同一已保存区域不能通过名称和 UUID 重复加入计划。"
            selected.add(owner)
    return ""
