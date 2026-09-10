# -*- coding: utf-8 -*-
"""Dynamic coverage-region catalogue parsing and fail-closed plan checks.

Natural-language matching deliberately belongs to the cloud planner.  This
module only proves that every selected region is an exact, unambiguous member
of the catalogue fetched for the current request.
"""
import json
import re
import unicodedata


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


_GENERIC_REGION_WORDS = re.compile(
    r"(?:入口|门口|大门|门厅|大厅|走廊|房间|广场|区域|片区|附近|那块|厅|门|的|大楼)")
_MARKERS = {
    "door": re.compile(r"门|入口"),
    "hall": re.compile(r"厅"),
    "corridor": re.compile(r"走廊|通道"),
    "room": re.compile(r"房间|房"),
    "square": re.compile(r"广场"),
}


def _normal(value):
    return re.sub(r"[^0-9a-z\u4e00-\u9fff]", "", unicodedata.normalize(
        "NFKC", str(value or "")).lower())


def _semantic_stem(value):
    return _GENERIC_REGION_WORDS.sub("", _normal(value))


def _markers(value):
    value = _normal(value)
    return {name for name, pattern in _MARKERS.items() if pattern.search(value)}


def coverage_region_ambiguity_error(plan, user_text, catalog_text):
    """Reject a cloud choice that ignores a same-place/same-direction sibling.

    This is deliberately a narrow safety backstop, not the natural-language
    matcher.  It only groups catalogue names that become identical after
    generic building/room words are removed.  Exact names remain authoritative;
    an explicit place marker such as ``门`` may select the sole matching sibling.
    """
    rows = catalog_rows(catalog_text)
    if len(rows) < 2:
        return ""
    normal_text = _normal(user_text)
    user_markers = _markers(user_text)
    by_reference = {}
    for row in rows:
        by_reference[row["id"]] = row
        by_reference[row["name"]] = row
    for step in plan.get("steps", []):
        if step.get("tool") != "start_coverage_cleaning":
            continue
        for reference in step.get("arguments", {}).get("regions", []):
            selected = by_reference.get(str(reference).strip())
            if selected is None or _normal(selected["name"]) in normal_text or _normal(selected["id"]) in normal_text:
                continue
            stem = _semantic_stem(selected["name"])
            siblings = [row for row in rows if _semantic_stem(row["name"]) == stem]
            if len(siblings) < 2:
                continue
            eligible = [row for row in siblings
                        if not user_markers or user_markers.issubset(_markers(row["name"]))]
            if len(eligible) == 1 and eligible[0]["owner"] == selected["owner"]:
                continue
            candidates = "、".join(row["name"] for row in siblings)
            return ("用户说法在同一地点和方位下仍对应多个目录候选（%s）。"
                    "若用户明确说了门/厅/走廊等部位，只能选择唯一包含该部位语义的候选；"
                    "否则必须返回 needs_clarification=true、steps=[]，不得按目录顺序任选。" % candidates)
    return ""
