"""Conservative user-text evidence for AI navigation targets; no robot I/O.

This is a numeric provenance gate, not a replacement semantic parser. Region
geometry and model-generated descriptions are deliberately never evidence.
Unrecognised coordinate notation requires clarification instead of guessing.
"""
import math
import re
import unicodedata


NUMBER = r"[-+负]?(?:\d+(?:\.\d+)?|[零〇一二两三四五六七八九十百]+(?:点[零〇一二两三四五六七八九]+)?)"
NUMBER_END = r"(?![\d.a-z零〇一二两三四五六七八九十百点])"
DIGITS = dict(zip("零〇一二两三四五六七八九", [0, 0, 1, 2, 2, 3, 4, 5, 6, 7, 8, 9]))


def number(value):
    value = value.replace("负", "-")
    try:
        return float(value)
    except ValueError:
        sign = -1 if value.startswith("-") else 1
        value = value.lstrip("-+")
        whole, _, fraction = value.partition("点")
        if not any(c in whole for c in "十百"):
            integer = int("".join(str(DIGITS[c]) for c in whole))
        else:
            integer, pending = 0, 0
            for char in whole:
                if char in "十百":
                    integer += (pending or 1) * (10 if char == "十" else 100)
                    pending = 0
                else:
                    pending = DIGITS[char]
            integer += pending
        decimal = sum(DIGITS[c] * 10 ** (-i - 1) for i, c in enumerate(fraction))
        return sign * (integer + decimal)


def _yaw(suffix):
    match = re.match(r"[\s,)米]*[,、]?\s*(?:yaw(?:_deg)?|朝向|航向角?|偏航角?)\s*[:=为是]?\s*(" + NUMBER + r")" + NUMBER_END + r"(?:\s*度)?", suffix)
    return number(match[1]) if match else 0.0


def map_targets(text):
    text = unicodedata.normalize("NFKC", text).lower()
    targets = []
    if re.search(r"(?:地图(?:坐标)?(?:上面|上|中|里面|里)?的?)?(?:原点|零点)", text):
        targets.append((0.0, 0.0, 0.0))
    patterns = [
        r"(?<![a-z])x(?:_m)?\s*[:=为是]?\s*(" + NUMBER + r")\s*(?:米)?[\s,、]+y(?:_m)?\s*[:=为是]?\s*(" + NUMBER + r")",
        r"(?:地图(?:坐标|位置|位姿|点)?|map(?:坐标)?|坐标)\s*[:=为是到]?\s*\(?\s*(" + NUMBER + r")[\s,、]+(" + NUMBER + r")",
    ]
    for pattern in patterns:
        for match in re.finditer(pattern + NUMBER_END, text):
            targets.append((number(match[1]), number(match[2]), _yaw(text[match.end():])))
    return targets


_UNSAVED_RETURN_POINT = re.compile(
    r"(?:回|返回|回到).{0,5}(?:(?:现在|当前|此刻|刚才|说话时)(?:的|这个|那个|所在的|所在)?"
    r"(?:点位|点|位置|地方)|这里|这儿|原位)")

_RETURN_CLAUSE = re.compile(
    r"(?:返回到|回到|返回|回)(?:来|[^，,。;；!?！？]{0,24}?(?:位置|点位|地点|地方|原地|原位|这里|这儿|这边|点))")
_CURRENT_DEICTIC = re.compile(r"现在|当前|此刻|目前|说话时|这|原地|原位")
_OTHER_POSITION = re.compile(r"我(?:们)?|操作员|操作者|刚才|之前|以前|上次|原来的|基地|充电|地图原点|地图零点")
_OTHER_SNAPSHOT = re.compile(r"(?:记住|记录|记下).{0,8}(?:我(?:们)?|操作员|刚才|之前|上次).{0,12}(?:位置|点|地方)")


def current_return_reference(text):
    """A robot position captured for this request, not the operator's location."""
    value = unicodedata.normalize("NFKC", str(text or ""))
    for match in _RETURN_CLAUSE.finditer(value):
        clause = match.group()
        if _OTHER_POSITION.search(clause) or (clause == "回来" and _OTHER_SNAPSHOT.search(value)):
            continue
        if _CURRENT_DEICTIC.search(clause) or clause == "回来":
            return match
    return None


def valid_return_anchor(value):
    if not isinstance(value, dict) or value.get("frame_id") != "map" or value.get("localized") is not True:
        return False
    digest = value.get("map_digest", "")
    if not isinstance(digest, str) or not re.fullmatch(r"[0-9a-f]{64}", digest):
        return False
    for key in ("x_m", "y_m", "yaw_deg", "stamp", "age_sec"):
        number_value = value.get(key)
        if isinstance(number_value, bool) or not isinstance(number_value, (int, float)) or not math.isfinite(number_value):
            return False
    return (value["stamp"] > 0 and -0.1 <= value["age_sec"] <= 0.5
            and -180 <= value["yaw_deg"] <= 180)


def matches_return_anchor(step, anchor):
    return bool(valid_return_anchor(anchor) and step["tool"] == "navigate_map_pose" and
                all(math.isclose(float(step["arguments"][k]), anchor[k], rel_tol=0, abs_tol=1e-6)
                    for k in ("x_m", "y_m", "yaw_deg")))


def unresolved_reference_error(plan, text, anchor=None):
    """Do not silently omit a requested return to an uncaptured transient point."""
    value = unicodedata.normalize("NFKC", str(text or "")).lower()
    reference = current_return_reference(value)
    if reference and valid_return_anchor(anchor):
        if not plan.get("steps") and plan.get("needs_clarification"):
            return ""
        returns = [i for i, step in enumerate(plan.get("steps", [])) if matches_return_anchor(step, anchor)]
        if not returns:
            return "不得省略返回步骤；必须用本轮已记录的 map 位姿完整 x_m/y_m/yaw_deg 调用 navigate_map_pose。"
        before, after = value[:reference.start()], value[reference.end():]
        tools = [step["tool"] for step in plan["steps"]]
        if (re.search(r"清扫|打扫|清理", before) and re.search(r"区|大厅|门厅|走廊", before)
                and not any(t == "start_coverage_cleaning" for t in tools[:returns[0]])):
            return "不得跳过或调换先清扫区域的步骤；覆盖完成后才返回本轮记录点。"
        if (re.search(r"视觉|伺服|饲服|(?:看看|搜索|检查|查看).{0,12}垃圾", after)
                and not any(t == "start_spot_cleaning" for t in tools[returns[-1] + 1:])):
            return "不得省略或提前执行返回后的视觉清扫；必须先完成返回导航，再调用 start_spot_cleaning。"
        return ""
    if ((reference or _UNSAVED_RETURN_POINT.search(value) or
         (_OTHER_SNAPSHOT.search(value) and "回来" in value)) and
            (plan.get("steps") or not plan.get("needs_clarification"))):
        return ("用户要求返回当前/刚才/说话时的位置，但本系统没有保存该瞬时点位。"
                "不得跳过这一步后执行后续动作；请返回 needs_clarification=true、steps=[]，"
                "保留完整 understood_sequence，并只询问该点的 map 坐标。原点则默认是地图原点。")
    return ""


def relative_targets(text):
    text = unicodedata.normalize("NFKC", text).lower()
    targets = []
    for clause in re.split(r"然后|接着|最后|再|[。;；]", text):
        values = [0.0, 0.0, 0.0]
        found = False
        patterns = [
            (0, 1, r"(?:向前|往前|前进)\s*(" + NUMBER + r")\s*(?:米|m\b)"),
            (0, -1, r"(?:向后|往后|后退|倒退)\s*(" + NUMBER + r")\s*(?:米|m\b)"),
            (1, 1, r"(?:向左|往左|左移)\s*(" + NUMBER + r")\s*(?:米|m\b)"),
            (1, -1, r"(?:向右|往右|右移)\s*(" + NUMBER + r")\s*(?:米|m\b)"),
            (2, 1, r"(?:左转|向左转|逆时针(?:旋转|转)?)\s*(" + NUMBER + r")\s*度"),
            (2, -1, r"(?:右转|向右转|(?<!逆)顺时针(?:旋转|转)?)\s*(" + NUMBER + r")\s*度"),
        ]
        for axis, sign, pattern in patterns:
            for match in re.finditer(pattern, clause):
                values[axis] += sign * number(match[1])
                found = True
        if found:
            targets.append(tuple(values))
    return targets


def navigation_error(plan, texts, anchor=None):
    """Require a complete matching point/offset from the current user context."""
    map_points = [point for text in texts for point in map_targets(text)]
    if valid_return_anchor(anchor) and any(current_return_reference(text) for text in texts):
        map_points.append(tuple(anchor[k] for k in ("x_m", "y_m", "yaw_deg")))
    offsets = [point for text in texts for point in relative_targets(text)]
    for step in plan["steps"]:
        tool = step["tool"]
        if tool not in ("navigate_map_pose", "navigate_relative"):
            continue
        args = step["arguments"]
        keys, candidates = (("x_m", "y_m", "yaw_deg"), map_points) if tool == "navigate_map_pose" else (
            ("forward_m", "left_m", "delta_yaw_deg"), offsets)
        if not any(all(math.isclose(float(args[k]), value, abs_tol=1e-6, rel_tol=0.0)
                       for k, value in zip(keys, candidate)) for candidate in candidates):
            return ("导航目标没有可核实的用户明确点位依据。区域名称不是导航点，禁止取区域中心、入口或猜测偏移。"
                    "到区域在那里清扫应只调用 start_coverage_cleaning，由覆盖系统负责入场。"
                    "独立导航请补充 map 的 x、y（朝向未指定时为0度），或明确方向和距离；"
                    "当前没有已登记命名点位服务，不能从区域名生成坐标。")
    return ""
