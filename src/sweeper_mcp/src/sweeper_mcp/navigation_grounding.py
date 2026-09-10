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
    if re.search(r"地图(?:坐标)?原点", text):
        targets.append((0.0, 0.0, 0.0))
    patterns = [
        r"(?<![a-z])x(?:_m)?\s*[:=为是]?\s*(" + NUMBER + r")\s*(?:米)?[\s,、]+y(?:_m)?\s*[:=为是]?\s*(" + NUMBER + r")",
        r"(?:地图(?:坐标|位置|位姿|点)?|map(?:坐标)?|坐标)\s*[:=为是到]?\s*\(?\s*(" + NUMBER + r")[\s,、]+(" + NUMBER + r")",
    ]
    for pattern in patterns:
        for match in re.finditer(pattern + NUMBER_END, text):
            targets.append((number(match[1]), number(match[2]), _yaw(text[match.end():])))
    return targets


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


def navigation_error(plan, texts):
    """Require a complete matching point/offset from the current user context."""
    map_points = [point for text in texts for point in map_targets(text)]
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
