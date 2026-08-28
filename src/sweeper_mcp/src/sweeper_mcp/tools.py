# -*- coding: utf-8 -*-
"""Strict MCP tool catalogue for the indoor J6M robot.

The cloud model chooses tools, but every argument is checked locally and every
mutating tool requires the private capability owned by the authorised Qt AI
session.  No tool writes ``/cmd_vel`` directly.
"""

import hmac
import math


class ToolResult:
    def __init__(self, text, is_error=False):
        self.text = text
        self.is_error = bool(is_error)

    def __repr__(self):
        return "ToolResult(is_error=%s, text=%s)" % (
            self.is_error, self.text[:60])


def _is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def validate_arguments(arguments, schema, path="arguments"):
    """Validate the JSON-schema subset used by this package."""
    expected = schema.get("type")
    if expected == "object":
        if not isinstance(arguments, dict):
            return "%s 必须是对象" % path
        properties = schema.get("properties", {})
        missing = [key for key in schema.get("required", [])
                   if key not in arguments]
        if missing:
            return "缺少必填参数: %s" % ", ".join(missing)
        if schema.get("additionalProperties") is False:
            extra = sorted(set(arguments) - set(properties))
            if extra:
                return "存在未定义参数: %s" % ", ".join(extra)
        for key, value in arguments.items():
            if key not in properties:
                continue
            error = validate_arguments(value, properties[key],
                                       "%s.%s" % (path, key))
            if error:
                return error
        return None
    if expected == "array":
        if not isinstance(arguments, list):
            return "%s 必须是数组" % path
        if len(arguments) < schema.get("minItems", 0):
            return "%s 数量不能少于 %d" % (path, schema["minItems"])
        if "maxItems" in schema and len(arguments) > schema["maxItems"]:
            return "%s 数量不能超过 %d" % (path, schema["maxItems"])
        if schema.get("uniqueItems"):
            try:
                unique = len(set(arguments)) == len(arguments)
            except TypeError:
                unique = False
            if not unique:
                return "%s 不能包含重复项" % path
        item_schema = schema.get("items", {})
        for index, item in enumerate(arguments):
            error = validate_arguments(item, item_schema,
                                       "%s[%d]" % (path, index))
            if error:
                return error
        return None
    if expected == "string":
        if not isinstance(arguments, str):
            return "%s 必须是字符串" % path
        if len(arguments.strip()) < schema.get("minLength", 0):
            return "%s 不能为空" % path
    elif expected == "boolean":
        if not isinstance(arguments, bool):
            return "%s 必须是布尔值" % path
    elif expected in ("number", "integer"):
        if not _is_number(arguments):
            return "%s 必须是数值" % path
        if expected == "integer" and int(arguments) != arguments:
            return "%s 必须是整数" % path
        if not math.isfinite(float(arguments)):
            return "%s 必须是有限数值" % path
        if "minimum" in schema and arguments < schema["minimum"]:
            return "%s 不能小于 %s" % (path, schema["minimum"])
        if "maximum" in schema and arguments > schema["maximum"]:
            return "%s 不能大于 %s" % (path, schema["maximum"])
    enum = schema.get("enum")
    if enum is not None and arguments not in enum:
        return "%s 必须是 %s 之一" % (path, ", ".join(map(str, enum)))
    return None


class Tool:
    def __init__(self, name, title, description, input_schema, handler,
                 mutating=False):
        self.name = name
        self.title = title
        self.description = description
        self.input_schema = input_schema
        self.handler = handler
        self.mutating = bool(mutating)

    def schema_dict(self):
        result = {
            "name": self.name,
            "description": self.description,
            "inputSchema": self.input_schema,
        }
        if self.title:
            result["title"] = self.title
        return result

    def run(self, arguments, authorised=False):
        error = validate_arguments(arguments, self.input_schema)
        if error:
            return ToolResult(error, True)
        if self.mutating and not authorised:
            return ToolResult(
                "AI 控制授权未生效；该 MCP 工具只允许当前 Qt 授权会话调用。",
                True,
            )
        try:
            result = self.handler(arguments)
        except Exception as exc:
            return ToolResult("工具执行异常: %s" % exc, True)
        return result if isinstance(result, ToolResult) else ToolResult(str(result))


class ToolRegistry:
    def __init__(self, control_token=""):
        self._tools = {}
        self._control_token = control_token or ""

    def register(self, tool):
        self._tools[tool.name] = tool

    def get(self, name):
        return self._tools.get(name)

    def schemas(self):
        return [tool.schema_dict() for tool in self._tools.values()]

    def is_authorised(self, token):
        return bool(self._control_token and token and hmac.compare_digest(
            self._control_token, token))


EMPTY_SCHEMA = {
    "type": "object", "properties": {}, "additionalProperties": False,
}


TOOL_SPECS = [
    {
        "name": "get_robot_status",
        "title": "查询机器人状态",
        "description": "查询电量、急停、定位、位姿、导航所有者和安全门；不改变机器人。",
        "inputSchema": EMPTY_SCHEMA,
    },
    {
        "name": "get_navigation_status",
        "title": "查询导航状态",
        "description": "查询 AI 当前关联 goal ID 的 move_base 状态；不以历史最新目标代替。",
        "inputSchema": EMPTY_SCHEMA,
    },
    {
        "name": "list_saved_coverage_regions",
        "title": "列出已保存清扫区域",
        "description": "列出与当前静态地图摘要严格匹配的已审核区域名称和 UUID。",
        "inputSchema": EMPTY_SCHEMA,
    },
    {
        "name": "get_coverage_status",
        "title": "查询覆盖清扫状态",
        "description": "查询 J6M 覆盖管理器状态、批次、区域和安全门。",
        "inputSchema": EMPTY_SCHEMA,
    },
    {
        "name": "get_visual_servo_status",
        "title": "查询视觉伺服状态",
        "description": "查询 FOD 模式仲裁与视觉伺服状态；定点清扫在本项目中即该功能。",
        "inputSchema": EMPTY_SCHEMA,
    },
    {
        "name": "navigate_relative",
        "title": "车体相对导航",
        "description": (
            "相对车辆当前朝向导航。forward_m 正值向前，left_m 正值向左，"
            "delta_yaw_deg 正值逆时针；仍由 move_base/TEB 和现有安全链执行。"
        ),
        "mutating": True,
        "inputSchema": {
            "type": "object",
            "properties": {
                "forward_m": {"type": "number", "minimum": -30.0, "maximum": 30.0},
                "left_m": {"type": "number", "minimum": -30.0, "maximum": 30.0},
                "delta_yaw_deg": {"type": "number", "minimum": -180.0, "maximum": 180.0},
            },
            "required": ["forward_m", "left_m", "delta_yaw_deg"],
            "additionalProperties": False,
        },
    },
    {
        "name": "navigate_map_pose",
        "title": "地图绝对位姿导航",
        "description": (
            "导航到任意有效的 map 绝对 x/y/yaw；只有三维 ICP LOCALIZED、"
            "锁存静态地图与覆盖管理器地图摘要一致，且目标位于空闲栅格时可用。"
        ),
        "mutating": True,
        "inputSchema": {
            "type": "object",
            "properties": {
                "x_m": {"type": "number"},
                "y_m": {"type": "number"},
                "yaw_deg": {"type": "number", "minimum": -180.0, "maximum": 180.0},
            },
            "required": ["x_m", "y_m", "yaw_deg"],
            "additionalProperties": False,
        },
    },
    {
        "name": "cancel_navigation",
        "title": "取消普通导航",
        "description": (
            "只显式取消当前 AI 会话持有的普通 move_base GoalID；"
            "不会取消 Qt 手工目标，也不会用取消全部目标破坏覆盖任务。"
        ),
        "mutating": True,
        "inputSchema": EMPTY_SCHEMA,
    },
    {
        "name": "start_spot_cleaning",
        "title": "启动视觉定点清扫",
        "description": (
            "请求启动现有 FOD 视觉伺服；原始 tools/call 只确认接管，"
            "AI 执行器随后轮询到完成。不控制主刷、边刷、风机或喷淋。"
        ),
        "mutating": True,
        "inputSchema": EMPTY_SCHEMA,
    },
    {
        "name": "stop_spot_cleaning",
        "title": "停止视觉定点清扫",
        "description": "退出 FOD 视觉伺服并恢复普通导航安全链。",
        "mutating": True,
        "inputSchema": EMPTY_SCHEMA,
    },
    {
        "name": "start_coverage_cleaning",
        "title": "启动已保存区域覆盖清扫",
        "description": (
            "按顺序清扫一个或多个 Qt 已保存区域。只能使用当前地图中的精确名称或 UUID，"
            "不能由模型临时生成多边形。"
        ),
        "mutating": True,
        "inputSchema": {
            "type": "object",
            "properties": {
                "regions": {
                    "type": "array", "items": {"type": "string", "minLength": 1},
                    "minItems": 1, "maxItems": 20, "uniqueItems": True,
                },
                "operation_width_m": {
                    "type": "number", "minimum": 0.30, "maximum": 3.00,
                    "default": 1.0,
                },
                "overlap_percent": {
                    "type": "number", "minimum": 0.0, "maximum": 50.0,
                    "default": 15.0,
                },
                "max_speed_mps": {
                    "type": "number", "minimum": 0.10, "maximum": 1.60,
                    "default": 0.8,
                },
                "allow_reverse_transit": {
                    "type": "boolean", "default": True,
                },
                "reverse_speed_mps": {
                    "type": "number", "minimum": 0.05, "maximum": 0.80,
                    "default": 0.3,
                },
                "max_angular_speed_rps": {
                    "type": "number", "minimum": 0.10, "maximum": 1.00,
                    "default": 0.6,
                },
                "linear_accel_mps2": {
                    "type": "number", "minimum": 0.10, "maximum": 2.00,
                    "default": 2.0,
                },
                "angular_accel_rps2": {
                    "type": "number", "minimum": 0.10, "maximum": 1.00,
                    "default": 0.5,
                },
                "direction_change_penalty_sec": {
                    "type": "number", "minimum": 0.0, "maximum": 30.0,
                    "default": 1.0,
                },
                "segment_handoff_penalty_sec": {
                    "type": "number", "minimum": 0.0, "maximum": 30.0,
                    "default": 0.5,
                },
            },
            "required": ["regions"],
            "additionalProperties": False,
        },
    },
    {
        "name": "pause_coverage",
        "title": "暂停覆盖清扫",
        "description": "暂停当前覆盖任务并保持权威覆盖状态。",
        "mutating": True,
        "inputSchema": EMPTY_SCHEMA,
    },
    {
        "name": "resume_coverage",
        "title": "恢复覆盖清扫",
        "description": "在安全门重新核验后恢复当前覆盖任务。",
        "mutating": True,
        "inputSchema": EMPTY_SCHEMA,
    },
    {
        "name": "skip_coverage_region",
        "title": "跳过当前覆盖区域",
        "description": "跳过批次中的当前区域并进入下一个已审核区域。",
        "mutating": True,
        "inputSchema": EMPTY_SCHEMA,
    },
    {
        "name": "cancel_coverage",
        "title": "取消覆盖清扫",
        "description": "取消整个当前覆盖任务或区域队列。",
        "mutating": True,
        "inputSchema": EMPTY_SCHEMA,
    },
]


def build_registry(backend, control_token=""):
    registry = ToolRegistry(control_token=control_token)
    for spec in TOOL_SPECS:
        name = spec["name"]

        def _make(name_):
            return lambda args: getattr(backend, name_)(**args)

        registry.register(Tool(
            name=name,
            title=spec.get("title", ""),
            description=spec["description"],
            input_schema=spec["inputSchema"],
            handler=_make(name),
            mutating=spec.get("mutating", False),
        ))
    return registry
