# -*- coding: utf-8 -*-
"""工具注册表 —— 把清扫车操作定义成 MCP 工具（schema 全中文描述给 LLM 看）。

工具与后端解耦：build_registry(backend) 把每个工具绑定到 backend 上同名方法。
backend（mock / ros）只需实现同名 handler 方法并返回 ToolResult 即可接入。

约定：
- tools/call 执行失败返回 ToolResult(is_error=True)，不抛异常（协议层转 isError）。
- 必填参数缺失在工具层校验，返回 is_error=True 的提示文本。
"""


class ToolResult:
    """工具执行结果。is_error=True 表示执行失败（对应 MCP result.isError）。"""

    def __init__(self, text, is_error=False):
        self.text = text
        self.is_error = is_error

    def __repr__(self):
        return "ToolResult(is_error=%s, text=%s)" % (self.is_error, self.text[:60])


class Tool:
    """一个 MCP 工具：name/description/inputSchema + handler。"""

    def __init__(self, name, title, description, input_schema, handler):
        self.name = name
        self.title = title
        self.description = description
        self.input_schema = input_schema
        self.handler = handler

    def schema_dict(self):
        """tools/list 返回的 schema（跨版本兼容：name/description/inputSchema 三件套）。"""
        d = {"name": self.name,
             "description": self.description,
             "inputSchema": self.input_schema}
        if self.title:
            d["title"] = self.title
        return d

    def run(self, arguments):
        """执行工具。参数校验失败或 handler 抛异常 → 返回 is_error 的 ToolResult。"""
        err = _validate_required(arguments, self.input_schema)
        if err:
            return ToolResult(err, True)
        try:
            res = self.handler(arguments)
        except Exception as exc:
            return ToolResult("工具执行异常: %s" % exc, True)
        return res if isinstance(res, ToolResult) else ToolResult(str(res))


class ToolRegistry:
    def __init__(self):
        self._tools = {}

    def register(self, tool):
        self._tools[tool.name] = tool

    def get(self, name):
        return self._tools.get(name)

    def schemas(self):
        return [t.schema_dict() for t in self._tools.values()]


def _validate_required(arguments, input_schema):
    """校验必填参数，缺失返回错误提示文本，否则 None。"""
    required = input_schema.get("required", [])
    missing = [k for k in required if k not in arguments]
    if missing:
        return "缺少必填参数: %s" % ", ".join(missing)
    return None


# ---------------- 工具定义 ----------------

TOOL_SPECS = [
    {
        "name": "get_robot_status",
        "title": "查询机器人状态",
        "description": (
            "查询清扫车当前状态：电量百分比、急停标志(硬/软/手柄/机器人)、当前位姿"
            "(本地坐标 x/y 米 + 朝向 yaw 弧度)、清扫装置开关状态、当前导航/回收模式。"
            "无参数。用于回答'车在哪/还有多少电/清扫开没开'，或在执行动作前先摸底。"
        ),
        "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False},
    },
    {
        "name": "navigate_pose",
        "title": "导航到本地坐标",
        "description": (
            "发布本地坐标目标到 /move_base_simple/goal，让底盘移动到指定位置。"
            "x/y 是本地坐标系(camera_init)下的坐标(米)，yaw 是目标朝向(弧度 0~2π，"
            "将转换为标准单位四元数写入 orientation)。本工具异步发布、发布即返回，不等待到达；"
            "是否到达请用 navigation_status 轮询。仅当你确切知道本地坐标时才用；"
            "用户说'往前走X米'等相对指令请用 navigate_relative；经纬度目标用 navigate_gps。"
        ),
        "inputSchema": {
            "type": "object",
            "properties": {
                "x": {"type": "number", "description": "本地坐标 x(米)"},
                "y": {"type": "number", "description": "本地坐标 y(米)"},
                "yaw": {"type": "number", "description": "目标朝向弧度(0~2π)，默认 0"},
                "frame_id": {"type": "string", "description": "坐标系，默认 camera_init"},
            },
            "required": ["x", "y"],
            "additionalProperties": False,
        },
    },
    {
        "name": "navigate_relative",
        "title": "按相对位移导航",
        "description": (
            "让底盘相对当前位姿移动 dx/dy 米并旋转 dyaw 弧度。dx=前后位移(正=前进)，"
            "dy=左右位移(正=向左)，dyaw=旋转弧度(正=逆时针)。工具会读取机器人当前位姿，"
            "自动换算成目标坐标后发布到 /move_base_simple/goal，异步执行。"
            "用于'往前走10米/后退/左移/左转90度'等相对指令。"
        ),
        "inputSchema": {
            "type": "object",
            "properties": {
                "dx": {"type": "number", "description": "前后位移(米)，正=前进，默认 0"},
                "dy": {"type": "number", "description": "左右位移(米)，正=向左，默认 0"},
                "dyaw": {"type": "number", "description": "旋转弧度，正=逆时针，默认 0"},
            },
            "required": [],
            "additionalProperties": False,
        },
    },
    {
        "name": "navigate_gps",
        "title": "导航到 GPS 经纬度",
        "description": (
            "发布 GPS 经纬度目标到 /gps/goal_fix，由 gps_goal_node 转成 move_base 目标。"
            "latitude/longitude 为 WGS84 度，altitude 可选(米)。异步发布即返回。"
            "用于'去某个经纬度'。"
        ),
        "inputSchema": {
            "type": "object",
            "properties": {
                "latitude": {"type": "number", "description": "纬度(度，WGS84)"},
                "longitude": {"type": "number", "description": "经度(度，WGS84)"},
                "altitude": {"type": "number", "description": "海拔(米)，可选"},
            },
            "required": ["latitude", "longitude"],
            "additionalProperties": False,
        },
    },
    {
        "name": "cancel_navigation",
        "title": "取消导航",
        "description": (
            "取消当前导航目标，底盘就地减速停止。用于'停下/别去了/取消'。"
        ),
        "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False},
    },
    {
        "name": "navigation_status",
        "title": "查询导航状态",
        "description": (
            "查询当前导航目标的状态，返回 idle(无目标)/active(执行中)/succeeded(已到达)"
            "/preempted/aborted/canceled。用于回答'到了吗/走到哪了'。"
        ),
        "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False},
    },
    {
        "name": "emergency_stop",
        "title": "急停",
        "description": (
            "触发或解除急停。active=true 立即急停(最高优先级，打断一切动作)，"
            "active=false 解除急停。检测到碰撞/危险时优先调用。"
        ),
        "inputSchema": {
            "type": "object",
            "properties": {
                "active": {"type": "boolean", "description": "true=急停，false=解除"},
                "reason": {"type": "string", "description": "急停原因说明，可选"},
            },
            "required": ["active"],
            "additionalProperties": False,
        },
    },
    {
        "name": "sweep_set",
        "title": "清扫装置开关",
        "description": (
            "控制清扫装置(滚刷/吸尘)开关。action 取值 on/off/toggle。"
            "装置通过 TCP 连接，掉线时返回失败。用于'打开清扫/关闭清扫'。"
            "注意：这是当前唯一已实现的清扫能力(定点清扫)；全覆盖/循迹清扫请用 sweep_coverage(未实现)。"
        ),
        "inputSchema": {
            "type": "object",
            "properties": {
                "action": {
                    "type": "string",
                    "enum": ["on", "off", "toggle"],
                    "description": "on=开启，off=关闭，toggle=切换",
                },
            },
            "required": ["action"],
            "additionalProperties": False,
        },
    },
    {
        "name": "sweep_coverage",
        "title": "全覆盖清扫（预留接口，尚未实现）",
        "description": (
            "进行全覆盖/按区域/按路线清扫（例如'把整条主路都扫一遍'、'全覆盖清扫A区'）。"
            "【当前尚未实现】：调用会返回未实现提示。此工具为后续版本预留的接口，"
            "参数 schema 已按计划定义(area/pattern/duration/width)，实现后无需改动工具定义。"
            "在未实现前，请勿主动调用它；用户要求'清扫'默认用 sweep_set 打开/关闭清扫装置即可。"
        ),
        "inputSchema": {
            "type": "object",
            "properties": {
                "area": {
                    "type": "string",
                    "enum": ["spot", "area", "route"],
                    "description": "清扫范围：spot=定点，area=区域，route=沿路线（预留）",
                },
                "pattern": {
                    "type": "string",
                    "enum": ["spiral", "zigzag"],
                    "description": "全覆盖路径模式：螺旋/弓字（预留）",
                },
                "duration": {"type": "number", "description": "清扫时长(秒)，预留"},
                "width": {"type": "number", "description": "清扫宽度(米)，预留"},
            },
            "required": [],
            "additionalProperties": False,
        },
    },
    {
        "name": "set_fod_mode",
        "title": "切换 GPS↔FOD 回收模式",
        "description": (
            "切换机器人导航/回收模式。enabled=true 切换到 FOD 视觉回收模式，"
            "false 切回 GPS 模式。需要 FOD 模式管理器在线（后补功能，未启用时返回失败）。"
        ),
        "inputSchema": {
            "type": "object",
            "properties": {
                "enabled": {"type": "boolean",
                            "description": "true=FOD 视觉回收模式，false=GPS 模式"},
            },
            "required": ["enabled"],
            "additionalProperties": False,
        },
    },
]


def build_registry(backend):
    """根据后端实例构建工具注册表（每个工具绑定 backend 同名方法）。"""
    registry = ToolRegistry()
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
        ))
    return registry
