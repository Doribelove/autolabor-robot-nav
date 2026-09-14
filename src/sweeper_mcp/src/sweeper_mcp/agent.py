# -*- coding: utf-8 -*-
"""Authorised plan-first cloud agent with sequential MCP execution.

There is no second robot behaviour state machine here.  The agent validates a
complete ordered plan, dispatches one existing subsystem command at a time and
waits for that subsystem's terminal result before considering the next step.
"""

import json
import logging
import re
import threading
import time

from sweeper_mcp.llm import OpenAIClient
from sweeper_mcp.mcp_client import MCPClient
from sweeper_mcp.navigation_grounding import (navigation_error, unresolved_reference_error,
    current_return_reference, valid_return_anchor, matches_return_anchor)
from sweeper_mcp.region_catalog import (compact_catalog, coverage_region_ambiguity_error,
                                        coverage_region_error, reference_map)
from sweeper_mcp.tools import TOOL_SPECS, validate_arguments
from sweeper_mcp.visual_completion import session_id_valid, sequence_valid, visual_completed


logger = logging.getLogger(__name__)


class AgentError(Exception):
    pass


class AgentCancelled(AgentError):
    pass


SYSTEM_PROMPT = """你是室内无人清扫车的控制规划器。你必须把用户输入一次性拆成完整、按顺序执行的 MCP 工具计划。

约束：
1. 不使用 GPS、经纬度或 WGS84。本项目只有车体相对导航和 map 绝对位姿导航。
2. “定点清扫”表示现有 FOD 视觉伺服，不代表主刷、边刷、风机或喷淋。
   “看看/检查/搜索周围有没有垃圾，有垃圾就清扫”“发现垃圾就清理”等说法也是启动一次
   FOD 视觉伺服的自然表达，必须直接调用一次 start_spot_cleaning。这里的“有垃圾就”描述
   视觉能力自身的目标发现与接管语义，不是需要大模型另行实现的条件分支，也不是无条件
   执行覆盖清扫；视觉后端没有满足接管条件的新鲜目标时会自行拒绝/结束。
3. 区域覆盖只能引用系统提供的已保存区域名称或 UUID，禁止生成临时多边形。
4. 参数不明确时返回空 steps 并在 answer 中要求用户补充，禁止编造坐标、区域或距离。
   “先去区域清扫”没有指定区域；没有本轮或待补充上下文中的地点依据时必须询问哪一区域，
   不能默认选目录第一项，也不能从可信返回点反推出清扫区域。
5. 连续语句按独立任务拆分并保留先后顺序，不按动词数量机械拆分。高层任务内部的入场不另拆导航。
6. 只能调用 submit_plan 一次。不要在 answer 中声称动作已经完成。
7. “原点、零点”默认就是地图原点，和“地图原点、地图坐标原点、地图上面的原点、
   地图零点”一样明确表示 map 位姿 x=0、y=0、yaw=0；“起点、基地、
   充电点”等名称不等同于地图原点，系统未提供精确坐标时必须要求用户补充。
8. 输入可能由用户自然停顿后的多个片段合并；连词、指代和“不是X，是Y”应结合本轮完整表达理解。
9. 历史对话仅用于补齐待答问题、解析指代。历史状态为 PREVIEW/SUCCEEDED/FAILED 的任务
   不得重新执行或声称已完成。当前明确的新指令优先，不能擅自把上一轮任务附加进来。
10. 缺少必要信息或整条指令包含当前工具无法表达的动作/条件分支时，返回
    needs_clarification=true、steps=[]，在 understood_sequence 中逐项保留完整动作顺序
    （包括暂不支持的动作），在 answer 说明具体缺项并提出补充问题。
    任意条件不得改成无条件动作，也不得悄悄省略中间动作后执行其他动作；但第2条所述
    “检测到垃圾就由视觉伺服清扫”是 start_spot_cleaning 已封装的能力，不属于缺失条件分支。
    用户只有在正在回答明确问题时，“是/好的/某区域”等短语才可用于补充，不代表开启控制授权。
11. “到某区域去，在那里清扫一下”“去A区然后清扫那里”表示区域覆盖任务：
    只调用 start_coverage_cleaning(regions=[对应已保存区域])。覆盖系统自己规划从当前位置到入口，
    不能先 navigate_map_pose/navigate_relative，再清扫，也不能用 start_spot_cleaning 替代区域覆盖。
    口语简称在上下文唯一明确时映射为目录中的精确名称/UUID；有多个候选或无匹配时要求补充。
12. 单独“到前门去”“去A区”等区域目标不是导航点；无明确点位时询问坐标，不能自行变成清扫。
    本系统没有命名导航点目录，清扫区域目录也不提供导航坐标，禁止从名字、中心、入口或机器人状态猜坐标。
    map 导航接受用户明确的 x/y、地图原点或下述本轮可信返回点；普通数字点未指定朝向时 yaw_deg=0。
    相对导航必须明确方向和距离/转角。“往前一点”“去附近”不满足条件。
    如果用户明确要求“先到地图坐标(1,2)，再清扫A区”，才保留独立导航和清扫两个任务。
13. 当前区域目录是每次规划时动态查询的，不得记忆或写死示例区域。用户说出的区域可以不与
    目录逐字一致：应结合地点主体、方位及其自然同义表达、通用建筑词/后缀的省略或替换、
    上下文指代，在本轮完整目录中做语义候选匹配。推理示例：如果本轮目录新增
    “研究院后大厅”，且没有其他后部候选，“到研究院后面清扫一下”可匹配该区域；
    这只是可替换的示例，不是固定区域、固定短语或别名表。
    选择前必须枚举目录中所有共享同一地点主体和方位的候选。如果用户省略了用于区分候选的
    功能/部位词（如门、门厅、大厅、走廊、房间、广场），则这些候选都仍然合理；不能偏爱
    名称更短、排列更早或更像示例的一项，也不能声称它是唯一候选。
    匹配优先级为：地点主体和用户明确说出的方位是硬约束，功能/部位词是软约束。
    名称中带有与用户明确方位冲突的候选必须排除；不能因为都含“门/大厅”等通用词而忽略
    前、后、左、右、东、西、南、北或侧向等方位差异。
    “区域、片区、那块、附近”只是范围泛称，不用于否定候选。“门、门口、入口、门前、
    门区域”常指对应入口一带；“厅、大厅、前厅、门厅、入口大厅、门的大厅”常指同一入口
    连通的室内区域。地点主体和方位一致、且只有一个候选时，这两组部位说法应视为可兼容的
    口语表达，必须直接选唯一候选，不得仅因“门”和“大厅/门厅”没有逐字相同而要求确认。
    通用例子：目录只有“某建筑北大厅”时，“某建筑北门区域”应匹配它；目录只有“某建筑
    东侧门厅”时，“某建筑东边门的大厅”应匹配它。例子中的建筑和方位均可替换。
    只有同一地点主体和方位下确有两个以上合理区域，或用户用“不是大厅、只到门外”等明确
    排除/边界表达时，功能/部位差异才构成歧义。如果答复中已经判断某项是唯一符合主体和
    方位的候选，就不得再为名称表面差异索要确认。
    只有一个明显合理候选时，regions 必须填写该候选在目录中的原始精确名称或 UUID；
    若有两个以上近似候选、地点主体/方位冲突或信心不足，必须要求用户从候选中确认。
    禁止把用户的近似说法直接填入 regions，也禁止因目录变化沿用上一轮候选。
14. 即使整条长指令因其他不支持的动作或条件分支而必须返回空 steps，也要在
    understood_sequence 中把上述唯一区域匹配写成目录原名；answer 只能询问真正缺失或不支持
    的部分，不能再次询问已经唯一匹配的区域。
15. “当前/现在的位置、现在这个点、说话时的位置、这里、原位”在返回指令中指本轮接收
    指令时记录的机器人位置，不是执行到返回步骤时再查询的位置，也不是地图原点。
    系统若提供本轮可信返回点，必须原样使用其 x_m/y_m/yaw_deg（保留原车头朝向），
    不得猜测或要求用户再次报坐标；没有可信快照才询问，且整单不执行。“我现在的位置”
    指人的位置，不等同于机器人位置；未保存的“刚才位置”也不能用新位置冒充。
    按指代语义理解，不要求固定词序：“原地/现在这儿/这个位置/机器人目前停着的位置”
    以及“记住现在的位置，完成后回来”都引用本轮机器人起始位姿。系统可以为每个新请求
    提供一次只读起始位姿，但只有用户要求返回该位置时才可用它作导航目标；不能将它当成
    任意区域、基地、人的位置或未记录历史点。唯一匹配的区域不因返回点问题再次追问。
16. 连续独立动作必须逐项保留：例如“先去某个已保存区域清扫，再回到现在的位置，
    进行视觉清扫”应为 start_coverage_cleaning → navigate_map_pose(本轮返回点)
    → start_spot_cleaning，不能把最后两项吞掉或合并成覆盖清扫。区域仅是动态目录示例。
    本地会等待覆盖任务完整结束并释放导航所有权，再执行返回导航；同一目标成功后才
    开启视觉伺服。不要另外添加开始前的区域导航，也不要在返回途中开启视觉。
"""

NAV_TOOLS = {"navigate_relative", "navigate_map_pose"}
COVERAGE_START_TOOLS = {"start_coverage_cleaning"}
VISUAL_START_TOOLS = {"start_spot_cleaning"}


def _spec_map():
    return {item["name"]: item for item in TOOL_SPECS}


def _tool_description(tools):
    lines = []
    for item in tools:
        lines.append("- %s: %s；参数=%s" % (
            item["name"], item.get("description", ""),
            json.dumps(item.get("inputSchema", {}), ensure_ascii=False),
        ))
    return "\n".join(lines)


def _plan_tool(tool_names):
    return {
        "type": "function",
        "function": {
            "name": "submit_plan",
            "description": "提交完整、有序、尚未执行的 MCP 工具调用计划。",
            "parameters": {
                "type": "object",
                "properties": {
                    "steps": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "tool": {"type": "string", "enum": tool_names},
                                "arguments": {"type": "object"},
                                "description": {"type": "string"},
                            },
                            "required": ["tool", "arguments", "description"],
                            "additionalProperties": False,
                        },
                    },
                    "answer": {"type": "string"},
                    "needs_clarification": {"type": "boolean"},
                    "understood_sequence": {"type": "array", "items": {"type": "string"}},
                },
                "required": ["steps"],
                "additionalProperties": False,
            },
        },
    }


def _sanitize_text(text, limit=1500):
    value = str(text or "")
    value = re.sub(r"(?i)(authorization\s*[:=]\s*bearer\s+)\S+", r"\1[REDACTED]", value)
    value = re.sub(r"\bsk-[A-Za-z0-9_-]{8,}\b", "[REDACTED]", value)
    value = re.sub(r"(/home|/map)/[^\s\"']+", "[LOCAL_PATH]", value)
    return value[:limit]


def _remove_json_trailing_commas(text):
    """Remove only commas immediately before ]/}, never commas in strings."""
    output = []
    in_string = False
    escaped = False
    index = 0
    while index < len(text):
        char = text[index]
        if in_string:
            output.append(char)
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                in_string = False
            index += 1
            continue
        if char == '"':
            in_string = True
            output.append(char)
            index += 1
            continue
        if char == ",":
            lookahead = index + 1
            while lookahead < len(text) and text[lookahead].isspace():
                lookahead += 1
            if lookahead < len(text) and text[lookahead] in "]}":
                index += 1
                continue
        output.append(char)
        index += 1
    return "".join(output)


def _single_json_object_text(text):
    """Extract one balanced top-level object with only textual wrappers."""
    start = text.find("{")
    if start < 0:
        return ""
    depth = 0
    in_string = False
    escaped = False
    end = -1
    for index in range(start, len(text)):
        char = text[index]
        if in_string:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                in_string = False
            continue
        if char == '"':
            in_string = True
        elif char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                end = index + 1
                break
            if depth < 0:
                return ""
    if end < 0 or in_string:
        return ""
    wrappers = text[:start] + text[end:]
    # Never discard another possible JSON value while repairing a plan.
    if any(char in wrappers for char in "{}[]"):
        return ""
    return text[start:end]


def _decode_plan_arguments(arguments):
    """Decode bounded, common tool-call JSON deviations.

    Every accepted result still passes the exact plan schema and per-tool
    argument validation below.  This deliberately does not repair missing
    braces, quotes, values or other ambiguous content.
    """
    if isinstance(arguments, dict):
        return arguments, "object"
    if not isinstance(arguments, str):
        raise ValueError("arguments is not a string or object")
    raw = arguments.strip().lstrip("\ufeff")
    if not raw:
        raise ValueError("arguments is empty")

    candidates = [(raw, "strict")]
    fenced = re.match(
        r"\A```(?:json)?\s*(.*?)\s*```\Z", raw,
        flags=re.IGNORECASE | re.DOTALL)
    if fenced:
        candidates.append((fenced.group(1).strip(), "code_fence"))
    extracted = _single_json_object_text(raw)
    if extracted and extracted != raw:
        candidates.append((extracted, "text_wrapper"))

    tried = set()
    last_error = None
    for candidate, repair in candidates:
        variants = [(candidate, repair)]
        without_trailing = _remove_json_trailing_commas(candidate)
        if without_trailing != candidate:
            variants.append((without_trailing, repair + "+trailing_comma"))
        for value, variant in variants:
            if value in tried:
                continue
            tried.add(value)
            try:
                document = json.loads(value)
            except ValueError as exc:
                last_error = exc
                try:
                    document = json.loads(value, strict=False)
                except ValueError as relaxed_exc:
                    last_error = relaxed_exc
                    continue
                variant += "+literal_control"
            if isinstance(document, str):
                nested = document.strip()
                if nested == value or nested in tried:
                    continue
                tried.add(nested)
                try:
                    document = json.loads(nested)
                except ValueError as exc:
                    last_error = exc
                    continue
                variant += "+double_encoded"
            return document, variant
    raise ValueError(str(last_error or "no complete JSON object"))


class AgentRunner:
    def __init__(self, base_url="https://api.deepseek.com",
                 model="deepseek-v4-flash", api_key=None, temperature=0.2,
                 timeout_s=20.0, max_retries=3, backend="ros",
                 control_token="", max_plan_steps=8, nav_wait_timeout=240.0,
                 navigation_cleanup_timeout=30.0,
                 coverage_wait_timeout=3600.0, visual_wait_timeout=300.0,
                 poll_interval=0.5, tool_text_limit=1500,
                 region_catalog_limit=6000, llm=None, mcp=None,
                 event_callback=None, authorization_checker=None,
                 cloud_authorization_checker=None, **_ignored):
        self.llm = llm or OpenAIClient(
            base_url=base_url, model=model, api_key=api_key,
            temperature=temperature, timeout_s=timeout_s,
            max_retries=max_retries,
        )
        self._owns_mcp = mcp is None
        self.mcp = mcp or MCPClient(
            server_env_backend=backend, control_token=control_token)
        self.control_token = control_token or ""
        self.max_plan_steps = int(max_plan_steps)
        if self.max_plan_steps < 1 or self.max_plan_steps > 32:
            raise ValueError("max_plan_steps must be between 1 and 32")
        self.nav_wait_timeout = float(nav_wait_timeout)
        self.navigation_cleanup_timeout = float(navigation_cleanup_timeout)
        self.coverage_wait_timeout = float(coverage_wait_timeout)
        self.visual_wait_timeout = float(visual_wait_timeout)
        self.poll_interval = float(poll_interval)
        self.tool_text_limit = int(tool_text_limit)
        self.region_catalog_limit = int(region_catalog_limit)
        if self.region_catalog_limit < 512 or self.region_catalog_limit > 50000:
            raise ValueError("region_catalog_limit must be between 512 and 50000")
        self.event_callback = event_callback
        self.authorization_checker = authorization_checker
        self.cloud_authorization_checker = cloud_authorization_checker
        self._cancel = threading.Event()
        self._run_lock = threading.Lock()
        self._context_lock = threading.RLock()
        self._history = []
        self._context_generation = 0
        self._region_context = None
        self._region_references = {}
        self._active_tool = ""
        self._tools = None
        self._specs = _spec_map()
        try:
            self.mcp.initialize()
        except Exception:
            if self._owns_mcp:
                self.mcp.close()
            raise

    def close(self):
        if self._owns_mcp:
            self.mcp.close()

    def _emit(self, kind, **fields):
        if self.event_callback:
            event = {"kind": kind}
            event.update(fields)
            try:
                self.event_callback(event)
            except Exception:
                # Qt/event telemetry must never tear down an in-flight safety
                # cleanup.  The robot state is authoritative; UI delivery is
                # best effort while an exact GoalID is being settled.
                pass

    def _cloud_event(self, phase):
        metrics = getattr(self.llm, "last_metrics", {}) or {}
        self._emit(
            "CLOUD", state=phase,
            duration_ms=float(metrics.get("rtt_ms", 0.0)),
            result_json=json.dumps({
                "http_status": metrics.get("http_status", 0),
                "request_id": metrics.get("request_id", ""),
                "usage": metrics.get("usage", {}),
            }, ensure_ascii=False),
        )

    def list_tools(self):
        if self._tools is None:
            response = self.mcp.list_tools()
            self._tools = response.get("tools", [])
        return list(self._tools)

    def _context(self):
        try:
            result = self.mcp.call_tool("list_saved_coverage_regions", {})
            if result.get("is_error"):
                raise RuntimeError(str(result.get("text", "当前已保存区域不可用")))
            catalog = str(result.get("text", ""))
            rendered = compact_catalog(catalog, self.region_catalog_limit)
            references = reference_map(catalog) if rendered else {}
            with self._context_lock:
                if self._region_context is not None and catalog != self._region_context:
                    self.reset_context()
                self._region_context = catalog
                self._region_references = references
            if not rendered:
                return ("当前动态区域目录为空、格式无效或超过完整展示上限；"
                        "不得规划区域清扫，必须要求用户检查区域配置。")
            return ("本轮动态区域目录（用户口语可近似表达；计划参数只能填写下列原始精确"
                    "名称或 UUID）：%s" % rendered)
        except Exception as exc:
            with self._context_lock:
                if self._region_context is not None:
                    self.reset_context()
                self._region_context = None
                self._region_references = {}
            return "当前已保存区域查询失败：%s" % exc

    def plan(self, text):
        self._check_cancelled()
        if not self._cloud_authorised():
            raise AgentCancelled("AI parse authorization was revoked")
        tools = self.list_tools()
        names = [item["name"] for item in tools]
        # Capture before the cloud call or any mutating step. Never resolve
        # "now" when navigation finally starts after a long coverage mission.
        anchor, anchor_error = None, ""
        # Snapshot availability must not depend on a closed list of phrases.
        # Only an unresolved reply may inherit the original request's snapshot.
        with self._context_lock:
            recent = [row for row in self._history if time.monotonic() - row["at"] < 60.0]
            if recent and recent[-1]["state"] == "NEEDS_CLARIFICATION" and not current_return_reference(text):
                anchor = recent[-1].get("return_anchor")
        if anchor is None and not re.fullmatch(r"\s*(?:停止|停|取消|暂停|停止清扫|取消任务)[。！!\s]*", text):
            try:
                snapshot = self.mcp.call_tool("get_robot_status", {"capture_map_pose": True})
                data = json.loads(snapshot.get("text", "{}"))
                if not snapshot.get("is_error") and valid_return_anchor(data.get("map_pose")):
                    anchor = dict(data["map_pose"])
                else:
                    anchor_error = str(data.get("map_pose_error", "定位未就绪，未记录本轮返回点"))
            except Exception as exc:
                anchor_error = str(exc)
        context = self._context()
        messages = []
        with self._context_lock:
            self._history = [row for row in self._history if time.monotonic() - row["at"] < 60.0][-3:]
            if not current_return_reference(text) and self._history and self._history[-1]["state"] == "NEEDS_CLARIFICATION":
                anchor = self._history[-1].get("return_anchor")
            for row in self._history:
                messages.extend([
                    {"role": "user", "content": row["text"]},
                    {"role": "assistant", "content": row["reply"]},
                ])
            region_references = dict(self._region_references)
        if anchor:
            catalog = self._decode(self._region_context or "")
            if catalog.get("map_digest") != anchor["map_digest"]:
                anchor, anchor_error = None, "记录返回点后地图已变更或当前地图目录不可用"
        anchor_context = ("本轮可信返回点（来自本机已定位 map->base_link，不是模型猜测）：" +
                          json.dumps(anchor, ensure_ascii=False)) if anchor else (
                              "本轮未记录可信返回点：" + (anchor_error or "起始位姿不可用"))
        if ((not anchor or not current_return_reference(text)) and
                unresolved_reference_error({"steps": [], "needs_clarification": False}, text)):
            anchor_context += ("\n本轮硬性预检结论：用户要求的返回位置没有可信记录。submit_plan 必须"
                               "严格填写 needs_clarification=true、steps=[]、非空 answer；所有拟议动作"
                               "只能放入 understood_sequence，绝不能同时填写覆盖或视觉等任何 steps。"
                               "即使上文提供了机器人位姿，也不能冒充人的位置或未保存的历史位置。")
        prompt = "%s\n%s\n%s\n\n可用 MCP 工具：\n%s" % (
            SYSTEM_PROMPT, context, anchor_context, _tool_description(tools))
        messages.insert(0, {"role": "system", "content": prompt})
        messages.append({"role": "user", "content": text})
        # Only unresolved clarification may contribute earlier user evidence.
        # Never ground a new point using an executed/previewed plan or a model reply.
        evidence = [text]
        with self._context_lock:
            for row in reversed(self._history):
                if row["state"] != "NEEDS_CLARIFICATION":
                    break
                evidence.insert(0, row["text"])
        planning_tool = _plan_tool(names)
        for attempt in range(2):
            plan = None
            response = self.llm.chat(
                messages, tools=[planning_tool],
                tool_choice={"type": "function", "function": {"name": "submit_plan"}},
            )
            self._check_cancelled()
            self._cloud_event("PLAN" if attempt == 0 else "PLAN_FORMAT_RETRY")
            try:
                plan = self._validate_plan_response(response)
                error = coverage_region_error(plan, region_references)
                if not error:
                    error = coverage_region_ambiguity_error(
                        plan, "\n".join(evidence), self._region_context)
                if not error:
                    error = unresolved_reference_error(plan, "\n".join(evidence), anchor)
                if not error:
                    error = navigation_error(plan, evidence, anchor)
                if error:
                    raise AgentError(error)
                for step in plan["steps"]:
                    supplied_digest = step["arguments"].get("expected_map_digest")
                    if supplied_digest and (not anchor or supplied_digest != anchor["map_digest"]):
                        raise AgentError("云端不得自行生成返回点的地图摘要。")
                    if anchor and matches_return_anchor(step, anchor):
                        step["arguments"]["expected_map_digest"] = anchor["map_digest"]
                if anchor and (any(current_return_reference(item) for item in evidence) or plan.get("needs_clarification")):
                    plan["return_anchor"] = dict(anchor)
                return plan
            except AgentError as exc:
                self._emit("PLAN_REJECTED", state="REJECTED", description=str(exc),
                           result_json=json.dumps({"candidate_plan": plan,
                                                   "cloud_response": response}, ensure_ascii=False))
                if attempt or not self._cloud_authorised():
                    raise
                # Retry only planning, before any tool execution.
                correction = (
                    "上次本地整计划校验失败：%s 请根据最初的用户指令，从头重新提交一次"
                    "完整合法的 submit_plan；不要解释、不要 Markdown、不要代码围栏或注释，"
                    "arguments 必须是使用双引号且无结尾多余逗号的单个严格 JSON 对象。"
                    "需要补充时 steps 必须为空，动作理解放 understood_sequence。" % exc)
                messages.append({"role": "user", "content": correction})

    def _validate_plan_response(self, response):
        calls = response.get("tool_calls") or []
        matching = [call for call in calls if call.get("function", {}).get(
            "name") == "submit_plan"]
        if len(matching) != 1:
            raise AgentError("云端未返回唯一的 submit_plan，已拒绝执行。")
        raw_arguments = matching[0]["function"].get("arguments")
        try:
            document, repair = _decode_plan_arguments(raw_arguments)
        except (TypeError, ValueError) as exc:
            logger.warning(
                "Rejected malformed submit_plan arguments: %s | raw=%s",
                exc, _sanitize_text(raw_arguments, 800))
            raise AgentError("云端计划不是有效 JSON。")
        if repair != "strict":
            logger.warning(
                "Normalized submit_plan arguments (%s); exact schema validation follows",
                repair)
        if not isinstance(document, dict) or set(document) - {
                "steps", "answer", "needs_clarification", "understood_sequence"}:
            raise AgentError("云端计划包含未定义字段。")
        steps = document.get("steps")
        answer = document.get("answer", "")
        clarification = document.get("needs_clarification", False)
        sequence = document.get("understood_sequence", [])
        if (not isinstance(sequence, list) or len(sequence) > 32 or
                any(not isinstance(s, str) or not s.strip() or len(s) > 1000 for s in sequence)):
            raise AgentError("understood_sequence 必须是最多32项的非空描述字符串数组。")
        if not isinstance(clarification, bool):
            raise AgentError("needs_clarification 必须是布尔值。")
        if clarification and (steps or not isinstance(answer, str) or not answer.strip()):
            raise AgentError("需要补充的计划必须 steps 为空且说明待补充内容。")
        if not isinstance(steps, list):
            raise AgentError("云端计划 steps 不是数组。")
        if len(steps) > self.max_plan_steps:
            raise AgentError("计划共 %d 步，超过上限 %d，未执行任何步骤。" % (
                len(steps), self.max_plan_steps))
        validated = []
        for index, step in enumerate(steps):
            if not isinstance(step, dict) or set(step) != {
                    "tool", "arguments", "description"}:
                raise AgentError("计划第 %d 步结构不完整。" % (index + 1))
            name = step["tool"]
            arguments = step["arguments"]
            description = step["description"]
            if name not in self._specs:
                raise AgentError("计划第 %d 步使用未知工具 %s。" % (index + 1, name))
            if not isinstance(description, str) or not description.strip():
                raise AgentError("计划第 %d 步缺少说明。" % (index + 1))
            error = validate_arguments(arguments, self._specs[name]["inputSchema"])
            if error:
                raise AgentError("计划第 %d 步参数无效：%s" % (index + 1, error))
            validated.append({
                "tool": name, "arguments": arguments,
                "description": description.strip(),
                "mutating": bool(self._specs[name].get("mutating", False)),
            })
        if not isinstance(answer, str):
            raise AgentError("云端计划 answer 必须是字符串。")
        plan = {"steps": validated, "answer": answer.strip()}
        if clarification:
            plan["needs_clarification"] = True
        if sequence:
            plan["understood_sequence"] = sequence
        return plan

    def reset_context(self):
        with self._context_lock:
            self._history = []
            self._context_generation += 1

    def awaiting_reply(self):
        with self._context_lock:
            return bool(self._history and
                        time.monotonic() - self._history[-1]["at"] < 60.0 and
                        self._history[-1]["state"] == "NEEDS_CLARIFICATION")

    def run(self, text, execute=False):
        with self._context_lock:
            generation = self._context_generation
        result = self._run(text, execute)
        with self._context_lock:
            if generation == self._context_generation:
                self._history.append({
                    "at": time.monotonic(), "text": text[:4000],
                    "state": result["state"],
                    "return_anchor": result.get("plan", {}).get("return_anchor"),
                    "reply": json.dumps({"state": result["state"],
                                         "plan": result.get("plan", {}),
                                         "answer": result.get("answer", "")},
                                        ensure_ascii=False)[:6000],
                })
                self._history = self._history[-3:]
        return result

    def _run(self, text, execute=False):
        if not self._run_lock.acquire(False):
            raise AgentError("已有 AI 请求正在处理。")
        self._cancel.clear()
        started = time.monotonic()
        try:
            self._emit("REQUEST", state="PARSING", description=text)
            plan = self.plan(text)
            self._emit(
                "PLAN", state="VALIDATED",
                result_json=json.dumps(plan, ensure_ascii=False),
            )
            if not plan["steps"]:
                return {
                    "state": "NEEDS_CLARIFICATION" if plan.get("needs_clarification") else "ANSWERED",
                    "plan": plan, "results": [],
                    "answer": plan["answer"] or "该请求不需要机器人工具。",
                    "total_ms": (time.monotonic() - started) * 1000.0,
                }
            mutating = any(step["mutating"] for step in plan["steps"])
            if not execute and mutating:
                return {
                    "state": "PREVIEW", "plan": plan, "results": [],
                    "answer": "计划已完成校验；AI 控制授权关闭，因此未执行机器人动作。",
                    "total_ms": (time.monotonic() - started) * 1000.0,
                }
            if execute and mutating and not self._authorised():
                raise AgentCancelled("AI control authorization was revoked")
            results = []
            failure = ""
            for index, step in enumerate(plan["steps"]):
                # Never let an exception in this step reuse a previous step's
                # GoalID/detail during fail-closed cleanup.
                detail = ""
                self._check_cancelled()
                if step["mutating"] and (not execute or not self._authorised()):
                    raise AgentCancelled("AI control authorization was revoked")
                if step["mutating"] and plan.get("return_anchor"):
                    status = self.mcp.call_tool("get_coverage_status", {})
                    current_map = self._decode(status.get("text", ""))
                    if (status.get("is_error") or current_map.get("map_digest") != plan["return_anchor"]["map_digest"]
                            or current_map.get("localized") is not True):
                        raise AgentError("返回点所属地图或定位状态已失效，未执行本步及后续动作。")
                self._active_tool = step["tool"]
                self._emit(
                    "STEP", step_index=index, tool=step["tool"],
                    state="RUNNING", description=step["description"],
                    arguments_json=json.dumps(step["arguments"], ensure_ascii=False),
                )
                step_started = time.monotonic()
                response = self.mcp.call_tool(
                    step["tool"], step["arguments"],
                    timeout=30.0, authorised=bool(execute and step["mutating"]),
                )
                detail = _sanitize_text(response.get("text"), self.tool_text_limit)
                ok = not response.get("is_error", False)
                if ok:
                    ok, detail = self._wait_if_async(step["tool"], detail)
                elif step["tool"] in VISUAL_START_TOOLS and self._decode(detail).get("visual_session_id"):
                    ok, detail = self._visual_failure(detail)
                elif step["tool"] in NAV_TOOLS:
                    failure_document = self._decode(detail)
                    if (failure_document.get("goal_id") or
                            detail.startswith("工具执行异常:")):
                        ok, detail = self._settle_failed_navigation(
                            detail, failure_document)
                elif step["tool"] in COVERAGE_START_TOOLS:
                    failure_document = self._decode(detail)
                    # The backend allocates the operation ID before handing
                    # the request to ROS.  Any error carrying that ID is a
                    # potentially accepted request and must stay under exact
                    # cleanup supervision.  Legacy transport exceptions are
                    # retained as a conservative fallback.
                    if (failure_document.get("batch_id") or
                            detail.startswith("工具执行异常:")):
                        failure_document.setdefault("error", detail)
                        ok, detail = self._settle_failed_coverage(
                            detail, failure_document)
                duration = (time.monotonic() - step_started) * 1000.0
                result = {
                    "tool": step["tool"], "ok": ok, "detail": detail,
                    "duration_ms": duration,
                }
                results.append(result)
                self._emit(
                    "STEP", step_index=index, tool=step["tool"],
                    state="SUCCEEDED" if ok else "FAILED",
                    description=step["description"], result_json=detail,
                    duration_ms=duration,
                )
                self._active_tool = ""
                if not ok:
                    failure = detail
                    break
            cancelled = self._cancel.is_set()
            state = "CANCELLED" if cancelled else (
                "FAILED" if failure else "SUCCEEDED")
            if cancelled:
                answer = (
                    "AI 任务已取消，剩余步骤未执行；各工具的退出/停车是否确认请以步骤结果为准。"
                )
            else:
                answer = self._summarise(text, plan, results, state)
            return {
                "state": state, "plan": plan, "results": results,
                "answer": answer,
                "total_ms": (time.monotonic() - started) * 1000.0,
            }
        except AgentCancelled as exc:
            results = locals().get("results", [])
            if self._active_tool in NAV_TOOLS:
                step_started = locals().get("step_started", time.monotonic())
                initial_detail = locals().get("detail", "")
                document = self._decode(initial_detail)
                document["error"] = (
                    "AI 导航执行期间收到取消/撤权请求: %s" % exc)
                _ok, cleanup_detail = self._settle_failed_navigation(
                    json.dumps(document, ensure_ascii=False), document)
                index = locals().get("index", len(results))
                if len(results) <= index:
                    results.append({
                        "tool": self._active_tool,
                        "ok": False,
                        "detail": cleanup_detail,
                        "duration_ms": (
                            time.monotonic() - step_started) * 1000.0,
                    })
            elif self._active_tool in COVERAGE_START_TOOLS:
                step_started = locals().get("step_started", time.monotonic())
                initial_detail = locals().get("detail", "")
                document = self._decode(initial_detail)
                document["error"] = (
                    "AI 覆盖任务执行期间收到取消/撤权请求: %s" % exc)
                _ok, cleanup_detail = self._settle_failed_coverage(
                    json.dumps(document, ensure_ascii=False), document)
                index = locals().get("index", len(results))
                if len(results) <= index:
                    results.append({
                        "tool": self._active_tool,
                        "ok": False,
                        "detail": cleanup_detail,
                        "duration_ms": (
                            time.monotonic() - step_started) * 1000.0,
                    })
            elif self._active_tool in VISUAL_START_TOOLS:
                # cancel() can race the enable reply, before the backend knows
                # the new visual ID. Retry exact cleanup after it is returned.
                _ok, cleanup_detail = self._visual_failure("AI视觉任务收到取消/撤权请求")
                results.append({"tool": self._active_tool, "ok": False,
                                "detail": cleanup_detail,
                                "duration_ms": (time.monotonic() - locals().get("step_started", started)) * 1000.0})
            return {
                "state": "CANCELLED", "plan": locals().get("plan", {"steps": []}),
                "results": results,
                "answer": (
                    "AI 任务已取消，剩余步骤未执行；各工具的退出/停车是否确认请以步骤结果为准。"
                ),
                "total_ms": (time.monotonic() - started) * 1000.0,
            }
        except Exception as exc:
            # A transport/query exception is not evidence that an already
            # published navigation goal stopped.  Keep the Qt task and run lock
            # alive until the backend either proves that it owns no goal or the
            # same explicit GoalID reaches a safe state.
            if self._active_tool in NAV_TOOLS:
                results = locals().get("results", [])
                step_started = locals().get("step_started", time.monotonic())
                initial_detail = locals().get("detail", "")
                document = self._decode(initial_detail)
                document["error"] = "AI 导航闭环异常: %s" % exc
                _ok, cleanup_detail = self._settle_failed_navigation(
                    json.dumps(document, ensure_ascii=False), document)
                index = locals().get("index", len(results))
                if len(results) <= index:
                    results.append({
                        "tool": self._active_tool,
                        "ok": False,
                        "detail": cleanup_detail,
                        "duration_ms": (
                            time.monotonic() - step_started) * 1000.0,
                    })
                return {
                    "state": "FAILED",
                    "plan": locals().get("plan", {"steps": []}),
                    "results": results,
                    "answer": (
                        "导航闭环发生异常；系统已等待同一 GoalID 的安全状态确认，"
                        "剩余步骤未执行。"
                    ),
                    "total_ms": (time.monotonic() - started) * 1000.0,
                }
            if self._active_tool in COVERAGE_START_TOOLS:
                results = locals().get("results", [])
                step_started = locals().get("step_started", time.monotonic())
                initial_detail = locals().get("detail", "")
                document = self._decode(initial_detail)
                document["error"] = "AI 覆盖闭环异常: %s" % exc
                _ok, cleanup_detail = self._settle_failed_coverage(
                    json.dumps(document, ensure_ascii=False), document)
                index = locals().get("index", len(results))
                if len(results) <= index:
                    results.append({
                        "tool": self._active_tool,
                        "ok": False,
                        "detail": cleanup_detail,
                        "duration_ms": (
                            time.monotonic() - step_started) * 1000.0,
                    })
                return {
                    "state": "FAILED",
                    "plan": locals().get("plan", {"steps": []}),
                    "results": results,
                    "answer": (
                        "覆盖任务闭环发生异常；系统保持任务所有权并等待后端"
                        "确认安全终结，剩余步骤未执行。"
                    ),
                    "total_ms": (time.monotonic() - started) * 1000.0,
                }
            raise
        finally:
            self._active_tool = ""
            self._run_lock.release()

    def _check_cancelled(self):
        if self._cancel.is_set():
            raise AgentCancelled("AI task cancelled")

    def _authorised(self):
        return (True if self.authorization_checker is None
                else bool(self.authorization_checker()))

    def _cloud_authorised(self):
        return (True if self.cloud_authorization_checker is None
                else bool(self.cloud_authorization_checker()))

    @staticmethod
    def _decode(text):
        try:
            value = json.loads(text)
            return value if isinstance(value, dict) else {}
        except (TypeError, ValueError):
            return {}

    def _wait_if_async(self, tool, initial):
        if tool in NAV_TOOLS:
            return self._poll_navigation(initial)
        if tool in COVERAGE_START_TOOLS:
            return self._poll_coverage(initial)
        if tool in VISUAL_START_TOOLS:
            return self._poll_visual(initial)
        return True, initial

    def _poll_navigation(self, initial):
        deadline = time.monotonic() + self.nav_wait_timeout
        last_text = _sanitize_text(initial, self.tool_text_limit)
        last_data = self._decode(last_text)
        expected_goal_id = str(last_data.get("goal_id", "")).strip()
        while time.monotonic() < deadline:
            try:
                self._check_cancelled()
            except AgentCancelled as exc:
                interrupted = dict(last_data)
                interrupted["error"] = "导航执行期间收到取消/撤权请求: %s" % exc
                return self._settle_failed_navigation(
                    json.dumps(interrupted, ensure_ascii=False), interrupted)
            try:
                response = self.mcp.call_tool(
                    "get_navigation_status", {}, timeout=10.0)
                if response.get("is_error"):
                    raise AgentError(response.get("text") or
                                     "导航状态工具返回错误")
            except Exception as exc:
                interrupted = dict(last_data)
                interrupted["error"] = "导航状态查询异常: %s" % exc
                return self._settle_failed_navigation(
                    json.dumps(interrupted, ensure_ascii=False), interrupted)
            last_text = _sanitize_text(
                response.get("text"), self.tool_text_limit)
            data = self._decode(last_text)
            status_goal_id = str(data.get("goal_id", "")).strip()
            if (expected_goal_id and status_goal_id and
                    status_goal_id != expected_goal_id):
                interrupted = dict(last_data)
                interrupted["goal_id"] = expected_goal_id
                interrupted["error"] = (
                    "导航状态返回了非当前 GoalID %s" % status_goal_id)
                return self._settle_failed_navigation(
                    json.dumps(interrupted, ensure_ascii=False), interrupted)
            if expected_goal_id and not data.get("goal_id"):
                data["goal_id"] = expected_goal_id
            last_data = data
            state = data.get("state", "unavailable")
            self._emit("PROGRESS", tool="navigate", state=state,
                       result_json=last_text)
            if state == "succeeded":
                return True, last_text
            if state in {"aborted", "preempted", "rejected", "recalled"}:
                return False, last_text
            if state in {"lost", "cancel_uncertain"} and data.get("goal_id"):
                return self._settle_failed_navigation(last_text, data)
            time.sleep(self.poll_interval)
        if last_data.get("goal_id"):
            timeout_document = dict(last_data)
            timeout_document["error"] = "等待导航完成超时"
            timeout_text = json.dumps(timeout_document, ensure_ascii=False)
            return self._settle_failed_navigation(timeout_text, timeout_document)
        try:
            self.mcp.call_tool(
                "cancel_navigation", {}, timeout=10.0, authorised=True)
        except Exception:
            pass
        return False, "等待导航完成超时；已请求取消 AI goal ID。"

    def _settle_failed_navigation(self, initial_text, document):
        """Keep an uncertain, already-published goal owned until it is safe.

        A tool-call error that carries a GoalID is a post-publication failure,
        not a preflight rejection.  The action may therefore still be moving.
        Reassert exact cancellation and keep the Qt step in RUNNING/cleanup
        until move_base proves a same-ID safe state.  The configured cleanup
        interval only emits a prolonged-wait diagnostic; it is never treated
        as proof that the vehicle stopped.
        """
        goal_id = str(document.get("goal_id", "")).strip()
        safe_states = {
            "aborted", "preempted", "rejected", "recalled",
            "recalled_before_forward", "succeeded",
        }
        confirmed = str(document.get("confirmed_state", "")).strip()
        if (goal_id and document.get("cancel_state") == "confirmed" and
                confirmed in safe_states):
            return False, _sanitize_text(initial_text, self.tool_text_limit)

        warning_deadline = time.monotonic() + self.navigation_cleanup_timeout
        last_text = _sanitize_text(initial_text, self.tool_text_limit)
        last_state = "cancel_uncertain"
        while True:
            try:
                response = self.mcp.call_tool(
                    "get_navigation_status", {}, timeout=10.0)
                if response.get("is_error"):
                    raise AgentError(response.get("text") or
                                     "导航状态工具返回错误")
                status_text = _sanitize_text(
                    response.get("text"), self.tool_text_limit)
                status = self._decode(status_text)
                status_goal_id = str(status.get("goal_id", "")).strip()
                last_state = str(status.get("state", "unavailable"))
                last_text = status_text or last_text
                if not goal_id and status_goal_id:
                    # This is not inference from a foreign/recent move_base
                    # goal.  get_navigation_status only exposes the exact ID
                    # retained by this MCP backend.
                    goal_id = status_goal_id
                self._emit(
                    "PROGRESS", tool="navigate",
                    state="cleanup_%s" % last_state,
                    result_json=last_text,
                )
                if status_goal_id == goal_id and last_state in safe_states:
                    return False, _sanitize_text(json.dumps({
                        "error": document.get(
                            "error", "导航提交后的闭环确认失败"),
                        "goal_id": goal_id,
                        "cleanup_state": "confirmed",
                        "confirmed_state": last_state,
                        "navigation_status": status,
                    }, ensure_ascii=False), self.tool_text_limit)
                if (not goal_id and not status_goal_id and
                        last_state == "idle"):
                    return False, _sanitize_text(json.dumps({
                        "error": document.get(
                            "error", "导航调用异常，但未提交任何目标"),
                        "goal_id": "",
                        "cleanup_state": "no_owned_goal",
                        "confirmed_state": "idle",
                        "navigation_status": status,
                    }, ensure_ascii=False), self.tool_text_limit)
            except Exception as exc:
                last_state = "unavailable"
                last_text = "导航撤销状态查询失败: %s" % exc
                self._emit(
                    "PROGRESS", tool="navigate", state="cleanup_unavailable",
                    result_json=_sanitize_text(last_text, self.tool_text_limit),
                )

            try:
                self.mcp.call_tool(
                    "cancel_navigation", {}, timeout=10.0, authorised=True)
            except Exception:
                # J6M also holds a heartbeat lease for this exact GoalID.  A
                # transient MCP error must not turn cleanup into an unrelated
                # cancel-all or allow the next plan step to start.
                pass
            if time.monotonic() >= warning_deadline:
                self._emit(
                    "PROGRESS", tool="navigate", state="cleanup_pending",
                    result_json=_sanitize_text(json.dumps({
                        "goal_id": goal_id or "UNKNOWN_PENDING_QUERY",
                        "last_state": last_state,
                        "detail": last_text,
                        "safety": (
                            "尚未证明同 ID 目标已停止；继续保留 GoalID、"
                            "重复精确取消并阻止后续步骤"
                        ),
                    }, ensure_ascii=False), self.tool_text_limit),
                )
                warning_deadline = (
                    time.monotonic() + self.navigation_cleanup_timeout)
            time.sleep(self.poll_interval)

    def _poll_coverage(self, initial):
        deadline = time.monotonic() + self.coverage_wait_timeout
        last_text = _sanitize_text(initial, self.tool_text_limit)
        last_data = self._decode(last_text)
        expected_batch_id = str(last_data.get("batch_id", "")).strip()
        while time.monotonic() < deadline:
            try:
                self._check_cancelled()
            except AgentCancelled as exc:
                interrupted = dict(last_data)
                interrupted["error"] = (
                    "覆盖任务执行期间收到取消/撤权请求: %s" % exc)
                return self._settle_failed_coverage(
                    json.dumps(interrupted, ensure_ascii=False), interrupted)
            try:
                response = self.mcp.call_tool(
                    "get_coverage_status", {}, timeout=10.0)
                if response.get("is_error"):
                    raise AgentError(response.get("text") or
                                     "覆盖状态工具返回错误")
            except Exception as exc:
                interrupted = dict(last_data)
                interrupted["error"] = "覆盖状态查询异常: %s" % exc
                return self._settle_failed_coverage(
                    json.dumps(interrupted, ensure_ascii=False), interrupted)
            last_text = _sanitize_text(
                response.get("text"), self.tool_text_limit)
            data = self._decode(last_text)
            status_batch_id = str(data.get("batch_id", "")).strip()
            if (expected_batch_id and status_batch_id and
                    status_batch_id != expected_batch_id):
                # CoverageStatus is a global/latest-batch view.  Never let a
                # foreign batch replace the exact operation ID returned by
                # this step: doing so could make cleanup wait on or describe
                # B while the MCP backend is still retaining/canceling A.
                interrupted = dict(data)
                interrupted["batch_id"] = expected_batch_id
                interrupted["observed_foreign_batch_id"] = status_batch_id
                interrupted["error"] = (
                    "等待覆盖批次 %s 时全局状态已切换到其他批次 %s；"
                    "转入原批次的精确清理" %
                    (expected_batch_id, status_batch_id))
                return self._settle_failed_coverage(
                    json.dumps(interrupted, ensure_ascii=False), interrupted)
            if (expected_batch_id and status_batch_id == expected_batch_id and
                    "ai_owned" in data and not bool(data.get("ai_owned"))):
                interrupted = dict(data)
                interrupted["batch_id"] = expected_batch_id
                interrupted["error"] = (
                    "覆盖状态不再确认批次 %s 属于当前 AI 会话；"
                    "转入同 ID 精确清理" % expected_batch_id)
                return self._settle_failed_coverage(
                    json.dumps(interrupted, ensure_ascii=False), interrupted)
            if expected_batch_id and not status_batch_id:
                data["batch_id"] = expected_batch_id
            last_data = data
            state = data.get("state", "unavailable")
            self._emit("PROGRESS", tool="coverage", state=state,
                       result_json=last_text)
            terminal = state in {
                "COMPLETED", "COMPLETED_PARTIAL", "CANCELED", "FAILED",
            }
            released = not bool(data.get("active")) and not bool(
                data.get("batch_active"))
            same_batch = (not expected_batch_id or
                          str(data.get("batch_id", "")) == expected_batch_id)
            if terminal and released and same_batch:
                return state == "COMPLETED", last_text
            if terminal and not released:
                interrupted = dict(data)
                interrupted["error"] = (
                    "覆盖后端已报告终态，但导航所有权或活动锁仍未安全释放")
                return self._settle_failed_coverage(last_text, interrupted)
            time.sleep(self.poll_interval)
        timeout_document = dict(last_data)
        if expected_batch_id:
            timeout_document["batch_id"] = expected_batch_id
        timeout_document["error"] = "等待覆盖清扫完成超时"
        return self._settle_failed_coverage(
            json.dumps(timeout_document, ensure_ascii=False), timeout_document)

    def _settle_failed_coverage(self, initial_text, document):
        """Retain the AI run lock until a coverage batch releases ownership."""
        expected_batch_id = str(document.get("batch_id", "")).strip()
        if (expected_batch_id and document.get("cancel_state") in {
                "confirmed_not_started", "confirmed_terminal"}):
            # /coverage/cancel_batch is keyed by the client-generated batch
            # ID.  These two responses are authoritative proof that the exact
            # operation never started or had already completed; no global or
            # newest-batch inference is involved.
            return False, _sanitize_text(initial_text, self.tool_text_limit)
        terminal_states = {
            "COMPLETED", "COMPLETED_PARTIAL", "CANCELED", "FAILED",
        }
        warning_deadline = time.monotonic() + self.navigation_cleanup_timeout
        no_batch_not_before = time.monotonic() + 2.5
        last_text = _sanitize_text(initial_text, self.tool_text_limit)
        last_state = str(document.get("state", "cleanup_pending"))
        while True:
            try:
                response = self.mcp.call_tool(
                    "get_coverage_status", {}, timeout=10.0)
                if response.get("is_error"):
                    raise AgentError(response.get("text") or
                                     "覆盖状态工具返回错误")
                status_text = _sanitize_text(
                    response.get("text"), self.tool_text_limit)
                status = self._decode(status_text)
                status_batch_id = str(status.get("batch_id", "")).strip()
                if (not expected_batch_id and status_batch_id and
                        bool(status.get("ai_owned"))):
                    expected_batch_id = status_batch_id
                last_state = str(status.get("state", "unavailable"))
                last_text = status_text or last_text
                active = bool(status.get("active"))
                batch_active = bool(status.get("batch_active"))
                same_batch = bool(
                    expected_batch_id and
                    status_batch_id == expected_batch_id
                )
                self._emit(
                    "PROGRESS", tool="coverage",
                    state="cleanup_%s" % last_state,
                    result_json=last_text,
                )
                if (same_batch and last_state in terminal_states and
                        not active and not batch_active):
                    return False, _sanitize_text(json.dumps({
                        "error": document.get(
                            "error", "覆盖任务终结后的清理尚未确认"),
                        "batch_id": expected_batch_id,
                        "cleanup_state": "confirmed",
                        "confirmed_state": last_state,
                        "coverage_status": status,
                    }, ensure_ascii=False), self.tool_text_limit)
                if (not expected_batch_id and not status_batch_id and
                        last_state == "IDLE" and
                        time.monotonic() >= no_batch_not_before and
                        not active and not batch_active):
                    return False, _sanitize_text(json.dumps({
                        "error": document.get(
                            "error", "覆盖调用异常，但未发现活动批次"),
                        "batch_id": "",
                        "cleanup_state": "no_owned_batch",
                        "confirmed_state": last_state,
                        "coverage_status": status,
                    }, ensure_ascii=False), self.tool_text_limit)
            except Exception as exc:
                last_state = "unavailable"
                last_text = "覆盖撤销状态查询失败: %s" % exc
                self._emit(
                    "PROGRESS", tool="coverage",
                    state="cleanup_unavailable",
                    result_json=_sanitize_text(last_text, self.tool_text_limit),
                )

            try:
                cancel_response = self.mcp.call_tool(
                    "cancel_coverage", {}, timeout=10.0, authorised=True)
                if not cancel_response.get("is_error"):
                    cancel_text = _sanitize_text(
                        cancel_response.get("text"), self.tool_text_limit)
                    cancel_status = self._decode(cancel_text)
                    cancel_batch_id = str(
                        cancel_status.get("batch_id", "")
                    ).strip()
                    cancel_state = str(
                        cancel_status.get("cancel_state", "")
                    ).strip()
                    if not expected_batch_id and cancel_batch_id:
                        expected_batch_id = cancel_batch_id
                    if (expected_batch_id and
                            cancel_batch_id == expected_batch_id and
                            cancel_state in {
                                "confirmed_not_started",
                                "confirmed_terminal",
                            }):
                        return False, _sanitize_text(json.dumps({
                            "error": document.get(
                                "error", "覆盖启动后的精确撤销已确认"),
                            "batch_id": expected_batch_id,
                            "cleanup_state": "confirmed",
                            "confirmed_state": cancel_state,
                            "cancel_result": cancel_status,
                        }, ensure_ascii=False), self.tool_text_limit)
            except Exception:
                pass
            if time.monotonic() >= warning_deadline:
                self._emit(
                    "PROGRESS", tool="coverage", state="cleanup_pending",
                    result_json=_sanitize_text(json.dumps({
                        "batch_id": expected_batch_id or "UNKNOWN_PENDING_QUERY",
                        "last_state": last_state,
                        "detail": last_text,
                        "safety": (
                            "覆盖后端尚未同时确认终态与 active=false；"
                            "继续保留 AI 任务并重复请求取消"
                        ),
                    }, ensure_ascii=False), self.tool_text_limit),
                )
                warning_deadline = (
                    time.monotonic() + self.navigation_cleanup_timeout)
            time.sleep(self.poll_interval)

    def _poll_visual(self, initial):
        started = self._decode(initial)
        expected_id = started.get("visual_session_id")
        baseline = started.get("completion_baseline")
        if not session_id_valid(expected_id) or not sequence_valid(baseline):
            return self._visual_failure("视觉请求缺少本次会话/完成基线，不能监督完成，后续步骤未执行")
        deadline = time.monotonic() + self.visual_wait_timeout
        failure = "等待视觉定点清扫完成超时"
        last_status = {}
        while time.monotonic() < deadline:
            self._check_cancelled()
            try:
                response = self.mcp.call_tool("get_visual_servo_status", {}, timeout=10.0)
            except Exception as exc:
                failure = "视觉状态传输异常: " + _sanitize_text(str(exc))
                break
            data = self._decode(response.get("text"))
            last_status = data
            state = data.get("state", "UNAVAILABLE")
            self._emit("PROGRESS", tool="visual", state=state,
                       result_json=_sanitize_text(response.get("text")))
            if (response.get("is_error") or data.get("visual_session_id") != expected_id
                    or state == "UNAVAILABLE" or data.get("ai_completion_protocol") != 1):
                failure = "视觉状态查询失败或会话已变化，不能认定本次完成"
                break
            if state in {"FOD_ABORTED", "FAULT_STOP"} or data.get("visual_result") in {"ABORT", "CANCELLED"}:
                controller_aborted = (data.get("visual_state") == "ABORT"
                                      or data.get("visual_result") in {"ABORT", "CANCELLED"})
                reason = (data.get("visual_reason") if controller_aborted else data.get("mode_reason"))
                reason = reason or data.get("reason") or "控制器未给出具体原因"
                failure = "视觉会话中止或取消: " + _sanitize_text(reason)
                break
            if visual_completed(data, expected_id, baseline):
                return True, _sanitize_text(response.get("text"))
            # Mode and visual receipt arrive on independent topics. A genuine
            # COMPLETE may precede its stopped receipt; keep polling within the
            # existing task timeout instead of failing this completed cleaning.
            if state == "RELATIVE_NAV_ACTIVE" and data.get("visual_result") != "COMPLETE":
                failure = "已退出视觉但缺少本次完成/停车证据，后续步骤未执行"
                break
            time.sleep(self.poll_interval)
        return self._visual_failure(failure, expected_id, last_status)

    def _visual_failure(self, failure, session_id="", status=None):
        document = self._decode(failure)
        session_id = session_id or document.get("visual_session_id", "")
        cleanup_state = "exit_unconfirmed"
        try:
            response = self.mcp.call_tool("stop_spot_cleaning", {}, authorised=True)
            cleanup = _sanitize_text(response.get("text", ""))
            if response.get("is_error"):
                cleanup = "退出未确认: " + cleanup
            else:
                # A service acknowledgement is not a physical stop receipt.
                cleanup_state = "exit_requested"
        except Exception as exc:
            cleanup = "退出请求异常: " + _sanitize_text(str(exc))
        evidence = {}
        for key in ("state", "visual_state", "visual_result", "visual_session_id",
                    "visual_reason", "mode_reason"):
            if status and key in status:
                evidence[key] = _sanitize_text(status[key], 400)
        # Bound individual strings, not serialized JSON: cutting the closing
        # brace would discard the execution-state evidence in the final summary.
        return False, json.dumps({
            "execution_state": "started_not_completed" if session_id_valid(session_id) else "start_unconfirmed",
            "message": _sanitize_text(document.get("message") or failure, 600),
            "visual_session_id": session_id if session_id_valid(session_id) else "",
            "cleanup_state": cleanup_state, "cleanup_detail": _sanitize_text(cleanup, 400),
            # Capture controller evidence before cleanup replaces ABORT with DISABLED.
            "visual_status": evidence,
        }, ensure_ascii=False)

    def cancel(self):
        self._cancel.set()
        tool = self._active_tool
        cleanup = ""
        if tool in NAV_TOOLS:
            cleanup = "cancel_navigation"
        elif tool in COVERAGE_START_TOOLS:
            cleanup = "cancel_coverage"
        elif tool in VISUAL_START_TOOLS:
            cleanup = "stop_spot_cleaning"
        if cleanup:
            try:
                self.mcp.call_tool(cleanup, {}, timeout=10.0, authorised=True)
            except Exception:
                pass

    def _summarise(self, original, plan, results, state):
        if state == "FAILED" and results and results[-1]["tool"] in VISUAL_START_TOOLS:
            # An LLM must not reinterpret "not completed" as "never executed".
            # Keep this safety-relevant outcome deterministic and evidence-based.
            last = results[-1]
            data = self._decode(last["detail"])
            outcome = {
                "not_started": "视觉清扫未启动",
                "started_not_completed": "视觉清扫已启动，但未完成",
                "start_unconfirmed": "视觉启动结果未确认，不能认定已清扫",
            }.get(data.get("execution_state"), "视觉清扫未确认完成")
            reason = _sanitize_text(data.get("message") or last["detail"], 600)
            answer = "已完成 %d 步；%s。原因：%s" % (
                sum(1 for item in results if item["ok"]), outcome, reason)
            if data.get("cleanup_state") == "exit_unconfirmed":
                answer += "；退出未确认，请检查步骤详情及车辆状态"
            elif data.get("cleanup_state") == "exit_requested":
                answer += "；已请求退出本 AI 视觉会话，服务应答不代表清扫完成或物理停车已确认"
            remaining = max(0, len(plan["steps"]) - len(results))
            if remaining:
                answer += "；后续 %d 步未执行" % remaining
            return answer + "。"
        safe_results = [{
            "tool": item["tool"], "ok": item["ok"],
            "detail": _sanitize_text(item["detail"], self.tool_text_limit),
        } for item in results]
        messages = [
            {"role": "system", "content": (
                "根据脱敏的机器人 MCP 执行记录，用一到两句中文如实总结。"
                "不得声称未执行步骤已经完成，不得输出密钥、路径或设备标识。")},
            {"role": "user", "content": json.dumps({
                "instruction": original,
                "validated_plan": plan["steps"],
                "result_state": state,
                "tool_results": safe_results,
            }, ensure_ascii=False)},
        ]
        try:
            self._check_cancelled()
            if not self._cloud_authorised():
                raise AgentCancelled("AI parse authorization was revoked")
            response = self.llm.chat(messages)
            self._cloud_event("SUMMARY")
            content = (response.get("content") or "").strip()
            if content:
                return content
        except Exception:
            pass
        succeeded = sum(1 for item in results if item["ok"])
        if state == "SUCCEEDED":
            return "计划中的 %d 个步骤已按顺序完成。" % succeeded
        return "已完成 %d 个步骤，随后失败并停止剩余计划：%s" % (
            succeeded, results[-1]["detail"] if results else "未知错误")
