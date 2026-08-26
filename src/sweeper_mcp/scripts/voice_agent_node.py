#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 语音 Agent 节点 —— 订阅 /voice/text，Agent 循环执行 MCP 工具，发布 /voice/agent_reply。

与 voice_agent_cli.py 共用 sweeper_mcp.agent.AgentRunner 核心逻辑：
  /voice/text (std_msgs/String) → DeepSeek(function calling) → MCP 工具 → /voice/agent_reply

进度实时回传：agent 每输出一行状态（[规划]拆分/ [执行]指令N / [导航]状态 / [完成]），
除终端 stdout 外还发布到 /voice/agent_progress (std_msgs/String)，供 CLI（如
voice_agent_sim.py）逐行实时显示"任务分解→执行指令1→车才开始动"的过程。

回调不阻塞：每收到一句指令在独立线程里跑 agent（LLM+工具是秒级，不能卡在回调里）。

启动:
  roslaunch sweeper_mcp voice_agent.launch
  # 或 rosrun sweeper_mcp voice_agent_node.py（配合 keyboard_input 发 /voice/text）
"""

import os
import sys
import threading

import rospy
from std_msgs.msg import String

_PKG_SRC = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
if _PKG_SRC not in sys.path:
    sys.path.insert(0, _PKG_SRC)

from sweeper_mcp.agent import AgentRunner, set_progress_listener  # noqa: E402


# 指令拆分/顺序执行/导航监控 的 agent 运行参数（可被 ros param 覆盖；未设置则用配置默认值）
_AGENT_PARAMS = ("enable_planning", "max_plan_steps", "nav_wait_timeout",
                 "nav_wait_poll", "continue_on_nav_fail", "llm_summary", "verbose")


class VoiceAgentNode:
    def __init__(self):
        base_url = rospy.get_param("~agent/base_url", "https://api.deepseek.com")
        # 模型/密钥优先级：环境变量 > 节点参数(来自 config/sweeper_mcp.yaml)
        model = os.environ.get("DEEPSEEK_MODEL") or rospy.get_param("~agent/model", "deepseek-v4-flash")
        api_key = os.environ.get("DEEPSEEK_API_KEY") or rospy.get_param("~agent/api_key", "")
        temperature = rospy.get_param("~agent/temperature", 0.2)
        timeout_s = rospy.get_param("~agent/timeout_s", 20.0)
        input_topic = rospy.get_param("~ros/input_topic", "/voice/text")
        reply_topic = rospy.get_param("~ros/reply_topic", "/voice/agent_reply")
        progress_topic = rospy.get_param("~ros/progress_topic", "/voice/agent_progress")
        backend = os.environ.get("MCP_BACKEND", "ros")

        agent_kwargs = {}
        for k in _AGENT_PARAMS:
            v = rospy.get_param("~agent/%s" % k, None)
            if v is not None:
                agent_kwargs[k] = v

        self._runner = AgentRunner(base_url=base_url, model=model, api_key=api_key,
                                   temperature=temperature, timeout_s=timeout_s,
                                   backend=backend, **agent_kwargs)
        self._pub = rospy.Publisher(reply_topic, String, queue_size=10)
        # 进度实时回传：agent 每输出一行状态就发布到 /voice/agent_progress
        self._progress_pub = rospy.Publisher(progress_topic, String, queue_size=50)
        set_progress_listener(self._publish_progress)
        rospy.Subscriber(input_topic, String, self._on_text, queue_size=10)
        rospy.loginfo("voice_agent 就绪: 输入 %s → 输出 %s, 进度 %s (后端 %s, 模型 %s)",
                      input_topic, reply_topic, progress_topic, backend, model)

    def _publish_progress(self, line):
        """把 agent 的一行状态发布到进度话题（发布失败不影响 agent 主流程）。"""
        try:
            self._progress_pub.publish(String(data=line))
        except Exception:
            pass

    def _on_text(self, msg):
        text = (msg.data or "").strip()
        if not text:
            return
        threading.Thread(target=self._handle, args=(text,), daemon=True).start()

    def _handle(self, text):
        rospy.loginfo("收到指令: %s", text)
        try:
            reply = self._runner.run(text)
        except Exception as exc:
            rospy.logerr("Agent 执行失败: %s", exc)
            reply = "抱歉，处理出错: %s" % exc
        self._pub.publish(String(data=reply))
        rospy.loginfo("回复: %s", reply)


def main():
    rospy.init_node("voice_agent", anonymous=True)
    node = VoiceAgentNode()  # noqa: F841  (持有引用防 GC)
    rospy.spin()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
