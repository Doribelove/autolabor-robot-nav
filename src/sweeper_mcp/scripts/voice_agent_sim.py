#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""语音控制仿真联调 —— 用户语音 → ASR → /voice/text → voice_agent_node(DeepSeek+MCP) → 机器人运动 → /voice/agent_reply。

与 voice_agent_cli.py 的区别：本脚本**不直接调 LLM/MCP**，只负责"听你说、把中文指令发出去、
把最终答复打出来"。真正的理解与执行由常驻的 voice_agent 节点完成（它订阅 /voice/text，
内部跑 DeepSeek function calling + MCP 工具，发布 /voice/agent_reply）。

进度实时回传：voice_agent 节点把 agent 每行状态（[规划]任务分解/ [执行]指令N/
[导航]状态 / [完成]）发布到 /voice/agent_progress，本脚本**逐行实时打印**——
你会在终端先看到任务分解结果，然后"执行第一条指令"，接着车才开始动，最后收到最终答复。

使用 sweeper_mcp.voice 子模块的 ASR 能力（pyaudio 16k + whisper + OpenCC 简体）。
麦克风输入 → 中文文本 → 发布 /voice/text → 车按指令移动 → 打印大模型回复。

用法（先拉起仿真 + voice_agent，再运行本脚本）:
  bash scripts/start_sim_voice.sh                        # 一键启动仿真
  python3 src/sweeper_mcp/scripts/voice_agent_sim.py      # 语音联调，对麦克风说话

终端命令:
  start / s       开始录音
  stop  / e       结束录音并识别 → 发布 /voice/text → 等待并打印 agent 回复
  exit  / q / quit    退出
  help  / h       帮助
"""

import argparse
import os
import sys
import time

_THIS = os.path.dirname(os.path.abspath(__file__))
_PKG_SRC = os.path.abspath(os.path.join(_THIS, "..", "src"))
if _PKG_SRC not in sys.path:
    sys.path.insert(0, _PKG_SRC)

import rospy  # noqa: E402
from std_msgs.msg import String  # noqa: E402

try:
    import numpy as np  # noqa: F401
    from sweeper_mcp.voice.asr_audio import AudioRecorder, list_input_devices, pcm_bytes_to_float32  # noqa: E402
    from sweeper_mcp.voice.asr_recognizer import WhisperRecognizer  # noqa: E402
except Exception as exc:
    print("加载 ASR 模块失败: %s\n请确认 whisper/pyaudio 已装。" % exc, file=sys.stderr)
    sys.exit(1)

HELP_TEXT = """可用命令:
  start / s        开始录音
  stop  / e        结束录音并识别 → 发布 /voice/text → 等待 agent 回复
  exit  / q / quit 退出程序
  help  / h        显示帮助"""

MIN_SECONDS = 0.2
MIN_RMS = 0.003
# 发布指令后等待最终答复的硬上限(秒)：多步指令（如"先X再Y最后Z"）单步导航+LLM
# 调用可到 1~3 分钟，60s 不够，放宽到 300s，靠活跃度提示兜底。
REPLY_TIMEOUT = 300.0
# 无进度且无回复超过该秒数，才提示"可能卡住"（仍继续等待到硬上限）。
IDLE_TIMEOUT = 40.0


def _rms(audio):
    return float(np.sqrt(np.mean(audio ** 2)))


class _AgentWatcher:
    """订阅 /voice/agent_reply（最终答复）+ /voice/agent_progress（实时进度）。

    进度到达即打印；wait_reply 等待最终答复，期间若无进度也无答复超过
    IDLE_TIMEOUT 秒会提示一次，但不会提前放弃。
    """

    def __init__(self, reply_topic, progress_topic):
        self._count = 0
        self._latest = None
        self._last_progress = time.time()
        self._sub_reply = rospy.Subscriber(reply_topic, String, self._on_reply, queue_size=10)
        self._sub_prog = rospy.Subscriber(progress_topic, String, self._on_progress, queue_size=50)

    def _on_reply(self, msg):
        self._count += 1
        self._latest = msg.data

    def _on_progress(self, msg):
        self._last_progress = time.time()
        print("  ⏳ %s" % msg.data, flush=True)

    def reset_activity(self):
        """在发指令前调用，把"最后活跃时间"清零到当前时刻。"""
        self._last_progress = time.time()

    def wait_reply(self, base, timeout=REPLY_TIMEOUT, idle_timeout=IDLE_TIMEOUT):
        """等待收到第 base+1 条回复，返回 (回复文本或 None, 是否超时)。"""
        start = time.time()
        warned = False
        while time.time() - start < timeout:
            if self._count > base:
                return self._latest, False
            idle = time.time() - self._last_progress
            if not warned and idle > idle_timeout:
                warned = True
                print("  ⚠ 已 %d 秒无进度更新，agent 可能仍在处理，请耐心等待…"
                      % int(idle_timeout), flush=True)
            time.sleep(0.2)
        return None, True


def main():
    parser = argparse.ArgumentParser(
        description="语音控制仿真联调：语音→ASR→/voice/text→agent→/voice/agent_reply")
    parser.add_argument("--input-topic", default="/voice/text",
                        help="中文指令话题（默认 /voice/text，voice_agent 节点订阅）")
    parser.add_argument("--reply-topic", default="/voice/agent_reply",
                        help="agent 回复话题（默认 /voice/agent_reply）")
    parser.add_argument("--progress-topic", default="/voice/agent_progress",
                        help="agent 实时进度话题（默认 /voice/agent_progress）")
    parser.add_argument("--reply-timeout", type=float, default=REPLY_TIMEOUT,
                        help="等待最终答复的硬上限秒数（默认 %.0f）" % REPLY_TIMEOUT)
    parser.add_argument("--save", action="store_true", help="保存录音 WAV 与识别文本（默认不保存）")
    parser.add_argument("--out-dir", default=None, help="录音保存目录（需 --save）")
    parser.add_argument("--asr-model", default="small", help="whisper 模型尺寸")
    parser.add_argument("--language", default="zh", help="识别语言，auto 自动检测")
    parser.add_argument("--device", default="auto", help="cuda / cpu / auto")
    parser.add_argument("--input-device", type=int, default=None, help="输入设备索引")
    parser.add_argument("--list-devices", action="store_true", help="列出输入设备后退出")
    args = parser.parse_args()

    if args.list_devices:
        devices = list_input_devices()
        for d in devices or []:
            print("  [%d] %s (声道数: %d)" % (d["index"], d["name"], d["channels"]))
        return 0

    # ROS 节点：不抢 SIGINT（避免与交互式 input 冲突）；订阅回调在后台线程运行
    rospy.init_node("voice_agent_sim", anonymous=True, disable_signals=True)
    pub = rospy.Publisher(args.input_topic, String, queue_size=10)
    watcher = _AgentWatcher(args.reply_topic, args.progress_topic)

    out_dir = args.out_dir or os.path.abspath(os.path.join(_THIS, "..", "..", "..", "voice_recordings"))
    os.makedirs(out_dir, exist_ok=True)

    recorder = AudioRecorder(device_index=args.input_device)
    recognizer = WhisperRecognizer(model=args.asr_model, language=args.language,
                                   device=args.device, to_simplified=True)

    print("=" * 60)
    print("语音控制仿真联调  ASR:%s" % args.asr_model)
    print("  指令: %s → agent → 回复: %s" % (args.input_topic, args.reply_topic))
    print("  进度: %s（任务分解/执行/导航 逐行实时显示）" % args.progress_topic)
    print("（提示：先确认已启动仿真与 voice_agent 节点，否则发指令没有执行者）")
    print("=" * 60)
    print(HELP_TEXT)
    print()

    try:
        while True:
            try:
                cmd = input("> ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                print()
                break

            if cmd in ("start", "s"):
                if recorder.recording:
                    print("已经在录音中。")
                    continue
                recorder.start()
                print("▶ 录音中… 输入 stop 结束", flush=True)

            elif cmd in ("stop", "e"):
                if not recorder.recording:
                    print("当前未在录音（先输入 start 开始）。")
                    continue
                pcm, duration = recorder.stop()
                if duration < MIN_SECONDS:
                    print("录音太短(%.1fs)，已丢弃。" % duration)
                    continue
                audio = pcm_bytes_to_float32(pcm)
                if _rms(audio) < MIN_RMS:
                    print("音频过静(可能无语音)，已丢弃。")
                    continue
                wav_path = None
                if args.save:
                    ts = time.strftime("%Y%m%d_%H%M%S")
                    wav_path = os.path.join(out_dir, "rec_%s.wav" % ts)
                    recorder.save_wav(wav_path, pcm)
                    print("已保存录音: %s (%.1fs)  识别中…" % (wav_path, duration), flush=True)
                else:
                    print("识别中… (%.1fs)" % duration, flush=True)

                if not recognizer.loaded:
                    print("正在加载 whisper 模型…", flush=True)
                text = recognizer.transcribe(audio)
                if not text:
                    print("未识别到有效语音。")
                    continue
                print("📝 识别: %s" % text, flush=True)
                if args.save and wav_path:
                    with open(wav_path.rsplit(".", 1)[0] + ".txt", "w", encoding="utf-8") as f:
                        f.write(text + "\n")

                # 发布到 /voice/text：等订阅者连接建立后只发一次。
                # 之前连发两次会让 agent 收到同一条指令两遍、并发跑两遍完整计划，
                # 导航互相抢占、耗时翻倍（这也是此前 60s 超时的一个原因）。
                base = watcher._count
                watcher.reset_activity()
                msg = String(data=text)
                for _ in range(20):
                    if pub.get_num_connections() > 0:
                        break
                    time.sleep(0.1)
                pub.publish(msg)

                print("🤖 等待 agent 处理并执行…（实时进度如下）", flush=True)
                reply, timed_out = watcher.wait_reply(base, args.reply_timeout)
                if timed_out:
                    print("⚠ %.0f 秒内未收到最终答复（检查 voice_agent 节点是否在线）"
                          % args.reply_timeout)
                else:
                    print("🤖 %s" % reply)

            elif cmd in ("exit", "q", "quit"):
                break
            elif cmd in ("help", "h"):
                print(HELP_TEXT)
            elif cmd == "":
                continue
            else:
                print("未知命令: %s（输入 help 查看命令）" % cmd)
    finally:
        if recorder.recording:
            recorder.stop()
        recorder.close()
        recognizer.unload()
        print("已退出。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
