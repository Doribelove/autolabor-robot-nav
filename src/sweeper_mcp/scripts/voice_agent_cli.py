#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""语音 Agent 命令行 —— 说话 → whisper 识别 → DeepSeek agent → MCP 工具执行 → 打印答复。

使用 sweeper_mcp.voice 子模块的 ASR 能力（pyaudio 16k + whisper + OpenCC 简体）。
核心逻辑是 sweeper_mcp.agent.AgentRunner（同一份代码，ROS 节点也用它）。

用法:
  MCP_BACKEND=mock python3 scripts/voice_agent_cli.py          # 离线(不执行真实操作)
  MCP_BACKEND=ros  python3 scripts/voice_agent_cli.py          # 真实 ROS(需已 source 环境)
  export DEEPSEEK_API_KEY=...                                  # 必填

终端命令:
  start / s       开始录音
  stop  / e       结束录音并识别 → Agent 执行 → 打印答复
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

from sweeper_mcp.agent import AgentRunner  # noqa: E402

try:
    import numpy as np  # noqa: F401
    from sweeper_mcp.voice.asr_audio import AudioRecorder, list_input_devices, pcm_bytes_to_float32  # noqa: E402
    from sweeper_mcp.voice.asr_recognizer import WhisperRecognizer  # noqa: E402
except Exception as exc:
    print("加载 ASR 模块失败: %s\n请确认 whisper/pyaudio 已装。" % exc, file=sys.stderr)
    sys.exit(1)

HELP_TEXT = """可用命令:
  start / s        开始录音
  stop  / e        结束录音并识别 → Agent 执行 → 打印答复
  exit  / q / quit 退出程序
  help  / h        显示帮助"""

MIN_SECONDS = 0.2
MIN_RMS = 0.003


def _load_config():
    """读 config/sweeper_mcp.yaml 的 agent 段作为 LLM 默认参数；yaml 缺失时用默认值。"""
    defaults = {"base_url": "https://api.deepseek.com", "api_key": "",
                "model": "deepseek-v4-flash", "temperature": 0.2, "timeout_s": 20.0}
    yaml_path = os.path.abspath(os.path.join(_THIS, "..", "config", "sweeper_mcp.yaml"))
    try:
        import yaml
        with open(yaml_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
        agent = cfg.get("agent") or {}
        defaults.update({k: v for k, v in agent.items() if k in defaults})
    except Exception:
        pass
    return defaults


def _rms(audio):
    return float(np.sqrt(np.mean(audio ** 2)))


def main():
    cfg = _load_config()
    parser = argparse.ArgumentParser(description="语音 Agent 命令行（ASR + DeepSeek + MCP 工具执行）")
    parser.add_argument("--backend", default=os.environ.get("MCP_BACKEND", "mock"),
                        help="mock=离线 | ros=真实（默认取 MCP_BACKEND 环境变量，缺省 mock）")
    parser.add_argument("--base-url", default=cfg["base_url"], help="LLM 端点")
    parser.add_argument("--llm-model", default=None, help="LLM 模型名（默认取配置文件）")
    parser.add_argument("--api-key", default=None, help="LLM API key（默认取配置文件/环境变量）")
    parser.add_argument("--temperature", type=float, default=cfg["temperature"])
    parser.add_argument("--timeout", type=float, default=cfg["timeout_s"])
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

    # 模型与密钥解析优先级：启动参数 > 环境变量 > 配置文件
    model = args.llm_model or os.environ.get("DEEPSEEK_MODEL") or cfg["model"]
    api_key = args.api_key or os.environ.get("DEEPSEEK_API_KEY") or cfg.get("api_key")
    if not api_key:
        print("⚠ 未配置 api_key（配置文件 / 环境变量 DEEPSEEK_API_KEY / --api-key 三选一）。", file=sys.stderr)
        return 1

    out_dir = args.out_dir or os.path.abspath(os.path.join(_THIS, "..", "..", "..", "voice_recordings"))
    os.makedirs(out_dir, exist_ok=True)

    recorder = AudioRecorder(device_index=args.input_device)
    recognizer = WhisperRecognizer(model=args.asr_model, language=args.language,
                                   device=args.device, to_simplified=True)
    runner = AgentRunner(base_url=args.base_url, model=model, api_key=api_key,
                         temperature=args.temperature, timeout_s=args.timeout,
                         backend=args.backend)

    print("=" * 60)
    print("语音 Agent  后端:%s  LLM:%s  ASR:%s" % (args.backend, model, args.asr_model))
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

                print("🤖 思考并执行中…", flush=True)
                reply = runner.run(text)
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
        runner.close()
        print("已退出。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
