"""sweeper_mcp.voice —— AI 语音子模块。

封装语音识别（ASR）与音频采集能力，供 sweeper_mcp 包内各语音入口统一使用：
- asr_audio: pyaudio 录音、设备枚举、PCM 格式转换
- asr_recognizer: whisper 离线识别 + OpenCC 繁转简

本模块从 ai_task_decomposition 迁移合并而来，成为 sweeper_mcp 的正式语音能力层。
"""

from .asr_audio import AudioRecorder, list_input_devices, pcm_bytes_to_float32, RATE, CHANNELS, FORMAT, CHUNK
from .asr_recognizer import WhisperRecognizer, DEFAULT_INITIAL_PROMPT

__all__ = [
    "AudioRecorder",
    "list_input_devices",
    "pcm_bytes_to_float32",
    "RATE",
    "CHANNELS",
    "FORMAT",
    "CHUNK",
    "WhisperRecognizer",
    "DEFAULT_INITIAL_PROMPT",
]
