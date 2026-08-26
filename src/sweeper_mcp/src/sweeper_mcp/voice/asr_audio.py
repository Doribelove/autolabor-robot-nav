#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""人声录音核心模块 —— pyaudio 采集，纯逻辑、不依赖 ROS。

贴合人声的标准录音设置（whisper 原生期望）：
  - 采样率 16000 Hz ：whisper 训练/推理的原生采样率，无需重采样
  - 位深 16-bit PCM (paInt16) ：人声的标准量化格式，体积/信噪比平衡
  - 单声道（mono）  ：人声识别单通道足够，减半带宽与体积
  - CHUNK=1024       ：≈64ms@16k，平衡回调延迟与 CPU 占用

本模块与 ROS 完全解耦，后续 ROS 语音节点可直接复用 AudioRecorder，
把识别文本发布到 /voice/text 即可无缝替换 keyboard_input 节点。
"""

import wave
from collections import deque

import numpy as np
import pyaudio

RATE = 16000
CHANNELS = 1
FORMAT = pyaudio.paInt16
CHUNK = 1024


def list_input_devices():
    """列出全部可用输入设备，供 --input-device 排错。"""
    pa = pyaudio.PyAudio()
    try:
        infos = []
        for i in range(pa.get_device_count()):
            info = pa.get_device_info_by_index(i)
            if info.get("maxInputChannels", 0) > 0:
                infos.append({
                    "index": i,
                    "name": info["name"],
                    "channels": info["maxInputChannels"],
                })
        return infos
    finally:
        pa.terminate()


def pcm_bytes_to_float32(pcm):
    """int16 PCM 字节 -> float32 numpy 数组（归一化到 [-1,1]）。

    whisper 的 transcribe() 直接接受 numpy 数组，可绕开其内部依赖的
    av/ffmpeg 音频解码链路，全程离线本地解码。
    """
    audio = np.frombuffer(pcm, dtype=np.int16).astype(np.float32)
    audio /= 32768.0
    return audio


class AudioRecorder(object):
    """pyaudio 录音器：callback 非阻塞采集，录音期间主线程可继续读终端输入。"""

    def __init__(self, rate=RATE, channels=CHANNELS, format_=FORMAT,
                 chunk=CHUNK, device_index=None):
        self.rate = rate
        self.channels = channels
        self.format = format_
        self.chunk = chunk
        self.device_index = device_index
        self._pa = pyaudio.PyAudio()
        self._stream = None
        self._frames = deque()

    def start(self):
        """开始录音（幂等：已在录音则返回 False）。"""
        if self._stream is not None:
            return False
        self._frames.clear()
        self._stream = self._pa.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            input_device_index=self.device_index,
            frames_per_buffer=self.chunk,
            stream_callback=self._on_audio,
        )
        self._stream.start_stream()
        return True

    def _on_audio(self, in_data, frame_count, time_info, status):
        self._frames.append(in_data)
        return (in_data, pyaudio.paContinue)

    def stop(self):
        """停止录音，返回 (pcm_bytes, 时长秒)。未在录音返回 None。"""
        if self._stream is None:
            return None
        self._stream.stop_stream()
        self._stream.close()
        self._stream = None
        pcm = b"".join(self._frames)
        duration = len(pcm) / (2 * self.channels * self.rate)
        return pcm, duration

    def save_wav(self, path, pcm=None):
        """把 PCM 写入 WAV（16k/mono/16bit）。不传 pcm 时用当前缓冲。"""
        pcm = pcm if pcm is not None else b"".join(self._frames)
        with wave.open(path, "wb") as wf:
            wf.setnchannels(self.channels)
            wf.setsampwidth(self._pa.get_sample_size(self.format))
            wf.setframerate(self.rate)
            wf.writeframes(pcm)

    @property
    def recording(self):
        return self._stream is not None

    def close(self):
        """释放资源：停流 + terminate PyAudio。"""
        if self._stream is not None:
            self.stop()
        self._pa.terminate()
