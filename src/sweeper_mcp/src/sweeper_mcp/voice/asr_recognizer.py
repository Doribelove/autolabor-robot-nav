#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""whisper 离线识别核心模块 —— 纯逻辑、不依赖 ROS。

封装 whisper 模型加载与识别：
  - 模型默认 small（中文准确率/体积平衡点），可用 --model 覆盖
  - device 自动探测 CUDA/CPU；CUDA 下用 fp16 加速
  - 语言默认 zh（中文清扫指令），--language auto 可自动检测
  - 懒加载：首次 transcribe 才 load_model，避免 CLI 启动等待
  - 简体中文保证：whisper 对中文有时输出繁体，用「简化字提示词 initial_prompt」
    + OpenCC 繁转简(t2s)后处理双重保证输出简体

后续可无痛替换为 sherpa-onnx / Paraformer（征程 6 部署路线），
只要保持 transcribe() 接口一致即可。
"""

import whisper

# 简化字提示词：短提示词即可引导模型按简体输出。
# 注意：不能用长句——whisper 在低置信度/短音频上会把 initial_prompt 整句回显
# （实测"说你好识别成提示词"就是长句回显导致）。"简体中文" 4 字零回显。
DEFAULT_INITIAL_PROMPT = "简体中文"


class WhisperRecognizer(object):
    """whisper 离线识别封装。"""

    def __init__(self, model="small", language="zh", device=None,
                 initial_prompt=None, to_simplified=True):
        self.model_name = model
        self.language = language
        self.initial_prompt = DEFAULT_INITIAL_PROMPT if initial_prompt is None else initial_prompt
        self.to_simplified = to_simplified
        self._device = device
        self._model = None
        self._cc = None

    @property
    def _simplifier(self):
        """懒加载 OpenCC 繁转简转换器。"""
        if self._cc is None:
            from opencc import OpenCC
            self._cc = OpenCC("t2s")
        return self._cc

    def _simplify(self, text):
        """繁转简（OpenCC 只转换汉字，不影响英文/数字）。"""
        if not self.to_simplified or not text:
            return text
        try:
            return self._simplifier.convert(text)
        except Exception:
            return text

    @property
    def device(self):
        if self._device in (None, "auto"):
            import torch
            self._device = "cuda" if torch.cuda.is_available() else "cpu"
        return self._device

    @property
    def loaded(self):
        return self._model is not None

    def load(self):
        """加载 whisper 模型（懒加载 + 缓存）。"""
        if self._model is None:
            self._model = whisper.load_model(self.model_name, device=self.device)
        return self._model

    def transcribe(self, audio, language=None):
        """识别音频，返回去除首尾空白的简体中文文本。

        audio: float32 [N] numpy 数组（16k 单声道）或音频文件路径。
        language 传 "auto" 时自动检测语种。
        """
        model = self.load()
        lang = language or self.language
        if lang == "auto":
            lang = None
        # 中文场景注入简化字提示词，引导模型按简体输出；
        # initial_prompt=""（空串）表示完全禁用提示词，回显风险为零。
        prompt = self.initial_prompt if lang in (None, "zh") else None
        if not prompt:
            prompt = None
        result = model.transcribe(
            audio,
            language=lang,
            fp16=(self.device == "cuda"),
            verbose=False,
            initial_prompt=prompt,
        )
        return self._simplify(result["text"].strip())

    def unload(self):
        """释放模型与显存（CLI 退出时调用）。"""
        self._model = None
        if self.device == "cuda":
            import torch
            torch.cuda.empty_cache()
