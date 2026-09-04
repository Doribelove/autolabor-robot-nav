"""Isolated LocateAnything-3B JSON-lines inference worker.

The downloaded checkpoint declares Python 3.10 and Transformers 4.57.1.  The
JetPack 5 CUDA wheel available on this robot is tied to Python 3.8.  This
worker therefore loads the checkpoint's Qwen2 path through a narrow adapter:
annotations are postponed and unused Qwen3/LoRA/video-only imports are stubbed
when absent.  No checkpoint source file is modified.  A full manifest check
and an offline CUDA inference load remain mandatory.
"""

import argparse
import base64
from hashlib import sha256
from io import BytesIO
import json
import os
from pathlib import Path
import sys
import time
import types
from typing import Dict, Iterable, Mapping

from .locateanything_runtime import (
    parse_categories,
    parse_locateanything_answer,
    validate_max_image_side,
    verify_model_manifest,
)


_PACKAGE_NAME = "autolabor_locateanything_model"


def _version_tuple(value: str):
    numbers = []
    for part in str(value).split("+")[0].split("."):
        digits = "".join(character for character in part if character.isdigit())
        if not digits:
            break
        numbers.append(int(digits))
    return tuple(numbers)


def _install_qwen3_stubs(torch) -> None:
    try:
        __import__("transformers.models.qwen3.configuration_qwen3")
        __import__("transformers.models.qwen3.modeling_qwen3")
        return
    except ImportError:
        pass

    from transformers.configuration_utils import PretrainedConfig

    package = types.ModuleType("transformers.models.qwen3")
    package.__path__ = []
    configuration = types.ModuleType(
        "transformers.models.qwen3.configuration_qwen3"
    )
    modeling = types.ModuleType("transformers.models.qwen3.modeling_qwen3")

    class Qwen3Config(PretrainedConfig):
        model_type = "qwen3-unavailable"

    class Qwen3ForCausalLM(torch.nn.Module):
        def __init__(self, *args, **kwargs):
            super().__init__()
            raise RuntimeError(
                "this checkpoint unexpectedly requested Qwen3; the JetPack adapter "
                "only permits its declared Qwen2ForCausalLM architecture"
            )

    configuration.Qwen3Config = Qwen3Config
    modeling.Qwen3ForCausalLM = Qwen3ForCausalLM
    sys.modules[package.__name__] = package
    sys.modules[configuration.__name__] = configuration
    sys.modules[modeling.__name__] = modeling


def _install_optional_stubs() -> None:
    try:
        __import__("peft")
    except ImportError:
        peft = types.ModuleType("peft")

        class LoraConfig:
            def __init__(self, *args, **kwargs):
                self.args = args
                self.kwargs = kwargs

        def get_peft_model(*args, **kwargs):
            raise RuntimeError("LoRA is disabled for the deployed LocateAnything checkpoint")

        peft.LoraConfig = LoraConfig
        peft.get_peft_model = get_peft_model
        sys.modules["peft"] = peft

    try:
        __import__("lmdb")
    except ImportError:
        lmdb = types.ModuleType("lmdb")

        def unavailable_lmdb(*args, **kwargs):
            raise RuntimeError("LMDB input is not enabled in the ROS image worker")

        lmdb.open = unavailable_lmdb
        sys.modules["lmdb"] = lmdb

    try:
        __import__("decord")
    except ImportError:
        decord = types.ModuleType("decord")

        class UnavailableVideoReader:
            def __init__(self, *args, **kwargs):
                raise RuntimeError("video input is not enabled in the ROS image worker")

        decord.VideoReader = UnavailableVideoReader
        sys.modules["decord"] = decord


def _load_source_module(root: Path, name: str):
    full_name = "{}.{}".format(_PACKAGE_NAME, name)
    path = root / "{}.py".format(name)
    source = path.read_text(encoding="utf-8")
    # AutoImageProcessor.register had a different signature in Transformers
    # 4.46.  The worker instantiates this class directly, so registration is
    # unnecessary and is the sole source transformation beyond annotations.
    source = source.replace(
        'AutoImageProcessor.register("LocateAnythingImageProcessor", LocateAnythingImageProcessor)',
        "# Directly instantiated by the JetPack worker.",
    )
    if name == "modeling_qwen2":
        mask_branch = 'elif self._attn_implementation == "sdpa":'
        if source.count(mask_branch) != 1:
            raise RuntimeError(
                "unexpected LocateAnything Qwen2 attention-mask source layout"
            )
        source = source.replace(
            mask_branch,
            'elif self._attn_implementation in ("sdpa", "eager"):',
        )
    source = "from __future__ import annotations\n" + source
    module = types.ModuleType(full_name)
    module.__file__ = str(path)
    module.__package__ = _PACKAGE_NAME
    sys.modules[full_name] = module
    exec(compile(source, str(path), "exec"), module.__dict__)
    return module


def _load_checkpoint_classes(root: Path, torch):
    package = types.ModuleType(_PACKAGE_NAME)
    package.__path__ = [str(root)]
    package.__package__ = _PACKAGE_NAME
    sys.modules[_PACKAGE_NAME] = package
    _install_qwen3_stubs(torch)
    _install_optional_stubs()
    modules = {}
    for name in (
        "configuration_qwen2",
        "configuration_locateanything",
        "generate_utils",
        "mask_sdpa_utils",
        "mask_magi_utils",
        "modeling_qwen2",
        "modeling_vit",
        "modeling_locateanything",
        "image_processing_locateanything",
        "processing_locateanything",
    ):
        modules[name] = _load_source_module(root, name)
    return modules


def _semantic_prompt_parts(processor, categories):
    """Build the immutable text prompt around one variable image-token span."""
    prompts = "</c>".join(
        prompt
        for category in categories
        for prompt in category.grounding_prompts
    )
    question = (
        "Locate all the instances that matches the following description: {}."
        .format(prompts)
    )
    messages = [
        {
            "role": "user",
            "content": [
                {"type": "image"},
                {"type": "text", "text": question},
            ],
        }
    ]
    template = processor.py_apply_chat_template(
        messages, tokenize=False, add_generation_prompt=True
    )
    placeholder = "<{}-1>".format(processor.image_placeholder)
    if template.count(placeholder) != 1:
        raise RuntimeError(
            "LocateAnything semantic template must contain exactly one image placeholder"
        )
    before_image, after_image = template.split(placeholder, 1)
    prefix = before_image + "<image 1>" + processor.image_start_token
    suffix = processor.image_end_token + after_image
    digest = sha256(template.encode("utf-8")).hexdigest()
    return prefix, suffix, template, digest


def _resize_source_image(image, max_image_side: int):
    """Apply only the explicitly configured adapter-level size limit."""
    original_width, original_height = image.size
    longest = max(original_width, original_height)
    if max_image_side <= 0 or longest <= max_image_side:
        return image
    scale = float(max_image_side) / float(longest)
    resized = (
        max(1, int(round(original_width * scale))),
        max(1, int(round(original_height * scale))),
    )
    return image.resize(resized)


class LocateAnythingModel:
    def __init__(self, model_root: str):
        import torch
        import transformers
        from transformers import AutoTokenizer

        if _version_tuple(torch.__version__) < (2, 0):
            raise RuntimeError("LocateAnything requires PyTorch 2.0 or newer")
        if _version_tuple(transformers.__version__) < (4, 46):
            raise RuntimeError("LocateAnything JetPack adapter requires Transformers 4.46+")
        if not torch.cuda.is_available():
            raise RuntimeError("LocateAnything requires the Jetson CUDA PyTorch runtime")
        capability = tuple(torch.cuda.get_device_capability(0))
        if capability < (8, 0):
            raise RuntimeError(
                "LocateAnything bfloat16 requires CUDA compute capability 8.0+; got {}.{}".format(
                    *capability
                )
            )
        supports_bf16 = getattr(torch.cuda, "is_bf16_supported", None)
        if callable(supports_bf16) and not supports_bf16():
            raise RuntimeError("the active CUDA PyTorch build does not support bfloat16")

        root = Path(model_root).resolve()
        config_data = json.loads((root / "config.json").read_text(encoding="utf-8"))
        architecture = (
            (config_data.get("text_config") or {}).get("architectures") or [""]
        )[0]
        if architecture != "Qwen2ForCausalLM":
            raise RuntimeError(
                "JetPack compatibility adapter refuses unexpected architecture {}".format(
                    architecture
                )
            )
        if int(config_data.get("use_backbone_lora", 0)) or int(
            config_data.get("use_llm_lora", 0)
        ):
            raise RuntimeError("JetPack compatibility adapter refuses LoRA checkpoints")

        print(
            "loading LocateAnything with torch={} transformers={} cuda={} device={}".format(
                torch.__version__,
                transformers.__version__,
                torch.version.cuda,
                torch.cuda.get_device_name(0),
            ),
            file=sys.stderr,
            flush=True,
        )
        modules = _load_checkpoint_classes(root, torch)
        if _version_tuple(torch.__version__) < (2, 1, 1):
            # JetPack torch 2.0 lacks CUDA bicubic interpolation for BF16.
            # Position embeddings are conventionally interpolated in FP32;
            # cast only that operation and return to the activation dtype.
            functional = torch.nn.functional
            position_class = modules[
                "modeling_vit"
            ].Learnable2DInterpPosEmb

            def jetpack_position_forward(instance, x, grid_hws):
                position_embeddings = []
                for shape in grid_hws.tolist():
                    if shape == list(instance.weight.shape[:-1]):
                        value = instance.weight.flatten(end_dim=1)
                    else:
                        value = (
                            functional.interpolate(
                                instance.weight.float()
                                .permute((2, 0, 1))
                                .unsqueeze(0),
                                size=shape,
                                mode=instance.interpolation_mode,
                            )
                            .squeeze(0)
                            .permute((1, 2, 0))
                            .flatten(end_dim=1)
                            .to(dtype=x.dtype)
                        )
                    position_embeddings.append(value.to(dtype=x.dtype))
                return x + torch.cat(position_embeddings)

            position_class.forward = jetpack_position_forward
        config_class = modules[
            "configuration_locateanything"
        ].LocateAnythingConfig
        model_class = modules[
            "modeling_locateanything"
        ].LocateAnythingForConditionalGeneration
        image_processor_class = modules[
            "image_processing_locateanything"
        ].LocateAnythingImageProcessor
        processor_class = modules["processing_locateanything"].LocateAnythingProcessor

        config = config_class(**config_data)
        # Transformers 4.46 deliberately gates its SDPA integration on
        # torch>=2.1.1.  JetPack 5 supplies torch 2.0; both checkpoint model
        # implementations expose an eager path, so use that path without
        # weakening any model-file or CUDA checks.
        attention_implementation = (
            "sdpa" if _version_tuple(torch.__version__) >= (2, 1, 1) else "eager"
        )
        config._attn_implementation = attention_implementation
        config._attn_implementation_autoset = False
        config.vision_config._attn_implementation = attention_implementation
        config.vision_config._attn_implementation_autoset = False
        config.text_config._attn_implementation = attention_implementation
        config.text_config._attn_implementation_autoset = False
        print(
            "LocateAnything attention implementation: {}".format(
                attention_implementation
            ),
            file=sys.stderr,
            flush=True,
        )

        tokenizer = AutoTokenizer.from_pretrained(
            str(root),
            local_files_only=True,
            trust_remote_code=False,
        )
        image_processor_data = json.loads(
            (root / "preprocessor_config.json").read_text(encoding="utf-8")
        )
        for key in ("auto_map", "image_processor_type", "processor_class"):
            image_processor_data.pop(key, None)
        image_processor = image_processor_class(**image_processor_data)
        processor_data = json.loads(
            (root / "processor_config.json").read_text(encoding="utf-8")
        )
        for key in ("auto_map", "processor_class"):
            processor_data.pop(key, None)
        self.processor = processor_class(
            image_processor=image_processor,
            tokenizer=tokenizer,
            **processor_data,
        )
        self.tokenizer = tokenizer
        self.device = "cuda:0"
        self.dtype = torch.bfloat16
        self.model = model_class.from_pretrained(
            str(root),
            config=config,
            torch_dtype=self.dtype,
            local_files_only=True,
            use_safetensors=True,
        ).to(self.device).eval()
        self.torch = torch
        self.categories = ()
        self.semantic_prompt_preloaded = False
        self.semantic_prompt_sha256 = ""
        self.semantic_prompt_token_count = 0
        self._prompt_prefix_ids = None
        self._prompt_suffix_ids = None
        self._prompt_tensor_cache = {}
        torch.backends.cuda.matmul.allow_tf32 = True
        print("LocateAnything checkpoint loaded", file=sys.stderr, flush=True)

    def configure(
        self,
        categories,
        generation_mode: str,
        max_new_tokens: int,
        temperature: float,
        max_image_side: int,
        max_detections: int,
        min_box_area_fraction: float,
        max_box_area_fraction: float,
    ):
        """Preload and tokenize immutable semantics before accepting frames."""
        if self.semantic_prompt_preloaded:
            raise RuntimeError("LocateAnything semantic prompt is already configured")
        categories = tuple(categories)
        generation_mode = str(generation_mode).strip().lower()
        max_new_tokens = int(max_new_tokens)
        temperature = float(temperature)
        max_image_side = validate_max_image_side(max_image_side)
        max_detections = int(max_detections)
        min_box_area_fraction = float(min_box_area_fraction)
        max_box_area_fraction = float(max_box_area_fraction)
        if generation_mode not in ("fast", "slow", "hybrid"):
            raise ValueError("generation_mode must be fast, slow, or hybrid")
        if not 32 <= max_new_tokens <= 4096:
            raise ValueError("max_new_tokens must be in [32, 4096]")
        if not 1 <= max_detections <= 300:
            raise ValueError("max_detections must be in [1, 300]")
        if not 0.0 <= min_box_area_fraction < max_box_area_fraction <= 1.0:
            raise ValueError("box-area fractions are invalid")

        prefix, suffix, _template, digest = _semantic_prompt_parts(
            self.processor, categories
        )
        prefix_ids = self.tokenizer(
            prefix,
            return_tensors="pt",
            padding=False,
            add_special_tokens=False,
        )["input_ids"]
        suffix_ids = self.tokenizer(
            suffix,
            return_tensors="pt",
            padding=False,
            add_special_tokens=False,
        )["input_ids"]
        image_token_id = int(self.processor.image_token_id)
        if image_token_id < 0:
            raise RuntimeError("LocateAnything tokenizer has no image token")

        # Prove that splitting around the special image-token span is exactly
        # equivalent to the upstream processor's full-string tokenization.
        probe_text = prefix + self.processor.image_token + suffix
        expected_probe = self.tokenizer(
            probe_text, return_tensors="pt", padding=False
        )["input_ids"]
        actual_probe = self.torch.cat(
            (
                prefix_ids,
                self.torch.full(
                    (1, 1), image_token_id, dtype=prefix_ids.dtype
                ),
                suffix_ids,
            ),
            dim=1,
        )
        if not self.torch.equal(expected_probe, actual_probe):
            raise RuntimeError(
                "LocateAnything semantic prompt cannot be safely split for caching"
            )

        self.categories = categories
        self.generation_mode = generation_mode
        self.max_new_tokens = max_new_tokens
        self.temperature = temperature
        self.max_image_side = max_image_side
        self.max_detections = max_detections
        self.min_box_area_fraction = min_box_area_fraction
        self.max_box_area_fraction = max_box_area_fraction
        self.semantic_prompt_sha256 = digest
        self.semantic_prompt_token_count = int(
            prefix_ids.numel() + suffix_ids.numel()
        )
        self._prompt_prefix_ids = prefix_ids.to(self.device)
        self._prompt_suffix_ids = suffix_ids.to(self.device)
        self._prompt_tensor_cache.clear()
        self.semantic_prompt_preloaded = True
        print(
            "LocateAnything semantic instruction preloaded: categories={} "
            "queries={} fixed_tokens={} input_limit={} sha256={}".format(
                ",".join(category.class_name for category in categories),
                sum(len(category.grounding_prompts) for category in categories),
                self.semantic_prompt_token_count,
                self.max_image_side or "native",
                self.semantic_prompt_sha256,
            ),
            file=sys.stderr,
            flush=True,
        )
        return {
            "semantic_prompt_preloaded": True,
            "semantic_prompt_sha256": self.semantic_prompt_sha256,
            "semantic_prompt_token_count": self.semantic_prompt_token_count,
            "category_count": len(self.categories),
            "query_count": sum(
                len(category.grounding_prompts)
                for category in self.categories
            ),
        }

    def _prompt_tensors(self, num_image_tokens: int):
        if not self.semantic_prompt_preloaded:
            raise RuntimeError("LocateAnything semantic prompt is not configured")
        num_image_tokens = int(num_image_tokens)
        if num_image_tokens < 1:
            raise ValueError("LocateAnything image token count must be positive")
        cached = self._prompt_tensor_cache.get(num_image_tokens)
        if cached is not None:
            return cached[0], cached[1], True
        image_ids = self.torch.full(
            (1, num_image_tokens),
            int(self.processor.image_token_id),
            dtype=self._prompt_prefix_ids.dtype,
            device=self.device,
        )
        input_ids = self.torch.cat(
            (self._prompt_prefix_ids, image_ids, self._prompt_suffix_ids), dim=1
        )
        attention_mask = self.torch.ones_like(input_ids)
        if len(self._prompt_tensor_cache) >= 16:
            self._prompt_tensor_cache.pop(next(iter(self._prompt_tensor_cache)))
        self._prompt_tensor_cache[num_image_tokens] = (
            input_ids,
            attention_mask,
        )
        return input_ids, attention_mask, False

    def predict(self, image):
        if not self.semantic_prompt_preloaded:
            raise RuntimeError("LocateAnything semantic prompt is not configured")
        original_width, original_height = image.size
        image = _resize_source_image(image, self.max_image_side)
        image_inputs = self.processor.image_processor(
            images=[image], return_tensors="pt"
        )
        pixel_values = image_inputs["pixel_values"].to(
            device=self.device, dtype=self.dtype
        )
        image_grid_hws = image_inputs["image_grid_hws"]
        if not self.torch.is_tensor(image_grid_hws):
            image_grid_hws = self.torch.as_tensor(image_grid_hws)
        grid_values = image_grid_hws.tolist()
        if len(grid_values) != 1 or len(grid_values[0]) != 2:
            raise RuntimeError("LocateAnything expects exactly one image grid")
        merge_area = int(
            self.processor.image_processor.merge_kernel_size[0]
            * self.processor.image_processor.merge_kernel_size[1]
        )
        grid_area = int(grid_values[0][0]) * int(grid_values[0][1])
        if grid_area < 1 or grid_area % merge_area:
            raise RuntimeError("LocateAnything image grid is incompatible with token merge")
        input_ids, attention_mask, prompt_cache_hit = self._prompt_tensors(
            grid_area // merge_area
        )
        image_grid_hws = image_grid_hws.to(self.device)
        self.torch.manual_seed(0)
        self.torch.cuda.manual_seed_all(0)
        with self.torch.inference_mode():
            response = self.model.generate(
                pixel_values=pixel_values,
                input_ids=input_ids,
                attention_mask=attention_mask,
                image_grid_hws=image_grid_hws,
                tokenizer=self.tokenizer,
                max_new_tokens=self.max_new_tokens,
                use_cache=True,
                generation_mode=self.generation_mode,
                temperature=self.temperature,
                do_sample=self.temperature > 0.0,
                top_p=0.9,
                repetition_penalty=1.1,
                verbose=False,
            )
        answer = response[0] if isinstance(response, tuple) else response
        detections, ignored = parse_locateanything_answer(
            answer,
            self.categories,
            original_width,
            original_height,
            max_detections=self.max_detections,
            min_box_area_fraction=self.min_box_area_fraction,
            max_box_area_fraction=self.max_box_area_fraction,
        )
        return (
            answer,
            detections,
            ignored,
            prompt_cache_hit,
            len(self._prompt_tensor_cache),
        )


def _read_request(line: bytes) -> Mapping[str, object]:
    if len(line) > 64 * 1024 * 1024:
        raise ValueError("LocateAnything worker request exceeds 64 MiB")
    request = json.loads(line.decode("utf-8"))
    if not isinstance(request, dict):
        raise ValueError("worker request must be an object")
    return request


def _send(payload: Mapping[str, object]) -> None:
    sys.stdout.buffer.write(
        json.dumps(payload, ensure_ascii=True, separators=(",", ":")).encode("utf-8")
        + b"\n"
    )
    sys.stdout.buffer.flush()


def _serve(model, manifest) -> None:
    from PIL import Image
    import torch
    import transformers

    _send(
        {
            "event": "ready",
            "ok": True,
            "model_sha256": manifest.digest,
            "revision": manifest.revision,
            "device": "cuda:0",
            "torch_version": torch.__version__,
            "transformers_version": transformers.__version__,
            "compatibility_adapter": "jetpack5-py38-qwen2-v2-semantic-cache",
            "motion_eligible": False,
        }
    )
    for line in sys.stdin.buffer:
        request_id = None
        operation = None
        try:
            request = _read_request(line)
            operation = request.get("op")
            if operation == "shutdown":
                return
            if operation == "configure":
                categories = parse_categories(request.get("categories", []))
                configured = model.configure(
                    categories,
                    str(request.get("generation_mode", "hybrid")),
                    int(request.get("max_new_tokens", 512)),
                    float(request.get("temperature", 0.0)),
                    int(request.get("max_image_side", 896)),
                    int(request.get("max_detections", 100)),
                    float(request.get("min_box_area_fraction", 0.00005)),
                    float(request.get("max_box_area_fraction", 0.75)),
                )
                _send(dict({"event": "configured", "ok": True}, **configured))
                continue
            if operation != "predict":
                raise ValueError("unsupported worker operation")
            if not model.semantic_prompt_preloaded:
                raise RuntimeError("semantic prompt must be configured before prediction")
            repeated_keys = {
                "categories",
                "generation_mode",
                "max_new_tokens",
                "temperature",
                "max_image_side",
                "max_detections",
                "min_box_area_fraction",
                "max_box_area_fraction",
            }.intersection(request)
            if repeated_keys:
                raise ValueError(
                    "predict request repeated preloaded semantic configuration: {}".format(
                        ",".join(sorted(repeated_keys))
                    )
                )
            request_id = int(request.get("id"))
            encoded = str(request.get("image_jpeg_b64", ""))
            image_bytes = base64.b64decode(encoded, validate=True)
            image = Image.open(BytesIO(image_bytes)).convert("RGB")
            start = time.perf_counter()
            (
                answer,
                detections,
                ignored,
                prompt_cache_hit,
                prompt_cache_entries,
            ) = model.predict(image)
            elapsed_ms = (time.perf_counter() - start) * 1000.0
            _send(
                {
                    "ok": True,
                    "id": request_id,
                    "detections": detections,
                    "ignored_boxes": ignored,
                    "inference_ms": elapsed_ms,
                    "answer": answer,
                    "semantic_prompt_preloaded": True,
                    "prompt_tensor_cache_hit": prompt_cache_hit,
                    "prompt_tensor_cache_entries": prompt_cache_entries,
                }
            )
        except Exception as error:
            print(
                "LocateAnything request failed: {}: {}".format(
                    type(error).__name__, error
                ),
                file=sys.stderr,
                flush=True,
            )
            response = {
                "ok": False,
                "id": request_id,
                "error": "{}: {}".format(type(error).__name__, error),
            }
            if operation == "configure":
                response["event"] = "configured"
            _send(response)


def _parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model-root", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--expected-sha256", required=True)
    parser.add_argument("--verify-only", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = _parse_arguments()
    try:
        manifest = verify_model_manifest(
            args.model_root,
            args.manifest,
            args.expected_sha256,
            verify_files=True,
        )
        if args.verify_only:
            _send(
                {
                    "event": "verified",
                    "ok": True,
                    "model_sha256": manifest.digest,
                    "revision": manifest.revision,
                }
            )
            return
        model = LocateAnythingModel(args.model_root)
        _serve(model, manifest)
    except Exception as error:
        print(
            "LocateAnything worker startup failed: {}: {}".format(
                type(error).__name__, error
            ),
            file=sys.stderr,
            flush=True,
        )
        _send(
            {
                "event": "ready",
                "ok": False,
                "error": "{}: {}".format(type(error).__name__, error),
            }
        )
        raise SystemExit(1)


if __name__ == "__main__":
    main()
