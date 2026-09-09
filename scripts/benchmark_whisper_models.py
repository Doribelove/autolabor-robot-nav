#!/usr/bin/env python3
"""Benchmark the production Whisper decoding options on a WAV manifest."""

import argparse
import gc
import json
import os
from pathlib import Path
import statistics
import time
import unicodedata
import wave


MODEL_FILES = {
    "small": "small.pt",
    "medium": "medium.pt",
    "large": "large-v3.pt",
}


def _read_wav(path, np):
    with wave.open(str(path), "rb") as stream:
        if (stream.getnchannels(), stream.getsampwidth(), stream.getframerate()) != \
                (1, 2, 16000):
            raise RuntimeError("expected 16 kHz mono S16 WAV: %s" % path)
        frames = stream.readframes(stream.getnframes())
    audio = np.frombuffer(frames, dtype="<i2").astype(np.float32)
    return audio / 32768.0


def _normalize(text, converter):
    value = converter(unicodedata.normalize("NFKC", str(text or ""))).upper()
    replacements = (
        ("负三点一", "-3.1"), ("五点二", "5.2"),
        ("九十", "90"), ("两", "2"), ("二", "2"),
    )
    for source, target in replacements:
        value = value.replace(source, target)
    return "".join(char for char in value if char.isalnum() or char in ".-")


def _edit_distance(left, right):
    previous = list(range(len(right) + 1))
    for row, left_char in enumerate(left, 1):
        current = [row]
        for column, right_char in enumerate(right, 1):
            current.append(min(
                current[-1] + 1,
                previous[column] + 1,
                previous[column - 1] + (left_char != right_char),
            ))
        previous = current
    return previous[-1]


def _percentile(values, fraction):
    if not values:
        return 0.0
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    low = int(position)
    high = min(len(ordered) - 1, low + 1)
    weight = position - low
    return ordered[low] * (1.0 - weight) + ordered[high] * weight


def _summarize(rows):
    latencies = [row["latency_ms"] for row in rows]
    edits = sum(row["edit_distance"] for row in rows)
    references = sum(row["reference_characters"] for row in rows)
    slots_hit = sum(row["slots_hit"] for row in rows)
    slots_total = sum(row["slots_total"] for row in rows)
    return {
        "clips": len(rows),
        "cer": edits / float(max(1, references)),
        "exact_rate": sum(bool(row["exact_match"]) for row in rows) /
                      float(max(1, len(rows))),
        "slot_recall": slots_hit / float(max(1, slots_total)),
        "latency_ms": {
            "mean": statistics.mean(latencies) if latencies else 0.0,
            "median": statistics.median(latencies) if latencies else 0.0,
            "p90": _percentile(latencies, 0.90),
            "min": min(latencies) if latencies else 0.0,
            "max": max(latencies) if latencies else 0.0,
        },
        "real_time_factor_mean": statistics.mean(
            row["real_time_factor"] for row in rows) if rows else 0.0,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--model-dir", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--models", nargs="+", choices=tuple(MODEL_FILES),
                        default=list(MODEL_FILES))
    parser.add_argument("--initial-prompt",
                        default="简体中文，无人清扫车语音指令")
    parser.add_argument("--without-timestamps", action="store_true")
    parser.add_argument(
        "--prewarm", action="store_true",
        help="run one silent production-style decode before measured clips")
    args = parser.parse_args()

    import numpy as np
    import torch
    import whisper
    from opencc import OpenCC

    if not torch.cuda.is_available():
        raise SystemExit("CUDA is unavailable; benchmark refuses CPU fallback")
    converter = OpenCC("t2s").convert
    manifest_path = Path(args.manifest).resolve()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    entries = manifest.get("entries") or []
    if not entries:
        raise SystemExit("benchmark manifest has no entries")
    model_dir = Path(args.model_dir).resolve()
    output_path = Path(args.output).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    result = {
        "schema_version": 1,
        "manifest": str(manifest_path),
        "synthetic_audio": True,
        "torch_version": torch.__version__,
        "cuda_version": torch.version.cuda,
        "device": torch.cuda.get_device_name(0),
        "decode_options": {
            "language": "zh",
            "task": "transcribe",
            "fp16": True,
            "initial_prompt": args.initial_prompt,
            "condition_on_previous_text": False,
            "temperature": 0.0,
            "without_timestamps": bool(args.without_timestamps),
            "prewarm": bool(args.prewarm),
        },
        "models": {},
    }

    for model_name in args.models:
        checkpoint = model_dir / MODEL_FILES[model_name]
        if not checkpoint.is_file():
            raise SystemExit("missing checkpoint: %s" % checkpoint)
        torch.cuda.empty_cache()
        torch.cuda.synchronize()
        load_started = time.monotonic()
        model = whisper.load_model(str(checkpoint), device="cuda")
        torch.cuda.synchronize()
        load_ms = (time.monotonic() - load_started) * 1000.0
        print("%s loaded in %.0f ms" % (model_name, load_ms), flush=True)

        warmup_ms = 0.0
        if args.prewarm:
            warmup_audio = np.zeros(16000 // 2, dtype=np.float32)
            torch.cuda.synchronize()
            warmup_started = time.monotonic()
            model.transcribe(
                warmup_audio,
                language="zh",
                task="transcribe",
                fp16=True,
                verbose=False,
                initial_prompt=args.initial_prompt or None,
                condition_on_previous_text=False,
                temperature=0.0,
                without_timestamps=bool(args.without_timestamps),
            )
            torch.cuda.synchronize()
            warmup_ms = (time.monotonic() - warmup_started) * 1000.0
            print("%s prewarmed in %.0f ms" % (model_name, warmup_ms), flush=True)

        rows = []
        for entry in entries:
            audio = _read_wav(manifest_path.parent / entry["wav"], np)
            torch.cuda.synchronize()
            started = time.monotonic()
            output = model.transcribe(
                audio,
                language="zh",
                task="transcribe",
                fp16=True,
                verbose=False,
                initial_prompt=args.initial_prompt or None,
                condition_on_previous_text=False,
                temperature=0.0,
                without_timestamps=bool(args.without_timestamps),
            )
            torch.cuda.synchronize()
            latency_ms = (time.monotonic() - started) * 1000.0
            hypothesis = converter(str((output or {}).get("text", ""))).strip()
            reference_normalized = _normalize(entry["text"], converter)
            hypothesis_normalized = _normalize(hypothesis, converter)
            distance = _edit_distance(reference_normalized, hypothesis_normalized)
            slots = entry.get("slots") or []
            slots_hit = sum(
                any(_normalize(alias, converter) in hypothesis_normalized
                    for alias in aliases)
                for aliases in slots)
            duration_s = float(entry.get("duration_s") or 0.0)
            row = {
                "id": entry["id"],
                "case_id": entry.get("case_id", ""),
                "snr_db": entry.get("snr_db"),
                "duration_s": duration_s,
                "reference": entry["text"],
                "hypothesis": hypothesis,
                "reference_normalized": reference_normalized,
                "hypothesis_normalized": hypothesis_normalized,
                "edit_distance": distance,
                "reference_characters": len(reference_normalized),
                "exact_match": reference_normalized == hypothesis_normalized,
                "slots_hit": slots_hit,
                "slots_total": len(slots),
                "latency_ms": latency_ms,
                "real_time_factor": latency_ms / max(1.0, duration_s * 1000.0),
            }
            rows.append(row)
            print(
                "%s %-35s %7.0f ms CER=%5.1f%% slots=%d/%d | %s" % (
                    model_name, entry["id"], latency_ms,
                    100.0 * distance / float(max(1, len(reference_normalized))),
                    slots_hit, len(slots), hypothesis),
                flush=True)

        clean = [row for row in rows if row["snr_db"] is None]
        noisy = [row for row in rows if row["snr_db"] is not None]
        model_result = {
            "checkpoint": str(checkpoint),
            "checkpoint_bytes": checkpoint.stat().st_size,
            "load_ms": load_ms,
            "warmup_ms": warmup_ms,
            "prepare_ms": load_ms + warmup_ms,
            "first_inference_ms": rows[0]["latency_ms"],
            "warm_inference": _summarize(rows[1:]),
            "overall": _summarize(rows),
            "clean": _summarize(clean),
            "noisy": _summarize(noisy),
            "utterances": rows,
        }
        result["models"][model_name] = model_result
        output_path.write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8")
        del model
        gc.collect()
        torch.cuda.empty_cache()

    print("results %s" % output_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
