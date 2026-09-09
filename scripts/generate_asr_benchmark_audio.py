#!/usr/bin/env python3
"""Generate a reproducible Mandarin command corpus with Edge TTS.

The generated media is validation-only.  Compressed TTS output is decoded by
the system GStreamer installation, then the second voice is mixed with
deterministic white noise using only the Python standard library.
"""

import argparse
from array import array
import hashlib
import json
import math
import os
from pathlib import Path
import random
import shutil
import subprocess
import sys
import time
import wave


CASES = (
    {
        "id": "relative_move",
        "text": "向前移动两米，然后左转九十度。",
        "slots": (("向前",), ("两米", "二米", "2米"),
                  ("左转",), ("九十度", "90度")),
    },
    {
        "id": "map_pose",
        "text": "导航到地图坐标，X五点二，Y负三点一，朝向九十度。",
        "slots": (("地图坐标",), ("X五点二", "X5.2"),
                  ("Y负三点一", "Y-3.1"), ("九十度", "90度")),
    },
    {
        "id": "coverage_ab",
        "text": "按A区、B区的顺序执行覆盖清扫。",
        "slots": (("A区",), ("B区",), ("顺序",), ("覆盖清扫",)),
    },
    {
        "id": "coverage_pause",
        "text": "暂停当前覆盖清扫任务。",
        "slots": (("暂停",), ("当前",), ("覆盖清扫",)),
    },
    {
        "id": "coverage_resume_skip",
        "text": "恢复覆盖清扫，然后跳过当前区域。",
        "slots": (("恢复",), ("覆盖清扫",), ("跳过",), ("当前区域",)),
    },
    {
        "id": "status_query",
        "text": "检查机器人状态和定位状态。",
        "slots": (("检查",), ("机器人状态",), ("定位状态",)),
    },
    {
        "id": "stop_spot_cancel_nav",
        "text": "停止定点清扫并取消导航。",
        "slots": (("停止",), ("定点清扫",), ("取消导航",)),
    },
    {
        "id": "map_origin",
        "text": "导航到地图坐标原点。",
        "slots": (("导航",), ("地图坐标原点",)),
    },
)

VOICES = (
    {
        "id": "female_clean",
        "voice": "zh-CN-XiaoxiaoNeural",
        "rate": "-5%",
        "snr_db": None,
    },
    {
        "id": "male_noise18",
        "voice": "zh-CN-YunxiNeural",
        "rate": "+12%",
        "snr_db": 18.0,
    },
)


def _sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _decode_mp3(source, target):
    command = [
        "gst-launch-1.0", "-q",
        "filesrc", "location=%s" % source,
        "!", "decodebin", "!", "audioconvert", "!", "audioresample",
        "!", "audio/x-raw,format=S16LE,channels=1,rate=16000",
        "!", "wavenc", "!", "filesink", "location=%s" % target,
    ]
    subprocess.run(command, check=True)


def _add_white_noise(path, snr_db, seed):
    with wave.open(str(path), "rb") as stream:
        params = stream.getparams()
        frames = stream.readframes(stream.getnframes())
    if params.sampwidth != 2 or params.nchannels != 1 or params.framerate != 16000:
        raise RuntimeError("unexpected benchmark WAV format: %r" % (params,))
    samples = array("h")
    samples.frombytes(frames)
    if not samples:
        raise RuntimeError("empty benchmark WAV: %s" % path)
    signal_rms = math.sqrt(sum(float(value) ** 2 for value in samples) / len(samples))
    noise_rms = signal_rms / (10.0 ** (float(snr_db) / 20.0))
    rng = random.Random(seed)
    for index, value in enumerate(samples):
        mixed = int(round(value + rng.gauss(0.0, noise_rms)))
        samples[index] = max(-32768, min(32767, mixed))
    with wave.open(str(path), "wb") as stream:
        stream.setparams(params)
        stream.writeframes(samples.tobytes())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--edge-tts", default="edge-tts")
    parser.add_argument(
        "--edge-tts-pythonpath",
        help="isolated site-packages directory; invoke edge_tts as a module")
    parser.add_argument("--retries", type=int, default=4)
    parser.add_argument("--keep-mp3", action="store_true")
    args = parser.parse_args()

    edge_tts_env = None
    if args.edge_tts_pythonpath:
        package_dir = str(Path(args.edge_tts_pythonpath).resolve())
        if not (Path(package_dir) / "edge_tts").is_dir():
            raise SystemExit("edge_tts package is unavailable: %s" % package_dir)
        edge_tts_command = [sys.executable, "-m", "edge_tts"]
        edge_tts_env = os.environ.copy()
        previous = edge_tts_env.get("PYTHONPATH", "")
        edge_tts_env["PYTHONPATH"] = package_dir + (os.pathsep + previous if previous else "")
    else:
        edge_tts = shutil.which(args.edge_tts) or args.edge_tts
        if not os.path.isfile(edge_tts) or not os.access(edge_tts, os.X_OK):
            raise SystemExit("edge-tts executable is unavailable: %s" % edge_tts)
        edge_tts_command = [edge_tts]
    if not shutil.which("gst-launch-1.0"):
        raise SystemExit("gst-launch-1.0 is unavailable")

    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    entries = []
    for case_index, case in enumerate(CASES):
        for voice_index, voice in enumerate(VOICES):
            stem = "%02d_%s_%s" % (case_index + 1, case["id"], voice["id"])
            mp3_path = output_dir / (stem + ".mp3")
            wav_path = output_dir / (stem + ".wav")
            command = edge_tts_command + [
                "--voice", voice["voice"],
                "--rate=%s" % voice["rate"],
                "--text", case["text"],
                "--write-media", str(mp3_path),
            ]
            for attempt in range(1, max(1, args.retries) + 1):
                try:
                    if mp3_path.exists():
                        mp3_path.unlink()
                    subprocess.run(command, check=True, env=edge_tts_env)
                    break
                except subprocess.CalledProcessError:
                    if attempt >= max(1, args.retries):
                        raise
                    print(
                        "retrying %s after edge-tts failure (%d/%d)" % (
                            stem, attempt, max(1, args.retries)),
                        flush=True)
                    time.sleep(float(attempt))
            _decode_mp3(mp3_path, wav_path)
            if voice["snr_db"] is not None:
                _add_white_noise(
                    wav_path, voice["snr_db"],
                    seed=20260909 + case_index * 10 + voice_index)
            with wave.open(str(wav_path), "rb") as stream:
                duration_s = stream.getnframes() / float(stream.getframerate())
            entries.append({
                "id": stem,
                "case_id": case["id"],
                "text": case["text"],
                "slots": [list(group) for group in case["slots"]],
                "voice": voice["voice"],
                "rate": voice["rate"],
                "snr_db": voice["snr_db"],
                "wav": wav_path.name,
                "duration_s": duration_s,
                "sha256": _sha256(wav_path),
                "synthetic": True,
            })
            if not args.keep_mp3:
                mp3_path.unlink()
            print("generated %s (%.2f s)" % (wav_path.name, duration_s), flush=True)

    manifest = {
        "schema_version": 1,
        "generator": "edge-tts",
        "notes": (
            "Synthetic Mandarin command corpus. male_noise18 clips contain "
            "deterministic white noise at 18 dB SNR. This is not a field-accuracy corpus."
        ),
        "entries": entries,
    }
    manifest_path = output_dir / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8")
    print("manifest %s (%d clips)" % (manifest_path, len(entries)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
