#!/usr/bin/env python3
"""Real-time PCM replay through production worker/node; cloud plans, no ROS I/O.

Source devel/setup.bash and run with runtime/asr/venv/bin/python3. The bare
node fixture creates no publishers/services, and CatalogMCP rejects all tools
except region enumeration. The optional cloud request is explicitly opt-in.
"""
import argparse
import hashlib
import importlib.util
import json
import sys
import threading
import time
import wave
from pathlib import Path

# The ASR venv is intentionally isolated; only the offline node harness needs
# Ubuntu's ROS Python dependencies. This does not alter the production worker.
sys.path.append("/usr/lib/python3/dist-packages")
import yaml

ROOT = Path(__file__).resolve().parents[1]


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


fixture = module("voice_fixture", ROOT / "src/sweeper_mcp/scripts/test_asr_authorization.py")
worker_module = module("voice_worker", ROOT / "src/sweeper_mcp/scripts/whisper_asr_worker.py")
from sweeper_mcp.agent import AgentRunner
from sweeper_mcp.tools import TOOL_SPECS

SOURCE_HASHES = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                 for p in [ROOT / "src/sweeper_mcp/scripts/whisper_asr_worker.py",
                           ROOT / "src/sweeper_mcp/scripts/sweeper_ai_node.py",
                           ROOT / "src/sweeper_mcp/src/sweeper_mcp/voice_turns.py",
                           ROOT / "src/sweeper_mcp/src/sweeper_mcp/agent.py",
                           ROOT / "src/sweeper_mcp/src/sweeper_mcp/navigation_grounding.py",
                           ROOT / "src/sweeper_mcp/src/sweeper_mcp/region_catalog.py",
                           ROOT / "src/sweeper_mcp/src/sweeper_mcp/tools.py"]}


class CatalogMCP:
    def __init__(self):
        path = ROOT / "global_maps/map_sets/map_20260829_221335_gimp_cleaned/coverage_regions/fused/regions.json"
        raw = json.loads(path.read_text())
        self.catalog = {"map_digest": raw["map_digest"], "regions": [
            {"id": r["id"], "name": r["name"]} for r in raw["regions"]]}

    def initialize(self):
        return {}

    def list_tools(self):
        return {"tools": TOOL_SPECS}

    def call_tool(self, name, arguments=None, **kwargs):
        if name != "list_saved_coverage_regions":
            raise AssertionError("Replay forbids tool execution: " + name)
        return {"text": json.dumps(self.catalog, ensure_ascii=False), "is_error": False}


class PreviewRunner(AgentRunner):
    def _run(self, text, execute=False):
        assert execute is False
        started = time.monotonic()
        plan = self.plan(text)
        return {"plan": plan, "results": [], "answer": plan["answer"],
                "state": "NEEDS_CLARIFICATION" if plan.get("needs_clarification") else "PREVIEW",
                "total_ms": (time.monotonic() - started) * 1000}


class PacedPCM:
    def __init__(self, pcm, stop):
        self.pcm, self.stop, self.offset = pcm, stop, 0
        self.started = time.monotonic()
        self.stdout = self

    def read(self, size):
        if self.stop.is_set():
            return b""
        target = self.started + (self.offset + size) / 32000.0
        self.stop.wait(max(0.0, target - time.monotonic()))
        if self.stop.is_set():
            return b""
        chunk = self.pcm[self.offset:self.offset + size]
        self.offset += size
        return chunk + b"\0" * (size - len(chunk))

    def close(self):
        pass

    def poll(self):
        return None


def normalize(text):
    return "".join(c.lower() for c in text if c.isalnum())


def distance(a, b):
    row = list(range(len(b) + 1))
    for i, left in enumerate(a, 1):
        new = [i]
        for j, right in enumerate(b, 1):
            new.append(min(new[-1] + 1, row[j] + 1, row[j - 1] + (left != right)))
        row = new
    return row[-1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--wav", required=True)
    parser.add_argument("--reference", required=True)
    parser.add_argument("--model", choices=("small", "medium", "large"), default="medium")
    parser.add_argument("--output", required=True)
    parser.add_argument("--cloud", action="store_true")
    parser.add_argument("--legacy-segmentation", action="store_true")
    args = parser.parse_args()
    output = Path(args.output)
    if output.exists():
        raise SystemExit("Refusing to overwrite " + str(output))
    with wave.open(args.wav, "rb") as wav:
        assert (wav.getnchannels(), wav.getsampwidth(), wav.getframerate()) == (1, 2, 16000)
        pcm = wav.readframes(wav.getnframes())
    node = fixture._bare_node()
    node._voice_authorized = node._parse_authorized = True
    node._smart_voice_enabled = node._smart_voice_listening = True
    node._smart_voice_session_id = "offline-replay"
    node._smart_voice_epoch = 1
    node._backend = "offline-replay"
    captures, plans, events, cloud_responses = [], [], [], []
    event_lock = threading.Lock()
    began = time.monotonic()

    def emit(kind, **fields):
        with event_lock:
            events.append(dict(at=time.monotonic() - began, kind=kind, **fields))

    node._emit = emit
    if args.cloud:
        config = yaml.safe_load((ROOT / "src/sweeper_mcp/config/sweeper_mcp.yaml").read_text())
        agent = config["agent"]
        node._runner = PreviewRunner(
            base_url=agent.get("base_url", "https://api.deepseek.com"),
            api_key=agent.get("api_key", ""), model=agent.get("model", "deepseek-v4-flash"),
            temperature=0.0, timeout_s=30.0, mcp=CatalogMCP(),
            cloud_authorization_checker=lambda: True)
        chat = node._runner.llm.chat
        def audited_chat(*args, **kwargs):
            response = chat(*args, **kwargs)
            cloud_responses.append(response)
            return response
        node._runner.llm.chat = audited_chat

    def begin_request(text, source, emit_transcript=True):
        assert source == "ASR" and not node._control_authorized
        node._task_active = True

        def plan():
            row = {"text": text, "at": time.monotonic() - began}
            try:
                if args.cloud:
                    row["result"] = node._runner.run(text, execute=False)
                print("TURN " + text, flush=True)
                if "result" in row:
                    print("PLAN " + json.dumps(row["result"], ensure_ascii=False), flush=True)
            except Exception as exc:
                row["error"] = str(exc)
                print("PLAN ERROR " + str(exc), flush=True)
            finally:
                plans.append(row)
                with node._lock:
                    node._task_active = False
                node._dispatch_next_smart_voice()
        threading.Thread(target=plan, daemon=True).start()
        return True, "offline-plan", "accepted"

    node._begin_request = begin_request

    def worker_event(event):
        if event.get("event") == "smart_transcript":
            payload = event["result"]
            captures.append(dict(payload, at=time.monotonic() - began))
            print("ASR " + json.dumps(payload, ensure_ascii=False), flush=True)
            if args.legacy_segmentation and payload.get("accepted"):
                with node._lock:
                    node._smart_voice_pending.append({
                        "epoch": 1, "session_id": "offline-replay", "utterance_id": payload["utterance_id"],
                        "utterance_index": payload["utterance_index"], "transcript": payload["transcript"],
                        "enqueued_at": time.monotonic()})
                node._dispatch_next_smart_voice()
                return
        node._asr_event(event)

    spec = fixture._MODULE.ASR_MODEL_SPECS[args.model]
    worker = worker_module.WhisperAsrWorker({
        "model": args.model, "device": "cuda",
        "model_path": str(ROOT / "runtime/asr/models" / spec["filename"]),
        "model_sha256": spec["sha256"], "input_device": "plughw:CARD=Audio,DEV=0",
        "initial_prompt": "简体中文，无人清扫车语音指令",
        "vad_backend": "energy" if args.legacy_segmentation else "webrtc",
        "smart_silence_ms": 600.0 if args.legacy_segmentation else 1000.0,
    }, event_callback=worker_event)
    stop = threading.Event()

    def heartbeat():
        while not stop.wait(0.1):
            worker.heartbeat()
            node._last_heartbeat = time.monotonic()
    threading.Thread(target=heartbeat, daemon=True).start()
    try:
        worker.heartbeat()
        prepared = worker.prepare_model()
        print("PREPARED " + json.dumps(prepared, ensure_ascii=False), flush=True)
        began = time.monotonic()
        stream = PacedPCM(pcm, stop)
        with worker._lock:
            worker._capture_generation += 1
            generation = worker._capture_generation
            worker._smart_generation = generation
            worker._smart_session_id = "offline-replay"
            worker._smart_listening = True
            worker._smart_proc = stream
        capture = threading.Thread(target=worker._smart_capture_loop,
                                   args=(stream, generation, "offline-replay"), daemon=True)
        capture.start()
        duration = len(pcm) / 32000.0
        while time.monotonic() - began < duration + 75:
            if not args.legacy_segmentation:
                node._poll_voice_turn()
            elapsed = time.monotonic() - began
            if elapsed > duration + 8 and not (
                    worker._smart_queue or worker._inference_active or node._task_active or
                    node._voice_buffer.parts or node._smart_voice_pending):
                break
            stop.wait(0.1)
        reference = json.loads(Path(args.reference).read_text())["text"]
        hypothesis = "".join(r["transcript"] for r in captures if r["accepted"])
        normal_ref, normal_hyp = normalize(reference), normalize(hypothesis)
        result = {"model": args.model, "wav": str(Path(args.wav).resolve()),
                  "source_sha256": SOURCE_HASHES,
                  "audio_sha256": hashlib.sha256(Path(args.wav).read_bytes()).hexdigest(),
                  "reference": reference, "reference_kind": "large-v3 machine transcript, not human ground truth",
                  "hypothesis": hypothesis, "cer_against_large_reference": distance(normal_ref, normal_hyp) / len(normal_ref),
                  "legacy_segmentation": args.legacy_segmentation, "prepared": prepared,
                  "captures": captures, "plans": plans, "events": events,
                  "cloud_responses": cloud_responses,
                  "pending_at_end": bool(node._task_active or worker._inference_active or
                                         worker._smart_queue or node._voice_buffer.parts or node._smart_voice_pending)}
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("x") as stream_out:
            json.dump(result, stream_out, ensure_ascii=False, indent=2)
        print("RESULT CER=%.4f clips=%d turns=%d pending=%s" % (
            result["cer_against_large_reference"], len(captures), len(plans), result["pending_at_end"]), flush=True)
    finally:
        with worker._lock:
            worker._smart_listening = False
            worker._smart_proc = None
        stop.set()
        worker.shutdown()


if __name__ == "__main__":
    main()
