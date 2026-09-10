#!/usr/bin/env python3
"""Bounded live AI text probe; parse preview only, never enables control/voice."""
import argparse
import json
import os
from pathlib import Path
import threading
import time

import rosgraph
import rospy
from sweeper_mcp.msg import AiControlStatus, AiEvent
from sweeper_mcp.srv import SetAiAuthorization, SubmitAiText


def find_session_token(workspace):
    supplied = os.environ.get("LIVE_AI_SESSION_TOKEN", "")
    supplied_pid = os.environ.get("LIVE_AI_PID", "")
    if supplied and supplied_pid.isdigit():
        return supplied_pid, supplied
    matches = []
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit():
            continue
        try:
            command = (proc / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
            if (workspace + "/devel/lib/sweeper_mcp/sweeper_ai_node.py" not in command or
                    "__name:=sweeper_ai" not in command):
                continue
            environment = dict(item.split("=", 1) for item in
                               (proc / "environ").read_bytes().decode(errors="replace").split("\0")
                               if "=" in item)
            token = environment.get("SWEEPER_AI_SESSION_TOKEN", "")
            if token:
                matches.append((proc.name, token))
        except (OSError, UnicodeError):
            continue
    if len(matches) != 1:
        raise RuntimeError("expected exactly one workspace-owned AI process, found %d" % len(matches))
    return matches[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--text", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()
    output = Path(args.output)
    if output.exists() or output.parent.resolve() != (Path(__file__).resolve().parents[1] / "validation").resolve():
        parser.error("output must be a new direct child of workspace validation/")
    workspace = str(Path(__file__).resolve().parents[1])
    pid, token = find_session_token(workspace)
    rospy.init_node("live_ai_text_probe", anonymous=True, disable_signals=True)
    topics = dict(rosgraph.Master(rospy.get_name()).getPublishedTopics(""))
    if "/cmd_vel" in topics or "/cmd_vel_safe" in topics:
        raise RuntimeError("refusing probe: visual-only graph must have no motion-output publisher")
    before = rospy.wait_for_message("/sweeper_ai/status", AiControlStatus, timeout=5.0)
    if before.control_authorized or before.task_active or not before.ui_session_alive:
        raise RuntimeError("refusing probe: control/task/UI precondition is not safe")
    events, final_event = [], threading.Event()
    request_id = [""]

    def callback(msg):
        if request_id[0] and msg.request_id == request_id[0]:
            events.append({field: getattr(msg, field) for field in
                           ("sequence", "kind", "step_index", "tool", "state", "description",
                            "arguments_json", "result_json", "duration_ms")})
            if msg.kind == "FINAL":
                final_event.set()

    subscriber = rospy.Subscriber("/sweeper_ai/events", AiEvent, callback, queue_size=100)
    rospy.wait_for_service("/sweeper_ai/set_authorization", timeout=5.0)
    rospy.wait_for_service("/sweeper_ai/submit_text", timeout=5.0)
    authorize = rospy.ServiceProxy("/sweeper_ai/set_authorization", SetAiAuthorization)
    submit = rospy.ServiceProxy("/sweeper_ai/submit_text", SubmitAiText)
    started = time.monotonic()
    error = ""
    response = None
    try:
        if not before.parse_authorized:
            changed = authorize(token, "parse", True)
            if not changed.accepted or changed.control_authorized:
                raise RuntimeError("could not enable parse-only preview")
        response = submit(token, "MANUAL", args.text)
        if not response.accepted:
            raise RuntimeError("live node rejected text: " + response.message)
        request_id[0] = response.request_id
        if not final_event.wait(args.timeout):
            raise RuntimeError("timed out waiting for live AI final event")
    except Exception as exc:
        error = str(exc)
    finally:
        if not before.parse_authorized:
            try:
                authorize(token, "parse", False)
            except Exception as exc:
                error += ("; " if error else "") + "failed to restore parse authorization: " + str(exc)
        subscriber.unregister()
    after = rospy.wait_for_message("/sweeper_ai/status", AiControlStatus, timeout=5.0)
    plans = [json.loads(event["result_json"]) for event in events
             if event["kind"] == "PLAN" and event["result_json"]]
    actual = [(step["tool"], step["arguments"]) for plan in plans for step in plan.get("steps", [])]
    expected = [("start_coverage_cleaning", {"regions": ["研究院大楼前大厅"]})]
    passed = (not error and len(plans) == 1 and actual == expected and
              not any(event["kind"] == "STEP" for event in events) and
              not after.voice_authorized and not after.parse_authorized and
              not after.control_authorized and not after.task_active)
    record = {
        "scope": "live visual-only AI node; parse preview; no voice/control/motion",
        "ai_pid": int(pid), "text": args.text, "request_id": request_id[0],
        "seconds": time.monotonic() - started, "service_accepted": bool(response and response.accepted),
        "events": events, "actual": actual, "expected": expected, "error": error,
        "authorization_after": {"voice": after.voice_authorized, "parse": after.parse_authorized,
                                "control": after.control_authorized},
        "passed": passed,
    }
    output.write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(record, ensure_ascii=False))
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
