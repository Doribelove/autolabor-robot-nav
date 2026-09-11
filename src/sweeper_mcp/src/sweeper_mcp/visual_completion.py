"""Fail-closed validation of a fresh, stopped result for one visual session."""
import re


def session_id_valid(value):
    return isinstance(value, str) and re.fullmatch(r"[0-9a-f]{32}", value) is not None


def sequence_valid(value):
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def visual_completed(data, expected_id, baseline):
    receipt = data.get("completion", {})
    return bool(
        session_id_valid(expected_id) and sequence_valid(baseline)
        and type(data.get("ai_completion_protocol")) is int and data["ai_completion_protocol"] == 1
        and data.get("visual_session_id") == expected_id
        and data.get("visual_result") == "COMPLETE"
        and data.get("visual_state") in {"COMPLETE", "DISABLED"}
        and data.get("visual_active") is False
        and data.get("state") == "RELATIVE_NAV_ACTIVE"
        and isinstance(receipt, dict)
        and receipt.get("session_id") == expected_id
        and receipt.get("result") == "COMPLETE" and receipt.get("stopped") is True
        and sequence_valid(receipt.get("sequence")) and receipt["sequence"] > baseline
    )
