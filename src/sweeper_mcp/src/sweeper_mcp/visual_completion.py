"""Fail-closed validation of a fresh, stopped result for one visual session."""
import re


def session_id_valid(value):
    return isinstance(value, str) and re.fullmatch(r"[0-9a-f]{32}", value) is not None


def sequence_valid(value):
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def visual_cleaning_completed(data, expected_id, baseline):
    """Use the controller's completion receipt, not another distance/angle gate.

    Cleaning can finish before physical braking and control handoff finish.
    Keep those separate so the AI can report success while awaiting safe dispatch.
    """
    receipt = data.get("completion", {})
    return bool(
        session_id_valid(expected_id) and sequence_valid(baseline)
        and type(data.get("ai_completion_protocol")) is int and data["ai_completion_protocol"] == 1
        and data.get("visual_session_id") == expected_id
        and data.get("visual_result") == "COMPLETE"
        and data.get("visual_state") in {"COMPLETE", "DISABLED"}
        and data.get("visual_active") is False
        and isinstance(receipt, dict)
        and receipt.get("session_id") == expected_id
        and receipt.get("result") == "COMPLETE"
        and sequence_valid(receipt.get("sequence")) and receipt["sequence"] > baseline
    )


def visual_completed(data, expected_id, baseline):
    """A successful cleaning session is also ready to release the next step."""
    return bool(visual_cleaning_completed(data, expected_id, baseline)
                and data["completion"].get("stopped") is True
                and data.get("state") == "RELATIVE_NAV_ACTIVE")
