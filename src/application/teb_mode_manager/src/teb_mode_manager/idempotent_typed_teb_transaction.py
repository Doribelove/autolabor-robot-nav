"""Idempotent simulation-only wrapper for typed TEB parameter writes."""

from typing import Any, Dict

from .action_pipeline import ExecutionReceipt, TypedProfile
from .typed_teb_transaction import TypedTebTransactionBackend


class IdempotentTypedTebTransactionBackend(TypedTebTransactionBackend):
    """Coalesce an already-active typed profile without touching TEB again.

    The commanded/feasible/safe/executed transaction remains observable on
    every decision tick.  Only the redundant dynamic_reconfigure side effect is
    removed once all typed values equal the last acknowledged readback.
    """

    # Preserve the frozen four-stage action contract backend identity. The
    # coalescing optimization is an execution detail, not a fifth action stage.
    backend_id = "simulation_teb_dynamic_reconfigure"

    def __init__(self, *args: Any, **kwargs: Any):
        super().__init__(*args, **kwargs)
        self.coalesced_apply_count = 0
        self.write_apply_count = 0

    def _values_equal_current(self, values: Dict[str, Any]) -> bool:
        if self.current is None:
            return False
        for name in self.bank.parameter_names:
            parameter_type = self.bank.definitions[name].parameter_type
            left = values[name]
            right = self.current.values[name]
            if parameter_type in ("bool", "int"):
                if left != right:
                    return False
            elif abs(float(left) - float(right)) > self._tolerance:
                return False
        return True

    def apply(self, requested: TypedProfile, now_s: float) -> ExecutionReceipt:
        values = self._validate_request(requested)
        if not self._values_equal_current(values):
            self.write_apply_count += 1
            return super().apply(requested, now_s)

        active = TypedProfile(
            requested.anchor_id, requested.profile_id, dict(values)
        )
        stamp = float(now_s)
        self.current = active
        self.coalesced_apply_count += 1
        self.audit_records.append({
            "operation": "coalesced_apply",
            "profile_id": requested.profile_id,
            "request": dict(values),
            "ack": dict(values),
            "readback": dict(values),
            "passed": True,
        })
        return ExecutionReceipt(
            requested=active,
            acknowledgement=active,
            readback=active,
            executed=active,
            t_request_s=stamp,
            t_ack_s=stamp,
            t_readback_s=stamp,
            t_active_s=stamp,
        )
