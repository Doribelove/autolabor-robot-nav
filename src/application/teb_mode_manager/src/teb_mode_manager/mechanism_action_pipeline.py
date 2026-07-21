"""V2-04G residual-aware extension of the frozen typed Anchor loop."""

from typing import Any, Mapping, Optional

from .action_pipeline import (
    ActionPipelineError,
    RuleAnchorTransactionLoop,
    TypedProfile,
    _finite_double,
)


class MechanismAnchorTransactionLoop(RuleAnchorTransactionLoop):
    """Add bounded semantic residuals without changing the frozen V2-04B loop."""

    def update(
        self,
        now_s: float,
        world_model_seq: int,
        mode_seq: int,
        geometry_mode: str,
        dynamic_overlay: str,
        transition_state: str,
        context_valid: bool,
        maneuver_reverse: bool = False,
        residuals: Optional[Mapping[str, Any]] = None,
    ):
        stamp = _finite_double(now_s, "now_s")
        if self.last_update_s is not None and stamp < self.last_update_s:
            raise ActionPipelineError("transaction time moved backwards")
        dt = 1.0 / self.frequency_hz if self.last_update_s is None else stamp - self.last_update_s
        if dt <= 0.0:
            raise ActionPipelineError("transaction dt must be positive")
        self.last_update_s = stamp
        self.config_seq += 1
        if not context_valid or transition_state == "FAULTED":
            current = self.executed
            vector = self.numeric_vector(current)
            return self._trace(
                world_model_seq, mode_seq, geometry_mode, dynamic_overlay,
                transition_state, current.anchor_id, current.profile_id,
                vector, vector, vector, vector, 0, 0,
                stamp, stamp, stamp, stamp, False, False, False,
                "invalid_or_faulted_context_hold_previous_executed",
            )
        decoded = self.decoder.decode(
            geometry_mode,
            dynamic_overlay,
            residuals=residuals or {},
            maneuver_reverse=maneuver_reverse,
        )
        safe = decoded.feasible
        candidate_values, slow_committed = self._rate_limited_candidate(safe.values, dt)
        requested = TypedProfile(decoded.anchor_id, decoded.profile_id, candidate_values)
        previous = self.executed
        try:
            receipt = self.backend.apply(requested, stamp)
            self.executed = receipt.executed
        except ActionPipelineError as exc:
            self.executed = previous
            return self._trace(
                world_model_seq, mode_seq, geometry_mode, dynamic_overlay,
                transition_state, decoded.anchor_id, decoded.profile_id,
                self.numeric_vector(decoded.commanded), self.numeric_vector(decoded.feasible),
                self.numeric_vector(safe), self.numeric_vector(previous),
                decoded.projection_reason_mask, 0,
                stamp, stamp, stamp, stamp, False, False, False, str(exc),
            )
        return self._trace(
            world_model_seq, mode_seq, geometry_mode, dynamic_overlay,
            transition_state, decoded.anchor_id, decoded.profile_id,
            self.numeric_vector(decoded.commanded), self.numeric_vector(decoded.feasible),
            self.numeric_vector(safe), self.numeric_vector(receipt.executed),
            decoded.projection_reason_mask, 0,
            receipt.t_request_s, receipt.t_ack_s, receipt.t_readback_s, receipt.t_active_s,
            True, slow_committed, True, "",
        )
