from pathlib import Path

import pytest

from teb_mode_manager.action_pipeline import AnchorBank
from teb_mode_manager.r6_execution_integration import (
    R6ExecutionIntegrationError,
    R6TwoPhaseIdempotentTypedTebTransactionBackend,
    R6TwoPhaseRestoreError,
    canonical_attempt_identity,
    canonical_profile_bytes,
    sha256_bytes,
)
from teb_mode_manager.typed_teb_transaction import SimulationWriteContext


WORKSPACE = Path(__file__).resolve().parents[4]
ANCHOR_BANK = (
    WORKSPACE
    / "src/application/teb_mode_manager/config/"
    "v2_04c_anchor_bank_frozen.yaml"
)


class FakeAdapter:
    def __init__(self, bank):
        self.bank = bank
        self.current = dict(bank.anchors["anchor_balanced"].values)
        self.update_calls = []
        self.closed = False

    def get_parameter_descriptions(self, _timeout_s):
        rows = []
        for name, definition in self.bank.definitions.items():
            row = {
                "name": name,
                "type": definition.parameter_type,
                "description": "R6 offline fixture",
            }
            if definition.parameter_type != "bool":
                if definition.parameter_type == "int":
                    row["min"] = int(definition.lower)
                    row["max"] = int(definition.upper)
                else:
                    row["min"] = definition.lower
                    row["max"] = definition.upper
            rows.append(row)
        return rows

    def get_configuration(self, _timeout_s):
        return dict(self.current)

    def update_configuration(self, values, _timeout_s):
        self.update_calls.append(dict(values))
        self.current = dict(values)
        return dict(values)

    def wait_for_configuration(self, _expected, _timeout_s):
        return dict(self.current)

    def close(self):
        self.closed = True


def identity():
    return {
        "stage": "V2-04G-R6-INTEGRATION-TEST",
        "profile_id": "r6_semantics_circle_contact",
        "scene_id": "offline-fixture",
        "seed": 6001,
        "attempt": 1,
    }


def backend():
    bank = AnchorBank.from_file(ANCHOR_BANK)
    adapter = FakeAdapter(bank)
    context = SimulationWriteContext(
        explicit_simulation_write=True,
        use_sim_time=True,
        simulation_marker=True,
        gazebo_clock_active=True,
        teb_namespace="/move_base/TebLocalPlannerROS",
    )
    result = R6TwoPhaseIdempotentTypedTebTransactionBackend(
        bank, adapter, context, time_source=lambda: 10.0
    )
    result.initialize()
    return bank, adapter, result


def test_startup_capture_is_complete_typed_canonical_profile():
    bank, adapter, result = backend()
    payload = canonical_profile_bytes(bank, adapter.current)
    assert result.startup_capture.payload == payload
    assert result.startup_capture.sha256 == sha256_bytes(payload)
    document = result.startup_capture.as_document(identity())
    assert document["identity"] == identity()
    assert document["startup_profile_canonical_json"] == payload.decode()
    result.close()


def test_explicit_restore_binds_ack_readback_and_independent_readback():
    bank, adapter, result = backend()
    result.apply(bank.anchors["anchor_cruise"], 1.0)
    writes_before_restore = len(adapter.update_calls)

    def independent_reader():
        assert result.backend_alive is True
        assert adapter.current == result.startup.values
        assert len(adapter.update_calls) == writes_before_restore + 1
        return dict(adapter.current)

    receipt = result.restore_startup_two_phase(
        identity(), independent_reader
    )
    expected = result.startup_capture.sha256
    assert receipt["status"] == "pass"
    assert receipt["restore_requested_while_backend_alive"] is True
    assert receipt["transaction_acknowledged"] is True
    assert receipt["transaction_readback_match"] is True
    assert receipt["independent_readback_match"] is True
    assert receipt["backend_alive_after_restore"] is True
    assert receipt["startup_profile_sha256"] == expected
    assert receipt["transaction_ack_sha256"] == expected
    assert receipt["transaction_readback_sha256"] == expected
    assert receipt["independent_readback_sha256"] == expected
    assert (
        receipt["restore_t_request_s"]
        <= receipt["restore_t_ack_s"]
        <= receipt["restore_t_readback_s"]
        <= receipt["restore_t_active_s"]
        <= receipt["independent_readback_t_s"]
    )
    assert len(adapter.update_calls) == writes_before_restore + 1

    with pytest.raises(R6ExecutionIntegrationError):
        result.apply(bank.anchors["anchor_static_dense"], 2.0)
    with pytest.raises(R6ExecutionIntegrationError):
        result.restore_startup_two_phase(identity(), independent_reader)
    assert result.teardown_receipt == receipt

    writes_before_close = len(adapter.update_calls)
    result.close()
    assert adapter.closed is True
    assert len(adapter.update_calls) == writes_before_close


def test_independent_mismatch_fails_closed_and_never_yields_pass_receipt():
    _bank, adapter, result = backend()
    assert result.teardown_attempted is False

    def mismatched_reader():
        values = dict(adapter.current)
        values["max_vel_x"] -= 0.01
        return values

    with pytest.raises(R6TwoPhaseRestoreError) as captured:
        result.restore_startup_two_phase(identity(), mismatched_reader)
    receipt = captured.value.receipt
    assert receipt["status"] == "fail"
    assert receipt["transaction_acknowledged"] is True
    assert receipt["transaction_readback_match"] is True
    assert receipt["independent_readback_match"] is False
    assert result.teardown_attempted is True
    assert result.teardown_verified is False
    with pytest.raises(R6ExecutionIntegrationError):
        result.restore_startup_two_phase(identity(), lambda: adapter.current)
    result.close()


def test_attempt_identity_rejects_type_coercion_and_missing_fields():
    assert canonical_attempt_identity(identity()) == identity()
    for malformed in (
        {**identity(), "seed": True},
        {**identity(), "attempt": 0},
        {key: value for key, value in identity().items() if key != "scene_id"},
        {**identity(), "extra": "forbidden"},
    ):
        with pytest.raises(R6ExecutionIntegrationError):
            canonical_attempt_identity(malformed)
