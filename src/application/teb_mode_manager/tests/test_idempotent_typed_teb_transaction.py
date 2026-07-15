from pathlib import Path

from teb_mode_manager.action_pipeline import TypedProfile
from teb_mode_manager.idempotent_typed_teb_transaction import (
    IdempotentTypedTebTransactionBackend,
)
from teb_mode_manager.typed_teb_transaction import SimulationWriteContext

from test_typed_teb_transaction import FakeAdapter


WORKSPACE = Path(__file__).resolve().parents[4]
ANCHOR_BANK = (
    WORKSPACE
    / "src/application/teb_mode_manager/config/v2_04c_anchor_bank_frozen.yaml"
)


def _backend():
    from teb_mode_manager.action_pipeline import AnchorBank

    bank = AnchorBank.from_file(ANCHOR_BANK)
    adapter = FakeAdapter(bank)
    context = SimulationWriteContext(
        explicit_simulation_write=True,
        use_sim_time=True,
        simulation_marker=True,
        gazebo_clock_active=True,
        teb_namespace="/move_base/TebLocalPlannerROS",
    )
    backend = IdempotentTypedTebTransactionBackend(
        bank, adapter, context, time_source=lambda: 10.0
    )
    backend.initialize()
    return bank, adapter, backend


def test_identical_profile_is_activated_without_reconfigure_write():
    bank, adapter, backend = _backend()
    requested = TypedProfile(
        "anchor_balanced", "same_values_new_semantic_id",
        dict(bank.anchors["anchor_balanced"].values),
    )
    receipt = backend.apply(requested, 4.0)
    assert receipt.executed == requested
    assert backend.current == requested
    assert backend.coalesced_apply_count == 1
    assert backend.write_apply_count == 0
    assert adapter.update_calls == []
    assert backend.audit_records[-1]["operation"] == "coalesced_apply"


def test_changed_profile_uses_original_atomic_write_and_readback():
    bank, adapter, backend = _backend()
    values = dict(bank.anchors["anchor_balanced"].values)
    values["max_vel_x"] = values["max_vel_x"] - 0.05
    requested = TypedProfile("anchor_balanced", "changed", values)
    receipt = backend.apply(requested, 4.0)
    assert receipt.executed.values == values
    assert backend.coalesced_apply_count == 0
    assert backend.write_apply_count == 1
    assert len(adapter.update_calls) == 1
    assert backend.audit_records[-1]["operation"] == "apply"
