from pathlib import Path

import pytest

from teb_mode_manager.action_pipeline import AnchorBank, RuleAnchorTransactionLoop
from teb_mode_manager.typed_teb_transaction import (
    AckMismatchError,
    ParameterTypeError,
    ReadbackMismatchError,
    SimulationWriteContext,
    SimulationWriteGateError,
    TypedTebTransactionBackend,
    validate_parameter_interface,
)


ROOT = Path(__file__).resolve().parents[4]
BANK_PATH = ROOT / "src/application/teb_mode_manager/config/v2_04_anchor_bank_candidate.yaml"


def load_bank():
    return AnchorBank.from_file(BANK_PATH)


def context(**changes):
    values = dict(
        explicit_simulation_write=True,
        use_sim_time=True,
        simulation_marker=True,
        gazebo_clock_active=True,
        teb_namespace="/move_base/TebLocalPlannerROS",
    )
    values.update(changes)
    return SimulationWriteContext(**values)


class FakeAdapter:
    def __init__(self, bank, initial=None):
        self.bank = bank
        self.current = dict((initial or bank.anchors["anchor_balanced"]).values)
        self.update_calls = []
        self.next_fault = ""
        self.closed = False

    def descriptions(self):
        rows = []
        for name, definition in self.bank.definitions.items():
            row = {"name": name, "type": definition.parameter_type, "description": "test"}
            if definition.parameter_type != "bool":
                row.update(min=definition.lower - 1.0, max=definition.upper + 1.0)
                if definition.parameter_type == "int":
                    row.update(min=int(definition.lower), max=int(definition.upper))
            rows.append(row)
        return rows

    def get_parameter_descriptions(self, timeout_s):
        return self.descriptions()

    def get_configuration(self, timeout_s):
        return dict(self.current)

    def update_configuration(self, values, timeout_s):
        self.update_calls.append(dict(values))
        fault, self.next_fault = self.next_fault, ""
        self.current = dict(values)
        if fault == "ack_mismatch":
            result = dict(values)
            result["max_vel_x"] += 0.01
            return result
        return dict(values)

    def wait_for_configuration(self, expected, timeout_s):
        if self.next_fault == "readback_mismatch":
            self.next_fault = ""
            result = dict(self.current)
            result["include_dynamic_obstacles"] = not result["include_dynamic_obstacles"]
            return result
        return dict(self.current)

    def close(self):
        self.closed = True


class ReadbackFaultAdapter(FakeAdapter):
    def __init__(self, bank, initial=None):
        super().__init__(bank, initial)
        self.fail_readback_once = False

    def wait_for_configuration(self, expected, timeout_s):
        if self.fail_readback_once:
            self.fail_readback_once = False
            result = dict(self.current)
            result["max_number_classes"] = 1 if result["max_number_classes"] != 1 else 2
            return result
        return dict(self.current)


def initialized_backend(bank=None, adapter=None):
    bank = bank or load_bank()
    adapter = adapter or FakeAdapter(bank)
    ticks = iter([1.01, 1.02, 1.03, 1.04, 1.05, 1.06, 1.07, 1.08])
    backend = TypedTebTransactionBackend(
        bank, adapter, context(), time_source=lambda: next(ticks)
    )
    backend.initialize()
    return backend, adapter


def test_simulation_gate_requires_all_independent_guards_and_exact_namespace():
    bank = load_bank()
    for change in (
        {"explicit_simulation_write": False},
        {"use_sim_time": False},
        {"simulation_marker": False},
        {"gazebo_clock_active": False},
        {"teb_namespace": "/move_base"},
    ):
        with pytest.raises(SimulationWriteGateError):
            TypedTebTransactionBackend(bank, FakeAdapter(bank), context(**change))


def test_interface_validates_all_20_live_types_and_candidate_domains():
    bank = load_bank()
    adapter = FakeAdapter(bank)
    specs = validate_parameter_interface(adapter.descriptions(), adapter.current, bank)
    assert tuple(specs) == bank.parameter_names
    assert sum(spec.parameter_type == "double" for spec in specs.values()) == 18
    descriptions = adapter.descriptions()
    next(row for row in descriptions if row["name"] == "include_dynamic_obstacles")["type"] = "int"
    with pytest.raises(ParameterTypeError):
        validate_parameter_interface(descriptions, adapter.current, bank)


def test_atomic_typed_request_ack_readback_and_shutdown_restore_startup():
    bank = load_bank()
    backend, adapter = initialized_backend(bank)
    startup = dict(backend.startup.values)
    target = bank.anchors["anchor_cruise"]
    receipt = backend.apply(target, 1.0)
    assert receipt.executed.values == target.values
    assert tuple(adapter.update_calls[-1]) == bank.parameter_names
    assert type(adapter.update_calls[-1]["include_dynamic_obstacles"]) is bool
    assert type(adapter.update_calls[-1]["max_number_classes"]) is int
    assert receipt.t_request_s <= receipt.t_ack_s <= receipt.t_readback_s <= receipt.t_active_s
    backend.close()
    assert adapter.current == startup
    assert adapter.closed is True


def test_ack_or_readback_failure_restores_previous_executed_not_startup():
    bank = load_bank()
    for fault in ("ack", "readback"):
        adapter = ReadbackFaultAdapter(bank)
        backend, adapter = initialized_backend(bank, adapter)
        previous = bank.anchors["anchor_static_dense"]
        backend.apply(previous, 1.0)
        if fault == "ack":
            adapter.next_fault = "ack_mismatch"
            expected_error = AckMismatchError
        else:
            adapter.fail_readback_once = True
            expected_error = ReadbackMismatchError
        with pytest.raises(expected_error):
            backend.apply(bank.anchors["anchor_cruise"], 1.1)
        assert backend.current.values == previous.values
        assert adapter.current == previous.values
        assert adapter.update_calls[-1] == previous.values


def test_rule_loop_uses_live_executed_profile_as_rate_limit_origin():
    bank = load_bank()
    backend, adapter = initialized_backend(bank)
    loop = RuleAnchorTransactionLoop(bank, backend=backend)
    trace = loop.update(1.0, 1, 1, "CRUISE", "NONE", "ENTERING", True)
    assert trace.execution_backend == "simulation_teb_dynamic_reconfigure"
    assert trace.valid and trace.activated and not trace.training_used
    index = bank.parameter_names.index("max_vel_x")
    assert trace.executed[index] == pytest.approx(1.0)
    assert adapter.current["max_vel_x"] == pytest.approx(1.0)
