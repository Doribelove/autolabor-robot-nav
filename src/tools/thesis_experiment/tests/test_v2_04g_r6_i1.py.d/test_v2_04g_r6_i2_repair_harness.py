import ast
import importlib.util
import inspect
import json
from pathlib import Path

import pytest


WORKSPACE = Path(__file__).resolve().parents[5]
HARNESS_PATH = WORKSPACE / (
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_repair_harness.py"
)
XACRO = "/opt/ros/noetic/lib/xacro/xacro"
SECRET = "offline-fixture-secret-value"
_HARNESS_MODULE = None


def _harness():
    global _HARNESS_MODULE
    if _HARNESS_MODULE is not None:
        return _HARNESS_MODULE
    specification = importlib.util.spec_from_file_location(
        "v2_04g_r6_i2_repair_harness_test",
        HARNESS_PATH,
    )
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    _HARNESS_MODULE = module
    return _HARNESS_MODULE


def _closure(target=XACRO, duplicate=False):
    row = {
        "binding": "package-executable:xacro:xacro",
        "target_canonical_path": target,
        "canonical_paths": [target],
    }
    rows = [row, dict(row)] if duplicate else [row]
    return {
        "execution_authorized": False,
        "unresolved": [],
        "external": {
            "runtime_bindings": rows,
            "files": [
                {
                    "canonical_path": target,
                    "sha256": "a" * 64,
                    "size_bytes": 17,
                }
            ],
            "unresolved": [],
        },
    }


def _callbacks(
    events,
    clocks=((0, 1), (0, 2)),
    *,
    acknowledgement=True,
    spawn_returncode=None,
    service_ready=True,
    spawn_error=None,
):
    def spawn(environment, xacro_executable):
        events.append("base_spawn")
        assert xacro_executable == XACRO
        assert environment["ROS_MASTER_URI"] == "http://127.0.0.1:11311"
        assert "SERVICE_TOKEN" not in environment
        if spawn_error is not None:
            raise RuntimeError(spawn_error)
        return {
            "monotonic_s": 10.0,
            "base_returncode": spawn_returncode,
            "diagnostic": "SERVICE_TOKEN={}".format(SECRET),
        }

    def request():
        events.append("unpause_request")
        return {
            "monotonic_s": 10.1,
            "base_returncode": None,
            "diagnostic": "unpause requested",
        }

    def acknowledge():
        events.append("unpause_success_ack")
        return {
            "monotonic_s": 10.2,
            "base_returncode": None,
            "service_success": acknowledgement,
            "diagnostic": "unpause acknowledged",
        }

    def clock(ordinal):
        events.append("clock_{}".format(ordinal))
        sec, nsec = clocks[ordinal - 1]
        return {
            "sec": sec,
            "nsec": nsec,
            "monotonic_s": 10.2 + ordinal * 0.1,
            "base_returncode": None,
            "diagnostic": "clock {}".format(ordinal),
        }

    def services(environment):
        events.append("service_wait")
        assert "SERVICE_TOKEN" not in environment
        return {
            "monotonic_s": 10.5,
            "base_returncode": None,
            "move_base_ready": service_ready,
            "teb_reconfigure_ready": service_ready,
            "diagnostic": "service wait complete",
        }

    return {
        "spawn_base_observation": spawn,
        "unpause_request_observation": request,
        "unpause_ack_observation": acknowledge,
        "clock_observation": clock,
        "service_wait_observation": services,
    }


def _run(tmp_path, callbacks, **overrides):
    module = _harness()
    arguments = {
        "dependency_closure": _closure(),
        "xacro_executable": XACRO,
        "timeout_s": 5.0,
        "attempt_root": (tmp_path / "attempt").resolve(),
        "source_environment": {
            "PATH": "/usr/bin",
            "ROS_DISTRO": "noetic",
            "SERVICE_TOKEN": SECRET,
            "NON_RUNTIME_VALUE": "discard-me",
        },
    }
    arguments.update(callbacks)
    arguments.update(overrides)
    return module, module.review_bootstrap_integration(**arguments)


def test_harness_enforces_exact_bootstrap_order_before_service_wait(tmp_path):
    events = []
    module, receipt = _run(tmp_path, _callbacks(events))
    assert events == [
        "base_spawn",
        "unpause_request",
        "unpause_success_ack",
        "clock_1",
        "clock_2",
        "service_wait",
    ]
    assert receipt["stage"] == "V2-04G-R6-I2"
    assert receipt["status"].endswith("execution_not_authorized")
    assert receipt["execution_authorized"] is False
    assert receipt["authorization_created"] is False
    assert receipt["seed_values"] == []
    assert receipt["seed_or_evidence_units_allocated"] == 0
    assert receipt["seed_or_evidence_units_consumed"] == 0
    assert receipt["ros_or_subprocess_started"] is False
    assert receipt["observed_call_order"] == list(module.REQUIRED_CALL_ORDER)
    assert receipt["call_order_exact"] is True
    assert receipt["bootstrap_receipt"]["state"] == "service_wait_released"
    assert receipt["bootstrap_receipt"]["positive_clock"] == {
        "sec": 0,
        "nsec": 1,
    }
    assert receipt["bootstrap_receipt"]["clock_progression_sample"] == {
        "sec": 0,
        "nsec": 2,
    }
    assert receipt["service_readiness"] == {
        "move_base_ready": True,
        "teb_reconfigure_ready": True,
        "wait_callback_invoked_after_clock_release": True,
    }
    assert receipt["xacro_runtime_binding"] == {
        "binding": "package-executable:xacro:xacro",
        "target_canonical_path": XACRO,
        "supplied_executable_exact_match": True,
    }
    rendered = json.dumps(receipt, sort_keys=True)
    assert SECRET not in rendered
    assert "<redacted>" in rendered
    assert receipt["child_environment_audit"]["credential_value_copied"] is False
    assert receipt["log_policy"]["execution_authorized"] is False


@pytest.mark.parametrize(
    "clocks,expected_last_event,reason",
    [
        (((0, 0), (0, 1)), "clock_1", "first_post_ack_clock"),
        (((1, 0), (1, 0)), "clock_2", "second_post_ack_clock"),
        (((2, 0), (1, 999_999_999)), "clock_2", "simulation_clock_regressed"),
    ],
)
def test_zero_repeated_and_regressed_clock_fail_before_service_wait(
    tmp_path,
    clocks,
    expected_last_event,
    reason,
):
    module = _harness()
    events = []
    with pytest.raises(module.R6I2RepairHarnessError) as captured:
        _run(tmp_path, _callbacks(events, clocks=clocks))
    error = captured.value
    assert events[-1] == expected_last_event
    assert "service_wait" not in events
    assert reason in error.receipt["failure_reason"]
    assert error.receipt["execution_authorized"] is False
    assert error.receipt["seed_values"] == []
    assert error.receipt["status"] == (
        "failed_closed_execution_not_authorized"
    )


@pytest.mark.parametrize(
    "callback_options,reason,expected_events",
    [
        (
            {"acknowledgement": False},
            "unpause_service_not_acknowledged",
            ["base_spawn", "unpause_request", "unpause_success_ack"],
        ),
        (
            {"spawn_returncode": 7},
            "base_exited_before_clock_barrier",
            ["base_spawn"],
        ),
        (
            {"spawn_error": "SERVICE_TOKEN=" + SECRET},
            "offline_callback_failed",
            ["base_spawn"],
        ),
    ],
)
def test_ack_base_exit_and_callback_failure_are_closed_and_redacted(
    tmp_path,
    callback_options,
    reason,
    expected_events,
):
    module = _harness()
    events = []
    with pytest.raises(module.R6I2RepairHarnessError) as captured:
        _run(tmp_path, _callbacks(events, **callback_options))
    assert events == expected_events
    assert reason in captured.value.receipt["failure_reason"]
    assert SECRET not in json.dumps(captured.value.receipt, sort_keys=True)
    assert captured.value.receipt["execution_authorized"] is False


@pytest.mark.parametrize(
    "closure,xacro,reason",
    [
        (_closure(), "/usr/bin/xacro", "does_not_exactly_match"),
        (_closure(duplicate=True), XACRO, "must_resolve_exactly_once"),
        (
            {
                "execution_authorized": False,
                "unresolved": [],
                "external": {
                    "runtime_bindings": [],
                    "files": [],
                    "unresolved": [],
                },
            },
            XACRO,
            "must_resolve_exactly_once",
        ),
    ],
)
def test_xacro_must_exactly_match_single_closed_runtime_binding(
    tmp_path,
    closure,
    xacro,
    reason,
):
    module = _harness()
    events = []
    with pytest.raises(module.R6I2RepairHarnessError) as captured:
        _run(
            tmp_path,
            _callbacks(events),
            dependency_closure=closure,
            xacro_executable=xacro,
        )
    assert events == []
    assert reason in captured.value.receipt["failure_reason"]
    assert captured.value.receipt["execution_authorized"] is False


def test_service_readiness_failure_occurs_only_after_clock_release(tmp_path):
    module = _harness()
    events = []
    with pytest.raises(module.R6I2RepairHarnessError) as captured:
        _run(tmp_path, _callbacks(events, service_ready=False))
    receipt = captured.value.receipt
    assert events[-1] == "service_wait"
    assert receipt["observed_call_order"] == list(module.REQUIRED_CALL_ORDER)
    assert receipt["bootstrap_receipt"]["service_wait_allowed"] is True
    assert "move_base_or_teb_service_not_ready" in receipt["failure_reason"]
    assert receipt["execution_authorized"] is False
    assert receipt["seed_values"] == []


def test_harness_has_no_runtime_entrypoint_or_ros_process_imports():
    module = _harness()
    tree = ast.parse(HARNESS_PATH.read_text(encoding="utf-8"))
    imported_roots = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_roots.update(
                alias.name.split(".", 1)[0] for alias in node.names
            )
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported_roots.add(node.module.split(".", 1)[0])
    assert not imported_roots.intersection(
        {"argparse", "os", "rospy", "roslaunch", "rosgraph", "subprocess"}
    )
    assert not any(
        isinstance(node, ast.If)
        and isinstance(node.test, ast.Compare)
        and any(
            isinstance(candidate, ast.Constant)
            and candidate.value == "__main__"
            for candidate in ast.walk(node.test)
        )
        for node in ast.walk(tree)
    )
    signature = inspect.signature(module.review_bootstrap_integration)
    assert "seed" not in signature.parameters
    assert "authorization" not in signature.parameters
