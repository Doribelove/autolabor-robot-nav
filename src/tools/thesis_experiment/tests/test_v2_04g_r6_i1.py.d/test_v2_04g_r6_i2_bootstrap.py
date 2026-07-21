import ast
from pathlib import Path

import pytest

from thesis_experiment.v2_04g_r6_i1_r6_i2_bootstrap import (
    BOOTSTRAP_PROTOCOL_ID,
    ENVIRONMENT_POLICY_ID,
    R6I2BootstrapError,
    R6I2EnvironmentPolicyError,
    R6I2PositiveClockBarrier,
    assert_credential_safe_environment,
    build_credential_safe_environment,
    credential_safe_log_policy_receipt,
    redact_credential_material,
    validate_credential_safe_command,
)


WORKSPACE = Path(__file__).resolve().parents[5]
MODULE = WORKSPACE / (
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_r6_i2_bootstrap.py"
)


def _armed_barrier():
    barrier = R6I2PositiveClockBarrier(timeout_s=5.0)
    barrier.mark_base_spawned(10.0)
    barrier.mark_unpause_requested(10.1)
    barrier.mark_unpause_acknowledged(10.2, service_success=True)
    return barrier


def test_positive_clock_barrier_releases_service_wait_in_exact_order():
    barrier = _armed_barrier()
    assert barrier.observe_clock(0, 0, 10.3) is False
    assert barrier.service_wait_allowed is False
    assert barrier.observe_clock(0, 1, 10.4) is False
    assert barrier.observe_clock(0, 1, 10.5) is False
    assert barrier.observe_clock(0, 2, 10.6) is True
    barrier.release_service_wait(10.7, base_returncode=None)
    receipt = barrier.receipt()
    assert receipt["protocol_id"] == BOOTSTRAP_PROTOCOL_ID
    assert receipt["state"] == "service_wait_released"
    assert receipt["service_wait_allowed"] is True
    assert receipt["strict_clock_progression_required"] is True
    assert receipt["post_ack_clock_sample_count"] == 4
    assert receipt["post_ack_zero_clock_sample_count"] == 1
    assert receipt["post_positive_equal_clock_sample_count"] == 1
    assert receipt["positive_clock"] == {"sec": 0, "nsec": 1}
    assert receipt["clock_progression_sample"] == {"sec": 0, "nsec": 2}


def test_topic_existence_zero_and_pre_ack_positive_do_not_release_barrier():
    barrier = R6I2PositiveClockBarrier(timeout_s=5.0)
    barrier.mark_base_spawned(20.0)
    assert barrier.observe_clock(3, 0, 20.1) is False
    barrier.mark_unpause_requested(20.2)
    assert barrier.observe_clock(4, 0, 20.3) is False
    barrier.mark_unpause_acknowledged(20.4, service_success=True)
    assert barrier.observe_clock(0, 0, 20.5) is False
    assert barrier.observe_clock(1, 0, 20.55) is False
    assert barrier.service_wait_allowed is False
    with pytest.raises(R6I2BootstrapError, match="illegal_transition"):
        barrier.release_service_wait(20.6, base_returncode=None)
    receipt = barrier.receipt()
    assert receipt["state"] == "failed"
    assert receipt["pre_ack_positive_clock_sample_count_ignored"] == 2


def test_simulation_clock_must_strictly_progress_after_first_positive_sample():
    barrier = _armed_barrier()
    assert barrier.observe_clock(2, 0, 10.3) is False
    assert barrier.observe_clock(2, 0, 10.4) is False
    with pytest.raises(R6I2BootstrapError, match="simulation_clock_regressed"):
        barrier.observe_clock(1, 999_999_999, 10.5)
    assert barrier.receipt()["state"] == "failed"


def test_deadline_base_exit_and_malformed_clock_fail_closed_without_reset():
    deadline = _armed_barrier()
    with pytest.raises(
        R6I2BootstrapError, match="positive_clock_deadline_expired"
    ):
        deadline.check_deadline(15.0)
    with pytest.raises(R6I2BootstrapError, match="terminal_failure"):
        deadline.observe_clock(1, 0, 15.1)

    exited = _armed_barrier()
    with pytest.raises(
        R6I2BootstrapError, match="base_exited_before_clock_barrier"
    ):
        exited.observe_base_exit(returncode=1, monotonic_s=10.3)

    malformed = _armed_barrier()
    with pytest.raises(R6I2BootstrapError, match="malformed_clock_sample"):
        malformed.observe_clock(0, 1_000_000_000, 10.3)
    assert malformed.receipt()["state"] == "failed"


def test_unpause_must_be_acknowledged_and_monotonic_time_cannot_regress():
    failed_ack = R6I2PositiveClockBarrier(timeout_s=5.0)
    failed_ack.mark_base_spawned(1.0)
    failed_ack.mark_unpause_requested(1.1)
    with pytest.raises(
        R6I2BootstrapError, match="unpause_service_not_acknowledged"
    ):
        failed_ack.mark_unpause_acknowledged(
            1.2, service_success=False
        )

    regressed = _armed_barrier()
    with pytest.raises(R6I2BootstrapError, match="monotonic_time_regressed"):
        regressed.observe_clock(1, 0, 10.0)


def test_child_environment_is_exact_allowlist_and_audit_has_no_values(tmp_path):
    source = {
        "PATH": "/usr/bin",
        "PYTHONPATH": "/workspace/devel/lib/python3/dist-packages",
        "ROS_DISTRO": "noetic",
        "OPENAI_API_KEY": "sk-test-do-not-copy",
        "AWS_SECRET_ACCESS_KEY": "aws-test-do-not-copy",
        "SSH_AUTH_SOCK": "/tmp/agent.sock",
        "http_proxy": "http://proxy.invalid",
        "RANDOM_NON_RUNTIME_SETTING": "not-needed",
    }
    attempt_root = tmp_path.resolve() / "attempt"
    child, audit = build_credential_safe_environment(source, attempt_root)
    assert child["PATH"] == "/usr/bin"
    assert child["ROS_IP"] == "127.0.0.1"
    assert child["ROS_HOME"] == str(attempt_root / "ros_home")
    assert child["ROS_LOG_DIR"] == str(attempt_root / "ros_logs")
    assert "OPENAI_API_KEY" not in child
    assert "AWS_SECRET_ACCESS_KEY" not in child
    assert "SSH_AUTH_SOCK" not in child
    assert "http_proxy" not in child
    assert audit["policy_id"] == ENVIRONMENT_POLICY_ID
    assert audit["credential_value_copied"] is False
    assert audit["removed_credential_like_key_names"] == [
        "AWS_SECRET_ACCESS_KEY",
        "OPENAI_API_KEY",
        "SSH_AUTH_SOCK",
    ]
    assert "sk-test-do-not-copy" not in repr(audit)
    assert "aws-test-do-not-copy" not in repr(audit)
    assert_credential_safe_environment(child, attempt_root)


def test_child_environment_rejects_remote_or_credentialed_ros_master(tmp_path):
    root = tmp_path.resolve() / "attempt"
    for uri in (
        "http://robot.example:11311",
        "http://user:password@127.0.0.1:11311",
        "https://127.0.0.1:11311",
    ):
        with pytest.raises(
            R6I2EnvironmentPolicyError, match="loopback-only"
        ):
            build_credential_safe_environment({}, root, uri)
    with pytest.raises(R6I2EnvironmentPolicyError, match="malformed"):
        build_credential_safe_environment(
            {}, root, "http://127.0.0.1:not-a-port"
        )
    with pytest.raises(
        R6I2EnvironmentPolicyError, match="absolute non-traversing"
    ):
        build_credential_safe_environment({}, Path("relative/attempt"))
    with pytest.raises(
        R6I2EnvironmentPolicyError, match="aliased through allowlisted"
    ):
        build_credential_safe_environment(
            {
                "SERVICE_TOKEN": "token-test-value",
                "PATH": "/usr/bin/token-test-value",
            },
            root,
        )


def test_command_policy_and_log_redaction_remove_credential_material():
    environment = {
        "OPENAI_API_KEY": "sk-test-value",
        "SERVICE_TOKEN": "token-test-value",
    }
    command = validate_credential_safe_command(
        ["roslaunch", "m2_gazebo", "offline.launch"], environment
    )
    assert command[0] == "roslaunch"
    with pytest.raises(
        R6I2EnvironmentPolicyError, match="credential-like"
    ):
        validate_credential_safe_command(
            ["tool", "--token=token-test-value"], environment
        )
    with pytest.raises(
        R6I2EnvironmentPolicyError, match="credential-like"
    ):
        validate_credential_safe_command(
            ["tool", "--password", "not-even-inspected"], {}
        )
    with pytest.raises(
        R6I2EnvironmentPolicyError, match="credential-like"
    ):
        validate_credential_safe_command(
            ["tool", "sk-test-value"], environment
        )
    with pytest.raises(
        R6I2EnvironmentPolicyError, match="credential-like"
    ):
        validate_credential_safe_command(
            ["tool", "--config=sk-test-value"], environment
        )
    raw = (
        "OPENAI_API_KEY=sk-test-value "
        "Authorization: Bearer abc.def "
        "url=http://alice:secret@example.invalid/path"
    )
    safe = redact_credential_material(raw, environment)
    assert "sk-test-value" not in safe
    assert "abc.def" not in safe
    assert "alice:secret" not in safe
    assert safe.count("<redacted") >= 3


def test_policy_is_offline_non_authorizing_and_has_no_runtime_imports():
    receipt = credential_safe_log_policy_receipt()
    assert receipt["execution_authorized"] is False
    assert receipt["ros_or_subprocess_started"] is False
    tree = ast.parse(MODULE.read_text(encoding="utf-8"))
    imported_roots = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_roots.update(
                alias.name.split(".", 1)[0] for alias in node.names
            )
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported_roots.add(node.module.split(".", 1)[0])
    assert not imported_roots.intersection(
        {"rospy", "roslaunch", "rosgraph", "subprocess"}
    )
