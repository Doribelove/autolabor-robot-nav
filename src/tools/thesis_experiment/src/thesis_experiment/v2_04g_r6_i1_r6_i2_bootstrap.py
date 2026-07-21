"""Offline-verifiable R6-I2 bootstrap and child-environment policy.

This module deliberately has no ROS or process-launching imports.  A future
execution runner may adapt ROS observations into the state machine and pass the
returned environment to its process launcher only after a separate
authorization.  The module itself cannot launch a process, contact a ROS
master, or consume a seed.
"""

from __future__ import annotations

import math
import re
from pathlib import Path
from typing import Dict, Mapping, Optional, Sequence, Tuple
from urllib.parse import urlsplit


BOOTSTRAP_PROTOCOL_ID = "v2_04g_r6_i2_positive_clock_bootstrap_v1"
ENVIRONMENT_POLICY_ID = "v2_04g_r6_i2_credential_safe_environment_v1"
LOG_REDACTION_POLICY_ID = "v2_04g_r6_i2_credential_safe_log_redaction_v1"


class R6I2BootstrapError(RuntimeError):
    """Raised when the positive-clock bootstrap protocol fails closed."""


class R6I2EnvironmentPolicyError(ValueError):
    """Raised when a child environment or command violates the I2 policy."""


class R6I2PositiveClockBarrier:
    """One-shot state machine that gates readiness on post-unpause sim time.

    The caller must:

    1. report the base launch spawn;
    2. report the unpause request and successful acknowledgement;
    3. feed ``/clock`` samples received after that acknowledgement; and
    4. explicitly release the later service-readiness wait.

    Topic existence, a zero-valued sample, a lone positive sample, and a
    positive sample received before the unpause acknowledgement cannot release
    the barrier.  Release requires two post-ack positive samples whose ROS
    times are strictly increasing.
    """

    NEW = "new"
    BASE_SPAWNED = "base_spawned"
    UNPAUSE_REQUESTED = "unpause_requested"
    CLOCK_BARRIER_ARMED = "clock_barrier_armed"
    POSITIVE_CLOCK_OBSERVED = "positive_clock_observed"
    CLOCK_PROGRESS_CONFIRMED = "clock_progress_confirmed"
    SERVICE_WAIT_RELEASED = "service_wait_released"
    FAILED = "failed"

    _PRE_RELEASE_STATES = frozenset(
        {
            BASE_SPAWNED,
            UNPAUSE_REQUESTED,
            CLOCK_BARRIER_ARMED,
            POSITIVE_CLOCK_OBSERVED,
            CLOCK_PROGRESS_CONFIRMED,
        }
    )

    def __init__(self, timeout_s: float) -> None:
        if (
            isinstance(timeout_s, bool)
            or not isinstance(timeout_s, (int, float))
            or not math.isfinite(float(timeout_s))
            or float(timeout_s) <= 0.0
        ):
            raise ValueError("timeout_s must be finite and positive")
        self._timeout_s = float(timeout_s)
        self._state = self.NEW
        self._base_spawn_monotonic_s: Optional[float] = None
        self._deadline_monotonic_s: Optional[float] = None
        self._last_event_monotonic_s: Optional[float] = None
        self._unpause_ack_monotonic_s: Optional[float] = None
        self._positive_clock = None  # type: Optional[Tuple[int, int]]
        self._positive_clock_monotonic_s: Optional[float] = None
        self._progress_clock = None  # type: Optional[Tuple[int, int]]
        self._progress_clock_monotonic_s: Optional[float] = None
        self._post_ack_clock_samples = 0
        self._post_ack_zero_samples = 0
        self._post_positive_equal_samples = 0
        self._pre_ack_clock_samples_ignored = 0
        self._pre_ack_positive_samples_ignored = 0
        self._failure_reason: Optional[str] = None

    @property
    def state(self) -> str:
        return self._state

    @property
    def service_wait_allowed(self) -> bool:
        return self._state == self.SERVICE_WAIT_RELEASED

    @property
    def terminal(self) -> bool:
        return self._state in {self.SERVICE_WAIT_RELEASED, self.FAILED}

    def _finite_monotonic(self, value: float) -> float:
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
        ):
            self._fail_without_time("invalid_monotonic_time")
        now = float(value)
        if (
            self._last_event_monotonic_s is not None
            and now < self._last_event_monotonic_s
        ):
            self._fail_without_time("monotonic_time_regressed")
        return now

    def _fail_without_time(self, reason: str) -> None:
        if self._state == self.FAILED:
            raise R6I2BootstrapError(
                "barrier_is_terminal_failure:{}".format(
                    self._failure_reason
                )
            )
        if self._state == self.SERVICE_WAIT_RELEASED:
            raise R6I2BootstrapError("barrier_already_released")
        self._failure_reason = reason
        self._state = self.FAILED
        raise R6I2BootstrapError(reason)

    def _advance_time(self, monotonic_s: float) -> float:
        if self._state == self.FAILED:
            raise R6I2BootstrapError(
                "barrier_is_terminal_failure:{}".format(self._failure_reason)
            )
        now = self._finite_monotonic(monotonic_s)
        self._last_event_monotonic_s = now
        if (
            self._state in self._PRE_RELEASE_STATES
            and self._deadline_monotonic_s is not None
            and now >= self._deadline_monotonic_s
        ):
            self._failure_reason = "positive_clock_deadline_expired"
            self._state = self.FAILED
            raise R6I2BootstrapError(self._failure_reason)
        return now

    def _require_state(self, expected: str, operation: str) -> None:
        if self._state != expected:
            self._fail_without_time(
                "illegal_transition:{}:{}->{}".format(
                    operation, self._state, expected
                )
            )

    def mark_base_spawned(self, monotonic_s: float) -> None:
        self._require_state(self.NEW, "mark_base_spawned")
        now = self._finite_monotonic(monotonic_s)
        self._last_event_monotonic_s = now
        self._base_spawn_monotonic_s = now
        self._deadline_monotonic_s = now + self._timeout_s
        self._state = self.BASE_SPAWNED

    def mark_unpause_requested(self, monotonic_s: float) -> None:
        self._require_state(self.BASE_SPAWNED, "mark_unpause_requested")
        self._advance_time(monotonic_s)
        self._state = self.UNPAUSE_REQUESTED

    def mark_unpause_acknowledged(
        self, monotonic_s: float, service_success: bool
    ) -> None:
        self._require_state(
            self.UNPAUSE_REQUESTED, "mark_unpause_acknowledged"
        )
        now = self._advance_time(monotonic_s)
        if service_success is not True:
            self._failure_reason = "unpause_service_not_acknowledged"
            self._state = self.FAILED
            raise R6I2BootstrapError(self._failure_reason)
        self._unpause_ack_monotonic_s = now
        self._state = self.CLOCK_BARRIER_ARMED

    @staticmethod
    def _validate_clock(clock_sec: int, clock_nsec: int) -> Tuple[int, int]:
        if (
            isinstance(clock_sec, bool)
            or isinstance(clock_nsec, bool)
            or not isinstance(clock_sec, int)
            or not isinstance(clock_nsec, int)
            or clock_sec < 0
            or clock_nsec < 0
            or clock_nsec >= 1_000_000_000
        ):
            raise R6I2BootstrapError("malformed_clock_sample")
        return clock_sec, clock_nsec

    def observe_clock(
        self, clock_sec: int, clock_nsec: int, monotonic_s: float
    ) -> bool:
        """Observe a clock sample and return whether the barrier is satisfied."""

        if self._state in {self.NEW, self.FAILED, self.SERVICE_WAIT_RELEASED}:
            self._fail_without_time(
                "clock_observation_not_allowed_in_state:{}".format(
                    self._state
                )
            )
        now = self._advance_time(monotonic_s)
        try:
            clock = self._validate_clock(clock_sec, clock_nsec)
        except R6I2BootstrapError as exc:
            self._failure_reason = str(exc)
            self._state = self.FAILED
            raise
        positive = clock != (0, 0)
        if self._state in {self.BASE_SPAWNED, self.UNPAUSE_REQUESTED}:
            self._pre_ack_clock_samples_ignored += 1
            if positive:
                self._pre_ack_positive_samples_ignored += 1
            return False
        if self._state == self.CLOCK_BARRIER_ARMED:
            self._post_ack_clock_samples += 1
            if not positive:
                self._post_ack_zero_samples += 1
                return False
            self._positive_clock = clock
            self._positive_clock_monotonic_s = now
            self._state = self.POSITIVE_CLOCK_OBSERVED
            return False
        if self._state == self.POSITIVE_CLOCK_OBSERVED:
            self._post_ack_clock_samples += 1
            if clock < self._positive_clock:
                self._failure_reason = "simulation_clock_regressed"
                self._state = self.FAILED
                raise R6I2BootstrapError(self._failure_reason)
            if clock == self._positive_clock:
                self._post_positive_equal_samples += 1
                return False
            self._progress_clock = clock
            self._progress_clock_monotonic_s = now
            self._state = self.CLOCK_PROGRESS_CONFIRMED
            return True
        if self._state == self.CLOCK_PROGRESS_CONFIRMED:
            return True
        self._fail_without_time(
            "unhandled_clock_state:{}".format(self._state)
        )
        return False

    def observe_base_exit(
        self, returncode: Optional[int], monotonic_s: float
    ) -> None:
        """Fail if the base process exits before service-wait release."""

        self._advance_time(monotonic_s)
        if returncode is None:
            return
        if isinstance(returncode, bool) or not isinstance(returncode, int):
            self._fail_without_time("invalid_base_returncode")
        if self._state != self.SERVICE_WAIT_RELEASED:
            self._failure_reason = "base_exited_before_clock_barrier:{}".format(
                returncode
            )
            self._state = self.FAILED
            raise R6I2BootstrapError(self._failure_reason)

    def check_deadline(self, monotonic_s: float) -> None:
        self._advance_time(monotonic_s)

    def release_service_wait(
        self, monotonic_s: float, base_returncode: Optional[int]
    ) -> None:
        """Release later move_base/TEB readiness only after a live barrier."""

        self._require_state(
            self.CLOCK_PROGRESS_CONFIRMED, "release_service_wait"
        )
        self._advance_time(monotonic_s)
        if base_returncode is not None:
            if isinstance(base_returncode, bool) or not isinstance(
                base_returncode, int
            ):
                self._fail_without_time("invalid_base_returncode")
            self._failure_reason = (
                "base_exited_before_service_wait_release:{}".format(
                    base_returncode
                )
            )
            self._state = self.FAILED
            raise R6I2BootstrapError(self._failure_reason)
        self._state = self.SERVICE_WAIT_RELEASED

    def receipt(self) -> Dict[str, object]:
        """Return a value-only receipt suitable for machine-readable evidence."""

        return {
            "protocol_id": BOOTSTRAP_PROTOCOL_ID,
            "state": self._state,
            "terminal": self.terminal,
            "positive_clock_required": True,
            "strict_clock_progression_required": True,
            "topic_existence_is_sufficient": False,
            "zero_clock_is_sufficient": False,
            "single_positive_clock_sample_is_sufficient": False,
            "pre_unpause_positive_clock_is_sufficient": False,
            "service_wait_allowed": self.service_wait_allowed,
            "timeout_s": self._timeout_s,
            "base_spawn_monotonic_s": self._base_spawn_monotonic_s,
            "deadline_monotonic_s": self._deadline_monotonic_s,
            "unpause_ack_monotonic_s": self._unpause_ack_monotonic_s,
            "positive_clock": (
                None
                if self._positive_clock is None
                else {
                    "sec": self._positive_clock[0],
                    "nsec": self._positive_clock[1],
                }
            ),
            "positive_clock_observed_monotonic_s": (
                self._positive_clock_monotonic_s
            ),
            "clock_progression_sample": (
                None
                if self._progress_clock is None
                else {
                    "sec": self._progress_clock[0],
                    "nsec": self._progress_clock[1],
                }
            ),
            "clock_progression_observed_monotonic_s": (
                self._progress_clock_monotonic_s
            ),
            "post_ack_clock_sample_count": self._post_ack_clock_samples,
            "post_ack_zero_clock_sample_count": self._post_ack_zero_samples,
            "post_positive_equal_clock_sample_count": (
                self._post_positive_equal_samples
            ),
            "pre_ack_clock_sample_count_ignored": (
                self._pre_ack_clock_samples_ignored
            ),
            "pre_ack_positive_clock_sample_count_ignored": (
                self._pre_ack_positive_samples_ignored
            ),
            "failure_reason": self._failure_reason,
        }


# These are exact names, not prefixes.  Credential-bearing and remote-control
# variables such as SSH_AUTH_SOCK, proxies, cloud SDK variables, tokens, and
# cookies are intentionally absent.
SAFE_INHERITED_ENVIRONMENT_KEYS = frozenset(
    {
        "CMAKE_PREFIX_PATH",
        "GAZEBO_MODEL_PATH",
        "GAZEBO_PLUGIN_PATH",
        "GAZEBO_RESOURCE_PATH",
        "HOME",
        "LANG",
        "LANGUAGE",
        "LC_ALL",
        "LD_LIBRARY_PATH",
        "LOGNAME",
        "OGRE_RESOURCE_PATH",
        "PATH",
        "PKG_CONFIG_PATH",
        "PYTHONPATH",
        "ROS_DISTRO",
        "ROS_ETC_DIR",
        "ROS_PACKAGE_PATH",
        "ROS_PYTHON_VERSION",
        "ROS_ROOT",
        "ROS_VERSION",
        "SHELL",
        "TZ",
        "USER",
    }
)

FORCED_ENVIRONMENT_KEYS = frozenset(
    {"ROS_HOME", "ROS_IP", "ROS_LOG_DIR", "ROS_MASTER_URI"}
)

_CREDENTIAL_KEY_PATTERN = re.compile(
    r"(?:^|[_-])(?:"
    r"ACCESS[_-]?KEY|API[_-]?KEY|AUTH|BEARER|COOKIE|"
    r"CREDENTIALS?|CRED|PASS(?:WORD|WD)?|PRIVATE[_-]?KEY|"
    r"SECRETS?|SESSION|TOKENS?"
    r")(?:$|[_-])",
    re.IGNORECASE,
)
_ASSIGNMENT_PATTERN = re.compile(
    r"(?i)\b("
    r"[A-Z0-9_-]*(?:ACCESS[_-]?KEY|API[_-]?KEY|AUTH|BEARER|"
    r"COOKIE|CREDENTIALS?|PASS(?:WORD|WD)?|PRIVATE[_-]?KEY|"
    r"SECRETS?|SESSION|TOKENS?)"
    r"[A-Z0-9_-]*"
    r")\s*([=:])\s*([^\s,;]+)"
)
_BEARER_PATTERN = re.compile(r"(?i)\bBearer\s+[A-Za-z0-9._~+/=-]+")
_URI_USERINFO_PATTERN = re.compile(
    r"(?P<scheme>https?://)(?P<userinfo>[^/@\s]+)@"
)
_CREDENTIAL_OPTION_PATTERN = re.compile(
    r"^--?[A-Z0-9_-]*(?:"
    r"ACCESS[_-]?KEY|API[_-]?KEY|BEARER|COOKIE|CREDENTIALS?|"
    r"PASS(?:WORD|WD)?|PRIVATE[_-]?KEY|SECRETS?|SESSION|TOKENS?"
    r")(?:[A-Z0-9_-]*)$",
    re.IGNORECASE,
)


def is_credential_like_key(name: str) -> bool:
    if not isinstance(name, str):
        return True
    return _CREDENTIAL_KEY_PATTERN.search(name.upper()) is not None


def credential_like_key_names(environment: Mapping[str, str]) -> Tuple[str, ...]:
    return tuple(
        sorted(str(key) for key in environment if is_credential_like_key(key))
    )


def _validate_local_ros_master_uri(uri: str) -> str:
    if not isinstance(uri, str) or "\n" in uri or "\x00" in uri:
        raise R6I2EnvironmentPolicyError("ROS_MASTER_URI is malformed")
    try:
        parsed = urlsplit(uri)
        hostname = parsed.hostname
        port = parsed.port
    except ValueError as exc:
        raise R6I2EnvironmentPolicyError(
            "ROS_MASTER_URI is malformed"
        ) from exc
    if (
        parsed.scheme != "http"
        or hostname not in {"127.0.0.1", "localhost", "::1"}
        or parsed.username is not None
        or parsed.password is not None
        or parsed.query
        or parsed.fragment
        or parsed.path not in {"", "/"}
        or port is None
    ):
        raise R6I2EnvironmentPolicyError(
            "ROS_MASTER_URI must be credential-free and loopback-only"
        )
    return uri


def _absolute_attempt_root(attempt_root: Path) -> Path:
    raw = str(attempt_root)
    if "\x00" in raw or "\n" in raw:
        raise R6I2EnvironmentPolicyError("attempt_root is malformed")
    path = Path(attempt_root)
    if not path.is_absolute() or ".." in path.parts:
        raise R6I2EnvironmentPolicyError(
            "attempt_root must be an absolute non-traversing path"
        )
    return path


def _validate_environment_value(key: str, value: str) -> str:
    if not isinstance(value, str):
        raise R6I2EnvironmentPolicyError(
            "environment value must be text: {}".format(key)
        )
    if "\x00" in value or "\n" in value or "\r" in value:
        raise R6I2EnvironmentPolicyError(
            "environment value contains a control delimiter: {}".format(key)
        )
    return value


def build_credential_safe_environment(
    source_environment: Mapping[str, str],
    attempt_root: Path,
    ros_master_uri: str = "http://127.0.0.1:11311",
) -> Tuple[Dict[str, str], Dict[str, object]]:
    """Build an allowlisted child environment and a value-free audit.

    The audit contains key names and policy decisions only.  It never copies a
    removed value, so credential material is not duplicated into evidence.
    """

    if not isinstance(source_environment, Mapping):
        raise R6I2EnvironmentPolicyError(
            "source_environment must be a mapping"
        )
    root = _absolute_attempt_root(Path(attempt_root))
    master_uri = _validate_local_ros_master_uri(ros_master_uri)
    child = {}  # type: Dict[str, str]
    inherited = []
    sensitive_source_values = {
        str(value)
        for key, value in source_environment.items()
        if is_credential_like_key(key) and str(value)
    }
    for key in sorted(SAFE_INHERITED_ENVIRONMENT_KEYS):
        if key not in source_environment:
            continue
        if is_credential_like_key(key):
            raise R6I2EnvironmentPolicyError(
                "credential-like key entered allowlist: {}".format(key)
            )
        candidate = _validate_environment_value(
            key, source_environment[key]
        )
        if any(secret in candidate for secret in sensitive_source_values):
            raise R6I2EnvironmentPolicyError(
                "credential value is aliased through allowlisted key: {}".format(
                    key
                )
            )
        child[key] = candidate
        inherited.append(key)
    child.update(
        {
            "ROS_HOME": str(root / "ros_home"),
            "ROS_IP": "127.0.0.1",
            "ROS_LOG_DIR": str(root / "ros_logs"),
            "ROS_MASTER_URI": master_uri,
        }
    )
    assert_credential_safe_environment(child, root, master_uri)
    removed_credential_keys = credential_like_key_names(source_environment)
    removed_non_allowlisted_keys = tuple(
        sorted(
            str(key)
            for key in source_environment
            if key not in SAFE_INHERITED_ENVIRONMENT_KEYS
            and key not in removed_credential_keys
        )
    )
    audit = {
        "policy_id": ENVIRONMENT_POLICY_ID,
        "allowlist_mode": "exact_key_allowlist",
        "source_value_material_recorded": False,
        "credential_value_copied": False,
        "inherited_key_names": inherited,
        "forced_key_names": sorted(FORCED_ENVIRONMENT_KEYS),
        "removed_credential_like_key_names": list(
            removed_credential_keys
        ),
        "removed_non_allowlisted_key_names": list(
            removed_non_allowlisted_keys
        ),
        "child_key_names": sorted(child),
        "loopback_only_ros_master": True,
        "ros_log_location_confined_to_attempt_root": True,
    }
    return child, audit


def assert_credential_safe_environment(
    environment: Mapping[str, str],
    attempt_root: Path,
    ros_master_uri: str = "http://127.0.0.1:11311",
) -> None:
    root = _absolute_attempt_root(Path(attempt_root))
    expected_master = _validate_local_ros_master_uri(ros_master_uri)
    allowed = SAFE_INHERITED_ENVIRONMENT_KEYS | FORCED_ENVIRONMENT_KEYS
    unexpected = sorted(set(environment) - allowed)
    if unexpected:
        raise R6I2EnvironmentPolicyError(
            "child environment has non-allowlisted keys: {}".format(
                ",".join(unexpected)
            )
        )
    credential_keys = credential_like_key_names(environment)
    if credential_keys:
        raise R6I2EnvironmentPolicyError(
            "child environment has credential-like keys: {}".format(
                ",".join(credential_keys)
            )
        )
    for key, value in environment.items():
        _validate_environment_value(key, value)
    expected = {
        "ROS_HOME": str(root / "ros_home"),
        "ROS_IP": "127.0.0.1",
        "ROS_LOG_DIR": str(root / "ros_logs"),
        "ROS_MASTER_URI": expected_master,
    }
    for key, value in expected.items():
        if environment.get(key) != value:
            raise R6I2EnvironmentPolicyError(
                "forced child environment value mismatched: {}".format(key)
            )


def validate_credential_safe_command(
    command: Sequence[str],
    source_environment: Optional[Mapping[str, str]] = None,
) -> Tuple[str, ...]:
    """Reject credentials in argv before any future subprocess invocation."""

    if (
        isinstance(command, (str, bytes))
        or not isinstance(command, Sequence)
        or not command
    ):
        raise R6I2EnvironmentPolicyError(
            "command must be a non-empty argument sequence"
        )
    sensitive_values = {
        str(value)
        for key, value in (source_environment or {}).items()
        if is_credential_like_key(key) and str(value)
    }
    result = []
    for raw in command:
        if not isinstance(raw, str) or "\x00" in raw or "\n" in raw:
            raise R6I2EnvironmentPolicyError(
                "command contains a malformed argument"
            )
        if (
            _ASSIGNMENT_PATTERN.search(raw)
            or _BEARER_PATTERN.search(raw)
            or _URI_USERINFO_PATTERN.search(raw)
            or _CREDENTIAL_OPTION_PATTERN.fullmatch(raw)
            or any(value in raw for value in sensitive_values)
        ):
            raise R6I2EnvironmentPolicyError(
                "command contains credential-like material"
            )
        result.append(raw)
    return tuple(result)


def redact_credential_material(
    text: str, source_environment: Optional[Mapping[str, str]] = None
) -> str:
    """Redact credential-like material before diagnostic text is persisted."""

    if not isinstance(text, str):
        raise TypeError("text must be str")
    redacted = text
    sensitive = []
    for key, value in (source_environment or {}).items():
        if is_credential_like_key(key) and isinstance(value, str) and value:
            sensitive.append((key, value))
    for key, value in sorted(
        sensitive, key=lambda item: len(item[1]), reverse=True
    ):
        redacted = redacted.replace(
            value, "<redacted:{}>".format(str(key).upper())
        )
    redacted = _BEARER_PATTERN.sub("Bearer <redacted>", redacted)
    redacted = _URI_USERINFO_PATTERN.sub(
        lambda match: "{}<redacted>@".format(match.group("scheme")),
        redacted,
    )
    redacted = _ASSIGNMENT_PATTERN.sub(
        lambda match: "{}{}<redacted>".format(
            match.group(1), match.group(2)
        ),
        redacted,
    )
    return redacted


def credential_safe_log_policy_receipt() -> Dict[str, object]:
    """Return the static, non-authorizing log policy declaration."""

    return {
        "policy_id": LOG_REDACTION_POLICY_ID,
        "execution_authorized": False,
        "ros_or_subprocess_started": False,
        "raw_parent_environment_logging_allowed": False,
        "raw_child_environment_logging_allowed": False,
        "child_environment_policy_id": ENVIRONMENT_POLICY_ID,
        "diagnostic_text_redaction_required_before_persistence": True,
        "removed_source_values_may_be_recorded": False,
        "credential_like_key_names_may_be_recorded": True,
    }
