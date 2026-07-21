#!/usr/bin/env python3
"""R6 typed transaction node with explicit two-phase startup restoration."""

import importlib.util
import json
from pathlib import Path
import threading

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

from teb_mode_manager.action_pipeline import ActionPipelineError
from teb_mode_manager.r6_execution_integration import (
    R6ExecutionIntegrationError,
    R6TwoPhaseIdempotentTypedTebTransactionBackend,
    R6TwoPhaseRestoreError,
    canonical_attempt_identity,
)
from teb_mode_manager.typed_teb_transaction import (
    EXPECTED_TEB_NAMESPACE,
    RosTypedDynamicReconfigureAdapter,
)


R2_NODE = Path(__file__).with_name(
    "v2_04g_r2_typed_anchor_transaction_node.py"
)
_SPEC = importlib.util.spec_from_file_location(
    "v2_04g_r2_frozen_transaction_node", R2_NODE
)
_R2 = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_R2)

# The frozen R2 node resolves the backend through the dynamically loaded R1
# module.  Replace only that execution-side class; its bounded context join,
# mechanism controller, trace schema, and timer behavior remain unchanged.
_R2._R1.TypedTebTransactionBackend = (  # pylint: disable=protected-access
    R6TwoPhaseIdempotentTypedTebTransactionBackend
)


def _attempt_identity_from_params():
    return canonical_attempt_identity({
        "stage": rospy.get_param("~attempt_stage"),
        "profile_id": rospy.get_param("~attempt_profile_id"),
        "scene_id": rospy.get_param("~attempt_scene_id"),
        "seed": rospy.get_param("~attempt_seed"),
        "attempt": rospy.get_param("~attempt_number"),
    })


def _sha256_param(name):
    value = rospy.get_param(name)
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(character not in "0123456789abcdef" for character in value)
    ):
        raise R6ExecutionIntegrationError(
            "{} must be a lowercase SHA256".format(name)
        )
    return value


class R6SimulationTypedAnchorTransactionNode(
    _R2.SimulationTypedAnchorTransactionNode
):
    """Frozen R2 episode transaction plus an explicit restore service."""

    def __init__(self):
        self.attempt_identity = _attempt_identity_from_params()
        self.supervisor_config_sha256 = _sha256_param(
            "~supervisor_config_sha256"
        )
        # The frozen base registers our overridden shutdown callback before it
        # creates its own lock.  Keep early-construction failures safe too.
        self.lock = threading.RLock()
        self.timer = None
        self.execution_armed = False
        self.teardown_service_started = False
        self.cached_teardown_receipt_json = None
        self.cached_teardown_success = False
        self.independent_adapter = None
        super().__init__()
        if not isinstance(
            self.backend, R6TwoPhaseIdempotentTypedTebTransactionBackend
        ):
            raise R6ExecutionIntegrationError(
                "R6 two-phase backend was not installed"
            )
        self.teb_namespace = rospy.get_param(
            "~teb_namespace", EXPECTED_TEB_NAMESPACE
        )
        self.transaction_timeout_s = float(
            rospy.get_param("~transaction_timeout_s", 2.0)
        )
        self.startup_profile_publisher = rospy.Publisher(
            rospy.get_param(
                "~startup_profile_topic",
                "/teb_mode_manager/r6/startup_profile",
            ),
            String,
            queue_size=1,
            latch=True,
        )
        self.teardown_receipt_publisher = rospy.Publisher(
            rospy.get_param(
                "~teardown_receipt_topic",
                "/teb_mode_manager/r6/teardown_receipt",
            ),
            String,
            queue_size=1,
            latch=True,
        )
        startup_document = self.backend.startup_capture.as_document(
            self.attempt_identity
        )
        startup_document["supervisor_config_sha256"] = (
            self.supervisor_config_sha256
        )
        self.startup_profile_publisher.publish(
            String(data=json.dumps(startup_document, sort_keys=True))
        )
        self.restore_service = rospy.Service(
            rospy.get_param(
                "~restore_startup_service",
                "/teb_mode_manager/r6/restore_startup",
            ),
            Trigger,
            self._restore_startup,
        )
        self.arm_service = rospy.Service(
            rospy.get_param(
                "~arm_execution_service",
                "/teb_mode_manager/r6/arm_execution",
            ),
            Trigger,
            self._arm_execution,
        )

    def _tick(self, event):
        # Initialization needs a live move_base server, but an episode write
        # is forbidden until the runner captures the startup payload and arms
        # this exact attempt.
        with self.lock:
            if not self.execution_armed or self.teardown_service_started:
                return
            return super()._tick(event)

    def _arm_execution(self, _request):
        with self.lock:
            if self.teardown_service_started:
                return TriggerResponse(
                    success=False,
                    message="R6 teardown already started; arm denied",
                )
            if not self.backend.backend_alive:
                return TriggerResponse(
                    success=False,
                    message="R6 typed backend is not alive; arm denied",
                )
            self.execution_armed = True
            message = {
                "schema_version": "2.0",
                "record_type": "r6_execution_arm_receipt",
                "identity": dict(self.attempt_identity),
                "execution_armed": True,
                "startup_profile_sha256": (
                    self.backend.startup_capture.sha256
                ),
                "supervisor_config_sha256": (
                    self.supervisor_config_sha256
                ),
            }
            return TriggerResponse(
                success=True, message=json.dumps(message, sort_keys=True)
            )

    def _failure_receipt(self, exc):
        startup = self.backend.startup_capture
        return {
            "schema_version": "2.0",
            "record_type": "r6_two_phase_teardown_receipt",
            **self.attempt_identity,
            "identity": dict(self.attempt_identity),
            "status": "fail",
            "failure_reason": "{}: {}".format(type(exc).__name__, exc),
            "restore_requested_while_backend_alive": (
                self.backend.teardown_attempted
                and self.backend.backend_alive
            ),
            "transaction_acknowledged": False,
            "transaction_readback_match": False,
            "independent_readback_match": False,
            "backend_alive_after_restore": (
                self.backend.backend_alive
            ),
            "startup_profile_sha256": startup.sha256,
            "supervisor_config_sha256": self.supervisor_config_sha256,
            "startup_profile_canonical_json": (
                startup.payload.decode("utf-8")
            ),
            "transaction_ack_sha256": None,
            "transaction_ack_canonical_json": None,
            "transaction_readback_sha256": None,
            "transaction_readback_canonical_json": None,
            "independent_readback_sha256": None,
            "independent_readback_canonical_json": None,
            "restore_t_request_s": None,
            "restore_t_ack_s": None,
            "restore_t_readback_s": None,
            "restore_t_active_s": None,
            "independent_readback_t_s": None,
        }

    def _restore_startup(self, _request):
        with self.lock:
            if self.cached_teardown_receipt_json is not None:
                # Evidence retrieval is idempotent; no second restore occurs.
                return TriggerResponse(
                    success=self.cached_teardown_success,
                    message=self.cached_teardown_receipt_json,
                )
            self.teardown_service_started = True
            self.execution_armed = False
            self.timer.shutdown()
            receipt = None
            try:
                def independent_reader():
                    # Construct the independent client only after the primary
                    # restore transaction has completed.  A client created
                    # before restore could return a pre-restore cached config
                    # without a post-restore generation barrier.
                    self.independent_adapter = (
                        RosTypedDynamicReconfigureAdapter(
                            self.teb_namespace, self.transaction_timeout_s
                        )
                    )
                    values = self.independent_adapter.get_configuration(
                        self.transaction_timeout_s
                    )
                    if values is None:
                        raise R6ExecutionIntegrationError(
                            "independent final readback timed out"
                        )
                    return values

                receipt = self.backend.restore_startup_two_phase(
                    self.attempt_identity, independent_reader
                )
            except R6TwoPhaseRestoreError as exc:
                receipt = exc.receipt
            except Exception as exc:  # fail closed and persist a receipt
                receipt = self._failure_receipt(exc)
            finally:
                if self.independent_adapter is not None:
                    try:
                        self.independent_adapter.close()
                    except Exception as exc:
                        if receipt is None or receipt.get("status") == "pass":
                            receipt = self._failure_receipt(exc)
                    self.independent_adapter = None
            self.cached_teardown_success = (
                receipt is not None and receipt.get("status") == "pass"
            )
            receipt["supervisor_config_sha256"] = (
                self.supervisor_config_sha256
            )
            self.cached_teardown_receipt_json = json.dumps(
                receipt, sort_keys=True
            )
            self.teardown_receipt_publisher.publish(
                String(data=self.cached_teardown_receipt_json)
            )
            return TriggerResponse(
                success=self.cached_teardown_success,
                message=self.cached_teardown_receipt_json,
            )

    def _shutdown(self):
        with self.lock:
            self.execution_armed = False
            try:
                self.timer.shutdown()
            except Exception:
                pass
            if self.independent_adapter is not None:
                try:
                    self.independent_adapter.close()
                except Exception as exc:
                    rospy.logerr(
                        "failed to close R6 independent reader: %s", exc
                    )
                self.independent_adapter = None
            try:
                # After a verified explicit restore this performs no write.
                # Otherwise it retains the frozen backend's best-effort safety
                # restoration, which never creates passing teardown evidence.
                self.backend.close()
            except Exception as exc:
                rospy.logerr(
                    "R6 startup typed-profile restoration was not proven: %s",
                    exc,
                )


def main():
    rospy.init_node("v2_04g_r6_typed_anchor_transaction")
    try:
        R6SimulationTypedAnchorTransactionNode()
    except (
        ActionPipelineError,
        R6ExecutionIntegrationError,
        RuntimeError,
        ValueError,
    ) as exc:
        rospy.logfatal("R6 simulation typed transaction denied: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
