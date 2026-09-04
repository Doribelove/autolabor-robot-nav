"""Capability-aware runtime control for the active detector confidence."""

from dataclasses import dataclass
import math
import threading
from typing import Callable, Optional

from dynamic_reconfigure.msg import (
    BoolParameter,
    Config,
    DoubleParameter,
    StrParameter,
)
from dynamic_reconfigure.srv import ReconfigureResponse


GLOBAL_CONFIDENCE_PARAM = "/fod/vision/detector_confidence"
CONFIDENCE_SERVICE = "~set_detection_confidence"
CONFIDENCE_PARAMETER = "detector_confidence"
MIN_DETECTION_CONFIDENCE = 0.05
MAX_DETECTION_CONFIDENCE = 0.95


def validate_detection_confidence(value: float) -> float:
    confidence = float(value)
    if not math.isfinite(confidence):
        raise ValueError("detector confidence must be finite")
    if not MIN_DETECTION_CONFIDENCE <= confidence <= MAX_DETECTION_CONFIDENCE:
        raise ValueError(
            "detector confidence must be in [{:.2f}, {:.2f}]".format(
                MIN_DETECTION_CONFIDENCE, MAX_DETECTION_CONFIDENCE
            )
        )
    return confidence


def requested_confidence(config: Config) -> Optional[float]:
    for parameter in config.doubles:
        if parameter.name == CONFIDENCE_PARAMETER:
            return float(parameter.value)
    return None


@dataclass(frozen=True)
class ConfidenceUpdate:
    backend_id: str
    supported: bool
    accepted: bool
    effective: float
    message: str


class DetectionConfidenceController:
    """Thread-safe adapter shared by every selectable vision backend."""

    def __init__(
        self,
        backend_id: str,
        initial: float,
        setter: Optional[Callable[[float], None]],
    ):
        self.backend_id = str(backend_id)
        self._current = validate_detection_confidence(initial)
        self._setter = setter
        self._lock = threading.Lock()

    @property
    def supported(self) -> bool:
        return self._setter is not None

    @property
    def current(self) -> float:
        with self._lock:
            return self._current

    def update(self, requested: Optional[float]) -> ConfidenceUpdate:
        with self._lock:
            if not self.supported:
                return ConfidenceUpdate(
                    backend_id=self.backend_id,
                    supported=False,
                    accepted=False,
                    effective=self._current,
                    message=(
                        "{} does not expose a calibrated per-box detection "
                        "confidence".format(self.backend_id)
                    ),
                )
            if requested is None:
                return ConfidenceUpdate(
                    backend_id=self.backend_id,
                    supported=True,
                    accepted=True,
                    effective=self._current,
                    message="current detector confidence",
                )
            try:
                value = validate_detection_confidence(requested)
                self._setter(value)
            except Exception as error:
                return ConfidenceUpdate(
                    backend_id=self.backend_id,
                    supported=True,
                    accepted=False,
                    effective=self._current,
                    message=str(error),
                )
            self._current = value
            return ConfidenceUpdate(
                backend_id=self.backend_id,
                supported=True,
                accepted=True,
                effective=self._current,
                message="detector confidence applied to active backend",
            )

    def service_response(self, request) -> ReconfigureResponse:
        result = self.update(requested_confidence(request.config))
        config = Config()
        config.doubles.append(
            DoubleParameter(
                name=CONFIDENCE_PARAMETER,
                value=float(result.effective),
            )
        )
        config.bools.extend(
            [
                BoolParameter(name="supported", value=result.supported),
                BoolParameter(name="accepted", value=result.accepted),
            ]
        )
        config.strs.extend(
            [
                StrParameter(name="backend_id", value=result.backend_id),
                StrParameter(name="message", value=result.message),
            ]
        )
        return ReconfigureResponse(config=config)
