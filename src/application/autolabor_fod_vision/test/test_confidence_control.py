#!/usr/bin/env python3

import math
import unittest

from dynamic_reconfigure.msg import Config, DoubleParameter

from autolabor_fod_vision.confidence_control import (
    CONFIDENCE_PARAMETER,
    DetectionConfidenceController,
    validate_detection_confidence,
)


class DetectionConfidenceControllerTest(unittest.TestCase):
    def test_supported_backend_applies_and_reports_effective_value(self):
        applied = []
        controller = DetectionConfidenceController("yolo", 0.25, applied.append)
        request = type("Request", (), {"config": Config()})()
        request.config.doubles.append(
            DoubleParameter(name=CONFIDENCE_PARAMETER, value=0.20)
        )

        response = controller.service_response(request)

        self.assertEqual(applied, [0.20])
        self.assertAlmostEqual(controller.current, 0.20)
        self.assertTrue(next(item.value for item in response.config.bools if item.name == "supported"))
        self.assertTrue(next(item.value for item in response.config.bools if item.name == "accepted"))
        self.assertAlmostEqual(
            next(item.value for item in response.config.doubles if item.name == CONFIDENCE_PARAMETER),
            0.20,
        )

    def test_unsupported_backend_never_claims_application(self):
        controller = DetectionConfidenceController("locateanything", 0.20, None)
        outcome = controller.update(0.30)
        self.assertFalse(outcome.supported)
        self.assertFalse(outcome.accepted)
        self.assertAlmostEqual(outcome.effective, 0.20)

    def test_invalid_values_are_rejected_without_changing_runtime(self):
        applied = []
        controller = DetectionConfidenceController("detect_and_classify", 0.20, applied.append)
        for value in (float("nan"), float("inf"), 0.01, 0.99):
            outcome = controller.update(value)
            self.assertFalse(outcome.accepted)
            self.assertAlmostEqual(outcome.effective, 0.20)
        self.assertEqual(applied, [])

    def test_validation_range_includes_requested_default(self):
        self.assertTrue(math.isclose(validate_detection_confidence(0.20), 0.20))


if __name__ == "__main__":
    unittest.main()
