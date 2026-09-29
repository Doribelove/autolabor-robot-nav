#!/usr/bin/env python3
"""Exercise model selection and the effective nested servo launch parameters."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import roslaunch


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[2]
PT_SHA = "7bf99d4c61343e8cdb37289f2eece6cf18342b508f9b7f80723592edce398500"
ENGINE_SHA = "d540a5e890c88df998a0cb5246519c675901b14af5e7a057db2bef4e0edbd316"


class DetectorModelContractTest(unittest.TestCase):
    def select(self, camera="zed", profile="airport_recall", cache=True, enabled=True):
        source = (WORKSPACE / "scripts/operator_all_in_one.sh").read_text()
        start = source.index("# Select one model identity")
        end = source.index("NAVIGATION_ARGS=", start)
        self.assertLess(end, source.index('"$SCRIPT_DIR/bringup.sh"'))
        # Run only the production selection block: no ROS node, device, goal,
        # command publisher or navigation process can be launched here.
        block = source[start:end]
        with tempfile.TemporaryDirectory() as temporary:
            engine = Path(temporary) / "test.engine"
            if cache:
                engine.write_bytes(b"test fixture only")
                Path(str(engine) + ".sha256").write_text(ENGINE_SHA + "\n")
            environment = dict(os.environ, OPERATOR_CAMERA_DRIVER=camera,
                               OPERATOR_FOD_PROFILE=profile,
                               OPERATOR_START_VISION=str(enabled).lower(),
                               OPERATOR_ZED_TENSORRT_ENGINE=str(engine),
                               FOD_RECOVERY_EXPECTED_MODEL_SHA256="c" * 64)
            query = "python3 -c 'import os,sys,json; print(json.dumps([os.environ.get(\"FOD_RECOVERY_EXPECTED_MODEL_SHA256\"),sys.argv[1:]]))' \"${VISION_ARGS[@]}\""
            output = subprocess.check_output(["bash", "-euc", block + query], env=environment, text=True)
            return json.loads(output.splitlines()[-1])

    def launch_sha(self, expected=None, override=None, standalone=False, custom_config=None):
        environment = dict(os.environ)
        environment.pop("FOD_RECOVERY_EXPECTED_MODEL_SHA256", None)
        if expected is not None:
            environment["FOD_RECOVERY_EXPECTED_MODEL_SHA256"] = expected
        argv = ["camera_driver:=zed"]
        if override is not None:
            argv.append("expected_model_sha256:=" + override)
        if custom_config is not None:
            argv.append("config:=" + str(custom_config))
        filename = "visual_recovery.launch" if standalone else "gps_visual_recovery_standby.launch"
        config = roslaunch.config.ROSLaunchConfig()
        with patch.dict(os.environ, environment, clear=True):
            roslaunch.xmlloader.XmlLoader().load(str(PACKAGE / "launch" / filename), config,
                                               argv=argv, verbose=False)
        return config.params["/fod_visual_servo/expected_model_sha256"].value

    def test_native_engine_uses_same_digest_for_detector_and_servo(self):
        identity, arguments = self.select()
        self.assertEqual(identity, ENGINE_SHA)
        self.assertIn("expected_model_sha256:=" + identity, arguments)
        self.assertTrue(any(arg.startswith("weights:=") for arg in arguments))
        self.assertEqual(self.launch_sha(identity), identity)

    def test_missing_or_disabled_engine_restores_pt_identity_on_both_sides(self):
        identity, arguments = self.select(cache=False)
        self.assertEqual(identity, PT_SHA)
        self.assertEqual(arguments, ["expected_model_sha256:=" + PT_SHA])
        self.assertEqual(self.launch_sha(identity), PT_SHA)

    def test_original_profile_does_not_retain_an_engine_identity(self):
        identity, arguments = self.select(profile="original")
        self.assertEqual(identity, PT_SHA)
        self.assertEqual(arguments, ["expected_model_sha256:=" + PT_SHA])
        self.assertEqual(self.launch_sha(identity), PT_SHA)

    def test_hikrobot_keeps_original_model(self):
        identity, arguments = self.select(camera="hikrobot")
        self.assertEqual(identity, PT_SHA)
        self.assertEqual(arguments, ["expected_model_sha256:=" + PT_SHA])

    def test_external_detector_identity_is_preserved_when_sidecar_is_disabled(self):
        identity, arguments = self.select(enabled=False)
        self.assertEqual(identity, "c" * 64)
        self.assertEqual(arguments, [])
        self.assertEqual(self.launch_sha(identity), identity)

    def test_standalone_default_and_explicit_override(self):
        self.assertEqual(self.launch_sha(standalone=True), PT_SHA)
        self.assertEqual(self.launch_sha(ENGINE_SHA, standalone=True), ENGINE_SHA)
        self.assertEqual(self.launch_sha(ENGINE_SHA, override="b" * 64), "b" * 64)

    def test_custom_config_identity_is_preserved_without_explicit_override(self):
        with tempfile.TemporaryDirectory() as temporary:
            config=Path(temporary)/"custom.yaml"
            config.write_text('expected_model_sha256: "' + 'c'*64 + '"\n')
            self.assertEqual(self.launch_sha(standalone=True,custom_config=config),'c'*64)


if __name__ == "__main__":
    unittest.main()
