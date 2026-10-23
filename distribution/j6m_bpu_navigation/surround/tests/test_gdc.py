"""Offline GDC admission and no-CPU-fallback checks; no real SDK or device."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

from distribution.j6m_bpu_navigation.surround import CAMERAS
from distribution.j6m_bpu_navigation.surround.capture import NativeCapture, capture_issues
from distribution.j6m_bpu_navigation.surround.display import Renderer
from distribution.j6m_bpu_navigation.surround.geometry import Rectifier, PoseBuffer
from distribution.j6m_bpu_navigation.surround.pipeline import Pipeline
from distribution.j6m_bpu_navigation.surround.__main__ import live
from test_surround import calibration, frame, T0
import test_surround as fixtures


class GdcTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.cal = calibration()
        self.cal.synthetic = False
        self.cal.data["synthetic"] = False
        self.bin = self.root / "perspective.bin"
        self.bin.write_bytes(b"TEST ONLY LUT123")
        digest = hashlib.sha256(self.bin.read_bytes()).hexdigest()
        self.vpm = {}
        cameras = {}
        for index, name in enumerate(CAMERAS):
            self.cal.cameras[name]["gdc_sha256"] = digest
            cameras[name] = dict(pipeline=index, kind="pym", layout="NV12", pym_layer=0,
                                 vendor_format=30, format_verified=True, gdc_bin=str(self.bin),
                                 clock_mapping_file=str(self.root / "clock.json"))
            self.vpm["pipeline%d" % index] = dict(
                gdc_node0=dict(input_width=640, input_height=480, output_width=640, output_height=480),
                pym_node0=dict(chn=dict(src_in_width=640, src_in_height=480,
                    ds_roi_en=1, ds_roi_uv_bypass=0, ds_roi=[dict(
                        ds_roi_layer=0, ds_roi_sel=0, ds_roi_start_left=0, ds_roi_start_top=0,
                        ds_roi_region_width=640, ds_roi_region_height=480,
                        ds_roi_out_width=640, ds_roi_out_height=480)])))
        self.config = dict(board_verified=True, port_mapping_verified=True, cameras=cameras,
            exclusive_lock=str(self.root / "camera.lock"), vpm_config=str(self.root / "vpm.json"),
            camera_config=str(self.root / "camera.json"), library=str(self.bin),
            vio_library=str(self.bin), hbmem_library=str(self.bin))
        (self.root / "clock.json").write_text(json.dumps(fixtures.TimingSchedulerTests().mapping()))
        (self.root / "camera.json").write_text("{}")
        self.save_vpm()

    def save_vpm(self):
        (self.root / "vpm.json").write_text(json.dumps(self.vpm))

    def issues(self):
        return capture_issues(self.config, calibration=self.cal)

    def test_four_verified_gdc_paths_pass_preflight(self):
        self.assertEqual(self.issues(), [])

    def test_missing_or_wrong_lut_blocks_before_sdk_open(self):
        for value in (None, str(self.root / "missing.bin")):
            with self.subTest(value=value):
                self.config["cameras"]["rear"]["gdc_bin"] = value
                self.assertTrue(any("rear: Calibrated perspective GDC bin missing" in e for e in self.issues()))
                with patch("distribution.j6m_bpu_navigation.surround.capture.C.CDLL") as sdk:
                    with self.assertRaises(ValueError):
                        NativeCapture(self.config, self.cal, Mock())
                    sdk.assert_not_called()
                self.assertFalse((self.root / "camera.lock").exists())
        self.config["cameras"]["rear"]["gdc_bin"] = str(self.bin)
        self.bin.write_bytes(b"WRONG LUT DATA!!")
        self.assertTrue(any("SHA256" in e for e in self.issues()))

    def test_empty_and_unaligned_luts_rejected(self):
        for data in (b"", b"123"):
            self.bin.write_bytes(data)
            self.assertTrue(any("4-byte aligned" in e for e in self.issues()))

    def test_cim_and_lower_pym_layers_cannot_bypass_gdc(self):
        camera = self.config["cameras"]["front"]
        for change in (dict(kind="cim", layout="YUYV"), dict(layout="NV16"), dict(pym_layer=1)):
            original = camera.copy()
            camera.update(change)
            self.assertTrue(any(e.startswith("front:") and "GDC" in e for e in self.issues()))
            camera.clear()
            camera.update(original)

    def test_vpm_cropping_or_output_size_mismatch_rejected(self):
        pipeline = self.vpm["pipeline0"]
        original = copy.deepcopy(pipeline)
        pipeline["pym_node0"]["chn"]["ds_roi"][0]["ds_roi_region_height"] = 400
        self.save_vpm()
        self.assertTrue(any("full calibrated image" in e for e in self.issues()))
        self.vpm["pipeline0"] = original
        original["gdc_node0"]["output_width"] = 320
        self.save_vpm()
        self.assertTrue(any("sizes differ" in e for e in self.issues()))

    def test_live_admission_happens_before_bpu_and_output_creation(self):
        cal_file = self.root / "calibration.json"
        cal_file.write_text(json.dumps(self.cal.data))
        self.config["cameras"]["left"]["gdc_bin"] = None
        config = dict(calibration=str(cal_file), capture=self.config, inference={})
        with patch("distribution.j6m_bpu_navigation.surround.__main__.BpuEngine") as bpu:
            with self.assertRaisesRegex(ValueError, "GDC bin missing"):
                live(config, self.root, self.root / "output", 1)
            bpu.assert_not_called()
        self.assertFalse((self.root / "output").exists())

    def test_raw_calibration_acquisition_remains_explicit(self):
        self.assertTrue(capture_issues(self.config, require_clock=False, require_gdc=False))
        for c in self.config["cameras"].values():
            c.update(gdc_bin=None, kind="cim", layout="YUYV", clock_mapping_file=None)
        self.assertEqual(capture_issues(self.config, require_clock=False, require_gdc=False), [])
        self.assertTrue(self.issues())

    def test_live_pipeline_rejects_raw_image_without_cpu_work_or_inference(self):
        f = frame()
        f.synthetic, f.rectified = False, False
        engine = Mock()
        pipeline = Pipeline(self.cal, engine, PoseBuffer(), {})
        with patch("cv2.fisheye.initUndistortRectifyMap", side_effect=AssertionError("CPU map")), \
             patch("cv2.remap", side_effect=AssertionError("CPU remap")):
            result = pipeline.process(f, T0)
        self.assertEqual(result["state"], "awaiting_gdc")
        self.assertEqual(result["detections"], [])
        engine.infer.assert_not_called()

    def test_gdc_frame_is_passed_to_detector_unchanged(self):
        f = frame()
        f.synthetic = False
        engine = Mock()
        engine.infer.return_value = ([], {})
        with patch("cv2.remap", side_effect=AssertionError("CPU rectification")):
            result = Pipeline(self.cal, engine, PoseBuffer(), {}).process(f, T0)
        self.assertIs(engine.infer.call_args[0][0], f.image)
        self.assertEqual(result["inference_image"]["rectification_backend"], "gdc")

    def test_live_preview_does_not_rectify_raw_frames(self):
        with patch("cv2.fisheye.initUndistortRectifyMap", side_effect=AssertionError("CPU map")), \
             patch("cv2.remap", side_effect=AssertionError("CPU remap")):
            image = Renderer(self.cal).render(dict(synthetic=False),
                                              {"front": (T0, frame().image, False)}, T0)
        self.assertEqual(image.shape, (640, 1280, 3))

    def test_cpu_reference_requires_explicit_offline_call(self):
        f = frame()
        f.rectified = False
        with self.assertRaisesRegex(ValueError, "offline only"):
            Rectifier(self.cal).apply(f)
        output = Rectifier(self.cal, offline_reference=True).apply(f)
        self.assertEqual(output.shape, f.image.shape)

    def test_reader_rejects_missing_gdc_flag_and_wrong_size_before_decode(self):
        # Exercise the real reader admission with a device-free C ABI stand-in.
        for gdc, width, height, message in ((0, 640, 480, "completion flag"),
                                          (1, 320, 240, "output size")):
            cap = NativeCapture.__new__(NativeCapture)
            cap.config, cap.calibration, cap.calibration_only = self.config, self.cal, False
            cap.stop_event, cap.errors = threading.Event(), {}
            cap.context, cap.session, cap.on_frame = None, "test", Mock()
            cap.lib = Mock()
            def read(*args):
                meta = args[-1]._obj
                meta.vendor_format, meta.gdc_applied = 30, gdc
                meta.width, meta.height = width, height
                cap.stop_event.set()
                return 0
            cap.lib.sc_read.side_effect = read
            with patch("distribution.j6m_bpu_navigation.surround.capture.decode_yuv") as decode:
                cap._reader("front")
                decode.assert_not_called()
            cap.on_frame.assert_not_called()
            self.assertIn(message, cap.errors["front"])


if __name__ == "__main__":
    unittest.main()
