import copy
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import numpy as np
from qt_bpu_bridge import prepare, annotate, fresh, frame_period, NV12Preprocessor, IMAGE_TOPIC, STATUS_TOPIC, FRAME_ID
from live_protocol import PAYLOAD_BYTES


class QtBridgeTest(unittest.TestCase):
    def setUp(self):
        self.frame = np.zeros((360, 640, 3), np.uint8)
        self.result = dict(sequence=3, stamp_ns=123, width=640, height=360,
                           demo_only=True, motion_eligible=False,
                           detections=[dict(class_id=0, confidence=0.8,
                                            bbox_model_px=[14, 28, 140, 280])])

    def test_fixed_nv12_and_aspect_ratio(self):
        payload, scales = prepare(self.frame)
        self.assertEqual(len(payload), PAYLOAD_BYTES)
        self.assertEqual(scales, (1.4, 1.4))
        self.assertEqual(set(payload[:896 * 896]), {16})
        self.assertEqual(set(payload[896 * 896:]), {128})

    def test_reused_preprocessing_matches_original_and_clears_old_shape(self):
        import cv2
        random = np.random.default_rng(20)
        cached = NV12Preprocessor()
        previous = None
        for width, height in ((640, 360), (640, 360), (810, 1080), (643, 361), (896, 896)):
            frame = random.integers(0, 256, (height, width, 3), dtype=np.uint8)
            scale = min(896 / width, 896 / height)
            w, h = round(width * scale), round(height * scale)
            canvas = np.zeros((896, 896, 3), np.uint8)
            canvas[:h, :w] = cv2.resize(frame, (w, h))
            i420 = cv2.cvtColor(canvas, cv2.COLOR_BGR2YUV_I420).reshape(-1)
            count = 896 * 896
            uv = np.stack((i420[count:count * 5 // 4], i420[count * 5 // 4:]), axis=-1).reshape(-1)
            expected = i420[:count].tobytes() + uv.tobytes()
            actual, _ = cached.prepare(frame)
            self.assertEqual(actual, expected)
            if previous is not None:
                self.assertEqual(previous[0], previous[1])
            previous = (actual, bytes(actual))

    def test_source_image_is_not_changed_and_coordinates_match(self):
        image = annotate(self.frame, (1.4, 1.4), self.result, 3, 123, {0: "person"})
        self.assertTrue(np.any(image))
        self.assertFalse(np.any(self.frame))
        self.assertEqual(self.result['detections'][0]['bbox_source_px'], [10, 20, 100, 200])

    def test_wrong_frame_or_control_contract_rejected(self):
        for key, value in (('sequence', 2), ('stamp_ns', 124), ('width', 1280),
                           ('height', 720), ('demo_only', False), ('motion_eligible', True)):
            result = dict(self.result, **{key: value})
            with self.subTest(key=key), self.assertRaises(ValueError):
                annotate(self.frame, (1.4, 1.4), result, 3, 123, {0: 'person'})

    def test_invalid_detection_rejected(self):
        for key, value in (('confidence', float('nan')), ('confidence', 1.1),
                           ('class_id', 100), ('class_id', True),
                           ('bbox_model_px', [0, 0, float('inf'), 4])):
            result = copy.deepcopy(self.result)
            result['detections'][0][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                annotate(self.frame, (1.4, 1.4), result, 3, 123, {0: 'person'})

    def test_source_time_bounds(self):
        self.assertTrue(fresh(100, 102))
        for stamp, now in ((0, 0), (100, 104), (104, 100), (float('nan'), 100)):
            self.assertFalse(fresh(stamp, now))

    def test_frequency_bounds(self):
        self.assertEqual(frame_period(0.5), 2.0)
        self.assertEqual(frame_period(30), 1 / 30)
        for rate in (0, -1, 31, float('nan'), float('inf')):
            with self.subTest(rate=rate), self.assertRaises(ValueError):
                frame_period(rate)

    def test_bounded_display_only_source_contract(self):
        source = Path(__file__).with_name('qt_bpu_bridge.py').read_text()
        self.assertEqual(IMAGE_TOPIC, '/fod/bpu_preview/image')
        self.assertEqual(STATUS_TOPIC, '/fod/bpu_preview/status')
        self.assertEqual(FRAME_ID, 'bpu_preview_demo_only')
        self.assertEqual(source.count('rospy.Publisher('), 2)
        for token in ('get_num_connections() == 0', 'subscriber.unregister()',
                      'stop_worker(worker)', 'next_frame = prepared + period',
                      'queue_size=1', 'next_snapshot = time.monotonic() + 5.0',
                      'message.header.stamp.to_nsec() <= last_stamp'):
            self.assertIn(token, source)
        for token in ('/cmd_vel', 'FodDetectionArray', 'move_base', 'import torch',
                      'cv2.imshow', 'VideoCapture('):
            self.assertNotIn(token, source)

    def test_fixture_cannot_launch_chassis_and_rviz_default_unchanged(self):
        path = Path(__file__).resolve().parents[1] / 'qt_preview.launch'
        root = ET.parse(path).getroot()
        self.assertEqual(len(root.findall('.//include')), 2)
        self.assertEqual(root.find(".//arg[@name='enable_rviz']").attrib['value'], 'false')
        self.assertEqual(root.find(".//arg[@name='initial_bpu_preview']").attrib['value'], 'true')
        self.assertIsNone(root.find('.//node'))


if __name__ == '__main__':
    unittest.main()
