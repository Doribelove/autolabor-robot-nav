import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from replay_assets import recorded_map_directory


class RelocatedRecordingTest(unittest.TestCase):
    def test_moved_recording_prefers_bundled_map_over_old_machine_path(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            old = root / 'old_session_map'
            old.mkdir()
            (old / 'manifest.yaml').write_text('map_id: old\n')
            (root / 'session.json').write_text(json.dumps({'local_map_directory': str(old)}))
            bundled = root / 'assets/static_map'
            bundled.mkdir(parents=True)
            (bundled / 'manifest.yaml').write_text('map_id: correct\n')
            self.assertEqual(recorded_map_directory(root), bundled)

    def test_legacy_recording_still_uses_original_map(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            old = root / 'old_map'
            old.mkdir()
            (old / 'manifest.yaml').write_text('map_id: legacy\n')
            (root / 'session.json').write_text(json.dumps({'local_map_directory': str(old)}))
            self.assertEqual(recorded_map_directory(root), old)

    def test_missing_map_reports_transfer_destination(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / 'session.json').write_text('{}')
            with self.assertRaisesRegex(FileNotFoundError, 'assets/static_map'):
                recorded_map_directory(root)


if __name__ == '__main__':
    unittest.main()
