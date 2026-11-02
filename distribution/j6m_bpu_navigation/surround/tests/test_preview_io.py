import hashlib
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

import numpy as np

from surround.preview_io import SnapshotWriter


class SnapshotTests(unittest.TestCase):
    def test_slow_disk_does_not_queue_old_images_or_block_producer(self):
        entered, release, finished = threading.Event(), threading.Event(), threading.Event()
        written = []

        def write(result):
            written.append(result)
            if len(written) == 1:
                entered.set()
                if not release.wait(3):
                    raise RuntimeError('test writer not released')
            else:
                finished.set()

        with tempfile.TemporaryDirectory() as directory, patch.object(SnapshotWriter, '_write', side_effect=write):
            writer = SnapshotWriter(Path(directory))
            try:
                writer.submit(1)
                self.assertTrue(entered.wait(2))
                writer.submit(2)
                writer.submit(3)
                self.assertEqual(writer.metrics()['replaced'], 1)
                release.set()
                self.assertTrue(finished.wait(2))
                self.assertEqual(written, [1, 3])
            finally:
                release.set()
                writer.close()
            self.assertFalse(writer.thread.is_alive())

    def test_snapshot_hashes_cover_all_inputs_without_mutating_live_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            writer = SnapshotWriter(output)
            image = np.full((8, 8, 3), 72, dtype=np.uint8)
            metadata = dict(fusion_sequence=12, source_exposure_synchronized=False)
            try:
                writer._write((dict(front=image), image, np.ones((8, 8), np.uint8), {}, metadata))
                saved = json.loads((output/'bev.json').read_text())
                self.assertEqual(metadata, dict(fusion_sequence=12, source_exposure_synchronized=False))
                self.assertEqual(set(saved['snapshot_files_sha256']),
                                 {'front_upright.png', 'bev.png', 'projection_support.png'})
                for name, digest in saved['snapshot_files_sha256'].items():
                    self.assertEqual(hashlib.sha256((output/name).read_bytes()).hexdigest(), digest)
            finally:
                writer.close()


if __name__ == '__main__':
    unittest.main()
