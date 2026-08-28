#!/usr/bin/env python3

import os
from pathlib import Path
import tempfile
import unittest

from autolabor_fod_vision.detector import resolve_ultralytics_layout


class UltralyticsRuntimeLayoutTest(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.workspace = Path(self.temporary_directory.name).resolve()
        self.import_root = self.workspace / "custom_runtime"
        self.package_root = self.import_root / "ultralytics"
        self.package_root.mkdir(parents=True)
        (self.package_root / "__init__.py").write_text(
            "__version__ = 'test'\n", encoding="utf-8"
        )

    def tearDown(self):
        self.temporary_directory.cleanup()

    def test_accepts_import_root(self):
        import_root, package_root = resolve_ultralytics_layout(
            str(self.import_root)
        )
        self.assertEqual(import_root, self.import_root)
        self.assertEqual(package_root, self.package_root)

    def test_accepts_package_root(self):
        import_root, package_root = resolve_ultralytics_layout(
            str(self.package_root)
        )
        self.assertEqual(import_root, self.import_root)
        self.assertEqual(package_root, self.package_root)

    def test_workspace_relative_root(self):
        previous = os.environ.get("DUAL_HOST_WS")
        os.environ["DUAL_HOST_WS"] = str(self.workspace)
        try:
            import_root, package_root = resolve_ultralytics_layout(
                "custom_runtime"
            )
        finally:
            if previous is None:
                os.environ.pop("DUAL_HOST_WS", None)
            else:
                os.environ["DUAL_HOST_WS"] = previous
        self.assertEqual(import_root, self.import_root)
        self.assertEqual(package_root, self.package_root)

    def test_rejects_missing_package(self):
        with self.assertRaises(FileNotFoundError):
            resolve_ultralytics_layout(str(self.workspace / "missing"))


if __name__ == "__main__":
    unittest.main()
