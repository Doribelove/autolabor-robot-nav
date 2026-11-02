"""Environment contract tests only; these do not execute the vendor compiler."""
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools/bev_feature'))
import oe391_environment as env


class EnvironmentTests(unittest.TestCase):
    def setUp(self):
        self.versions = dict(env.EXPECTED, **{'horizon-plugin-pytorch': '3.3.10+cu128.torch280'})
        self.modules = {
            'hbdk4.compiler': SimpleNamespace(March=SimpleNamespace(nash_m='nash-m')),
            'torch': SimpleNamespace(__version__='2.8.0+cu128', version=SimpleNamespace(cuda='12.8'),
                                     cuda=SimpleNamespace(is_available=lambda: True)),
            'horizon_plugin_pytorch': SimpleNamespace(), 'heal': SimpleNamespace()}

    def run_training_check(self, root):
        with patch.object(env.platform, 'machine', return_value='x86_64'), \
                patch.object(env.platform, 'system', return_value='Linux'), \
                patch.object(env.sys, 'version_info', (3, 10, 0)), \
                patch.object(env.importlib.metadata, 'version', side_effect=self.versions.__getitem__), \
                patch.object(env.importlib, 'import_module', side_effect=self.modules.__getitem__):
            return env.runtime_report('training', root)

    def test_official_build_tag_is_accepted_but_wrong_torch_abi_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/'configs/bev_vtv2_resnet50_nuscenes').mkdir(parents=True)
            module = root/'heal/models/task_modules/bev/view_transformer.py'
            module.parent.mkdir(parents=True)
            module.touch()
            result = self.run_training_check(root)
            self.assertTrue(result['ready'], result['errors'])
            self.modules['torch'].__version__ = '2.6.0+cu126'
            result = self.run_training_check(root)
            self.assertFalse(result['ready'])
            self.assertTrue(any('binary build tag' in error for error in result['errors']))

    def test_arm_diagnostic_never_imports_x86_vendor_extensions(self):
        with patch.object(env.platform, 'machine', return_value='aarch64'), \
                patch.object(env.importlib.metadata, 'version', side_effect=self.versions.__getitem__), \
                patch.object(env.importlib, 'import_module') as load:
            result = env.runtime_report('compiler')
            self.assertFalse(result['ready'])
            self.assertFalse(result['runtime_imports_tested'])
            load.assert_not_called()

    def test_missing_heal_source_is_not_satisfied_by_version_metadata(self):
        result = self.run_training_check(None)
        self.assertFalse(result['ready'])
        self.assertTrue(any('HEAL source missing' in error for error in result['errors']))


if __name__ == '__main__':
    unittest.main()
