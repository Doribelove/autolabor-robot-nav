import copy
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import production_ptq as ptq


class PtqBoundaries(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.spec = dict(ptq.contract()['models']['classifier'], input_shape=[1, 3, 2, 2],
                         minimum_calibration_samples=5)

    def tearDown(self):
        self.temp.cleanup()

    def manifest(self):
        records = []
        for index, name in enumerate(self.spec['classes']):
            path = self.root / ('%d.npy' % index)
            np.save(path, np.full(self.spec['input_shape'], index / 10, np.float32))
            records.append(dict(path=path.name, sha256=ptq.digest(path), class_name=name))
        data = dict(model='classifier', split='train', preprocess_id=self.spec['preprocess_id'],
                    samples=records)
        path = self.root / 'manifest.json'
        path.write_text(json.dumps(data))
        return path, data

    def test_source_paths_cannot_escape_or_follow_symlink(self):
        (self.root / 'link.npy').symlink_to('/etc/passwd')
        for relative in ('../escape.npy', '/tmp/escape.npy', 'link.npy'):
            with self.assertRaises(ValueError):
                ptq.inside(self.root, relative)

    def test_train_split_and_all_five_classes_are_required(self):
        path, data = self.manifest()
        _, samples = ptq.calibration_records(path, self.spec, 'classifier')
        self.assertEqual(len(samples), 5)
        for key, value in (('split', 'val'), ('preprocess_id', 'wrong-transform')):
            modified = dict(data, **{key: value})
            path.write_text(json.dumps(modified))
            with self.assertRaises(ValueError):
                ptq.calibration_records(path, self.spec, 'classifier')
        data['samples'][-1]['class_name'] = 'metal'
        path.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, 'every material'):
            ptq.calibration_records(path, self.spec, 'classifier')

    def test_changed_or_duplicate_tensor_rejected(self):
        path, data = self.manifest()
        data['samples'][0]['sha256'] = '0' * 64
        path.write_text(json.dumps(data))
        with self.assertRaises(ValueError):
            ptq.calibration_records(path, self.spec, 'classifier')
        path, data = self.manifest()
        data['samples'][-1] = data['samples'][0]
        path.write_text(json.dumps(data))
        with self.assertRaises(ValueError):
            ptq.calibration_records(path, self.spec, 'classifier')

    def test_wrong_shape_dtype_nan_range_or_pickle_rejected(self):
        for tensor in (np.zeros((2, 2), np.float32),
                       np.zeros(self.spec['input_shape'], np.float64),
                       np.full(self.spec['input_shape'], np.nan, np.float32),
                       np.full(self.spec['input_shape'], 255, np.float32),
                       np.array([{'object': True}], dtype=object)):
            path = self.root / 'bad.npy'
            np.save(path, tensor)
            with self.assertRaises(ValueError):
                ptq.validate_tensor(path, self.spec)

    def test_non_x86_never_invokes_compiler_or_writes(self):
        with patch.object(ptq.platform, 'machine', return_value='aarch64'), \
                patch.object(ptq.subprocess, 'run') as execute:
            with self.assertRaisesRegex(RuntimeError, 'x86_64'):
                ptq.run(self.root, self.root / 'missing_lock.json')
            execute.assert_not_called()
        self.assertEqual(list(self.root.iterdir()), [])

    def workspace(self):
        registry = ptq.contract()
        registry = copy.deepcopy(registry)
        registry['models'] = {'classifier': self.spec}
        model = self.root / 'models/classifier.onnx'
        model.parent.mkdir()
        model.write_bytes(b'private model fixture')
        registry['models']['classifier']['onnx_sha256'] = ptq.digest(model)
        (self.root / 'contract.json').write_text(json.dumps(registry))
        job = self.root / 'jobs/classifier'
        job.mkdir(parents=True)
        (job / 'compile.yaml').write_text(json.dumps(ptq.compile_config('classifier', self.spec)))
        return registry, job

    def test_config_mutation_and_missing_calibration_rejected(self):
        registry, job = self.workspace()
        with patch.object(ptq, 'contract', return_value=registry):
            ptq.verify_workspace(self.root)
            with self.assertRaises(FileNotFoundError):
                ptq.verify_workspace(self.root, require_calibration=True)
            changed = ptq.compile_config('classifier', self.spec)
            changed['model_parameters']['march'] = 'nash-p'
            (job / 'compile.yaml').write_text(json.dumps(changed))
            with self.assertRaisesRegex(ValueError, 'configuration changed'):
                ptq.verify_workspace(self.root)

    def test_existing_output_is_preserved_before_reading_exports(self):
        output = self.root / 'existing'
        output.mkdir()
        (output / 'keep').write_text('original')
        with self.assertRaises(FileExistsError):
            ptq.prepare(self.root / 'missing', output)
        self.assertEqual((output / 'keep').read_text(), 'original')

    def compiled_workspace(self):
        manifest, _ = self.manifest()
        registry, job = self.workspace()
        (self.root / 'calibration/classifier').mkdir(parents=True)
        with patch.object(ptq, 'contract', return_value=registry):
            ptq.attach_calibration(self.root, 'classifier', manifest)
        lock = self.root / 'sdk_lock.json'
        lock.write_text(json.dumps({'environment_id': 'test-fixture-only',
                                   'packages': {'horizon-tc-ui': 'test', 'hbdk4': 'test'}}))
        return registry, job, lock

    def test_compiler_failure_retains_log_and_does_not_claim_acceptance(self):
        registry, job, lock = self.compiled_workspace()
        with patch.object(ptq, 'contract', return_value=registry), \
                patch.object(ptq.platform, 'machine', return_value='x86_64'), \
                patch.object(ptq.importlib.metadata, 'version', return_value='test'), \
                patch.object(ptq.shutil, 'which', return_value='/fake/hb_compile'), \
                patch.object(ptq.subprocess, 'run', return_value=SimpleNamespace(returncode=2)):
            with self.assertRaisesRegex(RuntimeError, 'Compilation failed'):
                ptq.run(self.root, lock)
        report = json.loads((job / 'compile_result.json').read_text())
        self.assertEqual(report['returncode'], 2)
        self.assertIs(report['production_ready'], False)
        self.assertTrue((job / 'compile.log').exists())
        self.assertEqual(len(list((job / 'calibration_inputs').iterdir())), 5)
        self.assertFalse((job / 'calibration_inputs/manifest.json').exists())

    def test_sdk_version_mismatch_prevents_compiler_and_preserves_workspace(self):
        registry, job, lock = self.compiled_workspace()
        with patch.object(ptq, 'contract', return_value=registry), \
                patch.object(ptq.platform, 'machine', return_value='x86_64'), \
                patch.object(ptq.importlib.metadata, 'version', return_value='wrong'), \
                patch.object(ptq.subprocess, 'run') as execute:
            with self.assertRaisesRegex(ValueError, 'version mismatch'):
                ptq.run(self.root, lock)
            execute.assert_not_called()
        self.assertFalse((job / 'compile.log').exists())
        self.assertFalse((self.root / 'TOOLCHAIN_LOCK.json').exists())


if __name__ == '__main__':
    unittest.main()
