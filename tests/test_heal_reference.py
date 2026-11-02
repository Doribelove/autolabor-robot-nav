"""Coordinate/ABI checks against the supplied official pure geometry methods."""
import json
from pathlib import Path
import sys
import unittest

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools/bev_feature'))
from heal_reference import HealCPU, quantize_grids


class HealReferenceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(2)
        cls.heal = HealCPU(ROOT/'runtime/heal_source_20261101/Heal-main')
        run = ROOT/'validation/heal_bpu_real_20261101'
        cls.meta = json.loads((run/'geometry.json').read_text())['source_metadata']
        cls.specs = json.loads((run/'model_metadata.json').read_text())['inputs'][-2:]
        cls.grids, cls.stats, _ = cls.heal.grids(cls.meta)

    def test_paired_sample_indices_survive_s16_quantization(self):
        y, x = np.indices((128, 128)); cells = np.stack([x, y], axis=-1)
        feat, dep = [p+cells for p in self.grids]
        valid = (feat[..., 0] >= 0) & (feat[..., 0] < 44) & (feat[..., 1] >= 0) & (feat[..., 1] < 96)
        # Each feature sample and depth sample must refer to the same camera/pixel.
        np.testing.assert_array_equal((feat[..., 1]//16)[valid], (dep[..., 1]//60)[valid])
        np.testing.assert_array_equal(((feat[..., 1]%16)*44+feat[..., 0])[valid], dep[..., 0][valid])
        self.assertTrue(all(v > 0 for v in self.stats['selected_points_per_camera'].values()))
        self.assertTrue(np.all((feat[..., 1]//16)[valid] < 4))
        quantized = quantize_grids(self.grids, self.specs)
        for p, q, spec in zip(self.grids, quantized, self.specs):
            np.testing.assert_array_equal(p[valid], (q.astype(float)*spec['scales'][0])[valid])
        # Invalid points must stay outside the actual image after saturation.
        qfeat = quantized[0].astype(float)*self.specs[0]['scales'][0]+cells
        qdep = quantized[1].astype(float)*self.specs[1]['scales'][0]+cells
        self.assertTrue(np.all(qfeat[..., 0][~valid] > 44))
        self.assertTrue(np.all(qdep[..., 0][~valid] > 704))

    def test_decoder_axis_units_and_offsets(self):
        raw = [np.zeros((1,c,128,128), np.float32) for c in (2,1,3,2,2,10)]
        raw[-1].fill(-30)
        raw[-1][0,9,64,64] = np.log(9)
        raw[0][0,:,64,64] = [.125, -.25]
        rows, _ = self.heal.decode(raw)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['class_name'], 'traffic_cone')
        self.assertAlmostEqual(rows[0]['x_ground'], .1, places=5)
        self.assertAlmostEqual(rows[0]['y_ground'], -.2, places=5)
        self.assertAlmostEqual(rows[0]['score'], .9, places=6)

    def test_nominal_ground_points_face_each_camera(self):
        points = dict(front=[2,0,0,1], left=[0,2,0,1], right=[0,-2,0,1], rear=[-2,0,0,1])
        for name, point in points.items():
            optical = np.linalg.solve(self.meta['T_ground_from_upright_optical'][name], point)
            self.assertGreater(optical[2], 0)
            uvw = np.array(self.meta['projection_intrinsics'][name]) @ optical[:3]
            u, v = uvw[:2]/uvw[2]
            width, height = self.meta['camera_image_sizes'][name]
            self.assertLess(abs(u-width/2), 10)
            self.assertGreater(v*396/height-140, 0)
            self.assertLess(v*396/height-140, 256)


if __name__ == '__main__':
    unittest.main()
