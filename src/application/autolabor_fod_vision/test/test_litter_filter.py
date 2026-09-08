import copy
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np
import yaml
from autolabor_fod_vision.litter_filter import FilterConfig, LitterFilter

sys.path.insert(0, str(Path(__file__).resolve().parent))
from litter_scenes import scene, BOTTLE, PLANT


class LitterFilterTest(unittest.TestCase):
    def run_frames(self, objects, frames=3, **config):
        depth,k,mount,boxes = scene(objects)
        engine = LitterFilter(FilterConfig(budget_ms=100, **config))
        reports = [engine.process(boxes,depth,k,mount,True,10+i*.07) for i in range(frames)]
        return reports, (engine,depth,k,mount,boxes)

    def test_bottle_mislabeled_suitcase_is_kept(self):
        reports,_ = self.run_frames([BOTTLE])
        self.assertEqual(reports[-1]['decisions'][0]['state'], 'candidate')

    def test_horizontal_bottle_and_thin_paper_not_rejected(self):
        for high in ([2.1,.02,.09],[2.1,.08,.01]):
            with self.subTest(high=high):
                reports,_ = self.run_frames([([1.8,-.08,0],high,'paper')])
                self.assertNotEqual(reports[-1]['decisions'][0]['state'], 'environment')

    def test_plant_needs_three_current_observations(self):
        reports,_ = self.run_frames([PLANT])
        self.assertEqual([r['decisions'][0]['state'] for r in reports], ['uncertain','uncertain','environment'])
        self.assertEqual(reports[-1]['decisions'][0]['reason'], 'tall_structure')

    def test_label_flips_do_not_reset_geometric_evidence(self):
        _,(engine,depth,k,mount,boxes) = self.run_frames([PLANT],frames=1)
        for index,name in enumerate(('bottle','suitcase')):
            boxes[0].update(class_id=39 if index==0 else 28,class_name=name,object_id=10+index)
            result = engine.process(boxes,depth,k,mount,True,10.07+index*.07)
        self.assertEqual(result['decisions'][0]['state'],'environment')

    def test_missing_depth_clears_rejection_immediately(self):
        _,(engine,depth,k,mount,boxes) = self.run_frames([PLANT])
        result = engine.process(boxes,None,None,mount,True,10.21)
        self.assertEqual(result['decisions'][0]['state'],'uncertain')
        result = engine.process(boxes,depth,k,mount,True,10.28)
        self.assertEqual(result['decisions'][0]['rejection_streak'],1)

    def test_bad_calibration_does_not_hide_objects(self):
        _,(engine,depth,k,mount,boxes) = self.run_frames([PLANT])
        result = engine.process(boxes,depth,k,mount,False,10.21)
        self.assertEqual(result['decisions'][0]['reason'],'uncalibrated')
        self.assertEqual(result['counts']['environment'],0)

    def test_ground_missing_is_unknown(self):
        _,(engine,depth,k,mount,boxes) = self.run_frames([PLANT])
        depth[:] = np.nan
        result = engine.process(boxes,depth,k,mount,True,10.21)
        self.assertFalse(result['ground']['valid'])
        self.assertEqual(result['counts']['uncertain'],1)

    def test_transparent_depth_holes_are_retained(self):
        depth,k,mount,boxes = scene([BOTTLE])
        x1,y1,x2,y2 = map(int,boxes[0]['bbox'])
        depth[y1:y2,x1:x2] = np.nan
        engine = LitterFilter(FilterConfig(budget_ms=100))
        for i in range(4):
            result = engine.process(boxes,depth,k,mount,True,10+i*.07)
        self.assertEqual(result['decisions'][0]['reason'],'sparse_depth')
        self.assertEqual(result['counts']['environment'],0)
        json.dumps(result,allow_nan=False)

    def test_transparent_object_returning_floor_is_retained(self):
        depth,k,mount,_ = scene()
        _,_,_,boxes = scene([BOTTLE])
        result = LitterFilter().process(boxes,depth,k,mount,True,10)
        self.assertEqual(result['counts']['environment'],0)

    def test_wall_depth_inside_fake_box_is_not_object_geometry(self):
        depth,k,mount,_ = scene()
        boxes = [dict(bbox=[240.,5.,400.,105.],object_id=1,class_id=39,class_name='bottle',confidence=.9)]
        engine = LitterFilter(FilterConfig(budget_ms=100))
        for i in range(4):
            result = engine.process(boxes,depth,k,mount,True,10+i*.07)
        self.assertEqual(result['counts']['environment'],0)

    def test_image_edge_truncation_is_retained(self):
        _,(engine,depth,k,mount,boxes) = self.run_frames([PLANT])
        boxes[0]['bbox'][1] = 0
        result = engine.process(boxes,depth,k,mount,True,10.21)
        self.assertEqual(result['decisions'][0]['reason'],'image_edge')

    def test_distant_gap_or_disappearance_does_not_reuse_rejection(self):
        for empty in (True,False):
            _,(engine,depth,k,mount,boxes) = self.run_frames([PLANT])
            if empty:
                engine.process([],depth,k,mount,True,10.21)
            result = engine.process(boxes,depth,k,mount,True,10.49)
            self.assertEqual(result['decisions'][0]['state'],'uncertain')

    def test_duplicate_or_backward_stamp_rejected(self):
        _,(engine,depth,k,mount,boxes) = self.run_frames([PLANT])
        for stamp in (10.14,10.,float('nan')):
            with self.assertRaises(ValueError):
                engine.process(boxes,depth,k,mount,True,stamp)

    def test_three_burst_frames_do_not_replace_elapsed_confirmation(self):
        depth,k,mount,boxes = scene([PLANT])
        engine = LitterFilter(FilterConfig(budget_ms=100))
        for i in range(3):
            result = engine.process(boxes,depth,k,mount,True,10+i*.01)
        self.assertEqual(result['counts']['environment'],0)

    def test_overlapping_ambiguous_association_cannot_transfer_rejection(self):
        _,(engine,depth,k,mount,boxes) = self.run_frames([PLANT])
        boxes.append(dict(boxes[0],object_id=2))
        result = engine.process(boxes,depth,k,mount,True,10.21)
        self.assertEqual(result['counts']['environment'],0)

    def test_object_limit_and_time_budget_keep_unprocessed_detections(self):
        depth,k,mount,boxes = scene([PLANT, BOTTLE])
        engine = LitterFilter(FilterConfig(budget_ms=100,max_geometry_objects=1))
        result = engine.process(boxes,depth,k,mount,True,10)
        self.assertEqual(result['decisions'][1]['reason'],'geometry_budget')
        with patch('autolabor_fod_vision.litter_filter.time.monotonic',side_effect=[0,1,2]):
            result = LitterFilter().process(boxes,depth,k,mount,True,10)
        self.assertEqual(result['counts']['uncertain'],2)

    def test_raw_detections_and_depth_not_mutated(self):
        depth,k,mount,boxes = scene([PLANT])
        original_depth = depth.copy();original_boxes = copy.deepcopy(boxes)
        LitterFilter().process(boxes,depth,k,mount,True,10)
        np.testing.assert_array_equal(depth,original_depth)
        self.assertEqual(boxes,original_boxes)

    def test_disabled_filter_keeps_all_boxes(self):
        reports,_ = self.run_frames([PLANT,BOTTLE],enabled=False)
        self.assertEqual(reports[-1]['counts']['uncertain'],2)
        self.assertEqual(reports[-1]['counts']['environment'],0)

    def test_noise_and_resolution_do_not_change_basic_outcome(self):
        for width,height in ((640,360),(1280,720)):
            depth,k,mount,boxes = scene([PLANT,BOTTLE],width,height)
            depth += np.random.RandomState(1).normal(0,.002,depth.shape).astype(np.float32)
            engine = LitterFilter(FilterConfig(budget_ms=100))
            for i in range(3):
                report = engine.process(boxes,depth,k,mount,True,10+i*.07)
            self.assertEqual([d['state'] for d in report['decisions']],['environment','candidate'])

    def test_configuration_validated_and_pinned(self):
        config = yaml.safe_load((Path(__file__).resolve().parents[1]/'config/litter_filter.yaml').read_text())
        self.assertEqual(FilterConfig(**config).digest,FilterConfig().digest)
        for values in ({'budget_ms':float('nan')},{'confirm_frames':1},{'enabled':'false'},
                       {'association_iou':.1},{'candidate_max_height_m':.9}):
            with self.assertRaises(ValueError):FilterConfig(**values)


if __name__ == '__main__':
    unittest.main()
