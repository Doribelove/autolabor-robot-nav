import sys,unittest,json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'src/application/autolabor_fod_vision/src'))
from autolabor_fod_vision.bev_feature.geometry import sampling,DEFAULT,decode,letterbox_k
from autolabor_fod_vision.bev_feature.data import validate,targets
class TestGeometry(unittest.TestCase):
 def setUp(self):self.rig=json.loads((ROOT/'config/four_camera_bev.json').read_text())
 def test_crop_pixel_center_intrinsics(self):
  k=np.array([[500,0,639.5],[0,500,359.5],[0,0,1.]])
  result,box=letterbox_k(k,[1280,720],[256,448]);np.testing.assert_allclose(result[:2,2],[223.5,127.5])
 def test_base_translation_changes_sampling_without_guessing(self):
  g=sampling(self.rig);m=sampling(self.rig,rig_to_base=[.2,0,0]);self.assertEqual(g['feature_grid'].shape,(4,480,120,2));self.assertFalse(np.array_equal(g['feature_grid'],m['feature_grid']))
  for i in range(4):self.assertGreater(g['valid'][i].sum(),0)
 def test_labels_decode_metric_and_global_dedup(self):
  objects=[dict(instance_id='a',center_base_m=[1.233,.467])];heat,off,mask=targets(objects,DEFAULT);got=decode(heat,off)
  self.assertEqual(len(got),1);self.assertAlmostEqual(got[0]['x'],1.233,places=5);self.assertAlmostEqual(got[0]['y'],.467,places=5)
 def test_capture_not_trainable_by_receipt_stamp(self):
  s=dict(camera_order=['front','left','right','rear'],clock_domain='mixed_unverified',clock_verified=False)
  with self.assertRaisesRegex(ValueError,'capture clocks'):validate(s,ROOT)
 def test_two_close_objects_not_merged_outside_radius(self):
  h=np.zeros((1,120,120),np.float32);o=np.full((2,120,120),.5,np.float32);h[0,60,60]=.9;h[0,60,64]=.8
  self.assertEqual(len(decode(h,o,radius=.12)),2)
if __name__=='__main__':unittest.main()
