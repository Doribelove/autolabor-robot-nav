import unittest
import numpy as np
from surround.fcos_image import prepare_rgb, source_boxes


class FcosImageTests(unittest.TestCase):
    def test_widescreen_rgb_keeps_aspect_and_nv12_black_padding(self):
        payload, geometry = prepare_rgb(np.full((720,1280,3),255,np.uint8))
        self.assertEqual(len(payload),896*896*3//2)
        y=np.frombuffer(payload,np.uint8,count=896*896).reshape(896,896)
        self.assertTrue((y[:504]==235).all())
        self.assertTrue((y[504:]==16).all())
        self.assertTrue((np.frombuffer(payload,np.uint8,offset=896*896)==128).all())
        box=source_boxes([dict(class_id=39,confidence=.8,bbox_model_px=[70,140,210,280])],geometry)[0]
        np.testing.assert_allclose(box[:4],[100,200,300,400])

    def test_rounded_resize_restores_contacts_and_discards_padding_boxes(self):
        _, geometry=prepare_rgb(np.zeros((721,1281,3),np.uint8))
        expected=np.array([100,200,500,721.])
        scaled=expected*np.array([geometry['scale_x'],geometry['scale_y']]*2)
        boxes=source_boxes([dict(class_id=41,confidence=.7,bbox_model_px=scaled),
            dict(class_id=0,confidence=.9,bbox_model_px=[50,700,100,850])],geometry)
        self.assertEqual(len(boxes),1)
        np.testing.assert_allclose(boxes[0][:4],expected)
        self.assertEqual(boxes[0][-1],41)


if __name__=='__main__':unittest.main()
