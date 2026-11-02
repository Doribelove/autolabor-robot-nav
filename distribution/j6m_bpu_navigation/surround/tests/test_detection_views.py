import copy
import json
from pathlib import Path
import struct
import unittest

import cv2
import numpy as np
from surround.detection_views import DetectionMasks, encode_vision_frame


class DetectionViewsTests(unittest.TestCase):
    def setUp(self):
        self.mask=DetectionMasks(Path(__file__).resolve().parents[4]/'config/bev_detection_masks.json')

    def test_vehicle_input_is_masked_and_adjacent_ground_is_preserved(self):
        rear=np.full((540,960,3),200,np.uint8)
        front=np.full((720,1280,3),80,np.uint8)
        images=self.mask.apply(dict(rear=rear,front=front))
        self.assertTrue((rear==200).all())
        self.assertTrue((images['rear'][490,480]==0).all())
        self.assertTrue((images['rear'][490,640]==200).all())
        self.assertTrue((images['rear'][400,480]==200).all())
        self.assertIs(images['front'],front)
        with self.assertRaisesRegex(ValueError,'resolution'):
            self.mask.apply({'rear':np.zeros((270,480,3),np.uint8)})

    def test_real_vehicle_boxes_excluded_but_neighbouring_bottles_kept(self):
        def box(label,coords):return dict(class_name=label,class_id=39,confidence=.8,bbox=coords)
        rows={'rear':[box('toilet',[354,420,614,538]),box('sink',[355,419,613,538]),
                      box('bottle',[625,450,650,530]),box('bottle',[475,350,490,399]),
                      box('bottle',[595,430,625,490])],
              'left':[box('bottle',[579,315,599,360])]}
        kept,excluded=self.mask.filter(rows)
        self.assertEqual([o['class_name'] for o in kept['rear']],['bottle']*3)
        self.assertEqual(len(kept['left']),1)
        self.assertEqual(len(excluded),2)
        self.assertEqual(len(rows['rear']),5)

    def test_preview_envelope_contains_exact_group_and_image_bytes(self):
        previews={n:np.full((540,960,3),i*60,np.uint8) for i,n in enumerate(['front','left','right','rear'])}
        ok,jpeg=cv2.imencode('.jpg',np.zeros((600,600,3),np.uint8));self.assertTrue(ok)
        meta=dict(source_id=dict(session='capture',sequence=25))
        packet=encode_vision_frame(meta,jpeg.tobytes(),jpeg.tobytes(),previews)
        magic,n,r,a=struct.unpack('!4sIII',packet[:16]);self.assertEqual(magic,b'BV02')
        header=json.loads(packet[16:16+n]);offset=16+n+r+a
        self.assertNotIn('camera_previews',meta)
        import hashlib
        for row in header['camera_previews']:
            self.assertEqual(row['source_id'],meta['source_id'])
            image=packet[offset:offset+row['bytes']];offset+=row['bytes']
            self.assertEqual(hashlib.sha256(image).hexdigest(),row['sha256'])
            decoded=cv2.imdecode(np.frombuffer(image,np.uint8),cv2.IMREAD_COLOR)
            self.assertEqual(decoded.shape,previews[row['camera']].shape)
        self.assertEqual(offset,len(packet))


if __name__=='__main__':unittest.main()
