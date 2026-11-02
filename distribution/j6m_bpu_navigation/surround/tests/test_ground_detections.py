import copy
import json
import math
from pathlib import Path
import struct
import unittest

import cv2
import numpy as np

from surround.ground_bev import GroundBev
from surround.ground_detections import ground_ray, localize_detections, render_ground_detections
from surround.ground_protocol import encode_view_frame, decode_view_frame


class GroundDetectionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = json.loads((Path(__file__).resolve().parents[4]/'config/four_camera_bev.json').read_text())
        cls.bev = GroundBev(cls.config)
        cls.config = cls.bev.config
        cls.meta = dict(frame_id=cls.config['frame_id'],x_range_m=cls.config['x_range_m'],
            y_range_m=cls.config['y_range_m'],metres_per_pixel=cls.bev.resolution,
            observation_bounds_m=cls.config.get('observation_bounds_m', cls.config),
            camera_image_sizes={n:c['perspective_size'] for n,c in cls.config['cameras'].items()},
            projection_intrinsics={n:k.tolist() for n,k in cls.bev.intrinsics.items()},
            T_ground_from_upright_optical={n:t.tolist() for n,t in cls.bev.transforms.items()})

    def box(self,name,x,y,confidence=.8):
        uv,depth=self.bev.project(name,[x,y,0]); self.assertGreater(depth,0)
        u,v=uv
        return dict(class_name='trash',confidence=confidence,bbox=[u-5,v-12,u+5,v])

    def test_contact_ray_matches_independently_measured_rig_axes(self):
        for name,sign in [('left',1),('right',-1)]:
            k=self.meta['projection_intrinsics'][name]
            p=ground_ray(k,self.meta['T_ground_from_upright_optical'][name],[k[0][2],k[1][2]])
            np.testing.assert_allclose(p,[0,sign*(.215+.8/math.tan(math.radians(22)))],atol=1e-10)
        items,rejected=localize_detections(self.meta,{'front':[self.box('front',2.,.3)]})
        self.assertFalse(rejected);np.testing.assert_allclose([items[0]['x'],items[0]['y']],[2.,.3],atol=1e-10)

    def test_five_metres_are_measured_from_each_physical_body_edge(self):
        for name,inside,outside in [
                ('front',(6.21,0),(6.23,0)), ('rear',(-5.51,0),(-5.53,0)),
                ('left',(0,5.39),(0,5.41)), ('right',(0,-5.39),(0,-5.41))]:
            with self.subTest(camera=name):
                points,rejected=localize_detections(self.meta,{name:[self.box(name,*inside)]})
                self.assertEqual(len(points),1);self.assertFalse(rejected)
                points,rejected=localize_detections(self.meta,{name:[self.box(name,*outside)]})
                self.assertFalse(points);self.assertEqual(rejected[0]['reason'],'outside_bev_range')

    def test_square_display_padding_does_not_become_a_cleaning_target(self):
        points,rejected=localize_detections(self.meta,{'left':[self.box('left',0,5.6)]})
        self.assertFalse(points);self.assertEqual(rejected[0]['reason'],'outside_bev_range')
        self.assertEqual((self.bev.width,self.bev.height),(600,600))
        padding=np.abs(self.bev.y)>=5.4
        self.assertTrue(padding.any())
        self.assertFalse(self.bev.support_counts[padding].any())

    def test_editing_one_edge_distance_changes_only_that_observation_limit(self):
        config=copy.deepcopy(self.config)
        config['observation_region']['distance_from_edges_m']['front']=3.
        bev=GroundBev(config)
        self.assertEqual(bev.config['observation_bounds_m'],
                         dict(x_range_m=[-5.52,4.22],y_range_m=[-5.4,5.4]))
        self.assertEqual((bev.width,bev.height),(600,600))

    def test_horizon_clipped_contact_and_outside_range_are_not_ground_targets(self):
        rows=[dict(class_name='trash',confidence=.9,bbox=[600,100,650,200]),
              dict(class_name='trash',confidence=.9,bbox=[600,650,650,720]),self.box('front',8,0)]
        items,rejected=localize_detections(self.meta,{'front':rows})
        self.assertFalse(items)
        self.assertEqual({r['reason'] for r in rejected},
                         {'ray_above_horizon','bottom_contact_clipped','outside_bev_range'})

    def test_overlap_merges_duplicate_but_preserves_close_objects_seen_together(self):
        rows={'front':[self.box('front',3,2.1,.95),self.box('front',3.08,2.1,.9)],
              'left':[self.box('left',3.01,2.1,.8),self.box('left',3.09,2.1,.75)]}
        objects,rejected=localize_detections(self.meta,rows)
        self.assertFalse(rejected);self.assertEqual(len(objects),2)
        self.assertTrue(all(o['source_cameras']==['front','left'] for o in objects))
        self.assertAlmostEqual(objects[0]['x'],3,delta=.02)
        self.assertAlmostEqual(objects[1]['x'],3.08,delta=.02)

    def test_vehicle_mask_rejects_its_own_camera_contact(self):
        meta=copy.deepcopy(self.meta)
        meta['camera_exclusion_polygons']={'front':[[[0,0],[1279,0],[1279,719],[0,719]]]}
        objects,rejected=localize_detections(meta,{'front':[self.box('front',2,0)]})
        self.assertFalse(objects);self.assertEqual(rejected[0]['reason'],'vehicle_image_mask')

    def test_one_trash_with_cross_view_material_disagreement_is_one_target(self):
        front=self.box('front',3,2.1,.95);front['class_name']='plastic';front['class_id']=1
        side=self.box('left',3.01,2.1,.8);side['class_name']='glass';side['class_id']=3
        objects,rejected=localize_detections(self.meta,{'front':[front],'left':[side]})
        self.assertFalse(rejected);self.assertEqual(len(objects),1)
        self.assertEqual(objects[0]['label'],'plastic')
        self.assertEqual(objects[0]['association_count'],2)
        self.assertTrue(objects[0]['class_disagreement'])

    def test_nearby_cross_view_points_outside_each_others_image_boxes_stay_separate(self):
        # Within the old 15 cm radius, but separated in the front image by >20px.
        rows={'front':[self.box('front',3,2.1)],'left':[self.box('left',3,2.22)]}
        objects,rejected=localize_detections(self.meta,rows)
        self.assertFalse(rejected);self.assertEqual(len(objects),2)

    def test_generic_objects_keep_class_identity_when_ground_points_overlap(self):
        front=self.box('front',3,2.1,.95);front.update(class_name='person',class_id=0)
        side=self.box('left',3.01,2.1,.8);side.update(class_name='bottle',class_id=39)
        objects,rejected=localize_detections(self.meta,{'front':[front],'left':[side]},merge_material_classes=False)
        self.assertFalse(rejected);self.assertEqual(len(objects),2)
        self.assertEqual({o['label'] for o in objects},{'person','bottle'})
        front.update(class_name='bottle',class_id=39)
        objects,rejected=localize_detections(self.meta,{'front':[front],'left':[side]},merge_material_classes=False)
        self.assertFalse(rejected);self.assertEqual(len(objects),1)
        self.assertEqual(objects[0]['source_cameras'],['front','left'])

    def test_render_marks_metric_point_without_modifying_inference_image(self):
        objects,_=localize_detections(self.meta,{'front':[self.box('front',2,.3)]})
        image=np.full((600,600,3),50,np.uint8)
        view,markers=render_ground_detections(image,self.meta,objects)
        self.assertTrue((image==50).all())
        u,v=self.bev.pixel(2,.3)
        self.assertEqual(markers[0]['bbox_role'],'ground_point_marker')
        np.testing.assert_array_equal(view[int(round(v)),int(round(u))],[0,245,255])

    def test_atomic_view_packet_preserves_sources_and_rejects_corruption(self):
        meta=dict(self.meta,configured_cameras=list(self.config['cameras']),
            sources={n:dict(sequence=i+1) for i,n in enumerate(self.config['cameras'])},
            produced_epoch_s=1000.,fusion_session='snapshot',fusion_sequence=9,
            protocol='conbine.ground_bev.frame.v1',image_width=600,image_height=600)
        views={n:np.full((sz[1],sz[0],3),i*40,np.uint8) for i,(n,sz) in enumerate(meta['camera_image_sizes'].items())}
        _,jpeg=cv2.imencode('.jpg',np.zeros((600,600,3),np.uint8)); h=json.dumps(meta).encode()
        bg=struct.pack('!4sII',b'BG01',len(h),len(jpeg))+h+jpeg.tobytes()
        packet=encode_view_frame(bg,views)
        decoded,_,raw,images=decode_view_frame(packet,now=1000.1)
        self.assertEqual(decoded['sources'],meta['sources']);self.assertEqual(raw,jpeg.tobytes())
        self.assertEqual(set(images),set(views))
        with self.assertRaises(ValueError):decode_view_frame(packet[:-3],now=1000.1)
        damaged=bytearray(packet);damaged[-20]^=1
        with self.assertRaisesRegex(ValueError,'digest'):decode_view_frame(damaged,now=1000.1)
        with self.assertRaisesRegex(ValueError,'stale'):decode_view_frame(packet,now=1004)


if __name__=='__main__':unittest.main()
