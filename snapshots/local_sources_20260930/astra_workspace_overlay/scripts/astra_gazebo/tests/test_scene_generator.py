import importlib.util
from pathlib import Path
import random
import unittest
import xml.etree.ElementTree as ET

spec=importlib.util.spec_from_file_location('astra_scene_generator',Path(__file__).parents[1]/'scene_generator.py')
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)


class SceneGeneratorTests(unittest.TestCase):
    def test_six_families_have_distinct_real_geometry_and_split_isolation(self):
        guard=g.SeedGuard();layouts=[]
        for family in g.FAMILIES:
            for index in range(3):
                layout=g.sample_layout(family,index,'development',guard,layouts)
                self.assertEqual(g.group_split(layout['parent_geometry_id']),'development')
                self.assertTrue(g.initial_geometry_qc(layout)['horizontal_initial_geometry_pass'])
                self.assertFalse(any(g.near_duplicate(layout,old) for old in layouts))
                layouts.append(layout)
        self.assertEqual(len({s['canonical_geometry_sha256'] for s in layouts}),18)
        self.assertEqual(len({s['parent_geometry_id'] for s in layouts}),18)

    def test_no_silent_seed_collision_or_historical_seed_reuse(self):
        guard=g.SeedGuard();guard.register(263000001,'first','development')
        with self.assertRaises(ValueError):guard.register(263000001,'other','development')
        with self.assertRaises(ValueError):guard.register(198001,'historical','development')

    def test_actor_parameters_vary_and_actual_sdf_is_not_constant(self):
        guard=g.SeedGuard();layout=g.sample_layout('single_crossing',0,'development',guard,[])
        scenes=[g.episode_scene(layout,i,guard) for i in range(6)]
        speeds=[]
        for scene in scenes:
            sdf=ET.fromstring(g.render(scene));p=sdf.find("world/model[@name='benchmark_actor_1']/plugin")
            speeds.append(float(p.findtext('base_speed')))
            self.assertEqual(float(p.findtext('response_time_margin')),scene['actors'][0]['response_time_margin_s'])
            self.assertEqual(g.group_split(scene['actors'][0]['parameter_cell_id']),'development')
            self.assertEqual(len(scene['expected_models']),len(set(scene['expected_models'])))
            self.assertIsNotNone(sdf.find("world/model[@name='ground_plane']/link/visual"))
            self.assertEqual(sdf.findtext("world/model[@name='astra_evidence_camera']/link/sensor/camera/image/width"),'2560')
        self.assertGreater(len(set(speeds)),1)

    def test_near_duplicate_and_reflection_are_one_geometry_group(self):
        a=g._sample_geometry('static_avoidance','one_block_offset',random.Random(9))
        import copy
        reflected=copy.deepcopy(a)
        for b in reflected['static_boxes']:b['y']=-b['y']
        self.assertEqual(g.parent_id(a),g.parent_id(reflected))
        near=copy.deepcopy(a);near['static_boxes'][0]['x']+=.01
        self.assertTrue(g.near_duplicate(a,near))
        far=copy.deepcopy(a);far['static_boxes'][0]['x']+=.6
        self.assertFalse(g.near_duplicate(a,far))

    def test_initial_rotated_actor_overlap_is_rejected(self):
        scene={'corridor_half_width_m':None,'static_boxes':[],
               'actor_geometry':[{'start':[-5.4,0.],'end':[-4.4,1.],'conflict':[-5.,.4]}]}
        self.assertFalse(g.initial_geometry_qc(scene)['horizontal_initial_geometry_pass'])

    def test_shared_registry_rejects_cross_split_relabelling_and_near_clone(self):
        import copy
        guard=g.SeedGuard();a=g.sample_layout('static_avoidance',0,'development',guard,[])
        b=copy.deepcopy(a);b['split']='validation';b['layout_id']='wrong_validation_clone'
        b['static_boxes'][0]['x']+=.01
        types={x['type'] for x in g.audit_geometry_registry([a,b])}
        self.assertIn('near_duplicate_cross_split',types)
        self.assertIn('split_assignment_mismatch',types)
        valid=g.sample_layout('static_avoidance',0,'validation',guard,[a],cohort='validation_test')
        self.assertEqual(g.audit_geometry_registry([a,valid]),[])


if __name__=='__main__':unittest.main()
