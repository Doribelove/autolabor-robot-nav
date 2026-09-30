import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

P=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('scene_generator_v2',P/'scene_generator_v2.py')
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
D=g.STAGE/'scenes/development_a02'

class GeneratorV2Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.layouts=json.loads((D/'layouts.json').read_text())
        cls.priors=json.loads((g.STAGE/'scenes/development_a01/layouts.json').read_text())

    def test_collision_allocator_preserves_prior_and_logs_skip(self):
        key=g.v1.canonical_json([g.VERSION,'geometry','fixture']);lo,hi=g.v1.SEED_RANGES['geometry']
        occupied=lo+int(hashlib.sha256(key.encode()).hexdigest()[:16],16)%(hi-lo+1)
        record={'key':'retained_legacy_identity','purpose':'geometry'}
        guard=g.SeedGuard({str(occupied):record});chosen=guard.named('geometry','fixture')
        self.assertNotEqual(chosen,occupied);self.assertEqual(guard.records[str(occupied)],record)
        self.assertEqual(guard.collision_log[0]['occupied_seed'],occupied)
        self.assertEqual(chosen,guard.named('geometry','fixture'));self.assertEqual(len(guard.collision_log),1)
        rebuilt=g.SeedGuard(guard.records);self.assertEqual(chosen,rebuilt.named('geometry','fixture'))

    def test_full_sweep_detects_retained_v1_screen_penetration(self):
        old=next(s for s in self.priors if s['family']=='occluded_emergence')
        report=g.geometry_semantics(old)
        self.assertFalse(report['actor_swept_static_pass'])
        self.assertTrue(any(r['intersection'] for r in report['actor_swept_static_pairs']))

    def test_new_full_actor_sweeps_clear_all_static_objects(self):
        for scene in self.layouts:
            with self.subTest(family=scene['family']):
                report=g.geometry_semantics(scene)
                self.assertTrue(report['actor_swept_static_pass'])
                self.assertGreaterEqual(report['minimum_actor_swept_static_clearance_m'],.05)

    def test_multiray_occlusion_and_reveal_use_substantial_beam_counts(self):
        scene=next(s for s in self.layouts if s['family']=='occluded_emergence');report=g.geometry_semantics(scene)
        initial=report['initial_laser_visibility'];reveal=report['revealed_laser_visibility']
        self.assertGreaterEqual(initial['potential_actor_beams'],5)
        self.assertLessEqual(initial['visible_fraction'],.2)
        self.assertGreaterEqual(reveal['potential_actor_beams'],5)
        self.assertGreaterEqual(reveal['visible_fraction'],.8)
        self.assertGreaterEqual(report['reveal_static_physical_clearance_m'],.33)
        empty=g.laser_visibility((-4.,0.,0.),scene['actor_geometry'][0],[])
        self.assertEqual(empty['visible_fraction'],1.)

    def test_prior_registry_and_seed_history_not_overwritten(self):
        self.assertEqual(g.v1.audit_geometry_registry(self.priors+self.layouts),[])
        for scene in self.layouts:
            self.assertEqual(g.v1.group_split(scene['parent_geometry_id']),'development')
            self.assertFalse(any(g.v1.near_duplicate(scene,p) for p in self.priors))
        old=json.loads((g.STAGE/'scenes/development_a01/generation_report.json').read_text())
        new=json.loads((D/'generation_report.json').read_text())
        for k,v in old['rng_records'].items():self.assertEqual(new['rng_records'][k],v)
        self.assertTrue(set(map(str,new['episode_seeds'])).isdisjoint(old['rng_records']))
        self.assertFalse(new['final_set_generated'])

    def test_camera_and_world_evidence_plugin_match_manifest(self):
        import yaml
        for path in D.glob('*/replicate_*/scene_manifest.yaml'):
            scene=yaml.safe_load(path.read_text());root=ET.parse(path.with_name('world.sdf')).getroot().find('world')
            self.assertEqual(list(map(float,root.find("model[@name='astra_evidence_camera']/pose").text.split())),scene['evidence_camera']['pose_xyz_rpy'])
            plugin=root.find("plugin[@name='astra_evidence']")
            self.assertEqual(plugin.get('filename'),'libastra_evidence_plugin.so')
            self.assertEqual(float(plugin.findtext('state_hz')),50.)
            self.assertEqual(scene['robot_collision_reference']['spawn_xyz_rpy'][2],0.)
            self.assertEqual(scene['physics_evidence']['live_topic_qualification'],'pending')
            for actor in scene['actors']:
                p=root.find("model[@name='%s']/plugin"%actor['model_name'])
                self.assertEqual(float(p.findtext('base_speed')),actor['base_speed_mps'])
                self.assertLessEqual(abs(actor['nominal_arrival_difference_s']),2.)

if __name__=='__main__':unittest.main()
