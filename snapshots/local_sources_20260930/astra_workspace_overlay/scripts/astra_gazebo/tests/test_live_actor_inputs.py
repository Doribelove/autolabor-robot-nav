import csv,hashlib,importlib.util,json,math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import yaml
P=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('derive_live_actor_inputs',P/'derive_live_actor_inputs.py')
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
D=g.STAGE/'scenes/development_a02_live_v1'

class LiveActorInputTests(unittest.TestCase):
    def test_all_derived_actors_bind_delay_route_and_original_reaction_parameters(self):
        rows=list(csv.DictReader((D/'diagnostic_blocks.csv').read_text().splitlines()));self.assertEqual(len(rows),12)
        actors=0
        for row in rows:
            scene=yaml.safe_load(Path(row['manifest_path']).read_text());world=ET.parse(row['world_path']).getroot().find('world')
            self.assertEqual(g.sha(row['manifest_path']),row['manifest_sha256']);self.assertEqual(g.sha(row['world_path']),row['world_sha256'])
            self.assertEqual(scene['input_derivation']['script_sha256'],g.sha(P/'derive_live_actor_inputs.py'))
            for a in scene['actors']:
                actors+=1;p=world.find("model[@name='%s']/plugin"%a['model_name'])
                self.assertEqual(p.get('filename'),'libastra_reactive_actor_plugin.so')
                self.assertIsNone(p.find('start_delay'));self.assertIsNone(p.find('activation_ego_x'))
                self.assertEqual(float(p.findtext('gate_delay')),a['gate_delay_s'])
                self.assertEqual(float(p.findtext('gate_route_x')),-4.02)
                self.assertEqual(p.findtext('route_origin'),'0 0');self.assertEqual(float(p.findtext('route_yaw')),0.)
                for k in ('start','end','conflict'):self.assertEqual(list(map(float,p.findtext(k).split())),a[k])
                for k,f in [('base_speed','base_speed_mps'),('response_distance','response_distance_m'),('response_time_margin','response_time_margin_s')]:self.assertEqual(float(p.findtext(k)),a[f])
                self.assertFalse(a['response_seed_behavioral_effect'])
            self.assertFalse(scene['readiness']['gate_relative_actor_delay_live_qualified'])
            self.assertFalse(scene['randomization_scope']['unused_reserved_streams_block_execution'])
        self.assertEqual(actors,10)

    def test_parent_evidence_and_geometry_seed_identities_are_preserved(self):
        report=json.loads((D/'derivation_report.json').read_text())
        self.assertEqual(report['new_independent_geometries'],0)
        for path,sha in report['parent_bindings'].items():self.assertEqual(g.sha(path),sha)
        for p in D.glob('*/replicate_*/scene_manifest.yaml'):
            new=yaml.safe_load(p.read_text());old=yaml.safe_load(Path(new['input_derivation']['parent_manifest_path']).read_text())
            for k in ('seed','rng_substreams','canonical_geometry_sha256','parent_geometry_id','static_boxes','route'):self.assertEqual(new[k],old[k])
            for a,b in zip(new['actors'],old['actors']):
                for k in ('policy','base_speed_mps','response_distance_m','response_time_margin_s','gate_delay_s','response_seed','start','end','conflict'):self.assertEqual(a[k],b[k])

    def test_actor_path_se2_helper_composes_and_inverse_recovers_geometry(self):
        p=next(D.glob('*single_crossing*/replicate_*/scene_manifest.yaml'));a=yaml.safe_load(p.read_text())['actors'][0]
        f=g.RouteFrame(2.3,-1.7,1.17);b=g.transformed_actor(a,f)
        for k in ('start','end','conflict'):
            recovered=f.point_to_route(b[k])
            for x,y in zip(recovered,a[k]):self.assertAlmostEqual(x,y,12)
        self.assertAlmostEqual(math.dist(b['start'],b['end']),math.dist(a['start'],a['end']),12)
        yaw=math.atan2(b['end'][1]-b['start'][1],b['end'][0]-b['start'][0])
        self.assertAlmostEqual(math.sin(b['initial_pose_xyz_rpy'][5]-yaw),0.,12)
        second=g.RouteFrame(-.8,4.2,-.63)
        composed=g.RouteFrame(*second.point_to_world((f.translation_x_m,f.translation_y_m)),f.yaw_rad+second.yaw_rad)
        twice=g.transformed_actor(b,second);once=g.transformed_actor(a,composed)
        for k in ('start','end','conflict'):
            for x,y in zip(twice[k],once[k]):self.assertAlmostEqual(x,y,12)

    def test_nonidentity_full_scene_fails_until_planner_bridge_exists(self):
        p=next((g.STAGE/'scenes/development_a02').glob('*/replicate_*/scene_manifest.yaml'))
        with self.assertRaisesRegex(ValueError,'planner/map/evaluator'):
            g.derive(yaml.safe_load(p.read_text()),p.with_name('world.sdf').read_text(),g.RouteFrame(1.,0.,.5))

    def test_mapping_predeclares_real_and_reserved_streams(self):
        mapping=json.loads(g.MAPPING.read_text())
        self.assertTrue(mapping['rng_fields']['rng_substreams.actor_parameters']['behavioral_variation'])
        for name in ('rng_substreams.actor_response','rng_substreams.sensor_noise','rng_substreams.actuator','actor.response_seed'):
            self.assertFalse(mapping['rng_fields'][name]['behavioral_variation'])
            self.assertFalse(mapping['rng_fields'][name]['blocking'])

if __name__=='__main__':unittest.main()
