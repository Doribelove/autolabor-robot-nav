import ast
import copy
import json
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import prepare_navigation_n06_static as p
import navigation_evidence_n06 as evidence
from navigation_coordinator_n06 import CoordinatorConfig,NavigationCoordinator

class ExactPreparationTests(unittest.TestCase):
    def test_fixed_counterbalanced_four_pairs_unallocated(self):
        rows=p.schedule();self.assertEqual(len(rows),8)
        self.assertEqual(len({r['run_id'] for r in rows}),8)
        for block in range(4):
            pair=rows[2*block:2*block+2]
            self.assertEqual({r['seed'] for r in pair},{263003001+block})
            self.assertEqual([r['arm'] for r in pair],['control','treatment'] if block%2==0 else ['treatment','control'])
            self.assertEqual({r['method'] for r in pair},set(p.METHODS))
            self.assertTrue(all(r['seed_status']=='proposal_unallocated' and r['attempt']==1 for r in pair))
    def test_frozen_design(self):self.assertIsNotNone(p.validate_design(p.read(p.DESIGN)))
    def test_design_rejects_factor_or_identity_drift(self):
        base=p.read(p.DESIGN)
        for key,value in [('execution_authorized',True),('allocated_budget',8),('methods',list(reversed(p.METHODS)))]:
            altered=copy.deepcopy(base);altered[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):p.validate_design(altered)
        for key,value in [('treatment_m',1.3),('control_m',.8),('N05_preview_eligibility_used',True)]:
            altered=copy.deepcopy(base);altered['single_behavioral_factor'][key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):p.validate_design(altered)
    def test_only_lookahead_and_identity_source_difference(self):
        r=p.treatment_equivalence();self.assertEqual(r['configuration_delta'],{'lookahead_m':[.7,1.2]})
        self.assertFalse(r['N05_preview_selector_used'])
    def test_closed_method_whitelists_reject_preview_and_pct(self):
        self.assertEqual(evidence.METHODS,p.METHODS)
        for method in ('astra_analytic_preview_eligible_n05','astra_pct_route_v1','actual_teb_common_route'):
            with self.subTest(method=method),self.assertRaises(ValueError):CoordinatorConfig('fixture',263003001,method)
    def test_parent_manifest_exact_freeze_and_fixed_static_sentinel(self):
        parents=p.verify_d04();self.assertEqual(len(parents),4)
        self.assertTrue(all(s['actor_parameter_cell_id']=='actor_none_v1' and not s['actors'] for s in parents))
    def test_world_preserves_every_native_collision(self):
        for parent in p.verify_d04():
            folder=p.D04/parent['layout_id'];xml=p.world_from_parent(folder)
            self.assertEqual(p.collision_signature(xml),p.collision_signature((folder/'raster_and_guide/static_geometry.sdf').read_text()))
            root=ET.fromstring(xml)
            self.assertEqual(root.findtext('world/physics/max_step_size'),'.002')
            self.assertFalse(root.findall('world/model/plugin'))
    def test_single_command_authority_and_collector_required(self):
        root=ET.parse(p.LAUNCH).getroot();nodes=root.findall('node')
        collector=next(n for n in nodes if n.get('name')=='astra_navigation_collector')
        self.assertEqual(collector.get('required'),'true')
        self.assertEqual(next(n for n in nodes if n.get('name')=='astra_navigation_coordinator').get('type'),'astra_navigation_coordinator_n06.py')
        for group in root.findall('group'):
            self.assertEqual(group.find('node/remap').attrib,{'from':'/cmd_vel_navigation','to':'/astra/method_cmd'})
    def test_no_execution_or_seed_registry_code_in_preparer(self):
        tree=ast.parse(Path(p.__file__).read_text())
        names=[ast.dump(n.func) for n in ast.walk(tree) if isinstance(n,ast.Call)]
        self.assertFalse(any(x in n for n in names for x in ('Popen','roslaunch','SeedGuard','register_seed','unpause')))
    def test_shared_coordinator_gate_identity_both_methods(self):
        for method in p.METHODS:
            core=NavigationCoordinator(CoordinatorConfig('fixture',263003001,method))
            core.method_status({'run_id':'fixture','seed':263003001,'method':method,'state':'bootstrap','ready':True})
            core.clock(4_000_000_000);core.observe_odom(4_000_000_000,(-5.4,0,0),(0,0,0));core.arm(4_000_000_000)
            core.clock(5_000_000_000);core.observe_odom(5_000_000_000,(-5.4,0,0),(0,0,0));core.output(5_000_000_000)
            core.clock(7_000_000_000);gate=core.observe_odom(7_000_000_000,(-4.,0.,0.),(.8,0,0))
            self.assertEqual(gate['method'],method);self.assertEqual(gate['source_ns'],7_000_000_000)
    def test_prepared_paired_exact_geometry_and_physics(self):
        data=p.read(p.PREPARED/'prepared.json');self.assertEqual(len(data['units']),8)
        for i in range(0,8,2):
            a,b=data['units'][i:i+2]
            self.assertEqual(a['world']['sha256'],b['world']['sha256'])
            self.assertEqual(a['robot_description']['sha256'],b['robot_description']['sha256'])
            ca,cb=p.read(a['contract']['path']),p.read(b['contract']['path'])
            self.assertEqual(ca['shared_bundle'],cb['shared_bundle'])
            self.assertEqual(ca['scene_manifest'],cb['scene_manifest'])
            for c in (ca,cb):self.assertEqual(evidence.validate_contract(c),c)
            for key in ('run_id','method','world','robot_description'):ca.pop(key);cb.pop(key)
            self.assertEqual(ca,cb)
    def test_prepared_has_no_actual_gate_source_or_execution_authority(self):
        for unit in p.read(p.PREPARED/'prepared.json')['units']:
            gate=p.read(unit['gate_expectation']['path'])
            self.assertFalse(gate['execution_authorized']);self.assertFalse(gate['current_source_timestamp_available'])
            self.assertFalse(gate['N05_preview_selector_used'])
            self.assertEqual(gate['lookahead_m'],unit['lookahead_m'])
    def test_map_and_qualified_raster_witness_copied_exactly(self):
        for block in p.read(p.PREPARED/'prepared.json')['blocks']:
            source=p.D04/block['layout_id']/'raster_and_guide'
            prepared=Path(block['manifest']['path']).parent
            for name in ('map.pgm','map.yaml','common_route.json'):
                self.assertEqual((source/'common_bundle'/name).read_bytes(),(prepared/'common_bundle'/name).read_bytes())
            self.assertEqual((source/'raster_witness.csv').read_bytes(),(prepared/'d04_raster_and_guide/raster_witness.csv').read_bytes())
    def test_world_shell_does_not_change_actual_exported_raster(self):
        from static_map_export import make_artifacts
        import yaml
        for block in p.read(p.PREPARED/'prepared.json')['blocks']:
            scene=yaml.safe_load(Path(block['manifest']['path']).read_text())
            pgm,meta,request,audit=make_artifacts(Path(block['world']['path']).read_text(),scene)
            folder=Path(block['bundle']['path']).parent
            self.assertEqual(pgm,(folder/'map.pgm').read_bytes())
            self.assertEqual(meta,(folder/'map.yaml').read_bytes())
            self.assertEqual(request['map_from_odom_xy_yaw'],[-5.4,0.,0.])
    def test_preparation_cannot_overwrite_previous_inputs(self):
        with self.assertRaises(FileExistsError):p.prepare()

if __name__=='__main__':unittest.main()
