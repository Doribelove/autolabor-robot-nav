import copy,json,math,pathlib,sys,unittest
P=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(P))
import baseline_static_qualification_b05 as b
S=P.parents[2];A=S/'audits/baseline_integration_b04/launch_preflight_final_r2'

class FullParameters(unittest.TestCase):
 def test_actual_two_prepared_namespaces(self):
  for method in b.METHODS:
   p=json.loads((A/method/'effective_move_base.json').read_text());r=b.compare_full_parameters(p,copy.deepcopy(p));self.assertTrue(r['pass']);self.assertEqual(r['prepared_typed_sha256'],r['observed_typed_sha256'])
 def test_actual_B04_velocity_counterexample_rejected(self):
  import baseline_rear_evidence_b04 as old
  for method in b.METHODS:
   p=json.loads((A/method/'effective_move_base.json').read_text());a=copy.deepcopy(p);k='TebLocalPlannerROS' if method==b.METHODS[0] else 'DWAPlannerROS';a[k]['max_vel_x']=17.
   self.assertTrue(old.planner_parameters(a,method)['pass_']);r=b.compare_full_parameters(p,a);self.assertFalse(r['pass']);self.assertEqual(r['mismatches'],[{'path':'/move_base/'+k+'/max_vel_x','kind':'value'}])
 def test_missing_extra_and_empty_struct_not_ignored(self):
  for p,a in [({'a':1},{}),({'a':1},{'a':1,'default_added':2}),({'a':{}},{'a':{'late':0}})]:self.assertFalse(b.compare_full_parameters(p,a)['pass'])
 def test_bool_int_double_are_distinct(self):
  for x,y in [(False,0),(0,0.),(True,1),(1.,1)]:self.assertFalse(b.compare_full_parameters({'a':x},{'a':y})['pass'])
 def test_float_exact_no_tolerance_or_signed_zero_folding(self):
  for x,y in [(1.2,float.fromhex('0x1.3333333333334p+0')),(0.,-0.)]:self.assertFalse(b.compare_full_parameters({'a':x},{'a':y})['pass'])
 def test_full_footprint_array_order_and_every_vertex(self):
  p=json.loads((A/b.METHODS[0]/'effective_move_base.json').read_text());a=copy.deepcopy(p);a['global_costmap']['footprint'][63][1]+=1e-12
  self.assertFalse(b.compare_full_parameters(p,a)['pass']);a=copy.deepcopy(p);a['global_costmap']['footprint'].reverse();self.assertFalse(b.compare_full_parameters(p,a)['pass'])
 def test_all_nonfinite_and_non_XMLRPC_types_rejected(self):
  for v in [float('nan'),float('inf'),-float('inf'),None,(),2**31,bytes([1]),{1:'key'}]:
   with self.subTest(value=str(v)),self.assertRaises(ValueError):b.compare_full_parameters({'a':1},{'a':v})
 def test_string_key_order_semantically_unchanged(self):self.assertTrue(b.compare_full_parameters({'a':1,'b':2},{'b':2,'a':1})['pass'])
 def test_three_phases_cannot_omit_lazy_ready(self):
  p={k:{'a':1} for k in b.PHASES};self.assertTrue(b.compare_parameter_phases(p,copy.deepcopy(p))['pass']);a=copy.deepcopy(p);del a['positive_clock_ready']
  with self.assertRaises(ValueError):b.compare_parameter_phases(p,a)
 def test_phase_specific_default_is_never_learned(self):
  p={k:{'a':1} for k in b.PHASES};a=copy.deepcopy(p);a['positive_clock_ready']['autodefault']=2;self.assertFalse(b.compare_parameter_phases(p,a)['pass'])

class PairedAccounting(unittest.TestCase):
 def setUp(self):
  self.scenes=['clear','static0','static1','static2','static3'];self.s=b.proposal_schedule(self.scenes,[])
  self.rows=[dict(run_id=x['run_id'],seed=x['seed'],method=x['method'],attempt=1,record_kind='terminal',phase='complete',capture_integrity_pass=True,geometry_pass=True,goal_elapsed_s=8.,reason='actual_base_goal') for x in self.s['schedule']]
 def test_balanced_exact20_and_zero_authority(self):
  self.assertFalse(self.s['execution_authorized']);self.assertEqual(self.s['allocated_budget'],0);self.assertEqual(len(self.s['schedule']),20)
  for i in range(0,20,2):
   a,c=self.s['schedule'][i:i+2];self.assertEqual(a['seed'],c['seed']);self.assertEqual(a['scene_id'],c['scene_id']);self.assertNotEqual(a['method'],c['method']);self.assertEqual(a['method'],b.METHODS[(i//2)%2])
 def test_any_historical_status_conflict_rejected(self):
  for status in ['consumed','retired','unconsumed','failed','reserved']:
   with self.assertRaises(ValueError):b.proposal_schedule(self.scenes,[dict(seed=b.PROPOSED_SEEDS[0],run_id='old',status=status)])
 def test_historical_id_and_other_proposal_conflicts(self):
  with self.assertRaises(ValueError):b.proposal_schedule(self.scenes,[dict(seed=1,run_id=self.s['schedule'][0]['run_id'])])
  with self.assertRaises(ValueError):b.proposal_schedule(self.scenes,[],[b.PROPOSED_SEEDS[9]])
 def test_no_silent_scene_or_repeat_population_change(self):
  for scenes in [self.scenes[:-1],self.scenes+['sixth'],['same']*5]:
   with self.assertRaises(ValueError):b.proposal_schedule(scenes,[])
 def test_all_success_equal_pairs(self):
  r=b.failure_inclusive_summary(self.s['schedule'],self.rows);self.assertTrue(r['all20_attempted']);self.assertFalse(r['performance_claim']);self.assertTrue(all(p['dwa_minus_teb_administrative_s']==0 for p in r['pairs']))
 def test_timeout_integrity_and_geometry_failures_all_cap(self):
  self.rows[0].update(phase='timeout',goal_elapsed_s=None,reason='timeout');self.rows[1]['capture_integrity_pass']=False;self.rows[2]['geometry_pass']=False
  r=b.failure_inclusive_summary(self.s['schedule'],self.rows);self.assertEqual([x['restricted_time_s'] for x in r['rows'][:3]],[22.,22.,22.]);self.assertEqual(sum(x['valid_completed'] for x in r['rows']),17)
 def test_retirement_administrative_not_observation(self):
  self.rows[0].update(record_kind='retired_unstarted',phase=None,capture_integrity_pass=None,geometry_pass=None,goal_elapsed_s=None,reason='earlier_infrastructure_stop')
  r=b.failure_inclusive_summary(self.s['schedule'],self.rows);m=r['methods'][self.rows[0]['method']];self.assertEqual(m['attempted'],9);self.assertEqual(m['attempted_failure_inclusive_mean_s'],8.);self.assertEqual(m['all_schedule_administrative_cap_mean_s'],9.4);self.assertFalse(r['pairs'][0]['both_attempted'])
 def test_cannot_omit_failed_or_duplicate_record(self):
  with self.assertRaises(ValueError):b.failure_inclusive_summary(self.s['schedule'],self.rows[:-1])
  x=copy.deepcopy(self.rows);x[1]=x[0]
  with self.assertRaises(ValueError):b.failure_inclusive_summary(self.s['schedule'],x)
 def test_id_method_type_budget_or_order_tamper(self):
  for key,value in [('seed',1),('attempt',True),('method','actual_teb_common_route')]:
   x=copy.deepcopy(self.rows);x[0][key]=value
   with self.assertRaises(ValueError):b.failure_inclusive_summary(self.s['schedule'],x)
  s=copy.deepcopy(self.s['schedule']);s[0]['attempt']=True
  with self.assertRaises(ValueError):b.failure_inclusive_summary(s,self.rows)
 def test_invalid_elapsed_private_success_and_fake_retirement(self):
  for change in [dict(goal_elapsed_s=True),dict(goal_elapsed_s=22.0001),dict(goal_elapsed_s=float('nan')),dict(phase='private_success'),dict(record_kind='retired_unstarted')]:
   x=copy.deepcopy(self.rows);x[0].update(change)
   with self.assertRaises(ValueError):b.failure_inclusive_summary(self.s['schedule'],x)

if __name__=='__main__':unittest.main(verbosity=2)
