import copy,csv,json,pathlib,sys,unittest
P=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(P));S=P.parents[2]
import baseline_static_qualification_b05 as old
import baseline_static_qualification_b06 as new

class SeedTypeRepair(unittest.TestCase):
 def setUp(self):self.scenes=['clear','s0','s1','s2','s3']
 def test_all_ten_old_string_bypasses_now_rejected(self):
  for seed in old.PROPOSED_SEEDS:
   self.assertEqual(old.proposal_schedule(self.scenes,[],[str(seed)]),old.proposal_schedule(self.scenes,[]))
   with self.subTest(seed=seed),self.assertRaises(ValueError):new.proposal_schedule(self.scenes,[],[str(seed)])
 def test_all_ten_real_integer_collisions_still_rejected(self):
  for seed in old.PROPOSED_SEEDS:
   for values in ([seed],(seed,)):
    with self.subTest(seed=seed),self.assertRaises(ValueError):new.proposal_schedule(self.scenes,[],values)
 def test_no_bool_float_subclass_or_null_coercion(self):
  class DerivedInt(int):pass
  for seed in [True,False,263004001.,float('nan'),None,DerivedInt(263004001),'263004001']:
   with self.subTest(seed=str(seed)),self.assertRaises(ValueError):new.proposal_schedule(self.scenes,[],[seed])
 def test_wrong_containers_fail_before_interpretation(self):
  for values in ['263004001',b'263004001',{263004001:None},{263004001},iter([263004001]),263004001,None]:
   with self.subTest(container=type(values).__name__),self.assertRaises(ValueError):new.proposal_schedule(self.scenes,[],values)
 def test_mixed_sequence_does_not_filter_bad_element(self):
  for values in [[263003001,'263004001'],(263003001,False)]:
   with self.assertRaises(ValueError):new.proposal_schedule(self.scenes,[],values)
 def test_valid_empty_and_nonconflicting_sequences_unchanged(self):
  for values in [[],(),[263003001,263003002],(263003001,263003002),[-1,0,2**63]]:
   self.assertEqual(new.proposal_schedule(self.scenes,[],values),old.proposal_schedule(self.scenes,[],values))
 def test_existing_registry_csv_string_policy_unchanged(self):
  for seed in old.PROPOSED_SEEDS:
   with self.assertRaises(ValueError):new.proposal_schedule(self.scenes,[dict(seed=str(seed),run_id='past')])
 def test_no_input_mutation(self):
  prior=[263003001,263003002];scenes=self.scenes[:];history=[dict(seed='1',run_id='old')];before=copy.deepcopy((prior,scenes,history));new.proposal_schedule(scenes,history,prior);self.assertEqual((prior,scenes,history),before)
 def test_actual_frozen_inputs_exact_old_schedule(self):
  d=json.loads((S/'contracts/baseline_static_qualification_b05_draft.json').read_text());scenes=[x['scene_id'] for x in d['scenes']]
  registry=list(csv.DictReader((S/'audits/baseline_static_qualification_b05/seed_registry_at_proposal.csv').open()))
  n06=json.loads((S/'contracts/navigation_n06_prepared_r01/prepared.json').read_text());prior=[x['seed'] for x in n06['units']]
  self.assertTrue(all(type(x) is int for x in prior));expected=json.loads((S/'audits/baseline_static_qualification_b05/proposed_schedule.json').read_text())
  result=new.proposal_schedule(scenes,registry,prior);self.assertEqual(result,expected)
  self.assertEqual(json.dumps(result,sort_keys=True,separators=(',',':')),json.dumps(old.proposal_schedule(scenes,registry,prior),sort_keys=True,separators=(',',':')))
 def test_authority_and_actual_math_not_added(self):
  result=new.proposal_schedule(self.scenes,[]);self.assertFalse(result['execution_authorized']);self.assertFalse(result['training_authorized']);self.assertEqual(result['allocated_budget'],0);self.assertEqual(result['stage'],old.STAGE)
  self.assertFalse(hasattr(new,'compare_full_parameters'));self.assertFalse(hasattr(new,'failure_inclusive_summary'))

if __name__=='__main__':unittest.main(verbosity=2)
