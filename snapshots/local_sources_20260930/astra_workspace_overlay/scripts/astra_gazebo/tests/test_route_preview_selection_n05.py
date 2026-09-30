"""Small pure selection/inherited-build checks, no ROS or physical rollout."""
import copy
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import route_candidate_core as old
from route_preview_selection_n05 import PreviewEligibleRuntime,select_preview_eligible,METHOD,OBSERVATION_SCHEMA


def observation(mask=None,prior=None):
    return {'action_mask':np.asarray([True]*8 if mask is None else mask,dtype=bool),
            'analytic_prior':np.asarray([0,1,2,3,4,5,6,7] if prior is None else prior,dtype=np.float32),
            'candidates':np.zeros((8,7),np.float32)}


class SelectionTests(unittest.TestCase):
    def test_negative_winner_yields_to_existing_eligible(self):
        x=observation();d=select_preview_eligible(x,[1,1,1,1,1,1,1,-.01]);self.assertEqual((d.original_analytic,d.chosen),(7,6));self.assertTrue(d.backup_mask[7]);self.assertFalse(d.selection_eligibility[7])
    def test_nonnegative_boundary_included(self):
        self.assertEqual(select_preview_eligible(observation(),[0]*8).chosen,7)
    def test_unrounded_negative_not_erased_by_float32_underflow(self):
        x=observation();self.assertEqual(float(np.float32(-1e-50)),0.)
        self.assertEqual(select_preview_eligible(x,[1]*7+[-1e-50]).chosen,6)
    def test_positive_preview_does_not_unmask_failed_backup(self):
        x=observation(mask=[True]*7+[False]);d=select_preview_eligible(x,[1]*8);self.assertEqual(d.chosen,6);self.assertFalse(d.selection_eligibility[7])
    def test_stop_retains_backup_only_eligibility(self):
        d=select_preview_eligible(observation(),[-1]*8);self.assertEqual(d.chosen,0);self.assertTrue(d.selection_eligibility[0]);self.assertIsNone(d.reason)
    def test_stop_can_still_win_original_prior(self):
        d=select_preview_eligible(observation(prior=[9,1,2,3,4,5,6,7]),[-1]+[1]*7);self.assertEqual(d.chosen,0)
    def test_no_eligible_stop_is_not_fabricated_valid(self):
        x=observation(mask=[False]+[True]*7);d=select_preview_eligible(x,[-1]*8);self.assertEqual(d.legacy_result(),(0,None,'no_full_preview_eligible_action',0.));self.assertFalse(any(d.selection_eligibility))
    def test_all_backup_fail_keeps_original_reason(self):
        d=select_preview_eligible(observation(mask=[False]*8),[1]*8);self.assertEqual(d.legacy_result(),(0,None,'no_prediction_model_backup_pass',0.))
    def test_lowest_index_tie_unchanged(self):
        d=select_preview_eligible(observation(prior=[0,3,3,0,0,0,0,0]),[1]*8);self.assertEqual(d.chosen,1)
    def test_no_input_mutation(self):
        x=observation();saved=copy.deepcopy(x);full=np.ones(8);select_preview_eligible(x,full)
        for k in x:np.testing.assert_array_equal(x[k],saved[k])
        np.testing.assert_array_equal(full,np.ones(8))
    def test_nonfinite_and_malformed_values_fail_closed(self):
        for full in ([1]*7,[1]*7+[float('nan')],[1]*7+[float('inf')],['1']*8):
            with self.subTest(full=full),self.assertRaises(ValueError):select_preview_eligible(observation(),full)
        for key,value in [('action_mask',np.ones(8)),('analytic_prior',np.full(8,np.nan)),('analytic_prior',np.ones(7))]:
            x=observation();x[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):select_preview_eligible(x,[1]*8)
    def test_actor_and_checkpoint_rejected_before_geometry(self):
        for kwargs in ({'actor':lambda x:0},{'checkpoint_metadata':{}}):
            with self.assertRaisesRegex(ValueError,'analytic-only'):PreviewEligibleRuntime(None,None,None,**kwargs)


class Fields:
    def __init__(self):
        self.config=old.RouteConfig();self.previous_action=0;self.stop_count=0;self.actor=None
        self.route=SimpleNamespace(length=9.);self._n05_capture=None;self._n05_expected_calls=None
    def validate_inputs(self,*args):pass
    def state(self,actual):return {'base':(0.,0.,0.),'vx':.5}
    def predict(self,scan):return {'dynamic_count':0,'stamp_ns':scan.stamp_ns,'dynamic_shape':None}
    def project(self,xy):return {'arc_m':float(xy[0]),'lateral_m':float(xy[1]),'heading_rad':0.}
class Baseline(Fields,old.RouteAwareRuntime):pass
class Treatment(Fields,PreviewEligibleRuntime):pass

def fixture_rollout(self,state,action,backup=False):
    value=.2+action*.01 if backup else ((-1e-50 if action==7 else .1+action*.01))
    return [{'base':(0.,0.,0.),'vx':.5,'steer':0.,'t':0.,'clearance':value},
            {'base':(.2*action,0.,0.),'vx':.5,'steer':0.,'t':.1,'clearance':value}]
def fixture_verify(self,states,prediction,now_ns):return states[0]['clearance']


class IntegrationTests(unittest.TestCase):
    def build(self,obj):
        scan=SimpleNamespace(stamp_ns=1_000_000_000,model_ranges=np.ones(72))
        return obj.build({},scan,np.ones((6,72)),1.,1_000_000_000)
    def test_real_inherited_build_outputs_equal_and_no_second_verification(self):
        a,b=Baseline(),Treatment();calls=[]
        def verify(self,states,prediction,now):calls.append((id(self),id(states)));return fixture_verify(self,states,prediction,now)
        with patch.object(old.RouteAwareRuntime,'rollout',fixture_rollout),patch.object(old.RouteAwareRuntime,'verify',verify):
            control=self.build(a);treated=self.build(b)
        self.assertEqual(len(calls),32)
        for key,value in control['observation'].items():np.testing.assert_array_equal(value,treated['observation'][key])
        for key in ['previews','backup_clearances','prediction_audit','mask']:self.assertEqual(control[key],treated[key])
        self.assertEqual(treated['full_preview_clearances_m'][-1],-1e-50)
        self.assertEqual(float(treated['observation']['candidates'][-1,3]),0.)
        self.assertFalse(treated['preview_selection_audit']['selection_eligibility'][-1])
        self.assertTrue(treated['observation']['action_mask'][-1]);self.assertIsNone(b._n05_capture)
    def test_unexpected_verify_protocol_rejected_and_capture_cleared(self):
        obj=Treatment()
        with patch.object(old.RouteAwareRuntime,'build',return_value={}):
            with self.assertRaisesRegex(RuntimeError,'protocol'):self.build(obj)
        self.assertIsNone(obj._n05_capture);self.assertIsNone(obj._n05_expected_calls)
    def test_parent_exception_kept_and_capture_cleared(self):
        obj=Treatment()
        with patch.object(old.RouteAwareRuntime,'build',side_effect=ValueError('original fault')):
            with self.assertRaisesRegex(ValueError,'original fault'):self.build(obj)
        self.assertIsNone(obj._n05_capture)
    def test_recursive_build_rejected(self):
        obj=Treatment();obj._n05_capture=[]
        with self.assertRaisesRegex(RuntimeError,'serialized'):self.build(obj)
    def test_installed_actor_rejected(self):
        obj=Treatment();obj.actor=lambda x:0
        with self.assertRaisesRegex(ValueError,'actor'):obj.select({})
    def test_command_revalidation_and_all_dynamics_are_inherited(self):
        for name in ['step','command','state','predict','goal_reached']:
            self.assertIs(getattr(PreviewEligibleRuntime,name),getattr(old.RouteAwareRuntime,name))
    def test_namespace_distinct(self):
        self.assertEqual(METHOD,'astra_analytic_preview_eligible_n05');self.assertNotEqual(OBSERVATION_SCHEMA,old.OBSERVATION_SCHEMA)

if __name__=='__main__':unittest.main()
