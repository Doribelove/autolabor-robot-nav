"""Small stdlib-only interface checks; no tensor model/GPU/ROS import."""
import unittest
from interfaces import *
H='a'*64
class InterfaceTests(unittest.TestCase):
    def obs(self,mask=True):return dict(scans=[[1.]*72 for _ in range(6)],robot=[0.]*10,candidates=[[0.]*7 for _ in range(8)],analytic_prior=[0.]*8,action_mask=[mask]*8)
    def context(self,mask=True):return DecisionContext('d',2_000_000_000,2_000_000_000,2_000_000_000,2_000_000_000,self.obs(mask),H,H,tuple(1_000_000_000+i*200_000_000 for i in range(6)))
    def test_policy_oracle_keys_not_silently_filtered(self):
        x=self.obs();x['oracle_actor_x']=0
        with self.assertRaises(ValueError):validate_policy_observation(x)
    def test_empty_mask_preserved_and_no_gradient(self):
        c=self.context(False).validate();p=ProposalReceipt('d',None,None,0,True,False,H).validate(c);self.assertFalse(p.policy_gradient_eligible())
    def test_latent_probability_is_not_executed_action(self):
        c=self.context().validate();p=ProposalReceipt('d',7,-2.,3,True,True,H).validate(c);self.assertTrue(p.policy_gradient_eligible());self.assertNotEqual(p.sampled_latent_action,p.post_advantage_gate_action)
    def test_expired_actor_not_an_onpolicy_sample(self):
        p=ProposalReceipt('d',7,-2.,3,False,True,H).validate(self.context());self.assertFalse(p.policy_gradient_eligible())
    def test_critic_future_rejected(self):
        with self.assertRaises(ValueError):PrivilegedCriticState('d',2_000_000_001,(0.,)*48,H).validate(self.context())
    def test_no_training_on_final_and_no_replay_counterfactual(self):
        with self.assertRaises(ValueError):EpisodeSpec('x','gazebo_train','final_test',FAMILIES[0],'p',261000001,260927001,H,H,H,H).validate()
        with self.assertRaises(RuntimeError):require_interactive_backend('sensor_replay')
if __name__=='__main__':unittest.main()
