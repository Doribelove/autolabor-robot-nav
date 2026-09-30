"""Tiny CPU forward-only fixture; no optimizer, training, GPU, ROS or run seeds."""
from pathlib import Path
import sys,json,hashlib,collections
S=Path(__file__).resolve().parents[2];A=Path(__file__).parent
source=S/'workspace/src/application/v5_e2e_scope_rl/src'
sys.path.insert(0,str(source))
import numpy as np
import torch
from torch.distributions import Categorical
from v5_e2e_scope_rl.asymmetric_ppo import SensorActor,stack_observations,observations_to_torch

torch.set_num_threads(1);torch.set_num_interop_threads(1);torch.manual_seed(17)
actor=SensorActor();actor.eval();observations=[];names=[]
cases=[('all_valid_gradient',[0.,.1,.2,.3,.4,.5,.6,.7],[1]*8),('four_valid',[4.,3.,2.,1.,0.,-1.,-2.,-3.],[0,1,0,1,0,1,0,1]),('tied_prior',[0.]*8,[1]*8),('single_valid',[0.,1.,2.,3.,4.,5.,6.,7.],[0,0,0,1,0,0,0,0])]
for label,prior,mask in cases:
 for j in range(16):
  observations.append(dict(scans=np.linspace(.01,.95,6*72,dtype=np.float32).reshape(6,72)*(1+j/64),robot=np.linspace(-.4,.4,10,dtype=np.float32),candidates=np.linspace(-.5,.5,8*7,dtype=np.float32).reshape(8,7),analytic_prior=np.asarray(prior,np.float32),action_mask=np.asarray(mask,bool)));names.append(label)
batch=observations_to_torch(stack_observations(observations),torch.device('cpu'))
before={k:v.clone() for k,v in actor.state_dict().items()}
with torch.no_grad():
 torch.manual_seed(29);final,info=actor.choose_actions(batch,deterministic=False,advantage_gate_margin=.10)
 torch.manual_seed(29);ungated,ungated_info=actor.choose_actions(batch,deterministic=False,advantage_gate_margin=None)
 torch.manual_seed(29);zero_margin,zero_info=actor.choose_actions(batch,deterministic=False,advantage_gate_margin=0.)
 distribution=Categorical(logits=info['logits']);latent_logprob=distribution.log_prob(info['learned_action'])
 # Enumerate the deterministic gate mapping for every latent action, not only sampled actions.
 analytic=info['analytic_action'];row=torch.arange(len(names));margins=info['residual']-info['residual'][row,analytic][:,None]
 mapping=torch.where((margins>=.10)&(torch.arange(8)[None,:]!=analytic[:,None]),torch.arange(8)[None,:],analytic[:,None])
 executed_probability=torch.zeros_like(distribution.probs);executed_probability.scatter_add_(1,mapping,distribution.probs)
 output=[]
 for label,_,_ in cases:
  ids=[i for i,x in enumerate(names) if x==label];latent=info['learned_action'][ids];f=final[ids];an=analytic[ids]
  output.append(dict(case=label,rows=len(ids),mask_valid=int(batch['action_mask'][ids[0]].sum()),analytic_actions=an.tolist(),latent_action_counts=dict(collections.Counter(map(int,latent))),final_action_counts=dict(collections.Counter(map(int,f))),latent_nonanalytic=int((latent!=an).sum()),final_nonanalytic=int((f!=an).sum()),gate_interventions=int(info['intervened'][ids].sum()),max_abs_residual=float(info['residual'][ids].abs().max()),max_abs_residual_margin=float(info['residual_margin'][ids].abs().max()),latent_entropy_nats=float(distribution.entropy()[ids].mean()),enumerated_final_analytic_probability=executed_probability[ids,an].tolist(),latent_logprob=latent_logprob[ids].tolist(),ungated_nonanalytic=int((ungated[ids]!=an).sum()),zero_margin_nonanalytic=int((zero_margin[ids]!=an).sum())))
 checks={'actual_fresh_head_exact_zero':all(torch.count_nonzero(v)==0 for v in (actor.residual_head[-1].weight,actor.residual_head[-1].bias)),'residuals_exact_zero':bool(torch.count_nonzero(info['residual'])==0),'all_sampled_postgate_actions_analytic':bool(torch.equal(final,analytic)),'latent_sampling_really_nonanalytic':bool((info['learned_action']!=analytic).any()),'same_latent_sample_in_all_three_calls':bool(torch.equal(info['learned_action'],ungated_info['learned_action']) and torch.equal(info['learned_action'],zero_info['learned_action'])),'ungated_equals_actual_latent':bool(torch.equal(ungated,info['learned_action'])),'zero_margin_equals_actual_latent':bool(torch.equal(zero_margin,info['learned_action'])),'enumerated_final_probability_one':bool(torch.allclose(executed_probability[row,analytic],torch.ones(len(names)),atol=2e-7,rtol=0)),'weights_unchanged':all(torch.equal(v,before[k]) for k,v in actor.state_dict().items()),'no_gradients_created':all(p.grad is None for p in actor.parameters()),'cpu_only':all(p.device.type=='cpu' for p in actor.parameters())}
 assert all(checks.values()),checks
 def bind(p):return dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
 report=dict(schema='astra_m09_fresh_actor_gate_support_fixture_v1',status='dead_zone_confirmed_forward_only',scope='synthetic observation schema fixtures, not training or simulation evidence',interpreter=sys.executable,python=sys.version,torch_version=torch.__version__,torch_file=torch.__file__,numpy_version=np.__version__,threads=torch.get_num_threads(),observations=len(names),gate_margin=.10,training_steps=0,optimizer_created=False,gpu_used=False,simulation_seeds_used=False,checks=checks,cases=output,source_bindings=[bind(p) for p in [Path(__file__),source/'v5_e2e_scope_rl/asymmetric_ppo.py',S/'workspace/scripts/astra_gazebo/route_tracks_actor_adapter_m08.py',S/'audits/route_training_alignment_m09/survey.json',S/'audits/route_training_alignment_m09/design.md']])
 (A/'probe_result.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps({'checks':checks,'cases':[{k:x[k] for k in ['case','latent_nonanalytic','final_nonanalytic','ungated_nonanalytic','zero_margin_nonanalytic','latent_entropy_nats']} for x in output]},indent=2))
