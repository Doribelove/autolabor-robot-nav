"""Actual unchanged gate method and tiny CPU tensor algebra only; no actor instance/optimizer/RNG."""
from pathlib import Path
import hashlib,json,math,sys,struct
S=Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926');W=S/'workspace';A=Path(__file__).resolve().parent
sys.path.insert(0,str(W/'scripts/astra_gazebo'));sys.path.insert(0,str(W/'src/application/v5_e2e_scope_rl/src'))
import route_training_proposal_m10 as m
import torch
from v5_e2e_scope_rl.asymmetric_ppo import SensorActor
torch.set_num_threads(1);torch.set_num_interop_threads(1)
class Forward:
 def __init__(self,l,r):self.l,self.r=l,r
 def forward(self,o):return self.l,self.r,None
cases=0;score_error=0.
for bits in [1,3,85,170,255]:
 mask=torch.tensor([[bool(bits&(1<<i)) for i in range(8)]])
 prior=torch.tensor([[0.,2.,2.,-1.,1.,-2.,0.,2.]])
 base=max((i for i in range(8) if bool(mask[0,i])),key=lambda i:float(prior[0,i]))
 for target in range(8):
  if not mask[0,target]:continue
  for delta in [.09999999,.1,.10000001,.25]:
   residual=torch.zeros(1,8);residual[0,target]=delta
   logits=torch.zeros(1,8).masked_fill(~mask,-1.e9);logits[0,target]=1.
   obs={'analytic_prior':prior,'action_mask':mask}
   expected,_=SensorActor.choose_actions(Forward(logits,residual),obs,True,.1)
   output=m.ActorOutput(tuple(logits[0].tolist()),tuple(residual[0].tolist()),tuple(prior[0].tolist()),tuple(mask[0].tolist()))
   result=m.deterministic_deployment(output);assert result['proposal_action']==int(expected.item());cases+=1
   # Independent masked softmax and scatter-add; differentiate tiny logits only.
   d=m.distribution(output);x=logits[0].double().detach().requires_grad_(True);pi=torch.softmax(x.masked_fill(~mask[0],float('-inf')),0)
   index=torch.tensor([g if g is not None else base for g in d.gate_map]);q=.2*pi+.8*torch.zeros_like(pi).scatter_add(0,index,pi)
   for action in (base,target):
    g,=torch.autograd.grad(q[action].log(),x,retain_graph=True);err=max(abs(float(g[j])-d.logit_score(action)[j]) for j in range(8));score_error=max(score_error,err)
assert score_error<1e-14
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
result=dict(status='actual_gate_and_CPU_tensor_pushforward_check_pass',original_method_cases=cases,max_score_error=score_error,actor_instances=0,optimizer_created=False,model_parameters_touched=False,training_steps=0,ROS_started=False,torch_version=torch.__version__,python=sys.version,resources=[dict(path=str(p),sha256=sha(p)) for p in [Path(__file__),Path(m.__file__),W/'src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/asymmetric_ppo.py']])
(A/'native_math_result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='resources'}))
