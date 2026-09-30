"""Independent stdlib math and receipt fixtures; no model/ROS/training."""
from dataclasses import replace,asdict
from pathlib import Path
from decimal import Decimal,localcontext
import math,struct,sys
import pytest
S=Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926');sys.path.insert(0,str(S/'workspace/scripts/astra_gazebo'))
import route_training_proposal_m10 as m
f32=lambda x:struct.unpack('f',struct.pack('f',x))[0]
def output(mask=(True,)*8):
 prior=tuple(map(f32,[-2,3,3,-1,0,2,-3,1]));res=tuple(map(f32,[.2,0,.1,-.1,.3,0,.2,-.1]));lo=tuple(map(f32,[p/3+r for p,r in zip(prior,res)]))
 return m.ActorOutput(lo,res,prior,mask)
def context():return m.Context('independent_fixture','a'*64,'b'*64,'c'*64,1_000_000_000,900_000_000,950_000_000,980_000_000,2_000_000_000)
def proposal(o=None):return m.sample(context(),o or output(),2_005_000_000,2_015_000_000,.1,.35)
def receipt(p):return m.finalize(context(),p,'proposal_applied',1_030_000_000,2_040_000_000,p.proposal_action)
def decimal_q(o):
 # High-precision independent branch/latent enumeration; only gate scalar rounding is declared f32.
 with localcontext() as c:
  c.prec=80
  valid=[i for i in range(8) if o.mask[i]];base=sorted(valid,key=lambda i:(-o.prior[i],i))[0];w=[Decimal.from_float(x).exp() if o.mask[i] else Decimal(0) for i,x in enumerate(o.logits)];z=sum(w);p=[v/z for v in w];q=[Decimal(0)]*8
  for i in valid:
   a=i if i!=base and f32(o.residual[i]-o.residual[base])>=f32(.1) else base
   q[i]+=p[i]/5;q[a]+=p[i]*4/5
  return tuple(float(x) for x in q)

def test_all255_nonempty_masks_mass_and_independent_enumeration():
 for bitmask in range(1,256):
  o=output(tuple(bool(bitmask&(1<<i)) for i in range(8)));d=m.distribution(o);ref=decimal_q(o)
  assert max(abs(x-y) for x,y in zip(d.q,ref))<3e-16
  assert math.isclose(sum(d.q),1,abs_tol=4e-16)
  for i,valid in enumerate(o.mask):
   assert (d.q[i]>0)==valid
   assert not valid or d.q[i]>=.2*d.pi[i]
   assert valid or all(x[i]==0 for x in (d.pi,d.q))

def test_conditional_score_and_zero_expected_score_all_masks():
 for bitmask in (1,3,15,85,170,255):
  d=m.distribution(output(tuple(bool(bitmask&(1<<i)) for i in range(8))))
  for j in range(8):
   assert abs(math.fsum(d.q[a]*d.logit_score(a)[j] for a in range(8) if d.q[a]>0))<4e-16
  for a in range(8):
   if d.q[a]>0:assert abs(sum(d.logit_score(a)))<4e-16

def test_branch_and_cdf_halfopen_boundaries():
 o=m.ActorOutput((0.,)*8,(0.,)*8,(0.,)*8,(True,)*8)
 below=struct.unpack('d',struct.pack('Q',struct.unpack('Q',struct.pack('d',.2))[0]-1))[0]
 for u,branch in [(below,'exploration'),(.2,'residual_gate')]:
  for latent in range(8):
   p=m.sample(context(),o,2_000_000_000,2_010_000_000,u,latent/8)
   assert p.branch==branch and p.latent_action==latent
   assert p.proposal_action==(latent if branch=='exploration' else 0)

@pytest.mark.parametrize('offset,eligible',[(50_000_000,True),(50_000_001,False)])
def test_final_deadline_exact_integer_boundary(offset,eligible):
 p=proposal();r=m.finalize(context(),p,'proposal_applied',1_000_000_000+offset,2_000_000_000+offset,p.proposal_action)
 assert r.policy_gradient_eligible is eligible

@pytest.mark.parametrize('change',[
 {'latent_uniform':.99},{'branch_uniform':.9},{'latent_log_probability':123.},
 {'proposal_log_probability':0.},{'latent_gate_action':5},
])
def test_receipt_tamper_replay_rejects(change):
 p=proposal();q=replace(p,**change)
 # Cases are chosen to differ from the original deterministic replay, even if branch leads to same final action.
 with pytest.raises(ValueError,match='receipt mismatch'):receipt(q)

@pytest.mark.parametrize('name',['actor_invalid','empty_mask','deadline_dropped','source_expired','clock_fault'])
def test_negative_dispositions_never_enter_update(name):
 p=proposal();r=m.finalize(context(),p,name,1_020_000_000,2_030_000_000,0)
 assert not r.policy_gradient_eligible
 with pytest.raises(ValueError,match='no_policy_gradient'):m.update_likelihood(r,output(),'d'*64,'a'*64,'b'*64)

def test_current_gate_ratio_and_KL_decimal_reference():
 p=proposal();r=receipt(p);new=replace(output(),residual=tuple(map(f32,[0,.3,0,.2,0,0,0,0])))
 x=m.update_likelihood(r,new,'d'*64,'a'*64,'b'*64);oldq=decimal_q(output());newq=decimal_q(new)
 assert x.ratio==pytest.approx(newq[x.action]/oldq[x.action],abs=1e-15)
 kl=math.fsum(a*(math.log(a)-math.log(b)) for a,b in zip(oldq,newq) if a>0)
 assert x.old_to_new_kl==pytest.approx(kl,abs=2e-16)
 assert any(x.gate_changed_indices)

def test_single_valid_action_score_zero_even_when_stop_invalid():
 mask=(False,False,False,False,True,False,False,False);d=m.distribution(output(mask))
 assert d.q[4]==1 and d.logit_score(4)==(0.,)*8


def test_extreme_inconsistent_logits_numeric_limit_is_reproducible():
 # This is deliberately NOT a reachable frozen actor forward output. It documents
 # the pure API syntactic domain beyond authenticated bounded original outputs.
 o=m.ActorOutput((0.,)*8,(0.,)*8,(0.,)*8,(True,)*8);p=m.sample(context(),o,2_000_000_000,2_010_000_000,.9,.1);r=receipt(p)
 impossible=replace(o,logits=(0.,-740.,0.,0.,0.,0.,0.,0.))
 x=m.update_likelihood(r,impossible,'d'*64,'a'*64,'b'*64)
 assert math.isfinite(x.ratio) and math.isinf(x.old_to_new_kl)
