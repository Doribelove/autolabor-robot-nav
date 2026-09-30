"""Small CPU forward/algebra fixture. No optimizer, learning, ROS or GPU calls."""
from pathlib import Path
import hashlib
import json
import sys

A = Path(__file__).resolve().parent
S = A.parents[1]
W = S / 'workspace'
sys.path.insert(0, str(W / 'scripts/astra_gazebo'))
sys.path.insert(0, str(W / 'src/application/v5_e2e_scope_rl/src'))
import route_training_proposal_m10 as m
import torch
from v5_e2e_scope_rl.asymmetric_ppo import SensorActor

torch.set_num_threads(1)
torch.set_num_interop_threads(1)
torch.manual_seed(41)  # Synthetic CPU fixture only, not an episode/learning seed.
actor = SensorActor().eval()
before = {k: v.clone() for k, v in actor.state_dict().items()}
obs = dict(scans=torch.linspace(.01, .95, 6 * 72).reshape(1, 6, 72),
           robot=torch.linspace(-.4, .4, 10).reshape(1, 10),
           candidates=torch.linspace(-.5, .5, 8 * 7).reshape(1, 8, 7),
           analytic_prior=torch.arange(8, dtype=torch.float32).reshape(1, 8),
           action_mask=torch.ones(1, 8, dtype=torch.bool))
with torch.no_grad():
    logits, residual, _ = actor.forward(obs)
    actual_deploy, _ = actor.choose_actions(obs, deterministic=True, advantage_gate_margin=.10)
output = m.ActorOutput(tuple(logits[0].tolist()), tuple(residual[0].tolist()),
                       tuple(obs['analytic_prior'][0].tolist()), tuple(obs['action_mask'][0].tolist()))
d = m.distribution(output)
assert residual.count_nonzero() == 0 and actual_deploy.item() == d.analytic_action == 7
assert d.q[0] > 0 and sum(d.q[:7]) > 0

# Exact original method exercised on controlled forward output for each latent,
# including nonzero residual, float32 subtraction and equality threshold edges.
class ForwardFixture:
    def __init__(self, lo, re):
        self.lo, self.re = lo, re
    def forward(self, observation):
        return self.lo, self.re, torch.zeros(1, 1)

parity = []
for base in (0., -.2, .4):
    for delta in (.09999999, .1, .10000001, .2):
        for latent in range(8):
            lo = torch.zeros(1, 8)
            lo[0, latent] = 1.
            re = torch.full((1, 8), base, dtype=torch.float32)
            re[0, latent] = base + delta
            proposed, info = SensorActor.choose_actions(ForwardFixture(lo, re), obs, True, .10)
            value = m.ActorOutput(tuple(lo[0].tolist()), tuple(re[0].tolist()), output.prior, output.mask)
            ours = m.distribution(value).gate_map[latent]
            assert ours == proposed.item()
            parity.append((base, delta, latent, ours))

# Independent differentiable pushforward using native torch scatter-add.
# This differentiates a tiny logits tensor, not actor weights; no training step.
x = logits[0].double().detach().requires_grad_(True)
pi = torch.softmax(x, dim=0)
mapping = torch.full((8,), 7, dtype=torch.int64)
q = .2 * pi + .8 * torch.zeros_like(pi).scatter_add(0, mapping, pi)
assert max(abs(float(q[i].detach()) - d.q[i]) for i in range(8)) < 2e-16
max_score_error = 0.
for action in (0, 7):
    gradient, = torch.autograd.grad(q[action].log(), x, retain_graph=True)
    error = max(abs(float(gradient[j]) - d.logit_score(action)[j]) for j in range(8))
    max_score_error = max(error, max_score_error)
assert max_score_error < 3e-16
assert all(torch.equal(v, before[k]) for k, v in actor.state_dict().items())
assert all(p.grad is None and p.device.type == 'cpu' for p in actor.parameters())

def bind(path):
    path = path.resolve()
    return dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())

report = dict(schema='astra_m10_actual_actor_cpu_probe_v1',
              status='forward_gate_and_pushforward_algebra_pass',
              interpreter=sys.executable, python=sys.version, torch_version=torch.__version__,
              torch_file=torch.__file__, threads=torch.get_num_threads(),
              gate_cases=len(parity), original_gate_exact_parity=True,
              fresh_nonanalytic_proposal_probability=sum(d.q[:7]),
              fresh_deployment_action=actual_deploy.item(), pi=d.pi, q=d.q,
              max_autograd_score_error=max_score_error, model_weights_unchanged=True,
              model_gradients_absent=True, cpu_only=True, optimizer_created=False,
              training_steps=0, simulation_episodes=0, benchmark=False,
              bindings=[bind(Path(__file__)), bind(Path(m.__file__)),
                        bind(W / 'src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/asymmetric_ppo.py')])
(A / 'actual_actor_probe.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
print(json.dumps(report, indent=2, allow_nan=False))
