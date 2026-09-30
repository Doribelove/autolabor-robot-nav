"""M09 implementation contract draft. Pure types/guards; no trainer or simulator.

A future adapter implements these protocols by calling frozen M08 core methods.
This module does not authorize episodes, allocate seeds, load weights or train.
"""
from dataclasses import dataclass
import math
from typing import Any,Mapping,Optional,Protocol,Tuple

SCHEMA='astra_route_tracks_observation_m08_v1'
FORMAT='astra_route_tracks_sensor_actor_m08_v1'
POLICY_KEYS=frozenset(('scans','robot','candidates','analytic_prior','action_mask'))
SHAPES={'scans':(6,72),'robot':(10,),'candidates':(8,7),'analytic_prior':(8,),'action_mask':(8,)}
BACKENDS=frozenset(('gazebo_train','sensor_replay','fast_proxy'))
FAMILIES=('static_avoidance','single_crossing','head_on_yield','sequential_interaction','occluded_emergence','width_change_corridor')
PROPOSED_INITIALIZATION_SEEDS=(260927001,260927002,260927003,260927004,260927005)


def digest(value):
    if type(value)is not str or len(value)!=64 or any(c not in '0123456789abcdef' for c in value):raise ValueError('lowercase SHA256 required')
    return value


def _shape(value):
    if hasattr(value,'shape'):return tuple(value.shape)
    if not isinstance(value,(list,tuple)):return ()
    if not value:return (0,)
    shapes=[_shape(v) for v in value]
    if any(s!=shapes[0] for s in shapes):raise ValueError('ragged policy tensor')
    return (len(value),)+shapes[0]


def validate_policy_observation(value):
    # Do not silently slice out oracle keys: their presence is an error.
    if set(value)!=POLICY_KEYS:raise ValueError('policy must contain exactly five deployment keys')
    for key,shape in SHAPES.items():
        if _shape(value[key])!=shape:raise ValueError('policy shape mismatch: '+key)
    def scalars(x):
        if hasattr(x,'tolist'):x=x.tolist()
        if isinstance(x,(tuple,list)):
            for v in x:yield from scalars(v)
        else:yield x
    for key in POLICY_KEYS:
        for number in scalars(value[key]):
            if key=='action_mask':
                if type(number)is not bool:raise ValueError('mask must be bool, including all-false case')
            elif isinstance(number,bool) or not isinstance(number,(int,float)) or not math.isfinite(number):raise ValueError('finite policy numeric required')
    # All-false is valid input. The collector must not create a Categorical on it.
    return value


@dataclass(frozen=True)
class EpisodeSpec:
    run_id:str
    backend:str
    split:str
    family:str
    parent_geometry_id:str
    episode_seed:int
    initialization_seed:int
    scene_sha256:str
    bundle_sha256:str
    candidate_semantics_sha256:str
    release_sha256:Optional[str]=None
    def validate(self):
        if not self.run_id or not self.parent_geometry_id or self.backend not in BACKENDS or self.family not in FAMILIES:raise ValueError('closed episode identity required')
        if self.split!='train':raise ValueError('training backend cannot open dev/validation/final outcomes')
        if self.initialization_seed not in PROPOSED_INITIALIZATION_SEEDS:raise ValueError('draft five-seed roster only; not a reservation')
        if type(self.episode_seed)is not int or not 261000000<=self.episode_seed<=262999999:raise ValueError('train namespace required; global collision audit remains external')
        for value in (self.scene_sha256,self.bundle_sha256,self.candidate_semantics_sha256):digest(value)
        if self.backend=='gazebo_train':digest(self.release_sha256)
        return self


@dataclass(frozen=True)
class DecisionContext:
    decision_id:str
    decision_source_ns:int
    actual_state_source_ns:int
    scan_source_ns:int
    observation_available_ns:int
    observation:Mapping[str,Any]
    sensor_receipt_sha256:str
    candidate_semantics_sha256:str
    history_source_ns:Tuple[int,...]
    def validate(self):
        for stamp in (self.decision_source_ns,self.actual_state_source_ns,self.scan_source_ns,self.observation_available_ns):
            if type(stamp)is not int or stamp<=0:raise ValueError('integer source time required')
        if not self.observation_available_ns<=self.decision_source_ns:raise ValueError('decision predates available observation')
        for source in (self.actual_state_source_ns,self.scan_source_ns):
            if not 0<=self.decision_source_ns-source<=200000000:raise ValueError('future/stale decision source')
        if len(self.history_source_ns)!=6 or any(type(t)is not int or t<=0 or t>self.scan_source_ns for t in self.history_source_ns) or any(b<=a for a,b in zip(self.history_source_ns,self.history_source_ns[1:])):raise ValueError('six causal unique real scans required')
        validate_policy_observation(self.observation);digest(self.sensor_receipt_sha256);digest(self.candidate_semantics_sha256)
        return self


@dataclass(frozen=True)
class PrivilegedCriticState:
    """Training process only. Three actors x12 + shared12 =48; no policy cast."""
    decision_id:str
    source_ns:int
    values:Tuple[float,...]
    source_binding_sha256:str
    def validate(self,context):
        if self.decision_id!=context.decision_id or type(self.source_ns)is not int or not 0<=context.decision_source_ns-self.source_ns<=200000000:raise ValueError('privilege join future/stale/wrong decision')
        if len(self.values)!=48 or not all(math.isfinite(v) for v in self.values):raise ValueError('critic-only48 schema required')
        digest(self.source_binding_sha256);return self


@dataclass(frozen=True)
class ProposalReceipt:
    decision_id:str
    sampled_latent_action:Optional[int]
    sampled_log_probability:Optional[float]
    post_advantage_gate_action:int
    actor_completed_within20ms:bool
    nonempty_mask:bool
    behavior_checkpoint_sha256:str
    def policy_gradient_eligible(self):
        return self.actor_completed_within20ms and self.nonempty_mask and self.sampled_latent_action is not None
    def validate(self,context):
        if self.decision_id!=context.decision_id:raise ValueError('proposal context mismatch')
        if type(self.post_advantage_gate_action)is not int or not 0<=self.post_advantage_gate_action<8:raise ValueError('post-gate action required')
        digest(self.behavior_checkpoint_sha256)
        actual_nonempty=any(context.observation['action_mask'])
        if type(self.nonempty_mask)is not bool or self.nonempty_mask!=actual_nonempty:raise ValueError('mask receipt mismatch')
        if self.policy_gradient_eligible():
            a=self.sampled_latent_action
            if type(a)is not int or not 0<=a<8 or not context.observation['action_mask'][a]:raise ValueError('sampled action outside behavior mask')
            if self.sampled_log_probability is None or not math.isfinite(self.sampled_log_probability) or self.sampled_log_probability>0:raise ValueError('latent sampled log-probability required')
        return self


class EnvironmentBackend(Protocol):
    backend_kind:str
    def reset(self,spec:EpisodeSpec)->DecisionContext:...
    def step(self,proposal:ProposalReceipt)->Mapping[str,Any]:
        """Returns all20Hz control receipts, actual delta_ns, reward/outcome and next context.
        Gazebo alone supplies actual sensor/physical transitions. No action-logprob relabel.
        """
        ...
    def critic_state(self,context:DecisionContext)->PrivilegedCriticState:...


def require_interactive_backend(backend_kind):
    if backend_kind=='sensor_replay':raise RuntimeError('replay cannot produce a counterfactual environment.step for a new action')
    if backend_kind not in ('gazebo_train','fast_proxy'):raise ValueError('declared backend required')
    return backend_kind
