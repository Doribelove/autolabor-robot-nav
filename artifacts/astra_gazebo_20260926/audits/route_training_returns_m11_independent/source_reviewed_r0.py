"""Source-time reward/GAE candidate for later Gazebo training, never a simulator.

The caller supplies actual joined measurements and checkpoint critic values.
No rollout prediction is treated as a physical transition. All-false-mask and
deadline-drop transitions retain their rewards and value targets; policy-gradient
eligibility is a separate mask and does not sever earlier temporal credit.
This component grants no training authority and does not choose a step budget.
"""
from dataclasses import dataclass
import math

SCHEMA = 'astra_source_time_returns_m11_v1'
OUTCOMES = ('running', 'goal', 'contact', 'timeout')

def finite(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('finite numeric '+label+' required')
    return float(value)

def timestamp(value):
    if type(value) is not int or value <= 0:
        raise ValueError('positive integer source nanoseconds required')
    return value

@dataclass(frozen=True)
class RewardTerms:
    """Candidate coefficients; a future training release must bind this recipe."""
    progress_per_m: float = 1.
    elapsed_per_s: float = -.30
    absolute_speed_change: float = -.01
    executed_switch: float = -.005
    goal: float = 8.
    contact: float = -30.
    timeout: float = -4.

    def validate(self):
        for name, value in vars(self).items():finite(value, name)
        return self

def physical_reward(*, start_ns, end_ns, route_arc_start_m,
                    route_arc_end_m, measured_forward_speed_start_mps,
                    measured_forward_speed_end_mps, executed_action_before,
                    executed_actions, outcome, terms=RewardTerms()):
    """Reward over one actual interval. No fabricated sensor/physics transitions.

    executed_actions is an ordered (control_source_ns, executed_candidate_index)
    stream in (start,end]. It includes zero/drop commands. A persisted preceding
    executed_action_before carries switch counting across decision boundaries.
    This is the executed candidate index, never the actor log-probability label.
    Arc/speed endpoints must be joined to start/end before calling this function.
    Contact takes priority if goal and contact share the terminal source instant;
    caller must supply the mutually exclusive resolved outcome and its evidence.
    """
    timestamp(start_ns);timestamp(end_ns);terms.validate()
    if end_ns <= start_ns:raise ValueError('positive actual transition duration required')
    if outcome not in OUTCOMES:raise ValueError('infrastructure failure is not a fabricated physical terminal')
    scalars=[finite(v,n) for v,n in zip((route_arc_start_m,route_arc_end_m,
        measured_forward_speed_start_mps,measured_forward_speed_end_mps),('start arc','end arc','start speed','end speed'))]
    if type(executed_action_before) is not int or not 0<=executed_action_before<8:
        raise ValueError('preceding actual action receipt required')
    previous=executed_action_before;last=start_ns;switches=0
    for stamp,action in executed_actions:
        timestamp(stamp)
        if not last < stamp <= end_ns:raise ValueError('unique ordered actual control sources within transition required')
        if type(action) is not int or not 0<=action<8:raise ValueError('actual executed candidate index required')
        switches += int(action != previous);previous=action;last=stamp
    dt=(end_ns-start_ns)/1e9
    pieces=dict(progress=terms.progress_per_m*(scalars[1]-scalars[0]),
        elapsed=terms.elapsed_per_s*dt,
        speed_change=terms.absolute_speed_change*abs(scalars[3]-scalars[2]),
        executed_switch=terms.executed_switch*switches,
        terminal=0. if outcome=='running' else getattr(terms,outcome))
    for key,value in pieces.items():finite(value,'computed reward term '+key)
    reward=finite(sum(pieces.values()),'computed reward')
    return dict(schema=SCHEMA,reward=reward,terms=pieces,
        elapsed_s=dt,executed_switch_count=switches,last_executed_action=previous,
        outcome=outcome,terminal=outcome!='running')

@dataclass(frozen=True)
class Transition:
    episode_id: str
    decision_id: str
    start_ns: int
    end_ns: int
    reward: float
    value: float
    next_value: float
    outcome: str
    policy_gradient_eligible: bool
    # A collection cut has a real next observation/value but no next recorded
    # transition in this contiguous rollout segment. It is not an environment
    # termination. Infrastructure failure before a valid end has NO Transition.
    collection_cut: bool = False

    def validate(self):
        if not self.episode_id or not self.decision_id:raise ValueError('explicit episode/decision IDs required')
        timestamp(self.start_ns);timestamp(self.end_ns)
        if self.end_ns<=self.start_ns:raise ValueError('positive source interval required')
        for n in ('reward','value','next_value'):finite(getattr(self,n),n)
        if self.outcome not in OUTCOMES:raise ValueError('unknown or infrastructure outcome cannot supply a physical transition')
        if type(self.policy_gradient_eligible) is not bool or type(self.collection_cut) is not bool:
            raise ValueError('strict boolean eligibility and cut flags required')
        if self.outcome!='running' and self.collection_cut:raise ValueError('terminal is not also a collection cut')
        if self.outcome!='running' and self.next_value!=0.:
            raise ValueError('finite-horizon terminal must explicitly have zero next value')
        return self

def source_time_gae(transitions, *, gamma_per_reference, lambda_per_reference,
                    reference_duration_s):
    """Semi-Markov GAE without dropping non-PG transitions.

    delta_i = r_i + gamma**(dt_i/ref) V(next_i) - V_i
    A_i = delta_i + (gamma*lambda)**(dt_i/ref) A_(i+1)
    Physical terminal and explicit collection cuts stop the recursion. A cut
    keeps its authenticated next value in delta. Missing/noncontiguous segments
    are rejected unless the preceding row explicitly closes a segment. Values
    are detached numeric inputs from the behavior critic; fitting occurs later.
    """
    gamma=finite(gamma_per_reference,'gamma');lam=finite(lambda_per_reference,'lambda')
    ref=finite(reference_duration_s,'reference duration')
    if not 0<gamma<=1 or not 0<=lam<=1 or ref<=0:raise ValueError('valid discounts/reference duration required')
    rows=tuple(t.validate() for t in transitions)
    if len({(t.episode_id,t.decision_id) for t in rows})!=len(rows):raise ValueError('duplicate decision identity')
    completed=set();last_end={}
    for i,t in enumerate(rows):
        if t.episode_id in completed:raise ValueError('episode identity reused after physical terminal')
        if t.start_ns<last_end.get(t.episode_id,t.start_ns):raise ValueError('episode source intervals overlap')
        last_end[t.episode_id]=t.end_ns
        if t.outcome!='running':completed.add(t.episode_id)
        closed=t.outcome!='running' or t.collection_cut
        if not closed:
            if i==len(rows)-1:raise ValueError('open rollout tail needs explicit cut and real bootstrap')
            n=rows[i+1]
            if n.episode_id!=t.episode_id or n.start_ns!=t.end_ns:
                raise ValueError('unacknowledged missing source interval')
            if n.value!=t.next_value:raise ValueError('adjacent behavior critic values disagree')
        elif i+1<len(rows) and rows[i+1].episode_id==t.episode_id:
            n=rows[i+1]
            if t.outcome!='running':raise ValueError('episode identity reused after physical terminal')
            if n.start_ns<t.end_ns:raise ValueError('collection segments overlap')
    result=[None]*len(rows);following=0.
    for i in reversed(range(len(rows))):
        t=rows[i];exponent=(t.end_ns-t.start_ns)/1e9/ref
        discount=gamma**exponent
        delta=t.reward+discount*t.next_value-t.value
        continuation=0. if t.outcome!='running' or t.collection_cut else (gamma*lam)**exponent*following
        advantage=finite(delta+continuation,'computed advantage');following=advantage
        target=finite(advantage+t.value,'computed value target')
        result[i]=dict(episode_id=t.episode_id,decision_id=t.decision_id,
            advantage=advantage,value_target=target,
            policy_gradient_eligible=t.policy_gradient_eligible,
            discount=discount,source_duration_ns=t.end_ns-t.start_ns)
    return result
