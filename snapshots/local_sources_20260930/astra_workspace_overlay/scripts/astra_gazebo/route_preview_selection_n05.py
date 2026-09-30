"""N05 analytic selection factor; frozen route generator/verifier stay inherited.

The genuine backup action_mask is never changed. Only analytic ranking eligibility
of moving actions additionally requires the ORIGINAL unrounded full-preview
verify result >= 0. This is a model-level diagnostic, not a safety certificate.
No actor, training, ROS, simulator truth or source mutation is implemented here.
"""
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import numpy as np
import route_candidate_core as legacy

METHOD='astra_analytic_preview_eligible_n05'
OBSERVATION_SCHEMA='astra_route_preview_selection_observation_n05_v1'
SELECTION_SCHEMA='astra_analytic_preview_eligibility_n05_v1'


@dataclass(frozen=True)
class Selection:
    chosen:int
    proposed:object
    reason:object
    backup_mask:tuple
    selection_eligibility:tuple
    full_preview_clearances_m:tuple
    original_analytic:int

    def legacy_result(self):
        return self.chosen,self.proposed,self.reason,0.

    def audit(self):
        return {'schema':SELECTION_SCHEMA,'method':METHOD,'chosen':self.chosen,
                'proposed':self.proposed,'reason':self.reason,
                'backup_mask':list(self.backup_mask),
                'selection_eligibility':list(self.selection_eligibility),
                'full_preview_clearances_m':list(self.full_preview_clearances_m),
                'original_analytic':self.original_analytic,
                'true_backup_mask_changed':False,'physical_safety_certified':False}


def select_preview_eligible(observation,full_preview_clearances_m):
    """Rank by the existing prior; do not normalize/reweight/mutate inputs.

    Full clearances are separate unrounded verify outputs, never reconstructed
    from float32 candidate features. Stop index0 keeps only its original backup
    eligibility, even if its full-preview value is negative. Ties choose the
    lowest action index exactly as the inherited numpy argmax.
    """
    mask=np.asarray(observation['action_mask']);prior=np.asarray(observation['analytic_prior'])
    full=np.asarray(full_preview_clearances_m)
    if mask.shape!=(8,) or mask.dtype!=np.bool_:raise ValueError('exact eight boolean backup mask required')
    if prior.shape!=(8,) or full.shape!=(8,) or prior.dtype.kind not in 'fi' or full.dtype.kind not in 'fi':raise ValueError('eight numeric prior and unrounded preview values required')
    if not np.all(np.isfinite(prior)) or not np.all(np.isfinite(full)):raise ValueError('finite selection values required')
    eligible=mask.copy();eligible[1:] &= full[1:]>=0.
    original=int(np.argmax(np.where(mask,prior,-np.inf))) if np.any(mask) else 0
    if np.any(eligible):
        chosen=int(np.argmax(np.where(eligible,prior,-np.inf)));proposed=chosen;reason=None
    else:
        chosen=0;proposed=None
        reason='no_prediction_model_backup_pass' if not np.any(mask) else 'no_full_preview_eligible_action'
    return Selection(chosen,proposed,reason,tuple(map(bool,mask)),tuple(map(bool,eligible)),tuple(map(float,full)),original)


class PreviewEligibleRuntime(legacy.RouteAwareRuntime):
    """Optional integration wrapper; no standalone node or execution authority.

    Capture the existing build's alternating preview/backup verify returns, so
    there are no second rollouts/queries and no precision-losing reconstruction.
    A single caller must serialize build/command as in the original ROS node.
    Fail closed if the inherited build's verified call protocol ever changes.
    """
    def __init__(self,grid,route,urdf_xml,config=None,actor=None,checkpoint_metadata=None):
        if actor is not None or checkpoint_metadata is not None:raise ValueError('N05 is analytic-only; no checkpoint or learned actor accepted')
        self._n05_capture=None;self._n05_expected_calls=None
        super().__init__(grid,route,urdf_xml,config=config)
        parent=dict(self.identity);parent_sha=self.semantics_sha256
        self.identity=dict(parent,schema=OBSERVATION_SCHEMA,method=METHOD,
            parent_observation_schema=legacy.OBSERVATION_SCHEMA,parent_semantics_sha256=parent_sha,
            selection_schema=SELECTION_SCHEMA,
            selection_rule='unchanged backup mask; moving selection requires original full_preview>=0; original prior including stop and lowest-index ties',
            candidate_arrays_numerically_unchanged=True,learned_actor_supported=False,
            selection_adapter_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        self.semantics_sha256=hashlib.sha256(json.dumps(self.identity,sort_keys=True,separators=(',',':')).encode()).hexdigest()

    def rollout(self,state,action,backup=False):
        states=super().rollout(state,action,backup)
        if self._n05_capture is not None:self._n05_expected_calls.append((id(states),action,backup))
        return states

    def verify(self,states,prediction,now_ns):
        value=super().verify(states,prediction,now_ns)
        if self._n05_capture is not None:
            if not self._n05_expected_calls:raise RuntimeError('inherited verify not paired with original rollout')
            state_id,action,backup=self._n05_expected_calls.pop(0)
            if state_id!=id(states):raise RuntimeError('inherited preview/backup verification order changed')
            self._n05_capture.append((action,backup,float(value)))
        return value

    def build(self,actual,fresh_scan,history,elapsed_s,now_ns):
        if self._n05_capture is not None:raise RuntimeError('N05 build must be serialized')
        self._n05_capture=[];self._n05_expected_calls=[]
        try:
            built=super().build(actual,fresh_scan,history,elapsed_s,now_ns)
            captured=tuple(self._n05_capture)
            if self._n05_expected_calls or [(a,b) for a,b,_ in captured]!=[(a,b) for a in range(8) for b in (False,True)]:raise RuntimeError('inherited build verification protocol changed')
            if [value for _,backup,value in captured if backup]!=list(built['backup_clearances']):raise RuntimeError('captured backup values differ from inherited output')
            full=[value for _,backup,value in captured if not backup]
            decision=select_preview_eligible(built['observation'],full)
            built['full_preview_clearances_m']=full
            built['preview_selection_audit']=decision.audit()
            return built
        finally:
            self._n05_capture=None;self._n05_expected_calls=None

    def select(self,built,clock_ns=None):
        if self.actor is not None:raise ValueError('N05 actor cannot be installed after construction')
        return select_preview_eligible(built['observation'],built['full_preview_clearances_m']).legacy_result()
