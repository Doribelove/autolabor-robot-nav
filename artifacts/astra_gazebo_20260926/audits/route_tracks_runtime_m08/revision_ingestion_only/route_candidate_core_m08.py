"""New sensor-track route core. Inherits exact v1 actions/rollout/physical model.

The vector layout is unchanged; clearance/prior/mask semantics are stronger,
so a new observation/checkpoint identity is mandatory. No learned interaction
predictor is fitted here and no backup result is a physical safety certificate.
"""
from collections import OrderedDict
from dataclasses import asdict
import copy
import hashlib
import json
import math
from pathlib import Path
import time
import numpy as np
import route_candidate_core as legacy
from route_candidate_core import RouteConfig,ACTION_NAMES,SPEEDS,OFFSETS,physical_backend,prepublish_guard
from route_world_adapter import wrap_angle
from axle_reference_adapter import rear_twist_to_base
from sensor_tracks_v1 import TrackConfig
from sensor_tracks_v2 import SensorTracks,ConvexFootprint,BatchHardOccupancy,prediction_request
from sensor_tracks_ingestion_m08 import IngestionSensorTracks
from route_tracks_actor_adapter_m08 import OBSERVATION_SCHEMA,FEATURE_SHAPES,DeadlineActor,ActorDeadlineExceeded,validate_metadata

METHODS=('astra_analytic_route_tracks_m08','astra_pct_route_tracks_m08')


class RouteTracksRuntime(legacy.RouteAwareRuntime):
    def __init__(self,grid,route,urdf_xml,config=None,actor=None,checkpoint_metadata=None):
        super().__init__(grid,route,urdf_xml,config,None,None)
        # Shared protocol upper span: activation5s + warmup cap12s + formal22s.
        # Forty seconds preserves retained supports through that entire window.
        self.tracker=IngestionSensorTracks(grid,TrackConfig(maximum_episode_span_ns=40_000_000_000))
        self.batch_footprint=ConvexFootprint(self.footprint.vertices);self.static_cache=OrderedDict()
        self.projection_cache=OrderedDict()
        self.identity=copy.deepcopy(self.identity)
        self.identity.update(schema=OBSERVATION_SCHEMA,parent_observation_schema=legacy.OBSERVATION_SCHEMA,
            parent_core_sha256=self.identity['core_sha256'],core_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            dynamic_predictor='sensor_tracks_v2_retained_supports_uncalibrated_no_learned_interaction',
            tracking_config=asdict(self.tracker.config),new_semantics=['clearance_feature','analytic_prior','backup_mask'],
            action_and_rollout_functions_inherited_exactly=True,feature_layout_unchanged=True,
            route_projection='exact parent calculation with bounded4096 exactXY memoization; no quantization',
            ingestion='v1 exact accept body; KD broad-phase with original threshold predicate in1e-12m boundary band; v2 availability guards',
            actor_deadline_ms=20.,transaction_deadline_ms=50.,physical_safety_certified=False,
            learned_candidate_conditioned_prediction_implemented=False,
            certified_stop_claim=False)
        self.identity['m08_source_sha256']={n:hashlib.sha256(Path(__file__).with_name(n).read_bytes()).hexdigest() for n in
            ('sensor_tracks_v1.py','sensor_tracks_v2.py','sensor_tracks_ingestion_m08.py','route_tracks_actor_adapter_m08.py')}
        self.semantics_sha256=hashlib.sha256(json.dumps(self.identity,sort_keys=True,separators=(',',':')).encode()).hexdigest()
        if actor is not None:
            validate_metadata(checkpoint_metadata,self)
            self.actor=DeadlineActor(actor,self.config.inference_deadline_ms)

    def accept_scan(self,frame,tf_stamp_ns,odom_state,now_ns):
        return self.tracker.accept(frame,tf_stamp_ns=tf_stamp_ns,odom_state=odom_state,now_ns=now_ns)

    def project(self,xy):
        key=tuple(xy)
        if key in self.projection_cache:
            value=self.projection_cache.pop(key);self.projection_cache[key]=value
        else:
            value=super().project(xy);self.projection_cache[key]=value
            if len(self.projection_cache)>4096:self.projection_cache.popitem(last=False)
        return dict(value)

    def predict(self,fresh_scan,now_ns=None):
        snapshot=self.tracker.snapshot
        if snapshot is None or snapshot.source_ns!=fresh_scan.stamp_ns:raise ValueError('current fresh scan not accepted by exactTF tracker')
        if now_ns is None:now_ns=snapshot.accepted_now_ns
        occupancy=self.tracker.batch_occupancy(now_ns)
        return dict(occupancy=occupancy,stamp_ns=snapshot.source_ns,accepted_now_ns=snapshot.accepted_now_ns,
                    dynamic_count=occupancy.support_count,model='retained observed supports +1.35m/s expansion; learned candidate conditioning pending',
                    track_count=len(snapshot.tracks),sparse_cells=len(snapshot.sparse_supports),
                    ambiguous_tracks=sum(t.ambiguous for t in snapshot.tracks),receipt_sha256=snapshot.receipt_sha256,
                    physical_safety_certified=False,learned_candidate_conditioned_prediction_implemented=False)

    def _static_at(self,pose):
        key=tuple(pose)
        if key in self.static_cache:
            value=self.static_cache.pop(key);self.static_cache[key]=value;return value
        shape=self.footprint.at(pose)
        value=shape.distance(self.static) if self.domain.contains(shape) else -1.
        self.static_cache[key]=value
        if len(self.static_cache)>4096:self.static_cache.popitem(last=False)
        return value

    def verify_many(self,trajectories,prediction,now_ns):
        states=[s for rows in trajectories for s in rows]
        if not states:raise ValueError('nonempty trajectory queries required')
        poses=np.asarray([s['base'] for s in states],dtype=float)
        # Outward integer conversion cannot shorten the old floating horizon.
        offsets=np.ceil(np.asarray([s['t'] for s in states])*1e9).astype(np.int64)
        dynamic=prediction['occupancy'].distance_lower_bounds(self.batch_footprint,poses,offsets)
        static=np.asarray([self._static_at(s['base']) for s in states])
        values=np.minimum(static,dynamic)-self.config.margin_m
        results=[];start=0
        for rows in trajectories:
            v=values[start:start+len(rows)].copy();start+=len(rows)
            for i,(a,b) in enumerate(zip(rows,rows[1:])):
                motion=math.dist(a['base'][:2],b['base'][:2])+self.footprint.radius*abs(wrap_angle(b['base'][2]-a['base'][2]))
                moving=0. if prediction['dynamic_count']==0 else self.config.dynamic_speed_bound_mps*(b['t']-a['t'])
                v[i]=min(v[i],v[i+1])-.5*(motion+moving)
            results.append(float(np.min(v)))
        return results

    def verify(self,states,prediction,now_ns):
        return self.verify_many([states],prediction,now_ns)[0]

    def build(self,actual,fresh_scan,history,elapsed_s,now_ns):
        self.validate_inputs(actual,fresh_scan,now_ns)
        if np.shape(history)!=(6,72) or not np.all(np.isfinite(history)):raise ValueError('six real finite model-history rows required')
        state=self.state(actual);prediction=self.predict(fresh_scan,now_ns)
        previews=[self.rollout(state,a) for a in range(8)];backups=[self.rollout(state,a,True) for a in range(8)]
        full=self.verify_many(previews+backups,prediction,now_ns);preview_clearances=full[:8];safe=full[8:]
        current=self.project(state['base'][:2]);progress=current['arc_m']/self.route.length;features=[];prior=[]
        for action,preview,clearance in zip(range(8),previews,preview_clearances):
            end=self.project(preview[-1]['base'][:2]);gain=end['arc_m']-current['arc_m'];switch=float(action!=self.previous_action)
            features.append([SPEEDS[action]/1.2,OFFSETS[action]/1.35,gain/3.6,np.clip(clearance,-3,3)/3,abs(SPEEDS[action]-state['vx'])/1.2,abs(end['lateral_m'])/1.35,switch])
            prior.append(gain+1.5*np.clip(clearance,-1,2)-.15*switch-.05*abs(end['lateral_m']))
        mask=[value>=0. for value in safe]
        observation={'scans':np.asarray(history,dtype=np.float32)/8.,'robot':np.asarray([progress,current['lateral_m']/2.35,wrap_angle(state['base'][2]-current['heading_rad'])/.7,state['vx']/1.2,SPEEDS[self.previous_action]/1.2,(self.route.length-current['arc_m'])/9.,self.previous_action/7.,min(elapsed_s/self.config.maximum_episode_duration_s,1.),min(self.stop_count,12)/12.,float(np.min(fresh_scan.model_ranges))/8.],dtype=np.float32),
                     'candidates':np.asarray(features,dtype=np.float32),'analytic_prior':np.asarray(prior,dtype=np.float32),'action_mask':np.asarray(mask,dtype=bool)}
        return dict(observation=observation,previews=previews,backups=backups,backup_clearances=safe,preview_clearances=preview_clearances,
                    prediction_audit={k:v for k,v in prediction.items() if k!='occupancy'},mask=mask,
                    query_count=sum(map(len,previews+backups)),state_source_ns=actual['stamp_ns'],scan_source_ns=fresh_scan.stamp_ns)

    def select(self,built,clock_ns=time.perf_counter_ns):
        observation=built['observation'];mask=observation['action_mask']
        if not np.any(mask):return 0,None,'no_prediction_model_backup_pass',0.
        analytic=int(np.argmax(np.where(mask,observation['analytic_prior'],-np.inf)))
        if self.actor is None:return analytic,analytic,None,0.
        begin=clock_ns()
        try:proposed=int(self.actor(observation))
        except ActorDeadlineExceeded:return analytic,None,'inference_deadline_drop',(clock_ns()-begin)/1e6
        except Exception as exc:return analytic,None,'inference_error:'+type(exc).__name__,(clock_ns()-begin)/1e6
        latency=(clock_ns()-begin)/1e6
        if latency>self.config.inference_deadline_ms:return analytic,proposed,'inference_deadline_drop',latency
        if not 0<=proposed<8 or not mask[proposed]:return analytic,proposed,'masked_action_drop',latency
        return proposed,proposed,None,latency

    def future_prediction_request(self,built,real_bucket_history,decision_ns):
        """Actual shared rollouts for future training/calibration, never a model.

        This explicit export is not called inside the live hard verifier. The
        snapshot/receipt must still be the one used to generate these features.
        No old actor or CV diagnostic may replace the retained hard occupancy.
        """
        snapshot=self.tracker.snapshot
        if snapshot is None or built['prediction_audit']['receipt_sha256']!=snapshot.receipt_sha256:
            raise ValueError('candidate observation snapshot changed')
        if type(built['state_source_ns'])is not int or not 0<=decision_ns-built['state_source_ns']<=200_000_000:
            raise ValueError('candidate actual state is future or stale')
        rows=[dict(candidate_id=ACTION_NAMES[i],states=[dict(offset_ns=int(np.ceil(s['t']*1e9)),
            base_pose=s['base'],rear_pose=s['rear'],body_twist=rear_twist_to_base((s['vx'],s['vy'],s['w']),self.config.wheelbase_m),
            center_steering=s['steer']) for s in states]) for i,states in enumerate(built['previews'])]
        request=prediction_request(snapshot,real_bucket_history,rows,decision_ns=decision_ns)
        return dict(request=request,candidate_semantics_sha256=self.semantics_sha256,actual_state_source_ns=built['state_source_ns'],
                    reference_semantics='base pose/twist and rear pose; initial actual sample not extrapolated to decision time',
                    learned_model_fitted=False,hard_occupancy_replacement_authorized=False)

    def command(self,actual,fresh_scan,action,now_ns):
        self.validate_inputs(actual,fresh_scan,now_ns)
        if type(action)is not int or not 0<=action<8:raise ValueError('declared action ID required')
        state=self.state(actual);prediction=self.predict(fresh_scan,now_ns);backup=self.rollout(state,action,True)
        clearance=self.verify(backup,prediction,now_ns)
        if clearance<0:return 0.,0.,0,'backup_revalidation_failed',clearance
        if action==0:return 0.,0.,0,'backup_stop_requested',clearance
        target=self.step(state,action,self.config.control_period_ns/1e9)
        return max(0.,target['vx']),max(0.,target['vx'])*math.tan(target['steer'])/self.config.wheelbase_m,action,None,clearance

    def transaction(self,actual,scan,history,elapsed_s,now_ns,*,decide,committed_action,source_clock,
                    refresh=None,begin_perf_ns=None,clock_ns=time.perf_counter_ns):
        begin=clock_ns() if begin_perf_ns is None else begin_perf_ns
        built=None;proposed=None;chosen=committed_action;selection_reason=None;inference_ms=0.;command_reason=None
        if decide:
            built=self.build(actual,scan,history,elapsed_s,now_ns)
            if (clock_ns()-begin)/1e6<=self.config.transaction_deadline_ms:
                chosen,proposed,selection_reason,inference_ms=self.select(built,clock_ns)
        command_now=source_clock()
        if refresh is not None:actual,scan,command_now=refresh()
        health_probe=prepublish_guard((0.,0.,0,None,None),command_now,actual['stamp_ns'],scan.stamp_ns,0.,self.config)
        if command_now<now_ns or command_now<self.tracker.snapshot.accepted_now_ns:
            command=(0.,0.,0,'prepublish_source_regression_drop',None)
        elif health_probe[3] is not None:
            command=health_probe
        elif (clock_ns()-begin)/1e6>self.config.transaction_deadline_ms:
            command=(0.,0.,0,'transaction_deadline_drop',None)
        else:
            command=self.command(actual,scan,chosen,command_now);command_reason=command[3]
        end=source_clock();duration=(clock_ns()-begin)/1e6
        if end<now_ns or end<command_now or end<self.tracker.snapshot.accepted_now_ns:
            command=(0.,0.,0,'prepublish_source_regression_drop',command[4])
        command=prepublish_guard(command,end,actual['stamp_ns'],scan.stamp_ns,duration,self.config)
        v,w,executed,reason,clearance=command
        self.previous_action=executed;self.stop_count=self.stop_count+1 if executed==0 else 0
        trace=dict(stamp_ns=end,state_source_ns=actual['stamp_ns'],fresh_scan_ns=scan.stamp_ns,
            observation_accepted_ns=self.tracker.snapshot.accepted_now_ns,proposed=proposed,chosen=chosen,executed=executed,v=v,w=w,
            stop_reason=reason or selection_reason,selection_reason=selection_reason,command_stop_reason=command_reason,
            transaction_ms=duration,inference_ms=inference_ms,inference_deadline_dropped=selection_reason=='inference_deadline_drop',
            transaction_deadline_dropped=duration>self.config.transaction_deadline_ms,
            backup_prediction_clearance_m=clearance,backup_stop_model_passed=(executed==0 and reason=='backup_stop_requested' and clearance is not None and clearance>=0.),
            certified_stop=False,physical_safety_certified=False,learned_candidate_conditioned_prediction_implemented=False)
        return dict(command=command,trace=trace,built=built,committed_action=executed)
