"""New M08 actor namespace and bounded asynchronous inference; no ROS/training."""
import copy
import io
import threading
import time

FORMAT='astra_route_tracks_sensor_actor_m08_v1'
OBSERVATION_SCHEMA='astra_route_tracks_observation_m08_v1'
FEATURE_SHAPES={'scans':[6,72],'robot':[10],'candidates':[8,7],'analytic_prior':[8],'action_mask':[8]}


class ActorDeadlineExceeded(RuntimeError):pass


class DeadlineActor:
    """One daemon worker, no unbounded queue and no publication side effects.

    Expired job output lives only in that job's private cell; it cannot become
    the result of a later request. A busy worker causes immediate fallback.
    CPU/GIL scheduling is still covered by the measured total transaction gate.
    """
    def __init__(self,actor,deadline_ms=20.):
        if deadline_ms!=20.:raise ValueError('frozen20ms actor deadline required')
        self.actor=actor;self.deadline_s=.020;self.lock=threading.Lock();self.wake=threading.Event();self.job=None;self.busy=False
        self.worker=threading.Thread(target=self._work,daemon=True,name='m08_actor_only');self.worker.start()

    def _work(self):
        while True:
            self.wake.wait();self.wake.clear()
            with self.lock:job=self.job
            if job is None:continue
            try:job['result']=self.actor(job['observation'])
            except Exception as exc:job['error']=exc
            finally:
                job['finished_perf_ns']=time.perf_counter_ns()
                with self.lock:self.busy=False;self.job=None
                job['done'].set()

    def __call__(self,observation):
        begin=time.perf_counter_ns()
        job={'observation':copy.deepcopy(observation),'done':threading.Event()}
        with self.lock:
            if self.busy:raise ActorDeadlineExceeded('previous inference still busy; expired output ignored')
            self.busy=True;self.job=job
        self.wake.set()
        remaining=max(0.,self.deadline_s-(time.perf_counter_ns()-begin)/1e9)
        if not job['done'].wait(remaining) or (job['finished_perf_ns']-begin)/1e6>20.:
            raise ActorDeadlineExceeded('inference exceeded20ms; private result discarded')
        if 'error' in job:raise job['error']
        return job['result']


def validate_metadata(metadata,core):
    if not isinstance(metadata,dict) or metadata.get('observation_schema')!=OBSERVATION_SCHEMA or metadata.get('candidate_semantics_sha256')!=core.semantics_sha256 or metadata.get('same_generator_predictor_verifier_used_in_training')is not True or metadata.get('feature_shapes')!=FEATURE_SHAPES:
        raise ValueError('new M08 semantics/checkpoint required; old route/R2 weights rejected')


def load_route_tracks_actor(blob,core,metadata):
    validate_metadata(metadata,core)
    import torch
    from v5_e2e_scope_rl.asymmetric_ppo import SensorActor,observations_to_torch,stack_observations
    torch.set_num_threads(2)
    checkpoint=torch.load(io.BytesIO(blob),map_location='cpu',weights_only=True)
    if not isinstance(checkpoint,dict) or checkpoint.get('format')!=FORMAT:raise ValueError('new M08 actor artifact required')
    validate_metadata(checkpoint,core)
    provenance=checkpoint.get('training_provenance',{})
    if not provenance.get('training_run_id') or provenance.get('candidate_semantics_sha256')!=core.semantics_sha256:raise ValueError('M08 training provenance absent')
    actor=SensorActor.from_actor_checkpoint(checkpoint['actor']);actor.eval()
    if (actor.config.robot_features,actor.config.candidate_features)!=(10,7):raise ValueError('feature dimensions differ')
    def choose(observation):
        with torch.no_grad():
            actions,_=actor.choose_actions(observations_to_torch(stack_observations([observation]),torch.device('cpu')),deterministic=True,advantage_gate_margin=.10)
        return int(actions[0])
    return choose
