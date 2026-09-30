"""Strict new route-observation actor artifact; legacy R2 weights stay distinct.

The network architecture is reused, not its historical observation semantics.
Packaging is an interface assertion, not evidence that training occurred or that
learned benefit/calibration is established; release review checks provenance.
"""
import io

FORMAT='astra_route_sensor_actor_v1'
FEATURE_SHAPES={'scans':[6,72],'robot':[10],'candidates':[8,7],'analytic_prior':[8],'action_mask':[8]}


def validate_metadata(metadata,core):
    if not isinstance(metadata,dict) or metadata.get('observation_schema')!='astra_route_observation_v1' or metadata.get('candidate_semantics_sha256')!=core.semantics_sha256 or metadata.get('same_generator_predictor_verifier_used_in_training')is not True:
        raise ValueError('new route-observation checkpoint provenance required; frozen R2 checkpoint cannot be relabelled')
    if metadata.get('feature_shapes')!=FEATURE_SHAPES:raise ValueError('new feature schema differs')


def package_route_actor(actor,core,training_provenance):
    """Called by a future trainer only after using this exact pure core."""
    if not isinstance(training_provenance,dict) or not training_provenance.get('training_run_id') or training_provenance.get('candidate_semantics_sha256')!=core.semantics_sha256:
        raise ValueError('actual training provenance required')
    return {'format':FORMAT,'observation_schema':'astra_route_observation_v1','candidate_semantics_sha256':core.semantics_sha256,
            'feature_shapes':FEATURE_SHAPES,'training_provenance':dict(training_provenance),'actor':actor.actor_checkpoint()}


def load_route_actor(blob,core,metadata):
    """Load already SHA-bound bytes; no ROS, no simulator truth, CPU inference."""
    validate_metadata(metadata,core)
    import torch
    from v5_e2e_scope_rl.asymmetric_ppo import SensorActor,observations_to_torch,stack_observations
    torch.set_num_threads(2)
    checkpoint=torch.load(io.BytesIO(blob),map_location='cpu',weights_only=True)
    if not isinstance(checkpoint,dict) or checkpoint.get('format')!=FORMAT or checkpoint.get('observation_schema')!='astra_route_observation_v1' or checkpoint.get('candidate_semantics_sha256')!=core.semantics_sha256 or checkpoint.get('feature_shapes')!=FEATURE_SHAPES:
        raise ValueError('old or mismatched actor artifact cannot use new route schema')
    provenance=checkpoint.get('training_provenance',{})
    if not provenance.get('training_run_id') or provenance.get('candidate_semantics_sha256')!=core.semantics_sha256:raise ValueError('missing artifact training provenance')
    actor=SensorActor.from_actor_checkpoint(checkpoint['actor']);actor.eval()
    if (actor.config.robot_features,actor.config.candidate_features)!=(10,7):raise ValueError('actor dimensions mismatch')
    def choose(observation):
        with torch.no_grad():
            actions,_=actor.choose_actions(observations_to_torch(stack_observations([observation]),torch.device('cpu')),deterministic=True,advantage_gate_margin=.10)
        return int(actions[0])
    return choose
