#!/usr/bin/env python3
"""Offline, conditional sweep budgets for an explicitly declared ODE extension.

No library-name whitelist qualifies a run. ODE stores discrete states, not an
observed continuous path. This module bounds the extension COM(s)=COM0+s*dCOM,
q(s)=normalize(q0+s*tan(theta/2)*q_perp), s in [0,1]. It is the normal
infinitesimal-quaternion dxStepBody extension, when its assumptions are verified.
End-of-step twist is diagnostic: damping and QuickStep ERP removal can change
velocity AFTER pose integration. Neither sampled maxima nor endpoint velocities
are promoted into all-time physical speed bounds.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from geometry import Pose3, RobotCollisionModel

REQUIRED_PROOFS = (
    'loaded_engine_source_and_binary_verified',
    'normal_infinitesimal_rotation_mode_verified',
    'one_pose_integration_per_dense_step_verified',
    'fixed_body_com_mapping_verified',
    'all_non_engine_pose_writes_classified',
    'all_actor_updates_bounded',
    'numerical_error_envelope_verified',
    'declared_simulator_extension_accepted',
)


def finite(x):
    if isinstance(x, bool):
        raise ValueError('boolean is not numeric evidence')
    x = float(x)
    if not math.isfinite(x):
        raise ValueError('nonfinite numeric evidence')
    return x


def vector(x, n=3):
    if len(x) != n:
        raise ValueError('vector length mismatch')
    return tuple(finite(v) for v in x)


def norm(x):
    return math.sqrt(sum(v*v for v in x))


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def pose(x):
    if isinstance(x, Pose3):
        return x
    x = vector(x, 7)
    if abs(norm(x[3:])-1.) > 1e-7:
        raise ValueError('source quaternion is not unit length')
    return Pose3(x[:3], x[3:])


def extension_point_budget(first, second, com_local, radius_m):
    """Unit-phase Lipschitz budget for every point within R of body COM.

    theta=2*atan2(||q_perp||,|q0.q1|), k=tan(theta/2).
    |d angle/ds|=2*k/(1+k*k*s*s)<=2*k, hence point budget
    B=|COM1-COM0|+2*R*k. B bounds path length AND phase derivative.
    This is a conditional real-arithmetic theorem, not a floating-point proof.
    Near-pi steps are refused instead of aliasing a rotation or overflowing.
    """
    first, second = pose(first), pose(second)
    com_local = vector(com_local); radius_m = finite(radius_m)
    if radius_m < 0:
        raise ValueError('negative shape radius')
    q0, q1 = first.xyzw, second.xyzw
    dot = sum(a*b for a, b in zip(q0, q1))
    if dot < 0:
        q1 = tuple(-x for x in q1); dot = -dot
    dot = min(1., dot)
    # Difference-based angle calculation retains small rotations; acos(dot)
    # alone can round to zero for a small but nonzero wheel motion.
    perpendicular = tuple(b-dot*a for a, b in zip(q0, q1))
    sine = norm(perpendicular)
    if dot < 1e-6:
        raise ValueError('near-pi interval: normal-Euler inverse ill-conditioned')
    k = sine/dot
    theta = 2*math.atan2(sine, dot)
    c0, c1 = first.compose(Pose3(com_local)).xyz, second.compose(Pose3(com_local)).xyz
    dc = tuple(b-a for a, b in zip(c0, c1))
    result = {'translation_m': norm(dc), 'rotation_arc_rad': theta,
              'rotation_phase_derivative_bound_rad': 2*k,
              'shape_radius_from_body_com_m': radius_m,
              'point_phase_budget_m': norm(dc)+2*radius_m*k,
              'com_displacement_world_m': dc,
              'normal_euler_integrating_omega_times_dt': None}
    # World rotation axis of q_perp*q0^-1, using q convention xyzw.
    x, y, z, w = q0; a, b, c, d = perpendicular
    result['normal_euler_integrating_omega_times_dt'] = (
        2*(-d*x+a*w-b*z+c*y)/dot,
        2*(-d*y+a*z+b*w-c*x)/dot,
        2*(-d*z-a*y+b*x+c*w)/dot)
    return result


def robot_body_envelopes(urdf_xml):
    """Aggregate fixed-link masses and collision radii into actual ODE bodies.

    A massless URDF base is NOT assigned zero COM: fixed chassis/sensors merge.
    A future URDF with preserveFixedJoint/disableFixedJointLumping is refused.
    Generated SDF/runtime correspondence remains an explicit proof obligation.
    """
    root = ET.fromstring(urdf_xml)
    if any(root.findall('.//'+name) for name in ('preserveFixedJoint', 'disableFixedJointLumping')):
        raise ValueError('nonstandard fixed joint lumping requires explicit body mapping')
    robot = RobotCollisionModel(urdf_xml)
    assignment = {}
    def assign(link):
        if link in assignment:
            return assignment[link]
        joint = robot.by_child.get(link)
        if joint is None or joint.kind != 'fixed':
            assignment[link] = (link, Pose3())
        else:
            body, origin = assign(joint.parent)
            assignment[link] = (body, origin.compose(joint.origin))
        return assignment[link]
    masses = {}
    for link in root.findall('link'):
        name = link.get('name'); body, transform = assign(name)
        data = masses.setdefault(body, {'mass': 0., 'moment': [0., 0., 0.], 'members': []})
        data['members'].append(name)
        inertial = link.find('inertial')
        if inertial is None:
            continue
        m = finite(inertial.find('mass').get('value'))
        if m <= 0:
            raise ValueError('nonpositive inertial mass')
        origin = inertial.find('origin')
        offset = vector((origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split())
        point = transform.compose(Pose3(offset)).xyz
        data['mass'] += m
        data['moment'] = [a+m*b for a, b in zip(data['moment'], point)]
    result = {}
    for body, data in masses.items():
        if data['mass'] <= 0:
            raise ValueError('surviving body has no positive mass: '+body)
        result[body] = {'mass_kg': data['mass'], 'com_local_m': tuple(x/data['mass'] for x in data['moment']),
                        'members': data['members'], 'radius_m': 0., 'collisions': []}
    for collision in robot.collisions:
        body, transform = assignment[collision.link]
        center = transform.compose(collision.origin).xyz
        displacement = norm(tuple(x-y for x, y in zip(center, result[body]['com_local_m'])))
        if collision.kind == 'box':
            extent = norm(tuple(x/2 for x in collision.dimensions))
        else:
            # Enclose geometry.py's outer48-gon as well as the true cylinder.
            extent = math.hypot(collision.dimensions[0]/math.cos(math.pi/48), collision.dimensions[1]/2)
        result[body]['radius_m'] = max(result[body]['radius_m'], displacement+extent)
        result[body]['collisions'].append(collision.name)
    return result


def index_stream(rows, name, issues):
    indexed = {}; last = None; roster = None
    for number, row in enumerate(rows):
        try:
            stamp = row['stamp_ns']
            if isinstance(stamp, bool) or not isinstance(stamp, int) or stamp <= 0:
                raise ValueError('positive integer source ns required')
            if last is not None and stamp <= last:
                raise ValueError('duplicate, out-of-order or clock reset')
            last = stamp
            if row.get('frame_id') != 'world':
                raise ValueError('world frame required')
            names, poses, twists = row['names'], row['poses'], row['twists']
            if len(set(names)) != len(names) or not len(names) == len(poses) == len(twists):
                raise ValueError('identity/pose/twist arrays mismatch')
            if roster is not None and roster != set(names):
                raise ValueError('body identity roster changed')
            roster = set(names)
            indexed[stamp] = {n: {'pose': pose(p), 'twist': vector(t, 6)} for n, p, t in zip(names, poses, twists)}
        except (ValueError, TypeError, KeyError) as error:
            issues.append(name+' row '+str(number)+': '+str(error))
    return indexed


def assess_dense_trace(model_rows, link_rows, urdf_xml, metadata,
                       actor_speed_bounds=None, step_ns=2000000, robot_name='autolabor_m2', proof_bindings=None):
    """Source-exact per-step motion budgets; never infers hidden all-time speed.

    actor_speed_bounds are separately reviewed max(initial,targets), not measured
    maxima. Their total bounded variation covers discrete SetWorldPose jumps;
    it does not assign a finite instantaneous velocity to those jumps.
    """
    issues = []; bounds = robot_body_envelopes(urdf_xml)
    models = index_stream(model_rows, 'models', issues); links = index_stream(link_rows, 'links', issues)
    if set(models) != set(links):
        issues.append('dense model/link source stamps are not exact matching sets')
    actor_speed_bounds = {name: finite(value) for name, value in (actor_speed_bounds or {}).items()}
    if any(value < 0 for value in actor_speed_bounds.values()):
        raise ValueError('negative actor speed bound')
    if not isinstance(step_ns, int) or isinstance(step_ns, bool) or step_ns <= 0:
        raise ValueError('positive integer expected solver step required')
    if metadata.get('schema') != 'astra_physics_evidence_dense_v2' or metadata.get('all_body_states_every_physics_step') is not True:
        issues.append('dense every-physics-step metadata absent')
    if metadata.get('phase') != 'WorldUpdateEnd' or metadata.get('pose_frame') != 'world':
        issues.append('dense state source phase/frame mismatch')
    if metadata.get('physics_type') != 'ode' or int(round(finite(metadata.get('max_step_s', 0))*1e9)) != step_ns:
        issues.append('ODE source step metadata mismatch')
    stamps = sorted(set(models)&set(links)); rows = []
    for start, end in zip(stamps, stamps[1:]):
        if end-start != step_ns:
            issues.append('missing dense step or changed physics dt: '+str(start)+'..'+str(end))
            continue
        dt = step_ns/1e9; body_rows = {}; robot_budget = 0.; actor_budget = 0.
        try:
            for body, config in bounds.items():
                name = robot_name+'::'+body; first, second = links[start][name], links[end][name]
                item = extension_point_budget(first['pose'], second['pose'], config['com_local_m'], config['radius_m'])
                robot_budget = max(robot_budget, item['point_phase_budget_m'])
                # End twist refers to link origin; convert it to body COM.
                v, omega = second['twist'][:3], second['twist'][3:]
                correction = cross(omega, second['pose'].rotate(config['com_local_m']))
                com_v = tuple(a+b for a, b in zip(v, correction))
                item['end_twist_translation_residual_m'] = norm(tuple(d-dt*v for d, v in zip(item['com_displacement_world_m'], com_v)))
                item['end_twist_rotation_residual_rad'] = norm(tuple(d-dt*v for d, v in zip(item['normal_euler_integrating_omega_times_dt'], omega)))
                item['end_twist_is_used_as_speed_bound'] = False
                body_rows[name] = item
            for name, speed in actor_speed_bounds.items():
                first, second = models[start][name]['pose'], models[end][name]['pose']
                delta = norm(tuple(b-a for a, b in zip(first.xyz, second.xyz)))
                if delta > speed*dt+1e-8:
                    raise ValueError('actor jump exceeds source-rule step bound: '+name)
                if 1-abs(sum(a*b for a, b in zip(first.xyzw, second.xyzw))) > 1e-12:
                    raise ValueError('actor changed orientation without rotation bound: '+name)
                # Sum is conservative even if actor updates occur sequentially
                # within Begin. Actual zero twist cannot justify zero budget.
                actor_budget += speed*dt
            rows.append({'start_ns': start, 'end_ns': end, 'dt_s': dt,
                         'robot_point_phase_budget_m': robot_budget,
                         'actor_total_variation_budget_m': actor_budget,
                         'relative_clearance_variation_budget_m': robot_budget+actor_budget,
                         'body_bounds': body_rows})
        except (KeyError, ValueError, TypeError) as error:
            issues.append('interval '+str(start)+': '+str(error))
    if not rows:
        issues.append('no valid dense interval')
    # Evidence must be a path+SHA reviewed artifact, not True or a plugin name.
    proof_bindings = proof_bindings or {}
    obligations = {}
    for name in REQUIRED_PROOFS:
        binding = proof_bindings.get(name)
        obligations[name] = bool(isinstance(binding, dict) and binding.get('reviewed') is True and
                                 isinstance(binding.get('sha256'), str) and len(binding['sha256']) == 64 and
                                 isinstance(binding.get('path'), str) and binding['path'])
    pending = [name for name, satisfied in obligations.items() if not satisfied]
    # This component does not itself authenticate or authorize proof documents.
    # Even complete binding syntax is reported for independent review, not pass.
    return {'schema': 'astra_dense_sweep_conditional_v1', 'integrity_pass': not issues,
            'issues': sorted(set(issues)), 'source_step_ns': step_ns, 'dense_interval_count': len(rows),
            'body_envelopes': bounds, 'actor_rule_speed_bounds_mps': actor_speed_bounds,
            'conditional_model': 'normal ODE Euler quaternion extension and COM linear extension; bounded actor Begin jumps',
            'qualification': 'pending', 'pending_obligations': pending,
            'qualification_reason': 'This component proves a conditional extension bound; authenticating engine/run correspondence and numeric error envelopes requires a separate reviewed artifact.',
            'proof_binding_syntax_checks': obligations, 'proof_bindings': proof_bindings,
            'proof_documents_authenticated_by_this_component': False,
            'claim': 'conditional simulator-extension point/clearance variation budgets only; no real continuous trajectory observed',
            'formal_safety': False, 'physical_velocity_bounds_inferred_from_sampled_maxima': False}, rows


def aggregate_clearance_bounds(endpoints, intervals):
    """Lift 2ms budgets to endpoint intervals with a bounded-variation proof.

    For every interior configuration, variations to the two endpoints total
    at most P. One is <=P/2, so d>=min(d0,d1)-P/2. Continuity in clock time is
    unnecessary: known actor pose jumps spend bounded variation, not speed.
    A missing solver interval produces no bound, never an interpolated repair.
    """
    indexed = {r['start_ns']: r for r in intervals}
    if len(indexed) != len(intervals):
        raise ValueError('duplicate interval starts')
    result = []
    for first, second in zip(endpoints, endpoints[1:]):
        start, end = first['stamp_ns'], second['stamp_ns']
        if end <= start:
            raise ValueError('endpoint clock is not increasing')
        d0, d1 = finite(first['clearance_m']), finite(second['clearance_m'])
        cursor = start; total = 0.; count = 0
        while cursor < end:
            interval = indexed.get(cursor)
            if interval is None or interval['end_ns'] <= cursor or interval['end_ns'] > end:
                break
            value = finite(interval['relative_clearance_variation_budget_m'])
            if value < 0:
                raise ValueError('negative variation budget')
            total += value; count += 1; cursor = interval['end_ns']
        complete = cursor == end
        result.append({'start_ns': start, 'end_ns': end, 'covered_exactly': complete,
                       'solver_intervals': count,
                       'total_variation_budget_m': total if complete else None,
                       'conditional_clearance_lower_bound_m': min(d0, d1)-total/2 if complete else None,
                       'qualification': 'conditional_model_only' if complete else 'pending_missing_step'})
    return result


def read_dense_bag(path):
    import rosbag
    topics = {'/astra/physics_models': [], '/astra/physics_links': []}; metadata = None
    with rosbag.Bag(str(path)) as bag:
        for topic, msg, _receipt in bag.read_messages(topics=list(topics)+['/astra/physics_metadata']):
            if topic.endswith('metadata'):
                incoming = json.loads(msg.data)
                if metadata is not None and incoming != metadata:
                    raise ValueError('evidence metadata changed')
                metadata = incoming; continue
            topics[topic].append({'stamp_ns': int(msg.header.stamp.to_nsec()), 'frame_id': msg.header.frame_id,
                'names': list(msg.name), 'poses': [(p.position.x, p.position.y, p.position.z,
                    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w) for p in msg.pose],
                'twists': [(t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z) for t in msg.twist]})
    return topics['/astra/physics_models'], topics['/astra/physics_links'], metadata or {}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True);parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--world', type=Path, required=True, help='actual saved world, to classify all nonrobot models')
    parser.add_argument('--clearance-endpoints', type=Path, help='JSON array with source stamp_ns and consistent conservative clearance_m')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('output already exists')
    sources = [args.bag, args.urdf, args.world, Path(__file__), Path(__file__).with_name('geometry.py'),
               Path(__file__).with_name('evaluate_episode.py')]
    if args.clearance_endpoints:
        sources.append(args.clearance_endpoints)
    def binding(p):
        return {'path': str(p.resolve()), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}
    before = [binding(p) for p in sources]
    model_rows, link_rows, metadata = read_dense_bag(args.bag)
    from evaluate_episode import WorldCollisions
    world = WorldCollisions(args.world.read_text())
    report, rows = assess_dense_trace(model_rows, link_rows, args.urdf.read_text(), metadata, world.actor_speed_bounds)
    report['issues'].extend('world: '+error for error in world.errors)
    expected = set(world.models)|{'autolabor_m2'}
    if any(set(row['names']) != expected for row in model_rows):
        report['issues'].append('source model roster differs from actual world plus robot')
    for name, model in world.models.items():
        if name not in world.actor_speed_bounds and model.findtext('static', 'false').lower() != 'true':
            report['issues'].append('unbounded nonrobot dynamic model: '+name)
        elif name not in world.actor_speed_bounds and model_rows:
            initial = None
            for row in model_rows:
                if name not in row['names']:
                    continue
                value = pose(row['poses'][row['names'].index(name)])
                if initial is None:
                    initial = value
                elif norm(tuple(b-a for a, b in zip(initial.xyz, value.xyz))) > 1e-8 or 1-abs(sum(a*b for a, b in zip(initial.xyzw, value.xyzw))) > 1e-12:
                    report['issues'].append('nominally static model moved: '+name)
                    break
    report['integrity_pass'] = not report['issues']
    if args.clearance_endpoints:
        report['coarse_clearance_bounds'] = aggregate_clearance_bounds(json.loads(args.clearance_endpoints.read_text()), rows)
    if before != [binding(p) for p in sources]:
        raise ValueError('input changed during offline evaluation')
    report['source_bindings'] = before
    args.output.mkdir(parents=True)
    with (args.output/'intervals.jsonl').open('w') as stream:
        for row in rows:
            stream.write(json.dumps(row, allow_nan=False)+'\n')
    report['intervals'] = binding(args.output/'intervals.jsonl')
    (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'integrity_pass': report['integrity_pass'], 'qualification': report['qualification'], 'intervals': len(rows)}))
    return 0 if report['integrity_pass'] else 3


if __name__ == '__main__':
    raise SystemExit(main())
