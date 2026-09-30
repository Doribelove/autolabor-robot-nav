"""Offline contract for the N05 coordinator-gated reactive actor.

The event establishes one shared source timestamp. Actual actor motion starts
only after receipt at a Gazebo Begin; transport delay is measured, not hidden.
"""
import math
import re
import xml.etree.ElementTree as ET

PLUGIN = 'libastra_event_actor_n05_plugin.so'
GATE_TOPIC = '/astra/navigation/gate'
STATUS_SCHEMA = 'astra_event_actor_gate_n05_v1'
MAX_TRANSPORT_NS = 50_000_000
METHODS = ('astra_analytic_route_v1', 'astra_pct_route_v1', 'astra_analytic_preview_eligible_n05',
           'frozen_r2_v060_sensor_lifecycle_v1',
           'actual_teb_common_route', 'dwa_common_route')


def plugin_contract(plugin):
    if plugin.get('filename') != PLUGIN:
        raise ValueError('expected versioned N05 event actor')
    tags = [child.tag for child in plugin]
    if len(tags) != len(set(tags)):
        raise ValueError('duplicate event actor SDF field')
    if any(plugin.find(key) is not None for key in
           ('route_origin', 'route_yaw', 'gate_route_x', 'start_delay', 'activation_ego_x')):
        raise ValueError('spatial or legacy actor activation forbidden in N05')
    required = ('expected_run_id', 'expected_seed', 'expected_method', 'gate_topic',
                'max_gate_transport_delay_ns', 'gate_delay', 'ego_model',
                'policy', 'start', 'end', 'conflict')
    if any(not plugin.findtext(key) for key in required):
        raise ValueError('required event actor field absent')
    run_id = plugin.findtext('expected_run_id')
    seed_text = plugin.findtext('expected_seed')
    method = plugin.findtext('expected_method')
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,159}', run_id):
        raise ValueError('event actor run identity invalid')
    if 'UNBOUND' in run_id.upper() or 'TEMPLATE' in run_id.upper():
        raise ValueError('unbound actor world is non-executable')
    if not re.fullmatch(r'[1-9][0-9]*', seed_text) or not 0 < int(seed_text) <= 2147483647:
        raise ValueError('event actor seed invalid')
    if method not in METHODS:
        raise ValueError('event actor method invalid')
    if (plugin.findtext('gate_topic') != GATE_TOPIC or
            plugin.findtext('max_gate_transport_delay_ns') != str(MAX_TRANSPORT_NS)):
        raise ValueError('fixed event topic or transport deadline differs')
    delay = float(plugin.findtext('gate_delay'))
    if not math.isfinite(delay) or delay < 0 or delay > 22:
        raise ValueError('bounded actor gate delay required')
    if plugin.findtext('ego_model') != 'autolabor_m2':
        raise ValueError('actor observes wrong ego identity')
    return dict(run_id=run_id, seed=int(seed_text), method=method,
                gate_delay_ns=round(delay * 1e9),
                max_gate_transport_delay_ns=MAX_TRANSPORT_NS)


def world_actors(xml):
    root = ET.fromstring(xml)
    world = root.find('world') if root.tag == 'sdf' else root
    result = {}
    for model in world.findall('model'):
        if any(p.get('filename') != PLUGIN for p in model.findall('plugin')):
            raise ValueError('N05 admits only reviewed event actor model plugins')
        plugins = [p for p in model.findall('plugin') if p.get('filename') == PLUGIN]
        if plugins:
            if len(plugins) != 1 or len(model.findall('plugin')) != 1:
                raise ValueError('one event actor plugin per model required')
            name = model.get('name')
            if not name or name in result or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', name):
                raise ValueError('unique ROS-compatible actor name required')
            result[name] = plugin_contract(plugins[0])
    return result


def status_topic(model_name):
    return '/astra/actors/' + model_name + '/gate_status'


def graph_audit(system, actor_names, coordinator_node):
    publishers=dict(system[0]);subscribers=dict(system[1])
    gate_publishers=sorted(publishers.get(GATE_TOPIC,[]))
    status_publishers={name:sorted(publishers.get(status_topic(name),[])) for name in actor_names}
    actor_gate_connected=not actor_names or '/gazebo' in subscribers.get(GATE_TOPIC,[])
    return dict(pass_=gate_publishers==[coordinator_node] and actor_gate_connected and
                all(nodes==['/gazebo'] for nodes in status_publishers.values()),
                gate_publishers=gate_publishers,status_publishers=status_publishers,
                actor_gate_subscriber_present=actor_gate_connected,
                scope='Actual ROS master registrations; trusted authorized process scope.')


def audit_status(rows, actors, gate, end_ns):
    """Validate all retained statuses and a near-terminal source observation.

    No inferred timestamp substitutes for actual Begin time. A pre-gate fault
    remains an integrity failure. Actor dynamics/contact are assessed separately.
    """
    failures = []
    metrics = {}
    for name, expected in actors.items():
        own = [r for r in rows if r.get('model_name') == name]
        reasons = []
        observed = []
        times = []
        latched_active = 0
        for row in own:
            if row.get('schema') != STATUS_SCHEMA:
                reasons.append('schema')
            for key in ('run_id', 'seed', 'method'):
                if type(row.get(key)) is not type(expected[key]) or row.get(key) != expected[key]:
                    reasons.append('identity:' + key)
            if type(row.get('fault')) is not bool or row.get('fault') is not False:
                reasons.append('fault')
            if not isinstance(row.get('reason'), str):
                reasons.append('fault_reason_type')
            keys = ('status_source_ns', 'gate_source_ns', 'receipt_ros_ns',
                    'receipt_steady_ns', 'first_begin_ns', 'transport_delay_ns',
                    'activation_source_ns', 'first_active_begin_ns', 'gate_delay_ns', 'duplicate_count')
            if any(type(row.get(k)) is not int or row[k] < 0 for k in keys):
                reasons.append('nonnegative_integer_timestamps'); continue
            times.append(row['status_source_ns'])
            if row['gate_delay_ns'] != expected['gate_delay_ns']:
                reasons.append('declared_delay')
            if type(row.get('max_gate_transport_delay_ns'))is not int or row['max_gate_transport_delay_ns']!=MAX_TRANSPORT_NS:
                reasons.append('declared_transport_deadline')
            if row['first_begin_ns'] == 0:
                continue
            if row['gate_source_ns'] != gate['source_ns']:
                reasons.append('shared_gate_source')
            delay = row['first_begin_ns'] - row['gate_source_ns']
            if delay != row['transport_delay_ns'] or not 0 <= delay <= MAX_TRANSPORT_NS:
                reasons.append('transport_deadline')
            if row['activation_source_ns'] != row['gate_source_ns'] + expected['gate_delay_ns']:
                reasons.append('relative_activation')
            if row['status_source_ns'] < row['first_begin_ns']:
                reasons.append('future_begin')
            step=2_000_000
            eligible_begin=max(row['first_begin_ns'],
                ((row['activation_source_ns']+step-1)//step)*step)
            active=row['first_active_begin_ns']
            if row['first_begin_ns']%step:
                reasons.append('off_grid_begin')
            if active:
                if active!=eligible_begin or active>row['status_source_ns']:
                    reasons.append('incorrect_or_future_first_active')
                if latched_active and active!=latched_active:
                    reasons.append('first_active_latch_mutated')
                latched_active=active
            elif latched_active:
                reasons.append('first_active_latch_cleared')
            elif row['status_source_ns']>eligible_begin:
                reasons.append('mature_status_missing_first_active')
            observed.append((row['gate_source_ns'], row['first_begin_ns'],
                             row['transport_delay_ns'], row['activation_source_ns']))
        if not own or not observed:
            reasons.append('missing_consumed_gate_status')
        if len(set(observed)) > 1:
            reasons.append('latched_event_mutated')
        if observed:
            eligible=max(observed[0][1],((observed[0][3]+1_999_999)//2_000_000)*2_000_000)
            if end_ns>eligible and not latched_active:
                reasons.append('terminal_missing_first_active')
        if any(a > b for a, b in zip(times, times[1:])):
            reasons.append('status_clock_regression')
        if not times or not any(end_ns - 40_000_000 <= t <= end_ns + 100_000_000 for t in times):
            reasons.append('terminal_status_coverage')
        metrics[name] = dict(status_count=len(own), event_observation_count=len(observed),
                             transport_delay_ns=observed[0][2] if observed else None,
                             pass_=not reasons, issues=sorted(set(reasons)))
        failures.extend(name + ':' + reason for reason in set(reasons))
    unknown = sorted({str(r.get('model_name')) for r in rows} - set(actors))
    if unknown:
        failures.append('unknown_actor_status:' + ','.join(unknown))
    return dict(pass_=not failures, actors=metrics, issues=sorted(failures),
                maximum_transport_delay_ns=MAX_TRANSPORT_NS,
                scope='Shared event identity and bounded receipt-to-Begin activation; no zero-latency claim.')
