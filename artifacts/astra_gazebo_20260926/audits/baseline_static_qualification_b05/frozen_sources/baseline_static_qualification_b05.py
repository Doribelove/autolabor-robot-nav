"""B05 offline proposal helpers. No ROS, execution, reservation or parameter writes.

Full parameter equality is type-sensitive, including IEEE float representations;
it does not manufacture expected runtime defaults from an observed run. A future
manager must bind complete, phase-specific prepared trees before execution.
"""
import hashlib
import json
import math

STAGE = 'astra_baseline_static_qualification_b05'
METHODS = ('actual_teb_rear_reference_b03', 'dwa_rear_reference_b03')
PROPOSED_SEEDS = tuple(range(263004001, 263004011))
PHASES = ('paused_bootstrap', 'positive_clock_ready', 'paused_terminal')
CAP_S = 22.0


def typed_tree(value):
    """Closed ROS/XMLRPC parameter types; preserve list order and float bits."""
    kind = type(value)
    if kind is bool:
        return ['bool', value]
    if kind is int:
        if not -(2**31) <= value < 2**31:
            raise ValueError('parameter integer outside XMLRPC signed32 domain')
        return ['int', value]
    if kind is float:
        if not math.isfinite(value):
            raise ValueError('nonfinite parameter')
        return ['double', value.hex()]
    if kind is str:
        return ['string', value]
    if kind is list:
        return ['array', [typed_tree(v) for v in value]]
    if kind is dict:
        if any(type(k) is not str or not k for k in value):
            raise ValueError('nonempty string parameter keys required')
        return ['struct', [[k, typed_tree(value[k])] for k in sorted(value)]]
    raise ValueError('unsupported parameter type: ' + kind.__name__)


def parameter_digest(value):
    return hashlib.sha256(json.dumps(typed_tree(value), ensure_ascii=False,
                                    separators=(',', ':')).encode('utf8')).hexdigest()


def compare_full_parameters(prepared, observed):
    """Compare every /move_base key/value; no ignored or learned extra fields.

    Both arguments are the contents of that namespace, not a parameter-server
    root. A pass proves equality to this supplied tree, not its preparation
    provenance/completeness; release binding must establish those separately.
    """
    if type(prepared) is not dict or type(observed) is not dict or not prepared:
        raise ValueError('nonempty prepared namespace and observed struct required')
    expected = typed_tree(prepared)
    actual = typed_tree(observed)
    mismatches = []

    def walk(a, b, path):
        if type(a) is not type(b):
            mismatches.append({'path': path, 'kind': 'type'})
        elif type(a) is dict:
            for k in sorted(a.keys() | b.keys()):
                child = path + '/' + k.replace('~', '~0').replace('/', '~1')
                if k not in a:
                    mismatches.append({'path': child, 'kind': 'unexpected'})
                elif k not in b:
                    mismatches.append({'path': child, 'kind': 'missing'})
                else:
                    walk(a[k], b[k], child)
        elif type(a) is list:
            if len(a) != len(b):
                mismatches.append({'path': path, 'kind': 'array_length'})
            for i, (x, y) in enumerate(zip(a, b)):
                walk(x, y, path + '/' + str(i))
        elif typed_tree(a) != typed_tree(b):
            mismatches.append({'path': path, 'kind': 'value'})

    walk(prepared, observed, '/move_base')
    assert bool(mismatches) == (expected != actual)
    return {'schema': 'astra_b05_exact_parameters_v1', 'pass': not mismatches,
            'prepared_typed_sha256': parameter_digest(prepared),
            'observed_typed_sha256': parameter_digest(observed),
            'mismatches': mismatches, 'ignored_paths': [],
            'runtime_default_completion_proved_by_this_function': False}


def compare_parameter_phases(prepared, observed):
    """No phase omitted; expected defaults must already exist before execution."""
    if type(prepared) is not dict or type(observed) is not dict or set(prepared) != set(PHASES) or set(observed) != set(PHASES):
        raise ValueError('all three exact parameter phases required')
    rows = {phase: compare_full_parameters(prepared[phase], observed[phase]) for phase in PHASES}
    return {'schema':'astra_b05_three_phase_parameters_v1', 'pass':all(r['pass'] for r in rows.values()), 'phases':rows}


def proposal_schedule(scene_ids, historical_rows, other_proposed_seeds=()):
    """Fixed 5 static geometries x2 repeats x2 methods; allocation remains zero."""
    if type(scene_ids) is not list or len(scene_ids) != 5 or any(type(x) is not str or not x for x in scene_ids) or len(set(scene_ids)) != 5:
        raise ValueError('exactly five distinct static scene identities required')
    historical_seeds = {int(r['seed']) for r in historical_rows}
    historical_ids = {r['run_id'] for r in historical_rows}
    if set(PROPOSED_SEEDS) & (historical_seeds | set(other_proposed_seeds)):
        raise ValueError('historical or proposed seed collision; do not silently select alternatives')
    rows = []
    for scene_index, scene in enumerate(scene_ids):
        for repeat in range(2):
            block = scene_index * 2 + repeat
            methods = METHODS if block % 2 == 0 else METHODS[::-1]
            for position, method in enumerate(methods):
                run_id = STAGE + '_b%02d_' % block + method
                if run_id in historical_ids:
                    raise ValueError('historical run identity collision')
                rows.append({'sequence': len(rows) + 1, 'block_index': block,
                             'scene_id': scene, 'repeat': repeat, 'pair_position': position + 1,
                             'seed': PROPOSED_SEEDS[block], 'method': method,
                             'run_id': run_id, 'attempt': 1, 'seed_status': 'proposal_unallocated'})
    return {'schema': 'astra_b05_zero_authority_schedule_v1', 'stage': STAGE,
            'execution_authorized': False, 'training_authorized': False,
            'allocated_budget': 0, 'proposed_maximum_episodes': 20,
            'retry_allowed': False, 'resume_allowed': False, 'seeds_registered': False,
            'schedule': rows}


def failure_inclusive_summary(schedule, records):
    """Descriptive accounting, not a qualification gate or performance test.

    All scheduled rows need a terminal or explicitly retired-unstarted record.
    Retired rows get an administrative cap in the all-schedule table, never a
    fabricated observed outcome. Only terminal rows enter attempted means.
    """
    if type(schedule) is not list or type(records) is not list or len(schedule) != 20:
        raise ValueError('exact20-row schedule and explicit record list required')
    try:
        canonical = proposal_schedule([schedule[i*4]['scene_id'] for i in range(5)], [])['schedule']
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError('invalid proposed schedule structure') from exc
    if typed_tree(schedule) != typed_tree(canonical):
        raise ValueError('accounting requires the exact fixed proposal schedule')
    expected = {r['run_id']: r for r in schedule}
    if len(expected) != 20 or len(records) != 20:
        raise ValueError('all scheduled identities must occur exactly once')
    seen = set(); result = []
    for r in records:
        if type(r) is not dict or r.get('run_id') not in expected or r['run_id'] in seen:
            raise ValueError('unknown or duplicate record')
        seen.add(r['run_id']); s = expected[r['run_id']]
        if any(type(r.get(k)) is not type(s[k]) or r[k] != s[k] for k in ('method', 'seed', 'attempt')):
            raise ValueError('record identity differs')
        if set(r) != {'run_id','method','seed','attempt','record_kind','phase','capture_integrity_pass','geometry_pass','goal_elapsed_s','reason'}:
            raise ValueError('closed accounting record keys required')
        if type(r['reason']) is not str or not r['reason']:
            raise ValueError('explicit record reason required')
        if r['record_kind'] == 'retired_unstarted':
            if any(r[k] is not None for k in ('phase','capture_integrity_pass','geometry_pass','goal_elapsed_s')):
                raise ValueError('retirement must not fabricate measurements')
            valid = False; attempted = False
        elif r['record_kind'] == 'terminal':
            attempted = True
            if r['phase'] not in ('complete','timeout','fault','stopped') or type(r['capture_integrity_pass']) is not bool or type(r['geometry_pass']) is not bool:
                raise ValueError('typed terminal evidence required')
            elapsed = r['goal_elapsed_s']
            if r['phase'] == 'complete':
                if type(elapsed) not in (int, float) or not math.isfinite(elapsed) or not 0 <= elapsed <= CAP_S:
                    raise ValueError('complete phase needs actual finite source elapsed within cap')
            elif elapsed is not None:
                raise ValueError('non-goal terminal cannot invent goal elapsed')
            valid = r['phase'] == 'complete' and r['capture_integrity_pass'] and r['geometry_pass']
        else:
            raise ValueError('unclosed execution state is not a terminal result')
        result.append(dict(s, observed=attempted, valid_completed=valid,
                           restricted_time_s=float(r['goal_elapsed_s']) if valid else CAP_S,
                           accounting_kind=r['record_kind'], reason=r['reason']))
    result.sort(key=lambda r:r['sequence'])
    methods = {}
    for method in METHODS:
        rs = [r for r in result if r['method'] == method]; attempted = [r for r in rs if r['observed']]
        if len(rs) != 10:
            raise ValueError('exact ten rows per method required')
        methods[method] = {'scheduled': 10, 'attempted': len(attempted),
                           'retired_unstarted': 10-len(attempted),
                           'valid_completed': sum(r['valid_completed'] for r in rs),
                           'all_schedule_administrative_cap_mean_s': sum(r['restricted_time_s'] for r in rs)/10,
                           'attempted_failure_inclusive_mean_s': sum(r['restricted_time_s'] for r in attempted)/len(attempted) if attempted else None}
    pairs=[]
    for block in range(10):
        rs = [r for r in result if r['block_index'] == block]
        if len(rs)!=2 or {r['method'] for r in rs}!=set(METHODS) or len({(r['scene_id'],r['seed'],r['repeat']) for r in rs})!=1:
            raise ValueError('counterpart geometry/seed/method pairing differs')
        by={r['method']:r for r in rs}
        pairs.append({'block_index':block,'both_attempted':all(r['observed'] for r in rs),
                      'dwa_minus_teb_administrative_s':by[METHODS[1]]['restricted_time_s']-by[METHODS[0]]['restricted_time_s']})
    return {'schema':'astra_b05_failure_inclusive_accounting_v1','rows':result,'methods':methods,'pairs':pairs,
            'all20_attempted':all(r['observed'] for r in result), 'qualification_determined':False,
            'performance_claim':False,'retired_values_are_administrative_not_simulated':True}
