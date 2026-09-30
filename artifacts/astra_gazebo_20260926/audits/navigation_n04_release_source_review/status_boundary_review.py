"""Focused independent regression of the two retained event timing negatives."""
import copy,json,sys
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace';sys.path.insert(0,str(W/'scripts/astra_gazebo'));sys.path.insert(0,str(W/'scripts/astra_gazebo/tests'))
import test_event_actor_contract_n04 as fixture
import event_actor_contract_n04 as event
from navigation_evidence_n04 import sha
cases=[]
def check(name,rows,end,expected):
    result=event.audit_status(rows,{'actor_1':event.plugin_contract(fixture.plugin())},{'source_ns':7_000_000_000},end)
    assert result['pass_'] is expected,(name,result)
    cases.append(dict(case=name,expected=expected,result=result))
base=fixture.rows()[0]
check('realistic_mature_activation',[base],8_000_000_000,True)
for name,value in [('never_activated',0),('future_activation',9_000_000_000),('activation_one_step_late',7_202_000_000)]:
    row=dict(base,first_active_begin_ns=value);check(name,[row],8_000_000_000,False)
pre=dict(base,status_source_ns=7_020_000_000,first_active_begin_ns=0)
active=dict(base,status_source_ns=7_200_000_000)
check('consumed_pending_delay',[pre],7_040_000_000,True)
# C++ can emit a periodic old state followed by newly-active state at same Begin.
check('same_Begin_zero_then_active',[pre,dict(active,first_active_begin_ns=0),active],7_200_000_000,True)
check('activation_latch_cleared',[pre,active,dict(base,first_active_begin_ns=0)],8_000_000_000,False)
check('activation_latch_changed',[pre,active,dict(base,first_active_begin_ns=7_202_000_000)],8_000_000_000,False)
check('mature_status_still_zero',[pre,dict(base,first_active_begin_ns=0)],8_000_000_000,False)
# Stop before an actor's intentional delay is valid; do not invent movement.
check('short_episode_before_delayed_activation',[pre],7_020_000_000,True)
(D/'status_boundary_review.json').write_text(json.dumps({'status':'pass','source':{'path':event.__file__,'sha256':sha(event.__file__)},'cases':cases,'old_counterexamples_preserved':str(D/'status_counterexamples_r01.json')},indent=2)+'\n')
print(json.dumps({'status':'pass','cases':len(cases)}))
