import copy
import hashlib
import importlib.util
from pathlib import Path
import sys
import pytest
import yaml

STAGE=Path(__file__).resolve().parents[4]
PACKAGE=STAGE/'workspace/src/simulation/m2_gazebo'
SCRIPT=PACKAGE/'scripts/astra_actuation_command_source_p02.py'
spec=importlib.util.spec_from_file_location('command_source_p02_test',SCRIPT)
m=importlib.util.module_from_spec(spec);sys.modules[spec.name]=m;spec.loader.exec_module(m)
CONTRACT=yaml.safe_load((STAGE/'contracts/m12_physics_p02.yaml').read_text())


def fresh(start_clock=123_456_789):
    s=m.IntegerSchedule(CONTRACT);start=s.start(start_clock)
    assert start==start_clock+100_000_000
    return s,start


@pytest.mark.parametrize('boundary,expected',[(0,'stationary'),(2,'accelerate'),(6,'turn_positive'),(9,'turn_negative'),
    (12,'saturation_positive'),(15,'saturation_negative'),(18,'zero_speed_yaw'),(21,'reaccelerate_brake'),
    (25,'commanded_brake'),(28,'reaccelerate_loss'),(32,None),(36,'terminal_zero')])
def test_integer_boundary_plus_minus_one_ns(boundary,expected):
    for delta in (-1,0,1):
        s,start=fresh(10**18+987_654_321)
        now=start+boundary*10**9+delta
        command=s.poll(now)
        if boundary==0 and delta<0:assert command is None;continue
        if boundary==32 and delta>=0:assert command is None;continue
        if boundary==36 and delta<0:assert command is None;continue
        phase=next((r[0] for r in s.phases if r[1]<=now-start<r[2]),'terminal_zero')
        assert command is not None and command.phase==phase
        if delta==0:assert command.phase==expected


def test_complete_36s_has640_commands_no_loss_commands_one_terminal():
    s,start=fresh();commands=[]
    for slot in range(721):
        now=start+slot*50_000_000
        command=s.poll(now)
        if command is not None:
            sent=s.prepare_send(command,now)
            assert sent is not None;commands.append(sent)
    assert len(commands)==641
    assert len([c for c in commands if c.phase=='terminal_zero'])==1
    assert all(not(start+32*10**9<=c.source_ns<start+36*10**9) for c in commands)
    assert s.state=='complete' and s.poll(start+37*10**9) is None
    expected_loss=[s.poll(start+36*10**9+i) for i in range(10)]
    assert expected_loss==[None]*10


def test_frozen_commands_match_p01_including_saturation_and_zero_speed_yaw():
    s,_=fresh()
    old=yaml.safe_load((STAGE/'contracts/m12_physics_p01.yaml').read_text())
    assert CONTRACT['phase_schedule']==old['phase_schedule']
    assert len(s.phases)==11
    assert s.phases[4][3]==pytest.approx((1.2,1.2*m.math.tan(.65)/.65))
    assert s.phases[6][3]==(0.,.6)


def test_skips_old_slots_without_burst_or_repeating_new_slot():
    s,start=fresh()
    assert s.poll(start).missed_slots==0
    command=s.poll(start+160_000_000)
    assert command.missed_slots==2 and command.scheduled_ns==start+150_000_000
    assert s.poll(start+160_000_001) is None
    command=s.poll(start+1_999_999_999)
    sent=s.prepare_send(command,start+2_000_000_001)
    assert sent.phase=='accelerate' and sent.scheduled_ns==start+2_000_000_000
    assert s.poll(start+2_000_000_002) is None


def test_send_recheck_cannot_leak_previous_phase_into_loss():
    s,start=fresh()
    command=s.poll(start+31_999_999_999)
    assert command.phase=='reaccelerate_loss'
    assert s.prepare_send(command,start+32_000_000_000) is None
    assert s.loss_seen
    # A subsequent reset/regression cannot inject fault-stop zero into loss.
    assert s.fault('clock reset',start+31_900_000_000) is None
    assert s.state=='fault'


@pytest.mark.parametrize('elapsed_ns',[32_000_000_000,32_000_000_001,35_999_999_999])
def test_all_special_commands_are_suppressed_in_loss(elapsed_ns):
    s,start=fresh();now=start+elapsed_ns
    for phase in ('bootstrap_zero','fault_stop_zero','terminal_zero'):
        assert s.prepare_send(m.Command(phase,0.,0.,now,now),now) is None
    assert s.fault('stall',now) is None


def test_start_is_one_shot_even_invalid_initial_attempt():
    s,start=fresh()
    with pytest.raises(RuntimeError):s.start(start)
    s=m.IntegerSchedule(CONTRACT)
    with pytest.raises(ValueError):s.start(0)
    with pytest.raises(RuntimeError):s.start(100)


def test_clock_guard_stall_regression_and_fault_stop():
    guard=m.SourceClockGuard(10_000_000_000);guard.reset(123,1_000)
    guard.observe(123,10_000_001_000)
    with pytest.raises(RuntimeError,match='stalled'):guard.observe(123,10_000_001_001)
    guard.reset(123,1_000);guard.observe(124,10_000_001_001)
    with pytest.raises(RuntimeError,match='regressed'):guard.observe(123,10_000_001_002)
    s,start=fresh();s.poll(start+1_000)
    with pytest.raises(RuntimeError,match='regressed'):s.poll(start+999)
    stop=s.fault('regression',start+999)
    assert stop.phase=='fault_stop_zero' and (stop.v,stop.w)==(0.,0.)
    assert s.fault('again',start+999) is None


@pytest.mark.parametrize('bad',[True,-1,'nan','0.0000000001'])
def test_noninteger_or_invalid_time_rejected(bad):
    with pytest.raises((ValueError,TypeError)):m.seconds_ns(bad)
    with pytest.raises((ValueError,TypeError)):m.require_ns(bad)


def test_schedule_semantic_changes_rejected():
    mutations=[lambda c:c.update(total_schedule_s=35.9),lambda c:c.update(publish_period_s=.049),
        lambda c:c['phase_schedule'][10].update(publish=True),lambda c:c['phase_schedule'][1].update(speed_mps=1.3),
        lambda c:c['phase_schedule'][0].update(duration_s=True)]
    for mutate in mutations:
        c=copy.deepcopy(CONTRACT);mutate(c)
        with pytest.raises(ValueError):m.IntegerSchedule(c)


def test_contract_hash_identity_and_runtime_guards(tmp_path):
    c=copy.deepcopy(CONTRACT);c['run_id']='m12_physics_p02';p=tmp_path/'contract.yaml';p.write_text(yaml.safe_dump(c))
    sha=hashlib.sha256(p.read_bytes()).hexdigest()
    assert m.load_contract(p,sha,c['seed'],c['run_id'])['stage']=='astra_m12_physics_p02'
    for badsha,seed,runid in [('0'*64,c['seed'],c['run_id']),(sha,True,c['run_id']),
                             (sha,c['seed']+1,c['run_id']),(sha,c['seed'],'wrong_run')]:
        with pytest.raises(ValueError):m.load_contract(p,badsha,seed,runid)
    xml=(STAGE/'audits/m12_p02_robot_description.xml').read_text().replace('<dynamicsSeed>42</dynamicsSeed>',f'<dynamicsSeed>{c["seed"]}</dynamicsSeed>')
    m.validate_runtime_environment('http://127.0.0.1:11311',True,True,xml,c,c['seed'])
    for uri,sim,marker in [('http://robot.lan:11311',True,True),('http://127.0.0.1:11311',False,True),
                          ('http://user@localhost:11311',True,True),('http://localhost:11311',True,False)]:
        with pytest.raises(RuntimeError):m.validate_runtime_environment(uri,sim,marker,xml,c,c['seed'])
    with pytest.raises(ValueError):m.validate_runtime_environment('http://localhost:11311',True,True,xml,c,c['seed']+1)


def test_runtime_transport_and_no_dense_or_heavy_io():
    text=SCRIPT.read_text()
    assert "rospy.init_node('actuation_command_source')" in text
    assert "'/astra_fixture/command_source_status'" in text
    assert "'/astra_fixture/start'" in text and "'/astra_fixture/schedule_start_ns'" in text
    assert "event_pub=rospy.Publisher('/astra_fixture/command_event',DiagnosticArray,queue_size=1000)" in text
    assert 'rospy.Subscriber(' not in text and 'write_text(' not in text and '.open(' not in text
    assert 'time.sleep(.001)' in text and 'rospy.sleep' not in text
    assert text.count("rospy.Publisher('/cmd_vel_navigation'")==1
    assert 'rospy.signal_shutdown' not in text
