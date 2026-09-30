"""Transport stubs around actual N05 methods; no ROS/signals/real socket binds."""
from pathlib import Path
import csv,json,sys,types
import pytest
S=Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926');P=S/'workspace/scripts/astra_gazebo'
sys.path.insert(0,str(P));sys.path.insert(0,str(P/'tests'))
import navigation_process_cleanup_n05 as cleanup
from test_navigation_n05_pipeline import fixture,manager

@pytest.mark.parametrize('occupied',[None,14830,14831])
def test_actual_cleanup_checks_new_ports_and_no_false_success(tmp_path,monkeypatch,occupied):
 calls=[];writes=[]
 class Socket:
  def __enter__(self):return self
  def __exit__(self,*a):pass
  def setsockopt(self,*a):pass
  def bind(self,address):
   calls.append(address)
   if address[1]==occupied:raise OSError('synthetic occupied new-stage port')
 monkeypatch.setattr(cleanup.socket,'socket',Socket)
 monkeypatch.setattr(cleanup,'owned_processes',lambda _:[])
 monkeypatch.setattr(cleanup,'atomic',lambda p,v:writes.append((p,v)))
 monkeypatch.setattr(cleanup.os,'kill',lambda *a:pytest.fail('unexpected signal'))
 monkeypatch.setattr(cleanup.os,'killpg',lambda *a:pytest.fail('unexpected group signal'))
 proc=types.SimpleNamespace(poll=lambda:0)
 if occupied is None:
  cleanup.stop_group(proc,tmp_path)
  assert calls==[('127.0.0.1',14830),('127.0.0.1',14831)]
  assert len(writes)==1 and writes[0][1]['ports_14830_14831_released'] is True
  assert 'ports_14820_14821_released' not in writes[0][1]
 else:
  with pytest.raises(OSError,match='occupied'):cleanup.stop_group(proc,tmp_path)
  assert not writes and calls[-1][1]==occupied

@pytest.mark.parametrize('kind',['foreign_same_seed','wrong_release'])
def test_registry_rejected_before_allocation(fixture,kind):
 f=fixture;p=f.stage/'seed_registry.csv';rows=list(csv.DictReader(p.open()))
 if kind=='foreign_same_seed':rows.append(dict(rows[0],run_id='foreign'))
 else:rows[0]['source_binding']='b'*64
 with p.open('w') as out:
  writer=csv.DictWriter(out,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
 with pytest.raises(ValueError):f.execute()
 assert not f.calls['popen'] and not (f.stage/'queue/navigation_n05_ledger.json').exists()

def test_preparation_error_consumes_one_retires_next(fixture,monkeypatch):
 f=fixture;original=manager.evidence.read_bound
 def read(item):
  if Path(item['path']).name=='contract.json':raise ValueError('synthetic corrupt preparation')
  return original(item)
 monkeypatch.setattr(manager.evidence,'read_bound',read)
 assert f.execute()==3
 assert [r['status'] for r in f.ledger()['units']]==['consumed','retired_unrun']
 assert not f.calls['popen']
 first=f.ledger()['units'][0];receipt=json.loads(Path(first['receipt']['path']).read_bytes())
 assert receipt['integrity_pass'] is False and 'corrupt preparation' in receipt['error']

def test_cleanup_port_failure_closes_receipt_and_retires_remaining(fixture,monkeypatch):
 f=fixture
 def fail(proc,root):raise OSError('synthetic port14831 still occupied')
 monkeypatch.setattr(manager.cleanup,'stop_group',fail)
 assert f.execute()==3
 ledger=f.ledger();assert ledger['status']=='terminal' and [x['status'] for x in ledger['units']]==['consumed','retired_unrun']
 assert len(f.calls['popen'])==1 and not f.calls['assess']
 receipt=json.loads(Path(ledger['units'][0]['receipt']['path']).read_bytes())
 assert receipt['integrity_pass'] is False and '14831' in receipt['error']
