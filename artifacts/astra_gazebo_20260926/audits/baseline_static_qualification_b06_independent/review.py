"""Independent narrow B06 wrapper review. Pure Python, no allocation."""
import ast,copy,csv,hashlib,json,sys
from pathlib import Path
from unittest.mock import patch
A=Path(__file__).resolve().parent;S=A.parents[1];P=S/'workspace/scripts/astra_gazebo';sys.path.insert(0,str(P))
import baseline_static_qualification_b05 as old
import baseline_static_qualification_b06 as new
def bind(path):
 path=Path(path).resolve();data=path.read_bytes();return {'path':str(path),'sha256':hashlib.sha256(data).hexdigest(),'bytes':len(data)}
def write(name,value):(A/name).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
def reject(fn):
 try:fn()
 except ValueError:return True
 return False
author=S/'audits/baseline_static_qualification_b06/component_report.json';ar=json.loads(author.read_text());assert bind(author)['sha256']=='fb6f99417bb8d5ba1b67210dacc9ee73f2e3cc59112766356779ff9f54a1ee90'
resources=json.loads(Path(ar['source_manifest']['path']).read_text())['resources'];assert len(resources)==278
assert all(bind(r['path'])['sha256']==r['sha256'] for r in resources)
checks=[];scenes=['clear','s0','s1','s2','s3']
for seed in old.PROPOSED_SEEDS:
 assert old.proposal_schedule(scenes,[],[str(seed)])==old.proposal_schedule(scenes,[])
 assert reject(lambda:new.proposal_schedule(scenes,[],[str(seed)]))
 assert reject(lambda:new.proposal_schedule(scenes,[],[seed]))
 assert reject(lambda:new.proposal_schedule(scenes,[{'seed':str(seed),'run_id':'historical'}]))
checks.append({'case':'ten_r01_string_aliases_rejected_integer_and_history_conflicts_retained','pass':True,'cases':10})
class DerivedInt(int):pass
class DerivedList(list):pass
class DerivedTuple(tuple):pass
class ExplosiveIterable:
 def __iter__(self):raise AssertionError('invalid lazy object must not be iterated')
bad_elements=[True,False,'263004001',263004001.,DerivedInt(263004001),None,float('nan')]
for value in bad_elements:
 for factory in (list,tuple):assert reject(lambda:new.proposal_schedule(scenes,[],factory([263003001,value])))
bad_containers=['263004001',b'263004001',{263004001},{263004001:1},iter([263004001]),ExplosiveIterable(),DerivedList([263004001]),DerivedTuple([263004001]),None]
for value in bad_containers:assert reject(lambda:new.proposal_schedule(scenes,[],value))
checks.append({'case':'exact_elements_and_exact_eager_container_failclosed','pass':True,'element_cases':14,'container_cases':len(bad_containers)})
seen=[];prior=[263003001,263003002];history=[{'seed':'1','run_id':'old'}];before=copy.deepcopy((scenes,history,prior))
def delegate(s,h,seeds):
 assert type(seeds)is tuple and seeds==(263003001,263003002)
 assert s is scenes and h is history
 prior.append('263004001');assert seeds==(263003001,263003002);seen.append(seeds)
 return {'sentinel':'delegate_return_identity'}
with patch.object(old,'proposal_schedule',side_effect=delegate) as mock:
 result=new.proposal_schedule(scenes,history,prior);assert result=={'sentinel':'delegate_return_identity'} and mock.call_count==1
checks.append({'case':'checked_tuple_frozen_before_single_delegate_despite_later_list_mutation','pass':True})
for seq in [[],(),[263003001,263003002],(263003001,263003002),[-1,0,2**63]]:
 assert new.proposal_schedule(scenes,[],seq)==old.proposal_schedule(scenes,[],seq)
checks.append({'case':'legal_typed_input_old_proposal_exact_equality','pass':True,'seed_value_domain_note':'wrapper only repairs type/alias conflict handling; does not propose these external seed values'})
design=json.loads((S/'contracts/baseline_static_qualification_b05_draft.json').read_text());actual_scenes=[x['scene_id'] for x in design['scenes']]
registry=list(csv.DictReader((S/'audits/baseline_static_qualification_b05/seed_registry_at_proposal.csv').read_text().splitlines()))
n06=json.loads((S/'contracts/navigation_n06_prepared_r01/prepared.json').read_text());prior=[x['seed'] for x in n06['units']]
before=copy.deepcopy((actual_scenes,registry,prior));result=new.proposal_schedule(actual_scenes,registry,prior)
expected=json.loads((S/'audits/baseline_static_qualification_b05/proposed_schedule.json').read_text())
assert result==expected and before==(actual_scenes,registry,prior)
assert result['stage']==old.STAGE and not result['execution_authorized'] and result['allocated_budget']==0
checks.append({'case':'actual_inputs_twenty_row_identity_json_parity_and_no_input_mutation','pass':True})
tree=ast.parse(Path(new.__file__).read_text());functions=[n for n in tree.body if isinstance(n,ast.FunctionDef)]
assert len(functions)==1 and functions[0].name=='proposal_schedule'
imports=[n for n in tree.body if isinstance(n,ast.Import)];assert len(imports)==1 and imports[0].names[0].name=='baseline_static_qualification_b05'
calls=[n for n in ast.walk(functions[0]) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
assert len(calls)==1 and calls[0].func.attr=='proposal_schedule'
checks.append({'case':'only_single_frozen_B05_delegate_no_math_or_IO_added','pass':True})
assert all(bind(r['path'])['sha256']==r['sha256'] for r in resources)
write('focused_results.json',{'checks':checks,'original_resources_unchanged':278,'B05_negative_retained':True,'zero_authority':True})
(A/'review.md').write_text('''# B06 独立窄审

新 wrapper 的种子类型修补通过。20个 string/int 历史语义对照中的原十个 string alias 均被拒绝；bool/float/int子类、混合序列、list/tuple子类与惰性对象均在 delegate 前拒绝。独立反例确认：delegate 执行时再修改 caller 原 list，不会改变已验证的 tuple；只调用一次冻结 B05 函数。

真实冻结输入的20行 B05 proposal 完全相同，调用者输入不被修改，原264与新278资源均未变。该组件返回原 B05 零权限 draft 身份，不构成运行身份复用或新B06执行阶段。外部种子数值域未扩展为资格结论；这里只验证 exact-int 的冲突比较。

完整运行默认值、九真实碰撞体包含、物理命令/回执闭包、新 manager/collector 与动态 actor 仍未实现或未资格。没有ROS、训练、seed登记、执行release或真实CSV修改。B05原负例保留。本审查者非B05/B06作者，曾提供需求提醒并作者上游B04，未把上游作者工作重新宣称独审。
''')
extra=[author,Path(ar['source_manifest']['path']),S/'audits/baseline_static_qualification_b05_independent/review.json']
extra += [p for p in A.iterdir() if p.is_file() and p.name not in ('resources.json','review.json')]
bound={r['path']:r for r in resources}
for p in extra:r=bind(p);bound[r['path']]=r
write('resources.json',{'resources':sorted(bound.values(),key=lambda r:r['path'])})
report={'schema':'astra_b06_independent_seed_type_repair_review_v1','status':'independent_offline_exact_int_seed_repair_pass_zero_authority','independent_review':True,
 'author_report':bind(author),'reviewed_source':bind(new.__file__),'B05_unchanged_source':bind(old.__file__),
 'author_manifest':ar['source_manifest'],'frozen_resource_count':278,
 'independent_tests':{'groups':len(checks),'result':bind(A/'focused_results.json')},
 'author_tests':{'passed':10,'log':bind(A/'author_tests_rerun.log')},
 'fixed_gap':'other_proposed_seeds numeric strings cannot bypass integer collision gate',
 'current_frozen_B05_schedule_changed':False,'new_execution_stage_created':False,
 'inherited_limits':['No complete initialized default namespace.','No actual nine-shape containment or physical receipt qualification.','No execution manager/collector closure or dynamic actor support.'],
 'authority':{'execution_authorized':False,'allocated_budget':0,'release_created':False,'seed_registered':False,'ROS_started':False,'training_started':False,'real_CSV_written':False},
 'resources':bind(A/'resources.json'),'narrative':bind(A/'review.md')}
log=(A/'author_tests_rerun.log').read_text();assert 'Ran 10 tests' in log and log.rstrip().endswith('OK')
write('review.json',report);print(json.dumps({'report':bind(A/'review.json'),'resources':bind(A/'resources.json')},indent=2))
