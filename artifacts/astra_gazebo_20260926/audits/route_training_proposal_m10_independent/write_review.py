from pathlib import Path
from dataclasses import replace
import hashlib,json,math,sys
S=Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926');W=S/'workspace';A=Path(__file__).resolve().parent;P=W/'scripts/astra_gazebo';sys.path.insert(0,str(P))
import route_training_proposal_m10 as m
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
bind=lambda p:dict(path=str(Path(p).resolve()),sha256=sha(p))
author_path=S/'audits/route_training_proposal_m10/component_report.json';author=json.loads(author_path.read_bytes());assert sha(author_path)=='19555dbdeeb695e36fe7cb7e1e0d544e5ead76d5e1d16adf9436f8b8b63a9f8f'
for row in author['resources']:assert sha(row['path'])==row['sha256']
assert sha(A/'source_snapshot.py')==author['source']['sha256'];assert sha(m.__file__)==author['source']['sha256']
assert '32 passed' in (A/'author_tests_replay.log').read_text();assert '18 passed' in (A/'tests_attempt02.log').read_text()
# Save exact out-of-precondition numerical example separately, with standards-compliant JSON.
c=m.Context('independent_fixture','a'*64,'b'*64,'c'*64,1_000_000_000,900_000_000,950_000_000,980_000_000,2_000_000_000)
o=m.ActorOutput((0.,)*8,(0.,)*8,(0.,)*8,(True,)*8);p=m.sample(c,o,2_000_000_000,2_010_000_000,.9,.1);receipt=m.finalize(c,p,'proposal_applied',1_030_000_000,2_040_000_000,p.proposal_action);new=replace(o,logits=(0.,-740.,0.,0.,0.,0.,0.,0.));result=m.update_likelihood(receipt,new,'d'*64,'a'*64,'b'*64);q=m.distribution(new).q
limit=dict(kind='syntactically_accepted_but_not_authenticated_original_forward_input',old_q=p.distribution.q,new_logits=new.logits,new_q=q,selected_action=result.action,ratio=result.ratio,returned_kl_repr=repr(result.old_to_new_kl),returned_kl_finite=math.isfinite(result.old_to_new_kl),stable_log_difference_kl=math.fsum(x*(math.log(x)-math.log(y)) for x,y in zip(p.distribution.q,q) if x>0),scope='Not a reachable valid frozen original actor forward under this fixture; demonstrates generic API finite-input domain exceeds supported provenance. Not an observed training/runtime failure.',future_requirement='Authenticate original actor forward/mask/prior and reject nonfinite update statistics; a future generic numerical helper should compute KL via log differences and finite validation.')
(A/'numeric_domain_limit.json').write_text(json.dumps(limit,indent=2,allow_nan=False)+'\n')
(A/'commands.txt').write_text('\n'.join([
 'PYTHONDONTWRITEBYTECODE=1 '+str(W/'.venv/bin/python')+' -m pytest -q -p no:cacheprovider '+str(W/'scripts/astra_gazebo/tests/test_route_training_proposal_m10.py')+' > '+str(A/'author_tests_replay.log')+' 2>&1',
 'PYTHONDONTWRITEBYTECODE=1 '+str(W/'.venv/bin/python')+' -m pytest -q -p no:cacheprovider '+str(A/'test_independent.py')+' > '+str(A/'tests_attempt02.log')+' 2>&1',
 'PYTHONDONTWRITEBYTECODE=1 '+str(W/'.venv/bin/python')+' '+str(A/'native_math_check.py')+' > '+str(A/'native_math_check.log')+' 2>&1'])+'\n')
checks={
 'all16_author_resource_hashes_and_frozen_source_match':True,
 '32_author_tests_replayed':True,
 'all255_nonempty_masks_independent80digit_branch_enumeration':True,
 'q_mass_support_and_no_artificial_stop':True,
 'branch_and_CDF_exact_halfopen_boundaries':True,
 'fixed_partition_score_zero_expectation_and_independent_CPU_gradient':True,
 '76_actual_unchanged_original_gate_cases_mask_and_thresholds':True,
 'old_new_marginal_ratio_current_gate_and_KL_on_supported_inputs':True,
 'proposal_label_distinct_from_latent_and_shield_execution':True,
 'receipt_tamper_and_invalid_dispositions_rejected':True,
 'integer_actor_transaction_source_age_limits_kept':True,
 'train_and_deterministic_deployment_distinct_identity':True,
 'zero_mask_reject_and_single_mask_no_alternative_claim':True,
 'generic_numeric_domain_limitation_explicit_and_preserved':True,
 'no_model_or_environment_training_or_ROS':True}
limits=[
 'This is a pure proposal distribution and receipt-reference component, not a differentiable trainer, optimizer, environment/collector, reward/GAE implementation, checkpoint loader, or live execution qualification.',
 'EPSILON=.20 is a fixed offline candidate choice, not a tuned or qualified safe exploration rate. Nonanalytic proposal mass equals epsilon*(1-pi(analytic)) only in the collapsed zero-residual partition; epsilon itself is not nonanalytic action frequency.',
 'q marginalization requires downstream transition/reward to depend on proposal action, not hidden branch/latent after conditioning. Branch-dependent rewards require a different joint objective.',
 'logit_score is the almost-everywhere fixed-gate derivative only. Gate changes can jump q; recomputing new membership gives the correct ratio but no smooth PPO/monotonic-improvement guarantee. No straight-through gradient is justified.',
 'Training stochastic q and deterministic deployment argmax+residual gate have different behavior/state visitation. Proposal-space fitting cannot alone establish deployment improvement or long-horizon off-policy validity.',
 'Context hashes, true mask/prior, actual actor outputs, RNG draw independence and timestamp authenticity must be checked by a future trusted collector. The module cannot prove them or recover the six real scan frames from a hash.',
 'Excluding invalid/deadline PG rows is an explicit engineering rule, not proof of an unbiased estimator when availability/deadlines depend on policy parameters/action. All failure rewards, critic targets and earlier valid credit must remain in future M11; this file does not implement sequencing.',
 'Generic finite inputs beyond authenticated actual-forward provenance can make KL nonfinite. Saved -740 logit counterexample returns inf KL while chosen ratio remains finite; stable log-difference reference is finite. This violates claimed actual original forward provenance for the fixture and is not classified as an observed actor/training fault, but forbids treating public ActorOutput.validate as complete numeric/provenance authentication.',
 'Finite precision softmax and inverse CDF follow the explicitly declared binary64 sampler, not arbitrary torch float32 categorical sampling. Future collector/trainer must use the same definition for draw and likelihood; no probability floor introduced.',
 'Inherited M08 full-chain69.239986ms>50ms and environment compatibility limitation are historical reported negatives, not repaired or rerun by this review. No real execution support, timing qualification, learning, safety or performance result exists here.']
resources={x['path']:x for x in author['resources']};resources[str(author_path)]=bind(author_path)
for name in ['source_snapshot.py','author_tests_replay.log','test_independent.py','test_independent_attempt01_snapshot.py','tests_attempt01.log','tests_attempt02.log','native_math_check.py','native_math_check.log','native_math_result.json','numeric_domain_limit.json','commands.txt']:
 resources[str(A/name)]=bind(A/name)
for p,x in resources.items():assert sha(p)==x['sha256']
report=dict(schema='astra_route_training_proposal_m10_independent_review_v1',status='independent_M10_pure_math_component_pass_with_explicit_provenance_and_numerical_domain_limits',independent_review=True,pass_within_declared_actor_output_precondition=True,generic_finite_input_numerical_robustness_qualified=False,reviewed_source=author['source'],author_report=bind(author_path),checks=checks,check_count=len(checks),test_evidence=dict(author32_pass=True,independent_tests=18,independent_positive_tests=17,independent_limit_reproduction=1,mask_patterns=255,actual_original_gate_cases=76,CPU_tensor_max_score_error=2.220446049250313e-16),first_test_failure=dict(log=bind(A/'tests_attempt01.log'),cause='Independent harness called math.nextafter absent in Python3.8; no source/math failure.',correction='Use exact preceding binary64 via struct bit representation; no component/gate changes.',preserved_test_source=bind(A/'test_independent_attempt01_snapshot.py')),numeric_domain_limit=bind(A/'numeric_domain_limit.json'),limitations=limits,training_policy_id=m.TRAIN_ID,deployment_policy_id=m.DEPLOY_ID,original_sources_changed=False,actor_instances_created_by_independent_native_test=0,optimizer_created=False,training_steps=0,simulation_episodes=0,ROS_started=False,training_authorized=False,execution_authorized=False,resources=list(resources.values()))
with (A/'review.json').open('x') as f:json.dump(report,f,indent=2,allow_nan=False);f.write('\n')
md='''M10 纯概率/receipt组件在声明的“实际原actor输出”前提下通过独立审查；不授予训练或运行资格。

32项作者测试复跑通过。新增18项独立测试包括17项正向/故障检查和1项明确数值域限制复现；用80位Decimal独立枚举全部255个非空mask的branch×latent分布，验证q质量、mask支持和epsilon下界。原SensorActor.choose_actions真实方法的76个masked/float32门边界案例一致；独立CPU scatter-add/autograd的score最大误差2.220446049250313e-16。没有创建actor实例、优化器、学习步骤、ROS或仿真。

数学核对：q(a)=0.2π(a)+0.8Σ[G(z)=a]π(z)。固定gate分区内的score公式正确；跨门更新使用当前gate重建q，保持proposal标签，不能替换成latent概率或shield执行标签。训练混合分布与确定性部署属于不同目标/identity，不推导部署收益。隐含branch/latent只能用于审计，若奖励依赖这些隐藏量，当前边缘化目标就不适用。

整数20ms actor、50ms transaction、200ms真实source年龄门保持；异常/discard不产生PG行，timely shield响应保留原proposal。但调用者仍须证明六帧历史、forward来源、mask/prior、RNG独立性、各时间与hash真实。receipt guard不执行超时中断，也不实现失败奖励/critic/GAE的保留；参数相关deadline筛选下无无偏性保证。

保留的数值域边界：构造与原forward不一致的logits[0,-740,0,…]，公开ActorOutput.validate通过，update_likelihood的KL使用p/newq先做除法可溢出，返回inf；同例log差式KL有限。它不是冻结actor合法forward的真实运行故障，不能用来声称训练失败；却说明该公开校验器并不认证来源或覆盖任意有限输入。未来trainer必须验证实际forward并拒绝非有限统计；若扩展为通用helper，应独立修复稳定KL数值计算。本次不改e034源码。

首轮独立测试1项失败来自Python3.8没有math.nextafter；原log和测试快照已留存，改为struct精确相邻binary64后18项通过。作者旧1ULP测试调整记录也保持。现有M08全链69.239986ms>50ms与环境负例只是继承限制，没有被此概率组件修复。本报告绑定27项实际资源；训练/仿真均0。
'''
md=md.replace('27项实际资源',str(len(resources))+'项实际资源')
with (A/'review.md').open('x') as f:f.write(md)
print(json.dumps({'report':bind(A/'review.json'),'markdown':bind(A/'review.md'),'status':report['status'],'resource_count':len(resources),'limit':limit},indent=2))
