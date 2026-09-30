#!/usr/bin/env python3
import json,hashlib
from pathlib import Path
D=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
d=json.loads((D/'terminal_tail_diagnosis.json').read_text());b=json.loads((D/'budget_binding_audit.json').read_text());k=d['runs']['m12_physics_p03_s009_kd6'];rows=[]
for arm in ('kd4','kd6'):
 rid='m12_physics_p03_s009_'+arm;x=json.loads((D/'rows'/rid/'independent_raw_diagnosis.json').read_text());p=json.loads((D/'rows'/rid/'protocol_audit.json').read_text());rows.append('| '+rid+' | '+str(p['passed'])+'/'+str(p['count'])+' | {:.9f} | {:.9f} |'.format(x['steering']['links']['center_rate_radps']['max_abs'],x['steering']['dense_links']['center_rate_radps']['max_abs']))
text='''# P03 原始证据独立结果审查

P03 按冻结规则在第二单元基础设施失败后终止，2 单元已消费、4 单元退休。原失败不改、不重跑、不补样；未取得完整三对矩阵，也未晋级 Kd6。所有分析只读离线完成，没有启动 ROS/Gazebo。

## 精确故障归因

Kd6 正式区间为 1.110–37.110 s。raw dense_links 最后源时间为 37.124 s；已关闭 bag 则完整保留 37.126–37.144 s 共十个遗漏尾包。共享的全部 raw/bag payload 完全相同；正式区间不存在 raw-only、bag-only 或 payload 差异。

- links_matches_dense_same_stamp：仅 37.142 s 的低率 link 找不到同源时刻 dense。
- actual_joint_positions_agree_with_dense_source_pose：仅 37.140 s joint 的最新 dense 为37.124 s，龄期16 ms超过原2 ms门。
- actual_odom_twist_agrees_with_dense_base_link：仅37.140 s odom缺少同源时刻 dense。

三项故障均位于正式结束后。独立将**原 raw 低率数据与 bag 原始 dense 数据对照**时，三项整段均成立；这只是定位证据，不将 bag 包补写到 raw 或替换原资格评估。匹配 joint 最大角差为 {:.9f} rad（原门0.01），odom 体轴速度误差最大{:.3g}。contacts 另有37.134–37.144 s六个采集尾包遗漏，也全部在正式结束后。

冻结 probe 的 finally（astra_physical_actuation_probe_p02.py:585–600）暂停后只等待每条流尾时刻≥formal_end，然后 active=false。该条件不能保证异步队列包含暂停时已发布的所有尾包，且 source_packet_integrity（同文件:128–199）检查全部raw。因此已收低率尾包可越过某条尚未排空的dense流。本次bag完整、raw截断，支持“probe关闭采集提前”的定位，而不是源物理状态不一致。

## 原评估与诊断边界

两个原 assessor 输出均逐字段精确重放。Kd4保持119/120通过、实际中心角变化率失败。Kd6原结果只评估54项，三项上述失败，assessment_incomplete=true。

经父代理明确授权，AST派生诊断只在这**恰好三项**失败时越过唯一早退条件，其他AST字段完全不变；raw不改，三项false及总pass=false保持。诊断得到117/120通过，无额外物理失败，中心角20 ms峰值{:.9f} rad/s，原门0.85。它不能把原infra失败改为资格通过，也不能满足3/3处理组晋级。

| 回合 | 正式时段协议子集 | 实际20 ms峰值 rad/s | 补充2 ms峰值 rad/s |
|---|---:|---:|---:|
'''.format(k['raw_cross_stream_recomputation']['maximum_matched_joint_angle_error_rad'],k['raw_cross_stream_recomputation']['maximum_matched_odom_body_twist_error'],k['diagnostic_continuation']['result']['metrics']['actual_center_rate_radps']['maximum_absolute'])+'\n'.join(rows)+'''

两行只有同seed的一对，第二行原资格不完整。差异是保留的离线机制观察；无统计性能、一般安全或真实车辆结论。实际2 ms峰值为补充量，不替代冻结20 ms门。

每回合640个正式命令、639个源时间间隔均为50,000,000 ns；整数32–36 s loss区间没有源命令，bag navigation/relay在原drain宽限后也无命令。每回合正式backend_status1800个，20 ms等间隔且与odom/target同ns；十条header流正式窗口raw/bag完整一致。source事件时刻是Python publish前采样；Bool和relay仍为接收时钟，不能等同驱动精确源时刻。

## 预算与绑定

预算/绑定14项检查全部通过：420个release依赖当前SHA与归档SHA均匹配；两回合receipt、raw、bag、contract、bindings精确校验；6个registry身份各一次，后四个退休身份没有运行目录。cleanup无owned活进程，端口释放，bag已关闭。release SHA为085224c0f1bea30542e070f8ea9ebc33f65ad0feced9997a0e7cf8a194d434c0。

## 新身份的最小测量修复建议

保持物理控制、命令序列和门限。pause成功后由GetPhysicsProperties.pause核验暂停状态，并取GetWorldProperties稳定sim_time整数ns作为终止watermark，核对2 ms格点。500 Hz三流必须完整到watermark；50 Hz各流按自己的首源时刻计算最后应有slot，保留world的2 ms相位与driver的0 ms相位。终止collector前在同一锁内验证完整格点、首尾与三项join，再active=false；保留原5 s wall deadline。服务超时、未暂停、时钟回退、缺包均失败，不能裁剪尾部或静默退回formal_end。

服务字段更正：早期建议误将paused字段归给GetWorldProperties；该服务只有sim_time等世界信息，暂停状态实际来自GetPhysicsProperties.pause。此更正不影响原始证据或诊断。

P03中最后bag clock仅能用于离线检验该设计，不冒称本次已经记录GetWorldProperties回复。Kd6离线watermark37.144 s要求world末包37.142 s、driver末包37.140 s，只有dense_links和contacts落后，精确对应本次故障。

## 图表

预注册PHY01要求三control峰值中位pair作为代表；目前只有一对已运行，规则不可执行，标记not_complete，无替代选择或补造缺失点。TIM01另展示P01到P02的测量修复；两个不同fresh单回合不是因果统计。

## 复现

命令、脚本与输入SHA保存在terminal_tail_diagnosis.json、budget_binding_audit.json及rows/*/{independent_raw_diagnosis,protocol_audit}.json。所有derived输出都在本审计目录。raw、report、receipt和release文件未修改。
'''
(D/'REPORT.md').write_text(text)
errors=[{'command':'read-only discovery: ls artifacts/astra_gazebo_20260926/releases/*p03*','exit_code':2,'error':'No such file or directory; actual release discovered under contracts/m12_physics_p03_release.json','effect':'no writes/no data changes'},{'command':'read-only metadata discovery attempted runs/m12_physics_p03_s009_kd4/code_bindings.json','exit_code':1,'error':'FileNotFoundError; actual file discovered as bindings.json','effect':'no writes/no data changes','stderr_warning':'requests dependency warning observed in read-only shell interpreter; audit computation uses /usr/bin/python3 and succeeded'}]
(D/'initial_tool_errors.json').write_text(json.dumps(errors,indent=2,ensure_ascii=False)+'\n')
manifest={'scope':'P03 independent offline audit; original evidence frozen','files':[{'path':str(p),'sha256':sha(p)} for p in sorted(D.rglob('*')) if p.is_file() and p.name!='source_manifest.json'],'original_inputs':d['inputs'],'commands':[d['command'],'/usr/bin/python3 '+str(D/'audit_budget_bindings.py')]+['/usr/bin/python3 '+str(D/'audit_protocol_row.py')+' --run-id m12_physics_p03_s009_'+a for a in ('kd4','kd6')]}
(D/'source_manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n');print(sha(D/'REPORT.md'))
