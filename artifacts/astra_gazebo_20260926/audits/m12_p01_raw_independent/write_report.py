#!/usr/bin/env python3
"""Render the P01 Chinese diagnosis from audited JSON without manual numbers."""
from pathlib import Path
import hashlib
import json

H=Path(__file__).resolve().parent
S=H.parents[1]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def f(v,d=6):return format(v,'.'+str(d)+'f')
r=json.loads((H/'independent_raw_diagnosis.json').read_text())
d=json.loads((H/'frozen_assessor_continuation.json').read_text())
for item in r['inputs']+d['inputs']:
    assert sha(item['path'])==item['sha256'],item['path']
low=r['steering']['links'];dense=r['steering']['dense_links'];m=r['motion'];fault=r['fault_timing'];cmd=r['command_timing'];result=d['diagnostic_result']
lines=['# M12-P01 独立离线诊断','',
'P01 保持原终态失败：`pass=false`、`assessment_incomplete=true`。这份诊断不修改运行目录，不更改冻结门，不消费新 seed，不启动 ROS。',
'',
f"冻结 probe 的原结果已精确复演。唯一修改 AST 的第 {d['transformation']['line']} 行提前返回谓词：只有接收时钟 Bool 完整性这一项为 false 且其余前置项全部为 true 时，才继续算原运动门；该项仍保留 false，总 pass 仍为 false。恢复这个谓词后，全文件 AST 与原文件一致。诊断得到 {len(result['checks'])} 项检查中的 {sum(result['checks'].values())} 项 true、{len(d['diagnostic_failed_checks'])} 项 false。",
'',
'| 保留的失败 | 直接证据 |','|---|---|',
f"| Bool 观察完整性 | probe 最大接收 ROS clock 间隔 {f(max(x['gap_s'] for x in fault['raw_gap_over_0_06_s']))} s，超过冻结 0.06 s。 |",
f"| 实际中心转向速率 | 50 Hz 峰值 {f(low['center_rate_radps']['max_abs'],9)} rad/s，超过冻结 0.85；共 {low['fixed_0_85_rate_exceedances']} 个 20 ms 区间。 |",
f"| command loss 阶段仍有发布 | {f(cmd['loss_start_s'],2)} s 发布上一阶段的 1.2 m/s 命令；运行时浮点减法得到 {cmd['boundary_float_elapsed'][0]!r} s，仍落在前一阶段。 |",
f"| 同阶段命令截止时间 | {len(cmd['same_phase_gaps_over_frozen_limit'])} 个间隙超过冻结 0.075 s，最大 {f(max(x['gap_sim_s'] for x in cmd['same_phase_gaps_over_frozen_limit']))} s；对应 wall 时间也延迟。 |",
'',
'## 真实转向与目标参考分离','',
f"原 50 Hz 峰值发生在源时间 {f(low['peak_interval']['a_ns']/1e9,3)}→{f(low['peak_interval']['b_ns']/1e9,3)} s（zero_speed_yaw 回正阶段）。独立父子四元数恢复左右实际角，再分别逆 Ackermann 曲率取均值得到中心角。用四元数 Z twist 重算，50 Hz 速率最大差仅 {f(low['maximum_z_twist_vs_yaw_rate_difference_radps'],9)} rad/s，不能用 Euler 投影解释超限。",
'',
f"附加 500 Hz 数据在 {f(dense['peak_interval']['a_ns']/1e9,3)}→{f(dense['peak_interval']['b_ns']/1e9,3)} s 得到 {f(dense['center_rate_radps']['max_abs'],9)} rad/s；{dense['fixed_0_85_rate_exceedances']} 个 2 ms 区间超过 0.85。这只是补充诊断，不替代原 50 Hz 资格门。实际中心角最大绝对值 {f(dense['center_rad']['max_abs'])} rad。原左右关节 URDF 角度/速度、中心角度、左右曲率一致性及稳态转向门均通过诊断。",
'',
f"目标中心速率范围 [{f(m['target_center_rate_radps']['min'])}, {f(m['target_center_rate_radps']['max'])}] rad/s，目标加速度范围 [{f(m['target_speed_acceleration_mps2']['min'])}, {f(m['target_speed_acceleration_mps2']['max'])}] m/s²，保持参考边界。实际与目标速度在全部 {m['nonzero_actual_target_speed_difference_count']} 个同源样本中都存在大于 1e-6 的差异；最大绝对差 {f(m['actual_minus_target_speed_mps']['max_abs'])} m/s。实际 odom 与同 ns dense base link 世界速度逆旋转后最大误差 {m['same_source_odom_vs_dense_body_velocity_max_error']:.3g}，证明本次 odom 实际读物理状态，并非把参考值改名发布。",
'',
'## 速度与制动','',
f"实际纵向速度范围 [{f(m['actual_odom_speed_mps']['min'])}, {f(m['actual_odom_speed_mps']['max'])}] m/s；20 ms 差分加速度 [{f(m['actual_odom_acceleration_mps2']['min'])}, {f(m['actual_odom_acceleration_mps2']['max'])}] m/s²，jerk [{f(m['actual_odom_jerk_mps3']['min'])}, {f(m['actual_odom_jerk_mps3']['max'])}] m/s³。原合同未给实际加速度/jerk 资格门，因此只报告测量，不能把参考 0.6/0.9 边界当实际物理保证。",
'',
'| 停止阶段 | 自阶段开始持续低于 0.02 m/s 的首次时刻 | 至此模型路径 | 尾段速度范围 |','|---|---:|---:|---:|']
for p in r['phase_metrics']:
    if p['name'] in ['zero_speed_yaw','commanded_brake','command_loss']:
        lines.append(f"| {p['name']} | {f(p['first_persistent_under_0_02_elapsed_s'],3)} s | {f(p['path_until_persistent_stop_m'],4)} m | [{f(p['tail_speed_mps']['min'])}, {f(p['tail_speed_mps']['max'])}] m/s |")
lines+=['',
'所有原稳态速度、尾段停止、模型静止与转弯激励门通过诊断。commanded_brake 是零速普通命令；command_loss 在 timeout 后使用被动轮速制动、清 PI 积分，因此实际停止时长不同并不矛盾。停止路径从该阶段第一个 50 Hz 模型样本累计，含采样边界误差；不是连续制动距离上界。',
'',
'## 源时间、bag 对照与 Bool 失败机制','',
f"正式窗口内 dense model/link/contact 各 {r['source_streams']['dense_models']['count']} 个样本，间隔精确 2,000,000 ns；旧 model/link 各 {r['source_streams']['models']['count']} 个样本、间隔精确 20,000,000 ns；odom/joint/target/commanded effort 各 {r['source_streams']['odom']['count']} 个样本、同样精确 20 ms。低频每包都与同 ns dense names/poses/twists 完全相同。50 Hz body 相位为 2 ms modulo 20 ms，而 driver 相位为 0 ms；同源比较使用 dense 匹配，不伪造二者同一时刻。",
'',
'浮点 start=0.12000000000000001 会将精确 ns=120000000 的某些首包排除；整数纳秒窗口敏感性计数已另存。该边界变化不影响峰值结论，也没有在本轮回写冻结评价。',
'',
'| topic | raw / bag | 共享包逐字段一致 | bag 缺少的 raw 包 |','|---|---:|---:|---:|']
for topic,v in r['rosbag_exact_payload_crosschecks'].items():
    lines.append(f"| `{topic}` | {v['raw_count']} / {v['bag_count']} | {v['shared_exact_payload']} / {v['shared']} | {len(v['raw_only_ns'])} |")
lines+=['',
'odom/joint/target/effort 的 bag 缺包仅在启动 0.02–0.18 s；raw 已记录。其中 0.12–0.18 s 落在整数纳秒正式窗口内，不能称 bag 全时段闭合。dense raw 少的末尾 link/contact 包都在 36.12 s 正式窗口结束后。所有共享 payload 无差异，明确保留两套采集边界。',
'',
f"probe Bool 共 {fault['raw_count']} 条、全部 false，存在 {fault['raw_duplicate_receipt_count']} 个重复接收时刻；四次大间隙后多条回调挤到同一 ROS clock。独立 bag 共 {fault['bag_count']} 条、全部 false，接收间隔最大 {f(fault['bag_receipt_gap_s']['max'])} s，无对应长间隙。bag 最早接收时间 {f(fault['bag_first_receipt_s'],3)} s。",
'',
'这支持“probe 进程回调/主循环调度及同步 JSON 写入发生延迟”的诊断，弱化“driver 停发 Bool”假说：单独 bag 订阅者仍按约 20 ms 收到 Bool。原 publisher queue=1 仍有潜在丢包风险，但本次数据不能将长 gap 归因为实际丢包。Bool 无 header/seq，raw 与 bag 不能逐消息证明源时间完整性；不把 count 相等或全 false 当完整无 fault 的证明。具体 GIL、I/O 或 OS 调度贡献未被 profiler 记录，保持未决。',
'',
'冻结 probe 的 line 374 在 record 锁内同步 json.dumps/stream.write；line 448–466 的主循环与所有回调共享 Python 进程。命令最长 gap 同时具有 wall 延迟并被 bag 证实，不能仅靠给 Bool 加 source stamp 修复。阶段边界应在新身份中统一整数 ns 判定；本轮始终保留现有边界失败。',
'',
'## 接触范围与可用结论','',
f"500 Hz contact 包完整保留，累计四轮地面接触 pair {sum(v['count'] for v in r['contact_counts']['pairs'])} 次；非轮地面 {r['contact_counts']['nonwheel_ground']}、机器人非地面接触 {r['contact_counts']['nonground']}。这是该空阔地资格回合的观测，不代表导航安全、连续扫掠安全或实车资格。commanded torque 仅为命令扭矩，不是测得反作用力。",
'',
'本次支持实际有限扭矩状态测量与 reference/actual 分离，尚未通过执行资格：接收完整性、实际转向速率、命令时序与边界四项保留失败。下一候选若仅改 source-stamped fault instrumentation，仍不能据此宣称运动门通过；控制器或调度修补须独立新身份与原门检验，不能复跑 P01。',
'',
'## 可复核文件与命令','',
'- `diagnose_frozen_assessor.py`：精确 AST 变换、原结果复演、原门继续诊断。',
'- `recompute_raw.py`：独立四元数/逆 Ackermann、源 ns、速度/制动、接触、bag payload 复算。',
'- `source_steering.csv`、`source_target_actual.csv`、`phase_tail_summary.csv`：全部由 raw 自动产生。',
'- `frozen_assessor_continuation.json`、`independent_raw_diagnosis.json`：输入 SHA、命令、语义限制与所有负面。',
'',
'```bash',d['command'],r['command'],'/usr/bin/python3 '+str(Path(__file__).resolve()),'```','',
'原运行目录所有输入已再次 SHA 复核不变。没有初次失败调用被抹除：本组两支复算脚本首次执行均成功；更早 probe fixtures 的失败版本保留在父目录 prior1–prior3，最后静态 source hash 旧期待值失败保留 initial。','']
(H/'REPORT.md').write_text('\n'.join(lines))
inputs={v['path']:v['sha256'] for v in r['inputs']+d['inputs']}
files=[p for p in H.iterdir() if p.is_file() and p.name!='source_manifest.json']
manifest={'status':'offline_diagnostic_complete','sources':[{'path':p,'sha256':v} for p,v in sorted(inputs.items())],
          'outputs':[{'path':str(p),'sha256':sha(p)} for p in sorted(files)],'raw_input_rehash_all_match':True,
          'no_ROS_started':True,'no_run_files_modified':True,'canonical_P01_pass':False}
(H/'source_manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n')
print(json.dumps({'report_sha256':sha(H/'REPORT.md'),'manifest_sha256':sha(H/'source_manifest.json')},indent=2))
