#!/usr/bin/env python3
"""Chinese P02 closed-run audit from computed data, without manual statistics."""
from pathlib import Path
import hashlib
import json

D=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
r=json.loads((D/'independent_raw_diagnosis.json').read_text());q=json.loads((D/'protocol_audit.json').read_text())
for p in r['inputs']+q['inputs']:assert sha(p['path'])==p['sha256']
low=r['steering']['links'];dense=r['steering']['dense_links'];motion=r['motion'];peak=low['peak_interval']
lines=['# P02 独立 raw / bag 审查','',
f"独立测量/协议检查 {q['passed']}/{q['count']} 通过；冻结评估器精确复演为 {q['frozen_assessor_pass_count']}/{q['frozen_assessor_checks']} 项通过，唯一失败 `{q['frozen_failed_checks'][0]}`。P02 总 pass 仍为 false，不重跑、不改数据或门。",
'',
'本轮测量与命令执行证据可支持后续独立新身份的物理控制候选评价；这不是 P02 整体物理资格通过，也不是导航性能或实车资格。所有审查只读，未启动 ROS/Gazebo。',
'',
'| 项目 | 独立复算 |','|---|---|',
f"| 正式区间 | {q['formal_start_ns']/1e9:.3f}–{q['formal_end_ns']/1e9:.3f} s，36 s |",
f"| 命令 | {q['formal_source_command_count']} 条；{q['source_command_gaps_ns']['count']} 个相邻间隔均为 {q['source_command_gaps_ns']['minimum']} ns；missed_slots 最大 {q['maximum_missed_slots_field']} |",
f"| loss 区间 | {q['loss_start_ns']/1e9:.3f}–{q['formal_end_ns']/1e9:.3f} s；source event、bag导航输入及relay均无发送 |",
f"| 新 physical_status | 正式 {q['source_status_formal_count']} 包，间隔精确 {q['source_status_formal_gaps_ns']['minimum']} ns；与odom/target源stamp列表完全相同 |",
f"| 正式 clock | {q['formal_clock_count']} 点，覆盖含首尾，间隔精确 {q['formal_clock_gaps_ns']['minimum']} ns |",
f"| 原 50 Hz 实际中心转向峰值 | {low['center_rate_radps']['max_abs']:.12f} rad/s，原门0.85；超限区间 {low['fixed_0_85_rate_exceedances']} 个 |",
f"| 附加 500 Hz 峰值 | {dense['center_rate_radps']['max_abs']:.12f} rad/s；仅补充诊断，不替代50Hz门 |",
'',
f"原20ms峰值源区间为 {peak['a_ns']/1e9:.3f}→{peak['b_ns']/1e9:.3f} s，属于 zero_speed_yaw 回正阶段。四元数Z twist独立算法的50Hz最大速率差仅 {low['maximum_z_twist_vs_yaw_rate_difference_radps']:.9f} rad/s，不能用Euler投影解释失败。实际中心角最大绝对值 {dense['center_rad']['max_abs']:.6f} rad。",
'',
f"target加速度 [{motion['target_speed_acceleration_mps2']['min']:.6f}, {motion['target_speed_acceleration_mps2']['max']:.6f}] m/s²、target slew [{motion['target_center_rate_radps']['min']:.6f}, {motion['target_center_rate_radps']['max']:.6f}] rad/s；实际加速度 [{motion['actual_odom_acceleration_mps2']['min']:.6f}, {motion['actual_odom_acceleration_mps2']['max']:.6f}] m/s²。实际速度/加速度来自物理观测，参考边界不能当实际保证；实际加速度/jerk没有事前资格门。",
'',
'| topic | raw / bag | 共享包完全相同 | 正式窗口缺包 |','|---|---:|---:|---:|']
for topic,v in r['rosbag_exact_payload_crosschecks'].items():
    lines.append(f"| `{topic}` | {v['raw_count']} / {v['bag_count']} | {v['shared_exact_payload']} / {v['shared']} | {len(v['formal_window_raw_only_ns'])+len(v['formal_window_bag_only_ns'])} |")
for topic,v in q['status_event_bag_crosschecks'].items():
    if topic=='/m2_driver/physical_status':continue
    lines.append(f"| `{topic}` | {v['raw_count']} / {v['bag_count']} | {v['shared']-len(v['payload_mismatch_keys'])} / {v['shared']} | {len(v['formal_raw_only_keys'])+len(v['formal_bag_only_keys'])} |")
lines+=['',
f"录包节点在 unpause 前完成全部26个topic的TCPROS incoming连接；warmup源区间 {q['warmup']['start_ns']/1e9:.3f}→{q['warmup']['end_ns']/1e9:.3f} s，随后Trigger、running与正式start顺序成立。source状态完整为bootstrap/running/complete，标记与Trigger身份一致，终态零命令恰一条并在正式结束之后。P01启动期bag缺包没有在P02正式区间复现。contacts的raw尾部缺包只在正式结束之后，保留在JSON而不称全文件完全相等。",
'',
f"旧Bool仍有 {len(r['fault_timing']['raw_gap_over_0_06_s'])} 个大于0.06s的collector接收间隙，最大 {max(x['gap_s'] for x in r['fault_timing']['raw_gap_over_0_06_s']):.3f} s；独立bag Bool周期仍约20ms，且新带header状态源包完整。这支持旧Bool接收时钟受collector调度影响的解释，不能再将该间隙当driver源停发；P02隔离命令源本轮没有P01的同窗主循环延迟。Bool无header仍不具备逐条精确源时间证明。",
'',
f"四轮地面接触pair累计 {sum(p['count'] for p in r['contact_counts']['pairs'])} 次，非轮地面 {r['contact_counts']['nonwheel_ground']}、非地面机器人接触 {r['contact_counts']['nonground']}；ground support保留而未误作障碍碰撞。全物理门与负面原样保留。",
'',
'`recompute_raw.py` 进行独立四元数、动力学、接触和共享payload复算；`audit_protocol.py` 重演冻结评估器，并独立检查整数命令、录包完整覆盖与source状态。CSV全部自动来自raw。初稿r0沿用的P01主循环因果措辞已修正并保留，数值结果未变。',
'',
'```bash',r['command'],q['command'],'/usr/bin/python3 '+str(Path(__file__).resolve()),'```','']
(D/'REPORT.md').write_text('\n'.join(lines))
source={x['path']:x['sha256'] for x in r['inputs']+q['inputs']}
manifest={'status':q['status'],'sources':[{'path':p,'sha256':v} for p,v in sorted(source.items())],'outputs':[{'path':str(p),'sha256':sha(p)} for p in sorted(D.iterdir()) if p.is_file() and p.name!='source_manifest.json'],'source_rehash_all_match':True,'no_ROS_started':True,'no_run_files_modified':True,'P02_overall_pass':False}
(D/'source_manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n')
print(json.dumps({'report_sha256':sha(D/'REPORT.md'),'manifest_sha256':sha(D/'source_manifest.json')},indent=2))
