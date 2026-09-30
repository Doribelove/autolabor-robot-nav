#!/usr/bin/python3
"""Independent read-only Q04 mass diagnosis. Writes only this audit's new outputs."""
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
import numpy as np
import rosbag

sys.dont_write_bytecode = True
S = Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926')
W = S / 'workspace'
R = S / 'runs/passive_sweep_q04_static'
O = S / 'audits/passive_q04_result_independent'
I = O / 'mass_diagnosis_inputs'
sys.path.insert(0, str(W / 'scripts/astra_gazebo'))
import dense_sweep
import dense_sweep_p03

def sha(p):
    h = hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda: f.read(4 * 1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()

def bind(p):
    p = Path(p)
    return dict(path=str(p.resolve()), sha256=sha(p), bytes=p.stat().st_size)

def serial(value):
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)

def matrix(origin):
    out = np.eye(4)
    if origin is None:
        return out
    r, p, y = map(float, origin.get('rpy', '0 0 0').split())
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    out[:3, :3] = rz @ ry @ rx
    out[:3, 3] = list(map(float, origin.get('xyz', '0 0 0').split()))
    return out

def main():
    output = O / 'independent_mass_diagnosis.json'
    if output.exists():
        raise FileExistsError('Preserve completed audit; do not overwrite it')
    sources = [R / n for n in ('fixture.bag', 'robot_description_runtime.xml', 'assessment.json', 'run_closure.json', 'sweep_evaluation/report.json')]
    sources += [W / 'scripts/astra_gazebo' / n for n in ('dense_sweep.py', 'dense_sweep_p03.py', 'geometry.py')]
    sources += [W / 'src/simulation/m2_gazebo/src/astra_physics_metadata_p03_plugin.cpp', Path(__file__)]
    before = [bind(p) for p in sources]
    xml = (R / 'robot_description_runtime.xml').read_text()
    root = ET.fromstring(xml)
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}
    assignment = {}
    def locate(name):
        if name in assignment:
            return assignment[name]
        joint = joints.get(name)
        if joint is None or joint.get('type') != 'fixed':
            value = name, np.eye(4)
        else:
            parent, transform = locate(joint.find('parent').get('link'))
            value = parent, transform @ matrix(joint.find('origin'))
        assignment[name] = value
        return value
    bodies = {}
    for link in root.findall('link'):
        name = link.get('name')
        body, transform = locate(name)
        row = bodies.setdefault(body, dict(members=[], inertial_members=[], collisions=[]))
        row['members'].append(name)
        inertial = link.find('inertial')
        if inertial is not None:
            pose = transform @ matrix(inertial.find('origin'))
            mass = float(inertial.find('mass').get('value'))
            a = {k: float(v) for k, v in inertial.find('inertia').attrib.items()}
            tensor = np.array([[a['ixx'], a['ixy'], a['ixz']], [a['ixy'], a['iyy'], a['iyz']], [a['ixz'], a['iyz'], a['izz']]])
            row['inertial_members'].append(dict(link=name, mass_kg=mass, com_in_body_m=pose[:3, 3].tolist(), rotated_inertia=(pose[:3, :3] @ tensor @ pose[:3, :3].T).tolist()))
        for collision in link.findall('collision'):
            center = (transform @ matrix(collision.find('origin')))[:3, 3]
            geometry = collision.find('geometry')
            box, cylinder = geometry.find('box'), geometry.find('cylinder')
            if box is not None:
                extent = math.sqrt(sum((float(x) / 2) ** 2 for x in box.get('size').split()))
            elif cylinder is not None:
                extent = math.hypot(float(cylinder.get('radius')) / math.cos(math.pi / 48), float(cylinder.get('length')) / 2)
            else:
                raise ValueError('unsupported collision geometry')
            row['collisions'].append(dict(source_link=name, center_m=center.tolist(), extent_m=extent))
    for row in bodies.values():
        mass = math.fsum(x['mass_kg'] for x in row['inertial_members'])
        com = np.array([math.fsum(x['mass_kg'] * x['com_in_body_m'][i] for x in row['inertial_members']) / mass for i in range(3)])
        tensor = np.zeros((3, 3))
        for item in row['inertial_members']:
            d = np.array(item['com_in_body_m']) - com
            tensor += np.array(item['rotated_inertia']) + item['mass_kg'] * ((d @ d) * np.eye(3) - np.outer(d, d))
        row.update(mass_kg=mass, com_local_m=com.tolist(), inertia_at_com=tensor.tolist(), radius_m=max(float(np.linalg.norm(np.array(c['center_m']) - com)) + c['extent_m'] for c in row['collisions']))
    nominal = dense_sweep.robot_body_envelopes(xml)
    metadata, statuses, raw_metadata = [], [], []
    with rosbag.Bag(str(R / 'fixture.bag')) as bag:
        for topic, message, _ in bag.read_messages(topics=['/astra/p03/runtime_metadata', '/astra/p03/step_status']):
            value = json.loads(message.data)
            if topic.endswith('runtime_metadata'):
                metadata.append(value)
                raw_metadata.append(message.data)
            else:
                statuses.append(value)
    window = [1160000000, 11160000000]
    formal = [s for s in statuses if window[0] <= s['stamp_ns'] <= window[1]]
    used = sorted({s['end_revision'] for s in formal})
    command = ['/usr/bin/gz', 'sdf', '-p', str(R / 'robot_description_runtime.xml')]
    converted = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
    (I / 'converted_robot.sdf').write_bytes(converted.stdout)
    (I / 'conversion_stderr.txt').write_bytes(converted.stderr)
    (I / 'runtime_metadata.jsonl').write_text('\n'.join(raw_metadata) + '\n')
    sdf = ET.fromstring(converted.stdout)
    sdf_links = {l.get('name'): l for l in sdf.findall('./model/link')}
    comparisons = []
    for record in metadata:
        if record['revision'] not in used:
            continue
        helper_runtime = dense_sweep_p03.runtime_geometry(record)
        for model in record['config']['models']:
            if model['name'] != 'autolabor_m2':
                continue
            for link in model['links']:
                name = link['name'].split('::')[1]
                own, old = bodies[name], nominal[name]
                inertial = sdf_links[name].find('inertial')
                sdf_com = list(map(float, inertial.findtext('pose', '0 0 0 0 0 0').split()))[:3]
                runtime_radius = 0.
                for c in link['collisions']:
                    distance = math.sqrt(sum((float(c['relative_pose']['xyz'][i]) - float(link['com_local_m'][i])) ** 2 for i in range(3)))
                    extent = math.sqrt(sum((float(x) / 2) ** 2 for x in c['size_xyz_m'])) if c['kind'] == 'box' else math.hypot(float(c['radius_m']) / math.cos(math.pi / 48), float(c['length_m']) / 2)
                    runtime_radius = max(runtime_radius, distance + extent)
                error = lambda a, b: math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))
                comparisons.append(dict(body=name, revision=record['revision'], independent_mass_kg=own['mass_kg'], old_helper_mass_kg=old['mass_kg'], runtime_mass_kg=link['mass_kg'], native_ode_mass_kg=link['native_ode_mass_kg'], native_ode_mass_center=link['native_ode_mass_center'], independent_com_local_m=own['com_local_m'], old_helper_com_local_m=old['com_local_m'], converted_sdf_com_local_m=sdf_com, runtime_com_local_m=link['com_local_m'], mass_abs_error_kg=abs(own['mass_kg'] - link['mass_kg']), com_abs_error_m=error(own['com_local_m'], link['com_local_m']), converted_vs_runtime_com_error_m=error(sdf_com, link['com_local_m']), independent_vs_helper_com_error_m=error(own['com_local_m'], old['com_local_m']), independent_nominal_radius_m=own['radius_m'], old_helper_radius_m=old['radius_m'], runtime_radius_m=runtime_radius, existing_runtime_helper_radius_m=helper_runtime[link['name']]['radius'], radius_delta_runtime_minus_ideal_m=runtime_radius - own['radius_m'], collision_count=len(link['collisions']), independent_nominal_inertia_diagonal=[own['inertia_at_com'][i][i] for i in range(3)], runtime_inertia_diagonal=link['inertia_ixx_iyy_izz_ixy_ixz_iyz'][:3], sdf_inertial_xml=ET.tostring(inertial, encoding='unicode')))
    ldd = subprocess.run(['ldd', '/usr/bin/gz'], capture_output=True, text=True, check=True).stdout
    libraries = []
    for line in ldd.splitlines():
        if any(k in line for k in ('libsdformat', 'libgazebo_common', 'libignition-math', 'libtinyxml', 'libstdc++')):
            match = re.search(r'=>\s+(/\S+)', line)
            if match:
                libraries.append(bind(match.group(1)))
    (I / 'converter_ldd.txt').write_text(ldd)
    old_assess = json.loads((R / 'assessment.json').read_text())
    old_report = json.loads((R / 'sweep_evaluation/report.json').read_text())
    assert before == [bind(p) for p in sources]
    base = next(c for c in comparisons if c['body'] == 'base_link')
    result = dict(schema='astra_q04_independent_mass_diagnosis_v1', status='root_cause_confirmed_offline_historical_failure_preserved', run_id='passive_sweep_q04_static', seed=260926029, independent_review=True,
        scope='Read-only closed bag/URDF; independent matrix/parallel-axis aggregation; offline converter only. No ROS/Gazebo server.',
        historical_assessment=dict(pass_=old_assess['pass'], passed=sum(v is True for v in old_assess['checks'].values()), total=len(old_assess['checks']), closure_issues=old_report['closure_audit']['issues'], preserved=True),
        source_bindings=before, sources_unchanged_after_review=True, formal_window_ns=window,
        metadata_audit=dict(metadata_count=len(metadata), revisions=[m['revision'] for m in metadata], stamps_ns=[m['stamp_ns'] for m in metadata], used_revisions=used, all_status_count=len(statuses), formal_status_count=len(formal), all_used_revisions_present=set(used) <= {m['revision'] for m in metadata}),
        independent_aggregation=dict(method='Independent NumPy RPY transforms, recursive fixed-joint membership, math.fsum mass moments, parallel-axis inertia. Existing helper only comparison.', body_count=len(bodies), total_mass_kg=math.fsum(b['mass_kg'] for b in bodies.values()), bodies=bodies, base_formula='(66*0.27 + 0.5*0.60 + 0.1*0.40)/66.6 = 0.2726726726726727'),
        comparisons=comparisons,
        converter=dict(command=command, exit_code=converted.returncode, tool=bind('/usr/bin/gz'), selected_relevant_ldd_libraries=libraries, binding_scope='Selected disk provenance; not a complete runtime /proc/maps claim.', internal_serialization_source_line_identified=False),
        reproduction_bindings=[bind(I / n) for n in ('reproduce.py', 'converted_robot.sdf', 'conversion_stderr.txt', 'runtime_metadata.jsonl', 'converter_ldd.txt')],
        diagnosis=dict(cause='Installed URDF-to-SDF fixed-joint aggregate COM/inertia quantization, reproduced before observer; old QC compared ideal unrounded URDF values to effective rounded runtime geometry.', missing_mass_or_wrong_assignment=False, observer_json_precision_loss=False, observer_evidence='number() uses std::setprecision(17); converted SDF and actual runtime COM both exactly 0.272673.', old_frozen_tolerance_m=1e-7, base_com_error_m=base['com_abs_error_m'], base_radius_delta_m=base['radius_delta_runtime_minus_ideal_m'], radius_follow_on_gate_would_fail=abs(base['radius_delta_runtime_minus_ideal_m']) > 1e-7, radius_direction='runtime enclosing radius is larger than ideal URDF radius', all_converted_runtime_com_equal=all(c['converted_vs_runtime_com_error_m'] == 0 for c in comparisons), other_bodies_mass_com_pass=all(c['mass_abs_error_kg'] <= 1e-7 and c['com_abs_error_m'] <= 1e-7 for c in comparisons if c['body'] != 'base_link')),
        future_repair_requirements=['Preserve Q04 result/seed/budget and all old assessor bytes; no retrospective qualification.', 'New version must bind actual conversion input/tool/output and compare effective SDF COM/inertia/collision geometry to runtime.', 'Keep exact URDF fixed-joint membership and explicit conversion residuals; do not globally weaken original gate.', 'Recompute radius around effective COM. A conservative nominal/effective union requires common-reference accounting for the COM shift.', 'New offline conversion/mismatch tests are not fresh simulation evidence.'],
        claims=dict(q04_pass=False, qualification_granted=False, new_simulation_started=False, source_or_historical_data_modified=False, formal_safety=False),
        audit_generation_notes=['Initial inline report serialization rejected NumPy bool_ before writing any report; this reproducible script normalizes NumPy scalar types.'])
    output.write_text(json.dumps(result, indent=2, default=serial, allow_nan=False) + '\n')
    (O / 'independent_mass_diagnosis.md').write_text(f'''# Q04 独立质量与质心诊断

固定关节归并与总质量正确；失败由 URDF→SDF 转换时的有效质心量化造成。原 Q04 仍为 11/13，不追认通过、不改旧门槛、不重跑。

实际 bag 的 1 条几何元数据 revision 1 覆盖正式区间全部 {len(formal)} 个 step status。独立矩阵/质量矩/平行轴复算确认 7 个刚体、整车 79.6 kg；base_link 合并 massless base、66 kg chassis、0.5 kg laser、0.1 kg IMU，合计 66.6 kg。

公式 `(66×0.27 + 0.5×0.60 + 0.1×0.40) / 66.6` 得到 `0.2726726726726727 m`，与旧 helper 一致。安装环境对实际 URDF 执行纯转换 `gz sdf -p` 得到 `0.272673 m`，与 bag 精确一致。差值 `{base['com_abs_error_m']:.15g} m` 超过原 `1e-7 m` 核验门槛。元数据使用 17 位输出精度，因此不是 observer JSON 精度丢失。

base_link 理想半径 `{base['independent_nominal_radius_m']:.17g} m`，实际 COM 推得 `{base['runtime_radius_m']:.17g} m`，增加 `{base['radius_delta_runtime_minus_ideal_m']:.15g} m`。仅改质心检查后原半径检查仍会失败；不能沿用偏小的理想半径。其余六个刚体质量/质心检查均通过。惯性张量亦存在转换舍入，逐项值和全部成员见 JSON。

后继版本应冻结实际转换输入、工具、输出，核对有效 SDF 的 COM/惯性/碰撞体与 runtime，同时保留原 URDF 合并关系及量化偏差。需要围绕有效 COM 重算包络，或在同一参考点构造保守并集。这里没有新执行证据，也不改变旧失败。

同目录 `mass_diagnosis_inputs/` 保存可复核脚本、转换命令产生的完整 SDF、stderr、实际 bag 元数据与 ldd 输出。JSON 绑定原数据和这些产物的 SHA，并确认原文件前后未变。转换器内部具体格式化源行未定位；因果定位止于已实证的转换输出边界。
''')
    print(json.dumps(dict(report=str(output), sha256=sha(output), base=base), default=serial))

if __name__ == '__main__':
    main()
