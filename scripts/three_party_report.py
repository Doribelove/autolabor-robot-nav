#!/usr/bin/env python3
"""Auditable arithmetic and editable DOCX/CSV reports; no ROS control imports."""
import csv
import datetime as dt
from fractions import Fraction
import json
from pathlib import Path
import uuid
import zipfile
from xml.sax.saxutils import escape

from three_party_test import (LIGHTS, SCENES, POINTS, TEST_PLAN, OPEN_PHASES, read_json,
                              write_json, session_dirs, metric, locked, clock_pair)


def ratio(value):
    return None if value is None else {'numerator': str(value.numerator), 'denominator': str(value.denominator), 'display': format(float(value), '.6f')}


def pending(s, check):
    missing = []
    obstacle = s.get('kind') == 'obstacle'
    if not s.get('light'): missing.append('本次光照类别')
    if not s.get('scene'): missing.append('本次场景')
    if not s.get('coordinates'): missing.append('本次真实 GPS 坐标')
    elif not s['coordinates'].get('crs'): missing.append('坐标系')
    if not s.get('illuminance'): missing.append('仪器实测照度')
    people = s.get('personnel') or {}
    if not all(people.get(k) for k in ('party_a', 'party_b', 'party_c')): missing.append('三方人员信息')
    if not s.get('task_started_at'): missing.append('巡检任务实际启动事件/时刻' if obstacle else '模块实际启动事件/时刻')
    end = s.get('obstacle', {}).get('completed_at') if obstacle else s.get('recovery_completed_at')
    if not (end or (s.get('recovery_completed_at') if obstacle else None)):
        missing.append('绕障后继续行驶的实际完成时刻及来源' if obstacle else '回收实际完成时刻及来源')
    if not s.get('evidence'): missing.append('用户提供的影像证据')
    elif any(not Path(e['path']).is_file() for e in s['evidence']): missing.append('影像文件路径不可读取')
    if any(e.get('position_in_file') in (None, '', '待补') for e in s.get('evidence', [])): missing.append('影像对应时间位置')
    if s.get('steps_conform') is None: missing.append('步骤符合预期的核验')
    if not s.get('gnss_quality'): missing.append('定位 RTK 质量证据（当前 NavSatFix 不区分固定解）')
    if not check.get('recording_integrity_ok'): missing.append('bag 完整性或关键话题数据')
    if check.get('task_time_covered') is not True: missing.append('bag 覆盖正式任务起止时间')
    if any(not v for v in check.get('critical_time_coverage', {}).values()): missing.append('关键话题未覆盖整个正式任务')
    if not s.get('recording_started_at') or not s.get('recording_stopped_at'): missing.append('录制实际起止时间')
    if s.get('kind') == 'recovery':
        if s.get('recovery_success') is None: missing.append('真实回收结果')
        if metric(s) is None: missing.append('有效正式耗时 Ti')
    elif s.get('kind') == 'obstacle':
        o = s.get('obstacle', {})
        for key in ('object_id', 'length_m', 'width_m', 'height_m', 'encounter_at', 'bypassed', 'continued'):
            if o.get(key) is None: missing.append('障碍物 ' + key)
        if not check.get('pre_obstacle_actual_speed', {}).get('samples'): missing.append('遇障前实际车速反馈')
        if o.get('evidence_schema') == 'report_20261017':
            for category, label in (('illuminance', '照度计照片'), ('obstacle', '障碍物照片'), ('client_speed', '遇障前客户端实际车速截图')):
                if not any(e.get('category') == category for e in s.get('evidence', [])):
                    missing.append(label)
            frames = {(e['path'], e.get('position_in_file')) for e in s.get('evidence', []) if e.get('category') == 'bypass_frame'}
            if len(frames) < 3: missing.append('绕障过程 3～4 张连续图片及时间位置')
    elif s.get('kind') == 'verification':
        if not s.get('verification', {}).get('result'): missing.append('倒退复查判定结果')
        if s.get('recovery_success') is None: missing.append('用户实物回收核验')
        if check.get('verification_time_covered') is not True: missing.append('bag 覆盖倒退复查全过程')
        if any(not v for v in check.get('verification_critical_time_coverage', {}).values()): missing.append('复查关键话题时间覆盖')
    if any(a.get('kind') == 'clock_discontinuity' for a in s.get('anomalies', [])): missing.append('时间基准跳变复核')
    if any(a.get('kind') == 'multiple_module_starts' for a in s.get('anomalies', [])): missing.append('同一次记录出现多次模块启动，需复核独立测试边界')
    return missing


def summarize(s, check):
    efficiency = metric(s) if s['kind'] == 'recovery' else None
    start, end = s.get('task_started_at'), s.get('recovery_completed_at')
    if s['kind'] == 'obstacle':
        end = s.get('obstacle', {}).get('completed_at') or end
    duration = Fraction(end['epoch_ns'] - start['epoch_ns'], 10**9) if start and end and end['epoch_ns'] > start['epoch_ns'] else None
    missing = pending(s, check)
    return {'session_id': s['session_id'], 'kind': s['kind'], 'light': s['light'], 'scene': s['scene'], 'point': s['point'],
            'point_description': s['point_description'], 'attempt': s['attempt'], 'retest_of': s.get('retest_of'), 'phase': s['phase'],
            'area_m2': s['area_m2'], 'coordinates': s.get('coordinates'), 'illuminance': s.get('illuminance'), 'personnel': s.get('personnel'),
            'recording_started_at': s.get('recording_started_at'), 'task_started_at': start,
            'recovery_completed_at': s.get('recovery_completed_at'), 'task_completed_at': end,
            'recording_stopped_at': s.get('recording_stopped_at'), 'software_completed_at': s.get('software_completed_at'),
            'duration_seconds': ratio(duration), 'efficiency_m2_s': ratio(efficiency),
            'single_efficiency_ge_50': None if efficiency is None else efficiency >= 50,
            'single_duration_reference_s': None if s['kind']!='recovery' else '374.8726' if s['scene']=='机坪' else '457.635',
            'recovery_success': s.get('recovery_success'), 'steps_conform': s.get('steps_conform'),
            'record_status': '记录不足/待补' if missing else '完整', 'missing': missing,
            'evidence': s.get('evidence', []), 'anomalies': s.get('anomalies', []), 'obstacle': s.get('obstacle', {}),
            'verification': s.get('verification'),
            'verification_started_at': s.get('verification_started_at'),
            'verification_completed_at': s.get('verification_completed_at'),
            'original_recovery_finished_at': s.get('original_recovery_finished_at'),
            'pre_obstacle_actual_speed': check.get('pre_obstacle_actual_speed')}


def calculate(sessions, checks, adoptions):
    rows = [summarize(s, checks.get(s['session_id'], {})) for s in sessions]
    index = {s['session_id']: s for s in sessions}
    selected, adoption_notes = {}, {}
    for s in sorted(sessions, key=lambda s: (s['attempt'], s['session_id'])):
        if s['kind'] == 'recovery' and s['point'] not in POINTS:
            continue  # Retain legacy records in all rows; do not rename/reassign.
        key = (s['kind'], s['light'], s['scene'], s['point'])
        selected.setdefault(key, s['session_id'])
    for entry in adoptions:
        if entry['session_id'] in index:
            s = index[entry['session_id']]
            key = (s['kind'], s['light'], s['scene'], s['point'])
            if list(key) == entry['key'] and key in selected:
                selected[key] = entry['session_id']
                adoption_notes[entry['session_id']] = entry
    selected_ids = set(selected.values())
    for row in rows:
        row['plan_status'] = '当前计划' if row['kind'] != 'recovery' or row['point'] in POINTS else '历史点位保留；不属于当前 pt1～pt6 计划'
        if row['kind'] == 'verification':
            row['plan_status'] = '垃圾回收判定演示；不计入 48 点效率测试'
        row['adopted'] = row['session_id'] in selected_ids
        row['adoption_basis'] = adoption_notes.get(row['session_id'], '默认首次尝试' if row['adopted'] else '保留但未采用')
    by_id = {row['session_id']: row for row in rows}
    progress, groups, scene_results = [], [], []
    for light in LIGHTS:
        for scene in SCENES:
            values, ids = [], []
            for point in POINTS:
                sid = selected.get(('recovery', light, scene, point))
                row = by_id.get(sid)
                attempts = [s['session_id'] for s in sessions if (s['kind'], s['light'], s['scene'], s['point']) == ('recovery', light, scene, point)]
                value = metric(index[sid]) if sid else None
                values.append(value); ids.append(sid)
                progress.append({'light': light, 'scene': scene, 'point': point, 'selected_attempt': sid, 'all_attempts': attempts,
                                 'status': ('未测试' if row is None else row['phase'] + ' / ' + row['record_status']),
                                 'missing': row['missing'] if row else ['尚未进行本次测试'],
                                 'recovery_success': row['recovery_success'] if row else None})
            mean = sum(values, Fraction()) / len(POINTS) if all(v is not None for v in values) else None
            groups.append({'light': light, 'scene': scene, 'selected_attempts': ids, 'average_efficiency': ratio(mean),
                           'efficiency_ge_50': None if mean is None else mean >= 50,
                           'all_recovered': all(sid and by_id[sid]['recovery_success'] is True for sid in ids),
                           'evidence_complete': all(sid and not by_id[sid]['missing'] for sid in ids)})
    for scene in SCENES:
        group = [g for g in groups if g['scene'] == scene]
        values = [Fraction(int(g['average_efficiency']['numerator']), int(g['average_efficiency']['denominator'])) if g['average_efficiency'] else None for g in group]
        mean = sum(values, Fraction()) / 4 if all(v is not None for v in values) else None
        scene_results.append({'scene': scene, 'four_light_average': ratio(mean), 'average_ge_50': None if mean is None else mean >= 50,
                              'each_light_ge_50': None if any(v is None for v in values) else all(v >= 50 for v in values),
                              'all_recovered': all(g['all_recovered'] for g in group), 'evidence_complete': all(g['evidence_complete'] for g in group)})
    obstacles = []
    previous_obstacles = []
    for row in [r for r in rows if r['kind'] == 'obstacle']:
        o = row['obstacle']
        dimensions = [o.get(k) for k in ('length_m', 'width_m', 'height_m')]
        dimensions_ok = None if any(v is None for v in dimensions) else all(Fraction(str(v)) >= minimum for v, minimum in zip(dimensions, (Fraction('0.2'), Fraction('0.2'), Fraction('0.5'))))
        location = row['coordinates']
        point = None if not location else tuple(Fraction(location[k]['decimal_degrees']) for k in ('longitude', 'latitude'))
        different = None if not point or not o.get('object_id') else all(o['object_id'] != old[0] and point != old[1] for old in previous_obstacles)
        if point and o.get('object_id'): previous_obstacles.append((o['object_id'], point))
        obstacles.append({'session_id': row['session_id'], 'light': row['light'], 'adopted': row['adopted'], 'dimensions_ok': dimensions_ok,
                          'changed_object_and_position': different, 'bypassed': o.get('bypassed'), 'continued': o.get('continued'),
                          'complete': not row['missing'], 'missing': row['missing']})
    obstacle_lights = {light: any(o['adopted'] and o['light'] == light and o['dimensions_ok'] and o['changed_object_and_position'] and o['bypassed'] is True and o['continued'] is True and o['complete'] for o in obstacles) for light in LIGHTS}
    known_failure = (any(g['efficiency_ge_50'] is False for g in groups)
                     or any(r['adopted'] and (r['recovery_success'] is False or r['steps_conform'] is False) for r in rows)
                     or any(o['adopted'] and any(o[k] is False for k in ('dimensions_ok', 'changed_object_and_position', 'bypassed', 'continued')) for o in obstacles))
    complete = all(g['efficiency_ge_50'] and g['all_recovered'] and g['evidence_complete'] for g in groups) and all(obstacle_lights.values())
    conclusion = ('当前有不合格项；所有失败及重测均保留，采用依据见明细。' if known_failure else
                  '采集结果满足回收及避障数据判据；正式模板、三方签字盖章仍待确认。' if complete else
                  '待补/待核验；效率、真实回收、步骤符合和证据完整须分别确认，避障四种光照另行验收。')
    reference = read_json(Path(__file__).resolve().parents[1] / 'config/obstacle_capture.json', {}).get('report_reference')
    template_status = ('已读取用户测试用例：' + reference['path'] + '；避障按原文记录，原文件保持不变。回收点数沿用用户后续 pt1～pt6 更正；本报告仍为待核验工作稿。'
                       if reference else '待补：按用户 2026-10-14 指令大纲生成，正式三方模板待提供')
    return {'test_plan': TEST_PLAN, 'rows': rows, 'progress': progress, 'groups': groups, 'scenes': scene_results, 'obstacles': obstacles,
            'obstacle_light_complete': obstacle_lights, 'adoptions': adoptions,
            'recovery_slots_with_attempts': sum(bool(p['all_attempts']) for p in progress),
            'recovery_attempts': sum(s['kind'] == 'recovery' for s in sessions),
            'template_status': template_status, 'report_reference': reference,
            'overall_conclusion': conclusion}


def display(value):
    if value is None: return '待补'
    if isinstance(value, bool): return '是' if value else '否'
    if isinstance(value, dict) and 'display' in value: return value['display']
    if isinstance(value, dict) and 'iso' in value: return value['iso']
    if isinstance(value, (list, dict)): return json.dumps(value, ensure_ascii=False)
    return str(value)


def write_csv(path, rows, fields):
    with Path(path).open('w', encoding='utf-8-sig', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            values = {key: display(row.get(key)) for key in fields}
            # Files and user-entered labels are data, not spreadsheet formulas.
            values = {key: "'" + v if v[:1] in ('=', '+', '-', '@') else v for key, v in values.items()}
            writer.writerow(values)


def write_docx(path, sections):
    ns = 'http://schemas.openxmlformats.org/wordprocessingml/2006/main'
    def paragraph(text, bold=False):
        props = '<w:rPr><w:b/></w:rPr>' if bold else ''
        return '<w:p><w:r>' + props + '<w:t xml:space="preserve">' + escape(str(text)) + '</w:t></w:r></w:p>'
    body = []
    for title, text, headers, rows in sections:
        body.extend((paragraph(title, True), paragraph(text)))
        if headers:
            table = '<w:tbl><w:tblPr><w:tblStyle w:val="TableGrid"/></w:tblPr>'
            for row in [headers] + rows:
                table += '<w:tr>' + ''.join('<w:tc><w:tcPr/><w:p><w:r><w:t xml:space="preserve">' + escape(display(c)) + '</w:t></w:r></w:p></w:tc>' for c in row) + '</w:tr>'
            body.append(table + '</w:tbl>')
    document = '<?xml version="1.0" encoding="UTF-8" standalone="yes"?><w:document xmlns:w="' + ns + '"><w:body>' + ''.join(body) + '<w:sectPr><w:pgSz w:w="16838" w:h="11906" w:orient="landscape"/><w:pgMar w:top="720" w:bottom="720" w:left="720" w:right="720"/></w:sectPr></w:body></w:document>'
    types = '<?xml version="1.0"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="xml" ContentType="application/xml"/><Override PartName="/word/document.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.document.main+xml"/><Override PartName="/word/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.styles+xml"/></Types>'
    rels = '<?xml version="1.0"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="word/document.xml"/></Relationships>'
    styles = '<?xml version="1.0"?><w:styles xmlns:w="' + ns + '"><w:docDefaults><w:rPrDefault><w:rPr><w:rFonts w:ascii="Calibri" w:eastAsia="宋体"/><w:sz w:val="20"/></w:rPr></w:rPrDefault></w:docDefaults><w:style w:type="table" w:styleId="TableGrid"><w:name w:val="Table Grid"/><w:tblPr><w:tblBorders>' + ''.join('<w:' + x + ' w:val="single" w:sz="4"/>' for x in ('top','bottom','left','right','insideH','insideV')) + '</w:tblBorders></w:tblPr></w:style></w:styles>'
    docrels = '<?xml version="1.0"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" Target="styles.xml"/></Relationships>'
    with zipfile.ZipFile(path, 'w', zipfile.ZIP_DEFLATED) as z:
        for name, data in {'[Content_Types].xml': types, '_rels/.rels': rels, 'word/document.xml': document,
                           'word/styles.xml': styles, 'word/_rels/document.xml.rels': docrels}.items(): z.writestr(name, data.encode())


def report(root, force=False):
    root = Path(root)
    if not force and read_json(root / 'report_policy.json', {}).get('defer_reports'):
        return {'report_directory': None, 'report_deferred': True,
                'message': '按用户要求暂不更新报告，保留原始记录，待统一汇总'}
    with locked(root / 'report.lock'):
        directories = session_dirs(root)
        sessions = [read_json(p / 'session.json') for p in directories]
        checks = {p.name: read_json(p / 'check.json', {}) for p in directories}
        adoptions = [json.loads(line) for line in (root / 'adoptions.jsonl').read_text().splitlines() if line] if (root / 'adoptions.jsonl').exists() else []
        result = calculate(sessions, checks, adoptions)
        result['generated_at'] = clock_pair()
        destination = root / 'reports' / (dt.datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:6])
        destination.mkdir(parents=True, exist_ok=False)
        write_json(destination / 'results.json', result)
        write_csv(destination / '测试明细.csv', result['rows'], ['session_id','kind','light','scene','point','attempt','plan_status','adopted','adoption_basis','area_m2','coordinates','illuminance','personnel','recording_started_at','task_started_at','recovery_completed_at','recording_stopped_at','duration_seconds','efficiency_m2_s','recovery_success','steps_conform','verification','verification_started_at','verification_completed_at','record_status','missing','anomalies'])
        write_csv(destination / '分组结果.csv', result['groups'], ['light','scene','selected_attempts','average_efficiency','efficiency_ge_50','all_recovered','evidence_complete'])
        write_csv(destination / '测试进度.csv', result['progress'], ['light','scene','point','selected_attempt','all_attempts','status','missing'])
        write_csv(destination / '避障明细.csv', [r for r in result['rows'] if r['kind'] == 'obstacle'],
                  ['session_id','light','scene','attempt','adopted','adoption_basis','coordinates','illuminance',
                   'task_started_at','task_completed_at','obstacle','pre_obstacle_actual_speed','steps_conform','evidence','record_status','missing'])
        evidence = [dict(e, session_id=r['session_id']) for r in result['rows'] for e in r['evidence']]
        dates = sorted({r['task_started_at']['iso'][:10] for r in result['rows'] if (r.get('task_started_at') or {}).get('iso')})
        people = {r['session_id']: r['personnel'] for r in result['rows'] if r.get('personnel')}
        write_csv(destination / '证据索引.csv', evidence, ['session_id','evidence_id','category','path','position_in_file','actual_time','source','bytes','sha256'])
        sections = [
            ('三方测试报告（可编辑工作稿）', result['template_status'] + '。生成时间：' + result['generated_at']['system_iso'], [], []),
            ('测试对象、配置与方法', 'Jetson AGX Orin 64GB、Ubuntu 20.04、ROS Noetic；实际版本、话题类型和参数见各次 capture_config.json/parameters.json。用户布置单个 FOD 并提供真实 GPS、照度和影像；Codex 被动采集。签字盖章待实际确认。', [], []),
            ('场景及点位', '按用户 2026-10-14 更正使用 pt1～pt6。四种光照 × 两场景 × 六点，共 48 次独立回收；每次只回收一个 FOD。原 pt1/pt2 位置说明保留；pt3～pt6 位置待用户提供，不自动挪用旧 pt0。', ['场景','面积㎡',*POINTS], [[k,v[0],*v[1]] for k,v in SCENES.items()]),
            ('计时与计算方法', 'Ti＝实际回收完成－模块实际启动。ηi＝S/Ti；六点效率取算术平均（ηpt1＋ηpt2＋ηpt3＋ηpt4＋ηpt5＋ηpt6）/6，四种光照组再取算术平均，均要求≥50㎡/s，每种光照独立判定。滑行道457.635s、机坪374.8726s为单次控制参考，不是录制时限。软件到点不代表实物入仓；聊天收到时间仅为人工报告时间。计算使用未舍入有理数。', [], []),
            ('48 点进度', '历史调试包不自动计入这 48 次；旧 pt0 保留原编号并在全部明细中标记为历史点位，不参与当前分组，不重命名。所有重测及失败均保留。', ['光照','场景','点位','采用编号','状态','缺失项'], [[p[k] for k in ('light','scene','point','selected_attempt','status','missing')] for p in result['progress']]),
            ('全部尝试明细', '完整坐标、照度、人员、四个时间、证据和异常见测试明细.csv；以下不隐藏任何失败或未采用尝试。', ['编号','光照/场景/点','Ti秒','η㎡/s','实际回收','证据状态'], [[r['session_id'],'/'.join(r[k] or '待补' for k in ('light','scene','point')),r['duration_seconds'],r['efficiency_m2_s'],r['recovery_success'],r['record_status']] for r in result['rows']]),
            ('各光照组效率', '效率达标、回收成功与证据完整分别列示。', ['光照','场景','平均η㎡/s','效率≥50','六个均回收','证据完整'], [[g[k] for k in ('light','scene','average_efficiency','efficiency_ge_50','all_recovered','evidence_complete')] for g in result['groups']]),
            ('场景最终平均', '其他光照的高效率不能覆盖不合格光照组。', ['场景','四光照平均η','平均≥50','每光照均≥50','全部回收','证据完整'], [[g[k] for k in ('scene','four_light_average','average_ge_50','each_light_ge_50','all_recovered','evidence_complete')] for g in result['scenes']]),
            ('避障测试', '四种光照至少各一次，每次更换障碍物和位置，尺寸至少0.2×0.2×0.5m；记录尺寸、坐标、照度、遇障前实际车速、绕过并继续行驶及影像。48次回收不能替代避障验收。', ['光照','完整合格记录'], [[k,v] for k,v in result['obstacle_light_complete'].items()]),
            ('异常、补测及采用依据', json.dumps({'adoptions':adoptions,'anomalies':[{r['session_id']:r['anomalies']} for r in result['rows'] if r['anomalies']]},ensure_ascii=False), [], []),
            ('证据索引', '只关联用户提供的已有文件；不启动录屏或拍摄设备。', ['证据编号','本机路径','视频位置/时间','来源'], [[e.get(k) for k in ('evidence_id','path','position_in_file','source')] for e in evidence]),
            ('总体结论与签署', result['overall_conclusion'], ['项目','实际信息'], [['测试用例来源',result.get('report_reference') or '待补'],['测试日期',dates or '待各次实际起止时刻确认'],['三方单位及人员',people or '待补'],['签字','待实际签署'],['盖章','待实际盖章']])]
        for row in result['rows']:
            details = [(k, row.get(k)) for k in ('plan_status','adopted','adoption_basis','area_m2','point_description','coordinates','illuminance','personnel',
                'recording_started_at','task_started_at','software_completed_at','recovery_completed_at','task_completed_at','recording_stopped_at',
                'duration_seconds','efficiency_m2_s','recovery_success','steps_conform','verification','verification_started_at','verification_completed_at','obstacle','pre_obstacle_actual_speed','missing')]
            sections.append(('单次记录：'+row['session_id'], ' / '.join(row[k] or '待补' for k in ('light','scene','point')), ['项目','实际数据及来源'], details))
        write_docx(destination / '三方测试报告.docx', sections)
        write_json(root / 'latest_report.json', {'directory': str(destination), 'generated_at': result['generated_at']})
        write_json(root / 'progress.json', result['progress'])
        return {'report_directory': str(destination), 'recovery_slots_with_attempts': result['recovery_slots_with_attempts'],
                'recovery_slots_total': len(result['progress']), 'recovery_attempts': result['recovery_attempts'], 'obstacle_light_complete': result['obstacle_light_complete'],
                'template_status': result['template_status']}
