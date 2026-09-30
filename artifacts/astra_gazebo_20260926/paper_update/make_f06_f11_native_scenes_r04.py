#!/usr/bin/env python3
"""Six N05 development scene panels: actual camera crop plus input geometry.

The geometry panel is reconstructed from the frozen scene manifest and is
visibly separate from the real Gazebo frame. These are six task families, not
six newly independent parent geometries or formal-performance evidence.
"""

import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import yaml


STAGE = Path(__file__).resolve().parents[1]
PREPARED = STAGE/'contracts/navigation_n05_prepared_r01/prepared.json'
RAW_QC = STAGE/'paper_update/raw_qc/navigation_n05_contact_sheets_r03/manifest.json'
OUT_ROOT = STAGE/'paper_update/figures'
CASES = [
    ('F06', 'b00', '静态绕障', 'core_static_avoidance'),
    ('F07', 'b02', '单参与者横穿', 'core_single_crossing'),
    ('F08', 'b04', '迎面让行', 'core_head_on_yield'),
    ('F09', 'b06', '连续交互', 'core_sequential_interaction'),
    ('F10', 'b08', '遮挡出现', 'core_partially_occluded'),
    ('F11', 'b10', '宽度变化走廊', 'core_width_change'),
]
CROP = (150, 0, 1950, 1440)  # fixed full-height ROI; no actor cut by crop


def binding(path):
    path = Path(path).resolve(strict=True)
    return {'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def main():
    prepared = json.loads(PREPARED.read_text())
    qc = json.loads(RAW_QC.read_text())
    rows = {r['run_id']: r for r in qc['rows']}
    plt.rcParams.update({'font.family':'Noto Sans CJK JP','font.size':7.4,
                         'pdf.fonttype':42,'svg.fonttype':'none'})
    for fid, block, title, scene_id in CASES:
        matches = [u for u in prepared['units'] if '_'+block+'_' in u['run_id']
                   and u['method'] == 'astra_analytic_route_v1']
        assert len(matches) == 1
        unit = matches[0]
        record = rows[unit['run_id']]
        assert record['integrity_pass'] is True and record['image_status'] == 'two_valid_native'
        native = record['images'][0]
        assert native['valid_native'] is True and native['source_ns'] == 8500000000
        image = Path(native['path'])
        assert binding(image)['sha256'] == native['sha256']
        contract_path = Path(unit['files'][0]['path'])
        contract = json.loads(contract_path.read_text())
        assert contract['scene_id'] == scene_id
        scene_path = Path(contract['scene_manifest']['path'])
        scene = yaml.safe_load(scene_path.read_text())
        assert scene['scene_id'] == scene_id and scene['new_independent_geometry'] is False
        expected_actors = len(scene['actor_geometry'])
        out = OUT_ROOT/(fid+'_n05_native_scene_r04')
        if out.exists():
            raise FileExistsError('preserve prior figure: '+str(out))
        out.mkdir(parents=True)
        fig = plt.figure(figsize=(6.7, 5.35), dpi=300)
        imax = fig.add_axes([.025,.22,.63,.59])
        geom = fig.add_axes([.69,.22,.285,.59])
        native_pixels = plt.imread(str(image))
        assert native_pixels.shape[:2] == (1440,2560)
        x0,y0,x1,y1 = CROP
        imax.imshow(native_pixels[y0:y1,x0:x1])
        imax.axis('off')
        imax.set_title('真实 Gazebo 原生帧 · 源时间 8.50 s',fontsize=8.0,pad=5)
        geom.set_title('场景输入俯视几何 · 非截图',fontsize=8.0,pad=5)
        for box in scene['static_boxes']:
            geom.add_patch(Rectangle((box['x']-box['length']/2,box['y']-box['width']/2),
                                     box['length'],box['width'],facecolor='#838F98',edgecolor='#59656B',linewidth=.4))
        for actor in scene['actor_geometry']:
            sx,sy=actor['start'];ex,ey=actor['end']
            geom.annotate('',xy=(ex,ey),xytext=(sx,sy),
                          arrowprops=dict(arrowstyle='->',color='#C86E42',lw=1.25))
            geom.plot(sx,sy,'o',ms=3.6,color='#C86E42')
        start=scene['route']['start'];goal=scene['route']['goal']
        geom.plot(start[0],start[1],marker='s',ms=5.2,color='#1262C8')
        geom.plot(goal[0],goal[1],marker='*',ms=7.0,color='#24836B')
        geom.plot([start[0],goal[0]],[start[1],goal[1]],ls='--',lw=.8,color='#6D8BB7')
        geom.set(xlim=(-7,7),ylim=(-5.6,5.6),xlabel='x / m',ylabel='y / m')
        geom.set_aspect('equal',adjustable='box')
        geom.grid(alpha=.2,lw=.4)
        geom.tick_params(labelsize=6)
        fig.suptitle(f'{fid} {title}：N05 开发场景与真实仿真画面',fontsize=11,weight='bold',y=.96)
        fig.text(.5,.155,
                 f'共同观察尺度；蓝色=机器人起点，绿色=目标，橙色={expected_actors}名参与者输入路径，灰色=静态障碍。',
                 ha='center',fontsize=6.55,color='#41515C')
        fig.text(.5,.075,
                 f'{block} / control / seed {unit["seed"]} / {record["task_outcome"]}；'
                 '固定裁切 x=150–1950、全高 y=0–1440 px；原始 2560×1440 帧保留。',
                 ha='center',fontsize=6.45,color='#6D4C43')
        fig.text(.5,.035,'旧开发几何与失败保留；俯视几何来自场景清单，不将新 seed 当作新布局。',
                 ha='center',fontsize=6.4,color='#6D4C43')
        for ext in ('png','pdf','svg'):
            fig.savefig(out/(fid+'.'+ext),dpi=600 if ext=='png' else None)
        plt.close(fig)
        result={
            'schema':'astra_n05_six_family_native_scene_figure_v1',
            'figure_id':fid,'run_id':unit['run_id'],'seed':unit['seed'],'scene_id':scene_id,
            'family':scene['family'],'layout_id':scene['layout_id'],
            'source_time_ns':native['source_ns'],'actor_geometry_count':expected_actors,
            'crop_xyxy_original_pixels':list(CROP),'original_native_dimensions':[2560,1440],
            'claim_scope':'actual N05 development frame plus separate scene-input geometry; no new independent geometry or formal performance claim',
            'original_native_frame':binding(image),'scene_manifest':binding(scene_path),
            'contract':binding(contract_path),'prepared':binding(PREPARED),
            'prior_native_qc':binding(RAW_QC),'generator':binding(Path(__file__)),
            'images':{ext:binding(out/(fid+'.'+ext)) for ext in ('png','pdf','svg')},
        }
        (out/'source_manifest.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        print(fid, out)


if __name__ == '__main__':
    main()
