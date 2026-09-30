#!/usr/bin/env python3
"""F04: only the preregistered, closed Q05 static seed031 may be rendered.

This consumes recorded evidence, never runs ROS or an evaluator, and never
substitutes Q04. The first formal source pose and full formal window are fixed.
"""
import argparse
from dataclasses import asdict
import csv
import hashlib
import importlib
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
STAGE = HERE.parents[2]
WORKSPACE = STAGE/'workspace'
SELECTED_RUN = 'passive_sweep_q05_static'
SELECTED_SEED = 260926031
EXPECTED_SCOPE = 'gazebo_normal_euler_extension_with_actor_begin_jumps'
FONT = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
MARGIN_M = .33


def sha(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda:stream.read(4*1024*1024),b''):digest.update(chunk)
    return digest.hexdigest()


def binding(path):
    path=Path(path).resolve()
    return dict(path=str(path),sha256=sha(path),bytes=path.stat().st_size)


def load_json(path):
    return json.loads(Path(path).read_text())


def check_selection(prereg,run,receipt):
    if (prereg.get('figure_id'),prereg.get('selected_future_run'),prereg.get('seed')) != ('F04',SELECTED_RUN,SELECTED_SEED):
        raise ValueError('F04 preregistered identity differs')
    if run.resolve() != (STAGE/'runs'/SELECTED_RUN).resolve():
        raise ValueError('F04 cannot use another run, historical substitute, or synthetic run')
    if (receipt.get('run_id'),receipt.get('seed'),receipt.get('attempt')) != (SELECTED_RUN,SELECTED_SEED,1) or type(receipt.get('seed')) is not int or type(receipt.get('attempt')) is not int:
        raise ValueError('closed Q05 seed031 attempt1 receipt required')
    if type(receipt.get('pass')) is not bool:
        raise ValueError('closed pass/fail outcome absent')


def finite_or_nan(value):
    if value is None or value=='':return math.nan
    value=float(value)
    return value if math.isfinite(value) else math.nan


def scalar_data(report,endpoint_rows,interval_rows):
    """Keep missing/failed evidence visible. Never interpolate across gaps."""
    if report.get('schema')!='astra_p04_engineering_sweep_v1' or report.get('scope')!=EXPECTED_SCOPE:
        raise ValueError('expected P04 evaluator and declared Gazebo extension')
    profile=report['numeric_profile']
    if profile['margin_m']!=MARGIN_M or profile['physics_step_ns']!=2_000_000 or profile['coarse_stride_steps']!=10:
        raise ValueError('geometry margin/time-grid semantics changed')
    start,end=report['source_window_ns']
    if type(start) is not int or type(end) is not int or not 0<start<end:
        raise ValueError('invalid formal source window')
    seen=[];points=[];issues=[]
    for row in endpoint_rows:
        stamp=int(row['stamp_ns'])
        if not start<=stamp<=end or (seen and stamp<=seen[-1]):raise ValueError('endpoint outside/order differs from complete formal window')
        physical=finite_or_nan(row['physical_xy_m']);conservative=finite_or_nan(row['conservative_xy_m'])
        if math.isfinite(physical) and math.isfinite(conservative) and abs(physical-conservative-MARGIN_M)>1e-10:
            raise ValueError('signed physical/conservative margin convention changed')
        seen.append(stamp)
        points.append(dict(stamp_ns=stamp,time_from_formal_start_s=(stamp-start)/1e9,
                           physical_xy_m=physical,conservative_xy_m=conservative))
    expected=list(range(start,end+1,20_000_000))
    if expected[-1]!=end:expected.append(end)
    missing_endpoints=sorted(set(expected)-set(seen))
    if missing_endpoints:issues.append('formal endpoint source slots missing')
    if set(seen)-set(expected):raise ValueError('endpoint source slots not prescribed by evaluator')
    intervals=[];previous=None
    for row in interval_rows:
        a,b=row['start_ns'],row['end_ns']
        if type(a) is not int or type(b) is not int or not start<=a<b<=end or b-a!=2_000_000 or (a-start)%2_000_000 or (previous is not None and a<previous):
            raise ValueError('2ms interval ordering/domain differs')
        previous=b
        fields=('robot_point_budget_m','actor_Begin_variation_budget_m','other_phase_variation_budget_m','relative_clearance_variation_budget_m')
        values={key:float(row[key]) for key in fields}
        if any(not math.isfinite(v) or v<0 for v in values.values()):raise ValueError('invalid variation budget')
        if abs(values[fields[0]]+values[fields[1]]+values[fields[2]]-values[fields[3]])>1e-12:
            raise ValueError('relative variation budget sum mismatch')
        intervals.append(dict(start_ns=a,end_ns=b,time_from_formal_start_s=(a-start)/1e9,**values))
    expected_intervals=set(range(start,end,2_000_000))
    missing_intervals=sorted(expected_intervals-{r['start_ns'] for r in intervals})
    if missing_intervals:issues.append('formal 2ms intervals missing')
    bounds=[];previous=None
    for row in report['coarse_intervals']:
        a,b=row['start_ns'],row['end_ns']
        if not start<=a<b<=end or (previous is not None and a<previous):raise ValueError('coarse bounds overlap/outside formal window')
        previous=b
        bound=finite_or_nan(row['conditional_clearance_lower_bound_m'])
        if row.get('covered_exactly') is not True:bound=math.nan
        bounds.append(dict(start_ns=a,end_ns=b,start_s=(a-start)/1e9,end_s=(b-start)/1e9,
                           lower_bound_m=bound,covered_exactly=row.get('covered_exactly') is True))
    return dict(start_ns=start,end_ns=end,duration_s=(end-start)/1e9,endpoints=points,
                intervals=intervals,bounds=bounds,missing_endpoint_ns=missing_endpoints,
                missing_interval_start_ns=missing_intervals,expected_endpoint_ns=expected,
                expected_interval_start_ns=sorted(expected_intervals),display_issues=issues)


def gapped_series(rows,stamp_field,expected_stamps,value_field,start_ns,scale=1.):
    """NaNs break drawn lines at absent slots; no synthetic measurements."""
    indexed={row[stamp_field]:row for row in rows}
    return ([(stamp-start_ns)/1e9 for stamp in expected_stamps],
            [finite_or_nan(indexed[stamp][value_field])*scale if stamp in indexed else math.nan for stamp in expected_stamps])


def read_first_geometry(run,report,module):
    """Exactly the formal first End packet; no nearest/favorable replacement."""
    import rosbag
    first=report['source_window_ns'][0]
    metadata={};status=None;links=None
    with rosbag.Bag(str(run/'fixture.bag')) as bag:
        for topic,msg,_receipt in bag.read_messages(topics=['/astra/p03/runtime_metadata','/astra/p03/step_status','/astra/p03/end_links']):
            if topic.endswith('/runtime_metadata'):
                row=json.loads(msg.data)
                if row['revision'] in metadata and metadata[row['revision']]!=row:raise ValueError('metadata revision changed')
                metadata[row['revision']]=row
            elif topic.endswith('/step_status'):
                row=json.loads(msg.data)
                if row['stamp_ns']==first:
                    if status is not None:raise ValueError('duplicate formal first step status')
                    status=row
            elif msg.header.stamp.to_nsec()==first:
                if links is not None:raise ValueError('duplicate formal first link packet')
                links=msg
    if links is None or status is None:raise ValueError('formal first source pose/status unavailable; do not substitute adjacent frame')
    record=metadata[status['end_revision']]
    conversion_error=None
    try:
        model=module.parse_converted_model((run/'robot_description_runtime.xml').read_text(),(run/'converted_robot.sdf').read_text())
        geometry=module.runtime_geometry(record,model)
    except (ValueError,KeyError,TypeError) as error:
        # A failed conversion/QC remains a failed result. If the actual native
        # shape record itself is valid, it can still be shown as a diagnostic.
        conversion_error=type(error).__name__+': '+str(error)
        geometry=module.runtime_geometry(record)
    states={name:{'pose':module.engineering_pose((p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w))}
            for name,p in zip(links.name,links.pose)}
    world=module.WorldCollisions((run/'world.sdf').read_text())
    if world.errors:raise ValueError('saved-world collision interpretation failed: '+str(world.errors))
    grounds={ground['name'] for ground in world.grounds}
    robot=module.projections(geometry,states,True,grounds)
    obstacles=module.projections(geometry,states,False,grounds)
    if len(robot)!=9:raise ValueError('F04 preregistered native nine-component robot absent')
    clearance=module.compound_clearance(robot,obstacles,MARGIN_M)
    first_endpoint=next((x for x in report['_endpoint_rows'] if int(x['stamp_ns'])==first),None)
    if first_endpoint is not None:
        expected=finite_or_nan(first_endpoint['physical_xy_m'])
        measured=clearance['minimum_physical_projected_clearance_m']
        if measured is not None and math.isfinite(expected) and abs(measured-expected)>1e-9:
            raise ValueError('first geometry projection does not reproduce saved endpoint')
    base=states['autolabor_m2::base_link']['pose']
    return dict(source_ns=first,metadata_revision=record['revision'],base_pose_world=asdict(base),
                robot_components=[asdict(x) for x in robot],obstacle_components=[asdict(x) for x in obstacles],
                first_clearance_audit=clearance,coordinate_reference='World XY projections reexpressed in first base_link planar frame; actual roll/pitch preserved before projection',
                no_nearest_time_substitution=True,conversion_correspondence_pass=conversion_error is None,conversion_error=conversion_error)


def clean_json(value):
    if isinstance(value,dict):return {str(k):clean_json(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [clean_json(v) for v in value]
    if isinstance(value,np.generic):return clean_json(value.item())
    if isinstance(value,float) and not math.isfinite(value):return None
    return value


def save_csv(path,rows):
    if not rows:return
    with path.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader()
        for row in rows:writer.writerow(clean_json(row))


def safe_review_status(path,assessment_binding):
    if path is None:return '独立复核：待完成',None
    value=load_json(path)
    evidence=[]
    for key in ('source_bindings','evidence_bindings'):
        if isinstance(value.get(key),list):evidence.extend(value[key])
    verified=(value.get('independent_review') is True and value.get('run_id')==SELECTED_RUN
              and any(isinstance(r,dict) and r.get('sha256')==assessment_binding['sha256'] for r in evidence))
    if verified and value.get('pass') is True:return '独立复核：通过',binding(path)
    if verified and value.get('pass') is False:return '独立复核：未通过',binding(path)
    return '独立复核：状态未确认（报告已绑定）',binding(path)


def render(data,geometry,receipt,assessment,report,review_label,output,panels):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from matplotlib.collections import LineCollection
    from matplotlib.lines import Line2D
    from matplotlib.patches import Polygon as PatchPolygon
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    from PIL import Image
    if hasattr(font_manager.fontManager,'addfont'):
        font_manager.fontManager.addfont(str(FONT))
    plt.rcParams.update({'font.family':font_manager.FontProperties(fname=str(FONT)).get_name(),
        'font.size':8.,'axes.titlesize':8.7,'axes.labelsize':8.,'xtick.labelsize':7.5,'ytick.labelsize':7.5,
        'legend.fontsize':7.4,'axes.spines.top':False,'axes.spines.right':False,'axes.linewidth':.6,
        'lines.linewidth':1.,'xtick.major.width':.6,'ytick.major.width':.6,
        'pdf.fonttype':42,'ps.fonttype':42,'svg.fonttype':'path','axes.unicode_minus':False})
    fig=plt.figure(figsize=(170/25.4,155/25.4))
    axa=fig.add_axes([.12,.685,.49,.215]);axb=fig.add_axes([.12,.37,.855,.21]);axc=fig.add_axes([.12,.11,.855,.175])
    state='通过' if receipt['pass'] else '未通过'
    fig.suptitle('实际碰撞几何、裕量与步进净空界',y=.985,fontsize=10)
    fig.text(.5,.95,f'Q05 静态资格 · seed031 · 最终回执：{state}',ha='center',fontsize=7.3,color='#555555')
    axa.set_title('(a) 首个正式源时刻：实际九组件投影',loc='left',pad=7)
    origin=geometry['base_pose_world']['xyz'];rotation=geometry['base_pose_world']['quaternion_xyzw'] if 'quaternion_xyzw' in geometry['base_pose_world'] else None
    # Pose3 stores a quaternion as xyzw; preserve direct record in data export.
    if rotation is None:rotation=geometry['base_pose_world']['xyzw']
    x,y,z,w=rotation;yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    c,s=math.cos(yaw),math.sin(yaw)
    def local(points):
        return [(c*(px-origin[0])+s*(py-origin[1]),-s*(px-origin[0])+c*(py-origin[1])) for px,py in points]
    shapes=[Polygon(local(row['polygon_xy'])) for row in geometry['robot_components']]
    union=unary_union(shapes)
    # Round-join segments circumscribe .33 m arcs with declared tiny excess;
    # this is a visual aid, never the numerical clearance evaluator.
    resolution=256;outer_radius=MARGIN_M/math.cos(math.pi/(4*resolution))
    buffered=union.buffer(outer_radius,resolution=resolution)
    for polygon in ([buffered] if buffered.geom_type=='Polygon' else buffered.geoms):
        axa.add_patch(PatchPolygon(np.asarray(polygon.exterior.coords),facecolor='#D9EAF5',edgecolor='#0072B2',linewidth=.8,linestyle='--'))
    colors=plt.get_cmap('tab10').colors
    labels=[]
    for index,(shape,row) in enumerate(zip(shapes,geometry['robot_components'])):
        axa.add_patch(PatchPolygon(np.asarray(shape.exterior.coords),facecolor=colors[index],alpha=.24,edgecolor=colors[index],linewidth=1.1))
        axa.plot(*shape.exterior.xy,color=colors[index],lw=.8)
        name=row['name'].split('::')[-1]
        labels.append(f'{index+1}. '+name.replace('_fixed_joint_lump__',' / ').replace('_collision',''))
    axa.plot(0,0,'k+',ms=5,mew=.8)
    bounds=buffered.bounds;axa.set_xlim(bounds[0]-.10,bounds[2]+.10);axa.set_ylim(bounds[1]-.10,bounds[3]+.10)
    axa.set_aspect('equal');axa.set_xlabel('首帧基座坐标 x / m');axa.set_ylabel('首帧基座坐标 y / m')
    axa.grid(color='#E6E6E6',lw=.4);axa.set_axisbelow(True)
    names={'chassis':'车体','laser':'激光雷达','imu':'IMU','front_left_steer':'左前转向节','front_right_steer':'右前转向节',
           'front_left_wheel':'左前轮','front_right_wheel':'右前轮','rear_left_wheel':'左后轮','rear_right_wheel':'右后轮'}
    for index,row in enumerate(geometry['robot_components']):
        name=next((zh for token,zh in names.items() if token in row['name']),str(index+1))
        fig.text(.67,.887-index*.020,'■',color=colors[index],fontsize=7.4)
        fig.text(.690,.887-index*.020,name,color='#222222',fontsize=7.4)
    fig.text(.67,.642,'蓝虚线：欧氏 0.33 m 外扩\n+：base_link 原点\n几何示意，非 Gazebo 截图',fontsize=7.3,linespacing=1.6)
    axb.set_title('(b) 全窗口带符号 XY 净空与声明模型下界',loc='left',pad=7)
    x,physical=gapped_series(data['endpoints'],'stamp_ns',data['expected_endpoint_ns'],'physical_xy_m',data['start_ns'])
    _,conservative=gapped_series(data['endpoints'],'stamp_ns',data['expected_endpoint_ns'],'conservative_xy_m',data['start_ns'])
    axb.plot(x,physical,color='#0072B2',marker='.',ms=1.5,label='端点物理投影间距')
    axb.plot(x,conservative,color='#009E73',marker='.',ms=1.5,label='保守间距（减 0.33 m）')
    lines=[[(r['start_s'],r['lower_bound_m']),(r['end_s'],r['lower_bound_m'])] for r in data['bounds'] if math.isfinite(r['lower_bound_m'])]
    axb.add_collection(LineCollection(lines,colors='#D55E00',linewidths=1.0,linestyles='dashed',zorder=4))
    handles,labels=axb.get_legend_handles_labels();handles.append(Line2D([],[],color='#D55E00',ls='--',label='区间工程下界'))
    axb.legend(handles=handles,loc='upper left',bbox_to_anchor=(0,1.02),ncol=3,frameon=False,handlelength=2.0,columnspacing=1.2)
    values=[v for v in physical+conservative+[r['lower_bound_m'] for r in data['bounds']] if math.isfinite(v)]
    if values:
        low=min(0.,min(values));high=max(values);span=max(.1,high-low);axb.set_ylim(low-.06*span,high+.30*span)
    axb.axhline(0,color='#777777',ls=':',lw=.7);axb.set_ylabel('带符号 XY 净空 / m')
    axb.text(.99,.10,'端点为 20 ms；连接线仅辅助阅读',transform=axb.transAxes,ha='right',fontsize=7.2,color='#555555')
    if report['integrity_pass'] is not True:
        axb.text(.99,.14,'完整性未通过：下界仅作失败诊断',transform=axb.transAxes,ha='right',fontsize=7.3,color='#A03520')
    axc.set_title('(c) 完整 2 ms 步进变化预算',loc='left',pad=7)
    for key,label,color,style in (
        ('relative_clearance_variation_budget_m','相对净空总预算','#222222','-'),
        ('robot_point_budget_m','机器人点位','#0072B2','-'),
        ('other_phase_variation_budget_m','其他相位/数值','#777777','--'),
        ('actor_Begin_variation_budget_m','参与者 Begin','#D55E00',':')):
        x,y=gapped_series(data['intervals'],'start_ns',data['expected_interval_start_ns'],key,data['start_ns'],1000)
        axc.plot(x,y,color=color,ls=style,label=label,lw=.9)
    axc.set_ylabel('每 2 ms 位移预算 / mm');axc.set_xlabel('距正式窗口起点的源时间 / s')
    axc.legend(loc='upper left',bbox_to_anchor=(0,1.02),ncol=4,frameon=False,columnspacing=1.,handlelength=2.)
    current=axc.get_ylim();axc.set_ylim(min(0,current[0]),max(.001,current[1])*1.32)
    for ax in (axb,axc):
        ax.set_xlim(0,data['duration_s']);ax.grid(axis='y',color='#E0E0E0',lw=.4);ax.set_axisbelow(True)
    if data['missing_endpoint_ns'] or data['missing_interval_start_ns']:
        fig.text(.5,.043,'源时隙缺失：显示不完整，未补点或替换区间',ha='center',fontsize=7.3,color='#A03520')
    fig.text(.5,.016,'仅限 Gazebo 正常 Euler 扩展及受限 Begin 更新；不构成形式证明或真实连续安全证据。',ha='center',fontsize=7.3,color='#555555')
    fig.canvas.draw();renderer=fig.canvas.get_renderer();clipped=[];font_sizes=[]
    for text in fig.findobj(matplotlib.text.Text):
        if not text.get_visible() or not text.get_text():continue
        font_sizes.append(text.get_fontsize());box=text.get_window_extent(renderer)
        if box.x0<fig.bbox.x0-1 or box.x1>fig.bbox.x1+1 or box.y0<fig.bbox.y0-1 or box.y1>fig.bbox.y1+1:
            clipped.append(dict(text=text.get_text(),bbox=list(box.bounds)))
    if min(font_sizes)<7:raise ValueError('paper font size below 7pt')
    for ext in ('pdf','svg','png'):fig.savefig(output/('F04.'+ext),dpi=600)
    plt.close(fig)
    pdf_info=subprocess.check_output(['pdfinfo',str(output/'F04.pdf')],text=True)
    pdf_fonts=subprocess.check_output(['pdffonts',str(output/'F04.pdf')],text=True)
    embedded=[re.search(r'\s+(yes|no)\s+(yes|no)\s+(yes|no)\s+\d+\s+\d+\s*$',line) for line in pdf_fonts.splitlines()[2:] if line.strip()]
    if not embedded or any(match is None or match.group(1)!='yes' for match in embedded):raise ValueError('PDF has unembedded/unknown font')
    if '<text' in (output/'F04.svg').read_text():raise ValueError('SVG still requires external fonts')
    prefix=panels/('F04_'+output.name+'_print_150dpi')
    subprocess.run(['pdftoppm','-r','150','-png','-singlefile',str(output/'F04.pdf'),str(prefix)],check=True,stdout=subprocess.DEVNULL)
    return dict(width_mm=170,height_mm=155,png_pixels=list(Image.open(output/'F04.png').size),png_dpi=600,
        minimum_visible_font_pt=min(font_sizes),text_outside_canvas=clipped,pdf_info=pdf_info,pdf_fonts=pdf_fonts,
        pdf_all_fonts_embedded=True,svg_font_storage='embedded glyph paths, no external font dependency',font=binding(FONT),
        buffer_visual_approximation=dict(type='outward round-join polygon approximation',quarter_circle_segments=resolution,
            requested_margin_m=MARGIN_M,construction_radius_m=outer_radius,maximum_radial_excess_m=outer_radius-MARGIN_M),
        print_render=binding(prefix.with_suffix('.png')),visual_review='pending_actual_render_inspection')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,default=STAGE/'runs'/SELECTED_RUN)
    parser.add_argument('--revision',required=True)
    parser.add_argument('--independent-review',type=Path)
    args=parser.parse_args()
    if not re.fullmatch(r'r[0-9]{2,}',args.revision):raise ValueError('immutable revision such as r01 required')
    prereg_path=HERE/'F04_selection_preregistration.json';prereg=load_json(prereg_path)
    run=args.run.resolve();receipt=load_json(run/'receipt.json');check_selection(prereg,run,receipt)
    report_path=run/'sweep_evaluation/report.json';report=load_json(report_path);assessment_path=run/'assessment.json';assessment=load_json(assessment_path)
    if (assessment.get('run_id'),assessment.get('seed'))!=(SELECTED_RUN,SELECTED_SEED) or type(assessment.get('pass')) is not bool:
        raise ValueError('assessment identity/outcome absent')
    if receipt.get('report',{}).get('sha256')!=sha(assessment_path):raise ValueError('assessment receipt hash mismatch')
    endpoint_path=run/'sweep_evaluation/endpoints.csv';interval_path=run/'sweep_evaluation/intervals.jsonl'
    with endpoint_path.open() as stream:endpoints=list(csv.DictReader(stream))
    intervals=[json.loads(line) for line in interval_path.read_text().splitlines() if line.strip()]
    for path in (endpoint_path,interval_path):
        if not any(row.get('path')==str(path) and row.get('sha256')==sha(path) for row in report['outputs']):raise ValueError('saved evaluator output hash mismatch')
    source_paths=[prereg_path,Path(__file__),run/'receipt.json',assessment_path,report_path,endpoint_path,interval_path,
        run/'fixture.bag',run/'robot_description_runtime.xml',run/'converted_robot.sdf',run/'conversion_manifest.json',run/'world.sdf']
    sys.dont_write_bytecode=True;sys.path.insert(0,str(WORKSPACE/'scripts/astra_gazebo'))
    module=importlib.import_module('dense_sweep_p04')
    for filename in ('dense_sweep_p04.py','dense_sweep.py','geometry.py','evaluate_episode.py'):
        path=WORKSPACE/'scripts/astra_gazebo'/filename
        if not any(row.get('path')==str(path) and row.get('sha256')==sha(path) for row in report['source_bindings']):
            raise ValueError('plot helper differs from recorded evaluator source: '+filename)
        source_paths.append(path)
    for path in (run/'fixture.bag',run/'robot_description_runtime.xml',run/'converted_robot.sdf',run/'world.sdf'):
        if not any(row.get('path')==str(path) and row.get('sha256')==sha(path) for row in report['source_bindings']):raise ValueError('geometry source not bound by original evaluator')
    before=[binding(p) for p in source_paths]
    data=scalar_data(report,endpoints,intervals)
    report['_endpoint_rows']=endpoints
    geometry=read_first_geometry(run,report,module)
    review_label,review_binding=safe_review_status(args.independent_review,binding(assessment_path))
    output=HERE/args.revision;output.mkdir(exist_ok=False)
    panels=STAGE/'visuals/panels/geometry_contract';panels.mkdir(parents=True,exist_ok=True)
    qc=render(data,geometry,receipt,assessment,report,review_label,output,panels)
    save_csv(output/'F04_endpoints.csv',data['endpoints']);save_csv(output/'F04_2ms_budgets.csv',data['intervals']);save_csv(output/'F04_interval_bounds.csv',data['bounds'])
    (output/'F04_first_geometry.json').write_text(json.dumps(clean_json(geometry),indent=2,allow_nan=False)+'\n')
    caption=('图 F04  实际碰撞几何、裕量与步进净空界。固定选择 Q05 静态资格首个正式源时刻及整个正式源时间窗口，'
        'run_id=passive_sweep_q05_static，seed260926031；不按结果挑选帧或区间。'
        '（a）实际运行时九个碰撞组件的 XY 投影及欧氏 0.33 m 外扩，重表达在该时刻 base_link 平面坐标中；'
        '投影保留实际 roll/pitch，箱体为八顶点投影，圆柱采用外接48边形。蓝线为图示用微小外偏近似，不替代数值评估，非 Gazebo 截图。'
        '（b）每20 ms端点的带符号凸组件最小 XY 间距，以及减去0.33 m后的保守间距；重叠为负，投影间距不是三维接触测量。'
        '虚线为原评估器按全部2 ms步进预算推得的区间工程下界；完整性失败时只作失败诊断。'
        '（c）完整2 ms源时隙的机器人点位、其他相位/数值项、参与者Begin更新及相对净空总预算；静态场景的参与者项为零。'
        '端点连接线仅辅助阅读，不代表测得的连续轨迹。适用范围仅为已声明的Gazebo正常Euler扩展及受限actor Begin更新，'
        '不是形式证明、真实连续物理安全或导航性能证据。原Q05最终回执：'+('通过' if receipt['pass'] else '未通过')+
        '，原单回合评估：'+('通过' if assessment['pass'] else '未通过')+'。')
    (output/'F04_caption.md').write_text(caption+'\n')
    (output/'F04_insert.tex').write_text('\\begin{figure}[tb]\n\\centering\n\\includegraphics[width=170mm]{F04.pdf}\n\\caption{实际碰撞几何、裕量与步进净空界。完整图注见绑定的 F04\\_caption.md；仅为声明的 Gazebo 工程模型证据。}\n\\label{fig:geometry-contract}\n\\end{figure}\n')
    after=[binding(p) for p in source_paths]
    if before!=after:raise ValueError('source changed while plotting; output is not reviewable')
    outputs=[binding(p) for p in sorted(output.iterdir()) if p.is_file()]
    manifest=dict(schema='astra_F04_geometry_figure_manifest_v1',figure_id='F04',status='generated_pending_independent_visual_review',
        selected_run=SELECTED_RUN,seed=SELECTED_SEED,preregistered_rule=prereg['selection_rule'],source_bindings=before,
        source_window_ns=[data['start_ns'],data['end_ns']],geometry_source_ns=geometry['source_ns'],endpoint_count=len(data['endpoints']),interval_count=len(data['intervals']),
        missing_endpoint_count=len(data['missing_endpoint_ns']),missing_interval_count=len(data['missing_interval_start_ns']),
        original_receipt_pass=receipt['pass'],original_assessment_pass=assessment['pass'],original_sweep_integrity_pass=report['integrity_pass'],original_sweep_qualification=report['qualification'],
        independent_review=review_binding,independent_review_display=review_label,display_issues=data['display_issues'],quality_control=qc,
        outputs=outputs,formal_safety=False,real_continuous_physical_safety=False,new_episode_created=False,native_screenshot_count=0)
    (output/'F04_manifest.json').write_text(json.dumps(clean_json(manifest),indent=2,allow_nan=False)+'\n')
    (output/'atlas_index_suggestion.json').write_text(json.dumps(dict(figure_id='F04',title_zh=prereg['title_zh'],
        path=str(output/'F04.pdf'),manifest=binding(output/'F04_manifest.json'),caption_path=str(output/'F04_caption.md'),
        chapter_suggestion='方法与测量链：物理几何及工程适用范围',selection='preregistered_first_formal_source_and_full_window',
        status='pending_independent_visual_review',global_atlas_modified=False),indent=2)+'\n')
    print(json.dumps(dict(manifest=binding(output/'F04_manifest.json'),assessment_pass=assessment['pass'],qc=qc['text_outside_canvas'])))


if __name__=='__main__':main()
