#!/usr/bin/env python3
"""Offline F04 final-artifact numeric audit; never generates/replaces plot evidence."""
import argparse,csv,hashlib,json,math,re,subprocess,sys
from dataclasses import asdict
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace';F=S/'visuals/plots/geometry_contract'
sys.path[:0]=[str(W/'scripts/astra_gazebo'),str(W/'devel/lib/python3/dist-packages'),'/opt/ros/noetic/lib/python3/dist-packages']
import rosbag
import dense_sweep_p04 as dense
from PIL import Image
parser=argparse.ArgumentParser();parser.add_argument('--revision',required=True);args=parser.parse_args();O=F/args.revision;R=S/'runs/passive_sweep_q05_static'
def read(p):return json.loads(p.read_bytes())
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb')as f:
  for chunk in iter(lambda:f.read(4194304),b''):h.update(chunk)
 return h.hexdigest()
def bind(p):return {'path':str(Path(p).resolve()),'sha256':sha(p)}
def csvrows(p):return list(csv.DictReader(p.open()))
manifest=read(O/'F04_manifest.json');report=read(R/'sweep_evaluation/report.json');start,end=report['source_window_ns'];prereg=read(F/'F04_selection_preregistration.json')
exports={'endpoint':csvrows(O/'F04_endpoints.csv'),'budget':csvrows(O/'F04_2ms_budgets.csv'),'bound':csvrows(O/'F04_interval_bounds.csv')}
original_endpoints=csvrows(R/'sweep_evaluation/endpoints.csv');original_intervals=[json.loads(x)for x in (R/'sweep_evaluation/intervals.jsonl').read_text().splitlines()]
checks={}
checks['all_manifest_source_and_output_SHA_bindings']=all(sha(x['path'])==x['sha256']for x in manifest['source_bindings']+manifest['outputs'])
checks['first_static_first_formal_source_full_window_selection']=manifest['selected_run']==prereg['selected_future_run']=='passive_sweep_q05_static'and manifest['seed']==prereg['seed']==260926031 and manifest['geometry_source_ns']==start and manifest['source_window_ns']==[start,end]
checks['complete_501_endpoint_source_grid']=len(exports['endpoint'])==501 and [int(x['stamp_ns'])for x in exports['endpoint']]==list(range(start,end+1,20000000))
checks['all_endpoint_values_exact_original_and_margin']=len(exports['endpoint'])==len(original_endpoints)and all(int(a['stamp_ns'])==int(b['stamp_ns'])and float(a['time_from_formal_start_s'])==(int(a['stamp_ns'])-start)/1e9 and all(float(a[k])==float(b[k])for k in ('physical_xy_m','conservative_xy_m'))and abs(float(a['physical_xy_m'])-float(a['conservative_xy_m'])-.33)<1e-14 for a,b in zip(exports['endpoint'],original_endpoints))
fields=['robot_point_budget_m','actor_Begin_variation_budget_m','other_phase_variation_budget_m','relative_clearance_variation_budget_m']
checks['complete_5000_budget_source_grid_and_exact_values']=len(exports['budget'])==len(original_intervals)==5000 and [int(x['start_ns'])for x in exports['budget']]==list(range(start,end,2000000))and all(int(a['start_ns'])==b['start_ns']and int(a['end_ns'])==b['end_ns']and float(a['time_from_formal_start_s'])==(b['start_ns']-start)/1e9 and all(float(a[k])==b[k]for k in fields)for a,b in zip(exports['budget'],original_intervals))
checks['budget_sum_and_zero_static_actor']=all(abs(sum(float(x[k])for k in fields[:3])-float(x[fields[3]]))<1e-14 and float(x['actor_Begin_variation_budget_m'])==0 for x in exports['budget'])
checks['all_500_piecewise_lower_bounds_exact']=len(exports['bound'])==len(report['coarse_intervals'])==500 and all(int(a['start_ns'])==b['start_ns']and int(a['end_ns'])==b['end_ns']and float(a['lower_bound_m'])==b['conditional_clearance_lower_bound_m']and a['covered_exactly']=='True'and b['covered_exactly']is True and float(a['start_s'])==(b['start_ns']-start)/1e9 and float(a['end_s'])==(b['end_ns']-start)/1e9 for a,b in zip(exports['bound'],report['coarse_intervals']))
checks['minimum_bound_matches_original_replayed_Q05']=min(float(x['lower_bound_m'])for x in exports['bound'])==report['minimum_engineering_conservative_swept_xy_m']==1.4422737777090144
metadata={};native=None;step=None
with rosbag.Bag(str(R/'fixture.bag'))as bag:
 for topic,m,_ in bag.read_messages(topics=['/astra/p03/runtime_metadata','/astra/p03/end_links','/astra/p03/step_status']):
  if topic.endswith('runtime_metadata'):
   v=json.loads(m.data);metadata[v['revision']]=v
  elif topic.endswith('step_status'):
   v=json.loads(m.data)
   if v['stamp_ns']==start:assert step is None;step=v
  elif m.header.stamp.to_nsec()==start:assert native is None;native=m
model=dense.parse_converted_model((R/'robot_description_runtime.xml').read_text(),(R/'converted_robot.sdf').read_text());geometry=dense.runtime_geometry(metadata[step['end_revision']],model)
states={name:{'pose':dense.engineering_pose((p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w))}for name,p in zip(native.name,native.pose)}
world=dense.WorldCollisions((R/'world.sdf').read_text());ground={x['name']for x in world.grounds}
robot=dense.projections(geometry,states,True,ground);obstacles=dense.projections(geometry,states,False,ground)
normal=lambda x:json.loads(json.dumps(x))
exportgeo=read(O/'F04_first_geometry.json');clearance=dense.compound_clearance(robot,obstacles,.33)
checks['nine_component_native_first_source_reprojection_exact']=len(robot)==9 and exportgeo['source_ns']==start and exportgeo['metadata_revision']==step['end_revision']and exportgeo['base_pose_world']==normal(asdict(states['autolabor_m2::base_link']['pose']))and exportgeo['robot_components']==normal([asdict(x)for x in robot])and exportgeo['obstacle_components']==normal([asdict(x)for x in obstacles])and exportgeo['first_clearance_audit']==normal(clearance)
checks['native_first_geometry_clearance_equals_endpoint']=clearance['minimum_physical_projected_clearance_m']==float(original_endpoints[0]['physical_xy_m'])
pdfinfo=subprocess.check_output(['pdfinfo',str(O/'F04.pdf')],text=True);fonts=subprocess.check_output(['pdffonts',str(O/'F04.pdf')],text=True)
size=re.search(r'Page size:\s+([\d.]+) x ([\d.]+) pts',pdfinfo);width,height=[float(x)*25.4/72 for x in size.groups()]
checks['actual_PDF_170_by_155_mm']=abs(width-170)<.002 and abs(height-155)<.002
fontrows=[re.search(r'\s+(yes|no)\s+(yes|no)\s+(yes|no)\s+\d+\s+\d+\s*$',line)for line in fonts.splitlines()[2:]if line.strip()]
checks['actual_PDF_fonts_embedded_and_SVG_glyph_paths']=bool(fontrows)and all(x and x.group(1)=='yes'for x in fontrows)and '<text'not in(O/'F04.svg').read_text()
with Image.open(O/'F04.png')as im:png={'size':list(im.size),'dpi':list(im.info.get('dpi',[]))}
checks['native_600dpi_png_full_canvas']=png['size']==[int(170/25.4*600),int(155/25.4*600)]and len(png['dpi'])==2 and all(abs(x-600)<.01 for x in png['dpi'])
caption=(O/'F04_caption.md').read_text();tex=(O/'F04_insert.tex').read_text()
checks['caption_scope_source_phase_and_units_explicit']=all(t in caption for t in ('seed260926031','0.33 m','20 ms','2 ms','不是三维接触','非 Gazebo 截图','不是形式证明','真实连续物理安全','导航性能'))and 'width=170mm'in tex
checks['no_stale_workflow_label_in_caption']=not any(x in caption for x in ('待完成','状态未确认'))
checks['no_data_changes_from_r05']=all(sha(O/n)==sha(F/'r05'/n)for n in ('F04_endpoints.csv','F04_2ms_budgets.csv','F04_interval_bounds.csv','F04_first_geometry.json'))
checks['author_declared_font_and_clipping_bound']=manifest['quality_control']['minimum_visible_font_pt']>=7 and manifest['quality_control']['text_outside_canvas']==[]
out={'status':'independent_F04_numeric_and_artifact_checks_pass'if all(checks.values())else'independent_F04_checks_failed','pass':all(checks.values()),'checks':checks,'revision':args.revision,'manifest':bind(O/'F04_manifest.json'),'native_source_pose_reprojected':True,'pdf_page_mm':[width,height],'pdf_info':pdfinfo,'pdf_fonts':fonts,'png':png,
 'numeric_summary':{'endpoint_count':len(exports['endpoint']),'budget_count':len(exports['budget']),'bound_count':len(exports['bound']),'physical_clearance_range_m':[min(float(x['physical_xy_m'])for x in exports['endpoint']),max(float(x['physical_xy_m'])for x in exports['endpoint'])],'minimum_engineering_bound_m':report['minimum_engineering_conservative_swept_xy_m'],'maximum_relative_budget_mm':1000*max(float(x['relative_clearance_variation_budget_m'])for x in exports['budget'])},
 'inputs':[bind(p)for p in (Path(__file__),R/'fixture.bag',R/'sweep_evaluation/report.json',O/'F04_first_geometry.json',F/'F04_selection_preregistration.json')],
 'visual_inspection_separate':True,'no_ROS_no_new_episode':True}
with (D/('numeric_'+args.revision+'.json')).open('x')as f:json.dump(out,f,indent=2,ensure_ascii=False);f.write('\n')
print(json.dumps({'pass':out['pass'],'checks':len(checks),'failed':[k for k,v in checks.items()if not v]},indent=2));assert out['pass']
