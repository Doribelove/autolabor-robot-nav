"""Create a paper-sized, explicitly diagnostic N04/N06 old-state figure."""
import csv
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import rosbag
import yaml

S=Path(__file__).resolve().parents[1]
OUT=S/'paper_update/figures/NAV02_lookahead_fixed_state_diagnostic_r03'
RUN=S/'runs/astra_navigation_n04_b00_astra_analytic_route_v1'
PROBE=S/'audits/navigation_n06_lookahead_hypothesis/old_trace_steering_demand.csv'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    if OUT.exists():raise FileExistsError('NAV02 figure identity exists')
    contract=json.loads((RUN/'contract.json').read_bytes())
    bundle_path=Path(contract['shared_bundle']['path'])
    if sha(bundle_path)!=contract['shared_bundle']['sha256']:
        raise ValueError('bound route changed')
    bundle=json.loads(bundle_path.read_bytes())
    scene=yaml.safe_load((RUN/'scene_manifest.yaml').read_bytes())
    with PROBE.open(newline='') as stream:probe=list(csv.DictReader(stream))
    if len(probe)!=7 or bundle['map_from_odom_xy_yaw']!=[-5.4,0.,0.]:
        raise ValueError('old-state probe/transform differs')
    odom=[]
    with rosbag.Bag(str(RUN/'fixture.bag')) as bag:
        for _,msg,_ in bag.read_messages(topics=['/odom']):
            stamp=msg.header.stamp.to_nsec()
            if 7_460_000_000<=stamp<=29_460_000_000:
                odom.append((msg.pose.pose.position.x-5.4,msg.pose.pose.position.y))
    if not odom:raise ValueError('old real odom absent')
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':8,'axes.linewidth':.7,
                         'svg.fonttype':'none','pdf.fonttype':42})
    fig,axes=plt.subplots(1,2,figsize=(6.7,3.15),gridspec_kw={'width_ratios':[1.13,1]})
    fig.subplots_adjust(left=.075,right=.99,bottom=.18,top=.86,wspace=.32)
    ax=axes[0]
    for box in scene['static_boxes']:
        if abs(box['x'])>9 or abs(box['y'])>4:continue
        ax.add_patch(Rectangle((box['x']-box['length']/2,box['y']-box['width']/2),
                               box['length'],box['width'],facecolor='#8B97A3',edgecolor='#45515D',lw=.6))
    guide=bundle['points_xy']
    ax.plot([p[0] for p in guide],[p[1] for p in guide],color='#146C94',lw=1.6,label='Fixed shared guide')
    ax.plot([p[0] for p in odom],[p[1] for p in odom],color='#E27D32',lw=1.5,label='Observed N04 base path')
    ax.scatter([-4],[0],s=22,color='#1B9E77',marker='o',zorder=5,label='Start')
    ax.scatter([5],[0],s=28,color='#B2182B',marker='*',zorder=5,label='Goal')
    ax.scatter([odom[-1][0]],[odom[-1][1]],s=23,color='#E27D32',marker='x',zorder=6)
    ax.set(xlim=(-4.5,5.5),ylim=(-3.2,3.2),xlabel='Map x (m)',ylabel='Map y (m)')
    ax.set_aspect('equal',adjustable='box');ax.grid(color='#E0E4E8',lw=.4)
    ax.set_title('(a) Valid old N04 timeout trace',loc='left',fontsize=8.5)
    ax.legend(loc='lower left',fontsize=6.7,frameon=True,ncol=2,handlelength=1.5)
    ax=axes[1]
    xs=[float(r['base_map_x_m']) for r in probe]
    original=[math.degrees(float(r['original_desired_steer_rad'])) for r in probe]
    proposed=[math.degrees(float(r['proposed_desired_steer_rad'])) for r in probe]
    ax.plot(xs,original,color='#146C94',marker='o',ms=3,lw=1.5,label='0.7 m control')
    ax.plot(xs,proposed,color='#B2182B',marker='s',ms=3,lw=1.5,label='1.2 m proposal')
    ax.axhline(math.degrees(.4964),color='#888888',ls=':',lw=.8)
    ax.axhline(-math.degrees(.4964),color='#888888',ls=':',lw=.8)
    ax.axvline(-.825,color='#899399',ls='--',lw=.8,label='First guide corner')
    ax.set(xlabel='Observed map x at fixed old states (m)',ylabel='Desired center steer (deg)',
           xlim=(-3.15,-.72),ylim=(-31,31))
    ax.grid(color='#E0E4E8',lw=.4)
    ax.set_title('(b) Fixed-state steering demand',loc='left',fontsize=8.5)
    ax.legend(loc='lower left',fontsize=6.7,frameon=True)
    fig.suptitle('Old trace and fixed-state lookahead hypothesis',fontsize=10,y=.96)
    OUT.mkdir(parents=True)
    stem=OUT/'NAV02_lookahead_fixed_state_diagnostic'
    for suffix in ('.svg','.pdf','.png'):
        fig.savefig(str(stem)+suffix,dpi=450,facecolor='white')
    plt.close(fig)
    report=dict(figure_id='NAV02',group='lookahead_fixed_state_diagnostic',
                status='generated_pending_machine_and_visual_qc',
                source_run=RUN.name,source_run_assessment_sha256=sha(RUN/'assessment.json'),
                source_bag_sha256=sha(RUN/'fixture.bag'),source_bundle_sha256=sha(bundle_path),
                source_probe_sha256=sha(PROBE),source_scene_manifest_sha256=sha(RUN/'scene_manifest.yaml'),
                source_script_sha256=sha(Path(__file__)),
                files={suffix:dict(path=str(stem)+suffix,sha256=sha(Path(str(stem)+suffix))) for suffix in ('.svg','.pdf','.png')},
                claim_scope='One old valid timeout trajectory plus seven fixed-state steering recomputations; not counterfactual execution, N06 improvement or general safety.')
    (OUT/'metadata.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(figure_id='NAV02',status=report['status'],files=list(report['files']))))


if __name__=='__main__':main()
