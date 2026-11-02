#!/usr/bin/env python3
"""Export a connected verified route for the existing coverage controller."""
import csv,json,math
from pathlib import Path
import numpy as np
from garage_route_validation import GarageMap
from garage_drivable_route import ROOT,PREVIOUS
from autolabor_coverage.coverage_geometry import GridMap
from autolabor_coverage.validated_path import check_path,same_pose


def dense(path):
    p=np.asarray(path);result=[p[0].tolist()]
    for a,b in zip(p,p[1:]):
        delta=b[:2]-a[:2];d=np.linalg.norm(delta);angle=math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]))
        if d<1e-8:continue
        mid=a[2]+angle/2;gear=1 if delta@[math.cos(mid),math.sin(mid)]>=0 else -1
        arc=d if abs(angle)<1e-8 else d*abs(angle)/(2*math.sin(abs(angle)/2))
        n=max(1,int(math.ceil(arc/.05)))
        for j in range(1,n+1):
            f=j/n;yaw=a[2]+f*angle
            xy=a[:2]+delta*f if abs(angle)<1e-8 else a[:2]+gear*arc/angle*np.array([math.sin(yaw)-math.sin(a[2]),-math.cos(yaw)+math.cos(a[2])])
            result.append([*xy,yaw])
    return result


def main():
    source=json.loads((ROOT/'route.json').read_text());old=json.loads((PREVIOUS/'contour_plan.json').read_text());g=GarageMap()
    grid=GridMap(g.width,g.height,g.resolution,*g.origin,g.cells.ravel())
    swaths=[];transitions=[];last=source['start'];total=0.;reverse=0.;max_k=0.;changes=0;checked=0
    with (ROOT/'route_xyyaw.csv').open('w') as stream:
        writer=csv.writer(stream);writer.writerow(['step','kind','x_m','y_m','yaw_rad','gear'])
        for i,step in enumerate(source['steps']):
            p=dense(step['path'])
            if not same_pose(last,p[0]):raise ValueError('Disconnected trajectory at step '+str(i))
            try:check_path(p,grid,1.32,.62,.5,1.35)
            except ValueError as error:raise ValueError('Step %d (%s): %s' % (i,step['kind'],error))
            last=p[-1];checked+=len(p)
            check=step['validation'];total+=check['length_m'];reverse+=check['reverse_m'];max_k=max(max_k,check['max_curvature']);changes+=check['direction_changes']
            if step['kind']=='sweep':swaths.append([p[0][:2],p[-1][:2]])
            else:transitions.append(p)
            for j,pose in enumerate(p):
                if j+1<len(p):
                    delta=np.subtract(p[j+1][:2],pose[:2]);gear=1 if delta@[math.cos(pose[2]),math.sin(pose[2])]>=0 else -1
                writer.writerow([i,step['kind'],*pose,gear])
    result=dict(format_version=2,pattern='connected_inward_contours_with_residual_fill',grid_digest=g.digest,
        region=old['region'],operation_width_m=1.2,overlap_ratio=.15,lane_spacing_m=1.02,
        minimum_turning_radius_m=1.35,footprint=dict(front_m=1.32,rear_m=.62,half_width_m=.5),
        swaths=swaths,transitions=transitions,preview_swept_known_area_m2=source['covered_area_m2'],
        known_free_area_m2=source['known_free_area_m2'],coverage_ratio=source['coverage_ratio'],
        target_coverage_ratio=.8,coverage_assumption=source['coverage_assumption'])
    (ROOT/'verified_plan.json').write_text(json.dumps(result,ensure_ascii=False))
    report=dict(known_free_area_m2=source['known_free_area_m2'],covered_area_m2=source['covered_area_m2'],
        geometric_coverage_ratio=source['coverage_ratio'],target=.8,swaths=len(swaths),verified_connectors=len(transitions),
        length_m=total,reverse_m=reverse,direction_changes_within_connectors=changes,checked_poses=checked,
        maximum_curvature_per_m=max_k,minimum_radius_m=1/max_k if max_k else None,
        invalid_footprints=0,disconnected_edges=0,source='saved static map; offline trajectory check only',
        tracking_tested=False,cleaning_actuators_tested=False)
    (ROOT/'final_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from matplotlib.patches import Patch
    fig,axes=plt.subplots(1,2,figsize=(16,11));background=np.where(g.cells<0,165,np.where(g.cells>0,25,247)).astype(np.uint8)
    extent=[g.origin[0],g.origin[0]+g.width*g.resolution,g.origin[1],g.origin[1]+g.height*g.resolution]
    covered=np.load(ROOT/'covered.npy');rgb=np.repeat(background[:,:,None],3,axis=2)
    rgb[(g.cells==0)&(covered!=0)]=[126,198,146]
    for ax in axes:
        ax.imshow(rgb,origin='lower',extent=extent);ax.set_aspect('equal');ax.set_xlabel('map X (m)');ax.set_ylabel('map Y (m)')
        ax.plot(*source['start'][:2],'r*',ms=11)
    for kind,color,lw in [('transit','#e09c22',.4),('sweep','#1768ad',.65)]:
        paths=[np.asarray(s['path'])[:,:2] for s in source['steps'] if s['kind']==kind]
        axes[0].add_collection(LineCollection(paths,colors=color,linewidths=lw,alpha=.8))
    axes[0].set_title('Verified continuous route (including turns and reverse)\nBlue: sweeps / orange: connectors / star: recorded start')
    axes[1].set_title('1.2 m front brush geometric union: %.1f%% of known free area\n%.0f / %.0f m2'%(100*source['coverage_ratio'],source['covered_area_m2'],source['known_free_area_m2']))
    axes[1].legend(handles=[Patch(color='#7ec692',label='Geometrically covered'),Patch(color='#f7f7f7',label='Uncovered known floor'),Patch(color='#a5a5a5',label='Unknown'),Patch(color='#191919',label='Occupied')],loc='lower left')
    fig.suptitle('Garage map 2 | padded footprint 1.94 x 1.00 m | minimum turning radius 1.35 m\nOffline feasibility; physical driving and pickup not tested',fontsize=13)
    fig.tight_layout();fig.savefig(ROOT/'verified_route.png',dpi=150);plt.close(fig)
    print(json.dumps(report,ensure_ascii=False),flush=True)

if __name__=='__main__':main()
