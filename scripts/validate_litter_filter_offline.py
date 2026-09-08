#!/usr/bin/env python3
"""Synthetic RGB-D fixture replay and local CPU benchmark. No live ROS/BPU."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
import argparse
import copy
import json
from pathlib import Path
import platform
import sys
import time

import cv2
import numpy as np
from autolabor_fod_vision.bpu_rgbd import estimate_boxes
from autolabor_fod_vision.litter_filter import LitterFilter,FilterConfig

WS=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(WS/'src/application/autolabor_fod_vision/test'))
from litter_scenes import scene,BOTTLE,PLANT


def summary(values):
    return dict(samples=len(values),median_ms=float(np.median(values)),p95_ms=float(np.percentile(values,95)),
                maximum_ms=float(np.max(values)))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True,help='new output directory')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    cv2.setNumThreads(1)
    depth,k,mount,boxes=scene([PLANT,BOTTLE])
    reports={};engine=LitterFilter();audit=None
    for count in (2,12,24,100):
        inputs=[dict(copy.deepcopy(boxes[i%2]),object_id=i+1) for i in range(count)]
        engine=LitterFilter();elapsed=[];legacy=[];budget_unknown=[]
        for i in range(45):
            result=engine.process(inputs,depth,k,mount,True,100+i/15)
            if i>=5:
                elapsed.append(result['elapsed_ms'])
                budget_unknown.append(sum(d['reason']=='geometry_budget' for d in result['decisions']))
            if i<10:
                start=time.monotonic();estimate_boxes(inputs,depth,k)
                legacy.append((time.monotonic()-start)*1000)
        if count==2:audit=result
        reports[str(count)]=dict(filter=summary(elapsed),existing_depth_fusion=summary(legacy),
                                mean_budget_unknown=float(np.mean(budget_unknown)))
    report=dict(kind='synthetic geometry replay; no detector inference or live sensor',
                platform=platform.platform(),python=sys.executable,opencv=cv2.__version__,
                numpy=np.__version__,config=vars(FilterConfig()),filter_sha256=FilterConfig().digest,
                benchmark=reports,example=audit,
                stress_note='12/24/100 object runs duplicate ROIs for bounded-work stress, not accuracy')
    (args.output/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    figure,axes=plt.subplots(1,2,figsize=(14,4.5),constrained_layout=True)
    for ax,filtered in zip(axes,(False,True)):
        ax.imshow(depth,cmap='gray',vmin=.3,vmax=5)
        for decision in audit['decisions']:
            if filtered and decision['state']=='environment':continue
            x1,y1,x2,y2=decision['bbox']
            color='#1aff66' if decision['state']=='candidate' else '#ffbe32'
            ax.add_patch(Rectangle((x1,y1),x2-x1,y2-y1,fill=False,color=color,linewidth=2))
            label=decision['class_name'] if not filtered else decision['state']+' / '+decision['class_name']
            ax.text(x1,y1-5,label,color=color,fontsize=9,backgroundcolor='#222222')
        ax.set_title('Filtered, same source frame' if filtered else 'Raw generic detections (injected)')
        ax.axis('off')
    figure.suptitle('SYNTHETIC depth scene: tall planter removed; bottle labeled suitcase retained\nNot real-camera detection accuracy',fontsize=12)
    figure.savefig(str(args.output/'synthetic_comparison.png'),dpi=150)
    plt.close(figure)
    print(json.dumps(dict(benchmark=reports,counts=audit['counts']),indent=2))


if __name__=='__main__':main()
