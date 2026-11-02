#!/usr/bin/env python3
"""Offline real-image comparison; no camera activation, navigation, or ROS publishing."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'distribution/j6m_bpu_navigation'))
from surround.image_detector import BpuViewsDetector,CudaViewsDetector
from surround.ground_detections import localize_detections,render_ground_detections


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend',choices=('bpu','cuda'),required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--label-samples',type=int,default=20)
    parser.add_argument('--image-size',type=int,default=640)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    cv2.setNumThreads(1)
    snapshot=ROOT/'runtime/surround_recalibrated_20261101/bev'
    meta=json.loads((snapshot/'bev.json').read_text())
    images={n:cv2.imread(str(snapshot/(n+'_upright.png'))) for n in meta['sources']}
    for n in images:
        file=n+'_upright.png'
        if hashlib.sha256((snapshot/file).read_bytes()).hexdigest()!=meta['snapshot_files_sha256'][file]:
            raise ValueError('Snapshot images/metadata do not match')
    meta['camera_image_sizes']={n:list(im.shape[1::-1]) for n,im in images.items()}
    detector=BpuViewsDetector(log_path=args.output/'worker.log') if args.backend=='bpu' else CudaViewsDetector(image_size=args.image_size)
    try:
        rounds=[]
        for i in range(5):
            begin=time.monotonic()
            detections,timing=detector.predict_views(images,dict(session=meta['fusion_session'],sequence=i+1))
            ground,rejected=localize_detections(meta,detections)
            rounds.append(dict(wall_ms=(time.monotonic()-begin)*1000,timing=timing,
                               detections=detections,ground_objects=ground,rejected=rejected))
        annotated,_=render_ground_detections(cv2.imread(str(snapshot/'bev.png')),meta,ground)
        cv2.imwrite(str(args.output/'bev_points.jpg'),annotated)
        for n,im in images.items():
            im=im.copy()
            for d in detections[n]:
                x0,y0,x1,y1=map(round,d['bbox'])
                cv2.rectangle(im,(x0,y0),(x1,y1),(0,245,255),2)
            cv2.imwrite(str(args.output/(n+'_detections.jpg')),im)
        (args.output/'camera_rounds.json').write_text(json.dumps(rounds,indent=2)+'\n')
        # Small existing image-box set: only a functional recognition check.
        # Its relationship to detector training is unknown; no held-out claim.
        data=Path('/home/slam/yolo11/image_label')
        positive=[p for p in sorted((data/'labels/test').glob('*.txt')) if p.read_text().strip()]
        chosen=positive[::max(1,len(positive)//max(1,args.label_samples))][:args.label_samples]
        labeled=[]
        for index,label in enumerate(chosen):
            candidates=list((data/'images/test').glob(label.stem+'.*'))
            if len(candidates)!=1:continue
            image=cv2.imread(str(candidates[0]))
            scale=min(1.,1280/max(image.shape[:2]))
            if scale<1:
                image=cv2.resize(image,(round(image.shape[1]*scale),round(image.shape[0]*scale)))
            height,width=image.shape[:2]
            truth=[]
            for line in label.read_text().splitlines():
                _,x,y,w,h=map(float,line.split())
                truth.append([(x-w/2)*width,(y-h/2)*height,(x+w/2)*width,(y+h/2)*height])
            found,timing=detector.predict_views({'sample':image},dict(session='offline_labeled_images',sequence=index+1))
            matches=set();tp=0
            for d in sorted(found['sample'],key=lambda d:-d['confidence']):
                a=d['bbox']; options=[]
                for j,b in enumerate(truth):
                    inter=max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
                    union=(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-inter
                    if j not in matches and union>0:options.append((inter/union,j))
                if options and max(options)[0]>=.5:tp+=1;matches.add(max(options)[1])
            labeled.append(dict(image=str(candidates[0]),truth_boxes=truth,predictions=found['sample'],
                                true_positives=tp,timing=timing))
        tp=sum(r['true_positives'] for r in labeled)
        predicted=sum(len(r['predictions']) for r in labeled);truth=sum(len(r['truth_boxes']) for r in labeled)
        report=dict(backend=detector.backend,model_sha256=detector.model_sha256,
            checkpoint_sha256=detector.checkpoint_sha256,
            mode='offline actual original camera images; no live exposure/TF',rounds=rounds,
            four_view_wall_ms_p50=float(np.median([r['wall_ms'] for r in rounds[1:]])),
            existing_image_box_check=dict(samples=len(labeled),true_positives=tp,predictions=predicted,labels=truth,
                precision=tp/predicted if predicted else None,recall=tp/truth if truth else None,
                iou=.5,held_out_status='unknown; not a deployment accuracy estimate',samples_detail=labeled),
            ground_metric_error_m=None,capture_to_map_publish_ms=None)
        (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ('rounds','existing_image_box_check')}))
        print(json.dumps({k:v for k,v in report['existing_image_box_check'].items() if k!='samples_detail'}))
    finally:detector.shutdown()


if __name__=='__main__':main()
