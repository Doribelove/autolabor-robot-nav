#!/usr/bin/env python3
"""Capture diagnostic views, project named ground references; no fake metric truth.
--anchors CSV: camera,u,v,instance_id,true_x,true_y. Coordinates in upright image.
Produces candidates for HUMAN review; projection is not a measured ground label.
"""
import argparse,csv,json,sys,struct,time,urllib.request,concurrent.futures,shutil
from pathlib import Path
import cv2,numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path[:0]=[str(ROOT/'src/application/autolabor_fod_vision/src'),str(ROOT/'distribution/j6m_bpu_navigation')]
from surround.ground_bev import GroundBev
from autolabor_fod_vision.bev_feature.geometry import ORDER

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--anchors');a=p.parse_args();out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
 rig=json.loads((ROOT/'config/four_camera_bev.json').read_text());bev=GroundBev(rig)
 if a.anchors:
  rows=[]
  for row in csv.DictReader(open(a.anchors)):
   n=row['camera'];r=bev.transforms[n][:3,:3];o=bev.transforms[n][:3,3];ray=r@np.linalg.solve(bev.intrinsics[n],[float(row['u']),float(row['v']),1])
   if ray[2]>=-1e-6:raise ValueError('anchor ray does not meet ground in front of camera')
   point=o-ray*o[2]/ray[2];row.update(candidate_x=float(point[0]),candidate_y=float(point[1]),human_verified=False)
   if row.get('true_x') and row.get('true_y'):row['error_m']=float(np.linalg.norm(point[:2]-[float(row['true_x']),float(row['true_y'])]))
   rows.append(row)
  (out/'anchor_candidates.json').write_text(json.dumps(rows,indent=2));return
 def fetch(name):
  cam=rig['cameras'][name]
  url='http://127.0.0.1:18885/frame' if name=='front' else 'http://127.0.0.1:18882/gdc/%d'%cam['rx1_port']
  data=urllib.request.urlopen(url,timeout=4).read()
  if name=='front':
   magic,n,size=struct.unpack('!4sII',data[:12]);assert magic==b'ZP01' and len(data)==12+n+size
   meta=json.loads(data[12:12+n])['image'];img=cv2.imdecode(np.frombuffer(data[12+n:],np.uint8),cv2.IMREAD_COLOR)
   source=dict(capture_ns=meta['source_stamp_ns'],clock_domain='zed_sdk_unix',sequence=meta['sequence'])
  else:
   n=struct.unpack('!I',data[:4])[0];meta=json.loads(data[4:4+n]);g=meta['gdc'];assert meta['age_s']<1.5
   assert (g['sequence'],g['raw_stamp'])==(meta['raw']['sequence'],meta['raw']['raw_stamp'])
   img=cv2.cvtColor(np.frombuffer(data[4+n:],np.uint8).reshape(810,960),cv2.COLOR_YUV2BGR_NV12)
   source=dict(capture_ns=g['raw_stamp'],clock_domain='j6m_vendor_counter_epoch_unverified',sequence=g['sequence'])
  return name,bev.upright(name,img),source
 with concurrent.futures.ThreadPoolExecutor(4) as ex:frames={n:(im,s) for n,im,s in ex.map(fetch,ORDER)}
 sample=dict(camera_order=list(ORDER),frame_id=rig['frame_id'],rig_to_base_xyyaw=None,clock_domain='mixed_unverified',clock_verified=False,stamp_ns=None,max_skew_ns=20000000,calibration='calibration.json',labels_verified=False,objects=[],cameras={})
 for name,(img,source) in frames.items():
  cv2.imwrite(str(out/(name+'.png')),img);sample['cameras'][name]=dict(source,image=name+'.png',upright=True)
 (out/'sample.jsonl').write_text(json.dumps(sample)+'\n');(out/'calibration.json').write_text(json.dumps(rig,indent=2))
 import matplotlib;matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 fig,axes=plt.subplots(2,3,figsize=(18,10));points=np.array([[2,0,0],[0,2,0],[0,-2,0],[-1.2,0,0],[1.5,1.5,0],[1.5,-1.5,0]])
 records=[]
 for ax,name in zip(axes.ravel()[:4],ORDER):
  img=frames[name][0];ax.imshow(cv2.cvtColor(img,cv2.COLOR_BGR2RGB));uv,depth=bev.project(name,points)
  for i,(pixel,d) in enumerate(zip(uv,depth)):
   valid=d>0 and 0<=pixel[0]<img.shape[1] and 0<=pixel[1]<img.shape[0]
   records.append(dict(camera=name,id=i,x=points[i,0],y=points[i,1],u=float(pixel[0]),v=float(pixel[1]),visible=bool(valid)))
   if valid:ax.plot(*pixel,'ro');ax.text(*pixel,str(i),color='yellow')
  ax.set_title(name+' | upright rectified; analytic reference points');ax.set_xlim(0,img.shape[1]);ax.set_ylim(img.shape[0],0)
 image,counts,_=bev.compose({n:f[0] for n,f in frames.items()});ax=axes.ravel()[4];ax.imshow(cv2.cvtColor(image,cv2.COLOR_BGR2RGB),extent=[3,-3,-2,4]);ax.set_xlabel('rig Y (m), left positive');ax.set_ylabel('rig X (m), forward');ax.grid()
 for i,pt in enumerate(points):ax.plot(pt[1],pt[0],'ro');ax.text(pt[1],pt[0],str(i),color='yellow')
 axes.ravel()[5].imshow(counts,origin='upper');axes.ravel()[5].set_title('Projection support camera count; black=blind')
 fig.suptitle('GEOMETRY DIAGNOSTIC ONLY — reference points are analytic, not measured targets; clocks not synchronized')
 fig.tight_layout();fig.savefig(out/'geometry_comparison.png',dpi=130);plt.close(fig)
 (out/'reference_points.json').write_text(json.dumps(records,indent=2));print(out)
if __name__=='__main__':main()
