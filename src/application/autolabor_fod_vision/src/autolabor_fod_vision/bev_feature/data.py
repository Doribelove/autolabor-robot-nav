import json
from pathlib import Path
import cv2
import numpy as np
from .geometry import ORDER,letterbox_k

def validate(sample, root, require_labels=True):
    if tuple(sample['camera_order'])!=ORDER:raise ValueError('camera order must be front,left,right,rear')
    if sample.get('clock_domain')!='ros_epoch_ns' or not sample.get('clock_verified'):raise ValueError('capture clocks not verified in ROS epoch')
    stamps=[sample['cameras'][n]['capture_ns'] for n in ORDER]
    if any(type(t) is not int for t in stamps):raise ValueError('capture timestamps must be integer nanoseconds')
    if min(stamps)<=0 or max(stamps)-min(stamps)>sample.get('max_skew_ns',20000000):raise ValueError('unsynchronized capture timestamps')
    if sample['stamp_ns']!=max(stamps):raise ValueError('reference capture must be latest camera capture')
    if sample.get('frame_id')!='base_link' or sample.get('rig_to_base_xyyaw') is None:raise ValueError('measured rig/base alignment required')
    transform=sample['rig_to_base_xyyaw']
    if len(transform)!=3 or not np.isfinite(transform).all():raise ValueError('rig/base requires finite x,y,yaw(rad)')
    for n in ORDER:
        c=sample['cameras'][n]
        if not (Path(root)/c['image']).is_file():raise ValueError('missing image '+n)
        if not c.get('upright'):raise ValueError('dataset images must be upright rectified images')
    if require_labels and not sample.get('labels_verified'):raise ValueError('metric centers need human verification')
    ids=[o['instance_id'] for o in sample['objects']]
    if len(ids)!=len(set(ids)):raise ValueError('one metric center per instance across views')
    if not all(np.isfinite(o['center_base_m']).all() and len(o['center_base_m'])==2 for o in sample['objects']):raise ValueError('invalid metric centers')

def images(sample, root, rig, spec):
    result=[]
    for name in ORDER:
        img=cv2.imread(str(Path(root)/sample['cameras'][name]['image']))
        if img is None or list(img.shape[1::-1])!=rig['cameras'][name]['perspective_size']:raise ValueError('image resolution disagrees with calibration')
        _,(nw,nh,l,t)=letterbox_k(rig['cameras'][name]['K_perspective'],img.shape[1::-1],spec['input_hw'])
        resized=cv2.resize(img,(nw,nh));out=np.zeros((*spec['input_hw'],3),np.uint8);out[t:t+nh,l:l+nw]=resized
        rgb=out[:,:,::-1].copy().astype(np.float32)/255.
        result.append(((rgb-[.485,.456,.406])/[.229,.224,.225]).transpose(2,0,1))
    return np.asarray(result,np.float32)

def targets(objects,spec):
    h=round((spec['x_range'][1]-spec['x_range'][0])/spec['resolution']);w=round((spec['y_range'][1]-spec['y_range'][0])/spec['resolution'])
    heat=np.zeros((1,h,w),np.float32);offset=np.zeros((2,h,w),np.float32);mask=np.zeros((1,h,w),np.float32)
    rr,cc=np.indices((h,w));used=set()
    for obj in objects:
        x,y=obj['center_base_m'];r=(x-spec['x_range'][0])/spec['resolution'];c=(y-spec['y_range'][0])/spec['resolution'];ri,ci=int(np.floor(r)),int(np.floor(c))
        if not (0<=ri<h and 0<=ci<w):raise ValueError('label outside BEV extent')
        if (ri,ci) in used:raise ValueError('two objects in same cell: use finer grid')
        used.add((ri,ci));heat[0]=np.maximum(heat[0],np.exp(-((rr-ri)**2+(cc-ci)**2)/2))
        offset[:,ri,ci]=[r-ri,c-ci];mask[0,ri,ci]=1
    return heat,offset,mask
