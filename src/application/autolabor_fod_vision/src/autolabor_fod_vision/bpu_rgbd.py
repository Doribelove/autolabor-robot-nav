"""Source-pixel FCOS preprocessing and robust synchronized RGB-D fusion."""
import math
import cv2
import numpy as np
from autolabor_fod_vision.two_stage import estimate_clustered_depth

FCOS_SHA='1daacc7dfde181b65872c47f4fe5db84fd31f19d3ad56aa1fa1d902bb5ffd7fa'


class Preprocessor:
    def __init__(self):
        self.canvas=np.zeros((896,896,3),np.uint8)
        self.i420=np.empty((1344,896),np.uint8)
        self.nv12=np.empty(896*896*3//2,np.uint8)
        self.shape=None

    def prepare(self,frame):
        if frame.ndim!=3 or frame.shape[2]!=3 or frame.dtype!=np.uint8:
            raise ValueError('Expected uint8 BGR image')
        h,w=frame.shape[:2]
        if not 64<=w<=1920 or not 64<=h<=1080:raise ValueError('Image dimensions outside model limits')
        scale=min(896/w,896/h);ow,oh=round(w*scale),round(h*scale)
        if self.shape!=(h,w):self.canvas.fill(0);self.shape=(h,w)
        cv2.resize(frame,(ow,oh),dst=self.canvas[:oh,:ow])
        cv2.cvtColor(self.canvas,cv2.COLOR_BGR2YUV_I420,dst=self.i420)
        flat=self.i420.reshape(-1);count=896*896
        self.nv12[:count]=flat[:count]
        self.nv12[count::2]=flat[count:count*5//4]
        self.nv12[count+1::2]=flat[count*5//4:]
        payload=self.nv12[:896*oh].tobytes()+self.nv12[count:count+896*((oh+1)//2)].tobytes()
        return payload,oh,(ow/w,oh/h)


def source_boxes(result,width,height,scales,classes):
    if result.get('model_sha256')!=FCOS_SHA or result.get('motion_eligible') is not False:
        raise ValueError('FCOS model/eligibility contract mismatch')
    detections=result.get('detections')
    if not isinstance(detections,list) or len(detections)>100:raise ValueError('Invalid detection count')
    output=[]
    for d in detections:
        box=np.asarray(d.get('bbox_model_px'),dtype=float)
        confidence,label=d.get('confidence'),d.get('class_id')
        if (box.shape!=(4,) or not np.isfinite(box).all() or type(label) is not int or
                label not in classes or not isinstance(confidence,(int,float)) or
                not math.isfinite(confidence) or not 0<=confidence<=1):raise ValueError('Malformed FCOS box')
        box[[0,2]]=np.clip(box[[0,2]]/scales[0],0,width)
        box[[1,3]]=np.clip(box[[1,3]]/scales[1],0,height)
        if box[2]-box[0]<1 or box[3]-box[1]<1:continue
        output.append(dict(bbox=box.tolist(),class_id=label,class_name=classes[label],confidence=confidence))
    return output


def estimate_boxes(detections,depth,intrinsics):
    estimates=[]
    for detection in detections:
        x1,y1,x2,y2=detection['bbox']
        left=max(0,int(math.floor(x1)));top=max(0,int(math.floor(y1)))
        right=min(depth.shape[1],int(math.ceil(x2)));bottom=min(depth.shape[0],int(math.ceil(y2)))
        # Bound clustering cost per object. Nearest-neighbour sampling preserves
        # invalid pixels and separate foreground/background depths; no averaging
        # across depth edges. Intrinsics remain tied to the sampled pixel grid.
        stride=max(1,int(math.ceil(max(right-left,bottom-top)/128)))
        region=depth[top:bottom:stride,left:right:stride]
        matrix=list(intrinsics)
        matrix[0]/=stride;matrix[4]/=stride
        matrix[2]=(matrix[2]-left)/stride;matrix[5]=(matrix[5]-top)/stride
        box=[(x1-left)/stride,(y1-top)/stride,(x2-left)/stride,(y2-top)/stride]
        estimates.append(estimate_clustered_depth(region,box,matrix,minimum_depth_m=.30,
            maximum_depth_m=15,minimum_samples=24,minimum_valid_fraction=.12))
    return estimates
