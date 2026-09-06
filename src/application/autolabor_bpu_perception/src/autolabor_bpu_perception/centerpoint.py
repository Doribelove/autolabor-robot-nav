"""Decode the pinned HEAL six-head CenterPoint outputs; single-sweep velocity is untrusted."""
import math
import numpy as np

CLASSES=('car','truck','construction_vehicle','bus','trailer','barrier',
         'motorcycle','bicycle','pedestrian','traffic_cone')
HEAD_CLASSES=(1,2,2,1,2,2)
SHAPES=[(128,128,c) for n in HEAD_CLASSES for c in (2,1,3,2,2,n)]
OUTPUT_FLOATS=sum(int(np.prod(s)) for s in SHAPES)


def decode(outputs, threshold=0.45, limit=100):
    if not 0<threshold<1 or not 1<=limit<=200 or len(outputs)!=36:
        raise ValueError('Invalid CenterPoint decoder configuration')
    for output,shape in zip(outputs,SHAPES):
        if output.shape!=shape or not np.isfinite(output).all():
            raise ValueError('Invalid CenterPoint output')
    detections=[]; class_offset=0
    for head,nclasses in enumerate(HEAD_CLASSES):
        reg,height,dim,rot,vel,logits=outputs[head*6:head*6+6]
        scores=1/(1+np.exp(-np.clip(logits,-60,60)))
        flat=scores.reshape(-1)
        selected=np.flatnonzero(flat>=threshold)
        if selected.size>limit:
            selected=selected[np.argpartition(flat[selected],-limit)[-limit:]]
        for index in selected:
            y,x,c=np.unravel_index(index,scores.shape)
            dimensions=np.exp(np.clip(dim[y,x],-8,5))
            position=[float((x+reg[y,x,0])*.8-51.2),float((y+reg[y,x,1])*.8-51.2),float(height[y,x,0])]
            if np.any(dimensions<0.05) or np.any(dimensions>20) or not -5<=position[2]<=3:
                continue
            rotation_norm=float(np.linalg.norm(rot[y,x]))
            if rotation_norm<1e-5:
                continue
            detections.append(dict(class_id=class_offset+int(c),class_name=CLASSES[class_offset+int(c)],
                confidence=float(scores[y,x,c]),position=position,dimensions=dimensions.tolist(),
                yaw=math.atan2(float(rot[y,x,0]),float(rot[y,x,1])),
                velocity=[float(vel[y,x,0]),float(vel[y,x,1]),0.0],velocity_valid=False))
        class_offset+=nclasses
    # Bounded class-aware center suppression for the initial sensor-domain trial.
    # This policy is explicit; it is not represented as a HEAL golden decoder.
    kept=[]
    for d in sorted(detections,key=lambda d:d['confidence'],reverse=True):
        radius=min(1.0,max(.15,.25*min(d['dimensions'][:2])))
        if any(k['class_id']==d['class_id'] and math.hypot(k['position'][0]-d['position'][0],
                k['position'][1]-d['position'][1])<radius for k in kept):
            continue
        kept.append(d)
        if len(kept)==limit:break
    return kept
