"""Class-independent paired evaluation on exhaustively labeled source frames."""
import math
from autolabor_bpu_perception.core import bbox_iou


def _box(box):
    if (len(box) != 4 or not all(isinstance(x,(int,float)) and math.isfinite(x) for x in box)
            or box[2] <= box[0] or box[3] <= box[1]):
        raise ValueError('Malformed annotation/detection box')
    return box


def _key(frame):
    session = frame.get('session_id')
    stamp = frame.get('source_stamp_ns')
    if not isinstance(session,str) or not session or type(stamp) is not int or stamp <= 0:
        raise ValueError('Each frame needs session_id and integer source_stamp_ns')
    return session,stamp


def _counts(boxes, truth, threshold):
    # Maximum-cardinality one-to-one matching. A duplicate prediction cannot
    # cover two objects and greedy matching cannot invent avoidable misses.
    edges = [[j for j,gt in enumerate(truth) if bbox_iou(box,gt) >= threshold] for box in boxes]
    assigned = {}
    def augment(index,visited):
        for j in edges[index]:
            if j in visited:continue
            visited.add(j)
            if j not in assigned or augment(assigned[j],visited):
                assigned[j]=index
                return True
        return False
    for index in range(len(boxes)):augment(index,set())
    return dict(tp=len(assigned),fp=len(boxes)-len(assigned),fn=len(truth)-len(assigned))


def evaluate(audits, annotations, iou_threshold=.5):
    if not .1 <= iou_threshold <= 1:raise ValueError('Invalid IoU threshold')
    indexed = {}
    for audit in audits:
        if audit.get('valid') is not True:continue
        key = _key(audit)
        if key in indexed:raise ValueError('Duplicate source audit frame')
        if audit.get('schema_version') != 1:raise ValueError('Unsupported audit schema')
        indexed[key] = audit
    frames = annotations.get('frames')
    if annotations.get('exhaustive_trash_labels') is not True or not isinstance(frames,list) or not frames:
        raise ValueError('Provide nonempty, exhaustively labeled trash frames')
    totals = {name:dict(tp=0,fp=0,fn=0) for name in ('raw','filtered')}
    evaluated=set();digests=set();per_frame=[]
    for frame in frames:
        key=_key(frame)
        if key in evaluated:raise ValueError('Duplicate labeled frame')
        if key not in indexed:raise ValueError('Labeled frame has no valid paired audit: '+str(key))
        evaluated.add(key)
        audit=indexed[key];digests.add((audit['filter_sha256'],audit['implementation_sha256']))
        truth=[_box(box) for box in frame['trash_boxes']]
        decisions=audit['decisions']
        if any(d['state'] not in ('candidate','uncertain','environment') for d in decisions):
            raise ValueError('Unknown filter state')
        raw=[_box(d['bbox']) for d in decisions]
        kept=[_box(d['bbox']) for d in decisions if d['state']!='environment']
        entry=dict(session_id=key[0],source_stamp_ns=key[1])
        for name,boxes in (('raw',raw),('filtered',kept)):
            entry[name]=_counts(boxes,truth,iou_threshold)
            for metric,value in entry[name].items():totals[name][metric]+=value
        per_frame.append(entry)
    if len(digests)!=1:raise ValueError('Evaluate different implementations/configurations separately')
    for counts in totals.values():
        counts['precision']=counts['tp']/(counts['tp']+counts['fp']) if counts['tp']+counts['fp'] else None
        counts['recall']=counts['tp']/(counts['tp']+counts['fn']) if counts['tp']+counts['fn'] else None
    config_sha,implementation_sha=digests.pop()
    return dict(labeled_frames=len(evaluated),iou_threshold=iou_threshold,filter_sha256=config_sha,
                implementation_sha256=implementation_sha,
                counts=totals,additional_false_negatives=totals['filtered']['fn']-totals['raw']['fn'],
                removed_false_positives=totals['raw']['fp']-totals['filtered']['fp'],per_frame=per_frame,
                interpretation='Counts are per labeled frame, not distinct physical objects or mAP')
