"""Pixel exclusions and paired original-view previews for BEV detection."""
import hashlib
import json
import math
import struct
from pathlib import Path

import cv2
import numpy as np


class DetectionMasks:
    def __init__(self, path):
        data = Path(path).read_bytes()
        self.sha256 = hashlib.sha256(data).hexdigest()
        config = json.loads(data)
        if config.get('schema') != 'conbine.bev.detection_masks.v1':
            raise ValueError('Invalid detection mask schema')
        self.regions, self.masks = config['cameras'], {}
        for name, region in self.regions.items():
            width, height = region['image_size']
            if name not in ('front','left','right','rear') or not 2 <= min(width,height) <= max(width,height) <= 4096:
                raise ValueError('Invalid detection mask image size')
            mask = np.zeros((height,width), np.uint8)
            for polygon in region['polygons']:
                points = np.asarray(polygon, dtype=float)
                if (points.ndim != 2 or points.shape[1] != 2 or len(points) < 3 or
                    not np.isfinite(points).all() or (points < 0).any() or
                    (points[:,0] >= width).any() or (points[:,1] >= height).any()):
                    raise ValueError('Invalid detection mask polygon')
                cv2.fillPoly(mask, [np.rint(points).astype(np.int32)], 255)
            self.masks[name] = mask

    def apply(self, images):
        result = dict(images)
        for name, image in images.items():
            if name not in self.masks:
                continue
            mask = self.masks[name]
            if image.shape[:2] != mask.shape:
                raise ValueError('Detection mask resolution mismatch: '+name)
            result[name] = image.copy()
            result[name][mask != 0] = 0
        return result

    def filter(self, detections):
        """Exclude boxes centred/contacting the vehicle or mostly inside it.

        A small box overlap alone does not remove a neighbouring ground object.
        Keep excluded rows only as diagnostics, never as display/ground targets.
        """
        kept, excluded = {}, []
        for name, rows in detections.items():
            kept[name] = []
            mask = self.masks.get(name)
            for index, item in enumerate(rows):
                reason = None
                if mask is not None:
                    x0,y0,x1,y1 = map(float,item['bbox'])
                    h,w = mask.shape
                    if not np.isfinite([x0,y0,x1,y1]).all() or not 0 <= x0 < x1 <= w or not 0 <= y0 < y1 <= h:
                        raise ValueError('Invalid detector box for mask')
                    probes = [((x0+x1)/2,(y0+y1)/2), ((x0+x1)/2,y1)]
                    if item.get('anchor') is not None:
                        probes.append(item['anchor'])
                    if any(0 <= u < w and 0 <= v <= h and mask[min(h-1,int(v)),int(u)]
                           for u,v in probes):
                        reason = 'vehicle_mask_center_or_contact'
                    roi=mask[math.floor(y0):math.ceil(y1),math.floor(x0):math.ceil(x1)]
                    fraction=float(np.count_nonzero(roi))/max(1,roi.size)
                    if fraction >= .5:
                        reason = 'vehicle_mask_majority_overlap'
                if reason:
                    excluded.append(dict(camera=name,detection_index=index,reason=reason,detection=item))
                else:
                    kept[name].append(item)
        return kept, excluded

    def annotate(self, images, detections):
        previews = {}
        for name, original in images.items():
            debug = original.copy()
            if name in self.masks:
                mask = self.masks[name] != 0
                debug[mask] = (debug[mask].astype(np.float32)*.4+np.array([35,35,160])*.6).astype(np.uint8)
                for polygon in self.regions[name]['polygons']:
                    cv2.polylines(debug,[np.asarray(polygon,np.int32)],True,(80,80,255),2)
                cv2.putText(debug,'VEHICLE EXCLUDED',(12,debug.shape[0]-14),cv2.FONT_HERSHEY_SIMPLEX,.65,(80,100,255),2)
            for item in detections[name]:
                x0,y0,x1,y1 = map(round,item['bbox'])
                cv2.rectangle(debug,(x0,y0),(x1,y1),(0,245,255),2)
                cv2.putText(debug,'%s %.2f'%(item['class_name'],item['confidence']),
                    (max(0,x0),max(18,y0-5)),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,245,255),2)
            previews[name] = debug
        return previews


def encode_vision_frame(meta, raw_jpeg, bev_jpeg, previews):
    """BV02: same BEV header/images as BV01, followed by bounded paired views."""
    metadata = dict(meta)
    rows, chunks = [], []
    for name in ('front','left','right','rear'):
        if name not in previews:
            continue
        im = previews[name]
        ok,jpeg = cv2.imencode('.jpg',im,[cv2.IMWRITE_JPEG_QUALITY,88])
        if not ok or not 0 < len(jpeg) <= 2*1024*1024:
            raise ValueError('Invalid original-view preview JPEG')
        data=jpeg.tobytes();chunks.append(data)
        rows.append(dict(camera=name,width=im.shape[1],height=im.shape[0],bytes=len(data),
            sha256=hashlib.sha256(data).hexdigest(),source_id=meta['source_id']))
    metadata['camera_previews'] = rows
    header=json.dumps(metadata,allow_nan=False).encode()
    if len(header)>128*1024:
        raise ValueError('Vision metadata too large')
    return struct.pack('!4sIII',b'BV02',len(header),len(raw_jpeg),len(bev_jpeg))+header+raw_jpeg+bev_jpeg+b''.join(chunks)
