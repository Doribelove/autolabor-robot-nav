"""Conservative, class-independent RGB-D evidence for small floor litter.

This is an observation/display filter, not a waste classifier or motion gate.
Unknown evidence is retained. No inpainting, image-space exclusion zones, or
semantic blacklist is used. All geometry is computed from the current frame.
"""
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np


@dataclass(frozen=True)
class FilterConfig:
    enabled: bool = True
    max_depth_m: float = 8.0
    ground_seed_height_m: float = .18
    ground_tolerance_m: float = .025
    ground_max_tilt_deg: float = 12.0
    candidate_max_height_m: float = .45
    candidate_max_span_m: float = .60
    environment_min_top_m: float = .70
    environment_min_height_span_m: float = .50
    environment_min_width_m: float = .85
    confirm_frames: int = 3
    confirm_duration_sec: float = .12
    maximum_gap_sec: float = .25
    association_iou: float = .65
    max_geometry_objects: int = 24
    budget_ms: float = 20.0

    def __post_init__(self):
        if type(self.enabled) is not bool:
            raise ValueError('enabled must be a boolean')
        for key, value in asdict(self).items():
            if key != 'enabled' and (isinstance(value, bool) or
                                    not isinstance(value, (int, float)) or
                                    not math.isfinite(value) or value <= 0):
                raise ValueError('Invalid litter filter parameter: ' + key)
        if (type(self.confirm_frames) is not int or not 2 <= self.confirm_frames <= 10 or
                type(self.max_geometry_objects) is not int or not 1 <= self.max_geometry_objects <= 100 or
                not .3 < self.max_depth_m <= 15 or not .005 <= self.ground_tolerance_m <= .05 or
                not self.ground_tolerance_m < self.ground_seed_height_m <= .25 or
                not 1 <= self.ground_max_tilt_deg <= 15 or
                not self.candidate_max_height_m < self.environment_min_top_m <= 2 or
                not self.candidate_max_span_m < self.environment_min_width_m <= 3 or
                not self.ground_tolerance_m < self.environment_min_height_span_m <= 2 or
                not .05 <= self.confirm_duration_sec <= .5 or
                not .05 <= self.maximum_gap_sec <= .5 or
                not .5 <= self.association_iou <= .95 or not 1 <= self.budget_ms <= 100):
            raise ValueError('Litter filter thresholds outside conservative bounds')

    @property
    def digest(self):
        return hashlib.sha256(json.dumps(asdict(self), sort_keys=True).encode()).hexdigest()


def _points(depth, u, v, k, mount):
    xyz = np.stack(((u-k[2])*depth/k[0], (v-k[5])*depth/k[4], depth), axis=-1)
    return np.einsum('...i,ji->...j', xyz, mount['optical_to_base'], optimize=False) + mount['left_position']


def _plane(points):
    center = np.mean(points, axis=0)
    centered = points-center
    # Only a 3x3 eigensystem: avoid BLAS launching many threads for skinny
    # thousands-by-three products when used by offline tools or other callers.
    _, vectors = np.linalg.eigh(np.einsum('ni,nj->ij', centered, centered, optimize=False))
    normal = vectors[:, 0]
    if normal[2] < 0:
        normal = -normal
    return normal, -float(normal @ center)


def ground_plane(depth, k, mount, boxes, cfg, deadline):
    """Bounded RANSAC near the calibrated floor, excluding every raw box.

    Both spatial support and height/tilt priors are required: a visible wall or
    tabletop must never silently replace the floor. Missing floor => unknown.
    """
    h, w = depth.shape
    stride = max(1, math.ceil(max(w/96, h/64)))
    v, u = np.mgrid[0:h:stride, 0:w:stride]
    z = depth[::stride, ::stride]
    valid = np.isfinite(z) & (z >= .3) & (z <= cfg.max_depth_m)
    for d in boxes:
        x1, y1, x2, y2 = d['bbox']
        valid &= ~((u >= x1-3) & (u <= x2+3) & (v >= y1-3) & (v <= y2+3))
    points = _points(z[valid], u[valid], v[valid], k, mount)
    points = points[np.abs(points[:, 2]) <= cfg.ground_seed_height_m]
    if len(points) < 100:
        return None
    rng = np.random.RandomState(7)
    best = np.zeros(len(points), dtype=bool)
    upright = math.cos(math.radians(cfg.ground_max_tilt_deg))
    for _ in range(32):
        if time.monotonic() >= deadline:
            return None
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        n = np.cross(b-a, c-a)
        norm = np.linalg.norm(n)
        if norm < 1e-8:
            continue
        n /= norm
        if abs(n[2]) < upright:
            continue
        inliers = np.abs(np.einsum('ij,j->i', points, n)-a @ n) <= cfg.ground_tolerance_m
        if np.count_nonzero(inliers) > np.count_nonzero(best):
            best = inliers
            if np.count_nonzero(best) >= .95*len(points):
                break
    if np.count_nonzero(best) < max(100, .65*len(points)):
        return None
    n, offset = _plane(points[best])
    inliers = np.abs(np.einsum('ij,j->i', points, n)+offset) <= cfg.ground_tolerance_m
    support = points[inliers]
    if (len(support) < max(100, .65*len(points)) or n[2] < upright or
            abs(offset) > cfg.ground_seed_height_m):
        return None
    span = np.percentile(support[:, :2], 90, axis=0) - np.percentile(support[:, :2], 10, axis=0)
    if span[0] < .5 or span[1] < .3:
        return None
    return dict(normal=n, offset=offset, samples=len(support),
                residual_m=float(np.median(np.abs(np.einsum('ij,j->i', support, n)+offset))))


def geometry_evidence(box, depth, k, mount, ground, cfg):
    """Measure visible foreground support, never box-size times one Z sample."""
    metrics = {}
    def unknown(reason):
        return dict(evidence='uncertain', reason=reason, metrics=metrics)
    h, w = depth.shape
    x1, y1, x2, y2 = box
    if x1 < 2 or y1 < 2 or x2 > w-2 or y2 > h-2:
        return unknown('image_edge')
    left, top, right, bottom = int(x1), int(y1), math.ceil(x2), math.ceil(y2)
    stride = max(1, math.ceil(max(right-left, bottom-top)/64))
    v, u = np.mgrid[top:bottom:stride, left:right:stride]
    z = depth[top:bottom:stride, left:right:stride]
    valid = np.isfinite(z) & (z >= .3) & (z <= cfg.max_depth_m)
    metrics['valid_fraction'] = float(np.mean(valid))
    if np.count_nonzero(valid) < 24 or metrics['valid_fraction'] < .55:
        return unknown('sparse_depth')
    points = _points(np.where(valid, z, 0), u, v, k, mount)
    heights = np.einsum('...i,i->...', points, ground['normal']) + ground['offset']
    above = valid & (heights > max(.03, 1.5*cfg.ground_tolerance_m))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(above.astype(np.uint8), connectivity=8)
    if count <= 1:
        return unknown('flat_or_transparent')
    best = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    mask = labels == best
    rows, columns = np.nonzero(mask)
    size = len(rows)
    metrics['foreground_samples'] = size
    metrics['foreground_fraction'] = size/float(z.size)
    metrics['dominance'] = size/float(max(1, np.count_nonzero(above)))
    if size < 24 or metrics['foreground_fraction'] < .20 or metrics['dominance'] < .75:
        return unknown('weak_or_mixed_foreground')
    # Check background in narrow strips beside the box, at corresponding rows.
    # Transparent objects returning background depth must not become large
    # environmental objects just because a wall is visible through them.
    pad = max(4, math.ceil((right-left)*.18))
    outside = np.concatenate((depth[top:bottom:stride, max(0,left-pad):left:stride],
                              depth[top:bottom:stride, right:min(w,right+pad):stride]), axis=1)
    outside_valid = np.isfinite(outside) & (outside >= .3) & (outside <= cfg.max_depth_m)
    references = np.full(z.shape[0], np.nan)
    for row in range(len(references)):
        if np.count_nonzero(outside_valid[row]) >= 2:
            references[row] = np.median(outside[row][outside_valid[row]])
    separated = np.isfinite(references[rows]) & ((references[rows]-z[mask]) > np.maximum(.08, .03*z[mask]))
    metrics['background_separation_fraction'] = float(np.mean(separated))
    if metrics['background_separation_fraction'] < .60:
        return unknown('background_not_separated')
    selected = points[mask]
    low, high = np.percentile(selected, [5, 95], axis=0)
    min_height, max_height = np.percentile(heights[mask], [5, 95])
    spans = high-low
    metrics.update(height_low_m=float(min_height), height_top_m=float(max_height),
                   height_span_m=float(max_height-min_height), span_x_m=float(spans[0]),
                   span_y_m=float(spans[1]), aspect_ratio_px=float((x2-x1)/(y2-y1)))
    # A small foreground occluder inside a large background box cannot authorize
    # removal of the whole box. Require measured support over both image axes.
    coverage_x = (columns.max()-columns.min()+1)/z.shape[1]
    coverage_y = (rows.max()-rows.min()+1)/z.shape[0]
    if coverage_x < .55 or coverage_y < .55:
        return unknown('partial_support')
    if (max_height > cfg.environment_min_top_m and
            max_height-min_height > cfg.environment_min_height_span_m):
        return dict(evidence='environment', reason='tall_structure', metrics=metrics)
    if spans[1] > cfg.environment_min_width_m and max_height > cfg.candidate_max_height_m:
        return dict(evidence='environment', reason='wide_tall_structure', metrics=metrics)
    if (min_height <= .12 and max_height <= cfg.candidate_max_height_m and
            max(spans[:2]) <= cfg.candidate_max_span_m):
        return dict(evidence='candidate', reason='small_floor_object', metrics=metrics)
    return unknown('geometry_ambiguous')


class LitterFilter:
    def __init__(self, config=None):
        self.config = config or FilterConfig()
        self.implementation_sha256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        self.previous = []
        self.last_stamp = 0.

    def process(self, boxes, depth, intrinsics, mount, calibrated, stamp):
        started = time.monotonic()
        cfg = self.config
        if not math.isfinite(stamp) or stamp <= self.last_stamp:
            raise ValueError('Filter requires strictly increasing source stamps')
        previous = self.previous if stamp-self.last_stamp <= cfg.maximum_gap_sec else []
        self.last_stamp = stamp
        k = np.asarray(intrinsics, dtype=float) if intrinsics is not None else np.array([])
        usable = (cfg.enabled and calibrated and depth is not None and depth.ndim == 2 and
                  k.shape == (9,) and np.isfinite(k).all() and k[0] > 0 and k[4] > 0)
        ground = ground_plane(depth, k, mount, boxes, cfg, started+cfg.budget_ms/1000) if usable and boxes else None
        ground_reason = ('disabled' if not cfg.enabled else 'uncalibrated' if not calibrated else
                         'no_synchronized_depth' if not usable else 'ground_unavailable')
        decisions = []
        used = set()
        # Conservative mutual one-to-one association. Ambiguous overlapping raw
        # boxes reset evidence rather than transferring rejection to a neighbor.
        matched = np.zeros((len(boxes),len(previous)),dtype=bool)
        if boxes and previous:
            current_boxes = np.asarray([d['bbox'] for d in boxes],dtype=float)
            previous_boxes = np.asarray([p['bbox'] for p in previous],dtype=float)
            intersections = np.prod(np.maximum(0,np.minimum(current_boxes[:,None,2:],previous_boxes[None,:,2:])-
                                                  np.maximum(current_boxes[:,None,:2],previous_boxes[None,:,:2])),axis=2)
            current_area = np.prod(current_boxes[:,2:]-current_boxes[:,:2],axis=1)
            previous_area = np.prod(previous_boxes[:,2:]-previous_boxes[:,:2],axis=1)
            matched = intersections/np.maximum(1e-9,current_area[:,None]+previous_area[None,:]-intersections) >= cfg.association_iou
        current_matches = np.sum(matched,axis=1)
        previous_matches = np.sum(matched,axis=0)
        for index, d in enumerate(boxes):
            evidence = dict(evidence='uncertain', reason=ground_reason, metrics={})
            if ground is not None:
                if index >= cfg.max_geometry_objects or (time.monotonic()-started)*1000 >= cfg.budget_ms:
                    evidence['reason'] = 'geometry_budget'
                else:
                    evidence = geometry_evidence(d['bbox'], depth, k, mount, ground, cfg)
            decision = dict(evidence, state=evidence['evidence'], bbox=list(d['bbox']),
                            object_id=d['object_id'], class_id=d['class_id'],
                            class_name=d['class_name'], confidence=d['confidence'],
                            rejection_streak=0, streak_start=stamp)
            prior = None
            if current_matches[index] == 1:
                j = int(np.argmax(matched[index]))
                if j not in used and previous_matches[j] == 1:
                    prior = previous[j]
                    used.add(j)
            if evidence['evidence'] == 'environment':
                same = prior is not None and prior['reason'] == decision['reason'] and prior['rejection_streak'] > 0
                decision['rejection_streak'] = min(cfg.confirm_frames, prior['rejection_streak']+1) if same else 1
                decision['streak_start'] = prior['streak_start'] if same else stamp
                if (decision['rejection_streak'] < cfg.confirm_frames or
                        stamp-decision['streak_start']+1e-9 < cfg.confirm_duration_sec):
                    decision['state'] = 'uncertain'
            decisions.append(decision)
        self.previous = decisions
        plane_report = dict(valid=ground is not None)
        if ground is not None:
            plane_report.update(normal=ground['normal'].tolist(), offset_m=ground['offset'],
                                samples=ground['samples'], residual_m=ground['residual_m'])
        return dict(schema_version=1, source_stamp=stamp, filter_sha256=cfg.digest,
                    implementation_sha256=self.implementation_sha256,
                    enabled=cfg.enabled, ground=plane_report, decisions=decisions,
                    counts={state: sum(d['state'] == state for d in decisions)
                            for state in ('candidate', 'uncertain', 'environment')},
                    elapsed_ms=(time.monotonic()-started)*1000, motion_eligible=False)
