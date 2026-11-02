"""Localize original-view detections on measured ground, then fuse observations.

Bounding-box bottom centre estimates ground contact, not object depth or a
measured footprint centroid. No claim of calibrated covariance/map accuracy.
"""
import math
import cv2
import numpy as np


def ground_ray(k, transform, pixel):
    k, transform = np.asarray(k, float), np.asarray(transform, float)
    if (k.shape != (3, 3) or transform.shape != (4, 4) or
            not np.isfinite(k).all() or not np.isfinite(transform).all() or
            min(k[0,0],k[1,1]) <= 0 or not np.allclose(k[2],[0,0,1]) or
            not np.allclose(transform[:3,:3].T @ transform[:3,:3],np.eye(3),atol=1e-6) or
            not np.allclose(transform[3], [0, 0, 0, 1]) or transform[2, 3] <= 0):
        raise ValueError('Invalid camera calibration')
    ray = transform[:3, :3] @ np.linalg.solve(k, [*pixel, 1.])
    if not np.isfinite(ray).all() or ray[2] >= -1e-6:
        return None
    distance = -transform[2, 3]/ray[2]
    point = transform[:3, 3]+distance*ray
    return point[:2]


def localize_detections(meta, detections, merge_radius=.15, merge_material_classes=True):
    """One-frame fusion; never merge two detections from the same camera.

    Complete-link matching avoids chaining several nearby pieces into one.
    Material-only detectors can merge conflicting material labels. Generic
    object detectors set merge_material_classes=False to retain class identity.
    Missing or incompatible exposure clocks remain explicit in source metadata.
    """
    if not 0 <= merge_radius <= .5:
        raise ValueError('Invalid overlap association radius')
    # The square preview can contain black padding beyond the body-edge limits.
    bounds = meta.get('observation_bounds_m', meta)
    xmin, xmax = bounds['x_range_m']; ymin, ymax = bounds['y_range_m']
    candidates, rejected = [], []
    for name, rows in sorted(detections.items()):
        width, height = meta['camera_image_sizes'][name]
        k = meta['projection_intrinsics'][name]
        t = meta['T_ground_from_upright_optical'][name]
        for index, item in enumerate(rows[:100]):
            reason = None
            x0, y0, x1, y1 = map(float, item['bbox'])
            score = float(item['confidence'])
            if (not np.isfinite([x0, y0, x1, y1, score]).all() or not 0 <= score <= 1
                    or not 0 <= x0 < x1 <= width or not 0 <= y0 < y1 <= height):
                reason = 'invalid_box'
            elif y1 >= height-1:
                reason = 'bottom_contact_clipped'
            pixel = [(x0+x1)/2, y1]
            anchor = item.get('anchor')
            if anchor is not None and len(anchor) == 2 and np.isfinite(anchor).all():
                if x0 <= anchor[0] <= x1 and y0 <= anchor[1] <= y1:
                    pixel = list(map(float, anchor))
            for polygon in meta.get('camera_exclusion_polygons', {}).get(name, []):
                if cv2.pointPolygonTest(np.asarray(polygon, np.float32), tuple(pixel), False) >= 0:
                    reason = 'vehicle_image_mask'
            point = ground_ray(k, t, pixel) if reason is None else None
            if reason is None:
                if point is None:
                    reason = 'ray_above_horizon'
                elif not xmin <= point[0] < xmax or not ymin <= point[1] < ymax:
                    reason = 'outside_bev_range'
            if reason:
                rejected.append(dict(camera=name, detection_index=index, reason=reason))
                continue
            # Pixel perturbation describes projection sensitivity only; it
            # does not account for unknown installation/pitch/ground error.
            perturbations = [ground_ray(k, t, [pixel[0]+du, pixel[1]+dv])
                             for du, dv in ((2, 0), (-2, 0), (0, 2), (0, -2))]
            sigma = max([float(np.linalg.norm(p-point)) for p in perturbations if p is not None]+[.01])
            candidates.append(dict(x=float(point[0]), y=float(point[1]), label=str(item['class_name']),
                class_id=int(item.get('class_id', 0)), confidence=score, camera=name,
                detection_index=index, image_bbox=[x0, y0, x1, y1], image_anchor=pixel,
                pixel_sensitivity_m=sigma, position_method='ground_contact_ray_plane'))
    def projects_into_box(point, observation):
        name = observation['camera']
        transform = np.asarray(meta['T_ground_from_upright_optical'][name])
        camera = (np.array([point['x'], point['y'], 0.])-transform[:3, 3]) @ transform[:3, :3]
        if camera[2] <= .05:
            return False
        pixel = np.asarray(meta['projection_intrinsics'][name]) @ camera
        u, v = pixel[:2]/pixel[2]
        x0, y0, x1, y1 = observation['image_bbox']
        # A small image tolerance covers detector/contact rounding. The ground
        # radius still bounds association; a close point alone is insufficient.
        pad = 3.
        return x0-pad <= u <= x1+pad and y0-pad <= v <= y1+pad

    groups = []
    for item in sorted(candidates, key=lambda o: (-o['confidence'], o['camera'], o['detection_index'])):
        choices = []
        for index, group in enumerate(groups):
            if any(m['camera'] == item['camera'] for m in group):
                continue
            if not merge_material_classes and any(m['class_id'] != item['class_id'] for m in group):
                continue
            distances = [math.hypot(m['x']-item['x'], m['y']-item['y']) for m in group]
            if (max(distances) <= merge_radius and
                    all(projects_into_box(m, item) and projects_into_box(item, m) for m in group)):
                choices.append((max(distances), index))
        if choices:
            groups[min(choices)[1]].append(item)
        else:
            groups.append([item])
    objects = []
    for group in groups:
        weights = [m['confidence']/max(.02, m['pixel_sensitivity_m'])**2 for m in group]
        total = max(sum(weights), 1e-10)
        objects.append(dict(x=sum(m['x']*w for m,w in zip(group,weights))/total,
            y=sum(m['y']*w for m,w in zip(group,weights))/total,
            label=group[0]['label'], class_id=group[0]['class_id'],
            confidence=max(m['confidence'] for m in group), source_cameras=sorted(m['camera'] for m in group),
            members=group, position_method='ground_contact_ray_plane', frame_id=meta['frame_id'],
            association_method='ground_distance_and_mutual_box_reprojection',
            association_count=len(group), class_disagreement=len({m['label'] for m in group}) > 1))
    return objects[:100], rejected


def render_ground_detections(image, meta, objects):
    """Draw point markers in BEV; their raster boxes are UI markers, not boxes predicted in BEV."""
    view, display = image.copy(), []
    height, width = view.shape[:2]
    resolution = meta['metres_per_pixel']
    grid = view.copy()
    for x in np.arange(math.ceil(meta['x_range_m'][0]),meta['x_range_m'][1],1.):
        row = int(round((meta['x_range_m'][1]-x)/resolution-.5))
        cv2.line(grid,(0,row),(width-1,row),(160,160,160),1)
    for y in np.arange(math.ceil(meta['y_range_m'][0]),meta['y_range_m'][1],1.):
        col = int(round((meta['y_range_m'][1]-y)/resolution-.5))
        cv2.line(grid,(col,0),(col,height-1),(160,160,160),1)
    view = cv2.addWeighted(view,.75,grid,.25,0)
    view[~image.any(axis=2)] = 0
    origin = (int(round(meta['y_range_m'][1]/resolution-.5)),int(round(meta['x_range_m'][1]/resolution-.5)))
    cv2.arrowedLine(view,origin,(origin[0],origin[1]-28),(0,200,255),2)
    cv2.putText(view,'rig (0,0)',(origin[0]+8,origin[1]+12),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,200,255),1)
    cv2.putText(view,'FRONT +X | LEFT +Y | grid 1m',(12,20),cv2.FONT_HERSHEY_SIMPLEX,.45,(220,220,220),1)
    for index, item in enumerate(objects):
        u = (meta['y_range_m'][1]-item['y'])/resolution-.5
        v = (meta['x_range_m'][1]-item['x'])/resolution-.5
        marker = [max(0.,u-6), max(0.,v-6), min(float(width),u+6), min(float(height),v+6)]
        if marker[0] >= marker[2] or marker[1] >= marker[3]:
            continue
        cv2.drawMarker(view, (int(round(u)), int(round(v))), (0, 245, 255), cv2.MARKER_CROSS, 13, 2)
        text = '%d %s %.2f,%.2fm' % (index+1, item['label'], item['x'], item['y'])
        cv2.putText(view, text, (max(0,min(int(round(u))+9,width-215)), max(15,int(round(v))-9)),
                    cv2.FONT_HERSHEY_SIMPLEX, .43, (0,245,255), 1, cv2.LINE_AA)
        display.append(dict(class_id=item['class_id'], class_name=item['label'],
                            confidence=item['confidence'], bbox=marker, bbox_role='ground_point_marker'))
    return view, display
