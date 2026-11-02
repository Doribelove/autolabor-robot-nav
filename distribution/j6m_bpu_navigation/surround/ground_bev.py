"""Ground-plane projection for a measured, heterogeneous camera rig.

Inputs are perspective images: hardware GDC for the fisheyes and SDK-rectified
left color for ZED. This module never performs fisheye undistortion.
"""
import hashlib
import copy
import json
import math

import cv2
import numpy as np


def resolve_observation_region(config):
    """Derive metric limits from physical body edges, with square display padding.

    The observation bounds exclude the padding used by the existing 600px Qt
    preview. Legacy rigs without edge settings retain their original extents.
    """
    region = config.get('observation_region')
    if region is None:
        return config
    if region.get('reference') != 'vehicle_edges':
        raise ValueError('Expected vehicle_edges observation reference')
    directions = ('front', 'rear', 'left', 'right')
    edges = {n: float(region['vehicle_edges_m'][n]) for n in directions}
    margins = {n: float(region['distance_from_edges_m'][n]) for n in directions}
    if any(not math.isfinite(v) or v < 0 for v in (*edges.values(), *margins.values())):
        raise ValueError('Vehicle edges and observation distances must be finite and nonnegative')
    bounds = dict(x_range_m=[-edges['rear']-margins['rear'], edges['front']+margins['front']],
                  y_range_m=[-edges['right']-margins['right'], edges['left']+margins['left']])
    bounds = {k: [round(v, 12) for v in values] for k, values in bounds.items()}
    pixels = config.get('preview_canvas_pixels', 600)
    if type(pixels) is not int or not 2 <= pixels <= 2048:
        raise ValueError('Invalid BEV preview canvas size')
    span = round(max(v[1]-v[0] for v in bounds.values()), 12)
    if span <= 0:
        raise ValueError('Empty observation region')
    result = copy.deepcopy(config)
    result['observation_bounds_m'] = bounds
    for key, limits in bounds.items():
        centre = (limits[0]+limits[1])/2
        result[key] = [round(centre-span/2, 12), round(centre+span/2, 12)]
    result['metres_per_pixel'] = span/pixels
    return result


def optical_rotation(yaw_left_deg, pitch_down_deg, optical_roll_deg=0):
    """Upright optical x-right/y-down/z-forward to rig x-forward/y-left/z-up."""
    yaw, pitch = np.deg2rad([yaw_left_deg, pitch_down_deg])
    cy, sy, cp, sp = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch)
    yaw_rotation = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    pitch_rotation = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    optical_to_forward = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    roll = np.deg2rad(optical_roll_deg)
    cr, sr = np.cos(roll), np.sin(roll)
    optical_roll = np.array([[cr, -sr, 0], [sr, cr, 0], [0, 0, 1]])
    return yaw_rotation @ pitch_rotation @ optical_to_forward @ optical_roll


def corrected_intrinsics(k, size, rotate_180):
    """Intrinsics in the upright image; physical roll is removed exactly once."""
    k = np.asarray(k, dtype=float).copy()
    if rotate_180:
        k[0, 2] = size[0]-1-k[0, 2]
        k[1, 2] = size[1]-1-k[1, 2]
    return k


class GroundBev:
    def __init__(self, config):
        config = resolve_observation_region(config)
        self.config = config
        self.cameras = config['cameras']
        if set(self.cameras) not in ({'front', 'left', 'right'}, {'front', 'left', 'right', 'rear'}):
            raise ValueError('Expected front/left/right cameras, optionally with rear')
        self.resolution = float(config['metres_per_pixel'])
        xmin, xmax = config['x_range_m']
        ymin, ymax = config['y_range_m']
        if not (0.005 <= self.resolution <= .2 and xmin < xmax and ymin < ymax):
            raise ValueError('Invalid ground extent or resolution')
        self.width = math.ceil((ymax-ymin)/self.resolution-1e-9)
        self.height = math.ceil((xmax-xmin)/self.resolution-1e-9)
        if not 2 <= min(self.width, self.height) <= max(self.width, self.height) <= 2048:
            raise ValueError('Ground image dimensions out of bounds')
        row, col = np.indices((self.height, self.width), dtype=float)
        self.x = xmax-(row+.5)*self.resolution
        self.y = ymax-(col+.5)*self.resolution
        self.world = np.stack((self.x, self.y, np.zeros_like(self.x)), axis=-1)
        bounds = config.get('observation_bounds_m', config)
        bx0, bx1 = bounds['x_range_m']
        by0, by1 = bounds['y_range_m']
        region_support = ((self.x >= bx0) & (self.x < bx1) &
                          (self.y >= by0) & (self.y < by1))
        self.maps = {}
        self.transforms = {}
        self.intrinsics = {}
        self.blend_mode = config.get('blend_mode', 'weighted')
        if self.blend_mode not in ('weighted', 'single_source'):
            raise ValueError('Unsupported ground blend mode')
        quality = {}
        for name, camera in self.cameras.items():
            missing = [key for key in ('perspective_size', 'K_perspective',
                'position_ground_rig_m', 'yaw_left_deg', 'pitch_down_deg', 'rotate_180')
                if camera.get(key) is None]
            if missing:
                raise ValueError('Missing installation parameters for %s: %s' %
                                 (name, ', '.join(missing)))
            size = camera['perspective_size']
            if len(size) != 2 or min(size) < 2:
                raise ValueError('Invalid camera size: '+name)
            k = corrected_intrinsics(camera['K_perspective'], size, camera['rotate_180'])
            position = np.asarray(camera['position_ground_rig_m'], dtype=float)
            rotation = optical_rotation(camera['yaw_left_deg'], camera['pitch_down_deg'],
                                        camera.get('upright_optical_roll_deg', 0))
            if (position.shape != (3,) or not np.isfinite(position).all() or position[2] <= 0
                    or k.shape != (3, 3) or not np.isfinite(k).all() or
                    min(k[0, 0], k[1, 1]) <= 0 or not np.allclose(k[2], [0, 0, 1])):
                raise ValueError('Invalid camera geometry: '+name)
            transform = np.eye(4)
            transform[:3, :3], transform[:3, 3] = rotation, position
            self.transforms[name], self.intrinsics[name] = transform, k
            uv, depth = self.project(name, self.world)
            mx, my = uv[..., 0].astype('float32'), uv[..., 1].astype('float32')
            edge = np.minimum.reduce((mx, my, size[0]-1-mx, size[1]-1-my))
            valid = ((depth > .05) & np.isfinite(mx) & np.isfinite(my) & (edge >= 1) & region_support)
            # Edge feathering blends only physically observed ground pixels.
            # Looking close to the horizon receives less weight than looking down.
            incidence = position[2]/np.maximum(np.linalg.norm(self.world-position, axis=-1), .01)
            weight = (np.clip(edge/config['feather_pixels'], 0, 1)*incidence**2*valid).astype('float32')
            if camera.get('exclude_polygons_upright_px'):
                source_mask = np.ones((size[1], size[0]), np.float32)
                for polygon in camera['exclude_polygons_upright_px']:
                    cv2.fillPoly(source_mask, [np.asarray(polygon, np.int32)], 0)
                weight *= cv2.remap(source_mask, mx, my, cv2.INTER_LINEAR,
                                    borderMode=cv2.BORDER_CONSTANT)
            # Minimum singular value of d(image pixel)/d(ground metre).
            # Prefer the view with better sampling in its worst direction,
            # accounting for each camera's focal length, height and pose.
            # This is geometric sampling quality, not measured image sharpness.
            q = np.zeros_like(weight)
            visible = weight > 0
            d = depth[visible]
            u, v = uv[visible].T
            jacobian = rotation @ k.T
            a = (jacobian[0, 0]-u*jacobian[0, 2])/d
            b = (jacobian[1, 0]-u*jacobian[1, 2])/d
            c = (jacobian[0, 1]-v*jacobian[0, 2])/d
            e = (jacobian[1, 1]-v*jacobian[1, 2])/d
            trace = a*a+b*b+c*c+e*e
            determinant_squared = (a*e-b*c)**2
            largest_squared = .5*(trace+np.sqrt(np.maximum(0, trace*trace-4*determinant_squared)))
            # det/sigma_max is more stable than subtracting similar eigenvalues.
            q[visible] = np.sqrt(determinant_squared/np.maximum(largest_squared, 1e-12))
            q[visible] *= np.clip(edge[visible]/config['feather_pixels'], 0, 1)
            quality[name] = q
            mx[~valid], my[~valid] = -1, -1
            self.maps[name] = mx, my, weight
        # Installation geometry is fixed for this instance. Cache visibility,
        # normalized weights and OpenCV's interpolation maps once, not per frame.
        weights = np.sum([m[2] for m in self.maps.values()], axis=0)
        self.support_counts = np.sum([m[2] > 0 for m in self.maps.values()],
                                     axis=0).astype(np.uint8)
        self.support_counts.setflags(write=False)
        self.source_names = sorted(self.cameras)
        self.source_owner = np.argmax(np.stack([quality[n] for n in self.source_names]), axis=0).astype(np.int8)
        self.source_owner[self.support_counts == 0] = -1
        self.source_owner.setflags(write=False)
        self.render_maps = {}
        for name, (mx, my, weight) in self.maps.items():
            rows, cols = np.nonzero(weight > 0)
            if not len(rows):
                self.render_maps[name] = None
                continue
            roi = (slice(rows.min(), rows.max()+1), slice(cols.min(), cols.max()+1))
            map1, map2 = cv2.convertMaps(mx[roi], my[roi], cv2.CV_16SC2)
            normalized = ((self.source_owner[roi] == self.source_names.index(name)).astype(np.float32)
                          if self.blend_mode == 'single_source' else
                          weight[roi]/np.maximum(weights[roi], 1e-10))
            self.render_maps[name] = (roi, map1, map2, weight[roi] <= 0,
                                     np.repeat(normalized[..., None], 3, axis=2))
        self.config_sha256 = hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest()

    def project(self, name, points):
        t = self.transforms[name]
        points = (np.asarray(points)-t[:3, 3]) @ t[:3, :3]
        pixels = points @ self.intrinsics[name].T
        with np.errstate(divide='ignore', invalid='ignore'):
            uv = pixels[..., :2]/pixels[..., 2:3]
        return uv, points[..., 2]

    def pixel(self, x, y):
        return ((self.config['y_range_m'][1]-y)/self.resolution-.5,
                (self.config['x_range_m'][1]-x)/self.resolution-.5)

    def upright(self, name, image):
        expected = self.cameras[name]['perspective_size']
        if (image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3
                or list(image.shape[1::-1]) != expected):
            raise ValueError('Unexpected perspective frame: '+name)
        return cv2.rotate(image, cv2.ROTATE_180) if self.cameras[name]['rotate_180'] else image

    def compose(self, upright_frames):
        if set(upright_frames) != set(self.cameras):
            raise ValueError('All configured camera inputs required: '+', '.join(self.cameras))
        total = np.zeros((self.height, self.width, 3), np.float32)
        layers = {}
        for name in self.cameras:
            frame = upright_frames[name]
            expected = self.cameras[name]['perspective_size']
            if (frame.dtype != np.uint8 or frame.ndim != 3 or
                    list(frame.shape[1::-1]) != expected or frame.shape[2] != 3):
                raise ValueError('Unexpected upright image: '+name)
            layer = np.zeros((self.height, self.width, 3), np.uint8)
            layers[name] = layer
            rendering = self.render_maps[name]
            if rendering is not None:
                roi, map1, map2, invisible, normalized = rendering
                projected = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR,
                                      borderMode=cv2.BORDER_CONSTANT)
                projected[invisible] = 0
                layer[roi] = projected
                total[roi] += cv2.multiply(projected, normalized, dtype=cv2.CV_32F)
        np.clip(total, 0, 255, out=total)
        return total.astype('uint8'), self.support_counts.copy(), layers
