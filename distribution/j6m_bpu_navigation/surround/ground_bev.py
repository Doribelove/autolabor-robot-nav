"""Ground-plane projection for a measured, heterogeneous camera rig.

Inputs are perspective images: hardware GDC for the fisheyes and SDK-rectified
left color for ZED. This module never performs fisheye undistortion.
"""
import hashlib
import json
import math

import cv2
import numpy as np


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
        self.config = config
        self.cameras = config['cameras']
        self.resolution = float(config['metres_per_pixel'])
        xmin, xmax = config['x_range_m']
        ymin, ymax = config['y_range_m']
        if not (0.005 <= self.resolution <= .2 and xmin < xmax and ymin < ymax):
            raise ValueError('Invalid ground extent or resolution')
        self.width = math.ceil((ymax-ymin)/self.resolution)
        self.height = math.ceil((xmax-xmin)/self.resolution)
        if not 2 <= min(self.width, self.height) <= max(self.width, self.height) <= 2048:
            raise ValueError('Ground image dimensions out of bounds')
        row, col = np.indices((self.height, self.width), dtype=float)
        self.x = xmax-(row+.5)*self.resolution
        self.y = ymax-(col+.5)*self.resolution
        self.world = np.stack((self.x, self.y, np.zeros_like(self.x)), axis=-1)
        self.maps = {}
        self.transforms = {}
        self.intrinsics = {}
        for name, camera in self.cameras.items():
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
            valid = ((depth > .05) & np.isfinite(mx) & np.isfinite(my) & (edge >= 1))
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
            mx[~valid], my[~valid] = -1, -1
            self.maps[name] = mx, my, weight
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
            raise ValueError('Three actual camera inputs required')
        total = np.zeros((self.height, self.width, 3), np.float32)
        weights = np.zeros((self.height, self.width), np.float32)
        counts = np.zeros((self.height, self.width), np.uint8)
        layers = {}
        for name, frame in upright_frames.items():
            expected = self.cameras[name]['perspective_size']
            if frame.dtype != np.uint8 or list(frame.shape[1::-1]) != expected or frame.shape[2] != 3:
                raise ValueError('Unexpected upright image: '+name)
            mx, my, weight = self.maps[name]
            projected = cv2.remap(frame, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            projected[weight <= 0] = 0
            layers[name] = projected
            total += projected.astype('float32')*weight[..., None]
            weights += weight
            counts += (weight > 0).astype('uint8')
        image = np.clip(total/np.maximum(weights[..., None], 1e-10), 0, 255).astype('uint8')
        return image, counts, layers
