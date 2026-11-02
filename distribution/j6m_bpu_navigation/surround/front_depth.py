"""Paired ZED RGB/depth transport and metric front-view orthographic rendering."""
import hashlib
import ctypes as C
import json
from pathlib import Path
import struct
import time
import zlib

import cv2
import numpy as np


def encode_rgbd(image, depth_mm, source):
    ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 94])
    if not ok or depth_mm.shape != image.shape[:2] or depth_mm.dtype != np.uint16:
        raise ValueError('Invalid paired RGB/depth')
    jpeg = jpeg.tobytes()
    depth = zlib.compress(depth_mm.astype('<u2', copy=False).tobytes(), 1)
    meta = dict(protocol='conbine.zed.rgbd.v1', image=source, depth_units='millimetres',
                depth_measure='optical_axis_z', width=image.shape[1], height=image.shape[0],
                jpeg_sha256=hashlib.sha256(jpeg).hexdigest(), depth_sha256=hashlib.sha256(depth).hexdigest())
    header = json.dumps(meta, allow_nan=False).encode()
    return struct.pack('!4sIII', b'ZD01', len(header), len(jpeg), len(depth))+header+jpeg+depth


def decode_rgbd(data, now=None):
    if len(data) < 16: raise ValueError('Truncated RGB/depth')
    magic, nh, ni, nd = struct.unpack('!4sIII', data[:16])
    if magic != b'ZD01' or not 0 < nh <= 65536 or not 0 < ni <= 2*1024*1024 or not 0 < nd <= 4*1024*1024 or len(data) != 16+nh+ni+nd:
        raise ValueError('Invalid RGB/depth envelope')
    meta = json.loads(data[16:16+nh]); source = meta['image']
    age = (time.time() if now is None else now)-source['source_stamp_ns']/1e9
    if (meta['protocol'] != 'conbine.zed.rgbd.v1' or meta['depth_units'] != 'millimetres'
            or meta['depth_measure'] != 'optical_axis_z' or not -.1 <= age <= 1.5
            or (meta['width'],meta['height']) != (1280,720)):
        raise ValueError('Stale or unsupported RGB/depth')
    jpeg, compressed = data[16+nh:16+nh+ni], data[16+nh+ni:]
    if hashlib.sha256(jpeg).hexdigest() != meta['jpeg_sha256'] or hashlib.sha256(compressed).hexdigest() != meta['depth_sha256']:
        raise ValueError('RGB/depth digest mismatch')
    image = cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
    count = meta['width']*meta['height']*2
    decoder = zlib.decompressobj(); raw = decoder.decompress(compressed,count+1)
    if len(raw) != count or not decoder.eof or decoder.unused_data or image is None or image.shape[:2] != (720,1280):
        raise ValueError('Invalid RGB/depth dimensions')
    depth = np.frombuffer(raw,'<u2').reshape(720,1280)
    k = np.asarray(source['K_rectified'],float)
    if k.shape != (3,3) or not np.isfinite(k).all() or min(k[0,0],k[1,1]) <= 0:
        raise ValueError('Invalid depth intrinsics')
    return image, depth, source


class FrontDepthRenderer:
    """Orthographically rasterize all valid RGB/depth points at measured X/Y.

    Unknown depth/occluded ground stays empty. Never fill a bin with stretched
    pixels or invent the ground hidden behind it. Other camera regions remain.
    """
    def __init__(self, bev, k=None, use_native=True):
        self.bev = bev
        self.region = bev.source_owner == bev.source_names.index('front')
        self.transform = bev.transforms['front']
        self.k = np.asarray(bev.intrinsics['front'] if k is None else k,dtype=float)
        height,width = bev.cameras['front']['perspective_size'][::-1]
        v,u = np.indices((height,width),dtype=np.float32)
        self.rays = np.stack(((u-self.k[0,2])/self.k[0,0],
                              (v-self.k[1,2])/self.k[1,1],np.ones_like(u)),axis=-1)
        self.ground_rays = self.rays @ self.transform[:3,:3].astype(np.float32).T
        self.native = None
        library = Path(__file__).resolve().parents[3]/'runtime/bev_depth/libfront_depth_raster.so'
        if use_native and library.exists():
            self.native = C.CDLL(str(library)).fd_raster
            self.native.argtypes = [C.c_void_p,C.c_void_p,C.c_void_p,C.c_int,C.c_void_p,
                                    C.c_void_p,C.c_int,C.c_int,C.c_float,C.c_float,C.c_float,C.c_void_p,C.c_void_p]
            self.native.restype = None
        self.ground_rays = np.ascontiguousarray(self.ground_rays,dtype=np.float32)
        self.position = np.ascontiguousarray(self.transform[:3,3],dtype=np.float32)
        self.region_u8 = self.region.astype(np.uint8)

    def compose(self, image, depth_mm, fused, layers):
        if depth_mm.shape != image.shape[:2] or image.shape[:2] != self.rays.shape[:2]:
            raise ValueError('Depth renderer dimensions mismatch')
        valid = (depth_mm >= 300) & (depth_mm <= 10000)
        result = fused.copy()
        region = self.region
        front = np.zeros_like(fused)
        result[region] = 0
        # Reject remote below-ground/ceiling returns, without fitting or silently
        # changing the user-supplied mounting height/pitch to this indoor scene.
        # Top-down z buffer: select the highest observed surface in each cell.
        highest = np.full(self.bev.height*self.bev.width,-np.inf,np.float32)
        if self.native:
            image = np.ascontiguousarray(image,np.uint8)
            depth_mm = np.ascontiguousarray(depth_mm,np.uint16)
            self.native(image.ctypes.data,depth_mm.ctypes.data,self.ground_rays.ctypes.data,depth_mm.size,
                self.position.ctypes.data,self.region_u8.ctypes.data,self.bev.width,self.bev.height,
                self.bev.config['x_range_m'][1],self.bev.config['y_range_m'][1],self.bev.resolution,
                front.ctypes.data,highest.ctypes.data)
        else:
            xyz = self.ground_rays*(depth_mm.astype(np.float32)*.001)[...,None]+self.position
            usable = valid & (xyz[...,2] > -.2) & (xyz[...,2] < 2.)
            points,colors = xyz[usable],image[usable]
            row = np.floor((self.bev.config['x_range_m'][1]-points[:,0])/self.bev.resolution).astype(np.int32)
            col = np.floor((self.bev.config['y_range_m'][1]-points[:,1])/self.bev.resolution).astype(np.int32)
            inside = (row>=0)&(row<self.bev.height)&(col>=0)&(col<self.bev.width)
            row,col,points,colors = row[inside],col[inside],points[inside],colors[inside]
            inside = region[row,col]
            row,col,points,colors = row[inside],col[inside],points[inside],colors[inside]
            cell = row*self.bev.width+col
            np.maximum.at(highest,cell,points[:,2])
            hits = np.flatnonzero(points[:,2] == highest[cell])
            indices = np.full(len(highest),-1,np.int32)
            np.maximum.at(indices,cell[hits],hits)
            chosen = indices[indices>=0]
            front[row[chosen],col[chosen]] = colors[chosen]
        seen = np.isfinite(highest).reshape(region.shape)
        observed_count = int(seen.sum())
        # Render observed samples as 3x3 point sprites (at most 1 cm per axis).
        # Never extend across a larger hole or reconstruct an occluded surface.
        index_image = np.where(seen,np.arange(len(highest)).reshape(region.shape)+1,0).astype(np.float32)
        neighbor = cv2.dilate(index_image,np.ones((3,3),np.uint8)).astype(np.int32)-1
        fill = region & ~seen & (neighbor>=0)
        front[fill] = front.reshape(-1,3)[neighbor[fill]]
        result[region] = front[region]
        stats = dict(mode='zed_metric_depth_orthographic', depth_valid_fraction=float(valid.mean()),
            render_backend='native_cpu' if self.native else 'numpy_cpu',
            rendered_cells=observed_count, rendered_front_fraction=observed_count/max(1,int(region.sum())),
            point_splat_radius_m=self.bev.resolution, interpolated_cells=int(fill.sum()),
            height_range_m=[-.2,2.], unknown_front_cells=int(region.sum())-observed_count-int(fill.sum()),
            hidden_ground_reconstructed=False, metric_accuracy_verified=False)
        return result,front,stats
