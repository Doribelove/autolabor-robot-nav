"""Decode organized/unorganized PointCloud2 using actual strides and endianness."""
import numpy as np


def pointcloud_xyzi(message):
    fields={f.name:f for f in message.fields}
    if any(name not in fields or fields[name].datatype!=7 or fields[name].count!=1
           for name in ('x','y','z','intensity')):
        raise ValueError('Expected float32 XYZ/intensity PointCloud2 fields')
    if (not 0<message.width*message.height<=300000 or message.point_step<16 or
            message.row_step<message.width*message.point_step or len(message.data)<message.row_step*message.height):
        raise ValueError('Invalid PointCloud2 layout')
    endian='>' if message.is_bigendian else '<'
    offsets=[fields[n].offset for n in ('x','y','z','intensity')]
    if any(o<0 or o+4>message.point_step for o in offsets):raise ValueError('Point field exceeds stride')
    dtype=np.dtype(dict(names=['x','y','z','intensity'],formats=[endian+'f4']*4,
                       offsets=offsets,itemsize=message.point_step))
    view=np.ndarray((message.height,message.width),dtype=dtype,buffer=message.data,
                    strides=(message.row_step,message.point_step))
    result=np.column_stack([view[n].ravel() for n in dtype.names]).astype(np.float32)
    return result[np.isfinite(result).all(axis=1)]


def model_points(xyzi,lidar_in_body=(-.011,-.02329,.04412),sensor_in_base=(.2,0,1),maximum_range=20):
    if xyzi.ndim!=2 or xyzi.shape[1]!=4:raise ValueError('Expected deskewed body-frame XYZI')
    sensor=np.asarray(sensor_in_base,float);extrinsic=np.asarray(lidar_in_body,float)
    if sensor.shape!=(3,) or extrinsic.shape!=(3,) or not np.isfinite(sensor).all() or not np.isfinite(extrinsic).all():
        raise ValueError('Invalid LiDAR installation')
    result=np.zeros((len(xyzi),5),np.float32)
    result[:,:3]=xyzi[:,:3]-extrinsic;result[:,3]=xyzi[:,3]
    base=result[:,:3]+sensor
    keep=(np.isfinite(result).all(axis=1)&(np.linalg.norm(result[:,:2],axis=1)>.5)&
          (np.linalg.norm(result[:,:2],axis=1)<maximum_range)&
          ~((np.abs(base[:,0])<=.75)&(np.abs(base[:,1])<=.50))&
          (result[:,2]>-5)&(result[:,2]<3)&(result[:,3]>=0)&(result[:,3]<=255))
    # A deskewed single sweep has t=0. Do not fabricate nine historical sweeps
    # or claim the model's velocity head is valid on this input distribution.
    return result[keep]


def supported_box(detection,points,minimum_points=8):
    center=np.asarray(detection['position'],float);dim=np.asarray(detection['dimensions'],float)
    if center.shape!=(3,) or dim.shape!=(3,) or not np.isfinite(center).all() or not np.isfinite(dim).all():return False
    if np.any(dim<=0) or np.any(dim>20):return False
    yaw=float(detection['yaw'])
    if not np.isfinite(yaw):return False
    delta=points[:,:3]-center
    c,s=np.cos(yaw),np.sin(yaw)
    x=delta[:,0]*c+delta[:,1]*s;y=-delta[:,0]*s+delta[:,1]*c
    inside=(np.abs(x)<=dim[0]/2+.1)&(np.abs(y)<=dim[1]/2+.1)&(np.abs(delta[:,2])<=dim[2]/2+.1)
    return int(np.count_nonzero(inside))>=minimum_points
