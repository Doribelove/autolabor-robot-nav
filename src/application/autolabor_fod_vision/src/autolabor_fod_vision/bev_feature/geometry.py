"""CPU geometry only. Sampling/fusion of learned tensors belongs to the model."""
import numpy as np

ORDER = ('front', 'left', 'right', 'rear')
DEFAULT = dict(order=list(ORDER), input_hw=[256,448], stride=16,
               x_range=[-2.,4.], y_range=[-3.,3.], resolution=.05,
               z_samples=[0.,.04,.08,.12], depth_start=.2, depth_step=.2, depth_bins=40)

def letterbox_k(k, size, hw):
    h,w=hw; sw,sh=size; scale=min(w/sw,h/sh)
    nw,nh=round(sw*scale),round(sh*scale); left,top=(w-nw)//2,(h-nh)//2
    # cv2.resize pixel-centre convention, including asymmetric integer padding.
    a=np.array([[nw/sw,0,left+(nw/sw-1)/2],[0,nh/sh,top+(nh/sh-1)/2],[0,0,1.]])
    return a@np.asarray(k), (nw,nh,left,top)

def rotation(yaw,pitch,roll=0):
    a,b,c=np.deg2rad([yaw,pitch,roll]);cy,sy,cp,sp,cr,sr=np.cos(a),np.sin(a),np.cos(b),np.sin(b),np.cos(c),np.sin(c)
    return np.array([[cy,-sy,0],[sy,cy,0],[0,0,1.]])@np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])@np.array([[0,0,1],[-1,0,0],[0,-1,0.]])@np.array([[cr,-sr,0],[sr,cr,0],[0,0,1.]])

def sampling(rig, spec=DEFAULT, rig_to_base=None):
    """Explicit BEV convention: row increases +X, column increases +Y.
    GridSample uses align_corners=False. No implicit zero rig/base alignment.
    Depth is camera optical Z, not Euclidean range. Four z hypotheses per cell
    are a ground-trash prototype, not HEAL's frustum distance-topk selection.
    """
    res=spec['resolution']; x0,x1=spec['x_range'];y0,y1=spec['y_range']
    h,w=round((x1-x0)/res),round((y1-y0)/res);p=len(spec['z_samples'])
    x,y=np.meshgrid(x0+(np.arange(h)+.5)*res,y0+(np.arange(w)+.5)*res,indexing='ij')
    world=np.stack([np.broadcast_to(x,(p,h,w)),np.broadcast_to(y,(p,h,w)),np.broadcast_to(np.asarray(spec['z_samples'])[:,None,None],(p,h,w))],axis=-1)
    if rig_to_base is not None:
        dx,dy,theta=rig_to_base;c,s=np.cos(theta),np.sin(theta)
        world[...,:2]=(world[...,:2]-[dx,dy])@np.array([[c,-s],[s,c]])
    fh,fw=[n//spec['stride'] for n in spec['input_hw']];D=spec['depth_bins'];grids=[];dgrids=[];masks=[];Ks=[]
    for name in ORDER:
        cam=rig['cameras'][name]; k=np.array(cam['K_perspective'],dtype=float)
        if cam['rotate_180']:k[0,2]=cam['perspective_size'][0]-1-k[0,2];k[1,2]=cam['perspective_size'][1]-1-k[1,2]
        k,box=letterbox_k(k,cam['perspective_size'],spec['input_hw']);Ks.append(k)
        q=(world-cam['position_ground_rig_m'])@rotation(cam['yaw_left_deg'],cam['pitch_down_deg'],cam.get('upright_optical_roll_deg',0))
        depth=q[...,2]; uv=(q@k.T)[...,:2]/np.maximum(depth[...,None],1e-8)
        # Feature coordinate implied by half-pixel resize to stride-16 maps.
        uf=(uv[...,0]+.5)/spec['stride']-.5;vf=(uv[...,1]+.5)/spec['stride']-.5
        di=np.floor((depth-spec['depth_start'])/spec['depth_step']).astype(int)
        nw,nh,l,t=box
        valid=(depth>0)&(di>=0)&(di<D)&(uv[...,0]>=l)&(uv[...,0]<l+nw)&(uv[...,1]>=t)&(uv[...,1]<t+nh)&(uf>=0)&(uf<fw-1)&(vf>=0)&(vf<fh-1)
        g=np.stack([2*(uf+.5)/fw-1,2*(vf+.5)/fh-1],-1)
        dg=np.stack([g[...,0],2*(np.clip(di,0,D-1)*fh+vf+.5)/(D*fh)-1],-1)
        g[~valid]=2;dg[~valid]=2
        grids.append(g.reshape(p*h,w,2));dgrids.append(dg.reshape(p*h,w,2));masks.append(valid.reshape(1,p*h,w))
    return dict(feature_grid=np.asarray(grids,np.float32),depth_grid=np.asarray(dgrids,np.float32),valid=np.asarray(masks,np.float32),K=np.asarray(Ks),shape=np.array([p,h,w]))

def decode(heatmap, offsets, spec=DEFAULT, threshold=.35, radius=.12, limit=100):
    """Global metric NMS over the fused grid, never per-camera detection merging."""
    import cv2
    heat=np.asarray(heatmap,dtype=np.float32).squeeze();off=np.asarray(offsets).reshape(2,*heat.shape)
    if not np.isfinite(heat).all() or not np.isfinite(off).all():raise ValueError('nonfinite model output')
    peaks=(heat>=cv2.dilate(heat,np.ones((3,3),np.uint8)))&(heat>=threshold)
    rows,cols=np.where(peaks);indices=np.argsort(-heat[rows,cols],kind='stable');out=[]
    for i in indices:
        r,c=rows[i],cols[i];ox,oy=off[:,r,c]
        if not (0<=ox<1 and 0<=oy<1):continue
        x=spec['x_range'][0]+(r+ox)*spec['resolution'];y=spec['y_range'][0]+(c+oy)*spec['resolution']
        if any((x-a['x'])**2+(y-a['y'])**2<radius**2 for a in out):continue
        out.append(dict(x=float(x),y=float(y),confidence=float(heat[r,c])))
        if len(out)>=limit:break
    return out
