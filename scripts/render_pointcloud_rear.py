"""Low rear chase-camera layout for the offline recorded-cloud renderer."""
import collections
import json
import math
from pathlib import Path
import subprocess

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.ndimage import gaussian_filter1d
from scipy.spatial.transform import Rotation, Slerp


def render_rear(args, root, cache, cloud_times, offsets, cloud_points,
                map_rotations, map_translations, pose_time, base_positions,
                base_rotations, prior, quality_times, quality_states):
    cv2.setNumThreads(1)
    width, height, fps, duration = 1920, 1080, args.fps, 300
    bird = args.style == 'bird'
    playback = float(args.speed)
    source_step = playback / fps
    eye_offset = [-5.5,0.,7.5] if bird else [-3.,0.,1.10]
    target_offset = [1.,0.,.35] if bird else [6.,0.,.75]
    work = cache / ('bird_chase' if bird else 'rear_chase')
    work.mkdir(exist_ok=True)
    times = np.arange(int(round(duration / playback * fps))) * source_step
    query = np.clip(times, pose_time[0], pose_time[-1])
    positions = np.column_stack([np.interp(query, pose_time, base_positions[:, i]) for i in range(3)])
    rotations = Slerp(pose_time, base_rotations)(query)
    yaw = np.unwrap(rotations.as_euler('xyz')[:, 2])
    # Only the virtual camera is smoothed; recorded points and poses are unchanged.
    camera_yaw = gaussian_filter1d(yaw, max(.14/source_step,.5), mode='nearest')
    camera_positions = gaussian_filter1d(positions, max(.10/source_step,.5), axis=0, mode='nearest')
    wheel = np.genfromtxt(root / 'analysis/wheel_pose.csv', delimiter=',', names=True)
    speeds = np.interp(times, wheel['t_header'], wheel['vx_reported'])

    # Retain the actual original-map density in the view, sampled only for rendering.
    with (root / 'map_reference/map_3d/map.pcd').open('rb') as stream:
        while not stream.readline().startswith(b'DATA '):
            pass
        raw = np.frombuffer(stream.read(), dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('i','<f4'),('n','<u4')])
        dense = np.column_stack([raw[k] for k in ('x','y','z')])
    lo = base_positions.min(axis=0) - [12,12,1]
    hi = base_positions.max(axis=0) + [12,12,8]
    dense = dense[np.isfinite(dense).all(axis=1) & (dense >= lo).all(axis=1) & (dense <= hi).all(axis=1)]
    _, ids = np.unique(np.floor(dense / .08).astype(np.int32), axis=0, return_index=True)
    dense = dense[np.sort(ids)]
    print('Rear-camera display map:',len(dense),'points',flush=True)

    def basis(eye, target):
        front = target - eye
        front /= np.linalg.norm(front)
        right = np.cross(front,[0.,0.,1.]); right /= np.linalg.norm(right)
        return np.column_stack([right,np.cross(right,front),front])

    def project(points, eye, axes, focal, size):
        w,h = size
        local = (points-eye).dot(axes)
        z = local[:,2]
        safe = np.maximum(z,.01)
        uv = np.column_stack([w/2+focal*local[:,0]/safe,h/2-focal*local[:,1]/safe])
        valid = (z>.18) & (uv[:,0]>=1) & (uv[:,0]<w-3) & (uv[:,1]>=1) & (uv[:,1]<h-3)
        return uv,z,valid

    def points(image, depth, xyz, color, eye, axes, focal, big=False):
        h,w = image.shape[:2]
        uv,z,valid = project(xyz,eye,axes,focal,(w,h))
        if not valid.any(): return
        xy=uv[valid].astype(np.int32); z=z[valid]
        colors=np.broadcast_to(color,(len(xyz),3))[valid]
        pixels=xy[:,1]*w+xy[:,0]; order=np.lexsort((z,pixels)); ordered=pixels[order]
        order=order[np.r_[True,ordered[1:]!=ordered[:-1]]]
        xy,z,colors=xy[order],z[order],colors[order]
        samples=((0,0),(1,0),(0,1),(1,1)) if big else ((0,0),)
        for dx,dy in samples:
            x,y=xy[:,0]+dx,xy[:,1]+dy
            show=z<=depth[y,x]+.09
            x,y=x[show],y[show]
            image[y,x]=colors[show]
            depth[y,x]=np.minimum(depth[y,x],z[show])

    def line(image, xyz, color, eye, axes, focal, thickness=2):
        uv,z,valid=project(xyz,eye,axes,focal,(image.shape[1],image.shape[0]))
        # Split lines at the near plane; a close chase camera may be inside the cloud.
        for segment in np.split(np.arange(len(xyz)),np.where(np.diff(valid.astype(int))!=0)[0]+1):
            if len(segment)>1 and valid[segment[0]]:
                cv2.polylines(image,[uv[segment].astype(np.int32).reshape(-1,1,2)],False,color,thickness,cv2.LINE_AA)

    font_path='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
    font=lambda size:ImageFont.truetype(font_path,size)
    canvas=Image.new('RGB',(width,height),(10,15,23));draw=ImageDraw.Draw(canvas)
    draw.rectangle((0,0,width,4),fill=(44,211,190))
    draw.text((32,21),'MID360  /  三维点云行驶回放',font=font(33),fill=(232,240,248))
    subtitle = ('后上方鸟瞰跟随' if bird else '车尾近距离追踪') + '   ·   前 5 分钟实测数据   ·   %g 倍速' % playback
    draw.text((34,66),subtitle,font=font(20),fill=(141,164,184))
    draw.text((34,1008),'灰蓝：原地图  彩色：实测点云  青色：轨迹  车体为位置示意',font=font(20),fill=(153,174,194))
    draw.text((1450,1018),'地图  /  2026.08.29',font=font(20),fill=(141,164,184))
    template=np.asarray(canvas)[:,:,::-1].copy()
    status_badges={}
    for key,text in [('forward','前进'),('reverse','倒车'),('stop','停留')]:
        badge=Image.new('RGB',(132,40),(20,36,47));d=ImageDraw.Draw(badge)
        d.text((36,6),text,font=font(21),fill=(82,235,201))
        status_badges[key]=np.asarray(badge)[:,:,::-1].copy()

    hero_size=(1880,880); mini_size=(452,282)
    target=(lo+hi)/2;target[2]=np.median(base_positions[:,2])+1.2
    overview_eye=target+[38.,-46.,35.];overview_axes=basis(overview_eye,target)
    local=(prior-overview_eye).dot(overview_axes)
    span=np.max(np.abs(local[:,:2]/local[:,2,None]),axis=0)
    overview_f=float(min(mini_size[0]*.44/span[0],mini_size[1]*.44/span[1]))
    mini_bg=np.full((mini_size[1],mini_size[0],3),(25,23,19),np.uint8)
    mini_depth=np.full(mini_size[::-1],np.inf,np.float32)
    points(mini_bg,mini_depth,prior,np.array([101,112,124],np.uint8),overview_eye,overview_axes,overview_f)
    mini_title=Image.new('RGB',(452,34),(18,29,40));d=ImageDraw.Draw(mini_title)
    d.text((14,3),'全局轨迹 / OVERVIEW',font=font(19),fill=(180,201,219))
    mini_title=np.asarray(mini_title)[:,:,::-1].copy()

    params=json.loads((root/'parameters_start.json').read_text())
    footprint=params['move_base']['local_costmap']['footprint']
    if isinstance(footprint,str):footprint=json.loads(footprint)
    shape=np.array(footprint,float); xmin,ymin=shape.min(axis=0);xmax,ymax=shape.max(axis=0)
    body=np.array([[xmin,ymin,.14],[xmax,ymin,.14],[xmax,ymax,.14],[xmin,ymax,.14],
                   [xmin,ymin,.62],[xmax,ymin,.62],[xmax,ymax,.62],[xmin,ymax,.62]])
    faces=[([0,1,5,4],(115,72,32)),([1,2,6,5],(135,94,40)),([2,3,7,6],(149,91,35)),
           ([0,3,7,4],((40,160,230) if bird else (166,111,48))),([4,5,6,7],((75,198,247) if bird else (190,137,67)))]
    world_cache=collections.OrderedDict()
    last=-1
    live=colors=None
    encoder=None;log=None
    preview=[0,10,60,150,299]
    indices=[int(round(t/source_step)) for t in preview] if args.preview_only else range(int(round(args.start_second/source_step)),int(round(args.end_second/source_step)))
    if not args.preview_only:
        log=(work/(args.output.stem+'_encode.log')).open('w')
        cmd=['ffmpeg','-hide_banner','-loglevel','warning','-y','-f','rawvideo','-pixel_format','bgr24',
             '-video_size','1920x1080','-framerate',str(fps),'-i','-','-an','-c:v','libx264','-preset','veryfast',
             '-crf','21','-threads','2','-pix_fmt','yuv420p','-movflags','+faststart',str(args.output)]
        encoder=subprocess.Popen(cmd,stdin=subprocess.PIPE,stderr=log)
    try:
        for i in indices:
            t=times[i];idx=int(np.argmin(abs(cloud_times-t)))
            if idx!=last:
                selected=range(max(0,idx-7),idx+1)
                for k in selected:
                    if k not in world_cache:
                        p=cloud_points[offsets[k]:offsets[k+1]]
                        world_cache[k]=(map_rotations[k].apply(p)+map_translations[k]).astype(np.float32)
                for k in list(world_cache):
                    if k not in selected:del world_cache[k]
                batches=[world_cache[k] for k in selected];live=np.concatenate(batches)
                z=np.clip((live[:,2]+1.4)/5.,0,1)
                stops=np.array([0,.25,.5,.75,1.])
                palette=np.array([[231,154,73],[231,218,106],[180,230,170],[137,203,239],[117,160,230]])
                colors=np.column_stack([np.interp(z,stops,palette[:,j]) for j in range(3)])
                fade=np.concatenate([np.full(len(batch),.50+.50*(j+1)/len(batches)) for j,batch in enumerate(batches)])
                colors=(colors*fade[:,None]).astype(np.uint8);last=idx

            hero=np.empty((hero_size[1],hero_size[0],3),np.uint8);hero[:]=(39,35,29) if bird else (24,20,15)
            depth=np.full(hero_size[::-1],np.inf,np.float32)
            heading=Rotation.from_euler('z',camera_yaw[i])
            eye=camera_positions[i]+heading.apply(eye_offset)
            aim=camera_positions[i]+heading.apply(target_offset)
            axes=basis(eye,aim);focal=1080.
            nearby=dense[np.sum((dense[:,:2]-camera_positions[i,:2])**2,axis=1)<35**2]
            visible_live, visible_colors = live, colors
            if bird:
                # Display-only roof cutaway, like an open-top point-cloud map view.
                cut_height = positions[i,2]+2.1
                nearby=nearby[nearby[:,2] <= cut_height]
                cut_mask=live[:,2] <= cut_height
                visible_live,visible_colors=live[cut_mask],colors[cut_mask]
            gray_z=np.clip((nearby[:,2]+1.4)/5.,0,1)
            map_colors=np.column_stack([80+gray_z*25,74+gray_z*25,65+gray_z*27]).astype(np.uint8)
            if bird:
                # A fixed map-coordinate reference grid; it is not sensed terrain.
                floor_z=float(np.median(base_positions[:,2]))-.08
                cx,cy=camera_positions[i,:2]
                gx=np.arange(math.floor((cx-22)/2)*2,cx+24,2.)
                gy=np.arange(math.floor((cy-22)/2)*2,cy+24,2.)
                for x in gx:
                    xyz=np.column_stack([np.full(len(gy),x),gy,np.full(len(gy),floor_z)])
                    line(hero,xyz,(49,45,38),eye,axes,focal,1)
                for y in gy:
                    xyz=np.column_stack([gx,np.full(len(gx),y),np.full(len(gx),floor_z)])
                    line(hero,xyz,(49,45,38),eye,axes,focal,1)
            points(hero,depth,nearby,map_colors,eye,axes,focal,True)
            points(hero,depth,visible_live,visible_colors,eye,axes,focal,True)
            trajectory=positions[max(0,i-int(20/source_step)):i+1]+[0,0,.045]
            line(hero,trajectory,(205,224,64),eye,axes,focal,2)

            # A shaded vehicle glyph makes its rear and forward direction legible.
            vertices=rotations[i].apply(body)+positions[i]
            uv,z,valid=project(vertices,eye,axes,focal,hero_size)
            overlay=hero.copy()
            for face,color in sorted(faces,key=lambda item:float(z[item[0]].mean()),reverse=True):
                if valid[face].all():
                    polygon=uv[face].astype(np.int32)
                    cv2.fillConvexPoly(overlay,polygon,color,cv2.LINE_AA)
                    cv2.polylines(overlay,[polygon],True,(235,192,106),1,cv2.LINE_AA)
            hero=cv2.addWeighted(overlay,.88,hero,.12,0)
            for side in (-1,1):
                y=side*(ymax*.72)
                lamp=np.array([[xmin-.008,y-.055,.30],[xmin-.008,y+.055,.30],
                               [xmin-.008,y+.055,.43],[xmin-.008,y-.055,.43]])
                q=rotations[i].apply(lamp)+positions[i]
                a,_,show=project(q,eye,axes,focal,hero_size)
                if show.all():cv2.fillConvexPoly(hero,a.astype(np.int32),(91,110,250),cv2.LINE_AA)

            if bird:
                # Visual chassis glyph: dark wheels and top sensor, not a measured mesh.
                shapes=[]
                for ax in (xmin*.65,xmax*.65):
                    for side in (-1,1):
                        ay=side*(ymax+.015)
                        shapes.append((np.array([[ax-.15,ay-.065,.035],[ax+.15,ay-.065,.035],
                                                [ax+.15,ay+.065,.035],[ax-.15,ay+.065,.035],
                                                [ax-.15,ay-.065,.30],[ax+.15,ay-.065,.30],
                                                [ax+.15,ay+.065,.30],[ax-.15,ay+.065,.30]]), (36,40,45)))
                # Top plate and a compact lidar cap improve heading/vehicle readability.
                shapes.append((np.array([[-.27,-.20,.625],[.27,-.20,.625],[.27,.20,.625],[-.27,.20,.625],
                                         [-.27,-.20,.69],[.27,-.20,.69],[.27,.20,.69],[-.27,.20,.69]]),(57,64,71)))
                shapes.append((np.array([[-.07,-.07,.70],[.07,-.07,.70],[.07,.07,.70],[-.07,.07,.70],
                                         [-.07,-.07,.79],[.07,-.07,.79],[.07,.07,.79],[-.07,.07,.79]]),(215,225,230)))
                for verts,color in shapes:
                    xyz=rotations[i].apply(verts)+positions[i]
                    a,d,show=project(xyz,eye,axes,focal,hero_size)
                    boxfaces=([0,1,5,4],[1,2,6,5],[2,3,7,6],[0,3,7,4],[4,5,6,7])
                    for face in sorted(boxfaces,key=lambda f:float(d[f].mean()),reverse=True):
                        if show[face].all():cv2.fillConvexPoly(hero,a[face].astype(np.int32),color,cv2.LINE_AA)

            mini=mini_bg.copy();md=mini_depth.copy()
            points(mini,md,live,colors,overview_eye,overview_axes,overview_f)
            line(mini,positions[:i+1]+[0,0,.08],(221,224,65),overview_eye,overview_axes,overview_f,2)
            line(mini,vertices[[0,1,2,3,0]],(104,245,154),overview_eye,overview_axes,overview_f,2)
            frame=template.copy();frame[112:992,20:1900]=hero
            frame[136:170,1424:1876]=mini_title;frame[170:452,1424:1876]=mini
            cv2.rectangle(frame,(1423,135),(1876,452),(89,111,125),1,cv2.LINE_AA)
            phase='forward' if speeds[i]>.03 else ('reverse' if speeds[i]<-.03 else 'stop')
            frame[64:104,1170:1302]=status_badges[phase]
            cv2.putText(frame,'SRC %02d:%02d / 05:00'%(int(t//60),int(t%60)),(1390,54),cv2.FONT_HERSHEY_SIMPLEX,.83,(232,242,247),1,cv2.LINE_AA)
            cv2.putText(frame,'%gx  |  %d FPS  |  %.2f m/s'%(playback,fps,speeds[i]),(1428,91),cv2.FONT_HERSHEY_SIMPLEX,.56,(155,196,202),1,cv2.LINE_AA)
            cv2.rectangle(frame,(570,1037),(1270,1041),(50,57,66),-1)
            cv2.rectangle(frame,(570,1037),(570+int(700*t/300),1041),(164,225,83),-1)
            if args.preview_only or i in [int(round(v/source_step)) for v in (0,60,150,299)]:
                cv2.imwrite(str(work/('rear_%03ds.png'%int(t))),frame)
            if encoder is not None:encoder.stdin.write(frame.tobytes())
            if i%max(1,int(round(30/source_step)))==0:print('Rear render %.1f / 300 s'%t,flush=True)
    finally:
        if encoder is not None:
            encoder.stdin.close();code=encoder.wait();log.close()
            assert code==0,'ffmpeg rear-camera encoding failed'
    if not args.preview_only:
        metadata={'output':str(args.output),'source_bag':str(root/'localization_first_300s.bag'),
                  'width':width,'height':height,'fps':fps,'duration_seconds':(args.end_second-args.start_second)/playback, 'playback_speed':playback,'frames':len(indices),
                  'source_interval_seconds':[args.start_second,args.end_second],
                  'view':('elevated rear-follow camera, moving and turning with vehicle heading' if bird else
                          'camera directly behind the vehicle, moving and turning with vehicle heading'),
                  'camera_body_offset_m':eye_offset,'camera_target_body_offset_m':target_offset,
                  'view_style':args.style,'roof_cutaway_above_base_m':2.1 if bird else None,
                  'camera_smoothing_seconds':{'position_sigma':.10,'yaw_sigma':.14},
                  'reference_grid': 'display-only, not measured terrain' if bird else None,
                  'near_clip_m':.18,'display_prior_voxel_m':.08,'cloud_trail_seconds':.8,
                  'body_glyph':'saved footprint with illustrative height 0.62m, not a measured 3D vehicle model',
                  'raw_PCD_and_bag_modified':False,'offline':True}
        (work/(args.output.stem+'_manifest.json')).write_text(json.dumps(metadata,ensure_ascii=False,indent=2)+'\n')
        print('Rear-camera video complete:',args.output,flush=True)
