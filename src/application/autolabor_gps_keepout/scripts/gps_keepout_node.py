#!/usr/bin/env python3
"""Apply/save GPS exclusion regions; publishes geometry, never motion."""
import json
import math
import os
from pathlib import Path
import tempfile
import threading

import rospy
import yaml
from shapely.geometry import shape, mapping
from geometry_msgs.msg import Point, Point32, Polygon as PolygonMsg
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from autolabor_gps_keepout.msg import KeepoutPolygons
from autolabor_gps_keepout.srv import KeepoutCommand, KeepoutCommandResponse
from autolabor_gps_keepout.geometry import (KeepoutGeometry, validate_config, gps_to_xy,
                                           polygon_parts, rasterize)
from autolabor_gps_keepout.targets import parse_targets, validate_targets, decode_file
from autolabor_gps_keepout.airport import import_airport_zip, project


def atomic_write(path, content):
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=str(path.parent), delete=False) as f:
            temporary = f.name
            f.write(content if isinstance(content, bytes) else content.encode('utf-8'))
        os.replace(temporary, str(path))
    finally:
        if temporary and os.path.exists(temporary): os.unlink(temporary)


class KeepoutNode:
    def __init__(self):
        self.lock = threading.RLock()
        self.frame = rospy.get_param('~frame_id', 'camera_init')
        self.radius = math.hypot(rospy.get_param('~body_length', 1.6)/2,
                                 rospy.get_param('~body_width', 0.8)/2)
        self.store = Path(rospy.get_param('~storage_file', '~/.config/autolabor/gps_keepout.json')).expanduser()
        self.config = validate_config({})
        self.origin = None
        self.anchor = (0., 0.)
        self.geometry = KeepoutGeometry()
        self.revision = 0
        self.startup_error = ''
        self.targets = []
        self.selected_target = ''
        self.targets_store = Path(rospy.get_param('~targets_file', '~/.config/autolabor/agx_gps_map_targets.json')).expanduser()
        if self.targets_store.exists():
            try: self.targets=validate_targets(json.loads(self.targets_store.read_text()))
            except (ValueError,TypeError,KeyError,OSError) as exc: rospy.logwarn('FOD target file: %s',exc)
        if self.store.exists():
            try: self.config = validate_config(json.loads(self.store.read_text()))
            except (ValueError, TypeError, KeyError, OSError) as exc:
                self.startup_error = '保存的禁入区未加载：'+str(exc)
                rospy.logerr(self.startup_error)
        self.polygons_pub = rospy.Publisher('/gps/keepout/polygons', KeepoutPolygons, queue_size=1, latch=True)
        self.state_pub = rospy.Publisher('/gps/keepout/state', String, queue_size=1, latch=True)
        self.markers_pub = rospy.Publisher('/gps/keepout/markers', MarkerArray, queue_size=1, latch=True)
        self.targets_pub = rospy.Publisher('/gps/targets/markers', MarkerArray, queue_size=1, latch=True)
        self.targets_state_pub = rospy.Publisher('/gps/targets/state', String, queue_size=1, latch=True)
        self.selected_sub = rospy.Subscriber('/gps/targets/selection',String,self.select_target,queue_size=1)
        self.odom_sub = rospy.Subscriber('/gps/odom', Odometry, self.odom, queue_size=1)
        self.service = rospy.Service('/gps/keepout/command', KeepoutCommand, self.command)
        self.refresh_origin(None)
        self.publish()
        self.publish_targets()
        self.timer = rospy.Timer(rospy.Duration(1), self.refresh_origin)

    def odom(self, msg):
        if msg.header.frame_id == self.frame:
            with self.lock: self.anchor = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def refresh_origin(self, _):
        lat = rospy.get_param('/gps/origin_lat', None)
        lon = rospy.get_param('/gps/origin_lon', None)
        if lat is None or lon is None: return
        origin = (float(lat), float(lon))
        if not all(math.isfinite(v) for v in origin): return
        with self.lock:
            if origin != self.origin:
                self.origin = origin
                self.rebuild()
                self.publish()
            self.publish_targets()

    def select_target(self,msg):
        with self.lock:
            self.selected_target=msg.data
            self.publish_targets()

    def publish_targets(self):
        markers=MarkerArray(); clear=Marker();clear.action=Marker.DELETEALL;markers.markers.append(clear)
        if self.origin:
            for i,point in enumerate(self.targets):
                x,y=gps_to_xy(point['latitude'],point['longitude'],self.origin)
                selected=point['name']==self.selected_target
                marker=Marker();marker.header.frame_id=self.frame;marker.header.stamp=rospy.Time.now()
                marker.ns='fod_gps_points';marker.id=i;marker.type=Marker.SPHERE;marker.action=Marker.ADD
                marker.pose.position=Point(x=x,y=y,z=.4);marker.pose.orientation.w=1.
                marker.scale.x=marker.scale.y=marker.scale.z=1.4 if selected else .8
                marker.color.r=1. if selected else .1;marker.color.g=.85;marker.color.b=.1 if selected else 1.;marker.color.a=1.
                markers.markers.append(marker)
                label=Marker();label.header=marker.header;label.ns='fod_gps_names';label.id=i
                label.type=Marker.TEXT_VIEW_FACING;label.action=Marker.ADD;label.pose.orientation.w=1.
                label.pose.position=Point(x=x,y=y,z=1.5);label.scale.z=1.2
                label.color=marker.color
                label.text='{}  {:.1f} m'.format(point['name'],math.hypot(x-self.anchor[0],y-self.anchor[1]))
                markers.markers.append(label)
        self.targets_pub.publish(markers)
        self.targets_state_pub.publish(String(data=json.dumps({'points':self.targets,'origin':self.origin,'selected':self.selected_target},ensure_ascii=False)))

    def rebuild(self):
        self.geometry = KeepoutGeometry.from_config(self.config, self.origin, self.radius)
        self.revision += 1

    def state(self):
        local = [[gps_to_xy(*p, self.origin) for p in zone['points']]
                 for zone in self.config['polygons']] if self.origin else []
        message = ('已应用 {} 个禁入区，车体外沿留 {:.1f} 米'.format(
            len(local), self.config['clearance_m']) if self.config['enabled'] and self.origin
            else '等待 GPS 原点' if self.config['enabled'] else 'GPS 通行范围 / 禁入区未启用')
        if self.config.get('airport') and self.config['enabled'] and self.origin:
            message = '机场地图「{}」已启用，外部及内部障碍区禁止通行；车体外沿留 {:.1f} 米，附加 {} 个禁入区'.format(
                self.config['airport']['name'], self.config['clearance_m'], len(local))
        return {'revision': self.revision, 'frame_id': self.frame, 'origin': self.origin,
                'enabled': self.config['enabled'] and self.origin is not None,
                'clearance_m': self.config['clearance_m'], 'body_radius_m': self.radius,
                'local_polygons': local, 'config': self.config,
                'local_allowed': mapping(self.geometry.allowed) if self.geometry.allowed is not None else None,
                'message': self.startup_error or message}

    def publish(self):
        array = KeepoutPolygons()
        array.header.stamp = rospy.Time.now(); array.header.frame_id = self.frame
        markers = MarkerArray()
        clear = Marker(); clear.action = Marker.DELETEALL; markers.markers.append(clear)
        # Union contours include interior rings. The costmap uses even/odd fill.
        for part in polygon_parts(self.geometry.obstacles):
            for ring in [part.exterior]+list(part.interiors):
                poly = PolygonMsg()
                poly.points = [Point32(x=x,y=y,z=0) for x,y in list(ring.coords)[:-1]]
                array.polygons.append(poly)
                marker = Marker(); marker.header = array.header
                marker.ns = 'gps_keepout_margin'; marker.id = len(markers.markers)
                marker.type = Marker.LINE_STRIP; marker.action = Marker.ADD
                marker.pose.orientation.w = 1.; marker.scale.x = .16
                marker.color.r = 1.; marker.color.g = .24; marker.color.b = .15; marker.color.a = .95
                marker.points = [Point(x=x,y=y,z=.06) for x,y in ring.coords]
                markers.markers.append(marker)
        array.keep_in_enabled = self.geometry.allowed is not None
        if array.keep_in_enabled:
            for part in polygon_parts(self.geometry.allowed_costmap):
                for ring in [part.exterior]+list(part.interiors):
                    poly = PolygonMsg()
                    poly.points = [Point32(x=x,y=y,z=0) for x,y in list(ring.coords)[:-1]]
                    array.keep_in_polygons.append(poly)
        # Numbered original boundaries remain visible beside the clearance line.
        if self.origin:
            if self.config.get('airport'):
                airport = self.config['airport']
                boundary = project(shape(airport['geometry']), self.origin)
                for part in polygon_parts(boundary):
                    for ring in [part.exterior]+list(part.interiors):
                        marker = Marker(); marker.header = array.header
                        marker.ns = 'airport_boundary'; marker.id = len(markers.markers)
                        marker.type = Marker.LINE_STRIP; marker.action = Marker.ADD
                        marker.pose.orientation.w = 1.; marker.scale.x = .20
                        marker.color.g = .85; marker.color.b = .85; marker.color.a = .95
                        marker.points = [Point(x=x,y=y,z=.07) for x,y in ring.coords]
                        markers.markers.append(marker)
                if airport.get('context'):
                    lines = project(shape(airport['context']), self.origin)
                    marker = Marker(); marker.header = array.header; marker.ns = 'airport_roads'; marker.id = 0
                    marker.type = Marker.LINE_LIST; marker.action = Marker.ADD
                    marker.pose.orientation.w = 1.; marker.scale.x = .08
                    marker.color.r = .55; marker.color.g = .65; marker.color.b = .70; marker.color.a = .7
                    for line in lines.geoms:
                        points = list(line.coords)
                        for a, b in zip(points, points[1:]):
                            marker.points.extend([Point(x=a[0],y=a[1],z=.04),Point(x=b[0],y=b[1],z=.04)])
                    markers.markers.append(marker)
            for index, zone in enumerate(self.config['polygons']):
                points = [gps_to_xy(*p, self.origin) for p in zone['points']]
                marker = Marker(); marker.header = array.header; marker.ns = 'gps_keepout_vertices'
                marker.id = index; marker.type = Marker.LINE_STRIP; marker.action = Marker.ADD
                marker.pose.orientation.w = 1.; marker.scale.x = .10
                marker.color.r = 1.; marker.color.g = .75; marker.color.a = .9
                marker.points = [Point(x=x,y=y,z=.08) for x,y in points+[points[0]]]
                markers.markers.append(marker)
                label = Marker(); label.header=array.header; label.ns='gps_keepout_names'; label.id=index
                label.type=Marker.TEXT_VIEW_FACING; label.action=Marker.ADD; label.pose.orientation.w=1.
                label.pose.position=Point(x=points[0][0],y=points[0][1],z=.8)
                label.scale.z=.7; label.color.r=1.;label.color.g=.8;label.color.a=1.;label.text=zone['name']
                markers.markers.append(label)
        self.polygons_pub.publish(array)
        self.state_pub.publish(String(data=json.dumps(self.state(),ensure_ascii=False)))
        self.markers_pub.publish(markers)

    def command(self, request):
        try:
            payload = json.loads(request.payload or '{}')
            with self.lock:
                if request.command=='targets_get':
                    return KeepoutCommandResponse(True,'FOD 目标点已读取',json.dumps(self.targets,ensure_ascii=False))
                if request.command=='targets_parse':
                    text=decode_file(Path(payload['path']).expanduser().read_bytes()) if 'path' in payload else payload['text']
                    points=parse_targets(text)
                    return KeepoutCommandResponse(True,'已解析 {} 个 FOD 点'.format(len(points)),json.dumps(points,ensure_ascii=False))
                if request.command=='targets_set':
                    points=validate_targets(payload)
                    atomic_write(self.targets_store,json.dumps(points,ensure_ascii=False,indent=2))
                    self.targets=points;self.publish_targets()
                    return KeepoutCommandResponse(True,'已保存并显示 {} 个 FOD 目标点'.format(len(points)),json.dumps(points,ensure_ascii=False))
                if request.command == 'get':
                    return KeepoutCommandResponse(True, self.state()['message'], json.dumps(self.config,ensure_ascii=False))
                if request.command == 'airport_import':
                    candidate = validate_config(payload.get('config', self.config))
                    candidate['airport'] = import_airport_zip(payload['path'])
                    candidate['enabled'] = True
                    candidate = validate_config(candidate)
                    return KeepoutCommandResponse(True,
                        '机场地图已导入编辑区（{:.1f} 万平方米）；点击“应用到导航”后生效'.format(candidate['airport']['area_m2']/10000),
                        json.dumps(candidate, ensure_ascii=False))
                if request.command == 'apply':
                    candidate = validate_config(payload)
                    if candidate['enabled'] and (candidate['polygons'] or candidate.get('airport')) and self.origin is None:
                        raise ValueError('GPS 原点尚未就绪，可先保存配置，定位就绪后再应用')
                    geometry = KeepoutGeometry.from_config(candidate, self.origin, self.radius)
                    atomic_write(self.store,json.dumps(candidate,ensure_ascii=False,indent=2))
                    self.config=candidate;self.geometry=geometry;self.revision+=1;self.startup_error=''
                    self.publish()
                    return KeepoutCommandResponse(True,self.state()['message'],json.dumps(self.config,ensure_ascii=False))
                if request.command == 'load':
                    candidate=validate_config(json.loads(Path(payload['path']).expanduser().read_text()))
                    return KeepoutCommandResponse(True,'已载入编辑区；点击“应用到导航”后生效',json.dumps(candidate,ensure_ascii=False))
                if request.command == 'save':
                    candidate=validate_config(payload['config'])
                    path=Path(payload['path']).expanduser()
                    atomic_write(path,json.dumps(candidate,ensure_ascii=False,indent=2))
                    return KeepoutCommandResponse(True,'已保存禁入区：'+str(path),json.dumps(candidate,ensure_ascii=False))
                if request.command == 'export':
                    if self.origin is None: raise ValueError('导出栅格需要当前 GPS 原点')
                    candidate=validate_config(payload['config'])
                    geometry=KeepoutGeometry.from_config(candidate,self.origin,self.radius,for_export=True)
                    grid, origin, resolution=rasterize(geometry.obstacles,payload.get('resolution',.1),self.anchor,
                                                      allowed=geometry.allowed_costmap)
                    path=Path(payload['path']).expanduser().with_suffix('')
                    image=path.with_suffix('.pgm'); metadata=path.with_suffix('.yaml'); config=path.with_suffix('.keepout.json')
                    header='P5\n{} {}\n255\n'.format(grid.shape[1],grid.shape[0]).encode('ascii')
                    atomic_write(image,header+grid.tobytes())
                    atomic_write(config,json.dumps(candidate,ensure_ascii=False,indent=2))
                    data={'image':image.name,'resolution':resolution,'origin':[origin[0],origin[1],0.],
                          'negate':0,'occupied_thresh':.65,'free_thresh':.196,'gps_origin':list(self.origin),
                          'frame_id':self.frame,'keepout_config':config.name,
                          'description':'Virtual exclusion grid; free cells do not certify observed clear ground.'}
                    atomic_write(metadata,yaml.safe_dump(data,allow_unicode=True,sort_keys=False))
                    return KeepoutCommandResponse(True,'已导出 PGM + YAML + GPS 配置：'+str(path),json.dumps(candidate,ensure_ascii=False))
                raise ValueError('未知操作：'+request.command)
        except (ValueError, TypeError, KeyError, IndexError, OSError, AttributeError) as exc:
            return KeepoutCommandResponse(False,str(exc),'')


if __name__ == '__main__':
    rospy.init_node('gps_keepout')
    KeepoutNode()
    rospy.spin()
