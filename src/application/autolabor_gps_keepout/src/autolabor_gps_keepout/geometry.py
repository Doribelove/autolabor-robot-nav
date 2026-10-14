"""Coordinate conversion, clearance and routing without ROS side effects."""
import heapq
import math

import numpy as np
from shapely.geometry import Point, Polygon, LineString, shape
from shapely.ops import unary_union
from shapely.prepared import prep
from shapely.validation import explain_validity

EARTH_RADIUS = 6378137.0
BODY_RADIUS = math.hypot(0.8, 0.4)


def gps_to_xy(lat, lon, origin):
    return (EARTH_RADIUS * math.radians(lon-origin[1]) * math.cos(math.radians(origin[0])),
            EARTH_RADIUS * math.radians(lat-origin[0]))


def xy_to_gps(x, y, origin):
    return (origin[0]+math.degrees(y/EARTH_RADIUS),
            origin[1]+math.degrees(x/(EARTH_RADIUS*math.cos(math.radians(origin[0])))))


def finite(value):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError('坐标和距离必须是有限数值')
    return result


def validate_config(value):
    if not isinstance(value, dict) or value.get('version', 1) != 1:
        raise ValueError('不支持的禁入区文件格式')
    if type(value.get('enabled', False)) is not bool:
        raise ValueError('启用状态必须是布尔值')
    clearance = finite(value.get('clearance_m', 1.0))
    if not 0 <= clearance <= 20:
        raise ValueError('车体外沿距离须在 0～20 米之间')
    zones = value.get('polygons', [])
    if not isinstance(zones, list) or len(zones) > 16:
        raise ValueError('最多支持 16 个禁入区')
    result = {'version': 1, 'enabled': value.get('enabled', False),
              'clearance_m': clearance, 'polygons': []}
    total = 0
    for index, zone in enumerate(zones):
        if not isinstance(zone, dict):
            raise ValueError('每个禁入区必须包含名称和顶点列表')
        name = str(zone.get('name', '禁区 {}'.format(index+1)))[:80]
        points = [[finite(p[0]), finite(p[1])] for p in zone.get('points', [])]
        if points and points[0] == points[-1]:
            points.pop()
        if not 3 <= len(points) <= 64:
            raise ValueError('{}：请按边界顺序添加 3～64 个不同 GPS 点'.format(name))
        if any(not -85 <= p[0] <= 85 or not -180 <= p[1] <= 180 for p in points):
            raise ValueError('{}：纬度或经度超出范围'.format(name))
        if len({tuple(p) for p in points}) != len(points):
            raise ValueError('{}：存在重复顶点'.format(name))
        local = Polygon([gps_to_xy(*p, points[0]) for p in points])
        if not local.is_valid or local.area < 0.1:
            raise ValueError('{}：多边形交叉、共线或面积太小 ({})'.format(name, explain_validity(local)))
        if max(local.bounds[2]-local.bounds[0], local.bounds[3]-local.bounds[1]) > 10000:
            raise ValueError('{}：跨度超过 10 公里，请检查经纬度'.format(name))
        result['polygons'].append({'name': name, 'points': points})
        total += len(points)
    if total > 256:
        raise ValueError('所有禁入区的顶点总数不能超过 256')
    if value.get('airport') is not None:
        from .airport import validate_airport
        result['airport'] = validate_airport(value['airport'])
    return result


def polygon_parts(geometry):
    if geometry.is_empty:
        return []
    return [geometry] if geometry.geom_type == 'Polygon' else list(geometry.geoms)


def rings(geometry):
    return [list(p.exterior.coords)[:-1] for p in polygon_parts(geometry)]


class KeepoutGeometry:
    def __init__(self, polygons=(), clearance=1.0, body_radius=BODY_RADIUS, allowed=None):
        self.raw = unary_union([Polygon(p) for p in polygons])
        self.clearance = float(clearance)
        self.body_radius = float(body_radius)
        self.allowed = allowed
        self.active = not self.raw.is_empty or allowed is not None
        # Half a diagonal cell at the 0.1 m navigation resolution, rounded up.
        self.obstacles = self.raw.buffer(clearance+0.08, join_style=2) if self.active else self.raw
        self.forbidden = self.raw.buffer(clearance+body_radius+0.10, join_style=2) if self.active else self.raw
        self.prepared = prep(self.forbidden)
        self.allowed_costmap = allowed.buffer(-clearance-0.08, join_style=2) if allowed is not None else None
        self.free = (allowed.buffer(-clearance-body_radius-0.10, join_style=2)
                     .difference(self.forbidden)) if allowed is not None else None
        self.free_prepared = prep(self.free) if self.free is not None else None
        self._airport_graphs = {}
        self._last_path = None
        self.vertices = None
        self.edges = None

    @classmethod
    def from_state(cls, state):
        return cls(state.get('local_polygons', []) if state.get('enabled') else [],
                   state.get('clearance_m', 1.0), state.get('body_radius_m', BODY_RADIUS),
                   shape(state['local_allowed']) if state.get('enabled') and state.get('local_allowed') else None)

    @classmethod
    def from_config(cls, config, origin, body_radius=BODY_RADIUS, for_export=False):
        if origin is None or (not config['enabled'] and not for_export):
            return cls()
        polygons = [[gps_to_xy(*p, origin) for p in zone['points']] for zone in config['polygons']]
        allowed = None
        if config.get('airport'):
            from .airport import project
            allowed = project(shape(config['airport']['geometry']), origin)
        return cls(polygons, config['clearance_m'], body_radius, allowed)

    def clear_point(self, point):
        if self.free_prepared is not None:
            return self.free_prepared.covers(Point(point))
        return not self.active or not self.prepared.intersects(Point(point))

    def clear_line(self, a, b):
        if a == b:
            return self.clear_point(a)
        if self.free_prepared is not None:
            return self.free_prepared.covers(LineString([a, b]))
        return not self.active or not self.prepared.intersects(LineString([a, b]))

    def _airport_path(self, start, goal):
        # Most 10 Hz rolling-goal updates can reuse the remaining exact-checked
        # path. No grid discretization or straight shortcut through green holes.
        if self._last_path and self._last_path[-1] == goal:
            for index in range(len(self._last_path)-1, 0, -1):
                if self.clear_line(start, self._last_path[index]):
                    return [start]+self._last_path[index:]
        component = None
        for index, part in enumerate(polygon_parts(self.free)):
            if part.covers(Point(start)):
                if not part.covers(Point(goal)):
                    raise ValueError('起点与目标之间的通道不足以容纳车体和预留距离')
                component = (index, part)
                break
        if component is None:
            raise ValueError('起点不在机场可通行区域内')
        index, part = component
        if index not in self._airport_graphs:
            # Simplify candidate corners only, never the actual collision
            # boundary. Every graph edge is checked against full geometry.
            outline = part.buffer(-0.12, join_style=2).simplify(.10, preserve_topology=True)
            vertices = []
            for polygon in polygon_parts(outline):
                for ring in [polygon.exterior]+list(polygon.interiors):
                    vertices.extend(tuple(p) for p in list(ring.coords)[:-1] if self.clear_point(tuple(p)))
            self._airport_graphs[index] = (vertices, {})
        vertices, cached = self._airport_graphs[index]
        nodes = vertices+[start, goal]
        source, target = len(vertices), len(vertices)+1
        distances = {source: 0.}; parents = {}
        queue = [(math.dist(start, goal), 0., source)]
        while queue:
            _, distance, current = heapq.heappop(queue)
            if distance != distances.get(current):
                continue
            if current == target:
                path = [goal]
                while current != source:
                    current = parents[current]; path.append(nodes[current])
                self._last_path = path[::-1]
                return self._last_path
            if current == source:
                neighbours = [(j, math.dist(start, p)) for j, p in enumerate(vertices)
                              if self.clear_line(start, p)]
            else:
                if current not in cached:
                    cached[current] = [(j, math.dist(nodes[current], p)) for j, p in enumerate(vertices)
                                       if j != current and self.clear_line(nodes[current], p)]
                neighbours = list(cached[current])
            if self.clear_line(nodes[current], goal):
                neighbours.append((target, math.dist(nodes[current], goal)))
            for neighbour, length in neighbours:
                cost = distance+length
                if cost < distances.get(neighbour, math.inf):
                    distances[neighbour] = cost; parents[neighbour] = current
                    heapq.heappush(queue, (cost+math.dist(nodes[neighbour], goal), cost, neighbour))
        raise ValueError('机场通行范围内没有找到满足车体余量的连通路线')

    def _build_graph(self):
        if self.vertices is not None:
            return
        outlines = polygon_parts(self.forbidden.buffer(0.20, join_style=2))
        self.vertices = []
        for polygon in outlines:
            for ring in [polygon.exterior] + list(polygon.interiors):
                self.vertices.extend(tuple(p) for p in list(ring.coords)[:-1] if self.clear_point(p))
        self.edges = [[] for _ in self.vertices]
        for i, a in enumerate(self.vertices):
            for j in range(i):
                b = self.vertices[j]
                if self.clear_line(a, b):
                    d = math.hypot(a[0]-b[0], a[1]-b[1])
                    self.edges[i].append((j, d)); self.edges[j].append((i, d))

    def path(self, start, goal):
        start, goal = tuple(start), tuple(goal)
        if not self.clear_point(goal):
            raise ValueError('GPS 目标在通行范围外、禁入区或车体外沿 {:.1f} 米预留范围内'.format(self.clearance))
        if not self.clear_point(start):
            raise ValueError('车辆当前位置不满足通行范围及车体预留距离，请检查地图边界')
        if self.clear_line(start, goal):
            return [start, goal]
        if self.allowed is not None:
            return self._airport_path(start, goal)
        self._build_graph()
        nodes = self.vertices+[start, goal]
        edges = [list(e) for e in self.edges]+[[], []]
        source, target = len(nodes)-2, len(nodes)-1
        for i in [source, target]:
            for j in range(len(self.vertices)):
                if self.clear_line(nodes[i], nodes[j]):
                    d = math.hypot(nodes[i][0]-nodes[j][0], nodes[i][1]-nodes[j][1])
                    edges[i].append((j, d)); edges[j].append((i, d))
        distances = {source: 0.0}; parents = {}; queue = [(0.0, source)]
        while queue:
            cost, i = heapq.heappop(queue)
            if cost != distances[i]:
                continue
            if i == target:
                path = [nodes[i]]
                while i != source:
                    i = parents[i]; path.append(nodes[i])
                return path[::-1]
            for j, d in edges[i]:
                candidate = cost+d
                if candidate < distances.get(j, math.inf):
                    distances[j] = candidate; parents[j] = i
                    heapq.heappush(queue, (candidate, j))
        raise ValueError('禁入区之间没有可用绕行路线')

    def motion_clear(self, x, y, yaw, linear, angular, distance=None):
        if not self.active or abs(linear)+abs(angular) < 1e-8:
            return True
        # Used only by the visual controller, which does not run through TEB.
        travel = max(0.30, abs(linear)*0.3+linear*linear/(2*0.25)) if distance is None else distance
        duration = travel/max(abs(linear), 0.05)
        previous = (x, y)
        for t in np.linspace(0, duration, max(2, int(travel/0.05)+1)):
            if abs(angular) < 1e-6:
                point = (x+linear*t*math.cos(yaw), y+linear*t*math.sin(yaw))
            else:
                point = (x+linear/angular*(math.sin(yaw+angular*t)-math.sin(yaw)),
                         y-linear/angular*(math.cos(yaw+angular*t)-math.cos(yaw)))
            if not self.clear_point(point) or not self.clear_line(previous, point):
                return False
            previous = point
        return True


def rasterize(geometry, resolution=0.1, anchor=(0, 0), max_cells=4000000, allowed=None):
    resolution = finite(resolution)
    if not 0.05 <= resolution <= 2.0:
        raise ValueError('栅格分辨率须在 0.05～2 米之间')
    extent = allowed if allowed is not None else geometry
    if allowed is not None and allowed.is_empty:
        raise ValueError('预留距离后机场没有可用通行区域')
    bounds = extent.bounds if not extent.is_empty else (anchor[0], anchor[1], anchor[0], anchor[1])
    x0 = math.floor((min(bounds[0], anchor[0])-10)/resolution)*resolution
    y0 = math.floor((min(bounds[1], anchor[1])-10)/resolution)*resolution
    width = int(math.ceil((max(bounds[2], anchor[0])+10-x0)/resolution))
    height = int(math.ceil((max(bounds[3], anchor[1])+10-y0)/resolution))
    if width*height > max_cells:
        raise ValueError('地图范围较大，请增大导出分辨率（最多 400 万栅格）')
    grid = np.full((height, width), 0 if allowed is not None else 254, dtype=np.uint8)
    def paint(geometry, value):
        for j in range(height):
            y = y0+(j+0.5)*resolution
            cross = geometry.intersection(LineString([(x0,y),(x0+width*resolution,y)]))
            parts = list(cross.geoms) if hasattr(cross, 'geoms') else [cross]
            for part in parts:
                if part.is_empty: continue
                lo, _, hi, _ = part.bounds
                i0=max(0,int(math.ceil((lo-x0)/resolution-0.5)))
                i1=min(width,int(math.floor((hi-x0)/resolution-0.5))+1)
                grid[height-1-j, i0:i1] = value
    half_diagonal = resolution*math.sqrt(2)/2
    if allowed is not None:
        paint(allowed.buffer(-half_diagonal, join_style=2), 254)
    if not geometry.is_empty:
        paint(geometry.buffer(half_diagonal, join_style=2), 0)
    return grid, (x0, y0), resolution
