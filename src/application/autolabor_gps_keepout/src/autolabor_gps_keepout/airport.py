"""Import the supplied airport Shapefile ZIP without extracting its files.

GHWJ is the supplied planning boundary with green-area holes. Road/runway
centre lines are for display only; they never imply an invented road width.
All persisted geometry uses GeoJSON order (longitude, latitude), WGS84.
"""
import hashlib
import io
import math
from pathlib import Path
import zipfile

from shapely.geometry import shape, mapping
from shapely.ops import transform, unary_union

RADIUS = 6378137.0


def project(geometry, origin, inverse=False):
    latitude, longitude = origin
    sx = math.pi / 180 * RADIUS * math.cos(math.radians(latitude))
    sy = math.pi / 180 * RADIUS
    if inverse:
        convert = lambda x, y, z=None: (x / sx + longitude, y / sy + latitude)
    else:
        convert = lambda x, y, z=None: ((x - longitude) * sx, (y - latitude) * sy)
    return transform(convert, geometry)


def _coordinates(value):
    if not isinstance(value, (list, tuple)) or not value:
        raise ValueError('机场地图的坐标数组为空或格式不正确')
    if isinstance(value[0], (int, float)):
        if len(value) != 2 or not all(math.isfinite(float(v)) for v in value):
            raise ValueError('机场地图含无效坐标')
        lon, lat = value
        if not -180 <= lon <= 180 or not -85 <= lat <= 85:
            raise ValueError('机场地图必须使用 WGS84 经度、纬度')
        yield value
    else:
        for child in value:
            yield from _coordinates(child)


def validate_airport(value):
    if not isinstance(value, dict):
        raise ValueError('机场地图格式不正确')
    geo = value.get('geometry', {})
    if geo.get('type') not in ('Polygon', 'MultiPolygon'):
        raise ValueError('机场通行范围必须是闭合多边形')
    coordinates = list(_coordinates(geo.get('coordinates')))
    if len(coordinates) > 20000:
        raise ValueError('机场地图最多支持 20000 个边界顶点')
    rings = [geo['coordinates']] if geo['type'] == 'Polygon' else geo['coordinates']
    if any(len(r) < 4 or list(r[0]) != list(r[-1]) for polygon in rings for r in polygon):
        raise ValueError('机场地图边界及内部空洞必须闭合')
    geometry = shape(geo)
    if geometry.is_empty or not geometry.is_valid:
        raise ValueError('机场地图边界交叉或内部空洞无效')
    local = project(geometry, (coordinates[0][1], coordinates[0][0]))
    if local.area < 1 or max(local.bounds[2]-local.bounds[0], local.bounds[3]-local.bounds[1]) > 10000:
        raise ValueError('机场地图面积太小或跨度超过 10 公里')
    result = {'name': str(value.get('name', '机场地图'))[:120],
              'source_sha256': str(value.get('source_sha256', ''))[:64],
              'crs': 'EPSG:4326', 'geometry': mapping(geometry),
              'area_m2': local.area}
    context = value.get('context')
    if context:
        if context.get('type') != 'MultiLineString':
            raise ValueError('机场底图线条格式不正确')
        if len(list(_coordinates(context.get('coordinates')))) > 20000:
            raise ValueError('机场底图线条过多')
        lines = shape(context)
        if not lines.is_valid:
            raise ValueError('机场底图线条无效')
        result['context'] = mapping(lines)
    return result


def import_airport_zip(path):
    # Optional dependency: ordinary/manual fences still work without pyshp.
    try:
        import shapefile
    except ImportError:
        raise ValueError('导入机场地图需要安装 python3-pyshp')
    path = Path(path).expanduser()
    data = path.read_bytes()
    if len(data) > 64 * 1024 * 1024:
        raise ValueError('机场地图压缩包超过 64 MB')
    try:
        with zipfile.ZipFile(io.BytesIO(data)) as archive:
            if sum(info.file_size for info in archive.infolist()) > 64 * 1024 * 1024:
                raise ValueError('机场地图解压内容超过 64 MB')
            files = {}
            for name in archive.namelist():
                if name.lower().endswith('.shp'):
                    key = Path(name).stem.lower()
                    if key in files:
                        raise ValueError('机场地图有重名图层：' + key)
                    files[key] = name
            required = ['ghwj', 'lh', 'building', 'sbuilding', 'construction_']
            missing = [key for key in required if key not in files]
            if missing:
                raise ValueError('缺少机场范围/障碍物图层：' + ', '.join(missing))
            layers = {}
            for key in required + [k for k in ('runway', 'pavement', 'road_line') if k in files]:
                name = files[key]
                prj = archive.read(name[:-4] + '.prj').decode('utf-8-sig').upper()
                if 'PROJCS' in prj or 'GEOGCS' not in prj or 'GCS_WGS_1984' not in prj:
                    raise ValueError(key + '：坐标系不是支持的 WGS84 经纬度')
                reader = shapefile.Reader(shp=io.BytesIO(archive.read(name)),
                                          shx=io.BytesIO(archive.read(name[:-4] + '.shx')))
                geometries = [shape(s.__geo_interface__) for s in reader.shapes()]
                if not geometries or any(not g.is_valid or g.is_empty for g in geometries):
                    raise ValueError(key + '：图层含无效几何')
                if key in required and any(g.geom_type not in ('Polygon', 'MultiPolygon') for g in geometries):
                    raise ValueError(key + '：需要面图层，不能用中心线推测可行区域')
                layers[key] = unary_union(geometries)
            bounds = layers['ghwj'].bounds
            origin = ((bounds[1]+bounds[3])/2, (bounds[0]+bounds[2])/2)
            local = {key: project(g, origin) for key, g in layers.items()}
            allowed = local['ghwj'].difference(unary_union([local[k] for k in required[1:]]))
            if allowed.geom_type not in ('Polygon', 'MultiPolygon'):
                raise ValueError('机场范围扣除障碍物后没有有效面区域')
            context = []
            for key in ('runway', 'pavement', 'road_line'):
                if key not in layers:
                    continue
                geometry = layers[key]
                lines = [geometry] if geometry.geom_type == 'LineString' else geometry.geoms
                context.extend(list(line.coords) for line in lines if line.geom_type == 'LineString')
            result = {'name': path.stem, 'source_sha256': hashlib.sha256(data).hexdigest(),
                      'geometry': mapping(project(allowed, origin, inverse=True))}
            if context:
                result['context'] = {'type': 'MultiLineString', 'coordinates': context}
            return validate_airport(result)
    except (zipfile.BadZipFile, KeyError) as exc:
        raise ValueError('机场 ZIP 不完整：' + str(exc))
