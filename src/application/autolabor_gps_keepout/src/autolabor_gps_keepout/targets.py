"""Import FOD goal points: name, longitude, latitude (not exclusion vertices)."""
import csv
import io
import math
import re


def coordinate(value):
    parts = re.split(r'[:°′\'″"\s]+', str(value).strip())
    parts = [p for p in parts if p]
    if len(parts) == 1:
        return float(parts[0])
    if len(parts) in (2,3):
        d,m = float(parts[0]),float(parts[1]); s = float(parts[2]) if len(parts)>2 else 0.
        if not 0<=m<60 or not 0<=s<60: raise ValueError('度分秒格式无效')
        return (-1 if d<0 else 1)*(abs(d)+m/60+s/3600)
    raise ValueError('经纬度格式无效')


def validate_targets(points):
    if not isinstance(points,list) or len(points)>500:
        raise ValueError('最多导入 500 个 FOD 目标点')
    result=[]; names=set()
    for point in points:
        name=str(point['name']).strip()
        lon,lat=coordinate(point['longitude']),coordinate(point['latitude'])
        if not name or len(name)>80 or name in names: raise ValueError('点名不能为空、重复或超过 80 字')
        if not math.isfinite(lat) or not math.isfinite(lon) or not -85<=lat<=85 or not -180<=lon<=180:
            raise ValueError(name+'：经纬度范围无效；输入顺序为 点名 经度 纬度')
        names.add(name); result.append({'name':name,'longitude':lon,'latitude':lat})
    return result


def parse_targets(text):
    lines=[line.strip() for line in text.lstrip('\ufeff').splitlines() if line.strip()]
    if not lines: raise ValueError('没有读到 FOD 点')
    if ',' in lines[0] and '点名' in lines[0]:
        rows=csv.DictReader(io.StringIO('\n'.join(lines)))
        return validate_targets([{'name':r['点名'],'longitude':r['L'],'latitude':r['B']} for r in rows])
    points=[]
    for number,line in enumerate(lines,1):
        fields=re.split(r'[\t,;，\s]+',line)
        if number==1 and fields[0].lower() in ('name','点名'): continue
        if len(fields)!=3: raise ValueError('第 {} 行应为：点名 经度 纬度'.format(number))
        points.append({'name':fields[0],'longitude':fields[1],'latitude':fields[2]})
    return validate_targets(points)


def decode_file(content):
    for encoding in ('utf-8-sig','gb18030','utf-16'):
        try: return content.decode(encoding)
        except UnicodeError: pass
    raise ValueError('无法识别文件编码，请使用 UTF-8 或 GB18030')
