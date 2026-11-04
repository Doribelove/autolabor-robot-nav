#!/usr/bin/env python3
"""Extract the supplied, already-unpacked IMU survey as WGS84 track geometry.

No ROS connections, inferred polygons, timestamp conversions or map activation.
Full samples remain in CSV; line simplification only affects the display layer.
"""
import argparse
import csv
import hashlib
import json
import math
from collections import Counter
from pathlib import Path

import numpy as np
from shapely.geometry import LineString


EARTH_RADIUS = 6378137.0  # Same local tangent approximation as gps_module.
GROUPS = {
    'taxiway_1': ('滑行道 · 第一次采集', '#1479b8'),
    'taxiway_2': ('滑行道 · 第二次采集', '#dd7230'),
    'taxiway_supplement': ('滑行道 · 补充', '#99710c'),
    'stand_505': ('505 机位', '#168568'),
    'stand_506': ('506 机位', '#884ab2'),
    'stand_507': ('507 机位', '#c24170'),
    'stands_combined': ('三个机位合并采集', '#437c84'),
}


def group_for(path):
    text = str(path)
    if '3个机位合并' in text:
        return 'stands_combined'
    for number in (505, 506, 507):
        if str(number)+'机位' in text:
            return 'stand_'+str(number)
    if '补充' in text:
        return 'taxiway_supplement'
    return 'taxiway_1' if '第一次采集' in text else 'taxiway_2'


def samples(path):
    # Source uses CR CR LF; split on LF only, retaining physical source rows.
    with path.open(encoding='utf-8-sig', newline='\n') as stream:
        columns = stream.readline().split()
        needed = {'dLatitude', 'dLongitude', 'ullTimestamp', 'GPS_Status'}
        if not needed.issubset(columns):
            raise ValueError('Missing coordinate fields: '+str(path))
        for line_number, line in enumerate(stream, 2):
            values = line.split()
            if not values:
                continue
            if len(values) != len(columns):
                raise ValueError(f'{path}:{line_number}: incomplete record')
            yield line_number, dict(zip(columns, values))


def split_tracks(points, origin, gap_m=5.0):
    """Never connect across missing coordinates or large sample discontinuities."""
    scale = np.array([math.pi/180*EARTH_RADIUS*math.cos(math.radians(origin[1])),
                      math.pi/180*EARTH_RADIUS])
    segments, current = [], []
    for point in points:
        if point is None:
            if len(current) >= 2:
                segments.append(current)
            current = []
            continue
        if current and np.linalg.norm((np.array(point)-current[-1])*scale) > gap_m:
            if len(current) >= 2:
                segments.append(current)
            current = []
        current.append(point)
    if len(current) >= 2:
        segments.append(current)
    return segments, scale


def extract(dataset, output):
    output.mkdir(parents=True, exist_ok=False)
    manifest = json.loads((dataset/'source_manifest.json').read_text())
    seen, files, duplicates, empty = {}, [], [], []
    # Prefer explicitly labelled taxiway copies, but retain all raw aliases.
    entries = sorted(manifest['extracted_files'],
                     key=lambda x: ('滑行道地图' not in x['path'], x['path']))
    features, total, invalid, origin, all_coords = [], 0, 0, None, []
    csv_path = output/'gps_samples.csv'
    fields = ['source_id','source_path','source_line','coordinate_valid',
              'dLongitude','dLatitude','fAltitude','fHeadingAngle',
              'GPS_Status','cSystemStatus','Time','ullTimestamp']
    with csv_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for entry in entries:
            path = dataset/entry['path']
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            if digest != entry['sha256']:
                raise ValueError('Source hash changed: '+str(path))
            if digest in seen:
                duplicates.append({'path':entry['path'],'same_as':seen[digest]})
                continue
            seen[digest] = entry['path']
            source_id = 'track_'+digest[:10]
            points, statuses, rows, bad = [], Counter(), 0, 0
            first_stamp = last_stamp = None
            for line_number, record in samples(path):
                rows += 1
                lon, lat = float(record['dLongitude']), float(record['dLatitude'])
                valid = (math.isfinite(lon) and math.isfinite(lat) and
                         -180 <= lon <= 180 and -85 <= lat <= 85 and (lon,lat)!=(0.,0.))
                item = {key:record.get(key,'') for key in fields}
                item.update(source_id=source_id,source_path=entry['path'],source_line=line_number,
                            coordinate_valid=int(valid))
                writer.writerow(item)
                statuses[record['GPS_Status']] += 1
                if first_stamp is None:
                    first_stamp = record['ullTimestamp']
                last_stamp = record['ullTimestamp']
                if not valid:
                    bad += 1; points.append(None); continue
                if origin is None:
                    origin = [lon, lat]
                points.append([lon,lat]); all_coords.append([lon,lat])
            if not rows:
                empty.append(entry['path']); continue
            group = group_for(path)
            info = {'source_id':source_id,'path':entry['path'],'sha256':digest,
                    'group':group,'group_label':GROUPS[group][0],'records':rows,
                    'invalid_coordinates':bad,'gps_status_counts':dict(statuses),
                    'timestamp_raw_first_last':[first_stamp,last_stamp]}
            files.append(info); total += rows; invalid += bad
            segments, scale = split_tracks(points, origin)
            for index, segment in enumerate(segments):
                xy = (np.array(segment)-origin)*scale
                line = LineString(xy).simplify(.05, preserve_topology=False)
                lonlat = np.array(line.coords)/scale+origin
                if line.length == 0:
                    continue
                features.append({'type':'Feature','properties':{
                    **info,'name':GROUPS[group][0]+' / '+path.stem,
                    'segment':index+1,'color':GROUPS[group][1],
                    'role':'survey_track_reference','closed_boundary_confirmed':False,
                    'original_segment_points':len(segment),
                    'display_points':len(lonlat),'simplification_tolerance_m':.05},
                    'geometry':{'type':'LineString','coordinates':lonlat.tolist()}})
    if not all_coords:
        raise ValueError('No geographic coordinates in source')
    data = np.array(all_coords)
    bbox = [*data.min(0), *data.max(0)]
    metadata = {'name':'IMU data GPS 采集轨迹','coordinate_system':'EPSG:4326',
                'coordinate_source':'User confirmed WGS84; original lon/lat retained without datum shift',
                'coordinate_order':['longitude','latitude'],'units':'degrees',
                'role':'reference_only','boundary_semantics':'pending: serpentine survey tracks, no inferred polygon',
                'gps_status_semantics':'raw numeric codes; vendor definitions not supplied',
                'timestamp_semantics':'raw strings preserved; no conversion to ROS/Unix time',
                'archive_sha256':manifest['archive_sha256'],'origin_lonlat':origin,
                'raw_text_files':len(entries),'unique_nonempty_files':len(files),
                'unique_samples':total,'invalid_coordinates':invalid,
                'duplicate_files':duplicates,'empty_files':empty,'files':files,
                'bbox_lonlat':bbox,'segment_count':len(features),
                'display_points':sum(f['properties']['display_points'] for f in features)}
    geo = {'type':'FeatureCollection','name':metadata['name'],'bbox':bbox,
           'metadata':metadata,'features':features}
    (output/'gps_tracks.geojson').write_text(json.dumps(geo,ensure_ascii=False,indent=2)+'\n')
    (output/'extraction_summary.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2)+'\n')
    return geo


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True,
                        help='A new directory; existing outputs are never overwritten')
    args = parser.parse_args()
    geo = extract(args.dataset,args.output)
    print(json.dumps({k:geo['metadata'][k] for k in ('unique_nonempty_files','unique_samples',
          'segment_count','display_points','invalid_coordinates')},ensure_ascii=False))


if __name__ == '__main__':
    main()
