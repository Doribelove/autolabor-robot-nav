"""Receive-only decoding of the field protocol. Never creates navigation commands."""
import json
import math


def position(item):
    if not isinstance(item, dict):
        raise ValueError('point must be an object')
    lat = item.get('lat', item.get('LAT'))
    lon = item.get('lon', item.get('LON', item.get('lng')))
    if isinstance(lat, bool) or isinstance(lon, bool):
        raise ValueError('coordinates cannot be booleans')
    lat, lon = float(lat), float(lon)
    if not math.isfinite(lat) or not math.isfinite(lon) or not (-90 <= lat <= 90 and -180 <= lon <= 180):
        raise ValueError('invalid WGS84 coordinates; do not swap LAT/LON silently')
    return {'latitude': lat, 'longitude': lon}


def invalid_constant(value):
    raise ValueError('non-finite JSON number: '+value)


def decode_incoming(body, vehicle_id):
    document = json.loads(body.decode('utf-8-sig'), parse_constant=invalid_constant)
    if isinstance(document, str):
        document = json.loads(document, parse_constant=invalid_constant)
    if not isinstance(document, (dict, list)):
        raise ValueError('message must be a JSON object or point groups')
    recipient = document.get('vehicleId') if isinstance(document, dict) else None
    result = {'kind': 'other_json', 'vehicle_id': recipient,
              'addressed_to_this_vehicle': None if recipient is None else str(recipient) == str(vehicle_id),
              'points': [], 'document': document, 'executed': False}
    if isinstance(document, dict):
        result['mission_id'] = document.get('missionId')
    if isinstance(document, list):
        result['kind'] = 'fod_point_groups'
        for group_index, group in enumerate(document):
            group = [group] if isinstance(group, dict) else group
            if not isinstance(group, list):
                raise ValueError('point group must be an array')
            result['points'].extend(dict(position(p), group=group_index) for p in group)
    elif 'TARGETS' in document:
        result['kind'] = 'fod_targets'
        if not isinstance(document['TARGETS'], list):
            raise ValueError('TARGETS must be an array')
        result['points'] = [position(p) for p in document['TARGETS']]
    elif 'waypoints' in document:
        result['kind'] = 'mission'
        if not isinstance(document['waypoints'], list):
            raise ValueError('waypoints must be an array')
        result['points'] = [position(p) for p in document['waypoints']]
        result['mission_id'] = document.get('missionId')
        result['mission_type'] = document.get('type')
    elif 'lines' in document:
        result['kind'] = 'route_information'
        for line in document['lines']:
            result['points'].extend(position(p) for p in line['points'])
    elif str(document.get('CMD', '')).lower() in ('1002', '0x1002'):
        result['kind'] = 'emergency_stop_notice'
    return result
