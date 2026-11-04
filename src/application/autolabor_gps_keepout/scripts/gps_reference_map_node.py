#!/usr/bin/env python3
"""Display surveyed GPS tracks. This node has no navigation/control outputs."""
import json
import math
from pathlib import Path

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from autolabor_gps_keepout.geometry import gps_to_xy


def read_tracks(path):
    data = json.loads(Path(path).expanduser().read_text())
    if data.get('type') != 'FeatureCollection' or data.get('metadata',{}).get('coordinate_system') != 'EPSG:4326':
        raise ValueError('Reference map must contain WGS84 track features')
    tracks = data.get('features', [])
    count = 0
    for feature in tracks:
        geometry = feature.get('geometry',{})
        if geometry.get('type') != 'LineString':
            raise ValueError('Reference map supports open track lines only')
        points = geometry.get('coordinates',[])
        if len(points) < 2:
            raise ValueError('Track has fewer than two samples')
        for point in points:
            if (len(point) != 2 or not all(math.isfinite(float(v)) for v in point) or
                    not -180 <= point[0] <= 180 or not -85 <= point[1] <= 85):
                raise ValueError('Invalid longitude/latitude in track')
        count += len(points)
    if not tracks or count > 100000:
        raise ValueError('Reference map is empty or too large for this display')
    return data


def make_markers(data, origin, frame):
    result = MarkerArray()
    clear = Marker(); clear.action = Marker.DELETEALL
    result.markers.append(clear)
    groups = set()
    for feature in data['features']:
        props = feature.get('properties',{})
        marker = Marker()
        marker.header.frame_id = frame
        marker.ns = 'imu_gps_survey_reference'
        marker.id = len(result.markers)
        marker.action = Marker.ADD; marker.type = Marker.LINE_STRIP
        marker.pose.orientation.w = 1.; marker.scale.x = .15
        color = props.get('color','#2c9bdd').lstrip('#')
        if len(color) != 6:
            raise ValueError('Invalid reference track colour')
        marker.color.r,marker.color.g,marker.color.b = [int(color[i:i+2],16)/255. for i in (0,2,4)]
        marker.color.a = .9
        for lon,lat in feature['geometry']['coordinates']:
            x,y = gps_to_xy(lat,lon,origin)
            marker.points.append(Point(x=x,y=y,z=.09))
        result.markers.append(marker)
        group = props.get('group','')
        if group not in groups:
            groups.add(group)
            label = Marker()
            label.header.frame_id = frame; label.ns = 'imu_gps_survey_names'
            label.id = len(result.markers); label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD; label.pose.orientation.w = 1.
            label.pose.position = Point(x=marker.points[0].x,y=marker.points[0].y,z=1.5)
            label.scale.z = 1.2; label.color = marker.color
            # RViz fonts may lack CJK glyphs; descriptive ASCII group keys.
            label.text = group.replace('_',' ')+' [survey reference]'
            result.markers.append(label)
    return result


class ReferenceMap:
    def __init__(self, data):
        self.data = data
        self.frame = rospy.get_param('~frame_id','camera_init')
        self.origin = None
        self.publisher = rospy.Publisher('/gps/reference_map/markers',MarkerArray,queue_size=1,latch=True)
        self.status = rospy.Publisher('/gps/reference_map/status',String,queue_size=1,latch=True)
        self.status.publish(String(data='GPS 参考轨迹已读取，等待当前定位原点；不参与导航边界计算'))
        self.refresh(None)
        self.timer = rospy.Timer(rospy.Duration(1.),self.refresh)

    def refresh(self, _):
        lat = rospy.get_param('/gps/origin_lat',None)
        lon = rospy.get_param('/gps/origin_lon',None)
        if lat is None or lon is None:
            return
        origin = (float(lat),float(lon))
        if not all(math.isfinite(v) for v in origin) or origin == self.origin:
            return
        self.publisher.publish(make_markers(self.data,origin,self.frame))
        self.origin = origin
        self.status.publish(String(data='已显示 {} 段 WGS84 GPS 参考轨迹；原导航边界保持不变'.format(len(self.data['features']))))


def main():
    rospy.init_node('gps_reference_map')
    try:
        data = read_tracks(rospy.get_param('~map_file'))
    except (OSError,ValueError,KeyError,TypeError) as error:
        rospy.logwarn('GPS reference map unavailable: %s; navigation is unaffected',error)
        return
    ReferenceMap(data)
    rospy.spin()


if __name__ == '__main__':
    main()
