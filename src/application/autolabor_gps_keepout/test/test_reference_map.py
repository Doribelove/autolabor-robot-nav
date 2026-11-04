#!/usr/bin/env python3
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

HERE = Path(__file__).resolve()


def module(name, path):
    spec = importlib.util.spec_from_file_location(name,path)
    result = importlib.util.module_from_spec(spec); spec.loader.exec_module(result)
    return result


display = module('gps_reference_display',HERE.parents[1]/'scripts/gps_reference_map_node.py')
extractor = module('imu_extractor',HERE.parents[4]/'scripts/extract_imu_gps_map.py')


class ReferenceMapTests(unittest.TestCase):
    def data(self):
        return {'type':'FeatureCollection','metadata':{'coordinate_system':'EPSG:4326'},'features':[
            {'type':'Feature','properties':{'group':'test','color':'#123456'},
             'geometry':{'type':'LineString','coordinates':[[104.,30.],[104.001,30.]]}}]}

    def test_projection_and_changed_gps_origin(self):
        a = display.make_markers(self.data(),(30.,104.),'camera_init').markers[1]
        b = display.make_markers(self.data(),(30.,104.001),'camera_init').markers[1]
        self.assertAlmostEqual(a.points[0].x,0.)
        self.assertAlmostEqual(a.points[0].y,0.)
        self.assertAlmostEqual(a.points[1].x,96.40550696,places=5)
        self.assertAlmostEqual(b.points[1].x,0.)
        self.assertAlmostEqual(b.points[0].x,-96.40550696,places=5)
        self.assertEqual(a.type,a.LINE_STRIP)
        self.assertEqual(a.header.frame_id,'camera_init')

    def test_separate_records_do_not_get_connected_or_closed(self):
        data = self.data(); data['features'].append({'type':'Feature','properties':{'group':'test'},
            'geometry':{'type':'LineString','coordinates':[[104.1,30.],[104.2,30.]]}})
        lines = [m for m in display.make_markers(data,(30.,104.),'camera_init').markers
                 if m.action == m.ADD and m.type == m.LINE_STRIP]
        self.assertEqual([len(m.points) for m in lines],[2,2])
        self.assertNotEqual(lines[0].points[0],lines[0].points[-1])

    def test_polygons_and_invalid_coordinates_not_accepted_as_tracks(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'map.json'
            for geometry in [{'type':'Polygon','coordinates':[]},
                             {'type':'LineString','coordinates':[[104.,30.],[104.,91.]]}]:
                data = self.data(); data['features'][0]['geometry'] = geometry
                path.write_text(json.dumps(data))
                with self.assertRaises(ValueError): display.read_tracks(path)

    def test_source_precision_and_cr_cr_lf(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'record.txt'
            path.write_bytes(b'Time ullTimestamp dLatitude dLongitude GPS_Status\r\r\n'
                             b'123.456 179306792644294 30.332804659320001 104.434045372910006 4\r\r\n')
            records = list(extractor.samples(path))
            self.assertEqual(records[0][0],2)
            self.assertEqual(records[0][1]['dLongitude'],'104.434045372910006')
            self.assertEqual(records[0][1]['ullTimestamp'],'179306792644294')

    def test_invalid_samples_and_position_jumps_split_lines(self):
        segments,_ = extractor.split_tracks([[104.,30.],[104.000001,30.],None,
                                              [104.01,30.],[104.010001,30.],
                                              [104.1,30.],[104.100001,30.]], [104.,30.])
        self.assertEqual([len(s) for s in segments],[2,2,2])


if __name__ == '__main__':
    unittest.main()
