#!/usr/bin/env python3
"""Offline tests: no vehicle topics or service calls."""
import copy
from fractions import Fraction
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import zipfile

import three_party_test as core
from three_party_capture import EventTracker, scan_bags, stop_owned_recorder, critical_topics, check_session
from three_party_report import calculate, report


class RecordsTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def create(self, light='白天', scene='机坪', point='pt1', **kwargs):
        return core.new_session(self.root, light, scene, point, **kwargs)

    def test_all_48_commands_and_optional_spaces(self):
        self.assertEqual(core.POINTS, ('pt1','pt2','pt3','pt4','pt5','pt6'))
        for light in core.LIGHTS:
            for scene in core.SCENES:
                for point in core.POINTS:
                    for text in ('测试 {} {} {}'.format(light, scene, point), '测试'+light+scene+point+'点'):
                        parsed = core.parse_command(text)
                        self.assertEqual((parsed['action'], parsed['light'], parsed['scene'], parsed['point']), ('start', light, scene, point))

    def test_all_six_point_names_and_metadata(self):
        for scene in core.SCENES:
            for point in core.POINTS:
                directory=self.create(scene=scene,point=point)
                s=core.read_json(directory/'session.json')
                self.assertIn('_'+point+'_a01_',directory.name)
                self.assertEqual(s['point'],point)
                self.assertEqual(s['test_plan']['recovery_slots_total'],48)
                self.assertIsNone(s['coordinates'])
                if point in ('pt3','pt4','pt5','pt6'):
                    self.assertEqual(s['point_description'],'待补')
        with self.assertRaises(ValueError):self.create(point='pt0')
        with self.assertRaises(ValueError):self.create(point='pt7')

    def test_examples_and_quotations_cannot_trigger(self):
        for text in ('例如：测试白天机坪pt1', '“测试 白天 机坪 pt1”', '`测试白天机坪pt1`', '上次测试白天机坪pt1', '测试白天机坪pt0', '测试白天机坪pt7', '测试白天机坪pt01', '测试白天机坪pt10', '测试清晨机坪pt1', '测试白天机坪pt1\n不要录制'):
            self.assertEqual(core.parse_command(text)['action'], 'not_a_command')

    def test_only_missing_fields_returned(self):
        self.assertEqual(core.parse_command('测试 白天 pt1')['missing'], ['scene'])
        self.assertEqual(core.parse_command('测试 滑行道 pt2')['missing'], ['light'])
        self.assertEqual(core.parse_command('测试 夜晚 机坪')['missing'], ['point'])

    def test_verification_demo_commands_and_separate_attempts(self):
        for command in ('垃圾回收判定开始', '垃圾回收判定测试开始', '测试 垃圾回收判定'):
            p = core.parse_command(command)
            self.assertEqual(p['kind'], 'verification')
            self.assertIsNone(p['light'])
        for command in ('新增垃圾回收判定开始功能', '“垃圾回收判定开始”', '例如：垃圾回收判定开始'):
            self.assertEqual(core.parse_command(command)['action'], 'not_a_command')
        a = core.new_session(self.root, None, None, None, 'verification')
        b = core.new_session(self.root, None, None, None, 'verification')
        s = core.read_json(b/'session.json')
        self.assertIn('_垃圾回收判定_a02_', b.name)
        self.assertEqual(s['retest_of'], a.name)
        self.assertIsNone(s['area_m2'])
        self.assertIsNone(core.metric(s))
        result = calculate([core.read_json(a/'session.json'), s], {}, [])
        self.assertEqual(result['recovery_attempts'], 0)
        self.assertEqual(len(result['progress']), 48)
        self.assertEqual(len(result['obstacles']), 0)
        self.assertTrue(Path(report(self.root)['report_directory']).is_dir())

    def test_camera_verification_result_never_invents_physical_success(self):
        d = core.new_session(self.root, None, None, None, 'verification')
        tracker = EventTracker(d)
        for i, phase in enumerate(('IDLE', 'PREPARING', 'REVERSING', 'OBSERVING', 'PASSED')):
            tracker.observe('/fod_navigation_mode/verification/status',
                            {'phase':phase, 'result':'PASSED' if phase=='PASSED' else None},
                            {'system_ns':(i+1)*10**9, 'ros_ns':(i+1)*10**9})
        s=core.read_json(d/'session.json')
        self.assertEqual(s['verification']['result'], 'PASSED')
        self.assertIsNone(s['recovery_completed_at'])
        self.assertIsNone(s['recovery_success'])
        core.manual_event(d, 'recovery_completed', 'current user report', '回收成功')
        self.assertTrue(core.read_json(d/'session.json')['recovery_success'])

    def test_obstacle_short_commands_and_quoted_text(self):
        for text in ('避障测试', '避障测试开始', '避障测试 白天 机坪', '避障测试夜晚滑行道开始', '重测避障测试黎明机坪'):
            p = core.parse_command(text)
            self.assertEqual((p['action'], p['kind'], p['point']), ('start', 'obstacle', 'obstacle'))
        self.assertIsNone(core.parse_command('避障测试')['light'])
        self.assertIsNone(core.parse_command('避障测试')['scene'])
        self.assertEqual(core.parse_command('避障测试结束'), {'action':'complete', 'expected_kind':'obstacle'})
        self.assertEqual(core.parse_command('避障测试中止')['action'], 'abort')
        for text in ('请定义避障测试为开始录包', '“避障测试”', '例如：避障测试', '避障测试明天机坪', '避障测试白天机坪pt2', '避障测试开始开始', '避障测试\n不要录制'):
            self.assertEqual(core.parse_command(text)['action'], 'not_a_command', text)

    def test_obstacle_pending_metadata_stays_traceable(self):
        d = core.new_session(self.root, None, None, 'obstacle', 'obstacle', raw_command='避障测试')
        self.assertIn('光照待补_场景待补_避障', d.name)
        core.set_info(d, {'light':'白天','scene':'机坪', 'coordinates':{'longitude':'104.4','latitude':'30.3','crs':'EPSG:4326'},
                         'obstacle':{'object_id':'A','length_m':'.2','width_m':'.2','height_m':'.5'}}, 'current operator information')
        s=core.read_json(d/'session.json')
        self.assertEqual(s['session_id'],d.name)
        self.assertEqual(s['coordinates']['binding']['light'],'白天')
        self.assertIsNone(s['area_m2']);self.assertIsNone(core.metric(s))
        self.assertIsNone(s['recovery_success'])
        self.assertNotIn('bypassed',s['obstacle'])
        with self.assertRaises(ValueError):core.set_info(d,{'obstacle':{'bypassed':'false'}},'operator')
        result=calculate([s],{},[])
        self.assertEqual(result['recovery_attempts'],0)
        self.assertEqual(len(result['obstacles']),1)
        self.assertTrue(Path(report(self.root)['report_directory'],'避障明细.csv').is_file())

    def test_obstacle_navigation_events_are_not_bypass_confirmation(self):
        d=core.new_session(self.root,'白天','机坪','obstacle','obstacle')
        tracker=EventTracker(d)
        def emit(data,t):tracker.observe('/gps/long_range/status',data,{'system_ns':t*10**9,'ros_ns':t*10**9})
        emit({'state':'ACTIVE','route_id':'old','active':True},1)
        self.assertIsNone(core.read_json(d/'session.json')['task_started_at'])
        emit({'state':'IDLE','route_id':'old','active':False},2)
        emit({'state':'ACTIVE','route_id':'new','active':True,'paused':False},3)
        emit({'state':'COMPLETE','route_id':'new','active':False},4)
        core.manual_event(d,'obstacle_completed','operator chat','避障测试结束')
        s=core.read_json(d/'session.json')
        self.assertEqual(s['task_started_at']['epoch_ns'],3*10**9)
        self.assertNotIn('completed_at',s['obstacle'])
        self.assertNotIn('bypassed',s['obstacle'])
        self.assertIsNone(s['recovery_completed_at'])
        self.assertIsNone(s['recovery_success'])

    def test_obstacle_camera_is_critical_controller_detections_are_not(self):
        config={'topics':[{'name':n,'critical':True} for n in ('/fod_camera/image_raw/compressed','/fod_camera/camera_info','/fod/detections','/fod_navigation_mode/status','/scan','/odom')]}
        required=critical_topics(config,'obstacle')
        self.assertIn('/fod_camera/image_raw/compressed',required)
        self.assertIn('/fod_camera/camera_info',required)
        self.assertNotIn('/fod/detections',required)

    def test_obstacle_actual_speed_does_not_use_command_speed(self):
        import rosbag,rospy
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import Twist
        d=core.new_session(self.root,None,None,'obstacle','obstacle')
        (d/'bags').mkdir()
        with rosbag.Bag(str(d/'bags/simulation.bag'),'w') as bag:
            for t,v in ((10,.1),(11,.7),(12,1.2),(13,5.),(14,6.)):
                odom=Odometry();odom.twist.twist.linear.x=v
                cmd=Twist();cmd.linear.x=8.
                bag.write('/odom',odom,rospy.Time(t))
                bag.write('/cmd_vel',cmd,rospy.Time(t))
        core.write_json(d/'capture_config.json',{'requested_topics':['/odom','/cmd_vel'],'topics':[{'name':n,'critical':True} for n in ('/odom','/cmd_vel')]})
        def times(s):
            s.update(recording_started_at={'clock_mapping':{'system_ns':100*10**9,'ros_ns':10*10**9}},task_started_at={'epoch_ns':101*10**9})
            s['obstacle'].update(encounter_at={'epoch_ns':103*10**9},completed_at={'epoch_ns':104*10**9})
        core.update_session(d,times)
        check=check_session(d);speed=check['pre_obstacle_actual_speed']
        self.assertTrue(check['recording_integrity_ok']);self.assertTrue(check['task_time_covered'])
        self.assertEqual(speed['samples'],2)
        self.assertAlmostEqual(speed['mean_mps'],.95)
        self.assertEqual(speed['last_before_mps'],1.2)
        self.assertEqual(speed['max_mps'],1.2)

    def test_obstacle_report_requires_the_report_image_categories(self):
        d=core.new_session(self.root,'白天','机坪','obstacle','obstacle')
        s=core.read_json(d/'session.json')
        row=calculate([s],{},[])['rows'][0]
        self.assertIn('遇障前客户端实际车速截图',row['missing'])
        self.assertIn('绕障过程 3～4 张连续图片及时间位置',row['missing'])
        p=self.root/'synthetic_evidence.jpg';p.write_bytes(b'SIMULATION ONLY')
        for c in ('illuminance','obstacle','client_speed'):
            core.attach_evidence(d,p,'simulated evidence','00:01',category=c)
        for position in ('00:02','00:03','00:04'):
            core.attach_evidence(d,p,'simulated video frame',position,category='bypass_frame')
        row=calculate([core.read_json(d/'session.json')],{},[])['rows'][0]
        self.assertNotIn('照度计照片',row['missing'])
        self.assertNotIn('绕障过程 3～4 张连续图片及时间位置',row['missing'])
        self.assertIn('仪器实测照度',row['missing'])

    def test_names_attempts_and_no_coordinate_inheritance(self):
        first = self.create()
        core.set_info(first, {'coordinates': {'longitude': '104.4', 'latitude': '30.3', 'crs': 'EPSG:4326'}}, 'user supplied')
        second = self.create(retest=True)
        s = core.read_json(second/'session.json')
        self.assertEqual(s['attempt'], 2)
        self.assertEqual(s['retest_of'], first.name)
        self.assertIsNone(s['coordinates'])
        self.assertIn('_白天_机坪_pt1_a02_', second.name)
        self.assertTrue((first/'session.json').exists())

    def test_dms_and_hemispheres(self):
        value = core.coordinate('104°23′21.5″E', 'longitude')
        self.assertAlmostEqual(float(value['decimal_degrees']), 104 + 23/60 + 21.5/3600)
        self.assertEqual(value['raw'], '104°23′21.5″E')
        self.assertEqual(core.coordinate('30°30′0″S', 'latitude')['decimal_degrees'], '-30.5')
        for text, axis in [('181','longitude'), ('91','latitude'), ('104°60′0″','longitude'), ('30E','latitude'), ('-30N','latitude')]:
            with self.assertRaises(ValueError): core.coordinate(text, axis)

    def test_nan_and_negative_lux_rejected(self):
        directory = self.create()
        for value in ('NaN','Infinity','-1'):
            with self.assertRaises(ValueError): core.set_info(directory, {'illuminance': {'lux': value}}, 'meter')

    def test_cross_midnight_nanosecond_precision(self):
        a = core.parse_time('2026-10-14T23:59:59.999999999+08:00')
        b = core.parse_time('2026-10-15T00:00:01.000000001+08:00')
        self.assertEqual(b-a, 1000000002)
        with self.assertRaises(ValueError): core.parse_time('00:00:01')

    def test_latched_old_task_and_navigation_complete_not_pickup(self):
        directory = self.create()
        tracker = EventTracker(directory)
        def observe(state, armed, seconds):
            tracker.observe('/fod_navigation_mode/status', {'state': state, 'recovery_armed': armed},
                            {'system_ns': seconds*10**9, 'ros_ns': seconds*10**9})
        observe('FOD_COMPLETE_STOP', False, 10)
        observe('GPS_ACTIVE', False, 11)
        self.assertIsNone(core.read_json(directory/'session.json')['task_started_at'])
        observe('GPS_ACTIVE', True, 12)
        observe('FOD_ACTIVE', False, 13)
        observe('FOD_COMPLETE_STOP', False, 20)
        s = core.read_json(directory/'session.json')
        self.assertEqual(s['task_started_at']['epoch_ns'], 12*10**9)
        self.assertEqual(s['software_completed_at']['epoch_ns'], 20*10**9)
        self.assertIsNone(s['recovery_completed_at'])
        self.assertIsNone(core.metric(s))

    def test_first_already_armed_is_not_new_start(self):
        directory = self.create()
        tracker = EventTracker(directory)
        for i in (1,2,3):
            tracker.observe('/fod_navigation_mode/status', {'state':'GPS_ACTIVE','recovery_armed':True}, {'system_ns':i,'ros_ns':i})
        self.assertIsNone(core.read_json(directory/'session.json')['task_started_at'])

    def test_chat_reports_cannot_invent_actual_times(self):
        directory = self.create()
        core.manual_event(directory, 'task_started', 'user chat', '任务开始')
        core.manual_event(directory, 'recovery_completed', 'user chat', '回收完成')
        s = core.read_json(directory/'session.json')
        self.assertTrue(s['recovery_success'])
        self.assertIsNone(s['task_started_at']); self.assertIsNone(s['recovery_completed_at'])
        core.manual_event(directory,'task_started','timestamped log',actual_time='2026-10-14T23:59:00+08:00')
        core.manual_event(directory,'recovery_completed','video + user confirmation',actual_time='2026-10-15T00:01:00+08:00')
        self.assertEqual(core.metric(core.read_json(directory/'session.json')), Fraction('18743.63')/120)

    def test_arithmetic_mean_not_aggregate_time(self):
        sessions=[]
        for point, seconds in zip(core.POINTS,(200,400,600,250,350,550)):
            d=self.create(point=point); s=core.read_json(d/'session.json')
            s.update(task_started_at={'epoch_ns':0},recovery_completed_at={'epoch_ns':seconds*10**9})
            sessions.append(s)
        g=next(g for g in calculate(sessions,{},[])['groups'] if g['light']=='白天' and g['scene']=='机坪')
        actual=Fraction(int(g['average_efficiency']['numerator']),int(g['average_efficiency']['denominator']))
        self.assertEqual(actual,sum(Fraction('18743.63')/s for s in (200,400,600,250,350,550))/6)
        self.assertNotEqual(actual,Fraction('18743.63')/(Fraction(2350,6)))

    def test_group_needs_all_six_points_and_legacy_pt0_is_retained(self):
        sessions=[]
        for point in core.POINTS[:-1]:
            s=core.read_json(self.create(point=point)/'session.json')
            s.update(task_started_at={'epoch_ns':0},recovery_completed_at={'epoch_ns':200*10**9})
            sessions.append(s)
        old=copy.deepcopy(sessions[0])
        old.update(session_id='legacy_pt0',point='pt0',point_description='中心',recovery_success=False)
        old.pop('test_plan')
        d=self.root/'sessions'/'legacy_pt0';d.mkdir()
        core.write_json(d/'session.json',old)
        original=(d/'session.json').read_bytes()
        adoption={'session_id':'legacy_pt0','key':['recovery','白天','机坪','pt0'],'reason':'old adoption'}
        result=calculate(sessions+[old],{},[adoption])
        g=next(g for g in result['groups'] if g['light']=='白天' and g['scene']=='机坪')
        self.assertIsNone(g['average_efficiency'])
        self.assertIsNone(g['selected_attempts'][-1])
        self.assertEqual(result['recovery_slots_with_attempts'],5)
        legacy=next(r for r in result['rows'] if r['point']=='pt0')
        self.assertFalse(legacy['adopted']);self.assertFalse(legacy['recovery_success'])
        self.assertIn('历史',legacy['plan_status'])
        self.assertEqual(original,(d/'session.json').read_bytes())
        self.assertEqual(len(result['progress']),48)
        self.assertNotIn('pt0',{r['point'] for r in result['progress']})

    def test_one_bad_light_not_masked_by_high_average(self):
        sessions=[]
        for light in core.LIGHTS:
            for point in core.POINTS:
                d=self.create(light=light,point=point);s=core.read_json(d/'session.json')
                seconds=400 if light=='黎明' else 200
                s.update(task_started_at={'epoch_ns':0},recovery_completed_at={'epoch_ns':seconds*10**9})
                sessions.append(s)
        result=calculate(sessions,{},[])['scenes'][0]
        self.assertTrue(result['average_ge_50']);self.assertFalse(result['each_light_ge_50'])

    def test_failure_and_retest_adoption_are_traceable(self):
        a=self.create();b=self.create(retest=True)
        x=core.read_json(a/'session.json');y=core.read_json(b/'session.json')
        x.update(phase='aborted',recovery_success=False);y.update(phase='completed',recovery_success=True)
        result=calculate([x,y],{},[])
        p=next(p for p in result['progress'] if p['light']=='白天' and p['scene']=='机坪' and p['point']=='pt1')
        self.assertEqual(p['selected_attempt'],a.name);self.assertEqual(len(p['all_attempts']),2)
        result=calculate([x,y],{},[{'session_id':y['session_id'],'key':['recovery','白天','机坪','pt1'],'reason':'confirmed retest'}])
        p=next(p for p in result['progress'] if p['light']=='白天' and p['scene']=='机坪' and p['point']=='pt1')
        self.assertEqual(p['selected_attempt'],b.name);self.assertEqual(len(result['rows']),2)
        self.assertFalse(result['rows'][0]['adopted']);self.assertTrue(result['rows'][1]['adopted'])
        self.assertNotIn('当前有不合格项', result['overall_conclusion'])

    def test_exact_threshold_without_display_rounding(self):
        for scene, reference in [('机坪','374.8726'), ('滑行道','457.635')]:
            s=core.read_json(self.create(scene=scene)/'session.json')
            duration=int(Fraction(reference)*10**9)
            s.update(task_started_at={'epoch_ns':0},recovery_completed_at={'epoch_ns':duration})
            self.assertEqual(core.metric(s),50)
            s['recovery_completed_at']['epoch_ns']+=1
            self.assertLess(core.metric(s),50)

    def test_obstacle_size_and_changed_object_and_location(self):
        sessions=[]
        for light, size, object_id, longitude in [('黎明','0.2','A','104.1'),('白天','0.19','B','104.2'),('黄昏','0.2','A','104.3'),('夜晚','0.2','D','104.1')]:
            d=self.create(light=light,point='obstacle',kind='obstacle')
            core.set_info(d, {'coordinates':{'longitude':longitude,'latitude':'30.3','crs':'EPSG:4326'},
                             'obstacle':{'object_id':object_id,'length_m':size,'width_m':'0.2','height_m':'0.5'}}, 'simulation')
            sessions.append(core.read_json(d/'session.json'))
        rows=calculate(sessions,{},[])['obstacles']
        self.assertTrue(rows[0]['dimensions_ok']);self.assertTrue(rows[0]['changed_object_and_position'])
        self.assertFalse(rows[1]['dimensions_ok'])
        self.assertFalse(rows[2]['changed_object_and_position']);self.assertFalse(rows[3]['changed_object_and_position'])

    def test_empty_progress_and_editable_report(self):
        result=report(self.root)
        p=Path(result['report_directory'])
        self.assertEqual(result['recovery_slots_with_attempts'],0)
        data=core.read_json(p/'results.json')
        self.assertEqual(len(data['progress']),48)
        self.assertEqual(result['recovery_slots_total'],48)
        self.assertTrue(all(not v for v in data['obstacle_light_complete'].values()))
        with zipfile.ZipFile(p/'三方测试报告.docx') as z:
            ET.fromstring(z.read('word/document.xml'))
            self.assertIn('待补',z.read('word/document.xml').decode())
            self.assertIn('48 点进度',z.read('word/document.xml').decode())
            self.assertIn('ηpt6）/6',z.read('word/document.xml').decode())

    def test_recorder_stop_does_not_signal_other_processes(self):
        directory=self.create()
        first=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)'],start_new_session=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        unrelated=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)'],start_new_session=True)
        try:
            core.write_json(directory/'recorder_process.json',core.process_identity(first.pid))
            self.assertTrue(stop_owned_recorder(directory));first.wait(timeout=3)
            self.assertIsNone(unrelated.poll())
        finally:
            if first.poll() is None:first.terminate()
            unrelated.terminate();unrelated.wait(timeout=3)

    def test_reused_pid_identity_is_rejected(self):
        directory=self.create()
        proc=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)'],start_new_session=True)
        try:
            identity=core.process_identity(proc.pid);identity['start_ticks']=str(int(identity['start_ticks'])+1)
            core.write_json(directory/'recorder_process.json',identity)
            self.assertTrue(stop_owned_recorder(directory));self.assertIsNone(proc.poll())
        finally:proc.terminate();proc.wait(timeout=3)

    def test_full_bag_read_and_corruption_are_distinguished(self):
        import rosbag,rospy
        from std_msgs.msg import String
        good=self.root/'good.bag'
        with rosbag.Bag(str(good),'w',compression='lz4') as bag:
            bag.write('/test',String(data='sample'),rospy.Time(10,123))
            bag.write('/test',String(data='end'),rospy.Time(11,456))
        result=scan_bags([good]);self.assertTrue(result['readable']);self.assertEqual(result['counts']['/test'],2)
        bad=self.root/'bad.bag';bad.write_bytes(good.read_bytes()[:100])
        result=scan_bags([bad]);self.assertFalse(result['readable'])

    def test_live_probe_leaves_original_unchanged_and_split_counts(self):
        import rosbag,rospy
        from std_msgs.msg import String
        paths=[]
        for i in range(2):
            p=self.root/('part_%d.bag'%i)
            with rosbag.Bag(str(p),'w',compression='lz4') as bag:
                bag.write('/test',String(data='sample'),rospy.Time(10+i,123))
            paths.append(p)
        before=[core.sha256(p) for p in paths]
        result=scan_bags(paths,live=True)
        self.assertTrue(result['readable']);self.assertEqual(result['counts']['/test'],2)
        self.assertEqual(before,[core.sha256(p) for p in paths])


if __name__=='__main__': unittest.main(verbosity=2)
