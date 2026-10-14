#!/usr/bin/env python3
"""Service/persistence integration on a separate ROS master; no vehicle nodes."""
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xmlrpc.client


def main():
    work=Path(tempfile.mkdtemp(prefix='gps-keepout-test-'))
    env=dict(os.environ,ROS_MASTER_URI='http://127.0.0.1:11531',ROS_HOME=str(work/'ros'))
    env.pop('ROS_IP',None);env.pop('ROS_HOSTNAME',None)
    os.environ.update(env)
    processes=[]
    def start(args,name):
        f=(work/name).open('w')
        p=subprocess.Popen(args,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        f.close();processes.append(p);return p
    try:
        start(['roscore','-p','11531'],'master.log')
        for _ in range(100):
            try:
                if xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/test')[0]==1:break
            except OSError:pass
            time.sleep(.1)
        import rospy
        import yaml
        from sensor_msgs.msg import NavSatFix
        from visualization_msgs.msg import MarkerArray
        from autolabor_gps_keepout.msg import KeepoutPolygons
        from autolabor_gps_keepout.srv import KeepoutCommand
        from autolabor_gps_keepout.geometry import xy_to_gps
        rospy.init_node('keepout_service_test',anonymous=True,disable_signals=True)
        rospy.set_param('/gps/origin_lat',30.3);rospy.set_param('/gps/origin_lon',104.39)
        backend=Path(__file__).resolve().parents[1]/'scripts/gps_keepout_node.py'
        args=['python3',str(backend),'_storage_file:='+str(work/'config.json'),
              '_targets_file:='+str(work/'targets.json')]
        node=start(args,'backend.log')
        rospy.wait_for_service('/gps/keepout/command',10)
        service=rospy.ServiceProxy('/gps/keepout/command',KeepoutCommand)
        goals=[];sub=rospy.Subscriber('/gps/goal_fix',NavSatFix,goals.append)
        def call(command,data,ok=True):
            r=service(command,json.dumps(data,ensure_ascii=False))
            assert r.success==ok,(command,r.message)
            return json.loads(r.config_json) if r.config_json else None
        config={'version':1,'enabled':True,'clearance_m':1.,'polygons':[{'name':'test',
            'points':[xy_to_gps(x,y,(30.3,104.39)) for x,y in [(10,-5),(20,-5),(20,5),(10,5)]]}]}
        applied=call('apply',config)
        call('apply',{'enabled':True,'polygons':[{'points':[[30.,104.],[30.1,104.]]}]},False)
        assert call('get',{})==applied
        call('save',{'path':str(work/'saved.json'),'config':applied})
        assert call('load',{'path':str(work/'saved.json')})==applied
        call('export',{'path':str(work/'map.yaml'),'config':applied,'resolution':.1})
        metadata=yaml.safe_load((work/'map.yaml').read_text())
        assert metadata['gps_origin']==[30.3,104.39]
        from PIL import Image
        grid=Image.open(work/'map.pgm')
        assert grid.getextrema()==(0,254)
        text='pt0\t104.3892865\t30.29387435\npt1\t104.3898634\t30.29726472\npt2\t104.3904909\t30.30145079'
        points=call('targets_parse',{'text':text});call('targets_set',points)
        assert call('get',{})==applied  # FOD import must not edit polygons.
        markers=rospy.wait_for_message('/gps/targets/markers',MarkerArray,3)
        assert len(markers.markers)==7
        assert not goals  # Displaying/importing must not drive.
        from test_airport import fixture_zip
        fixture_zip(work/'airport.zip')
        candidate=call('airport_import',{'path':str(work/'airport.zip'),'config':applied})
        assert call('get',{})==applied  # Import only stages, never activates.
        assert candidate['polygons']==applied['polygons']
        assert candidate['airport']['crs']=='EPSG:4326'
        applied=call('apply',candidate)
        message=rospy.wait_for_message('/gps/keepout/polygons',KeepoutPolygons,3)
        assert message.keep_in_enabled and len(message.keep_in_polygons)>1
        call('export',{'path':str(work/'airport_map.yaml'),'config':applied,'resolution':.5})
        airport_grid=Image.open(work/'airport_map.pgm')
        assert airport_grid.getpixel((0,0))==0  # Outside is occupied.
        assert airport_grid.getextrema()==(0,254)
        call('airport_import',{'path':str(work/'missing.zip'),'config':applied},False)
        assert call('get',{})==applied
        # Restart the backend and verify both independent stores survive.
        node.send_signal(signal.SIGINT);node.wait(5)
        start(args,'backend_restart.log');time.sleep(1)
        rospy.wait_for_service('/gps/keepout/command',10)
        assert call('get',{})==applied
        assert call('targets_get',{})==points
        call('apply',{'enabled':False,'polygons':[]})
        assert not call('get',{})['polygons']
        message=rospy.wait_for_message('/gps/keepout/polygons',KeepoutPolygons,3)
        assert not message.keep_in_enabled and not message.keep_in_polygons
        assert not goals
        print('PASS: apply/invalid input/save/load/raster/airport import/holes/markers/no-goal/restart/delete; '+str(work))
    finally:
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(5)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(5)


if __name__=='__main__':main()
