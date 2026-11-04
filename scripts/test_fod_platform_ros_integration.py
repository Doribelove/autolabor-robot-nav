#!/usr/bin/env python3
"""Isolated ROS acceptance test. Uses fake navigation and a fake AMQP channel.

Run with ROS_MASTER_URI=http://127.0.0.1:11321. Never connects to the field broker.
"""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import threading
import time
from types import SimpleNamespace as Obj
import xmlrpc.client


def main():
    uri = 'http://127.0.0.1:11321'
    if os.environ.get('ROS_MASTER_URI') != uri:
        raise SystemExit('Use the isolated ROS_MASTER_URI '+uri)
    probe = socket.socket()
    if probe.connect_ex(('127.0.0.1',11321)) == 0:
        raise SystemExit('Test port already in use; refusing to share a ROS master')
    probe.close()
    with tempfile.TemporaryDirectory(prefix='fod-platform-integration-') as directory:
        root = Path(directory)
        with (root/'master.log').open('w') as master_log:
            master = subprocess.Popen(['roscore','-p','11321'], stdout=master_log, stderr=subprocess.STDOUT,
                                      start_new_session=True)
        try:
            for _ in range(100):
                try:
                    if xmlrpc.client.ServerProxy(uri).getPid('/test')[0] == 1: break
                except OSError: pass
                time.sleep(.1)
            else: raise AssertionError('isolated master failed to start')
            run_test(root)
        finally:
            os.killpg(master.pid, signal.SIGINT)
            try: master.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(master.pid, signal.SIGTERM)
                master.wait(timeout=5)


def run_test(root):
    import rospy
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import String
    from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse
    from fod_platform_link import CloudPoseReporter

    rospy.init_node('platform_integration')
    config = json.loads((Path(__file__).resolve().parents[1]/'config/fod_platform_link.json').read_text())
    config.update(log_directory=str(root), mission_state_file=str(root/'missions.json'))
    (root/'config.json').write_text(json.dumps(config))
    rospy.set_param('~config',str(root/'config.json'))
    mode = dict(state='GPS_ACTIVE', recovery_armed=False, sweep_cleanup_pending=False,
                sweep_started_for_recovery=False, sweep_cleanup_confirmed=True)
    route = dict(state='IDLE', active=False, route_id='old')
    lock = threading.RLock()
    goals, calls, acks, sent = [], [], [], []
    behavior = {'recovery':False, 'hold_cleanup':False}
    mp = rospy.Publisher('/fod_navigation_mode/status',String,queue_size=20,latch=True)
    rp = rospy.Publisher('/gps/long_range/status',String,queue_size=20,latch=True)
    def publish(_=None):
        with lock:
            mp.publish(String(data=json.dumps(mode)))
            rp.publish(String(data=json.dumps(route)))
    timer = rospy.Timer(rospy.Duration(.05),publish)
    def clear(_):
        calls.append('clear')
        with lock: route.update(active=False, state='CANCELED')
        publish()
        return TriggerResponse(True,'cleared')
    def prepare(_):
        calls.append('prepare')
        with lock: mode.update(state='GPS_ACTIVE',recovery_armed=False)
        publish()
        return TriggerResponse(True,'prepared')
    def pause(request):
        calls.append(('pause', request.data))
        return SetBoolResponse(True,'ok')
    def arm(request):
        calls.append(('arm',request.data))
        with lock:
            mode.update(recovery_armed=request.data)
            behavior['recovery'] = request.data
        publish()
        return SetBoolResponse(True,'ok')
    services = [rospy.Service('/gps/long_range/clear_route',Trigger,clear),
                rospy.Service('/fod_navigation_mode/prepare_gps_goal',Trigger,prepare),
                rospy.Service('/gps/long_range/set_paused',SetBool,pause),
                rospy.Service('/fod_navigation_mode/set_fod_enabled',SetBool,arm)]
    def goal(message):
        goals.append((message.latitude,message.longitude))
        with lock:
            route.update(route_id='route-'+str(len(goals)), active=True, state='ACTIVE_FINAL',
                         final_latitude=message.latitude, final_longitude=message.longitude)
            if message.latitude == 33:
                route.update(state='REJECTED_NEW_GOAL', active=False, reason='模拟禁区')
        publish()
        if message.latitude == 33: return
        time.sleep(.12)
        with lock:
            route.update(state='COMPLETE',active=False)
            if behavior['recovery']:
                mode.update(state='FOD_VERIFY',recovery_armed=False)
        publish()
        time.sleep(.12)
        if behavior['recovery']:
            with lock:
                mode.update(state='FOD_COMPLETE_STOP',sweep_cleanup_pending=behavior['hold_cleanup'],
                            sweep_cleanup_confirmed=not behavior['hold_cleanup'],verification={'phase':'PASSED'})
            publish()
    sub = rospy.Subscriber('/gps/goal_fix',NavSatFix,goal,queue_size=5)
    reporter = CloudPoseReporter()
    channel = Obj(basic_ack=lambda **kw:acks.append(kw), basic_publish=lambda **kw:sent.append(kw))
    reporter.channel = channel
    reporter.connection = Obj(process_data_events=lambda:None)
    def receive(identity, **extra):
        doc = dict(missionId=identity,vehicleId='FOD-102',type='normal',waypoints=[{'lat':30,'lon':104}])
        doc.update(extra)
        reporter.receive(channel,Obj(exchange='isolated-test',routing_key='',redelivered=False,delivery_tag=len(acks)+1),
                         Obj(message_id=identity),json.dumps(doc).encode())
    def until(predicate):
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            if predicate(): return
            time.sleep(.03)
        raise AssertionError('timed out: '+str(reporter.task_runner.snapshot()))
    def state(): return reporter.task_runner.snapshot()
    try:
        until(lambda:reporter.task_runner.backend.idle() and rp.get_num_connections()>0)
        receive('COMM-TEST-FOD-102-ISOLATED')
        receive('OTHER',vehicleId='100')
        time.sleep(.3)
        assert not goals and not calls
        print('PASS communication-only/other vehicle cause no navigation calls', flush=True)
        receive('ISOLATED-NORMAL',waypoints=[{'lat':30,'lon':104},{'lat':31,'lon':104}])
        until(lambda:state().get('state')=='COMPLETED' and not state()['active'])
        assert goals==[(30,104),(31,104)]
        assert not any(c==('arm',True) for c in calls)
        receive('ISOLATED-NORMAL',waypoints=[{'lat':30,'lon':104},{'lat':31,'lon':104}])
        time.sleep(.3)
        assert len(goals)==2
        print('PASS receive -> persist -> ROS services -> two ordered goals -> dedup',flush=True)
        behavior['hold_cleanup']=True
        receive('ISOLATED-FOD',type='recovery')
        until(lambda:mode['state']=='FOD_COMPLETE_STOP')
        time.sleep(.2)
        assert state()['state']=='RUNNING'
        mode.update(sweep_cleanup_pending=False,sweep_cleanup_confirmed=True)
        publish()
        until(lambda:state().get('state')=='COMPLETED' and not state()['active'])
        assert state()['results'][0]['result']=='RECOVERY_FLOW_COMPLETED'
        print('PASS recovery completion waits for verification and sweep cleanup',flush=True)
        receive('ISOLATED-REJECTED',waypoints=[{'lat':33,'lon':104},{'lat':34,'lon':104}])
        until(lambda:state().get('state')=='FAILED' and not state()['active'])
        assert (34,104) not in goals
        print('PASS rejected goal fails current task without dispatching next point',flush=True)
        reporter.publish_once(dict(timestamp_ms=1,local_timestamp_ms=1,device='FOD-102',longitude=104,latitude=30,
            course_angle=0,roll=0,pitch=0,status=9,error_code=0,speed=0,mode=1,soc=50))
        assert json.loads(sent[-1]['body'])['DATA'][0]['STATUS']==7
        assert len(acks)==6
        print('PASS failed result reflected in heartbeat; six incoming packets ACKed',flush=True)
        print(json.dumps(dict(isolated_master=os.environ['ROS_MASTER_URI'], goals=goals,
                             result='PASS',real_broker_contacted=False,physical_motion=False)))
    finally:
        reporter.stop_task_worker()
        timer.shutdown()
        reporter.inbox_log.close()
        reporter.outbox_log.close()
        rospy.signal_shutdown('isolated tests complete')


if __name__=='__main__': main()
