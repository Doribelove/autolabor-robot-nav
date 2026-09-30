#!/usr/bin/env python3
"""Q04-only small-pulse command source. No environment steps or navigation policy."""
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from urllib.parse import urlparse
WORKSPACE=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(WORKSPACE/'scripts/astra_gazebo'))
from passive_sweep_q04 import CommandSchedule,validate_contract,read_bound,NAME,PERIOD_NS,validate_robot_description

def main():
    import rospy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String,Int64
    from std_srvs.srv import Trigger,TriggerResponse
    rospy.init_node('passive_command',anonymous=False)
    if urlparse(os.environ.get('ROS_MASTER_URI','')).hostname not in ('127.0.0.1','localhost'):
        raise RuntimeError('isolated loopback master required')
    if rospy.get_param('/use_sim_time',False)is not True or rospy.get_param('/m2_gazebo/simulation_only',False)is not True:
        raise RuntimeError('simulation-only guards required')
    value=validate_contract(json.loads(read_bound({'path':rospy.get_param('~contract_path'),'sha256':rospy.get_param('~contract_sha256')})))
    validate_robot_description(rospy.get_param('/robot_description'),value)
    schedule=CommandSchedule(value);lock=threading.RLock();start_called=[False]
    publisher=rospy.Publisher('/cmd_vel_navigation',Twist,queue_size=1)
    events=rospy.Publisher('/astra_q04/command_event',String,queue_size=1000)
    status=rospy.Publisher('/astra_q04/command_status',String,queue_size=100,latch=True)
    start_pub=rospy.Publisher('/astra_q04/start_ns',Int64,queue_size=1,latch=True)
    def publish_status():
        status.publish(String(json.dumps({'stage':NAME,'run_id':value['run_id'],'seed':value['seed'],
          'source_ns':rospy.Time.now().to_nsec(),'state':schedule.state,'start_ns':schedule.start_ns,'reason':schedule.reason})))
    def send(command):
        if command['slot'] is not None:
            command=schedule.prepare_send(command,rospy.Time.now().to_nsec())
        else:
            command=dict(command,actual_ns=rospy.Time.now().to_nsec(),scheduled_ns=rospy.Time.now().to_nsec())
        if not all(math.isfinite(command[k]) for k in ('v','w')):raise ValueError('nonfinite command')
        message=Twist();message.linear.x=command['v'];message.angular.z=command['w'];publisher.publish(message)
        events.publish(String(json.dumps(dict(command,run_id=value['run_id'],wall_monotonic_ns=time.monotonic_ns()))))
    def start(_request):
        with lock:
            if start_called[0]:return TriggerResponse(False,'one-shot start already requested')
            start_called[0]=True
            try:
                stamp=schedule.start(rospy.Time.now().to_nsec());start_pub.publish(Int64(stamp));publish_status()
                return TriggerResponse(True,json.dumps({'run_id':value['run_id'],'seed':value['seed'],'start_ns':stamp}))
            except Exception as error:
                schedule.state='fault';schedule.reason=str(error);publish_status()
                return TriggerResponse(False,str(error))
    rospy.Service('/astra_q04/start',Trigger,start)
    last_progress=time.monotonic();last_clock=None;last_zero=None;last_zero_state=None;last_state=None
    while not rospy.is_shutdown():
        with lock:
            now=rospy.Time.now().to_nsec();wall=time.monotonic()
            if last_clock is None or now>last_clock:last_progress=wall
            if last_clock is not None and now<last_clock:
                schedule.state='fault';schedule.reason='simulation source clock reset'
            last_clock=now
            if now>0 and wall-last_progress>value['clock_stall_wall_s']:
                schedule.state='fault';schedule.reason='source clock wall stall'
            try:
                command=schedule.poll(now) if schedule.state=='running' else None
                if command:send(command)
            except Exception as error:
                schedule.state='fault';schedule.reason=type(error).__name__+': '+str(error)
            if schedule.state in ('bootstrap','fault','complete'):
                # One zero on state entry, then only as source time advances.
                # Repeating wall-time zeros while paused can overflow the driver queue.
                ready=(schedule.state!='bootstrap' or publisher.get_num_connections()>=2)
                # At clock0 require both relay and recorder subscriptions before
                # the one zero, allowing ShapeShifter relay output advertisement.
                if ready and (last_zero is None or schedule.state!=last_zero_state or now-last_zero>=PERIOD_NS):
                    send({'phase':schedule.state+'_zero','actual_ns':now,'scheduled_ns':now,'slot':None,
                          'v':0.,'w':0.,'steering':0.,'lateness_ns':0});last_zero=now;last_zero_state=schedule.state
            if schedule.last_ns is not None:last_clock=max(last_clock,schedule.last_ns)
            if schedule.state!=last_state:publish_status();last_state=schedule.state
        time.sleep(.002)
    return 0
if __name__=='__main__':raise SystemExit(main())
