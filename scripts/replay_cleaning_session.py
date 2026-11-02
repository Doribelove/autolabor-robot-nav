#!/usr/bin/env python3
"""Replay recorded ROS and BV02 into the real operator Qt, on loopback only."""
import argparse
import copy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import queue
import shutil
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
import urllib.request
import xmlrpc.client

WS = Path(__file__).resolve().parents[1]
ROOT = Path(os.environ['CLEANING_REPLAY_ROOT'])
RECORD = Path(os.environ['CLEANING_RECORDING_DIR'])
URL = 'http://127.0.0.1:18894'


def atomic_json(path, data):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
    tmp.replace(path)


def replay_packet(data, clock_s, epoch, wall_s):
    """Keep JPEG bytes and recorded measurements; adapt only Qt wall-time freshness/IDs."""
    magic, nj, nr, na = struct.unpack('!4sIII', data[:16])
    if magic != b'BV02' or not 0 < nj <= 128*1024 or 16+nj+nr+na > len(data):
        raise ValueError('Invalid recorded BV02')
    meta = json.loads(data[16:16+nj])
    original_id = dict(meta['source_id'])
    replay_id = dict(original_id, session='replay-%d-%s' % (epoch, original_id['session']))
    shift = wall_s-clock_s
    original_times = {key:meta[key] for key in ('produced_epoch_s','result_epoch_s') if key in meta}
    for key, value in original_times.items():
        meta[key] = value+shift
    for key in ('source_id','image_source_id','result_source_id'):
        meta[key] = replay_id
    for row in meta['camera_previews']:
        row['source_id'] = replay_id
    meta['replay'] = dict(recorded_source_id=original_id, recorded_times=original_times,
                          clock_epoch_s=clock_s, wall_time_shift_s=shift,
                          image_bytes_unchanged=True, inference_rerun=False)
    header = json.dumps(meta, ensure_ascii=False, separators=(',',':')).encode()
    if len(header) > 128*1024:
        raise ValueError('Replay metadata exceeds Qt limit')
    return struct.pack('!4sIII', magic, len(header), nr, na)+header+data[16+nj:]


def controls():
    from PyQt5.QtCore import QTimer, Qt
    from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton
    app = QApplication([])
    panel = QWidget()
    panel.setWindowTitle('清扫回放控制')
    panel.setWindowFlag(Qt.WindowStaysOnTopHint)
    layout = QVBoxLayout(panel)
    label = QLabel('正在连接回放…')
    layout.addWidget(label)
    buttons = QHBoxLayout()
    layout.addLayout(buttons)
    def command(action):
        try:
            req = urllib.request.Request(URL+'/control',json.dumps(dict(action=action)).encode(),
                                         {'Content-Type':'application/json'})
            urllib.request.urlopen(req,timeout=2).close()
        except Exception as exc:
            label.setText(str(exc))
    for title, action in [('暂停','pause'),('继续','resume'),('从头播放','restart'),
                           ('原始Qt录像','video'),('关闭回放','stop')]:
        button = QPushButton(title)
        button.clicked.connect(lambda _, a=action: command(a))
        buttons.addWidget(button)
    def refresh():
        try:
            state=json.load(urllib.request.urlopen(URL+'/state',timeout=.5))
            label.setText('%s · %s · %.1f / %.1f 秒 · %s' %
                (state['name'],state['playback_state'],state['position_s'],state['duration_s'],
                 '重新定位建图' if state.get('mode')=='mapping' else '录包数据回放'))
        except Exception:
            label.setText('等待回放服务…')
    timer=QTimer();timer.timeout.connect(refresh);timer.start(500)
    panel.move(40,80)
    panel.show()
    app.exec_()


def main():
    if [name for _,name in socket.if_nameindex()] != ['lo']:
        raise RuntimeError('Recorded controls require the private loopback replay namespace')
    import rosbag
    import yaml
    mapping_mode=os.environ.get('CLEANING_REPLAY_MODE','recorded')=='mapping'
    fixed_frame='camera_init' if mapping_mode else 'map'
    children=[];streams=[];stop=threading.Event();commands=queue.Queue()
    state_lock=threading.Lock()
    latest=None
    state=dict(name=json.loads((RECORD/'recording.json').read_text()).get('experiment_name',RECORD.name),
        recording=str(RECORD),playback_state='正在加载',position_s=0.,duration_s=0.,epoch=0,
        network_interfaces=['lo'],robot_connected=False,vision_frames=0,qt_map_status='',
        localization_status='',cleaner_status={},graph_nodes=[],error=None,
        mode='mapping' if mapping_mode else 'recorded',fixed_frame=fixed_frame,
        workflow_snapshot=os.environ.get('CLEANING_REPLAY_SNAPSHOT',''),
        slam_odometry_frames=0,slam_cloud_frames=0,history_cloud_points=0,local_costmap_frames=0)
    for sig in (signal.SIGINT,signal.SIGTERM):
        signal.signal(sig,lambda *_:stop.set())
    def spawn(name,args,env=None):
        stream=(ROOT/(name+'.log')).open('a');streams.append(stream)
        child=subprocess.Popen(args,env=env,stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,
                               start_new_session=True)
        children.append(child)
        return child
    def finish(child):
        if child is not None and child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
    server=None;mapping=None
    try:
        map_dir=ROOT/'map'
        if mapping_mode:map_dir.mkdir(exist_ok=True)
        else:
            from replay_assets import recorded_map_directory
            shutil.copytree(recorded_map_directory(RECORD),map_dir,dirs_exist_ok=True)
        rviz=yaml.safe_load((RECORD/'assets/operator_navigation.rviz').read_text())
        rviz['Visualization Manager']['Displays'].append(dict(
            Class='rviz/Path',Name='Recorded localization path (estimate)',Enabled=True,
            Topic='/replay/localization_path',Value=True,Alpha=1,Color='255; 220; 50',
            **{'Buffer Length':1,'Line Style':'Billboards','Line Width':.06,'Pose Style':'None','Queue Size':2}))
        if mapping_mode:
            from replay_cleaning_mapping import MappingReplay,MAPPING_INPUTS,local_rviz
            rviz=local_rviz(rviz)
            mapping=MappingReplay(WS,ROOT,RECORD,spawn,finish)
        rviz_path=ROOT/'replay.rviz';rviz_path.write_text(yaml.safe_dump(rviz,sort_keys=False))
        bags=sorted(RECORD.glob('localization_*.bag'))+sorted((RECORD/'visual').glob('take_*/visual_*.bag'))
        if not bags:raise RuntimeError('No finalized recording bags')
        topics=set();starts=[];ends=[]
        for path in bags:
            with rosbag.Bag(str(path)) as bag:
                topics.update(bag.get_type_and_topic_info().topics)
                starts.append(bag.get_start_time());ends.append(bag.get_end_time())
        start,end=min(starts),max(ends)
        # Do not overwrite this Qt's own readiness or the replay clock with historical diagnostics.
        topics-=set(['/rosout','/rosout_agg','/clock','/autolabor_operator_gui/map_display_status'])
        if mapping_mode:topics &= MAPPING_INPUTS
        state.update(duration_s=end-start,start_epoch_s=start,end_epoch_s=end)
        master=spawn('roscore',['roscore','-p','11593'])
        deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            try:
                if xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/replay_start')[0]==1:break
            except OSError:pass
            if master.poll() is not None:raise RuntimeError('Replay ROS master exited')
            stop.wait(.1)
        else:raise RuntimeError('Replay ROS master unavailable')
        import rospy
        import rosparam
        import rosgraph
        from std_msgs.msg import UInt8MultiArray,String
        from rosgraph_msgs.msg import Clock
        from nav_msgs.msg import Odometry,Path as RosPath,OccupancyGrid
        from sensor_msgs.msg import PointCloud2
        from geometry_msgs.msg import PoseStamped
        from std_srvs.srv import SetBool
        params=json.loads((RECORD/'parameters_start.json').read_text())
        for key in ('run_id','roslaunch','rosversion','rosdistro','conbine','autolabor_operator_gui'):
            params.pop(key,None)
        if mapping_mode:
            params={k:v for k,v in params.items() if k in (
                'common','preprocess','mapping','publish','pcd_save','feature_extract_enable',
                'point_filter_num','max_iteration','filter_size_surf','filter_size_map',
                'cube_side_length','runtime_pos_log_enable')}
        rosparam.upload_params('/',params)
        rospy.set_param('/use_sim_time',True)
        rospy.init_node('cleaning_replay_bridge',disable_signals=True)
        clock_s=start
        trajectory=RosPath();trajectory.header.frame_id=fixed_frame
        path_pub=rospy.Publisher('/replay/localization_path',RosPath,queue_size=1,latch=True)
        def on_clock(msg):
            nonlocal clock_s
            clock_s=msg.clock.to_sec()
            with state_lock:
                state['position_s']=max(0,min(end-start,clock_s-start))
                if clock_s>=end-.1:state['playback_state']='播放结束'
        def on_vision(msg):
            nonlocal latest
            data=bytes(msg.data)
            if data[:4]!=b'BV02':return
            with state_lock:
                latest=data;state['vision_frames']+=1
        def on_localization(msg):
            if msg.header.frame_id!=fixed_frame:return
            if mapping_mode:
                with state_lock:
                    state['slam_odometry_frames']+=1
                    p=msg.pose.pose.position
                    state['slam_position_m']=[p.x,p.y,p.z]
                    state['slam_header_epoch_s']=msg.header.stamp.to_sec()
            pose=PoseStamped();pose.header=msg.header;pose.pose=msg.pose.pose
            trajectory.header.stamp=msg.header.stamp
            if not trajectory.poses or (msg.header.stamp-trajectory.poses[-1].header.stamp).to_sec()>=.2:
                trajectory.poses.append(pose);path_pub.publish(trajectory)
        def on_text(key,msg):
            value=msg.data
            if key=='cleaner_status':
                try:value=json.loads(value)
                except ValueError:pass
            with state_lock:state[key]=value
        rospy.Subscriber('/clock',Clock,on_clock,queue_size=5)
        rospy.Subscriber('/experiment/http/vision',UInt8MultiArray,on_vision,queue_size=2,buff_size=4*1024*1024)
        rospy.Subscriber('/Odometry' if mapping_mode else '/localization',Odometry,on_localization,queue_size=10)
        if mapping_mode:
            def on_cloud(msg,history=False):
                with state_lock:
                    if history:state['history_cloud_points']=msg.width*msg.height
                    else:state['slam_cloud_frames']+=1
            def on_costmap(msg):
                with state_lock:
                    state['local_costmap_frames']+=1
                    state['local_costmap_frame']=msg.header.frame_id
            rospy.Subscriber('/cloud_registered',PointCloud2,on_cloud,queue_size=1)
            rospy.Subscriber('/static_mapping/history_cloud',PointCloud2,lambda m:on_cloud(m,True),queue_size=1)
            rospy.Subscriber('/move_base/local_costmap/costmap',OccupancyGrid,on_costmap,queue_size=1)
        for topic,key in [('/autolabor_operator_gui/map_display_status','qt_map_status'),
                          ('/fast_lio/localization_status','localization_status'),('/bev_cleaner/status','cleaner_status')]:
            rospy.Subscriber(topic,String,lambda msg,k=key:on_text(k,msg),queue_size=5)
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*_):pass
            def do_GET(self):
                with state_lock:
                    packet=latest;info=copy.deepcopy(state);epoch=state['epoch']
                if self.path=='/state':
                    payload=json.dumps(info,ensure_ascii=False).encode();mime='application/json'
                elif self.path=='/vision_frame' and packet is not None:
                    payload=replay_packet(packet,clock_s,epoch,time.time());mime='application/octet-stream'
                else:self.send_error(503,'Waiting for recorded frame');return
                self.send_response(200);self.send_header('Content-Type',mime)
                self.send_header('Content-Length',str(len(payload)));self.end_headers()
                try:self.wfile.write(payload)
                except (BrokenPipeError,ConnectionResetError):pass
            def do_POST(self):
                if self.path!='/control':self.send_error(404);return
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<1024:self.send_error(400);return
                action=json.loads(self.rfile.read(size)).get('action')
                if action not in ('pause','resume','restart','stop','video'):self.send_error(400);return
                commands.put(action);self.send_response(204);self.end_headers()
        server=ThreadingHTTPServer(('127.0.0.1',18894),Handler);server.daemon_threads=True
        threading.Thread(target=server.serve_forever,daemon=True).start()
        gui=spawn('qt',[str(WS/'devel/lib/autolabor_operator_gui/autolabor_operator_gui_node'),
            '__name:=autolabor_operator_gui','_static_map_mode:='+str(not mapping_mode).lower(),
            '_rviz_startup_fixed_frame:='+fixed_frame,'_rviz_navigation_fixed_frame:='+fixed_frame,
            '_static_map_set:='+str(map_dir),'_coverage_region_root:='+str(map_dir),
            '_navigation_mode_label:='+('MAPPING_REPLAY' if mapping_mode else 'RECORDED_REPLAY'),
            '_configured_vision_backend:=j6m_surround_bpu',
            '_vision_backend_switch_script:=','_rviz_config:='+str(rviz_path),
            '_window_title:='+state['name']+(' · 局部建图导航回放（未连接实车）' if mapping_mode else ' · 完整导航回放（未连接实车）'),
            '_tuning_screenshot:='+str(ROOT/'qt_replay.png')])
        # Give Qt time to register its ROS subscriptions before the latched map is played.
        stop.wait(2)
        def start_player():
            nonlocal latest,clock_s,trajectory
            with state_lock:
                latest=None;state['epoch']+=1;state['vision_frames']=0
                state['playback_state']='播放中';state['position_s']=0;state['error']=None
                state.update(slam_odometry_frames=0,slam_cloud_frames=0,history_cloud_points=0,local_costmap_frames=0)
            clock_s=start;trajectory=RosPath();trajectory.header.frame_id=fixed_frame;path_pub.publish(trajectory)
            if mapping is not None:mapping.start(state['epoch'])
            return spawn('player',['rosbag','play','--clock','--hz=50','--delay=0.03','--keep-alive','--quiet',
                '--bags']+[str(p) for p in bags]+['--topics']+sorted(topics)+['__name:=cleaning_bag_player'])
        player=start_player()
        controller=spawn('controls',[sys.executable,'-B',str(Path(__file__).resolve()),'--controls'])
        last_command=None;last_graph=0
        while not stop.wait(.2):
            if gui.poll() is not None:break
            if master.poll() is not None:raise RuntimeError('Replay ROS master exited')
            if mapping is not None:mapping.check()
            if player.poll() is not None and player.returncode:
                raise RuntimeError('Recorded player exited; see player.log')
            request=ROOT/'command.json'
            if request.exists():
                data=json.loads(request.read_text())
                if data.get('id')!=last_command:
                    commands.put(data['action']);last_command=data.get('id')
            try:action=commands.get_nowait()
            except queue.Empty:action=None
            if action=='stop':break
            elif action=='restart':
                finish(player);player=start_player()
            elif action in ('pause','resume'):
                try:
                    rospy.wait_for_service('/cleaning_bag_player/pause_playback',timeout=3)
                    answer=rospy.ServiceProxy('/cleaning_bag_player/pause_playback',SetBool)(action=='pause')
                    already = 'Bag is already paused.' if action=='pause' else 'Bag is already running.'
                    if not answer.success and answer.message != already:raise RuntimeError(answer.message)
                    with state_lock:
                        state['playback_state']='已暂停' if action=='pause' else '播放中'
                        state['error']=None
                except Exception as exc:
                    with state_lock:state['error']=str(exc)
            elif action=='video':
                playlist=ROOT/'qt_video.ffconcat'
                videos=sorted((RECORD/'visual').glob('take_*/qt_*.mkv'))
                playlist.write_text('ffconcat version 1.0\n'+''.join("file '%s'\n"%str(p).replace("'","'\\''") for p in videos))
                spawn('video',['ffplay','-autoexit','-window_title',state['name']+' · 原始Qt录像',
                    '-f','concat','-safe','0','-i',str(playlist)])
            if time.monotonic()-last_graph>2:
                graph=rosgraph.Master('/replay_state').getSystemState()
                nodes=sorted(set(n for group in graph for _,ns in group for n in ns))
                with state_lock:state['graph_nodes']=nodes
                last_graph=time.monotonic()
            with state_lock:
                state['updated_epoch_s']=time.time()
                atomic_json(ROOT/'state.json',state)
    finally:
        if server is not None:server.shutdown();server.server_close()
        for child in reversed(children):finish(child)
        for stream in streams:stream.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--controls',action='store_true')
    if parser.parse_args().controls:controls()
    else:main()
