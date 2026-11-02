#!/usr/bin/env python3
"""One click starts the loaded garage plan through the existing coverage service."""
import argparse
import json
from pathlib import Path
import signal
import threading
import time

import gi
gi.require_version('Gtk','3.0')
from gi.repository import Gtk,GLib
import rospy
from autolabor_coverage.msg import CoverageStatus
from autolabor_coverage.srv import StartCoverage,StartCoverageRequest


def request_from_plan(plan):
    request=StartCoverageRequest(plan_id=plan['plan_id'])
    for name,value in plan['execution_parameters'].items():setattr(request,name,value)
    return request


class CoverageButton(Gtk.Window):
    def __init__(self,workspace):
        super().__init__(title='地下车库 · 一键覆盖清扫')
        self.workspace=workspace;self.pending=False;self.status=None;self.last_result='等待路线加载完成'
        self.plan_file=workspace/'validation/garage_drivable_coverage_20261101/live_plan/plan.json'
        self.output=workspace/'validation/garage_coverage66_start_20261101/button_status.json'
        self.output.parent.mkdir(parents=True,exist_ok=True)
        self.set_default_size(470,210);self.set_keep_above(True);self.move(90,780)
        self.connect('destroy',lambda *_:Gtk.main_quit())
        box=Gtk.Box(orientation=Gtk.Orientation.VERTICAL,spacing=9);box.set_border_width(14);self.add(box)
        title=Gtk.Label();title.set_markup('<b>地下车库覆盖 · 66.9% · 清扫宽 1.2 m</b>');box.pack_start(title,False,False,0)
        self.record_label=Gtk.Label(label='正在读取录制状态');box.pack_start(self.record_label,False,False,0)
        self.button=Gtk.Button(label='一键开始覆盖清扫');self.button.set_size_request(430,58)
        self.button.get_style_context().add_class('suggested-action');self.button.connect('clicked',self.start)
        self.button.set_sensitive(False);box.pack_start(self.button,False,False,0)
        self.message=Gtk.Label(label=self.last_result);self.message.set_line_wrap(True);self.message.set_max_width_chars(54)
        box.pack_start(self.message,False,False,0)
        rospy.Subscriber('/coverage/status',CoverageStatus,self.receive,queue_size=1)
        GLib.timeout_add(500,self.refresh);self.show_all()

    def receive(self,status):self.status=status

    def refresh(self):
        status=self.status;plan=None
        try:plan=json.loads(self.plan_file.read_text())
        except (OSError,ValueError):pass
        matches=bool(status and plan and status.plan_id==plan.get('plan_id'))
        ready=bool(matches and status.state=='READY' and not status.active and not self.pending)
        self.button.set_sensitive(ready)
        self.button.set_label('正在提交启动…' if self.pending else ('覆盖任务运行中' if status and status.active else '一键开始覆盖清扫'))
        if self.pending:message='正在从当前位姿准备入场，请等待服务返回'
        elif status and status.active:message='任务已开始；进度、暂停和取消请使用主 Qt 清扫页'
        elif not matches:message='正在加载这版覆盖路线…'
        else:message=self.last_result if self.last_result!='等待路线加载完成' else '路线已就绪，点击上方按钮开始导航'
        self.message.set_text(message)
        recording={}
        try:
            current=(self.workspace/'runtime/localization_recording/current').resolve()
            recording=json.loads((current/'visual/current/status.json').read_text())
            age=time.time()-recording['updated_epoch_s']
            self.record_label.set_text(('● 正在录制：地下车库覆盖清扫1' if age<5 and recording.get('frame_count',0)>0 else '录制状态待更新'))
        except (OSError,ValueError,KeyError):self.record_label.set_text('录制状态待更新')
        state=dict(button_enabled=ready,pending=self.pending,coverage_state=status.state if status else None,
                   coverage_active=bool(status and status.active),plan_id=status.plan_id if status else '',
                   plan_matches=matches,message=message,last_result=self.last_result,updated_epoch_s=time.time())
        temp=self.output.with_suffix('.tmp');temp.write_text(json.dumps(state,ensure_ascii=False,indent=2));temp.replace(self.output)
        return True

    def start(self,_button):
        if self.pending or not self.button.get_sensitive():return
        plan=json.loads(self.plan_file.read_text());request=request_from_plan(plan)
        self.pending=True;self.refresh()
        def submit():
            try:
                response=rospy.ServiceProxy('/coverage/start',StartCoverage)(request)
                result=dict(accepted=response.accepted,message=response.message,plan_id=request.plan_id,time=time.time(),source='user clicked one-click button')
                self.last_result=('已开始：' if response.accepted else '未启动：')+response.message
            except Exception as error:
                result=dict(accepted=False,message=str(error),plan_id=request.plan_id,time=time.time(),source='user clicked one-click button')
                self.last_result='未启动：'+str(error)
            (self.output.parent/'button_start_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
            GLib.idle_add(self.finished)
        threading.Thread(target=submit,daemon=True).start()

    def finished(self):self.pending=False;self.refresh();return False


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--workspace',type=Path,default=Path(__file__).resolve().parents[1]);args=parser.parse_args()
    rospy.init_node('garage_coverage_one_click',anonymous=True,disable_signals=True)
    for signum in (signal.SIGINT,signal.SIGTERM):signal.signal(signum,lambda *_:GLib.idle_add(Gtk.main_quit))
    window=CoverageButton(args.workspace);Gtk.main();rospy.signal_shutdown('Coverage button closed')

if __name__=='__main__':main()
