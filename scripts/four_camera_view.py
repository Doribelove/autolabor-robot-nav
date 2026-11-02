#!/usr/bin/env python3
"""Four actual camera views, independent of BEV calibration and BPU results."""
import argparse
import ctypes as C
import fcntl
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import threading
import time
import urllib.request
import uuid

import cv2
import gi
import numpy as np
gi.require_version('Gtk', '3.0')
gi.require_version('Gdk', '3.0')
from gi.repository import Gdk, GLib, Gtk
from three_camera_bev import Pane, atomic_json

PROJECT = Path(__file__).resolve().parents[1]
ROOT = Path(os.environ.get('CAMERA_VIEW_ASSETS', str(PROJECT/'runtime/four_camera_view_20261101')))
BOARD = os.environ.get('CAMERA_BEV_BOARD_ROOT', '/map/conbine_camera/surround_recalibrated_20261101')
SENDER = 'j6m-b-camera-sender.service'
SSH = ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
       '-o', 'ServerAliveInterval=3', '-o', 'ServerAliveCountMax=3']
STOP = threading.Event()
cv2.setNumThreads(1)


class Viewer:
    def __init__(self, headless=False):
        self.headless = headless
        self.window = None
        self.panes, self.labels = {}, {}
        self.lock = threading.Lock()
        self.latest, self.errors = {}, {}
        self.sequences = {n: 0 for n in ('front', 'left', 'right', 'rear')}
        self.rotate_rear = False
        self.saved = False
        self.started = time.monotonic()
        self.session = uuid.uuid4().hex
        if not headless:
            self.window = Gtk.Window(title='四路相机实时画面 · 前视 ZED / 左 / 右 / 后视')
            screen = Gdk.Screen.get_default()
            self.window.set_default_size(min(1600, screen.get_width()-60), min(960, screen.get_height()-80))
            self.window.set_position(Gtk.WindowPosition.CENTER)
            self.window.connect('destroy', lambda *_: Gtk.main_quit())
            outer = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
            outer.set_border_width(12)
            self.window.add(outer)
            title = Gtk.Label(xalign=0)
            title.set_markup('<span size="x-large" weight="bold">四路相机实时画面 · 核对方向与视野</span>')
            outer.pack_start(title, False, False, 0)
            grid = Gtk.Grid(column_spacing=12, row_spacing=12, column_homogeneous=True, row_homogeneous=True)
            outer.pack_start(grid, True, True, 0)
            self.panes, self.labels = {}, {}
            names = [('front', '前视 · ZED 左目校正图 · 本机直连'),
                     ('left', '左视 · J6M RX1 端口 0 · GDC · 已翻正'),
                     ('right', '右视 · J6M RX1 端口 1 · GDC · 已翻正'),
                     ('rear', '后视 · J6M RX1 端口 2 · GDC')]
            for index, (name, title) in enumerate(names):
                box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=5)
                box.pack_start(Gtk.Label(label=title, xalign=0), False, False, 0)
                pane = Pane(480, 230)
                box.pack_start(pane, True, True, 0)
                label = Gtk.Label(label='正在打开相机…', xalign=0)
                label.set_line_wrap(True)
                box.pack_start(label, False, False, 0)
                grid.attach(box, index % 2, index // 2, 1, 1)
                self.panes[name], self.labels[name] = pane, label
            controls = Gtk.Box(spacing=12)
            rotate = Gtk.CheckButton(label='后视画面旋转 180°（仅影响本窗口显示）')
            rotate.connect('toggled', lambda button: setattr(self, 'rotate_rear', button.get_active()))
            controls.pack_start(rotate, False, False, 0)
            outer.pack_start(controls, False, False, 0)
            outer.pack_start(Gtk.Label(label='各路独立实时显示，可核对安装方向与视野。关闭窗口后恢复原 ZED 发送服务。', xalign=0), False, False, 0)
            self.window.show_all()
        owner = self

        class FrontHandler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass

            def do_GET(self):
                with owner.lock:
                    frame = owner.latest.get('front')
                if self.path != '/frame':
                    self.send_error(404,'RGB preview only'); return
                if frame is None or time.monotonic()-frame[1]['received_monotonic'] > 1.5:
                    self.send_error(503, 'Waiting for fresh ZED image')
                    return
                image, source = frame
                meta_image = dict(session=owner.session, sequence=source['sequence'],
                        source_stamp_ns=source['sdk_stamp_ns'], serial=23748636,
                        width=1280, height=720, encoding='jpeg_display_copy',
                        K_rectified=source['K_rectified'],depth_mode=source['depth_mode'],depth_enabled=source['depth_enabled'])
                ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 92])
                if not ok:
                    self.send_error(500, 'JPEG encoding failed'); return
                meta = dict(protocol='conbine.local_zed.preview.v1', backend='local_ZED_SDK',image=meta_image)
                header = json.dumps(meta).encode()
                payload = struct.pack('!4sII', b'ZP01', len(header), len(jpeg))+header+jpeg.tobytes()
                self.send_response(200)
                self.send_header('Content-Type', 'application/octet-stream')
                self.send_header('Content-Length', str(len(payload)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                try:
                    self.wfile.write(payload)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self.front_server = ThreadingHTTPServer(('127.0.0.1', 18885), FrontHandler)
        self.front_server.daemon_threads = True
        threading.Thread(target=self.front_server.serve_forever, daemon=True).start()
        self.workers = []
        for port, name in ((0, 'left'), (1, 'right'), (2, 'rear')):
            worker = threading.Thread(target=self.receive_gdc, args=(name, port), daemon=True)
            self.workers.append(worker); worker.start()
        worker = threading.Thread(target=self.receive_zed, daemon=True)
        self.workers.append(worker); worker.start()
        GLib.timeout_add(200, self.refresh)

    def publish(self, name, image, source):
        with self.lock:
            source = dict(source, received_monotonic=time.monotonic())
            self.latest[name] = (image, source)
            self.sequences[name] += 1
            self.errors.pop(name, None)

    def receive_gdc(self, name, port):
        previous = None
        while not STOP.is_set():
            begin = time.monotonic()
            try:
                with urllib.request.urlopen('http://127.0.0.1:18882/gdc/%d'%port, timeout=1) as response:
                    data = response.read(1024*1024)
                if len(data) < 4: raise ValueError('相机图像包不完整')
                n = struct.unpack('!I', data[:4])[0]
                meta = json.loads(data[4:4+n]); gm, rm = meta['gdc'], meta['raw']
                if (meta['camera_port'] != port or gm['gdc_applied'] != 1 or
                        (gm['sequence'], gm['raw_stamp']) != (rm['sequence'], rm['raw_stamp']) or
                        meta['payload_layout'] != 'gdc_only' or len(data)-4-n != 777600 or
                        (gm['width'], gm['height'], gm['bytes']) != (960, 540, 777600) or
                        not 0 <= meta['age_s'] < 1.5):
                    raise ValueError('GDC图像或同源信息无效')
                if gm['sequence'] != previous:
                    image = cv2.cvtColor(np.frombuffer(data[4+n:], np.uint8).reshape(810, 960), cv2.COLOR_YUV2BGR_NV12)
                    if name in ('left', 'right'): image = cv2.rotate(image, cv2.ROTATE_180)
                    self.publish(name, image, dict(port=port, sequence=gm['sequence'],
                        raw_stamp=gm['raw_stamp'], hal_received_ns=gm['hal_received_ns'],
                        source='J6M hardware GDC', gdc_applied=True))
                    previous = gm['sequence']
            except Exception as exc:
                with self.lock: self.errors[name] = str(exc)
            STOP.wait(max(0, .2-(time.monotonic()-begin)))

    def receive_zed(self):
        handle = None
        lib = C.CDLL(str(ROOT/'libzed_preview.so'))
        lib.zp_error.restype = C.c_char_p
        lib.zp_open.argtypes = [C.c_uint, C.c_char_p]; lib.zp_open.restype = C.c_void_p
        lib.zp_read.argtypes = [C.c_void_p, C.c_void_p, C.c_size_t, C.POINTER(C.c_uint64)]
        lib.zp_depth_enabled.argtypes = [C.c_void_p]
        lib.zp_intrinsics.argtypes = [C.c_void_p,C.POINTER(C.c_double)]
        lib.zp_close.argtypes = [C.c_void_p]
        try:
            handle = lib.zp_open(23748636, b'/home/slam/j6m-dev-b/runtime/zed_link_20261019_6y_rm3_5/settings')
            if not handle: raise RuntimeError(lib.zp_error().decode())
            buf = (C.c_uint8*(1280*720*4))()
            depth_enabled = lib.zp_depth_enabled(handle)
            if depth_enabled != 0: raise RuntimeError('Expected ZED RGB-only mode')
            intrinsic = (C.c_double*4)()
            if lib.zp_intrinsics(handle,intrinsic) < 0: raise RuntimeError('ZED intrinsics unavailable')
            fx,fy,cx,cy = intrinsic
            k = [[fx,0,cx],[0,fy,cy],[0,0,1]]
            while not STOP.is_set():
                begin = time.monotonic(); stamp = C.c_uint64()
                if lib.zp_read(handle,buf,len(buf),C.byref(stamp)) < 0:
                    raise RuntimeError(lib.zp_error().decode())
                image = cv2.cvtColor(np.frombuffer(buf, np.uint8).reshape(720, 1280, 4), cv2.COLOR_BGRA2BGR)
                self.publish('front', image, dict(serial=23748636, sdk_stamp_ns=stamp.value,
                    source='local ZED SDK rectified left RGB only', sequence=self.sequences['front']+1,
                    K_rectified=k,depth_mode='NONE',depth_enabled=bool(depth_enabled)))
                STOP.wait(max(0, .2-(time.monotonic()-begin)))
        except Exception as exc:
            with self.lock: self.errors['front'] = str(exc)
            print('ZED_PREVIEW_ERROR', str(exc), flush=True)
        finally:
            if handle: lib.zp_close(handle)

    def refresh(self):
        now = time.monotonic()
        with self.lock: latest, errors, counts = dict(self.latest), dict(self.errors), dict(self.sequences)
        state = {}
        for name in self.sequences:
            frame = latest.get(name)
            if frame and now-frame[1]['received_monotonic'] < 1.5:
                image, meta = frame
                if name == 'rear' and self.rotate_rear: image = cv2.rotate(image, cv2.ROTATE_180)
                if not self.headless:
                    self.panes[name].set_image(image)
                    self.labels[name].set_text('%d × %d · 源帧 %d · 已显示 %d 帧' %
                        (image.shape[1], image.shape[0], meta['sequence'], counts[name]))
                state[name] = dict(meta, live=True, displayed_frames=counts[name], rear_display_rotated=self.rotate_rear if name=='rear' else False)
            else:
                if not self.headless:
                    self.panes[name].set_image(None)
                    self.labels[name].set_text(errors.get(name, '等待新相机画面…'))
                state[name] = dict(live=False, error=errors.get(name, 'waiting'))
        atomic_json(ROOT/'state.json', dict(at_ns=time.time_ns(), cameras=state, preview_visible=not self.headless))
        if not self.saved and now-self.started > 8 and all(s['live'] for s in state.values()):
            self.saved = True
            for name, (image, _) in latest.items(): cv2.imwrite(str(ROOT/(name+'.png')), image)
            if not self.headless:
                win = self.window.get_window()
                shot = Gdk.pixbuf_get_from_window(win, 0, 0, self.window.get_allocated_width(), self.window.get_allocated_height())
                shot.savev(str(ROOT/'four_live_views.png'), 'png', [], [])
            print('FOUR_LIVE_CAMERA_VIEWS', counts, flush=True)
        return not STOP.is_set()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--headless', action='store_true', help='Capture for Qt without a separate preview window')
    args = parser.parse_args()
    loop = GLib.MainLoop() if args.headless else None
    quit_loop = loop.quit if loop is not None else Gtk.main_quit
    ROOT.mkdir(parents=True, exist_ok=True)
    lock = (ROOT/'viewer.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    was_active = subprocess.run(['systemctl', '--user', 'is-active', '--quiet', SENDER]).returncode == 0
    atomic_json(ROOT/'restore_state.json', dict(sender_was_active=was_active, viewer_pid=os.getpid()))
    zed_lock = None; remote = None; viewer = None
    log = (ROOT/'board.log').open('ab', buffering=0)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: GLib.idle_add(quit_loop))
    try:
        if was_active: subprocess.run(['systemctl', '--user', 'stop', SENDER], check=True, timeout=20)
        zed_lock = open('/home/slam/j6m-dev-b/runtime/zed_link_20261019_6y_rm3_5/state/camera.lock', 'r')
        fcntl.flock(zed_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        remote = subprocess.Popen(SSH+['-o', 'ExitOnForwardFailure=yes',
            '-L', '127.0.0.1:18882:127.0.0.1:18882', 'root@192.168.10.100',
            'python3 -B -u '+BOARD+'/server.py'], stdout=log, stderr=log)
        viewer = Viewer(headless=args.headless)
        if loop is not None:
            loop.run()
        else:
            Gtk.main()
    finally:
        STOP.set()
        if viewer:
            viewer.front_server.shutdown()
            viewer.front_server.server_close()
            for worker in viewer.workers: worker.join(12)
        try:
            if remote:
                try:
                    subprocess.run(SSH+['root@192.168.10.100', 'python3 -B '+BOARD+'/stop.py'], timeout=10)
                finally:
                    try: remote.wait(timeout=10)
                    except subprocess.TimeoutExpired: remote.terminate(); remote.wait(timeout=5)
        finally:
            if zed_lock: zed_lock.close()
            if was_active: subprocess.run(['systemctl', '--user', 'start', SENDER], check=True, timeout=20)
            log.close()
            atomic_json(ROOT/'restore_state.json', dict(sender_was_active=was_active, closed=True))
            print('FOUR_CAMERA_PREVIEW_CLOSED', flush=True)


if __name__ == '__main__':
    main()
