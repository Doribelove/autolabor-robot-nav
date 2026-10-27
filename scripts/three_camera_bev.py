#!/usr/bin/env python3
"""Live GTK ground BEV from two hardware-GDC side cameras and existing ZED.

Manual installation geometry, independent camera acquisition times. Does not
publish ROS/TF or control a vehicle. Closing stops only its private J6M capture.
"""
import argparse
import base64
import fcntl
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import signal
import struct
import subprocess
import sys
import threading
import time
import urllib.request
import uuid
import zlib

import cv2
import gi
import numpy as np
gi.require_version('Gtk', '3.0')
gi.require_version('Gdk', '3.0')
from gi.repository import Gdk, GdkPixbuf, GLib, Gtk

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT/'distribution/j6m_bpu_navigation'))
from surround.ground_bev import GroundBev

SSH = ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
       '-o', 'ServerAliveInterval=3', '-o', 'ServerAliveCountMax=3']
BOARD = '/map/conbine_camera/bev_check_20261025'
STOP = threading.Event()
cv2.setNumThreads(1)


def atomic_json(path, value):
    tmp = path.with_name(path.name+'.tmp')
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    tmp.replace(path)


def atomic_png(path, image):
    tmp = path.with_name(path.stem+'.tmp.png')
    if not cv2.imwrite(str(tmp), image):
        raise RuntimeError('Could not write '+str(path))
    tmp.replace(path)


class Pane(Gtk.DrawingArea):
    def __init__(self, minimum_width, minimum_height):
        super().__init__()
        self.pixbuf = None
        self.set_size_request(minimum_width, minimum_height)
        self.set_hexpand(True)
        self.set_vexpand(True)
        self.connect('draw', self.draw_image)

    def set_image(self, bgr):
        if bgr is None:
            self.pixbuf = None
        else:
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            h, w = rgb.shape[:2]
            self.pixbuf = GdkPixbuf.Pixbuf.new_from_bytes(GLib.Bytes.new(rgb.tobytes()),
                GdkPixbuf.Colorspace.RGB, False, 8, w, h, w*3)
        self.queue_draw()

    def draw_image(self, _, cr):
        cr.set_source_rgb(.035, .045, .06)
        cr.paint()
        if self.pixbuf:
            w, h = self.get_allocated_width(), self.get_allocated_height()
            iw, ih = self.pixbuf.get_width(), self.pixbuf.get_height()
            scale = min(w/iw, h/ih)
            cr.translate((w-iw*scale)/2, (h-ih*scale)/2)
            cr.scale(scale, scale)
            Gdk.cairo_set_source_pixbuf(cr, self.pixbuf, 0, 0)
            cr.paint()


class Viewer:
    def __init__(self, config, assets):
        self.assets = assets
        self.output = assets/'bev'
        self.output.mkdir(exist_ok=True)
        self.bev = GroundBev(config)
        self.config = config
        self.latest, self.lock = {}, threading.Lock()
        self.errors = {}
        self.pending = False
        self.display_slot = None
        self.last_signature = None
        self.started = time.monotonic()
        self.first_live = None
        self.last_export = 0
        self.last_result = None
        self.count = 0
        self.fusion_sequence = 0
        self.fusion_session = uuid.uuid4().hex
        self.saved = False
        self.window = Gtk.Window(title='三相机地面 BEV · 左右 GDC + 前视 ZED')
        self.window.set_default_size(1320, 960)
        self.window.set_position(Gtk.WindowPosition.CENTER)
        self.window.connect('destroy', lambda *_: Gtk.main_quit())
        outer = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        outer.set_border_width(14)
        self.window.add(outer)
        heading = Gtk.Label(xalign=0)
        heading.set_markup('<span size="x-large" weight="bold">三相机地面 BEV · 安装尺寸估算预览</span>')
        outer.pack_start(heading, False, False, 0)
        body = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=16)
        outer.pack_start(body, True, True, 0)
        inputs = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        body.pack_start(inputs, True, True, 0)
        self.source_panes, self.source_labels = {}, {}
        for name, label in [('left', '左侧 · 端口 0 · GDC · 已旋转 180°'),
                             ('front', '前方 · ZED 左目校正图'),
                             ('right', '右侧 · 端口 1 · GDC · 已旋转 180°')]:
            inputs.pack_start(Gtk.Label(label=label, xalign=0), False, False, 0)
            pane = Pane(400, 145)
            inputs.pack_start(pane, True, True, 0)
            info = Gtk.Label(label='等待实时图像…', xalign=0)
            inputs.pack_start(info, False, False, 0)
            self.source_panes[name], self.source_labels[name] = pane, info
        result = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        body.pack_start(result, True, True, 0)
        result.pack_start(Gtk.Label(label='上方为车前方 · 左侧为车左侧 · 黑色为无图像区域'), False, False, 0)
        self.result_pane = Pane(600, 600)
        result.pack_start(self.result_pane, True, True, 0)
        controls = Gtk.Box(spacing=14)
        result.pack_start(controls, False, False, 0)
        self.mode = Gtk.ComboBoxText()
        for key, text in [('fusion', '三路融合'), ('left', '只看左侧投影'),
                           ('front', '只看 ZED 投影'), ('right', '只看右侧投影'),
                           ('support', '图像来源区域')]:
            self.mode.append(key, text)
        self.mode.set_active_id('fusion')
        self.mode.connect('changed', lambda *_: self.redraw())
        controls.pack_start(self.mode, False, False, 0)
        self.grid = Gtk.CheckButton(label='显示 1 m 网格')
        self.grid.set_active(True)
        self.grid.connect('toggled', lambda *_: self.redraw())
        controls.pack_start(self.grid, False, False, 0)
        result.pack_start(Gtk.Label(label='范围：前 4 m / 后 2 m / 左右各 3 m · 1 cm/像素'), False, False, 0)
        result.pack_start(Gtk.Label(label='来源颜色：左侧青色 / 前方绿色 / 右侧橙色'), False, False, 0)
        result.pack_start(Gtk.Label(label='高度 0.71 m · 侧视下俯 15° · ZED 下俯约 %.1f°（静态 IMU 估计）' %
                                   config['cameras']['front']['pitch_down_deg']), False, False, 0)
        self.status = Gtk.Label(label='正在接收三路图像…', xalign=0)
        outer.pack_start(self.status, False, False, 0)
        note = Gtk.Label(label='地面平面假设；高出地面的物体会拉伸。外参为安装估算，三路曝光尚未同步，尺寸精度待实测。', xalign=0)
        note.set_line_wrap(True)
        outer.pack_start(note, False, False, 0)
        self.window.show_all()
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass

            def do_GET(self):
                with owner.lock:
                    pair = owner.display_slot
                if (self.path != '/frame' or pair is None or any(
                        time.monotonic()-s['received_monotonic'] > config['max_local_age_s']
                        for s in pair[4]['sources'].values())):
                    self.send_error(503, 'Waiting for fresh ground BEV')
                    return
                ok, jpeg = cv2.imencode('.jpg', pair[1], [cv2.IMWRITE_JPEG_QUALITY, 94])
                if not ok:
                    self.send_error(500)
                    return
                meta = dict(pair[4], image_width=owner.bev.width, image_height=owner.bev.height,
                            encoding='jpeg', protocol='conbine.ground_bev.frame.v1')
                header = json.dumps(meta, allow_nan=False).encode()
                payload = struct.pack('!4sII', b'BG01', len(header), len(jpeg))+header+jpeg.tobytes()
                self.send_response(200)
                self.send_header('Content-Type', 'application/octet-stream')
                self.send_header('Content-Length', str(len(payload)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                try:
                    self.wfile.write(payload)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self.preview_server = ThreadingHTTPServer(('127.0.0.1', 18883), Handler)
        self.preview_server.daemon_threads = True
        threading.Thread(target=self.preview_server.serve_forever, daemon=True).start()
        for name in config['cameras']:
            threading.Thread(target=self.receive, args=(name,), daemon=True).start()
        threading.Thread(target=self.compose, daemon=True).start()
        GLib.timeout_add(250, self.check_freshness)

    def fetch(self, name):
        camera = self.config['cameras'][name]
        if camera['source'] == 'j6m_hardware_gdc':
            with urllib.request.urlopen('http://127.0.0.1:18882/gdc/%d'%camera['rx1_port'], timeout=2) as response:
                data = response.read()
            n = struct.unpack('!I', data[:4])[0]
            meta = json.loads(data[4:4+n])
            gm, rm = meta['gdc'], meta['raw']
            if (meta['camera_port'] != camera['rx1_port'] or gm['gdc_applied'] != 1 or
                    meta.get('payload_layout') != 'gdc_only' or len(data)-4-n != gm['bytes'] or
                    gm['sequence'] != rm['sequence'] or gm['raw_stamp'] != rm['raw_stamp'] or
                    meta['age_s'] > self.config['max_local_age_s']):
                raise ValueError('Invalid or stale GDC perspective source')
            nv12 = np.frombuffer(data[4+n:], np.uint8).reshape(gm['height']*3//2, gm['width'])
            image = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
            source = dict(camera=name, port=camera['rx1_port'], sequence=gm['sequence'],
                          raw_stamp=gm['raw_stamp'], hal_received_ns=gm['hal_received_ns'],
                          clock_domain='J6M vendor camera counter; epoch alignment unverified',
                          gdc_applied=True, producer_cache_age_s=meta['age_s'])
        else:
            # Read the existing RGB preview; do not reopen ZED or alter its sender.
            with urllib.request.urlopen(self.zed_request, timeout=2) as response:
                data = response.read()
            magic, n, size = struct.unpack('!4sII', data[:12])
            if magic != b'ZP01' or len(data) != 12+n+size:
                raise ValueError('Invalid ZED preview envelope')
            meta = json.loads(data[12:12+n])['image']
            age = (time.time_ns()-meta['source_stamp_ns'])/1e9
            if not -.1 <= age <= self.config['max_local_age_s']:
                raise ValueError('ZED image is stale')
            image = cv2.imdecode(np.frombuffer(data[12+n:], np.uint8), cv2.IMREAD_COLOR)
            source = dict(camera=name, session=meta['session'], sequence=meta['sequence'],
                          source_stamp_ns=meta['source_stamp_ns'],
                          clock_domain='ZED SDK UNIX nanoseconds', sdk_rectified=True, source_age_s=age)
        image = self.bev.upright(name, image)
        source.update(received_monotonic=time.monotonic(), received_epoch_ns=time.time_ns(),
                      rotate_180_applied=camera['rotate_180'])
        return image, source

    def receive(self, name):
        if name == 'front':
            config = json.loads(Path('/home/slam/j6m-dev-b/runtime/zed_link_20261019_6y_rm3_5/config.json').read_text())
            self.zed_request = urllib.request.Request('http://192.168.10.100:17446/frame',
                                                       headers={'Authorization': 'Bearer '+config['token']})
        previous = None
        last_error = 0
        while not STOP.is_set():
            began = time.monotonic()
            try:
                image, source = self.fetch(name)
                identity = (source.get('session'), source['sequence'], source.get('raw_stamp'))
                if identity != previous:
                    with self.lock:
                        self.latest[name] = (image, source)
                        self.errors.pop(name, None)
                    previous = identity
            except Exception as exc:
                with self.lock:
                    self.errors[name] = str(exc)
                if began-last_error > 5:
                    print('SOURCE_WAIT', name, str(exc), flush=True)
                    last_error = began
            STOP.wait(max(0, 1/self.config['preview_hz']-(time.monotonic()-began)))

    def compose(self):
        while not STOP.is_set():
            begin = time.monotonic()
            with self.lock:
                frames = dict(self.latest)
            if set(frames) == set(self.config['cameras']) and all(begin-s['received_monotonic'] < self.config['max_local_age_s'] for _, s in frames.values()):
                signature = tuple((n, f[1]['sequence']) for n, f in sorted(frames.items()))
                if signature != self.last_signature:
                    try:
                        images = {n: f[0] for n, f in frames.items()}
                        image, counts, layers = self.bev.compose(images)
                        self.fusion_sequence += 1
                        metadata = dict(frame_id=self.config['frame_id'], config_sha256=self.bev.config_sha256,
                            fusion_session=self.fusion_session, fusion_sequence=self.fusion_sequence,
                            produced_epoch_s=time.time(),
                            at_epoch_ns=time.time_ns(), source_selection='latest locally received frames; unsynchronized preview',
                            source_exposure_synchronized=False, metric_accuracy_verified=False,
                            actual_visible_ground_coverage_verified=False,
                            pose_source=self.config['pose_source'], sources={n: f[1] for n, f in frames.items()},
                            source_receive_span_ms=(max(s['received_monotonic'] for _, s in frames.values())-
                                                   min(s['received_monotonic'] for _, s in frames.values()))*1000,
                            x_range_m=self.config['x_range_m'], y_range_m=self.config['y_range_m'],
                            metres_per_pixel=self.bev.resolution, ground_plane=[0, 0, 1, 0],
                            dimensions=[self.bev.width, self.bev.height],
                            ground_support_zlib_b64=base64.b64encode(zlib.compress(
                                (counts > 0).astype('uint8').tobytes())).decode('ascii'),
                            T_ground_from_upright_optical={n: t.tolist() for n, t in self.bev.transforms.items()},
                            projection_intrinsics={n: k.tolist() for n, k in self.bev.intrinsics.items()},
                            vehicle_image_masks='manually bounded visible vehicle structure; not ground segmentation',
                            geometric_projection_support_fraction=float((counts > 0).mean()),
                            geometric_overlap_fraction=float((counts > 1).mean()),
                            processing_ms=(time.monotonic()-begin)*1000)
                        result = (images, image, counts, layers, metadata)
                        with self.lock:
                            self.display_slot = result
                            if not self.pending:
                                self.pending = True
                                GLib.idle_add(self.display)
                        if begin-self.last_export >= 2:
                            atomic_png(self.output/'bev.png', image)
                            atomic_png(self.output/'projection_support.png', counts*85)
                            metadata['bev_png_sha256'] = hashlib.sha256((self.output/'bev.png').read_bytes()).hexdigest()
                            metadata['projection_support_png_sha256'] = hashlib.sha256((self.output/'projection_support.png').read_bytes()).hexdigest()
                            for name, frame in images.items():
                                atomic_png(self.output/(name+'_upright.png'), frame)
                            atomic_json(self.output/'bev.json', metadata)
                            self.last_export = begin
                        self.last_signature = signature
                    except Exception as exc:
                        print('COMPOSE_ERROR', str(exc), flush=True)
            STOP.wait(max(0, 1/self.config['preview_hz']-(time.monotonic()-begin)))

    def display(self):
        with self.lock:
            result = self.display_slot
            self.pending = False
        images, _, _, _, meta = result
        now = time.monotonic()
        self.last_result = result
        self.count += 1
        if self.first_live is None:
            self.first_live = now
        for name, image in images.items():
            self.source_panes[name].set_image(image)
            s = meta['sources'][name]
            self.source_labels[name].set_text('源帧 %d · 本地接收距今 %.0f ms' %
                (s['sequence'], (now-s['received_monotonic'])*1000))
        fps = (self.count-1)/max(.001, now-self.first_live)
        self.status.set_text('三路实时地面投影 · %.1f fps · 计算 %.1f ms · 曝光未同步 · 精度待实测' % (fps, meta['processing_ms']))
        self.redraw()
        if not self.saved and now-self.first_live > 5:
            self.saved = True
            GLib.timeout_add(200, self.screenshot)
        return False

    def redraw(self):
        if self.last_result is None:
            return
        _, image, counts, layers, _ = self.last_result
        mode = self.mode.get_active_id()
        if mode == 'support':
            view = np.zeros_like(image, dtype='float32')
            for name, color in [('left', [230, 190, 30]), ('front', [70, 210, 70]), ('right', [20, 150, 240])]:
                view += (self.bev.maps[name][2] > 0)[..., None]*np.array(color)
            view = (view/np.maximum(counts[..., None], 1)).astype('uint8')
        else:
            view = (image if mode == 'fusion' else layers[mode]).copy()
        if self.grid.get_active():
            grid = view.copy()
            for x in np.arange(*self.config['x_range_m'], 1):
                _, row = self.bev.pixel(x, 0)
                cv2.line(grid, (0, int(round(row))), (self.bev.width-1, int(round(row))), (185, 185, 185), 1)
            for y in np.arange(*self.config['y_range_m'], 1):
                col, _ = self.bev.pixel(0, y)
                cv2.line(grid, (int(round(col)), 0), (int(round(col)), self.bev.height-1), (185, 185, 185), 1)
            view = cv2.addWeighted(view, .78, grid, .22, 0)
            view[counts == 0] = 0
        center = tuple(int(round(v)) for v in self.bev.pixel(0, 0))
        cv2.circle(view, center, 5, (0, 235, 255), -1)
        cv2.arrowedLine(view, center, (center[0], center[1]-32), (0, 235, 255), 2)
        cv2.putText(view, 'rig origin', (center[0]+9, center[1]+15), cv2.FONT_HERSHEY_SIMPLEX, .38, (0, 235, 255), 1)
        cv2.putText(view, 'FRONT +X', (self.bev.width//2-45, 20), cv2.FONT_HERSHEY_SIMPLEX, .5, (240, 240, 240), 1)
        self.result_pane.set_image(view)

    def check_freshness(self):
        now = time.monotonic()
        with self.lock:
            latest = dict(self.latest)
        stale = [name for name in self.config['cameras'] if name not in latest or
                 now-latest[name][1]['received_monotonic'] > self.config['max_local_age_s']]
        if stale:
            self.result_pane.set_image(None)
            self.last_result = None
            for name in stale:
                self.source_panes[name].set_image(None)
                self.source_labels[name].set_text('等待新图像…')
            self.status.set_text('等待三路新图像：'+', '.join(stale))
        return not STOP.is_set()

    def screenshot(self):
        window = self.window.get_window()
        pixbuf = Gdk.pixbuf_get_from_window(window, 0, 0, self.window.get_allocated_width(), self.window.get_allocated_height())
        pixbuf.savev(str(self.output/'live_window.png'), 'png', [], [])
        print('LIVE_THREE_CAMERA_BEV_VERIFIED', flush=True)
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default=str(PROJECT/'config/three_camera_bev.json'))
    parser.add_argument('--assets', default=str(PROJECT/'runtime/camera_bev_check_20261025'))
    args = parser.parse_args()
    assets = Path(args.assets).resolve()
    config = json.loads(Path(args.config).read_text())
    print('START_THREE_CAMERA_BEV', time.strftime('%Y-%m-%d %H:%M:%S'),
          'ZED_pitch_deg=', config['cameras']['front']['pitch_down_deg'], flush=True)
    lock = open(assets/'bev_viewer.lock', 'a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    log = open(assets/'bev_board.log', 'ab', buffering=0)
    proc = subprocess.Popen(SSH+['-o', 'ExitOnForwardFailure=yes', '-L', '18882:127.0.0.1:18882',
                                'root@192.168.10.100', 'python3 -u '+BOARD+'/server.py'], stdout=log, stderr=log)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: GLib.idle_add(Gtk.main_quit))
    try:
        Viewer(config, assets)
        Gtk.main()
    finally:
        STOP.set()
        subprocess.run(SSH+['root@192.168.10.100', 'python3 '+BOARD+'/stop.py'], timeout=10)
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            proc.terminate()
            proc.wait(timeout=5)
        log.close()


if __name__ == '__main__':
    main()
