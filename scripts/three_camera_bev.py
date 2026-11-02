#!/usr/bin/env python3
"""Live GTK ground BEV from configured hardware-GDC cameras and existing ZED.

Manual installation geometry, independent camera acquisition times. Does not
publish ROS/TF or control a vehicle. Closing stops only its private J6M capture.
"""
import argparse
import base64
import fcntl
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
from surround.preview_io import SnapshotWriter
from surround.ground_protocol import encode_view_frame

SSH = ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
       '-o', 'ServerAliveInterval=3', '-o', 'ServerAliveCountMax=3']
BOARD = '/map/conbine_camera/bev_check_20261025'
STOP = threading.Event()
cv2.setNumThreads(1)


def atomic_json(path, value):
    tmp = path.with_name(path.name+'.tmp')
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
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
    def __init__(self, config, assets, bev=None, headless=False):
        self.headless = headless
        self.window = None
        self.assets = assets
        self.output = assets/'bev'
        self.output.mkdir(exist_ok=True)
        self.bev = bev or GroundBev(config)
        config = self.bev.config
        self.config = config
        self.rig_label = '四路环视' if 'rear' in config['cameras'] else '三路 BEV'
        self.camera_labels = {'front': '前方 · ZED 左目校正图',
                              'left': '左侧', 'right': '右侧', 'rear': '后方'}
        self.latest, self.lock = {}, threading.Lock()
        self.preview_packet = None
        self.views_snapshot = None
        self.views_cache = None
        self.views_encode_lock = threading.Lock()
        self.exporter = SnapshotWriter(self.output)
        counts = self.bev.support_counts
        self.support_metadata = dict(
            blend_mode=self.bev.blend_mode,
            visual_camera_selection=('maximum minimum ground sampling density; static calibration-based ownership'
                                     if self.bev.blend_mode == 'single_source' else 'weighted incidence and edge feathering'),
            source_owner_applied=self.bev.blend_mode == 'single_source',
            source_owner_names=self.bev.source_names,
            source_owner_zlib_b64=base64.b64encode(zlib.compress(self.bev.source_owner.tobytes())).decode('ascii'),
            ground_support_zlib_b64=base64.b64encode(zlib.compress(
                (counts > 0).astype('uint8').tobytes())).decode('ascii'),
            geometric_projection_support_fraction=float((counts > 0).mean()),
            geometric_overlap_fraction=float((counts > 1).mean()))
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
        if not headless:
            self.window = Gtk.Window(title=self.rig_label+' · GDC + 前视 ZED')
            self.window.set_default_size(1320, 960)
            self.window.set_position(Gtk.WindowPosition.CENTER)
            self.window.connect('destroy', lambda *_: Gtk.main_quit())
            outer = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
            outer.set_border_width(14)
            self.window.add(outer)
            heading = Gtk.Label(xalign=0)
            heading.set_markup('<span size="x-large" weight="bold">'+self.rig_label+' · 安装尺寸估算预览</span>')
            outer.pack_start(heading, False, False, 0)
            body = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=16)
            outer.pack_start(body, True, True, 0)
            inputs = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
            body.pack_start(inputs, True, True, 0)
            self.source_panes, self.source_labels = {}, {}
            for name in ('left', 'front', 'right', 'rear'):
                if name not in config['cameras']:
                    continue
                camera = config['cameras'][name]
                label = self.camera_labels[name]
                if camera['source'] == 'j6m_hardware_gdc':
                    label += ' · 端口 %d · GDC%s' % (camera['rx1_port'],
                        ' · 已旋转 180°' if camera['rotate_180'] else '')
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
            modes = [('fusion', self.rig_label+'融合')]
            modes += [(name, '只看'+self.camera_labels[name].split(' · ')[0]+'投影')
                      for name in ('left', 'front', 'right', 'rear') if name in config['cameras']]
            modes.append(('support', '图像来源区域'))
            for key, text in modes:
                self.mode.append(key, text)
            self.mode.set_active_id('fusion')
            self.mode.connect('changed', lambda *_: self.redraw())
            controls.pack_start(self.mode, False, False, 0)
            self.grid = Gtk.CheckButton(label='显示 1 m 网格')
            self.grid.set_active(True)
            self.grid.connect('toggled', lambda *_: self.redraw())
            controls.pack_start(self.grid, False, False, 0)
            xmin, xmax = self.config['x_range_m']
            ymin, ymax = self.config['y_range_m']
            region = self.config.get('observation_region')
            if region:
                margins = region['distance_from_edges_m']
                range_label = ('距车体边缘：前 {front:g} m / 后 {rear:g} m / '
                               '左 {left:g} m / 右 {right:g} m').format(**margins)
            else:
                range_label = f'范围：前 {xmax:g} m / 后 {-xmin:g} m / 左 {ymax:g} m / 右 {-ymin:g} m'
            result.pack_start(Gtk.Label(label=range_label+
                f' · {100*self.bev.resolution:.2f} cm/像素'), False, False, 0)
            result.pack_start(Gtk.Label(label='来源颜色：左侧青色 / 前方绿色 / 右侧橙色'+
                                       (' / 后方紫色' if 'rear' in config['cameras'] else '')), False, False, 0)
            geometry_label = ' · '.join(
                '%s高 %.2f m / 下俯 %.1f°' % (self.camera_labels[name].split(' · ')[0],
                    camera['position_ground_rig_m'][2], camera['pitch_down_deg'])
                for name, camera in config['cameras'].items())
            result.pack_start(Gtk.Label(label=geometry_label), False, False, 0)
            self.status = Gtk.Label(label='正在接收'+self.rig_label+'图像…', xalign=0)
            outer.pack_start(self.status, False, False, 0)
            projection_note = '四路均使用彩色图地面投影，不使用深度；高出地面的物体会拉伸。'
            note = Gtk.Label(label=projection_note+'外参为安装估算，各路曝光尚未同步，尺寸精度待实测。', xalign=0)
            note.set_line_wrap(True)
            outer.pack_start(note, False, False, 0)
            self.window.show_all()
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass

            def do_GET(self):
                with owner.lock:
                    packet = owner.preview_packet
                    snapshot = owner.views_snapshot
                if (self.path not in ('/frame', '/views') or packet is None or any(
                        time.monotonic()-s['received_monotonic'] > config['max_local_age_s']
                        for s in packet[0]['sources'].values())):
                    self.send_error(503, 'Waiting for fresh ground BEV')
                    return
                payload = packet[1]
                if self.path == '/views':
                    if snapshot is None:
                        self.send_error(503, 'Waiting for camera views')
                        return
                    # Cache one atomic set. Requests never pair fresh camera
                    # images with an older BEV result from a separate endpoint.
                    with owner.views_encode_lock:
                        if owner.views_cache is None or owner.views_cache[0] != snapshot[0]:
                            owner.views_cache = (snapshot[0], encode_view_frame(snapshot[1], snapshot[2]))
                        payload = owner.views_cache[1]
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
        if not headless:
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
            image = cv2.imdecode(np.frombuffer(data[12+n:], np.uint8), cv2.IMREAD_COLOR)
            age = (time.time_ns()-meta['source_stamp_ns'])/1e9
            if not -.1 <= age <= self.config['max_local_age_s']:
                raise ValueError('ZED image is stale')
            source = dict(camera=name, session=meta['session'], sequence=meta['sequence'],
                          source_stamp_ns=meta['source_stamp_ns'],
                          clock_domain='ZED SDK UNIX nanoseconds', sdk_rectified=True, source_age_s=age)
            if 'depth_enabled' in meta:
                source.update(depth_mode=meta['depth_mode'],depth_enabled=meta['depth_enabled'])
        image = self.bev.upright(name, image)
        source.update(received_monotonic=time.monotonic(), received_epoch_ns=time.time_ns(),
                      rotate_180_applied=camera['rotate_180'])
        return image, source

    def receive(self, name):
        if name == 'front':
            if self.config.get('front_preview_url'):
                # Reuse the already-open local SDK camera from the four-view window.
                if self.config['front_preview_url'] != 'http://127.0.0.1:18885/frame':
                    raise ValueError('Expected local ZED preview endpoint')
                self.zed_request = urllib.request.Request(self.config['front_preview_url'])
            else:
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
                        projection_ms = (time.monotonic()-begin)*1000
                        encode_begin = time.monotonic()
                        ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 94])
                        if not ok:
                            raise RuntimeError('BEV JPEG encoding failed')
                        encode_ms = (time.monotonic()-encode_begin)*1000
                        self.fusion_sequence += 1
                        metadata = dict(frame_id=self.config['frame_id'], config_sha256=self.bev.config_sha256,
                            fusion_session=self.fusion_session, fusion_sequence=self.fusion_sequence,
                            produced_epoch_s=time.time(),
                            at_epoch_ns=time.time_ns(), source_selection='latest locally received frames; unsynchronized preview',
                            source_exposure_synchronized=False, metric_accuracy_verified=False,
                            actual_visible_ground_coverage_verified=False,
                            pose_source=self.config['pose_source'], sources={n: f[1] for n, f in frames.items()},
                            configured_cameras=list(self.config['cameras']),
                            source_receive_span_ms=(max(s['received_monotonic'] for _, s in frames.values())-
                                                   min(s['received_monotonic'] for _, s in frames.values()))*1000,
                            x_range_m=self.config['x_range_m'], y_range_m=self.config['y_range_m'],
                            observation_bounds_m=self.config.get('observation_bounds_m', {
                                'x_range_m': self.config['x_range_m'], 'y_range_m': self.config['y_range_m']}),
                            observation_region=self.config.get('observation_region'),
                            metres_per_pixel=self.bev.resolution, ground_plane=[0, 0, 1, 0],
                            dimensions=[self.bev.width, self.bev.height],
                            T_ground_from_upright_optical={n: t.tolist() for n, t in self.bev.transforms.items()},
                            projection_intrinsics={n: k.tolist() for n, k in self.bev.intrinsics.items()},
                            camera_image_sizes={n: list(im.shape[1::-1]) for n, im in images.items()},
                            camera_exclusion_polygons={n: c.get('exclude_polygons_upright_px', [])
                                                       for n, c in self.config['cameras'].items()},
                            vehicle_image_masks=('manually bounded visible vehicle structure; not ground segmentation'
                                if any(c.get('exclude_polygons_upright_px') for c in self.config['cameras'].values())
                                else 'none; masks for the new mounting have not been measured'),
                            **self.support_metadata,
                            projection_ms=projection_ms, jpeg_encode_ms=encode_ms,
                            snapshot_writer=self.exporter.metrics(),
                            processing_ms=(time.monotonic()-begin)*1000)
                        result = (images, image, counts, layers, metadata)
                        wire_meta = dict(metadata, image_width=self.bev.width, image_height=self.bev.height,
                                         encoding='jpeg', protocol='conbine.ground_bev.frame.v1')
                        header = json.dumps(wire_meta, allow_nan=False).encode()
                        packet = struct.pack('!4sII', b'BG01', len(header), len(jpeg))+header+jpeg.tobytes()
                        with self.lock:
                            self.display_slot = result
                            self.preview_packet = (metadata, packet)
                            self.views_snapshot = (self.fusion_sequence, packet, images)
                            if not self.headless and not self.pending:
                                self.pending = True
                                GLib.idle_add(self.display)
                        if begin-self.last_export >= 2:
                            self.exporter.submit(result)
                            self.last_export = begin
                        self.last_signature = signature
                    except Exception as exc:
                        print('COMPOSE_ERROR', str(exc), flush=True)
            STOP.wait(max(0, 1/self.config['preview_hz']-(time.monotonic()-begin)))

    def close(self):
        self.preview_server.shutdown()
        self.preview_server.server_close()
        self.exporter.close()

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
        self.status.set_text(self.rig_label+'实时地面投影 · %.1f fps · 计算 %.1f ms · 曝光未同步 · 精度待实测' % (fps, meta['processing_ms']))
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
            for name, color in [('left', [230, 190, 30]), ('front', [70, 210, 70]),
                                ('right', [20, 150, 240]), ('rear', [200, 70, 200])]:
                if name not in self.bev.maps:
                    continue
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
            self.status.set_text('等待'+self.rig_label+'新图像：'+', '.join(stale))
        return not STOP.is_set()

    def screenshot(self):
        window = self.window.get_window()
        pixbuf = Gdk.pixbuf_get_from_window(window, 0, 0, self.window.get_allocated_width(), self.window.get_allocated_height())
        pixbuf.savev(str(self.output/'live_window.png'), 'png', [], [])
        print('LIVE_GROUND_BEV_DISPLAYED', list(self.config['cameras']), flush=True)
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default=str(PROJECT/'config/three_camera_bev.json'))
    parser.add_argument('--assets', default=str(PROJECT/'runtime/camera_bev_check_20261025'))
    parser.add_argument('--board-root', default=BOARD)
    parser.add_argument('--headless', action='store_true', help='Serve BEV to Qt without a separate preview window')
    parser.add_argument('--check', action='store_true', help='Validate geometry without opening cameras')
    parser.add_argument('--reuse-capture', action='store_true', help='Read the existing four-view camera service')
    args = parser.parse_args()
    assets = Path(args.assets).resolve()
    config = json.loads(Path(args.config).read_text())
    bev = GroundBev(config)  # Validate before acquiring devices or launching any remote process.
    config = bev.config
    if not args.board_root.startswith('/map/conbine_camera/') or any(
            c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789/_-' for c in args.board_root):
        raise ValueError('Expected private /map/conbine_camera path')
    ports = [c['rx1_port'] for c in config['cameras'].values() if c['source'] == 'j6m_hardware_gdc']
    manifest = json.loads((assets/'manifest.json').read_text())
    if sorted(ports) != sorted(manifest['ports']):
        raise ValueError('Capture manifest ports do not match configured cameras')
    if args.check:
        print(json.dumps(dict(cameras=list(config['cameras']), ports=ports,
                              config_sha256=bev.config_sha256)))
        return
    print('START_GROUND_BEV', time.strftime('%Y-%m-%d %H:%M:%S'), list(config['cameras']),
          'ZED_pitch_deg=', config['cameras']['front']['pitch_down_deg'], flush=True)
    lock = open(assets/'bev_viewer.lock', 'a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    log, proc = None, None
    if not args.reuse_capture:
        log = open(assets/'bev_board.log', 'ab', buffering=0)
        proc = subprocess.Popen(SSH+['-o', 'ExitOnForwardFailure=yes', '-L', '18882:127.0.0.1:18882',
                                    'root@192.168.10.100', 'python3 -B -u '+args.board_root+'/server.py'], stdout=log, stderr=log)
    loop = GLib.MainLoop() if args.headless else None
    quit_loop = loop.quit if loop is not None else Gtk.main_quit
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: GLib.idle_add(quit_loop))
    viewer = None
    try:
        viewer = Viewer(config, assets, bev, headless=args.headless)
        if loop is not None:
            loop.run()
        else:
            Gtk.main()
    finally:
        STOP.set()
        if viewer is not None:
            viewer.close()
        if proc is not None:
            try:
                subprocess.run(SSH+['root@192.168.10.100', 'python3 -B '+args.board_root+'/stop.py'], timeout=10)
            finally:
                try: proc.wait(timeout=8)
                except subprocess.TimeoutExpired: proc.terminate(); proc.wait(timeout=5)
                log.close()


if __name__ == '__main__':
    main()
