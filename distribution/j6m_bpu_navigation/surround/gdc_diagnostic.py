#!/usr/bin/env python3
"""Private RX1 three-camera hardware GDC preview; no CPU image remapping."""
import ctypes as C
import fcntl
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import signal
import threading
import time
import argparse
import uuid

ROOT = Path(__file__).resolve().parent
SESSION = uuid.uuid4().hex
STOP = threading.Event()
MUTEX = threading.Lock()
STATE = {'mode': 'hardware_gdc_perspective_diagnostic', 'rx': 1,
         'horizontal_fov_deg': 120, 'intrinsic_source': 'camera_flash',
         'accuracy_verified': False, 'production_ready': False, 'motion_eligible': False,
         'ports': {str(p): {'error': 'starting'} for p in range(3)}}
FRAMES = {}


class Meta(C.Structure):
    _fields_ = [('raw_stamp', C.c_uint64), ('hal_received_ns', C.c_uint64),
                ('sequence', C.c_uint32), ('width', C.c_uint32), ('height', C.c_uint32),
                ('bytes', C.c_uint32), ('vendor_format', C.c_int32), ('gdc_applied', C.c_uint32)]


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def do_GET(self):
        now = time.monotonic()
        headers = {}
        with MUTEX:
            if self.path == '/state':
                status = dict(STATE, ports={p: dict(info) for p, info in STATE['ports'].items()})
                for info in status['ports'].values():
                    if 'received_monotonic' in info:
                        info['age_s'] = now - info['received_monotonic']
                payload, mime, code = json.dumps(status).encode(), 'application/json', 200
            elif self.path.startswith('/frame/') and self.path[7:] in FRAMES:
                info, payload = FRAMES[self.path[7:]]
                headers['X-Frame-Meta'] = json.dumps(dict(info, age_s=now-info['received_monotonic']))
                mime, code = 'application/octet-stream', 200
            else:
                payload, mime, code = b'No live GDC frame', 'text/plain', 404
        self.send_response(code)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(payload)))
        self.send_header('Cache-Control', 'no-store')
        for k, v in headers.items():
            self.send_header(k, v)
        self.end_headers()
        try:
            self.wfile.write(payload)
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    if not str(ROOT).startswith('/map/j6m-dev-b/vision-bpu-surround/') or ROOT.is_symlink():
        raise RuntimeError('Private board preview path mismatch')
    lock = Path('/map/j6m-dev-b/vision-bpu-surround/camera.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    manifest = json.loads((ROOT/'live_manifest.json').read_text())
    for name, expected in manifest['sha256'].items():
        if Path(name).name != name or hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != expected:
            raise RuntimeError('GDC input hash mismatch: '+name)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: STOP.set())
    pid_path = ROOT/'live.pid'
    pid_path.write_text(str(os.getpid())+'\n')
    lib = C.CDLL(str(ROOT/'libj6m_surround_capture.so'))
    lib.sc_error.restype = C.c_char_p
    lib.sc_open.argtypes = [C.c_char_p]*4
    lib.sc_open.restype = C.c_void_p
    lib.sc_start.argtypes = [C.c_void_p, C.c_uint]
    lib.sc_set_gdc.argtypes = [C.c_void_p, C.c_uint, C.c_void_p, C.c_size_t]
    lib.sc_read.argtypes = [C.c_void_p, C.c_uint, C.c_int, C.c_uint,
                            C.c_void_p, C.c_size_t, C.POINTER(Meta)]
    lib.sc_close.argtypes = [C.c_void_p]
    ctx = None
    workers = []
    server = ThreadingHTTPServer(('127.0.0.1', 18780), Handler)
    server.timeout = 0.25

    def check(ret):
        if ret < 0:
            raise RuntimeError(lib.sc_error().decode())

    def capture(p):
        # Private output is exactly 960x540 NV12. The native API bounds-checks copies.
        buf = (C.c_uint8*(960*540*3//2))()
        last, count = 0, 0
        try:
            while not STOP.is_set():
                meta = Meta()
                ret = lib.sc_read(ctx, p, 1, 0, buf, len(buf), C.byref(meta))
                if ret < 0:
                    with MUTEX:
                        STATE['ports'][str(p)] = {'error': lib.sc_error().decode()}
                        FRAMES.pop(str(p), None)
                    STOP.wait(.1)
                    continue
                if (meta.gdc_applied != 1 or meta.vendor_format != 2 or
                        (meta.width, meta.height, meta.bytes) != (960, 540, len(buf))):
                    raise RuntimeError('Hardware GDC output format validation failed')
                count += 1
                now = time.monotonic()
                if now-last < .19:
                    continue
                payload = C.string_at(buf, meta.bytes)
                # Small luminance sample is diagnostic only; no image warp on CPU.
                ys = payload[:960*540:32]
                info = {k: getattr(meta, k) for k, _ in Meta._fields_}
                info.update(layout='NV12', source_width=1920, source_height=1536,
                            received_monotonic=now, read_count=count, error=None,
                            camera_port=p, session=SESSION,
                            y_min=min(ys), y_max=max(ys), lut_sha256=manifest['sha256']['port%d.bin'%p])
                with MUTEX:
                    FRAMES[str(p)] = (info, payload)
                    STATE['ports'][str(p)] = info
                last = now
        except Exception as exc:
            with MUTEX:
                STATE['ports'][str(p)] = {'error': str(exc)}
                FRAMES.pop(str(p), None)
            print('CAPTURE_FAILED', p, str(exc), flush=True)

    try:
        ctx = lib.sc_open(str(ROOT/'vpm_gdc.json').encode(), str(ROOT/'hb_j6dev.json').encode(),
                          b'/usr/hobot/lib/libvio.so.1', b'/usr/hobot/lib/libhbmem.so.1')
        if not ctx:
            raise RuntimeError(lib.sc_error().decode())
        for p in range(3):
            data = (ROOT/('port%d.bin'%p)).read_bytes()
            blob = C.create_string_buffer(data)
            check(lib.sc_set_gdc(ctx, p, blob, len(data)))
        ports = []
        for p in range(3):
            ret = lib.sc_start(ctx, p)
            STATE['ports'][str(p)] = {'error': lib.sc_error().decode() if ret < 0 else None}
            if ret == 0:
                ports.append(p)
        print('GDC_STARTED_PORTS', ports, flush=True)
        if not ports:
            raise RuntimeError('No GDC camera started')
        for p in ports:
            worker = threading.Thread(target=capture, args=(p,), daemon=True, name='gdc-port-%d'%p)
            workers.append(worker)
            worker.start()
        while not STOP.is_set():
            server.handle_request()
    finally:
        STOP.set()
        for worker in workers:
            worker.join(5)
        if ctx and all(not worker.is_alive() for worker in workers):
            lib.sc_close(ctx)
        server.server_close()
        if pid_path.exists() and pid_path.read_text().strip() == str(os.getpid()):
            pid_path.unlink()
        print('GDC_PREVIEW_STOPPED', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--assets', required=True)
    args = parser.parse_args()
    ROOT = Path(args.assets).resolve(strict=True)
    main()
