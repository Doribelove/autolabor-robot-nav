"""Configured RX1 NV12 cameras with hardware GDC; no ROS or vehicle control."""
import ctypes as C
import fcntl
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import signal
import struct
import threading
import time

ROOT = Path(__file__).resolve().parent
STOP = threading.Event()
MUTEX = threading.Lock()
PAIRS = {}
STATE = {'status': 'starting', 'camera_port': 0, 'same_source_frame': True,
         'accuracy_verified': False, 'horizontal_fov_deg': 120, 'ports': {}}


class Meta(C.Structure):
    _fields_ = [('raw_stamp', C.c_uint64), ('hal_received_ns', C.c_uint64),
                ('sequence', C.c_uint32), ('width', C.c_uint32), ('height', C.c_uint32),
                ('bytes', C.c_uint32), ('vendor_format', C.c_int32), ('gdc_applied', C.c_uint32)]


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def do_GET(self):
        with MUTEX:
            state = dict(STATE)
            now = time.monotonic()
            state['age_s'] = max(0, now-state.get('received_monotonic', now))
            state['ports'] = {p: dict(m, age_s=max(0, now-m.get('received_monotonic', now)))
                              for p, m in STATE['ports'].items()}
            port = self.path.rsplit('/', 1)[-1] if self.path.startswith(('/pair/', '/gdc/')) else '0'
            pair = PAIRS.get(port)
            if self.path == '/state':
                payload, mime, code = json.dumps(state).encode(), 'application/json', 200
            elif (self.path == '/pair' or self.path.startswith(('/pair/', '/gdc/'))) and pair is not None and now-pair[0]['received_monotonic'] < 1.5:
                meta, data = pair
                gdc_only = self.path.startswith('/gdc/')
                if gdc_only:
                    data = data[meta['raw']['bytes']:]
                header = json.dumps(dict(meta, age_s=now-meta['received_monotonic'],
                                         payload_layout='gdc_only' if gdc_only else 'raw_and_gdc')).encode()
                payload, mime, code = struct.pack('!I', len(header))+header+data, 'application/octet-stream', 200
            else:
                payload, mime, code = b'No fresh camera pair', 'text/plain', 503
        self.send_response(code)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(payload)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        try:
            self.wfile.write(payload)
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    lock = open('/map/j6m-dev-b/vision-bpu-surround/camera.lock', 'a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    manifest = json.loads((ROOT/'manifest.json').read_text())
    for name, expected in manifest['sha256'].items():
        if Path(name).name != name or hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != expected:
            raise RuntimeError('Input hash mismatch: '+name)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: STOP.set())
    (ROOT/'server.pid').write_text(str(os.getpid())+'\n')
    lib = C.CDLL(str(ROOT/'libcapture_pair.so'))
    lib.sc_error.restype = C.c_char_p
    lib.sc_open.argtypes = [C.c_char_p]*4
    lib.sc_open.restype = C.c_void_p
    lib.sc_start.argtypes = [C.c_void_p, C.c_uint]
    lib.sc_set_gdc.argtypes = [C.c_void_p, C.c_uint, C.c_void_p, C.c_size_t]
    lib.sc_read_pair.argtypes = [C.c_void_p, C.c_uint, C.c_int, C.c_uint,
                                C.c_void_p, C.c_size_t, C.POINTER(Meta),
                                C.c_void_p, C.c_size_t, C.POINTER(Meta)]
    lib.sc_close.argtypes = [C.c_void_p]
    ctx = None
    workers = []
    server = ThreadingHTTPServer(('127.0.0.1', 18882), Handler)
    server.timeout = .25

    def check(ret):
        if ret < 0:
            raise RuntimeError(lib.sc_error().decode())

    def capture(port):
        raw = (C.c_uint8*(1920*1536*3//2))()
        gdc = (C.c_uint8*(960*540*3//2))()
        last = 0
        try:
            while not STOP.is_set():
                rm, gm = Meta(), Meta()
                check(lib.sc_read_pair(ctx, port, 1, 0, gdc, len(gdc), C.byref(gm),
                                       raw, len(raw), C.byref(rm)))
                # PYM exposes MEM_PIX_FMT_NV12=8; GDC exposes HB_YUV420SP=2.
                # The vendor PYM conversion copies its native format after conversion.
                if ((rm.width, rm.height, rm.bytes, rm.gdc_applied, rm.vendor_format) !=
                        (1920, 1536, len(raw), 0, 8) or
                        (gm.width, gm.height, gm.bytes, gm.gdc_applied, gm.vendor_format) !=
                        (960, 540, len(gdc), 1, 2) or
                        (rm.sequence, rm.raw_stamp) != (gm.sequence, gm.raw_stamp)):
                    raise RuntimeError('Paired frame metadata mismatch: '+json.dumps({
                        'raw': {k: getattr(rm, k) for k, _ in Meta._fields_},
                        'gdc': {k: getattr(gm, k) for k, _ in Meta._fields_}}))
                now = time.monotonic()
                if now-last < .18:
                    continue
                data = C.string_at(raw, len(raw))+C.string_at(gdc, len(gdc))
                meta = {'raw': {k: getattr(rm, k) for k, _ in Meta._fields_},
                        'gdc': {k: getattr(gm, k) for k, _ in Meta._fields_},
                        'camera_port': port, 'same_source_frame': True,
                        'received_monotonic': now, 'horizontal_fov_deg': 120,
                        'accuracy_verified': False}
                with MUTEX:
                    PAIRS[str(port)] = (meta, data)
                    STATE['ports'][str(port)] = dict(meta, status='live')
                    if port == 0:
                        STATE.update(meta, status='live')
                last = now
        except Exception as exc:
            with MUTEX:
                STATE['ports'][str(port)] = dict(status='error', error=str(exc))
                PAIRS.pop(str(port), None)
                if port == 0:
                    STATE.update(status='error', error=str(exc))
            print('CAPTURE_ERROR', port, str(exc), flush=True)

    try:
        ctx = lib.sc_open(str(ROOT/'vpm_gdc.json').encode(), str(ROOT/'hb_j6dev.json').encode(),
                          b'/usr/hobot/lib/libvio.so.1', b'/usr/hobot/lib/libhbmem.so.1')
        if not ctx:
            raise RuntimeError(lib.sc_error().decode())
        for port in manifest['ports']:
            data = (ROOT/('port%d.bin' % port)).read_bytes()
            blob = C.create_string_buffer(data)
            check(lib.sc_set_gdc(ctx, port, blob, len(data)))
        for port in manifest['ports']:
            check(lib.sc_start(ctx, port))
            print('PAIRED_CAMERA_STARTED port=%d hardware_gdc=1' % port, flush=True)
            worker = threading.Thread(target=capture, args=(port,), daemon=True)
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
        (ROOT/'server.pid').unlink(missing_ok=True)
        print('PAIRED_CAMERA_STOPPED', flush=True)


if __name__ == '__main__':
    main()
