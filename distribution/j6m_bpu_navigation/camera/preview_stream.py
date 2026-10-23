"""Bounded, authenticated display copies of an already inferred RGB frame."""
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from socketserver import TCPServer
import struct
import threading

DETECTOR_SHA256 = 'f2f1f7e9753b53b0494495be44af92a0603b3ee6c71e69e0f58a0d50544a8cb8'
PREFIX = struct.Struct('!4sII')
MAX_METADATA = 65536
MAX_IMAGE = 4 * 1024 * 1024


def unpack_preview(payload):
    if len(payload) < PREFIX.size:
        raise ValueError('Truncated preview')
    magic, nj, ni = PREFIX.unpack_from(payload)
    if magic != b'ZP01' or not 0 < nj <= MAX_METADATA or not 0 < ni <= MAX_IMAGE:
        raise ValueError('Invalid preview bounds')
    if len(payload) != PREFIX.size + nj + ni:
        raise ValueError('Preview length mismatch')
    meta = json.loads(payload[PREFIX.size:PREFIX.size + nj])
    result, frame = meta['result'], meta['image']
    if (meta.get('backend') != 'J6M_BPU' or meta.get('detector_sha256') != DETECTOR_SHA256
            or frame['session'] != result['session'] or frame['sequence'] != result['sequence']
            or frame['source_stamp_ns'] != result['source_stamp_ns']
            or frame.get('width') != 1280 or frame.get('height') != 720
            or result.get('motion_eligible') is not False):
        raise ValueError('Preview identity/backend mismatch')
    return meta, payload[PREFIX.size + nj:]


class PreviewServer:
    """Encoding runs in the HTTP worker, never in the BPU inference thread."""
    def __init__(self, config, cv2, age_check):
        self.cv2, self.age_check = cv2, age_check
        self.lock = threading.Lock()
        self.latest = None
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass

            def do_GET(self):
                self.connection.settimeout(.5)
                if (self.client_address[0] != config['agx_ip'] or not hmac.compare_digest(
                        self.headers.get('Authorization', ''), 'Bearer ' + config['token'])):
                    self.send_error(403)
                    return
                if self.path != '/frame':
                    self.send_error(404)
                    return
                with owner.lock:
                    pair = owner.latest
                if pair is None:
                    self.send_error(503, 'Waiting for BPU result')
                    return
                frame, result = pair
                try:
                    owner.age_check(result['source_stamp_ns'])
                    ok, data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
                    if not ok:
                        raise ValueError('JPEG encode failed')
                    owner.age_check(result['source_stamp_ns'])
                    meta = dict(protocol=1, backend='J6M_BPU', detector_sha256=DETECTOR_SHA256,
                                image=dict(session=result['session'], sequence=result['sequence'],
                                           source_stamp_ns=result['source_stamp_ns'], width=1280, height=720,
                                           encoding='jpeg_display_copy'), result=result)
                    header = json.dumps(meta, allow_nan=False, separators=(',', ':')).encode()
                    if len(header) > MAX_METADATA or len(data) > MAX_IMAGE:
                        raise ValueError('Preview too large')
                    payload = PREFIX.pack(b'ZP01', len(header), len(data)) + header + data.tobytes()
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/octet-stream')
                    self.send_header('Content-Length', str(len(payload)))
                    self.send_header('Cache-Control', 'no-store')
                    self.end_headers()
                    self.wfile.write(payload)
                except (ValueError, OSError):
                    # A closed/stale preview never changes acquisition or inference state.
                    self.close_connection = True

        class Server(ThreadingHTTPServer):
            daemon_threads = True
            request_queue_size = 2

            def server_bind(self):
                # The isolated wired subnet has no reverse DNS; do not block startup on it.
                TCPServer.server_bind(self)
                self.server_name, self.server_port = self.server_address[:2]

            def get_request(self):
                client, address = super().get_request()
                client.settimeout(.5)
                return client, address

            def __init__(self, *args):
                self.slots = threading.BoundedSemaphore(2)
                super().__init__(*args)

            def process_request(self, request, address):
                if not self.slots.acquire(blocking=False):
                    self.shutdown_request(request)
                    return
                try:
                    super().process_request(request, address)
                except BaseException:
                    self.slots.release()
                    raise

            def process_request_thread(self, request, address):
                try:
                    super().process_request_thread(request, address)
                finally:
                    self.slots.release()

        self.server = Server((config['j6m_ip'], config['preview_port']), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def publish(self, frame, result):
        # The receiver owns a distinct immutable image for every source frame.
        with self.lock:
            self.latest = (frame, result)

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=1)
