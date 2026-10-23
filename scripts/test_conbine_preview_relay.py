#!/usr/bin/env python3
"""Exercise actual TCP -> Unix -> TCP preview forwarding without robot access."""
import http.client
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import tempfile
import threading
import unittest

from conbine_preview_relay import MAX_BYTES, UnixConnection, UnixServer, handler


class RelayTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.calls = []
        cls.payload = b''
        cls.code = 200
        cls.length = None
        class Upstream(BaseHTTPRequestHandler):
            def log_message(self, *_): pass
            def do_GET(self):
                cls.calls.append(self.path)
                self.send_response(cls.code)
                self.send_header('Content-Length', str(len(cls.payload) if cls.length is None else cls.length))
                self.send_header('Content-Type', 'application/octet-stream')
                self.end_headers()
                try: self.wfile.write(cls.payload)
                except (BrokenPipeError, ConnectionResetError): pass
        cls.temp = tempfile.TemporaryDirectory()
        cls.up = ThreadingHTTPServer(('127.0.0.1', 0), Upstream)
        cls.unix = UnixServer(str(Path(cls.temp.name) / 'relay.sock'), handler(
            lambda: http.client.HTTPConnection('127.0.0.1', cls.up.server_port, timeout=.5)))
        cls.front = ThreadingHTTPServer(('127.0.0.1', 0), handler(
            lambda: UnixConnection(cls.unix.server_address)))
        cls.threads = []
        for server in (cls.up, cls.unix, cls.front):
            thread = threading.Thread(target=lambda s=server: s.serve_forever(poll_interval=.02), daemon=True)
            thread.start(); cls.threads.append(thread)

    @classmethod
    def tearDownClass(cls):
        for server in (cls.front, cls.unix, cls.up):
            server.shutdown(); server.server_close()
        for thread in cls.threads: thread.join(2)
        cls.temp.cleanup()

    def setUp(self):
        type(self).calls.clear()
        type(self).payload = b'SG01\x00\x00\xffsame-source-session-and-frame'
        type(self).code = 200
        type(self).length = None

    def get(self, path, method='GET'):
        connection = http.client.HTTPConnection('127.0.0.1', self.front.server_port, timeout=2)
        connection.request(method, path)
        response = connection.getresponse()
        result = response.status, response.read()
        connection.close()
        return result

    def test_frame_bytes_and_port_preserved(self):
        for port in range(3):
            self.assertEqual((200, self.payload), self.get('/frame/%d' % port))
        self.assertEqual(['/frame/0', '/frame/1', '/frame/2'], self.calls)

    def test_age_and_identity_not_rewritten(self):
        type(self).payload = b'{"read_age_ms":499,"session":"original","sequence":12}'
        self.assertEqual((200, self.payload), self.get('/state'))

    def test_upstream_loss_does_not_reuse_cached_image(self):
        self.assertEqual(200, self.get('/frame/0')[0])
        type(self).code, type(self).payload = 503, b'Expired'
        self.assertEqual((503, b'Expired'), self.get('/frame/0'))

    def test_only_fixed_read_endpoints_forwarded(self):
        for path in ('/frame/3', '/state?url=http://robot/', '/../../etc/passwd', '/control', 'http://robot/state'):
            self.assertEqual(404, self.get(path)[0])
        self.assertEqual([], self.calls)

    def test_mutating_methods_rejected(self):
        for method in ('POST', 'PUT', 'DELETE', 'CONNECT'):
            self.assertEqual(501, self.get('/state', method)[0])
        self.assertEqual([], self.calls)

    def test_oversized_payload_rejected(self):
        type(self).length = MAX_BYTES + 1
        self.assertEqual(503, self.get('/frame/0')[0])

    def test_truncated_payload_rejected(self):
        type(self).length = len(self.payload) + 8
        self.assertEqual(503, self.get('/frame/1')[0])


if __name__ == '__main__':
    unittest.main(verbosity=2)
