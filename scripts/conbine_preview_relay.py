#!/usr/bin/env python3
"""Read-only frame relay across the navigation preview's network namespace."""
import argparse
import http.client
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import os
from pathlib import Path
import signal
import socket
import socketserver
import threading

ALLOWED = frozenset(['/state', '/frame/0', '/frame/1', '/frame/2'])
MAX_BYTES = 12 + 128 * 1024 + 2 * 1024 * 1024


class UnixConnection(http.client.HTTPConnection):
    def __init__(self, path):
        super().__init__('localhost', timeout=.6)
        self.path = str(path)

    def connect(self):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect(self.path)


def handler(connection_factory):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_GET(self):
            if self.path not in ALLOWED:
                self.send_error(404)
                return
            connection = connection_factory()
            try:
                connection.request('GET', self.path, headers={'Connection': 'close'})
                response = connection.getresponse()
                length = int(response.getheader('Content-Length', '-1'))
                if not 0 <= length <= MAX_BYTES:
                    raise ValueError('Invalid upstream payload length')
                payload = response.read(length + 1)
                if len(payload) != length:
                    raise ValueError('Incomplete upstream payload')
                status = response.status
                mime = response.getheader('Content-Type', 'application/octet-stream')
            except (OSError, ValueError, http.client.HTTPException):
                status, mime, payload = 503, 'text/plain', b'Surround source unavailable'
            finally:
                connection.close()
            self.send_response(status)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            try:
                # Preserve bytes, frame identity and age. Qt accounts for this
                # entire request duration in its existing freshness check.
                self.wfile.write(payload)
            except (BrokenPipeError, ConnectionResetError):
                pass
    return Handler


class UnixServer(socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
    daemon_threads = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('side', choices=['host', 'inside'])
    parser.add_argument('--socket', required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    path = Path(args.socket)
    expected = root / 'runtime/conbine_preview'
    if not path.is_absolute() or expected not in path.resolve().parents:
        raise RuntimeError('A private combined-preview socket is required')
    if args.side == 'host':
        # Never unlink an existing endpoint belonging to another invocation.
        server = UnixServer(str(path), handler(lambda: http.client.HTTPConnection(
            '127.0.0.1', 18781, timeout=.6)))
        os.chmod(str(path), 0o600)
    else:
        if [name for _, name in socket.if_nameindex()] != ['lo']:
            raise RuntimeError('Frame reader must run in the isolated namespace')
        server = ThreadingHTTPServer(('127.0.0.1', 18782),
                                     handler(lambda: UnixConnection(path)))
    server.timeout = .2
    stop = threading.Event()
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: stop.set())
    try:
        while not stop.is_set():
            server.handle_request()
    finally:
        server.server_close()
        if args.side == 'host':
            path.unlink()


if __name__ == '__main__':
    main()
