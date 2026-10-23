"""Read-only preview transport; use the default loopback bind via an SSH tunnel."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


def serve(directory, host="127.0.0.1", port=18776):
    directory = Path(directory).resolve(strict=True)
    allowed = {"/state.json": ("state.json", "application/json", 8 * 1024 * 1024),
               "/preview.jpg": ("preview.jpg", "image/jpeg", 4 * 1024 * 1024),
               "/preview_meta.json": ("preview_meta.json", "application/json", 65536),
               "/bev.png": ("bev.png", "image/png", 16 * 1024 * 1024),
               "/bev.json": ("bev.json", "application/json", 65536),
               "/coverage.png": ("coverage.png", "image/png", 8 * 1024 * 1024)}
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path not in allowed:
                self.send_error(404)
                return
            name, mime, limit = allowed[self.path]
            try:
                path = directory / name
                if path.is_symlink():
                    raise OSError("Symlink not served")
                with path.open("rb") as f:
                    data = f.read(limit + 1)
                if len(data) > limit:
                    raise OSError("Preview exceeds size budget")
            except OSError:
                self.send_error(503)
                return
            self.send_response(200)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)
        def log_message(self, *_):
            pass
    server = ThreadingHTTPServer((host, port), Handler)
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
