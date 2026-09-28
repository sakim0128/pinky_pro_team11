"""Bounded read-only routes; no control endpoints or ROS dependencies."""
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit


def make_server(host, port, store, map_asset, frames=None, controls=None):
    assets = Path(__file__).with_name('live_static')
    routes = {'/': ('index.html', 'text/html; charset=utf-8'),
              '/fleet-lanes.svg': ('fleet-lanes.svg', 'image/svg+xml'),
              '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
              '/style.css': ('style.css', 'text/css; charset=utf-8')}

    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(3)

        def log_message(self, *_args):
            pass

        def respond(self, status, body, content_type):
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == '/api/state':
                snapshot = store.snapshot()
                snapshot['cameras'] = frames.snapshot() if frames is not None else {}
                snapshot['controls_enabled'] = bool(controls and controls.enabled)
                snapshot['map_compatibility'] = {
                    robot['name']: map_asset.compatibility(robot['state']['data'])
                    for robot in snapshot['robots']}
                body = json.dumps(snapshot, allow_nan=False).encode()
                self.respond(200, body, 'application/json')
            elif path == '/api/map/metadata':
                body = json.dumps(map_asset.metadata, allow_nan=False).encode()
                self.respond(200, body, 'application/json')
            elif path == '/api/map/image.png':
                self.respond(200, map_asset.png, 'image/png')
            elif path.startswith('/api/camera/') and path.endswith('.jpg'):
                name = path.removeprefix('/api/camera/').removesuffix('.jpg')
                frame = frames.get(name) if frames is not None else None
                self.respond(200, frame, 'image/jpeg') if frame else self.respond(404, b'Camera frame unavailable', 'text/plain')
            elif path in routes:
                filename, mime = routes[path]
                self.respond(200, (assets / filename).read_bytes(), mime)
            else:
                self.respond(404, b'Not found', 'text/plain')

        def do_POST(self):
            if urlsplit(self.path).path != '/api/control':
                self.respond(404, b'Not found', 'text/plain'); return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if size <= 0 or size > 512:
                    raise ValueError
                payload = json.loads(self.rfile.read(size))
            except (ValueError, json.JSONDecodeError):
                self.respond(400, b'Invalid control request', 'text/plain'); return
            if controls is None or not controls.submit(payload):
                self.respond(403, b'Control disabled or invalid', 'text/plain'); return
            self.respond(202, b'Control request queued', 'text/plain')

    return ThreadingHTTPServer((host, port), Handler)
