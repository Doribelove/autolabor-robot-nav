"""Three GDC inputs scheduled fairly on one resident pair of existing HBM models."""
import argparse
import collections
import copy
import fcntl
import http.client
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import signal
import threading
import time
import uuid

from .operator_protocol import (MAX_AGE_MS, DETECTOR_SHA, CLASSIFIER_SHA,
                                capture_identity, SourceGuard, packet)


def fetch_gdc(port, config):
    connection = http.client.HTTPConnection('127.0.0.1', 18780, timeout=.15)
    try:
        connection.request('GET', '/frame/%d'%port)
        reply = connection.getresponse()
        if reply.status != 200 or reply.getheader('Content-Length') != '777600':
            raise ValueError('GDC port %d unavailable'%port)
        header = reply.getheader('X-Frame-Meta', '')
        if not 0 < len(header) < 8192:
            raise ValueError('Invalid GDC frame header')
        meta = json.loads(header)
        source = capture_identity(meta, port, config['lut_sha256'][str(port)])
        raw = reply.read(777601)
        if len(raw) != 777600:
            raise ValueError('Truncated GDC frame')
        return source, meta['received_monotonic'], raw
    finally:
        connection.close()


def main(config):
    import cv2
    import numpy as np
    from .inference import BpuEngine
    cv2.setNumThreads(1)
    root = Path(config['runtime_root']).resolve()
    if not str(root).startswith('/map/j6m-dev-b/vision-bpu-surround/'):
        raise ValueError('Private surround deployment required')
    runtime_lock = (root/'operator.lock').open('a')
    fcntl.flock(runtime_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    if (config['inference']['detector_zed']['sha256'] != DETECTOR_SHA
            or config['inference']['classifier']['sha256'] != CLASSIFIER_SHA):
        raise ValueError('Expected the existing YOLO11-GAM and material HBM models')
    stop, mutex = threading.Event(), threading.Lock()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: stop.set())
    session = uuid.uuid4().hex
    frames, errors = {}, {p: 'starting' for p in range(3)}
    counts = collections.Counter()
    history = {p: collections.deque(maxlen=40) for p in range(3)}
    guard = SourceGuard()
    (root/'inference.pid').write_text(str(os.getpid())+'\n')

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_GET(self):
            now = time.monotonic()
            with mutex:
                if self.path == '/state':
                    states = {}
                    for p in range(3):
                        cached = frames.get(p)
                        age = (now-cached[2])*1000 if cached else None
                        states[str(p)] = dict(error=errors[p], inferred_frames=counts[p],
                            read_age_ms=age, fresh=bool(cached and not errors[p] and age <= MAX_AGE_MS),
                            source=cached[0]['source'] if cached else None,
                            timing=cached[0]['timing'] if cached else None)
                    data = dict(schema='j6m.surround.operator.state.v1', session=session,
                        inference_device='J6M_BPU', ports=states,
                        production_ready=False, motion_eligible=False, source_clock_verified=False)
                    payload, mime, code = json.dumps(data).encode(), 'application/json', 200
                elif self.path in ['/frame/0', '/frame/1', '/frame/2']:
                    p = int(self.path[-1])
                    cached = frames.get(p)
                    if not cached or errors[p] or not 0 <= (now-cached[2])*1000 <= MAX_AGE_MS:
                        payload, mime, code = b'Waiting for fresh GDC/BPU result', 'text/plain', 503
                    else:
                        meta = dict(cached[0], read_age_ms=(now-cached[2])*1000)
                        payload, mime, code = packet(meta, cached[1]), 'application/octet-stream', 200
                else:
                    payload, mime, code = b'Not found', 'text/plain', 404
            self.send_response(code)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            try:
                self.wfile.write(payload)
            except (BrokenPipeError, ConnectionResetError):
                pass

    def infer(engine):
        while not stop.is_set():
            did_work = False
            for p in range(3):
                if stop.is_set():
                    return
                try:
                    source, received, raw = fetch_gdc(p, config)
                    if not guard.accept(source):
                        continue
                    remaining = MAX_AGE_MS/1000 - (time.monotonic()-received)
                    if remaining <= 0:
                        raise ValueError('GDC input expired before inference')
                    image = cv2.cvtColor(np.frombuffer(raw, np.uint8).reshape(810, 960), cv2.COLOR_YUV2BGR_NV12)
                    # Only a processing deadline; never presented as exposure time.
                    objects, timing = engine.infer(image, time.time_ns()+int(remaining*1e9))
                    ok, jpg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 90])
                    if not ok or time.monotonic()-received > MAX_AGE_MS/1000:
                        raise ValueError('GDC/BPU output expired or JPEG encode failed')
                    now = time.monotonic()
                    history[p].append(now)
                    hz = (len(history[p])-1)/(history[p][-1]-history[p][0]) if len(history[p])>1 else 0
                    meta = dict(schema='j6m.surround.operator.frame.v1', inference_session=session,
                        source=source, image_source=copy.deepcopy(source), width=960, height=540,
                        encoding='jpeg_display_copy', gdc_applied=True,
                        lut_sha256=config['lut_sha256'][str(p)],
                        detector_sha256=DETECTOR_SHA, classifier_sha256=CLASSIFIER_SHA,
                        inference_device='J6M_BPU', objects=objects, timing=timing, inference_fps=hz,
                        read_to_result_ms=(now-received)*1000, source_clock_verified=False,
                        accuracy_verified=False, production_ready=False, motion_eligible=False)
                    with mutex:
                        frames[p] = (meta, jpg.tobytes(), received)
                        errors[p] = None
                        counts[p] += 1
                    did_work = True
                except (OSError, ValueError, KeyError, TypeError, RuntimeError,
                        http.client.HTTPException, cv2.error) as exc:
                    with mutex:
                        errors[p] = '%s: %s'%(type(exc).__name__, exc)
                        frames.pop(p, None)
            stop.wait(.005 if did_work else .03)

    engine, worker, server = None, None, None
    try:
        engine = BpuEngine(config['inference'])  # Preserves the existing global exclusive BPU lock.
        server = ThreadingHTTPServer(('127.0.0.1', 18781), Handler)
        server.timeout = .25
        worker = threading.Thread(target=infer, args=(engine,), name='three-port-bpu', daemon=True)
        worker.start()
        print('SURROUND_BPU_READY', session, flush=True)
        while not stop.is_set() and worker.is_alive():
            server.handle_request()
    finally:
        stop.set()
        if worker:
            worker.join(5)
        if engine and (not worker or not worker.is_alive()):
            engine.close()
        if server:
            server.server_close()
        print('SURROUND_BPU_STOPPED', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    main(json.loads(Path(parser.parse_args().config).read_text()))
