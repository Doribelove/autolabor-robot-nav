#!/usr/bin/env python3
"""Run the existing CUDA YOLO model on actual BEV frames; serve atomic pairs."""
import argparse
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import signal
import struct
import sys
import threading
import time
import urllib.request

import cv2
import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT/'src/application/autolabor_fod_vision/src'))
from autolabor_fod_vision.detector import UltralyticsDetector, annotate_image

STOP = threading.Event()
LOCK = threading.Lock()
LATEST = None
STATUS = {'status': 'loading_yolo'}
EXPECTED_MODEL = '5efaafa1503db11c2ba261b4429389d96335b4eef4d0fc44d6ca41e7431f2d0f'


def read_ground_frame(url):
    with urllib.request.urlopen(url, timeout=2) as response:
        data = response.read(3*1024*1024)
    if len(data) < 12:
        raise ValueError('Truncated ground frame')
    magic, nj, ni = struct.unpack('!4sII', data[:12])
    if magic != b'BG01' or not 0 < nj <= 128*1024 or not 0 < ni <= 2*1024*1024 or len(data) != 12+nj+ni:
        raise ValueError('Invalid ground frame envelope')
    meta = json.loads(data[12:12+nj])
    if (meta.get('protocol') != 'conbine.ground_bev.frame.v1' or
            set(meta.get('sources', {})) != {'left', 'right', 'front'} or
            not meta.get('fusion_session') or time.time()-meta['produced_epoch_s'] > 2):
        raise ValueError('Ground frame is stale or has missing inputs')
    jpeg = data[12+nj:]
    image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if image is None or list(image.shape[1::-1]) != [meta['image_width'], meta['image_height']]:
        raise ValueError('Ground image dimensions mismatch')
    return meta, image, jpeg


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def do_GET(self):
        with LOCK:
            latest, status = LATEST, dict(STATUS)
        if self.path == '/state':
            payload = json.dumps(status, allow_nan=False).encode()
            mime = 'application/json'
        elif self.path == '/frame' and latest and time.time()-latest[0] < 2:
            payload, mime = latest[1], 'application/octet-stream'
        else:
            self.send_error(503, 'Waiting for fresh YOLO BEV result')
            return
        self.send_response(200)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(payload)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        try:
            self.wfile.write(payload)
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    global LATEST
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', default='http://127.0.0.1:18883/frame')
    parser.add_argument('--port', type=int, default=18884)
    parser.add_argument('--output', default=str(PROJECT/'runtime/bev_qt'))
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    cv2.setNumThreads(1)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: STOP.set())
    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    detector = None
    try:
        detector = UltralyticsDetector(
            weights=str(PROJECT/'src/application/autolabor_fod_vision/models/best6.pt'),
            ultralytics_root=str(PROJECT/'ultralytics_yolo11_custom'), device='0',
            image_size=640, confidence=.25, iou=.45, require_gam=True,
            expected_sha256=EXPECTED_MODEL)
        print('YOLO_READY', detector.device, detector.names, detector.model_sha256, flush=True)
        last_id, last_export, processed, first = None, 0, 0, time.monotonic()
        last_error = 0
        while not STOP.is_set():
            begin = time.monotonic()
            try:
                source, image, raw_jpeg = read_ground_frame(args.input)
                identity = dict(session=source['fusion_session'], sequence=source['fusion_sequence'])
                if identity == last_id:
                    STOP.wait(.04)
                    continue
                result = detector.predict(image)
                if time.time()-source['produced_epoch_s'] > 2:
                    raise ValueError('BEV source expired during YOLO inference')
                annotated = annotate_image(image, result.detections, result.inference_ms, 'YOLO11-GAM / BEV')
                ok, encoded = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 94])
                if not ok:
                    raise ValueError('Annotated image encoding failed')
                objects = [dict(class_id=d.class_id, class_name=d.class_name, confidence=d.confidence,
                                bbox=[d.xmin, d.ymin, d.xmax, d.ymax]) for d in result.detections]
                processed += 1
                if processed == 1:
                    first = time.monotonic()
                meta = dict(schema='conbine.bev.yolo.frame.v1', source_id=identity,
                    image_source_id=identity, result_source_id=identity,
                    produced_epoch_s=source['produced_epoch_s'], result_epoch_s=time.time(),
                    width=image.shape[1], height=image.shape[0], objects=objects,
                    inference_ms=result.inference_ms, inference_fps=(processed-1)/max(.001, time.monotonic()-first),
                    backend='CUDA', model_name='YOLO11-GAM / best6.pt', model_sha256=detector.model_sha256,
                    class_names=detector.names, confidence_threshold=.25, source_metadata=source,
                    inference_input_sha256=hashlib.sha256(image.tobytes()).hexdigest(),
                    bev_detection_accuracy_verified=False, source_exposure_synchronized=False,
                    metric_accuracy_verified=False)
                header = json.dumps(meta, allow_nan=False).encode()
                packet = struct.pack('!4sIII', b'BV01', len(header), len(raw_jpeg), len(encoded))+header+raw_jpeg+encoded.tobytes()
                with LOCK:
                    LATEST = (source['produced_epoch_s'], packet)
                    STATUS.update(status='live', processed_frames=processed, source_id=identity,
                                  objects=len(objects), inference_ms=result.inference_ms,
                                  inference_fps=meta['inference_fps'], model_sha256=detector.model_sha256,
                                  result_epoch_s=meta['result_epoch_s'])
                if begin-last_export >= 2:
                    for name, content in [('latest_result.json', json.dumps(meta, ensure_ascii=False, indent=2)+'\n')]:
                        tmp = output/(name+'.tmp')
                        tmp.write_text(content)
                        tmp.replace(output/name)
                    cv2.imwrite(str(output/'latest_yolo.jpg'), annotated)
                    cv2.imwrite(str(output/'latest_bev.jpg'), image)
                    last_export = begin
                last_id = identity
            except (OSError, ValueError, KeyError) as exc:
                with LOCK:
                    STATUS.update(status='waiting', error=str(exc))
                if begin-last_error > 5:
                    print('YOLO_WAIT', str(exc), flush=True)
                    last_error = begin
            STOP.wait(max(0, .16-(time.monotonic()-begin)))
    finally:
        server.shutdown()
        server.server_close()
        if detector:
            detector.shutdown()


if __name__ == '__main__':
    main()
