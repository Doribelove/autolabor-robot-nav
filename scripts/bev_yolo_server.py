#!/usr/bin/env python3
"""Detect in original camera views, localize ground contacts, publish one BEV result."""
import argparse
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
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
sys.path.insert(0, str(PROJECT/'src/application/autolabor_fod_control/src'))
sys.path.insert(0, str(PROJECT/'distribution/j6m_bpu_navigation'))
from surround.ground_protocol import decode_view_frame
from surround.ground_detections import localize_detections, render_ground_detections
from surround.image_detector import CudaViewsDetector, BpuViewsDetector, FcosViewsDetector
from surround.detection_views import DetectionMasks, encode_vision_frame

STOP = threading.Event()
LOCK = threading.Lock()
LATEST = None
OBSERVATIONS = None
STATUS = {'status': 'loading_detector'}


def read_ground_frame(url):
    with urllib.request.urlopen(url, timeout=2) as response:
        data = response.read(11*1024*1024)
    return decode_view_frame(data)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def do_GET(self):
        with LOCK:
            latest, status, observations = LATEST, dict(STATUS), OBSERVATIONS
        if self.path == '/state':
            payload = json.dumps(status, allow_nan=False).encode()
            mime = 'application/json'
        elif self.path == '/frame' and latest and time.time()-latest[0] < 2:
            payload, mime = latest[1], 'application/octet-stream'
        elif self.path == '/vision_frame' and latest and time.time()-latest[0] < 2:
            payload, mime = latest[2], 'application/octet-stream'
        elif self.path == '/detections' and observations and -.2 <= time.time()-observations['observed_epoch_s'] <= 2:
            payload, mime = json.dumps(observations, allow_nan=False).encode(), 'application/json'
        else:
            self.send_error(503, 'Waiting for fresh detector BEV result')
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
    global LATEST, OBSERVATIONS
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', default='http://127.0.0.1:18883/views')
    parser.add_argument('--port', type=int, default=18884)
    parser.add_argument('--backend', choices=('cuda', 'bpu', 'fcos_bpu'), default=os.environ.get('BEV_DETECTOR_BACKEND', 'cuda'))
    parser.add_argument('--output', default=str(PROJECT/'runtime/bev_qt'))
    parser.add_argument('--mask-config', default=str(PROJECT/'config/bev_detection_masks.json'))
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    cv2.setNumThreads(1)
    masks = DetectionMasks(args.mask_config)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: STOP.set())
    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    detector = None
    try:
        detector = ({'bpu': BpuViewsDetector, 'fcos_bpu': FcosViewsDetector}[args.backend](log_path=output/'bpu.log')
                    if args.backend != 'cuda' else CudaViewsDetector())
        print('VIEWS_DETECTOR_READY', detector.backend, detector.names, detector.model_sha256, flush=True)
        last_id, last_export, processed, first = None, 0, 0, time.monotonic()
        last_error = 0
        while not STOP.is_set():
            begin = time.monotonic()
            try:
                source, image, raw_jpeg, views = read_ground_frame(args.input)
                identity = dict(session=source['fusion_session'], sequence=source['fusion_sequence'])
                if identity == last_id:
                    STOP.wait(.04)
                    continue
                infer_begin = time.monotonic()
                detector_views = masks.apply(views)
                camera_objects, camera_timing = detector.predict_views(detector_views, identity)
                camera_objects, excluded = masks.filter(camera_objects)
                inference_ms = (time.monotonic()-infer_begin)*1000
                ground, rejected = localize_detections(source, camera_objects,
                    merge_material_classes=detector.merge_material_classes)
                cleaning = [o for o in ground if detector.cleaning_classes is None or o['label'] in detector.cleaning_classes]
                if time.time()-source['produced_epoch_s'] > 2:
                    raise ValueError('BEV source expired during inference')
                annotated, objects = render_ground_detections(image, source, ground)
                ok, encoded = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 94])
                if not ok:
                    raise ValueError('Annotated image encoding failed')
                # Source capture clocks are not synchronized. Use a conservative
                # receive/cache-time estimate and expose that limitation.
                observed = min(s['received_epoch_ns']/1e9 -
                    s.get('source_age_s', s.get('producer_cache_age_s', 0.))
                    for s in source['sources'].values())
                observations = dict(schema='conbine.bev.cleaning.observations.v1',
                    source_id=identity, config_sha256=source['config_sha256'],
                    ground_frame=source['frame_id'], observed_epoch_s=observed,
                    configured_cameras=sorted(source['sources']),
                    camera_count=len(source['sources']),
                    detection_mask_sha256=masks.sha256,
                    timestamp_basis='oldest source receive time minus reported source/cache age estimate',
                    ground_objects=cleaning, scene_ground_objects=ground,
                    cleaning_candidate_classes=sorted(detector.cleaning_classes) if detector.cleaning_classes is not None else None,
                    position_method='ground_contact_ray_plane',
                    association_method='ground_distance_and_mutual_box_reprojection',
                    metric_accuracy_verified=False, source_exposure_synchronized=False)
                processed += 1
                if processed == 1:
                    first = time.monotonic()
                meta = dict(schema='conbine.bev.yolo.frame.v1', source_id=identity,
                    image_source_id=identity, result_source_id=identity,
                    produced_epoch_s=source['produced_epoch_s'], result_epoch_s=time.time(),
                    width=image.shape[1], height=image.shape[0], objects=objects,
                    inference_ms=inference_ms, inference_fps=(processed-1)/max(.001, time.monotonic()-first),
                    backend=detector.backend, model_name=detector.model_name, model_sha256=detector.model_sha256,
                    source_checkpoint_sha256=detector.checkpoint_sha256,
                    inference_image_space='original_upright_cameras', fusion_method='ground_observation_fusion',
                    camera_detections=camera_objects, camera_timing=camera_timing, rejected_localizations=rejected,
                    excluded_detections=excluded, detection_masks=masks.regions, detection_mask_sha256=masks.sha256,
                    class_names=detector.names, confidence_threshold=detector.confidence, source_metadata=source,
                    detector_metadata=getattr(detector, 'metadata', {}),
                    cleaning_candidate_classes=observations['cleaning_candidate_classes'],
                    inference_input_sha256=hashlib.sha256(b''.join(detector_views[n].tobytes() for n in sorted(detector_views))).hexdigest(),
                    bev_detection_accuracy_verified=False, source_exposure_synchronized=False,
                    metric_accuracy_verified=False)
                meta['ground_objects'] = observations['ground_objects']
                meta['scene_ground_objects'] = ground
                meta['latency'] = dict(fusion_to_result_ms=(meta['result_epoch_s']-source['produced_epoch_s'])*1000,
                    estimated_observation_to_result_ms=(meta['result_epoch_s']-observed)*1000,
                    timestamp_basis=observations['timestamp_basis'], exposure_latency_verified=False)
                meta['ground_association'] = dict(method=observations['association_method'],
                    input_boxes=sum(len(v) for v in camera_objects.values()), rejected=len(rejected),
                    valid_observations=sum(len(o['members']) for o in ground), unique_targets=len(ground),
                    merged_duplicates=sum(len(o['members'])-1 for o in ground), merge_radius_m=.15)
                header = json.dumps(meta, allow_nan=False).encode()
                packet = struct.pack('!4sIII', b'BV01', len(header), len(raw_jpeg), len(encoded))+header+raw_jpeg+encoded.tobytes()
                previews = masks.annotate(views,camera_objects)
                vision_packet = encode_vision_frame(meta,raw_jpeg,encoded.tobytes(),previews)
                with LOCK:
                    LATEST = (source['produced_epoch_s'], packet, vision_packet)
                    OBSERVATIONS = observations
                    STATUS.update(status='live', processed_frames=processed, source_id=identity,
                                  backend=detector.backend, model_name=detector.model_name,
                                  inference_image_space='original_upright_cameras',
                                  configured_cameras=sorted(source['sources']), camera_count=len(source['sources']),
                                  objects=len(objects), input_boxes=meta['ground_association']['input_boxes'],
                                  cleaning_candidates=len(cleaning), inference_ms=inference_ms,
                                  inference_fps=meta['inference_fps'], model_sha256=detector.model_sha256,
                                  result_epoch_s=meta['result_epoch_s'])
                    STATUS.pop('error',None)
                if begin-last_export >= 2:
                    for name, content in [('latest_result.json', json.dumps(meta, ensure_ascii=False, indent=2)+'\n')]:
                        tmp = output/(name+'.tmp')
                        tmp.write_text(content)
                        tmp.replace(output/name)
                    cv2.imwrite(str(output/'latest_yolo.jpg'), annotated)
                    cv2.imwrite(str(output/'latest_bev.jpg'), image)
                    for name, preview in previews.items():
                        cv2.imwrite(str(output/('latest_'+name+'.jpg')), preview)
                    last_export = begin
                last_id = identity
            except (OSError, ValueError, KeyError, RuntimeError, EOFError) as exc:
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
