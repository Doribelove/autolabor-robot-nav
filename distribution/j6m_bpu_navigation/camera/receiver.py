"""J6M private RGB-D/IMU endpoint and optional BPU observation worker; no ROS."""
import collections
import fcntl
import hashlib
import json
import os
from pathlib import Path
import queue
import signal
import socket
import struct
import sys
import threading
import time

CONFIG = json.loads(Path(sys.argv[1]).read_text()) if __name__ == '__main__' else {}


def atomic_json(path, value):
    path = Path(path)
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temp.replace(path)


def main(c):
    root = Path(c['board_root'])
    if root.parent != Path('/map/j6m-dev-b/camera/releases') or root.resolve() != root:
        raise RuntimeError('Use the private B camera release')
    state = root / 'state'
    state.mkdir(exist_ok=True)
    lock = (state / 'receiver.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    model_root = Path(c['model_root'])
    vendor = Path('/map/robot_j6m_navigation_20260907/bpu_lab_20260905/vendor')
    runtime = str(vendor / 'runtime_4_9_2/lib') + ':/usr/hobot/lib'
    if os.environ.get('LD_LIBRARY_PATH') != runtime:
        os.execve(sys.executable, [sys.executable, '-B', *sys.argv],
                  dict(os.environ, LD_LIBRARY_PATH=runtime, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1'))
    sys.path[:0] = [str(model_root / 'deps'), str(vendor / 'python311'), str(model_root / 'tools')]
    import cv2
    import numpy as np
    from camera_wire import receive, SequenceGuard, exact, imu_source_timing
    cv2.setNumThreads(1)
    done = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: done.set())
    signal.signal(signal.SIGINT, lambda *_: done.set())
    stats = collections.Counter()
    ring = {k: collections.deque(maxlen=9000) for k in ['frame', 'imu', 'imu_age', 'imu_read_age', 'imu_sdk_skew', 'age', 'pair', 'receive_decode', 'infer', 'result_age']}
    ring_lock = threading.Lock()
    frame_queue = queue.Queue(maxsize=1)
    sync = dict(good=False)
    latest = {}
    start = time.monotonic()

    def time_probe():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((c['j6m_ip'], 0)); sock.settimeout(.3)
        while not done.is_set():
            candidates = []
            for _ in range(5):
                try:
                    t1 = time.time_ns()
                    sock.sendto(json.dumps(dict(token=c['token'], t1=t1)).encode(), (c['agx_ip'], c['clock_port']))
                    value, peer = sock.recvfrom(4096); t4 = time.time_ns(); value = json.loads(value)
                    if peer != (c['agx_ip'], c['clock_port']) or value['t1'] != t1 or not value['synchronized']:
                        continue
                    delay = (t4-t1) - (value['t3']-value['t2'])
                    offset = ((value['t2']-t1) + (value['t3']-t4)) / 2
                    if 0 <= delay < 10_000_000:
                        candidates.append((delay, offset))
                except (OSError, ValueError, KeyError):
                    pass
            if candidates:
                delay, offset = min(candidates)
                sync.update(good=abs(offset) < 20_000_000, at=time.monotonic(), offset_ns=offset,
                            uncertainty_ns=delay/2, at_ns=time.time_ns())
            else:
                sync['good'] = False
            done.wait(2)
        sock.close()

    def age_ms(stamp):
        if not sync.get('good') or time.monotonic()-sync.get('at', 0) > 5:
            raise ValueError('clock_unsynchronized')
        age = (time.time_ns()+sync['offset_ns']-stamp)/1e6
        if not -5 <= age <= 500:
            raise ValueError('frame_age_out_of_bounds')
        return age

    preview = None
    if c.get('preview_port'):
        from preview_stream import PreviewServer
        preview = PreviewServer(c, cv2, age_ms)

    def imu_loop():
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        sock.bind((c['j6m_ip'], c['imu_port'])); sock.listen(2); sock.settimeout(.5)
        guard = SequenceGuard()
        client = None
        while not done.is_set():
            try:
                if client is None:
                    client, peer = sock.accept()
                    client.settimeout(.5)
                    if peer[0] != c['agx_ip']:
                        stats['imu_foreign_peer'] += 1;client.close();client=None;continue
                    stats['imu_connections'] += 1
                size, = struct.unpack('!I',exact(client,4))
                if not 1 <= size <= 8192:
                    raise ValueError('imu_packet_bounds')
                payload = exact(client,size)
                j = json.loads(payload)
                if (j.get('token') != c['token'] or j.get('serial') != c['serial'] or j.get('protocol') != 2
                        or j.get('frame_id') != 'zed2_imu_sdk_image_frame'):
                    raise ValueError('imu_identity')
                age_ms(j['host_read_end_ns'])  # Clock/transport gate, not a replacement acquisition stamp.
                timing = imu_source_timing(j,time.time_ns()+sync['offset_ns'])
                j['timing'] = timing
                imu_age = timing['sdk_source_age_ms']
                if not timing['fusion_time_valid']:
                    stats['imu_source_time_anomalies'] += 1
                stats['imu_sequence_gaps'] += guard.accept(j['session'], j['sequence'], j['stamp_ns'])
                for key, size in [('acceleration_m_s2', 3), ('angular_velocity_deg_s', 3), ('orientation_xyzw', 4)]:
                    if len(j[key]) != size or not np.isfinite(j[key]).all():
                        raise ValueError('imu_nonfinite')
                del j['token']; latest['imu'] = j
                with ring_lock:
                    ring['imu'].append(j['stamp_ns']/1e9)
                    ring['imu_age'].append(imu_age)
                    ring['imu_read_age'].append(timing['host_read_age_ms'])
                    ring['imu_sdk_skew'].append(timing['sdk_minus_host_read_ms'])
                stats['imu_received'] += 1
            except socket.timeout:
                if client is not None:client.close();client=None
                continue
            except (OSError,EOFError,ValueError,KeyError,TypeError) as e:
                stats['imu_rejected'] += 1; latest['imu_error'] = str(e)
                if client is not None:client.close();client=None
        if client is not None:client.close()
        sock.close()

    def inference_loop():
        if not c.get('inference', False):
            latest['inference_state'] = 'disabled'; return
        try:
            from native_tensor import TensorEngine, sha256
            from board_preprocess import detector_input, classifier_input
            from production_io import detector_output, classifier_output, validate_production_interface
            from vision_two_stage import context_crop
            reservation = Path('/map/robot_j6m_navigation_20260907/bpu_lab_20260905/preview_worker.lock').open('a')
            fcntl.flock(reservation, fcntl.LOCK_EX | fcntl.LOCK_NB)
            library = '/map/robot_j6m_navigation_20260907/bpu_production_20261010/build_v5/libj6m_tensor.so'
            if sha256(library) != '5c99bafb0531fac9f8e84c49c8c2d0a6d8e852e5c2217e62750dce93b4a14e3d':
                raise ValueError('Executor hash mismatch')
            with TensorEngine(library, model_root/'models/detector_zed_attempt03.hbm', 'detector_zed',
                              'f2f1f7e9753b53b0494495be44af92a0603b3ee6c71e69e0f58a0d50544a8cb8') as detector, \
                 TensorEngine(library, model_root/'models/classifier_attempt03.hbm', 'classifier',
                              'd4cb80f5f7364d1156ea2275eba9e522991dec309f2c397e74649254fe36a89f') as classifier:
                validate_production_interface(detector.metadata, 'detector_zed')
                validate_production_interface(classifier.metadata, 'classifier')
                latest['inference_state'] = 'BPU_observation_only'
                materials = ['metal', 'plastic', 'paper', 'glass', 'kitchen_waste']
                while not done.is_set():
                    try:
                        h, image, depth = frame_queue.get(timeout=.5)
                    except queue.Empty:
                        continue
                    try:
                        age_ms(h['color_stamp_ns'])
                        t = time.monotonic()
                        tensor, geometry = detector_input(image)
                        outputs, timing = detector.infer([tensor], timeout_ms=200)
                        boxes = detector_output(outputs[0], geometry)
                        objects = []
                        # Bounded observation budget, explicitly reported when truncating.
                        for box in boxes[:8]:
                            age_ms(h['color_stamp_ns'])
                            crop, bounds = context_crop(image, box[:4], context_fraction=.20)
                            if crop is None or crop.size == 0:
                                continue
                            values, ct = classifier.infer([classifier_input(crop)], timeout_ms=200)
                            probabilities = classifier_output(values[0]); index = int(probabilities.argmax())
                            x1,y1,x2,y2 = box[:4]
                            cx, cy = int((x1+x2)/2), int((y1+y2)/2)
                            patch = depth[max(0,cy-2):min(720,cy+3),max(0,cx-2):min(1280,cx+3)]
                            valid = patch[np.isfinite(patch) & (patch > 0)]
                            xyz = None
                            if valid.size:
                                z = float(np.median(valid)); k = h['calibration']['left_rectified']
                                xyz = [(cx-k['cx'])*z/k['fx'], (cy-k['cy'])*z/k['fy'], z]
                            objects.append(dict(box=box.tolist(), crop_bounds=list(bounds), material=materials[index],
                                                probabilities=probabilities.tolist(), optical_xyz_m=xyz,
                                                depth_method='center 5x5 finite positive median; observation only',
                                                classifier_timing=ct))
                        elapsed = (time.monotonic()-t)*1000
                        result_age = age_ms(h['color_stamp_ns'])
                        result = dict(session=h['session'], sequence=h['sequence'], source_stamp_ns=h['color_stamp_ns'],
                                      frame_id=h['frame_id'], at_ns=time.time_ns(), objects=objects,
                                      detections=boxes.tolist(),
                                      detected_count=len(boxes), unclassified_count=max(0,len(boxes)-8),
                                      processing_ms=elapsed, source_age_ms=result_age, detector_timing=timing,
                                      tracking_implemented=False, production_ready=False, motion_eligible=False)
                        atomic_json(state/'latest_result.json', result)
                        if objects:
                            # Historical observation for inspection, never a replacement for latest_result.
                            atomic_json(state/'latest_nonempty_result.json', result)
                        stats['inferred_frames'] += 1; stats['classified_objects'] += len(objects)
                        with ring_lock:
                            ring['infer'].append(elapsed); ring['result_age'].append(result_age)
                        latest['result'] = result
                        if preview is not None:
                            preview.publish(image, result)
                    except ValueError as e:
                        stats['inference_rejected'] += 1; latest['inference_error'] = str(e)
        except Exception as e:
            latest['inference_state'] = 'failed'; latest['inference_error'] = repr(e)
            stats['inference_fatal'] += 1

    def status_loop():
        while not done.wait(1):
            with ring_lock:
                samples = {k:list(v) for k,v in ring.items()}
            def hz(a):
                return (len(a)-1)/(a[-1]-a[0]) if len(a)>1 and a[-1]>a[0] else None
            metrics = {}
            for name in ['frame','imu']:
                samples[name+'_interval'] = np.diff(samples[name])*1000
            for key in ['age','imu_age','imu_read_age','imu_sdk_skew','pair','receive_decode','infer','result_age','frame_interval','imu_interval']:
                a = samples[key]
                metrics[key+'_ms'] = dict(zip(['p50','p95','p99','max'],map(float,np.percentile(a,[50,95,99,100])))) if len(a) else None
            now = time.time_ns()
            last_receive = latest.get('last_receive_monotonic', 0)
            status = dict(at_ns=now, pid=os.getpid(), elapsed_s=time.monotonic()-start, counters=dict(stats),
                          rgbd_hz=hz(samples['frame']), imu_hz=hz(samples['imu']), clock=dict(sync), metrics=metrics,
                          state='streaming' if time.monotonic()-last_receive < .5 else 'waiting_for_fresh_data',
                          latest={k:v for k,v in latest.items() if k != 'result'},
                          production_ready=False, motion_eligible=False)
            atomic_json(state/'receiver.json',status)
            if 'imu' in latest:
                atomic_json(state/'latest_imu.json',latest['imu'])
    threads = [threading.Thread(target=f,daemon=True) for f in [time_probe,imu_loop,inference_loop,status_loop]]
    for th in threads: th.start()
    server = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    server.bind((c['j6m_ip'],c['image_port'])); server.listen(2); server.settimeout(.5)
    guard = SequenceGuard(); saved=(state/'proof_metadata.json').is_file()
    calibrations = {}
    try:
        while not done.is_set():
            try:
                client, peer = server.accept()
            except socket.timeout:
                continue
            if peer[0] != c['agx_ip']:
                client.close(); stats['foreign_peer'] += 1; continue
            stats['connections'] += 1; client.settimeout(.5)
            with client:
                while not done.is_set():
                    try:
                        begin = time.monotonic()
                        h,color,raw_depth,wire_bytes = receive(client,c['token'])
                        age = age_ms(h['color_stamp_ns'])
                        calibration_hash = hashlib.sha256(json.dumps(h['calibration'],sort_keys=True).encode()).hexdigest()
                        if h['session'] in calibrations and calibrations[h['session']] != calibration_hash:
                            raise ValueError('Calibration changed within capture session')
                        stats['frame_sequence_gaps'] += guard.accept(h['session'],h['sequence'],h['color_stamp_ns'])
                        if h['session'] not in calibrations:
                            atomic_json(state/'current_calibration.json',dict(session=h['session'],
                                        source_stamp_ns=h['color_stamp_ns'],sha256=calibration_hash,
                                        calibration=h['calibration']))
                        calibrations[h['session']] = calibration_hash
                        color = np.frombuffer(color,dtype=np.uint8).reshape(720,1280,4)
                        depth = np.frombuffer(raw_depth,dtype='<f4').reshape(720,1280)
                        decode_ms=(time.monotonic()-begin)*1000
                        stats['frames_received'] += 1; stats['wire_bytes'] += wire_bytes; stats['crc_verified_planes'] += 2
                        with ring_lock:
                            ring['frame'].append(h['color_stamp_ns']/1e9); ring['age'].append(age)
                            ring['pair'].append(abs(h['color_stamp_ns']-h['depth_stamp_ns'])/1e6); ring['receive_decode'].append(decode_ms)
                        latest.update(session=h['session'],sequence=h['sequence'],source_stamp_ns=h['color_stamp_ns'],
                                      calibration_sha256=calibration_hash,
                                      last_receive_monotonic=time.monotonic(),depth_finite_fraction=float(np.isfinite(depth).mean()))
                        if not saved:
                            cv2.imwrite(str(state/'proof_color.png'),color)
                            np.save(state/'proof_depth.npy',depth,allow_pickle=False)
                            atomic_json(state/'proof_metadata.json',h)
                            atomic_json(state/'calibration.json',h['calibration']);saved=True
                        if c.get('inference',False):
                            try:
                                frame_queue.put_nowait((h,np.ascontiguousarray(color[:,:,:3]),depth))
                            except queue.Full:
                                try:frame_queue.get_nowait()
                                except queue.Empty:pass
                                stats['inference_slot_overwritten'] += 1
                                try:frame_queue.put_nowait((h,np.ascontiguousarray(color[:,:,:3]),depth))
                                except queue.Full:pass
                    except (OSError,EOFError,ValueError,KeyError,TypeError) as e:
                        stats['connection_ended'] += 1; latest['connection_error']=str(e)
                        break
    finally:
        done.set();server.close()
        if preview is not None:
            preview.close()
        for th in threads:th.join(timeout=3)


if __name__ == '__main__':
    main(CONFIG)
