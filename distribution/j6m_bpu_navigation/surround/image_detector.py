"""Original-image detector backends. Camera geometry/fusion stays on CPU."""
import hashlib
import re
import shlex
import subprocess
import sys
import time
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'distribution/j6m_bpu_navigation/tools'))
from tensor_wire import send, receive


class CudaViewsDetector:
    backend = 'CUDA'
    model_name = 'YOLO11-GAM / best6.pt / original views'
    merge_material_classes = True
    cleaning_classes = None

    def __init__(self, confidence=.25, image_size=640):
        self.confidence = confidence
        sys.path.insert(0, str(ROOT/'src/application/autolabor_fod_vision/src'))
        from autolabor_fod_vision.detector import UltralyticsDetector
        self.detector = UltralyticsDetector(weights=str(ROOT/'src/application/autolabor_fod_vision/models/best6.pt'),
            ultralytics_root=str(ROOT/'ultralytics_yolo11_custom'), device='0', image_size=image_size,
            confidence=confidence, iou=.45, max_detections=25, require_gam=True,
            expected_sha256='5efaafa1503db11c2ba261b4429389d96335b4eef4d0fc44d6ca41e7431f2d0f')
        self.model_sha256, self.names = self.detector.model_sha256, self.detector.names
        self.checkpoint_sha256 = self.model_sha256

    def predict_views(self, images, source_id):
        results, timing = {}, {}
        for name, image in images.items():
            result = self.detector.predict(image)
            results[name] = [dict(class_id=d.class_id, class_name=d.class_name, confidence=d.confidence,
                bbox=[d.xmin,d.ymin,d.xmax,d.ymax], anchor=[d.anchor_u,d.anchor_v]) for d in result.detections]
            timing[name] = dict(model_call_ms=result.inference_ms)
        return results, timing

    def shutdown(self):
        self.detector.shutdown()


class BpuViewsDetector:
    backend = 'BPU'
    model_name = 'YOLO11-GAM / existing trash HBM / original views'
    model_sha256 = 'f2f1f7e9753b53b0494495be44af92a0603b3ee6c71e69e0f58a0d50544a8cb8'
    names = {0:'trash'}
    worker_script = 'bev_bpu_worker.py'
    extra_files = ()
    merge_material_classes = True
    cleaning_classes = None

    def __init__(self, confidence=.25, log_path=None):
        self.process, self.log = None, None
        self.confidence = confidence
        ssh = ['ssh','-o','BatchMode=yes','-o','ConnectTimeout=5','root@192.168.10.100']
        board = subprocess.check_output(ssh+['mktemp -d /tmp/conbine-ground-detector-XXXXXXXX'],text=True,timeout=10).strip()
        if not re.fullmatch('/tmp/conbine-ground-detector-[A-Za-z0-9]+', board):
            raise ValueError('Invalid board temporary directory')
        self.board_directory = board
        try:
            subprocess.run(['scp','-q',str(ROOT/'scripts'/self.worker_script),
                            str(ROOT/'distribution/j6m_bpu_navigation/tools/tensor_wire.py'),
                            *[str(ROOT/path) for path in self.extra_files],
                            'root@192.168.10.100:'+board+'/'],check=True,timeout=20)
            if log_path:
                Path(log_path).parent.mkdir(parents=True,exist_ok=True)
                self.log = Path(log_path).open('a')
            self.process = subprocess.Popen(ssh+[shlex.join(['python3','-B','-u',board+'/'+self.worker_script])],
                stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log,bufsize=0)
            ready, payload = receive(self.process.stdout,timeout=15)
            if payload or ready.get('ready') is not True or ready.get('model_sha256') != self.model_sha256:
                raise ValueError('Unexpected BPU model identity')
            self.checkpoint_sha256, self.metadata = ready['checkpoint_sha256'], ready['metadata']
        except Exception:
            self.shutdown()
            raise

    def predict_views(self, images, source_id):
        rows, chunks = [], []
        for name, image in images.items():
            ok, jpeg = cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,94])
            if not ok:
                raise ValueError('JPEG encode failed')
            data = jpeg.tobytes(); chunks.append(data)
            if len(data) > 2*1024*1024:
                raise ValueError('Camera JPEG exceeds 2 MiB; resize non-camera diagnostic images first')
            rows.append(dict(camera=name,bytes=len(data),size=list(image.shape[1::-1]),
                             sha256=hashlib.sha256(data).hexdigest()))
        begin = time.monotonic()
        send(self.process.stdin,dict(source_id=source_id,views=rows,confidence=self.confidence),b''.join(chunks),timeout=3)
        header, payload = receive(self.process.stdout,timeout=5)
        if (payload or header.get('source_id') != source_id or header.get('model_sha256') != self.model_sha256
                or set(header.get('results',{})) != set(images)):
            raise ValueError('BPU image/result identity mismatch')
        results, timing = {}, {}
        for name,row in header['results'].items():
            results[name] = self.convert_boxes(row['boxes'])
            timing[name] = row['timing']
        self.last_roundtrip_ms = (time.monotonic()-begin)*1000
        return results,timing

    def convert_boxes(self, boxes):
        return [dict(class_id=0,class_name='trash',confidence=score,bbox=[x0,y0,x1,y1])
                for x0,y0,x1,y1,score in boxes]

    def shutdown(self):
        p,self.process = self.process,None
        if p:
            if p.stdin: p.stdin.close()
            try: p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                p.terminate()
                try: p.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    p.kill(); p.wait(timeout=2)
            if p.stdout: p.stdout.close()
        if self.log:
            self.log.close(); self.log=None


class FcosViewsDetector(BpuViewsDetector):
    """Four RGB views, one resident Horizon FCOS model on J6M BPU."""
    from .fcos_image import CLASS_NAMES
    model_name = 'Horizon FCOS-EfficientNet-B3 / COCO80 / J6M BPU'
    model_sha256 = '1daacc7dfde181b65872c47f4fe5db84fd31f19d3ad56aa1fa1d902bb5ffd7fa'
    names = dict(enumerate(CLASS_NAMES))
    worker_script = 'bev_fcos_worker.py'
    extra_files = ('distribution/j6m_bpu_navigation/surround/fcos_image.py',
                   'experiments/j6m_bpu_20260905/tools/resident_fcos.py',
                   'experiments/j6m_bpu_20260905/tools/decode_fcos_demo.py',
                   'experiments/j6m_bpu_20260905/tools/live_protocol.py')
    merge_material_classes = False
    cleaning_classes = frozenset(('bottle', 'cup'))

    def __init__(self, confidence=.20, log_path=None):
        super().__init__(confidence=confidence, log_path=log_path)

    def convert_boxes(self, boxes):
        return [dict(class_id=int(label), class_name=self.names[int(label)],
                     confidence=score, bbox=[x0,y0,x1,y1])
                for x0,y0,x1,y1,score,label in boxes]
