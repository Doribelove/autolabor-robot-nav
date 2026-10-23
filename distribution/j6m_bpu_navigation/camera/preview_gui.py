"""AGX display only: RGB and matching results returned by the J6M BPU worker."""
import argparse
import collections
import http.client
import json
import os
from pathlib import Path
import signal
import threading
import time

from PyQt5 import QtCore, QtGui, QtWidgets
from preview_stream import unpack_preview, MAX_IMAGE, MAX_METADATA, PREFIX

MATERIAL_NAMES = ('金属', '塑料', '纸类', '玻璃', '厨余')

class Window(QtWidgets.QWidget):
    def __init__(self, config, state_file):
        super().__init__()
        self.config, self.state_file = config, state_file
        self.done = threading.Event()
        self.lock = threading.Lock()
        self.latest, self.error = None, '正在连接 J6M…'
        self.displayed = None
        self.image = None
        self.meta = None
        self.arrivals = collections.deque(maxlen=60)
        self.last_report = 0
        self.received = 0
        self.stale = True
        self.setWindowTitle('ZED RGB + YOLO11 | J6M BPU')
        self.setStyleSheet('QWidget {background:#101820;color:#e9f0f5;font-size:15px;} '
                           'QPushButton {padding:6px 14px;background:#263744;border-radius:4px;}')
        layout = QtWidgets.QVBoxLayout(self)
        top = QtWidgets.QHBoxLayout()
        heading = QtWidgets.QLabel('ZED RGB  ·  YOLO11-GAM  ·  J6M BPU')
        heading.setStyleSheet('font-size:21px;font-weight:600;padding:6px;')
        top.addWidget(heading)
        top.addStretch()
        self.boxes = QtWidgets.QCheckBox('显示检测框')
        self.boxes.setChecked(True)
        self.boxes.toggled.connect(self.render)
        top.addWidget(self.boxes)
        self.rotate = QtWidgets.QPushButton('旋转 180° [R]')
        self.rotate.setCheckable(True)
        self.rotate.toggled.connect(self.render)
        top.addWidget(self.rotate)
        layout.addLayout(top)
        self.canvas = QtWidgets.QLabel('等待 J6M 新鲜推理画面…')
        self.canvas.setAlignment(QtCore.Qt.AlignCenter)
        self.canvas.setMinimumSize(640, 360)
        self.canvas.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
        layout.addWidget(self.canvas, 1)
        self.status = QtWidgets.QLabel('正在连接 J6M BPU 推理服务…')
        self.status.setStyleSheet('padding:5px;')
        layout.addWidget(self.status)
        hint = QtWidgets.QLabel('画面与检测结果按同一采集帧配对  |  Q / Esc 关闭预览  |  后台采集与推理继续运行')
        hint.setStyleSheet('color:#9baeba;font-size:12px;padding:3px;')
        layout.addWidget(hint)
        self.resize(1320, 840)
        self.move(70, 60)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(25)
        self.worker = threading.Thread(target=self.fetch, daemon=True)
        self.worker.start()

    def fetch(self):
        while not self.done.is_set():
            started = time.monotonic()
            connection = http.client.HTTPConnection(self.config['j6m_ip'], self.config['preview_port'],
                                                    timeout=.5, source_address=(self.config['agx_ip'], 0))
            try:
                connection.request('GET', '/frame', headers={'Authorization': 'Bearer ' + self.config['token']})
                response = connection.getresponse()
                size = int(response.getheader('Content-Length', '0'))
                if response.status != 200 or not 0 < size <= MAX_IMAGE + MAX_METADATA + PREFIX.size:
                    raise ValueError('J6M 暂无新鲜 BPU 结果 (HTTP %s)' % response.status)
                payload = response.read(size)
                meta, jpeg = unpack_preview(payload)
                age = (time.time_ns() - meta['result']['source_stamp_ns']) / 1e6
                if not -5 <= age <= 500:
                    raise ValueError('推理画面已过期')
                with self.lock:
                    self.latest = (meta, jpeg, time.monotonic())
                    self.error = ''
            except (OSError, ValueError, KeyError, TypeError, http.client.HTTPException) as exc:
                with self.lock:
                    self.error = str(exc)
                self.done.wait(.1)
            finally:
                connection.close()
            self.done.wait(max(0, 1 / 30 - (time.monotonic() - started)))

    def refresh(self):
        with self.lock:
            pair, error = self.latest, self.error
        now = time.monotonic()
        age = None
        if pair:
            meta, jpeg, received_at = pair
            age = (time.time_ns() - meta['result']['source_stamp_ns']) / 1e6
        live = bool(pair and -5 <= age <= 500 and now - received_at <= .5)
        if live:
            result = meta['result']
            identity = (result['session'], result['sequence'])
            if identity != self.displayed or self.stale:
                image = QtGui.QImage.fromData(jpeg, 'JPEG')
                if image.isNull() or image.width() != 1280 or image.height() != 720:
                    self.latest = None
                    return
                self.image, self.meta = image, meta
                self.displayed = identity
                self.arrivals.append(now)
                self.received += 1
                self.stale = False
                self.render()
            fps = ((len(self.arrivals)-1) / (self.arrivals[-1]-self.arrivals[0])
                   if len(self.arrivals) > 1 else 0)
            message = ('BPU 在线  |  显示 %.1f FPS  |  J6M 处理 %.1f ms  |  采集至显示 %.0f ms'
                       '  |  目标 %d  |  帧 #%d' % (fps, result['processing_ms'], age,
                                                   result['detected_count'], result['sequence']))
            self.status.setStyleSheet('color:#6fdfad;padding:5px;')
        else:
            self.stale = True
            self.canvas.clear()
            self.canvas.setText('等待 J6M 新鲜推理画面…\n旧帧检测框已清除')
            self.status.setStyleSheet('color:#ffb16e;padding:5px;')
            message = error or '数据超过 500 ms，等待恢复'
        self.status.setText(message)
        if now - self.last_report >= 1:
            self.last_report = now
            print(time.strftime('%H:%M:%S'), message, flush=True)
            if self.state_file:
                data = dict(at_ns=time.time_ns(), pid=os.getpid(), live=live, displayed_frames=self.received,
                            status=message, sequence=self.displayed, source_age_ms=age,
                            backend='J6M_BPU', result=self.meta['result'] if self.meta and live else None)
                tmp = self.state_file.with_suffix('.tmp')
                tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2))
                tmp.replace(self.state_file)

    def render(self, *_):
        if self.image is None or self.stale:
            return
        image = self.image.copy()
        if self.boxes.isChecked():
            painter = QtGui.QPainter(image)
            painter.setRenderHint(QtGui.QPainter.Antialiasing)
            font = QtGui.QFont('Noto Sans CJK SC', 13)
            painter.setFont(font)
            result = self.meta['result']
            classes = {tuple(o['box'][:4]): o for o in result['objects']}
            for box in result.get('detections', [o['box'] for o in result['objects']]):
                x1,y1,x2,y2,confidence = box[:5]
                obj = classes.get(tuple(box[:4]))
                title = '检测置信度 %.1f%%' % (100 * confidence)
                if obj:
                    probabilities = obj['probabilities']
                    best = max(range(len(probabilities)), key=probabilities.__getitem__)
                    title += ' | 分类：%s %.1f%%' % (MATERIAL_NAMES[best], 100 * probabilities[best])
                    if obj.get('optical_xyz_m'):
                        title += ' · %.2f m' % obj['optical_xyz_m'][2]
                else:
                    title += ' | 分类：待分类'
                painter.setPen(QtGui.QPen(QtGui.QColor('#46efab'), 3))
                painter.drawRect(QtCore.QRectF(x1, y1, x2-x1, y2-y1))
                width = painter.fontMetrics().horizontalAdvance(title) + 16
                tx = max(0, min(x1, 1280-width))
                ty = max(0, y1-30)
                painter.fillRect(QtCore.QRectF(tx,ty,width,29), QtGui.QColor('#123f32'))
                painter.drawText(QtCore.QPointF(tx+8,ty+21), title)
            painter.end()
        if self.rotate.isChecked():
            image = image.transformed(QtGui.QTransform().rotate(180))
        self.canvas.setPixmap(QtGui.QPixmap.fromImage(image).scaled(
            self.canvas.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.render()

    def keyPressEvent(self, event):
        if event.key() in (QtCore.Qt.Key_Escape, QtCore.Qt.Key_Q):
            self.close()
        elif event.key() == QtCore.Qt.Key_R:
            self.rotate.toggle()
        else:
            super().keyPressEvent(event)

    def closeEvent(self, event):
        self.done.set()
        self.timer.stop()
        event.accept()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('config', type=Path)
    parser.add_argument('--state', type=Path)
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    config.setdefault('preview_port', 17446)
    app = QtWidgets.QApplication([])
    app.setApplicationName('J6M BPU Camera Preview')
    window = Window(config, args.state)
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    window.show()
    window.raise_()
    window.activateWindow()
    print('ZED RGB + YOLO11-GAM：检测和材质分类均复用 J6M BPU 服务。Q / Esc 关闭预览。', flush=True)
    return app.exec_()


if __name__ == '__main__':
    raise SystemExit(main())
