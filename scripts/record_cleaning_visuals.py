#!/usr/bin/env python3
"""Record the existing HTTP image streams and Qt window without controlling ROS."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import struct
import subprocess
import threading
import time
import urllib.request

import genpy
import rosbag
from std_msgs.msg import UInt8MultiArray
from PyQt5.QtCore import QBuffer, QIODevice, QTimer, Qt
from PyQt5.QtGui import QImage, QPainter
from PyQt5.QtWidgets import QApplication


def packet_identity(data, kind):
    if kind == 'vision':
        magic, nj, nr, na = struct.unpack('!4sIII', data[:16])
        if magic != b'BV02' or not 0 < nj <= 128*1024 or 16+nj+nr+na > len(data):
            raise ValueError('Invalid BV02 packet')
        meta = json.loads(data[16:16+nj])
        identity = meta['source_id']
        return (identity['session'], identity['sequence']), meta['produced_epoch_s']
    magic, nj, nb = struct.unpack('!4sII', data[:12])
    if magic != b'MV01' or not 0 < nj <= 128*1024 or len(data) != 12+nj+nb:
        raise ValueError('Invalid MV01 packet')
    offset = 12+nj
    magic, nj, ni = struct.unpack('!4sII', data[offset:offset+12])
    if magic != b'BG01' or not 0 < nj <= 128*1024:
        raise ValueError('Invalid embedded BEV packet')
    meta = json.loads(data[offset+12:offset+12+nj])
    return (meta['fusion_session'], meta['fusion_sequence']), meta['produced_epoch_s']


class PacketRecorder:
    def __init__(self, root):
        self.root = root
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.bag = None
        self.segment = -1
        self.rows = {}
        self.manifest = (root/'packets.jsonl').open('a', buffering=1)
        self.threads = []
        for name, url in [('views', 'http://127.0.0.1:18883/views'),
                          ('vision', 'http://127.0.0.1:18884/vision_frame')]:
            worker = threading.Thread(target=self.poll, args=(name, url), daemon=True)
            self.threads.append(worker)
            worker.start()

    def close_segment(self):
        if self.bag is not None:
            self.bag.close()
            self.active_path.rename(self.active_path.with_suffix(''))
            self.bag = None

    def poll(self, name, url):
        previous = None
        count = 0
        missed = 0
        while not self.stop.is_set():
            begin = time.monotonic()
            try:
                requested = time.time()
                with urllib.request.urlopen(url, timeout=2) as response:
                    data = response.read(16*1024*1024+1)
                received = time.time()
                if len(data) > 16*1024*1024:
                    raise ValueError('Oversized packet')
                identity, produced = packet_identity(data, name)
                if identity != previous:
                    # Fusion IDs can skip in the detector itself. Preserve gaps,
                    # rather than implying sensor-rate, lossless acquisition.
                    gap = max(0, identity[1]-previous[1]-1) if previous and identity[0] == previous[0] else 0
                    missed += gap
                    with self.lock:
                        if self.bag is None or time.monotonic()-self.segment_started >= 60:
                            self.close_segment()
                            self.segment += 1
                            self.active_path = self.root/('visual_%04d.bag.active' % self.segment)
                            self.bag = rosbag.Bag(str(self.active_path), 'w', compression='lz4')
                            self.segment_started = time.monotonic()
                        topic = '/experiment/http/'+name
                        self.bag.write(topic, UInt8MultiArray(data=data), genpy.Time.from_sec(received))
                        self.manifest.write(json.dumps(dict(topic=topic, bag=self.active_path.with_suffix('').name,
                            source_id=identity, requested_epoch_s=requested, received_epoch_s=received,
                            produced_epoch_s=produced, source_sequence_gap=gap, bytes=len(data),
                            sha256=hashlib.sha256(data).hexdigest()))+'\n')
                        count += 1
                        self.rows[name] = dict(frames=count, source_id=identity, bytes=len(data),
                            last_received_epoch_s=received, source_sequence_gaps=missed, error=None)
                    previous = identity
            except Exception as exc:
                with self.lock:
                    self.rows[name] = dict(self.rows.get(name, {}), error=str(exc), error_epoch_s=time.time())
            self.stop.wait(max(0, .1-(time.monotonic()-begin)))

    def status(self):
        with self.lock:
            return dict(self.rows)

    def close(self):
        self.stop.set()
        for worker in self.threads:
            worker.join()
        self.close_segment()
        self.manifest.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    root = Path(args.output)
    root.mkdir(parents=True, exist_ok=True)
    app = QApplication([])
    app.setQuitOnLastWindowClosed(False)
    windows = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True)
    match = re.search(r'(0x[0-9a-f]+) "Autolabor 合并版 · 真实导航"', windows)
    if not match:
        raise RuntimeError('Live Qt window was not found')
    window_id = int(match[1], 16)
    initial = app.primaryScreen().grabWindow(window_id)
    if initial.isNull():
        raise RuntimeError('Cannot capture Qt window')
    size = initial.size()
    fps = 5
    video_log = (root/'video.log').open('ab', buffering=0)
    ffmpeg = subprocess.Popen(['ffmpeg', '-hide_banner', '-loglevel', 'warning', '-n',
        '-f', 'image2pipe', '-vcodec', 'mjpeg', '-framerate', str(fps), '-i', 'pipe:0',
        '-an', '-c:v', 'libx264', '-preset', 'ultrafast', '-crf', '20', '-threads', '2',
        '-vf', 'pad=ceil(iw/2)*2:ceil(ih/2)*2', '-pix_fmt', 'yuv420p',
        '-f', 'segment', '-segment_time', '60', '-reset_timestamps', '1',
        '-g', str(fps*2), str(root/'qt_%04d.mkv')], stdin=subprocess.PIPE,
        stdout=video_log, stderr=video_log)
    frames = (root/'qt_frames.jsonl').open('a', buffering=1)
    packets = PacketRecorder(root)
    count = 0
    state = dict(started_epoch_s=time.time(), qt_window_id=hex(window_id), video_fps=fps,
                 capture_size=[size.width(),size.height()], frame_count=0, video_error=None)

    def save_status():
        value = dict(state, updated_epoch_s=time.time(), streams=packets.status())
        tmp = root/'status.json.tmp'
        tmp.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
        tmp.replace(root/'status.json')

    def capture():
        nonlocal count
        try:
            at = time.time()
            shot = app.primaryScreen().grabWindow(window_id)
            if shot.isNull():
                raise RuntimeError('Qt window unavailable')
            image = QImage(size, QImage.Format_RGB32)
            image.fill(Qt.black)
            painter = QPainter(image)
            fitted = shot.scaled(size, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            painter.drawPixmap((size.width()-fitted.width())//2, (size.height()-fitted.height())//2, fitted)
            painter.end()
            buf = QBuffer()
            buf.open(QIODevice.WriteOnly)
            image.save(buf, 'JPEG', 90)
            if ffmpeg.poll() is not None:
                raise RuntimeError('Video encoder exited with code %s' % ffmpeg.returncode)
            ffmpeg.stdin.write(bytes(buf.data()))
            ffmpeg.stdin.flush()
            frames.write(json.dumps(dict(frame=count, captured_epoch_s=at, video_pts_s=count/fps))+'\n')
            count += 1
            state.update(frame_count=count, last_video_epoch_s=at, video_error=None)
        except Exception as exc:
            state['video_error'] = str(exc)
        save_status()

    timer = QTimer()
    timer.timeout.connect(capture)
    timer.start(1000//fps)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: app.quit())
    try:
        capture()
        app.exec_()
    finally:
        timer.stop()
        packets.close()
        ffmpeg.stdin.close()
        ffmpeg.wait(timeout=30)
        frames.close()
        video_log.close()
        state['stopped_epoch_s'] = time.time()
        save_status()


if __name__ == '__main__':
    main()
