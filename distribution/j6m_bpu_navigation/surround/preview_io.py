"""Bounded background export of diagnostic BEV images.

The producer owns immutable frame arrays. A slow disk replaces the pending
snapshot with the newest one, without queuing image history or delaying video.
"""
import hashlib
import json
import threading
import time

import cv2
import numpy as np


class SnapshotWriter:
    def __init__(self, output):
        self.output = output
        self.condition = threading.Condition()
        self.pending = None
        self.stopping = False
        self.status = dict(written=0, replaced=0, write_ms=None, error=None)
        self.thread = threading.Thread(target=self._run, name='bev-snapshot', daemon=True)
        self.thread.start()

    def submit(self, result):
        with self.condition:
            if self.stopping:
                return
            if self.pending is not None:
                self.status['replaced'] += 1
            self.pending = result
            self.condition.notify()

    def metrics(self):
        with self.condition:
            return dict(self.status)

    def close(self):
        with self.condition:
            self.stopping = True
            self.pending = None
            self.condition.notify()
        self.thread.join(timeout=5)

    def _run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.pending is not None or self.stopping)
                if self.stopping:
                    return
                result, self.pending = self.pending, None
            started = time.monotonic()
            try:
                self._write(result)
                with self.condition:
                    self.status.update(written=self.status['written']+1, error=None,
                                       write_ms=(time.monotonic()-started)*1000)
            except Exception as exc:
                with self.condition:
                    self.status['error'] = str(exc)

    def _write(self, result):
        images, image, counts, _, metadata = result
        # Never add export hashes to the live HTTP/GUI metadata from another thread.
        metadata = dict(metadata)
        frames = dict(bev=image, projection_support=np.minimum(
            counts.astype('uint16')*85, 255).astype('uint8'))
        frames.update({name+'_upright': frame for name, frame in images.items()})
        hashes = {}
        for name, frame in frames.items():
            path = self.output/(name+'.png')
            tmp = self.output/(name+'.tmp.png')
            if not cv2.imwrite(str(tmp), frame):
                raise RuntimeError('Could not write '+str(path))
            hashes[path.name] = hashlib.sha256(tmp.read_bytes()).hexdigest()
            tmp.replace(path)
        metadata.update(bev_png_sha256=hashes['bev.png'],
                        projection_support_png_sha256=hashes['projection_support.png'],
                        snapshot_files_sha256=hashes)
        # Written last: readers can detect an in-progress multi-file update using
        # these hashes. This preview snapshot is not a synchronized training sample.
        tmp = self.output/'bev.json.tmp'
        tmp.write_text(json.dumps(metadata, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
        tmp.replace(self.output/'bev.json')
