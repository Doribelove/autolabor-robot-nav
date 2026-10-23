"""Qt viewer plus CPU BEV debug renderer. Neither participates in inference."""
from pathlib import Path
import queue
import threading
import time

import cv2
import numpy as np

from . import CAMERAS
from .contracts import atomic_json, read_json
from .geometry import Rectifier


class Renderer:
    def __init__(self, calibration, side=640, extent_m=8., *, offline_reference=False):
        self.calibration, self.side, self.extent = calibration, side, extent_m
        self.maps = {}
        self.rectifier = Rectifier(calibration, offline_reference=offline_reference)

    def ground_map(self, camera):
        if camera in self.maps:
            return self.maps[camera]
        c = self.calibration.ground_camera(camera)
        v, u = np.indices((self.side, self.side), dtype=np.float64)
        x = (self.side / 2 - v) * (2 * self.extent / self.side)
        y = (self.side / 2 - u) * (2 * self.extent / self.side)
        plane = np.asarray(self.calibration.data["ground_plane_base"])
        if abs(plane[2]) < .1:
            raise ValueError("Display ground plane is nearly vertical")
        z = -(plane[0] * x + plane[1] * y + plane[3]) / plane[2]
        xyz = np.stack((x, y, z, np.ones_like(x)), axis=0).reshape(4, -1)
        optical = np.linalg.inv(np.asarray(c["T_base_camera_optical"])) @ xyz
        pixels = np.asarray(c["K_rectified"]) @ optical[:3]
        depth = optical[2].reshape(x.shape)
        pixels[:2] /= np.where(abs(pixels[2:3]) > 1e-6, pixels[2:3], 1e-6)
        mx, my = [p.reshape(x.shape).astype(np.float32) for p in pixels[:2]]
        width, height = c["rectified_size"]
        mask = np.zeros((height, width), np.uint8)
        cv2.fillPoly(mask, [np.asarray(c["valid_ground_polygon_px"], np.int32)], 255)
        valid = cv2.remap(mask, mx, my, cv2.INTER_NEAREST) > 0
        valid &= (depth > .05) & (np.hypot(x, y) >= .3) & (np.hypot(x, y) <= 10)
        self.maps[camera] = mx, my, valid
        return mx, my, valid

    def render(self, snapshot, images, now_ns):
        total = np.zeros((self.side, self.side, 3), np.float32)
        count = np.zeros((self.side, self.side, 1), np.float32)
        panels = []
        for camera in CAMERAS:
            latest = snapshot.get("camera_results", {}).get(camera) or {}
            value = images.get(camera)
            valid = value is not None and 0 <= now_ns - value[0] <= 500_000_000
            panel = np.full((240, 320, 3), 28, np.uint8)
            if valid:
                stamp, image, rectified = value
                if not rectified and self.rectifier.offline_reference:
                    try:
                        image = self.rectifier.raw(camera, image)
                        rectified = True
                    except ValueError:
                        pass
                annotated = image.copy()
                if (rectified and latest.get("source", {}).get("camera") == camera
                        and latest["source"]["stamp_ns"] == stamp):
                    for obj in latest["detections"]:
                        x1, y1, x2, y2 = map(int, obj["bbox"])
                        cv2.rectangle(annotated, (x1, y1), (x2, y2), (40, 220, 255), 2)
                        label = "%s %.2f | %.2f" % (obj["material"], obj["detect_confidence"],
                                                    obj["classify_confidence"])
                        cv2.putText(annotated, label, (x1, max(18, y1 - 4)),
                                    cv2.FONT_HERSHEY_SIMPLEX, .5, (40, 220, 255), 1)
                panel = cv2.resize(annotated, (320, 240))
                if rectified:
                    try:
                        mx, my, mask = self.ground_map(camera)
                        projected = cv2.remap(image, mx, my, cv2.INTER_LINEAR)
                        total += projected * mask[:, :, None]
                        count += mask[:, :, None]
                    except ValueError:
                        pass
            label = camera + ("  %.0fms" % ((now_ns - value[0]) / 1e6) if valid else " : NO FRESH FRAME")
            cv2.rectangle(panel, (0, 0), (320, 27), (20, 20, 20), -1)
            cv2.putText(panel, label, (8, 19), cv2.FONT_HERSHEY_SIMPLEX, .5, (230, 230, 230), 1)
            panels.append(panel)
        bird = (total / np.maximum(count, 1)).astype(np.uint8)
        for m in range(-int(self.extent), int(self.extent) + 1, 2):
            p = int(self.side / 2 + m * self.side / (2 * self.extent))
            cv2.line(bird, (p, 0), (p, self.side - 1), (45, 45, 45), 1)
            cv2.line(bird, (0, p), (self.side - 1, p), (45, 45, 45), 1)
        pose = snapshot.get("display_pose")
        if pose and 0 <= now_ns - pose["stamp_ns"] <= 500_000_000:
            inverse = np.linalg.inv(np.asarray(pose["T_map_base"]))
            for target in snapshot["targets"]:
                p = inverse @ np.r_[target["map_xyz"], 1.]
                uv = (int(self.side / 2 - p[1] * self.side / (2 * self.extent)),
                      int(self.side / 2 - p[0] * self.side / (2 * self.extent)))
                color = (60, 220, 80) if target["status"] == "confirmed" else (80, 150, 240)
                cv2.circle(bird, uv, 6, color, 2)
        mid = self.side // 2
        cv2.rectangle(bird, (mid - 12, mid - 20), (mid + 12, mid + 20), (210, 210, 210), 2)
        cv2.arrowedLine(bird, (mid, mid), (mid, mid - 60), (200, 200, 200), 2)
        text = "SYNTHETIC / OFFLINE" if snapshot["synthetic"] else "GROUND-PLANE DEBUG"
        cv2.putText(bird, text, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 190, 255), 2)
        if not self.maps:
            cv2.putText(bird, "AWAITING CALIBRATION", (100, mid),
                        cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 190, 255), 2)
        mosaic = np.vstack((np.hstack(panels[:2]), np.hstack(panels[2:])))
        canvas = np.full((self.side, self.side + 640, 3), 22, np.uint8)
        canvas[:480, :640] = mosaic
        canvas[:, 640:] = bird
        cv2.putText(canvas, "Observation only | production_ready=false | motion_eligible=false",
                    (12, 520), cv2.FONT_HERSHEY_SIMPLEX, .48, (190, 190, 190), 1)
        cv2.putText(canvas, "X forward / Y left | detector input: individual perspective images",
                    (12, 550), cv2.FONT_HERSHEY_SIMPLEX, .45, (190, 190, 190), 1)
        return canvas


class PreviewWriter:
    """Bounded display slot on a separate CPU thread; never queues BPU work."""
    def __init__(self, directory, calibration):
        self.directory = Path(directory)
        self.renderer = Renderer(calibration)
        self.queue = queue.Queue(maxsize=1)
        self.stop_event = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def offer(self, snapshot, images, now_ns):
        try:
            self.queue.put_nowait((snapshot, dict(images), now_ns))
        except queue.Full:
            pass

    def _loop(self):
        while not self.stop_event.is_set():
            try:
                snapshot, images, now = self.queue.get(timeout=.2)
            except queue.Empty:
                continue
            try:
                image = self.renderer.render(snapshot, images, now)
                temporary = self.directory / "preview.tmp.jpg"
                if not cv2.imwrite(str(temporary), image):
                    raise RuntimeError("Could not encode preview")
                temporary.replace(self.directory / "preview.jpg")
                atomic_json(self.directory / "preview_meta.json",
                            dict(produced_ns=now, synthetic=snapshot["synthetic"],
                                 session=snapshot["session"]))
            except Exception as e:
                self.error = str(e)

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=5)


def show(directory=None, url=None):
    from PyQt5 import QtCore, QtGui, QtWidgets
    directory = Path(directory) if directory else None
    def fetch(name, limit):
        if directory is not None:
            with (directory / name).open("rb") as f:
                data = f.read(limit + 1)
        else:
            from urllib.request import urlopen
            with urlopen(url.rstrip("/") + "/" + name, timeout=.5) as response:
                data = response.read(limit + 1)
        if len(data) > limit:
            raise ValueError("Preview payload too large")
        return data
    app = QtWidgets.QApplication([])
    window = QtWidgets.QWidget()
    window.setWindowTitle("J6M 四路环视实验 — 观察模式")
    layout = QtWidgets.QVBoxLayout(window)
    status = QtWidgets.QLabel("等待环视数据")
    preview = QtWidgets.QLabel()
    preview.setMinimumSize(960, 480)
    preview.setAlignment(QtCore.Qt.AlignCenter)
    table = QtWidgets.QTableWidget(0, 5)
    table.setHorizontalHeaderLabels(["目标", "状态", "材质", "地图 X/Y (m)", "帧龄 (ms)"])
    layout.addWidget(status)
    layout.addWidget(preview, 1)
    layout.addWidget(table)
    def update():
        try:
            import json
            state = json.loads(fetch("state.json", 8 * 1024 * 1024))
            wall_age = max(0, time.time_ns() - state["produced_ns"]) / 1e6
            synthetic = state["synthetic"]
            offline = synthetic or state.get("offline", False)
            bev_only = state.get("service_kind") == "bev_only"
            stale = (wall_age > 1000 or state.get("service_stopped")
                     or (bev_only and not state.get("bev_valid"))) and not offline
            if not offline:
                meta = json.loads(fetch("preview_meta.json", 65536))
                stale = stale or meta["session"] != state["session"] or not (
                    0 <= time.time_ns() - meta["produced_ns"] <= 1_000_000_000)
            status.setText(("离线合成演示" if synthetic else "离线图像" if offline else "实时观察") +
                           (" · 数据已过期" if stale else "") +
                           (" · BEV 地面拼接" if bev_only else
                            " · 定位不可用" if not state.get("localization_valid") else "") +
                           " · 待标定项 %d · 禁止用于车辆控制" % len(state["calibration_issues"]))
            status.setStyleSheet("color: #d96b2b; font-weight: bold")
            if stale:
                preview.clear()
                preview.setText("数据已过期，等待新会话")
            else:
                pix = QtGui.QPixmap()
                pix.loadFromData(fetch("preview.jpg", 4 * 1024 * 1024), "JPEG")
                preview.setPixmap(pix.scaled(preview.size(), QtCore.Qt.KeepAspectRatio,
                                            QtCore.Qt.SmoothTransformation))
            targets = state["targets"]
            table.setVisible(not bev_only)
            table.setRowCount(len(targets))
            for row, t in enumerate(targets):
                age = t["age_ms"] if synthetic else max(0, time.time_ns() - t["last_observed_ns"]) / 1e6
                values = [t["id"].split("/")[-1], t["status"], t["material"],
                          "%.2f / %.2f" % tuple(t["map_xyz"][:2]), "%.0f" % age]
                for col, value in enumerate(values):
                    table.setItem(row, col, QtWidgets.QTableWidgetItem(value))
        except (OSError, ValueError, KeyError) as e:
            status.setText("等待有效数据：" + str(e))
            preview.clear()
    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(250)
    window.resize(1280, 900)
    window.show()
    return app.exec_()
