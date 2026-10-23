"""Explicitly started native capture. Merely importing this module opens no device."""
import ctypes as C
import fcntl
import hashlib
from pathlib import Path
import threading
import time
import uuid

import cv2
import numpy as np

from . import CAMERAS
from .contracts import ClockMapping, Frame, read_json


class Meta(C.Structure):
    _fields_ = [("raw_stamp", C.c_uint64), ("hal_received_ns", C.c_uint64),
                ("sequence", C.c_uint32), ("width", C.c_uint32), ("height", C.c_uint32),
                ("bytes", C.c_uint32), ("vendor_format", C.c_int32), ("gdc_applied", C.c_uint32)]


def decode_yuv(payload, width, height, layout):
    array = np.asarray(payload, dtype=np.uint8)
    if layout == "NV12":
        expected = width * height * 3 // 2
        shape, code = (height * 3 // 2, width), cv2.COLOR_YUV2BGR_NV12
    elif layout in ("YUYV", "UYVY"):
        expected = width * height * 2
        shape = (height, width, 2)
        code = cv2.COLOR_YUV2BGR_YUY2 if layout == "YUYV" else cv2.COLOR_YUV2BGR_UYVY
    elif layout == "NV16":
        if array.size != width * height * 2 or width % 2 or height % 2:
            raise ValueError("NV16 shape/byte count mismatch")
        y = array[:width * height].reshape(height, width)
        uv = array[width * height:].reshape(height, width)
        return cv2.cvtColor(np.stack((y, uv), axis=2), cv2.COLOR_YUV2BGR_YUY2)
    else:
        raise ValueError("Pixel layout must be measured, not inferred from file extension")
    if width % 2 or height % 2 or array.size != expected:
        raise ValueError("YUV shape/byte count mismatch")
    return cv2.cvtColor(array.reshape(shape), code)


def _gdc_bin(name, camera, calibration, vpm):
    """Validate the full-frame PYM -> calibrated perspective GDC path before SDK init."""
    if camera.get("kind") != "pym" or camera.get("layout") != "NV12":
        raise ValueError("GDC requires PYM NV12 input; no live CPU rectification fallback")
    if camera.get("pym_layer") != 0:
        raise ValueError("GDC requires the full calibrated PYM layer0")
    if calibration is None:
        raise ValueError("Calibration required to verify GDC perspective output")
    calibration.intrinsics(name)
    cal = calibration.cameras[name]
    pipeline = vpm.get("pipeline%s" % camera.get("pipeline"), {})
    gdc = pipeline.get("gdc_node0", {})
    raw, rect = cal["raw_size"], cal["rectified_size"]
    if ([gdc.get("input_width"), gdc.get("input_height")] != raw
            or [gdc.get("output_width"), gdc.get("output_height")] != rect):
        raise ValueError("VPM GDC input/output sizes differ from calibrated perspective")
    pym = pipeline.get("pym_node0", {}).get("chn", {})
    rois = pym.get("ds_roi", [])
    roi = rois[0] if rois else {}
    if (pym.get("ds_roi_en") != 1 or pym.get("ds_roi_uv_bypass") != 0
            or [pym.get("src_in_width"), pym.get("src_in_height")] != raw
            or roi.get("ds_roi_sel") != 0 or roi.get("ds_roi_layer") != 0
            or roi.get("ds_roi_start_left") != 0 or roi.get("ds_roi_start_top") != 0
            or [roi.get("ds_roi_region_width"), roi.get("ds_roi_region_height")] != raw
            or [roi.get("ds_roi_out_width"), roi.get("ds_roi_out_height")] != raw):
        raise ValueError("PYM layer0 must preserve the full calibrated image and NV12 chroma")
    path = camera.get("gdc_bin")
    if not path or not Path(path).is_file():
        raise ValueError("Calibrated perspective GDC bin missing")
    data = Path(path).read_bytes()
    if not data or len(data) % 4:
        raise ValueError("GDC bin must be nonempty and 4-byte aligned")
    if hashlib.sha256(data).hexdigest() != cal.get("gdc_sha256"):
        raise ValueError("GDC bin SHA256 differs from calibrated perspective transform")
    return data


def capture_issues(config, require_clock=True, *, calibration=None, require_gdc=True):
    issues = []
    if config.get("board_verified") is not True:
        issues.append("Board/image/SDK compatibility not verified")
    if config.get("port_mapping_verified") is not True:
        issues.append("Physical front/right/rear/left cable mapping not verified")
    ports = [config.get("cameras", {}).get(k, {}).get("pipeline") for k in CAMERAS]
    if set(config.get("cameras", {})) != set(CAMERAS):
        issues.append("Exactly front/right/rear/left capture entries required")
    if any(type(p) is not int or not 0 <= p < 24 for p in ports) or len(set(ports)) != 4:
        issues.append("Four distinct pipeline numbers required")
    vpm = {}
    if require_gdc:
        try:
            vpm = read_json(config["vpm_config"])
        except (OSError, ValueError, TypeError, KeyError) as e:
            issues.append("GDC VPM configuration unavailable: " + str(e))
    for name in CAMERAS:
        camera = config.get("cameras", {}).get(name, {})
        if camera.get("layout") not in ("NV12", "NV16", "YUYV", "UYVY"):
            issues.append(name + ": pixel layout missing")
        if camera.get("kind") not in ("cim", "pym"):
            issues.append(name + ": capture kind missing")
        if camera.get("kind") == "pym" and camera.get("layout") != "NV12":
            issues.append(name + ": PYM path requires NV12")
        if camera.get("kind") == "cim" and camera.get("layout") not in ("YUYV", "UYVY", "NV16"):
            issues.append(name + ": CIM path requires verified YUV422 layout")
        if require_clock and not camera.get("clock_mapping_file"):
            issues.append(name + ": measured exposure clock mapping missing")
        if not camera.get("format_verified") or type(camera.get("vendor_format")) is not int:
            issues.append(name + ": vendor output format not verified")
        if require_gdc:
            try:
                _gdc_bin(name, camera, calibration, vpm)
            except (OSError, ValueError, TypeError, KeyError) as e:
                issues.append(name + ": " + str(e))
        elif camera.get("gdc_bin"):
            issues.append(name + ": intrinsic calibration acquisition requires raw images; remove GDC bin")
    for key in ("library", "vio_library", "hbmem_library", "vpm_config", "camera_config"):
        value = config.get(key)
        if not value or not Path(value).is_file():
            issues.append(key + ": file missing")
    return issues


class NativeCapture:
    def __init__(self, config, calibration, on_frame, calibration_only=False):
        self.calibration_only = calibration_only
        problems = capture_issues(config, require_clock=not calibration_only,
                                  calibration=calibration, require_gdc=not calibration_only)
        if problems or calibration.synthetic:
            raise ValueError("; ".join(problems or ["Synthetic calibration cannot open cameras"]))
        self.config, self.calibration, self.on_frame = config, calibration, on_frame
        self.context = None
        self.threads = []
        self.stop_event = threading.Event()
        self.errors = {}
        self.session = uuid.uuid4().hex
        # Pin the exact verified bytes before opening any camera or SDK. No LUT
        # file reads/hashes occur per frame; four cameras reuse their loaded LUTs.
        self.gdc_bins = {}
        if not calibration_only:
            vpm = read_json(config["vpm_config"])
            self.gdc_bins = {name: _gdc_bin(name, c, calibration, vpm)
                             for name, c in config["cameras"].items()}
            for c in config["cameras"].values():
                ClockMapping(read_json(c["clock_mapping_file"]))
        self.lock = Path(config["exclusive_lock"]).open("a")
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.lib = C.CDLL(str(Path(config["library"]).resolve()))
            self.lib.sc_error.restype = C.c_char_p
            self.lib.sc_abi_version.restype = C.c_uint
            if self.lib.sc_abi_version() != 1:
                raise ValueError("Capture ABI mismatch")
            self.lib.sc_open.argtypes = [C.c_char_p] * 4
            self.lib.sc_open.restype = C.c_void_p
            self.lib.sc_start.argtypes = [C.c_void_p, C.c_uint]
            self.lib.sc_set_gdc.argtypes = [C.c_void_p, C.c_uint, C.c_void_p, C.c_size_t]
            self.lib.sc_read.argtypes = [C.c_void_p, C.c_uint, C.c_int, C.c_uint,
                                        C.c_void_p, C.c_size_t, C.POINTER(Meta)]
            self.lib.sc_close.argtypes = [C.c_void_p]
            self.context = self.lib.sc_open(*[
                str(Path(config[k]).resolve()).encode() for k in
                ("vpm_config", "camera_config", "vio_library", "hbmem_library")])
            if not self.context:
                raise RuntimeError(self.lib.sc_error().decode())
            for camera, c in config["cameras"].items():
                if not calibration_only:
                    data = self.gdc_bins[camera]
                    self._check(self.lib.sc_set_gdc(self.context, c["pipeline"], data, len(data)))
            for c in config["cameras"].values():
                self._check(self.lib.sc_start(self.context, c["pipeline"]))
        except Exception:
            if self.context:
                self.lib.sc_close(self.context)
                self.context = None
            self.lock.close()
            raise

    def _check(self, ret):
        if ret != 0:
            raise RuntimeError(self.lib.sc_error().decode(errors="replace"))

    def start(self):
        for camera in CAMERAS:
            thread = threading.Thread(target=self._reader, args=(camera,), daemon=True)
            self.threads.append(thread)
            thread.start()

    def _reader(self, camera):
        c = self.config["cameras"][camera]
        buffer = np.empty(4096 * 4096 * 2, np.uint8)
        failures = 0
        while not self.stop_event.is_set():
            try:
                meta = Meta()
                kind = 1 if c["kind"] == "pym" else 2 if c["layout"] == "NV16" else 0
                self._check(self.lib.sc_read(self.context, c["pipeline"],
                    kind, c.get("pym_layer", 0),
                    buffer.ctypes.data, buffer.size, C.byref(meta)))
                now = time.time_ns()
                if meta.vendor_format != c["vendor_format"]:
                    raise ValueError("Vendor format changed; reject frame")
                expected_gdc = 0 if self.calibration_only else 1
                if meta.gdc_applied != expected_gdc:
                    raise ValueError("GDC completion flag mismatch; reject frame without CPU fallback")
                size_key = "raw_size" if self.calibration_only else "rectified_size"
                if [meta.width, meta.height] != self.calibration.cameras[camera][size_key]:
                    raise ValueError("Capture output size differs from calibrated " + size_key)
                if self.calibration_only:
                    stamp, clock_id, mapping_id = 0, "UNVERIFIED", "UNVERIFIED"
                else:
                    mapping = ClockMapping(read_json(c["clock_mapping_file"]))
                    stamp = mapping.convert(int(meta.raw_stamp), now)
                    clock_id, mapping_id = mapping.data["clock_id"], mapping.data["id"]
                image = decode_yuv(buffer[:meta.bytes], meta.width, meta.height, c["layout"])
                frame = Frame(camera, self.session, int(meta.sequence), int(meta.raw_stamp),
                              stamp, now, clock_id, mapping_id,
                              self.calibration.id, image, bool(meta.gdc_applied),
                              self.calibration.cameras[camera].get("rectification_id", "")
                              if meta.gdc_applied else "", False)
                self.on_frame(frame, time.time_ns())
                failures = 0
                self.errors.pop(camera, None)
            except Exception as e:
                failures += 1
                self.errors[camera] = str(e)
                if failures >= 10:
                    self.errors[camera] += " (reader stopped after 10 consecutive failures)"
                    return
                self.stop_event.wait(.1)

    def close(self):
        self.stop_event.set()
        for thread in self.threads:
            thread.join(timeout=3)
        if any(thread.is_alive() for thread in self.threads):
            raise RuntimeError("Vendor read did not return; terminate this service, do not start another owner")
        if self.context:
            self.lib.sc_close(self.context)
            self.context = None
        self.lock.close()
