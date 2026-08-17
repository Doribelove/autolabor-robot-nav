"""ROS camera-calibration YAML loading without a camera vendor dependency."""

from dataclasses import dataclass
from pathlib import Path
from typing import List

import yaml


@dataclass(frozen=True)
class CameraCalibration:
    width: int
    height: int
    camera_name: str
    distortion_model: str
    d: List[float]
    k: List[float]
    r: List[float]
    p: List[float]

    @property
    def calibrated(self) -> bool:
        return (
            self.width > 0
            and self.height > 0
            and len(self.k) == 9
            and self.k[0] > 0.0
            and self.k[4] > 0.0
        )

    def scaled(self, width: int, height: int) -> "CameraCalibration":
        """Scale pinhole intrinsics when a source is resized."""
        if width <= 0 or height <= 0:
            raise ValueError("camera image dimensions must be positive")
        if not self.calibrated or (width == self.width and height == self.height):
            return CameraCalibration(
                width=width,
                height=height,
                camera_name=self.camera_name,
                distortion_model=self.distortion_model,
                d=list(self.d),
                k=list(self.k),
                r=list(self.r),
                p=list(self.p),
            )

        sx = float(width) / float(self.width)
        sy = float(height) / float(self.height)
        k = list(self.k)
        p = list(self.p)
        k[0] *= sx
        k[2] *= sx
        k[4] *= sy
        k[5] *= sy
        p[0] *= sx
        p[2] *= sx
        p[3] *= sx
        p[5] *= sy
        p[6] *= sy
        p[7] *= sy
        return CameraCalibration(
            width=width,
            height=height,
            camera_name=self.camera_name,
            distortion_model=self.distortion_model,
            d=list(self.d),
            k=k,
            r=list(self.r),
            p=p,
        )


def uncalibrated(width: int, height: int, camera_name: str) -> CameraCalibration:
    return CameraCalibration(
        width=width,
        height=height,
        camera_name=camera_name,
        distortion_model="plumb_bob",
        d=[],
        k=[0.0] * 9,
        r=[0.0] * 9,
        p=[0.0] * 12,
    )


def _matrix_data(document, key: str, expected_size: int) -> List[float]:
    value = document.get(key, {})
    data = value.get("data", []) if isinstance(value, dict) else []
    if len(data) != expected_size:
        raise ValueError("{} must contain {} values".format(key, expected_size))
    return [float(item) for item in data]


def load_camera_calibration(url: str) -> CameraCalibration:
    """Load the conventional camera_calibration/camera_info_manager YAML format."""
    if not url:
        raise ValueError("camera calibration URL is empty")
    path_text = url[7:] if url.startswith("file://") else url
    path = Path(path_text).expanduser()
    if not path.is_file():
        raise FileNotFoundError("camera calibration does not exist: {}".format(path))

    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError("camera calibration YAML must contain a mapping")

    width = int(document.get("image_width", 0))
    height = int(document.get("image_height", 0))
    if width <= 0 or height <= 0:
        raise ValueError("camera calibration image_width/image_height must be positive")

    distortion = document.get("distortion_coefficients", {})
    d = distortion.get("data", []) if isinstance(distortion, dict) else []
    return CameraCalibration(
        width=width,
        height=height,
        camera_name=str(document.get("camera_name", path.stem)),
        distortion_model=str(document.get("distortion_model", "plumb_bob")),
        d=[float(item) for item in d],
        k=_matrix_data(document, "camera_matrix", 9),
        r=_matrix_data(document, "rectification_matrix", 9),
        p=_matrix_data(document, "projection_matrix", 12),
    )
