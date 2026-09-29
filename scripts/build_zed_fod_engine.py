#!/usr/bin/env python3
"""Build the existing six-class model on this Jetson; never modify its .pt file.

Run with ~/python_env/fod_yolo/bin/python3. The engine is specific to the
installed CUDA/TensorRT and GPU. Rebuild after changing those or the model/ROI.
"""
import argparse
from datetime import datetime
from hashlib import sha256
import json
from pathlib import Path
import shutil
import tempfile


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", type=Path, default=root / "src/yolo/fod_yolo11n_img640_e300_orig/weights/best.pt")
    parser.add_argument("--output-dir", type=Path, default=Path.home() / ".cache/autolabor_fod")
    args = parser.parse_args()
    import tensorrt
    import torch
    import ultralytics
    from ultralytics import YOLO

    args.output_dir.mkdir(parents=True, exist_ok=True)
    source_hash = sha256(args.weights.read_bytes()).hexdigest()
    # 1280x720 frame, y=180..449 ground band, scaled to width 960 and padded to
    # a multiple of 32: exactly the existing PyTorch rectangular preprocessing.
    name = "fod_zed_roi_224x960_fp16"
    with tempfile.TemporaryDirectory(prefix="zed_engine_", dir=str(args.output_dir)) as work:
        checkpoint = Path(work) / (name + ".pt")
        shutil.copy2(args.weights, checkpoint)
        model = YOLO(str(checkpoint))
        class_names = model.names
        engine = Path(model.export(format="engine", imgsz=(224, 960), half=True,
                                  dynamic=False, batch=1, device=0, workspace=2,
                                  simplify=False, opset=12))
        engine_hash = sha256(engine.read_bytes()).hexdigest()
        output = args.output_dir / (name + ".engine")
        shutil.copy2(engine, output)
        Path(str(output) + ".sha256").write_text(engine_hash + "\n")
        metadata = {
            "built_at": datetime.now().astimezone().isoformat(),
            "source_weights": str(args.weights.resolve()), "source_sha256": source_hash,
            "engine_sha256": engine_hash, "shape": [1, 3, 224, 960],
            "source_frame": [1280, 720], "roi_rows": [180, 450],
            "class_names": class_names, "torch": torch.__version__,
            "ultralytics": ultralytics.__version__, "tensorrt": tensorrt.__version__,
            "gpu": torch.cuda.get_device_name(0),
        }
        Path(str(output) + ".json").write_text(json.dumps(metadata, indent=2) + "\n")
        print(output)


if __name__ == "__main__":
    main()
