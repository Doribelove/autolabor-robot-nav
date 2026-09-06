#!/usr/bin/env python3
"""Compare CPU FP32 ONNX and CPU FP32 original checkpoints on fixed inputs.

Not an INT8 accuracy test, not a full validation-set mAP/classification report,
and not equivalence to the production CUDA FP16 pipeline or tracker.
"""
import os
os.environ["CUDA_VISIBLE_DEVICES"] = ""
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "2"
os.environ["MKL_NUM_THREADS"] = "2"
os.environ["YOLO_AUTOINSTALL"] = "false"
os.environ["NO_ALBUMENTATIONS_UPDATE"] = "1"
import hashlib
import json
from pathlib import Path
import sys
import time

LAB = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(LAB / "vendor/python"))
sys.path.insert(0, "/home/slam/yolo11/yolo11_GAM")
import cv2
import numpy as np
import onnxruntime as ort
from PIL import Image
import torch
from ultralytics import YOLO
from ultralytics.data.augment import LetterBox, classify_transforms
from ultralytics.utils import torch_utils


def images(directory, count):
    files = sorted(path for path in directory.iterdir()
                   if path.suffix.lower() in {".jpg", ".jpeg", ".png"})
    if len(files) < count:
        raise ValueError("Insufficient test images: " + str(directory))
    return [files[index] for index in np.linspace(0, len(files) - 1, count, dtype=int)]


def main():
    if os.environ.get("ROBOT_OPTIMIZED_SANDBOX") != "1":
        raise RuntimeError("Use scripts/optimized.sh run")
    cv2.setNumThreads(1)
    torch_utils.NUM_THREADS = 2
    torch.set_num_threads(2)
    torch.set_num_interop_threads(1)
    options = ort.SessionOptions()
    options.intra_op_num_threads = 2
    options.inter_op_num_threads = 1
    options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    options.log_severity_level = 3
    root = Path("/home/slam/yolo11/detect_classify")
    cases = []
    started = time.monotonic()
    for kind in ("detector", "detector_zed", "classifier"):
        directory = LAB / "exports" / kind
        manifest = json.loads((directory / "export_manifest.json").read_text())
        checkpoint = YOLO(str(directory / "best.pt"))
        model = checkpoint.model.float().cpu().eval().fuse(verbose=False)
        session = ort.InferenceSession(str(directory / "best.onnx"), sess_options=options,
                                       providers=["CPUExecutionProvider"])
        if session.get_providers() != ["CPUExecutionProvider"]:
            raise RuntimeError("Unexpected ONNX execution provider")
        shape = session.get_inputs()[0].shape
        if kind == "classifier":
            selected = [path for name in manifest["classes"]
                        for path in images(root / "material_classification_dataset/val" / name, 4)]
            transform = classify_transforms(size=224)
        else:
            selected = images(root / "trash_detection_dataset/images/val", 5)
        for path in selected:
            bgr = cv2.imread(str(path))
            if kind == "classifier":
                tensor = transform(Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))).unsqueeze(0)
            else:
                # Both engines get the exact same fixed-size RGB tensor.
                prepared = LetterBox(new_shape=(shape[2], shape[3]), auto=False, stride=32)(image=bgr)
                tensor = torch.from_numpy(np.ascontiguousarray(prepared[:, :, ::-1].transpose(2, 0, 1))).float()[None] / 255.0
            with torch.inference_mode():
                reference = model(tensor)
                if isinstance(reference, (tuple, list)):
                    reference = reference[0]
                reference = reference.detach().numpy()
            actual = session.run(None, {session.get_inputs()[0].name: tensor.numpy()})[0]
            if reference.shape != actual.shape or not np.isfinite(actual).all():
                raise RuntimeError("Shape/nonfinite output mismatch")
            difference = np.abs(reference - actual)
            case = {"kind": kind, "source": str(path),
                    "source_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                    "max_abs_error": float(difference.max()), "mean_abs_error": float(difference.mean())}
            if kind == "classifier":
                case["torch_top1"] = int(reference.argmax())
                case["onnx_top1"] = int(actual.argmax())
                case["passed"] = bool(case["torch_top1"] == case["onnx_top1"] and difference.max() <= 1e-4)
            else:
                case["max_box_delta_px"] = float(difference[:, :4].max())
                case["max_score_delta"] = float(difference[:, 4:].max())
                case["passed"] = case["max_box_delta_px"] <= 0.1 and case["max_score_delta"] <= 1e-4
            cases.append(case)
            print(json.dumps(case), flush=True)
        del session, checkpoint, model
    report = {"scope": "fixed-input CPU FP32 Torch vs CPU FP32 ONNX only", "cases": cases,
              "passed": all(case["passed"] for case in cases), "count": len(cases),
              "seconds": time.monotonic() - started, "cuda_initialized": torch.cuda.is_initialized(),
              "torch_threads": torch.get_num_threads(), "onnxruntime_version": ort.__version__,
              "bpu_quantization_accuracy_tested": False, "production_fp16_equivalence_tested": False}
    with (LAB / "results/onnx_equivalence.json").open("x") as stream:
        json.dump(report, stream, indent=2)
        stream.write("\n")
    print(json.dumps({key: value for key, value in report.items() if key != "cases"}, indent=2))
    return int(not report["passed"])


if __name__ == "__main__":
    raise SystemExit(main())
