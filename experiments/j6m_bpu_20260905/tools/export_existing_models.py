#!/usr/bin/env python3
"""CPU-only, fixed-shape exports of unchanged, hash-pinned production weights.

No installations, ROS, camera, BPU, or CUDA inference. Exportability to ONNX
does not establish BPU operator support or quantized-model accuracy.
"""
import os

os.environ["CUDA_VISIBLE_DEVICES"] = ""
os.environ["OMP_NUM_THREADS"] = "2"
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "2"
os.environ["NUMEXPR_NUM_THREADS"] = "1"
os.environ["YOLO_AUTOINSTALL"] = "false"
os.environ["NO_ALBUMENTATIONS_UPDATE"] = "1"

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import shutil
import sys
import time

LAB = Path(__file__).resolve().parents[1]
MODELS = {
    "detector": {
        "source": "/home/slam/yolo11/detect_classify/detect/trash_yolo11s_gam/best.pt",
        "sha256": "711b6bb4b4debebcf993f033f23e7e641a02dd279254779f8dafed11b6a79233",
        "task": "detect", "classes": ["trash"], "imgsz": 1024,
    },
    "classifier": {
        "source": "/home/slam/yolo11/detect_classify/classify/material_yolo11s_cls/best.pt",
        "sha256": "d0cce9310e184e8acd7a6142face16d39aadc9a6e5405b18694346f2315899e9",
        "task": "classify", "classes": ["metal", "plastic", "paper", "glass", "kitchen_waste"],
        "imgsz": 224,
    },
}
# The current single 640x360 ZED frame uses Ultralytics rectangular letterbox:
# imgsz=1024 -> tensor height 576, width 1024, rather than a square tensor.
MODELS["detector_zed"] = dict(MODELS["detector"], imgsz=[576, 1024])


def digest(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("kind", choices=MODELS)
    args = parser.parse_args()
    if os.environ.get("ROBOT_OPTIMIZED_SANDBOX") != "1":
        raise RuntimeError("Use scripts/optimized.sh run for read-only source isolation")
    if LAB != Path("/home/slam/robot_j6m_ws_optimized_20260905/experiments/j6m_bpu_20260905"):
        raise RuntimeError("Unexpected laboratory location")
    spec = MODELS[args.kind]
    if digest(spec["source"]) != spec["sha256"]:
        raise RuntimeError("Production checkpoint hash mismatch")
    output = LAB / "exports" / args.kind
    if output.exists():
        raise FileExistsError("Refusing to overwrite an earlier export: " + str(output))
    output.mkdir()
    checkpoint = output / "best.pt"
    shutil.copyfile(spec["source"], checkpoint)
    if digest(checkpoint) != spec["sha256"]:
        raise RuntimeError("Private checkpoint copy changed")

    sys.path.insert(0, "/home/slam/yolo11/yolo11_GAM")
    import torch
    import onnx
    import cv2
    from ultralytics import YOLO
    from ultralytics.utils import torch_utils
    # select_device('cpu') resets torch's thread count from this module-level
    # constant. Override it only in this offline export process, not on disk.
    torch_utils.NUM_THREADS = 2
    torch.set_num_threads(2)
    torch.set_num_interop_threads(1)
    cv2.setNumThreads(1)
    model = YOLO(str(checkpoint))
    classes = [str(model.names[i]) for i in range(len(model.names))]
    if classes != spec["classes"] or model.task != spec["task"]:
        raise RuntimeError("Task/class contract changed")
    gam_count = sum("GAM" in type(module).__name__.upper() for module in model.model.modules())
    if spec["task"] == "detect" and not gam_count:
        raise RuntimeError("Expected GAM layers are missing")
    started = time.monotonic()
    exported = Path(model.export(format="onnx", device="cpu", imgsz=spec["imgsz"],
                                 batch=1, half=False, dynamic=False, simplify=False,
                                 nms=False, opset=13, verbose=False))
    if exported.resolve().parent != output.resolve():
        raise RuntimeError("Export escaped the laboratory")
    graph = onnx.load(str(exported))
    onnx.checker.check_model(graph, full_check=True)

    def tensor_info(tensor):
        tensor_type = tensor.type.tensor_type
        return {"name": tensor.name, "dtype": tensor_type.elem_type,
                "shape": [dim.dim_value if dim.HasField("dim_value") else dim.dim_param
                          for dim in tensor_type.shape.dim]}

    report = dict(spec, source_sha256_after=digest(spec["source"]),
                  onnx_path=str(exported), onnx_sha256=digest(exported),
                  export_seconds=time.monotonic() - started, gam_layers=gam_count,
                  inputs=[tensor_info(tensor) for tensor in graph.graph.input],
                  outputs=[tensor_info(tensor) for tensor in graph.graph.output],
                  operators=dict(Counter(node.op_type for node in graph.graph.node)),
                  custom_domains=sorted({node.domain for node in graph.graph.node if node.domain}),
                  opset=13, batch=1, onnx_checker_passed=True,
                  bpu_compatible=None, numerical_equivalence_tested=False,
                  cuda_initialized=torch.cuda.is_initialized(),
                  cpu_threads=torch.get_num_threads())
    with (output / "export_manifest.json").open("x") as stream:
        json.dump(report, stream, indent=2)
        stream.write("\n")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
