"""Summarize recorded service results without replaying anything into ROS."""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path

import numpy as np

from .contracts import atomic_json


def report(source):
    counts, samples = Counter(), defaultdict(list)
    first = last = None
    backends = set()
    for line in Path(source).open():
        item = json.loads(line)
        camera = item.get("source", {}).get("camera", "unknown")
        counts["frames_" + camera] += 1
        stamp = item.get("source", {}).get("stamp_ns")
        if stamp:
            first = stamp if first is None else min(first, stamp)
            last = stamp if last is None else max(last, stamp)
        if item.get("error"):
            counts["rejected_frames"] += 1
            continue
        timing = item.get("timing", {})
        backends.add(timing.get("inference_backend", "no_inference"))
        counts["detections"] += len(item.get("detections", []))
        counts["ground_observations"] += len(item.get("observations", []))
        counts["rejected_geometry_or_pose"] += len(item.get("rejected", []))
        counts["unclassified"] += timing.get("unclassified", 0)
        for key, value in (("source_age_ms", item.get("source_age_ms")),
                           ("processing_ms", timing.get("processing_ms"))):
            if value is not None and np.isfinite(value):
                samples[camera + "/" + key].append(float(value))
    percentiles = {key: dict(zip(("p50", "p95", "p99"), np.percentile(values, [50, 95, 99]).tolist()))
                   for key, values in samples.items()}
    return dict(counts=dict(counts), percentiles_ms=percentiles,
                source_span_sec=0 if first is None else (last - first) / 1e9,
                inference_backends=sorted(backends), production_ready=False, motion_eligible=False,
                note="Fixture timings are not BPU benchmarks; geometry accuracy needs independent ground truth.")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--results", required=True)
    p.add_argument("--output", required=True)
    args = p.parse_args()
    if Path(args.output).exists():
        raise SystemExit("Choose a new report path")
    atomic_json(args.output, report(args.results))
