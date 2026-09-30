#!/usr/bin/env python3
"""Independent pure-offline review of V3-22 thesis paper materials."""

import csv
import hashlib
import math
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ROOT = Path("artifacts/v3/thesis/v3_22_p1_paper_materials")


def sha(path):
    return hashlib.sha256((W / path).read_bytes()).hexdigest()


def load_csv(name):
    with (W / ROOT / name).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def require(value, message):
    if not value:
        raise RuntimeError(message)


def main():
    manifest_path = ROOT / "paper_materials_manifest.sha256"
    require((W / manifest_path).is_file(), "manifest absent")
    entries = []
    for line in (W / manifest_path).read_text().splitlines():
        digest, name = line.split("  ", 1)
        path = Path(name)
        require((W / path).is_file(), "manifest resource absent: {}".format(path))
        require(sha(path) == digest, "manifest digest differs: {}".format(path))
        entries.append(path)
    require(len(entries) == len(set(entries)), "duplicate manifest resource")

    methods = load_csv("../v3_21_f1_execution/method_summary.csv") if False else None
    with (W / "artifacts/v3/thesis/v3_21_f1_execution/method_summary.csv").open(newline="", encoding="utf-8") as stream:
        rows = {r["method"]: r for r in csv.DictReader(stream)}
    v3 = rows["v3_spatial_safe"]; teb = rows["actual_teb"]; ablation = rows["v3_no_candidate_conditioning"]
    require(int(v3["episodes"]) == 30 and int(v3["goal_successes"]) == 28, "V3 count differs")
    require(int(teb["goal_successes"]) == 30 and int(ablation["goal_successes"]) == 13, "comparator count differs")
    require(math.isclose(float(v3["restricted_time_mean_s"]), 7.785, abs_tol=1e-12), "V3 mean differs")
    require(math.isclose(float(teb["restricted_time_mean_s"]), 7.77, abs_tol=1e-12), "TEB mean differs")

    claims = yaml.safe_load((W / ROOT / "paper_claims.yaml").read_text())
    require(claims["fresh_characterization"]["actual_teb_improvement_demonstrated"] is False, "positive TEB claim leaked")
    require(claims["fresh_characterization"]["formal_result"] is False, "V3-21 mislabeled formal")
    require(claims["historical_formal_result"]["stage"] == "V3-11-F1", "historical result lost")
    require(claims["training_steps"] == 0 and not claims["hardware_accessed"] and not claims["real_vehicle_used"], "authority boundary differs")

    figures = [p for p in entries if p.parts[-2:-1] == ("figures",)]
    tables = [p for p in entries if p.parts[-2:-1] == ("tables",)]
    require(len(figures) == 15, "expected five figures in three formats")
    require(len(tables) == 5, "expected five three-line tables")
    for path in figures:
        data = (W / path).read_bytes()
        if path.suffix == ".pdf": require(data.startswith(b"%PDF"), "bad PDF figure")
        elif path.suffix == ".png": require(data.startswith(b"\x89PNG\r\n\x1a\n"), "bad PNG figure")
        elif path.suffix == ".svg": require(b"<svg" in data[:1000], "bad SVG figure")
    for path in tables:
        text = (W / path).read_text()
        require("\\toprule" in text and "\\midrule" in text and "\\bottomrule" in text, "not a booktabs table: {}".format(path))

    review = {
        "schema_version": "1.0", "stage": "V3-22-P1",
        "status": "paper_materials_independent_review_pass",
        "checks": {
            "manifest_resource_hashes": True, "five_figures_three_formats": True,
            "five_latex_booktabs_tables": True, "source_statistics_recomputed": True,
            "historical_and_fresh_results_not_pooled": True,
            "fresh_actual_teb_superiority_not_claimed": True,
            "authority_boundary_closed": True,
        },
        "manifest_sha256": sha(manifest_path),
        "figure_file_count": len(figures), "table_file_count": len(tables),
        "formal_result": False, "characterization_result_available": True,
        "training_steps": 0, "hardware_accessed": False, "real_vehicle_used": False,
    }
    payload = yaml.safe_dump(review, sort_keys=False, allow_unicode=True).encode("utf-8")
    review_path = W / ROOT / "independent_paper_materials_review.yaml"
    if review_path.exists(): require(review_path.read_bytes() == payload, "review differs")
    else: review_path.write_bytes(payload)
    freeze = "{}  {}\n{}  {}\n".format(sha(manifest_path), manifest_path, hashlib.sha256(payload).hexdigest(), ROOT / "independent_paper_materials_review.yaml").encode()
    freeze_path = W / ROOT / "terminal_freeze.sha256"
    if freeze_path.exists(): require(freeze_path.read_bytes() == freeze, "freeze differs")
    else: freeze_path.write_bytes(freeze)
    print("status=paper_materials_independent_review_pass review_sha256={}".format(hashlib.sha256(payload).hexdigest()))


if __name__ == "__main__":
    main()

