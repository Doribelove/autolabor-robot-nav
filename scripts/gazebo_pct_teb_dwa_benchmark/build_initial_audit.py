#!/usr/bin/env python3
"""Build the immutable P00 repository/environment audit without reading formal outcomes."""

import csv
import hashlib
import json
import os
import platform
import subprocess
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "artifacts" / "gazebo_pct_teb_dwa_benchmark"


def run(command, *, cwd=ROOT, env=None):
    completed = subprocess.run(
        command,
        cwd=str(cwd),
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    return {"command": command, "exit_code": completed.returncode, "output": completed.stdout}


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def isolated_ros(command):
    shell = (
        "source /opt/ros/noetic/setup.bash && "
        "source /home/robot/robot_ws_base_rl/devel/setup.bash && " + command
    )
    environment = {
        "HOME": os.environ.get("HOME", "/home/robot"),
        "PATH": "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
        "LANG": "C.UTF-8",
        "LC_ALL": "C.UTF-8",
    }
    return run(["bash", "--noprofile", "--norc", "-c", shell], env=environment)


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    status = run(["git", "status", "--short", "--branch"])
    submodules = run(["git", "submodule", "status", "--recursive"])
    status_path = OUTPUT / "initial_git_status.txt"
    submodule_path = OUTPUT / "initial_submodule_status.txt"
    status_path.write_text(status["output"], encoding="utf-8")
    submodule_path.write_text(submodules["output"], encoding="utf-8")

    probes = {
        "git_branch": run(["git", "branch", "--show-current"]),
        "git_head": run(["git", "rev-parse", "HEAD"]),
        "git_commit": run(["git", "show", "-s", "--format=%H%n%aI%n%s", "HEAD"]),
        "os_release": run(["bash", "--noprofile", "--norc", "-c", "cat /etc/os-release"]),
        "kernel": run(["uname", "-a"]),
        "cpu": run(["bash", "--noprofile", "--norc", "-c", "lscpu | sed -n '1,30p'"]),
        "memory": run(["free", "-h"]),
        "python": run(["python3", "--version"]),
        "ros_distribution": isolated_ros("rosversion -d"),
        "ros_version": isolated_ros("rosversion roslaunch"),
        "gazebo_version": isolated_ros("gazebo --version"),
        "catkin_version": isolated_ros("python3 -c 'import catkin_pkg; print(catkin_pkg.__version__)'"),
        "m2_package": isolated_ros("rospack find m2_gazebo"),
        "v5_package": isolated_ros("rospack find v5_e2e_scope_rl"),
        "teb_package": isolated_ros("rospack find teb_local_planner"),
        "dwa_package": isolated_ros("rospack find dwa_local_planner"),
        "gpu": run(["bash", "--noprofile", "--norc", "-c", "command -v nvidia-smi >/dev/null && nvidia-smi -L || true"]),
    }
    now = datetime.now(timezone.utc).isoformat()
    manifest = {
        "schema_version": "1.0",
        "captured_at_utc": now,
        "workspace": str(ROOT),
        "stable_workspace_modified": False,
        "branch": probes["git_branch"]["output"].strip(),
        "head": probes["git_head"]["output"].strip(),
        "git_status_snapshot": {
            "path": str(status_path.relative_to(ROOT)),
            "sha256": sha256(status_path),
            "line_count": len(status["output"].splitlines()),
        },
        "submodule_snapshot": {
            "path": str(submodule_path.relative_to(ROOT)),
            "sha256": sha256(submodule_path),
            "line_count": len(submodules["output"].splitlines()),
        },
        "host": {"hostname": platform.node(), "machine": platform.machine()},
        "probes": probes,
        "process_precheck": run(
            [
                "bash", "--noprofile", "--norc", "-c",
                "ps -eo pid,ppid,stat,cmd | grep -E '[r]oscore|[r]osmaster|[r]oslaunch|[g]zserver|[g]zclient|[m]ove_base|[t]rain_v[56]|[r]un_v6' || true",
            ]
        ),
    }
    (OUTPUT / "environment_manifest.yaml").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    critical = [
        "AGENTS.md",
        "CODEX_MASTER_PROMPT_PCT_VS_TRADITIONAL_PLANNERS.md",
        "CODEX_HANDOFF_PCT_TEB_DWA_GAZEBO_BENCHMARK.md",
        "docs/thesis_experiment/CURRENT_V6_PCT_SCOPE_RL_HANDOFF.md",
        "docs/thesis_experiment/CURRENT_V6_06_THESIS_TRANSFER_HANDOFF.md",
        "artifacts/v6/paper/v6_06_p1_r1_thesis_transfer_package/combined_claims_evidence_matrix.yaml",
        "src/application/v5_e2e_scope_rl/config/v6_00_pct_scope_rl_contract.yaml",
        "src/application/v5_e2e_scope_rl/config/v6_03_gazebo_contract.yaml",
        "src/application/v5_e2e_scope_rl/config/v6_03_r1_environment_repair.yaml",
        "src/application/v5_e2e_scope_rl/config/v6_03_r2_safety_alignment.yaml",
        "src/application/v5_e2e_scope_rl/config/v6_04_directional_shield_contract.yaml",
        "src/application/v5_e2e_scope_rl/scripts/run_v6_fresh_formal.py",
        "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/evaluation.py",
        "artifacts/v6/training/v6_01_r1_training_release.json",
        "artifacts/v6/training/v6_01_r1_validation/selection_report.json",
        "artifacts/v6/formal/v6_02_f1_fresh_formal/formal_assessment.json",
        "artifacts/v6/training/v6_01_r1/pct_seed9501/actor_step_0120000.pt",
        "artifacts/v6/training/v6_01_r1/pct_seed9502/actor_step_0080640.pt",
        "artifacts/v6/training/v6_01_r1/pct_seed9503/actor_step_0040704.pt",
        "artifacts/v6/training/v6_01_r1/pct_seed9504/actor_step_0080640.pt",
        "src/simulation/m2_gazebo/config/simulation_candidates.yaml",
    ]
    hash_path = OUTPUT / "key_sha256.csv"
    with hash_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["path", "exists", "size_bytes", "sha256"])
        writer.writeheader()
        for relative in critical:
            path = ROOT / relative
            writer.writerow(
                {
                    "path": relative,
                    "exists": path.is_file(),
                    "size_bytes": path.stat().st_size if path.is_file() else "",
                    "sha256": sha256(path) if path.is_file() else "",
                }
            )
    report = {
        "status": "p00_initial_audit_generated",
        "captured_at_utc": now,
        "git_status_exit_code": status["exit_code"],
        "submodule_status_exit_code": submodules["exit_code"],
        "critical_file_count": len(critical),
        "missing_critical_files": [relative for relative in critical if not (ROOT / relative).is_file()],
        "formal_seeds_accessed": False,
        "ros_or_gazebo_started": False,
        "stable_workspace_modified": False,
    }
    report_path = OUTPUT / "p00_initial_audit_report.json"
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
