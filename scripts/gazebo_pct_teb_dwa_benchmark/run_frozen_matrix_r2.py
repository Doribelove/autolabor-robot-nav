#!/usr/bin/env python3
"""R2 entry point for the hash-bound pilot and formal-round-2 runner."""

from __future__ import annotations

import run_frozen_matrix as frozen


frozen.LAUNCH = "m2_pct_teb_dwa_benchmark_r2_episode.launch"
frozen.STAGES["pilot"] = {
    "split": "pilot",
    "report_stage": "PCT-BENCH-PILOT-R2",
    "seed_min": 196601,
    "seed_max": 196620,
}


if __name__ == "__main__":
    frozen.main()
