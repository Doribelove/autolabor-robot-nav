#!/usr/bin/env python3
import importlib.util
from pathlib import Path

W = Path("/home/robot/robot_ws_base_rl")
BASE = W / "src/tools/thesis_experiment/scripts/run_v3_01_r1_qualification.py"
MODULE = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_07_b1.py"
spec = importlib.util.spec_from_file_location("v3_07_b1_run", BASE)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
runner.M = MODULE
runner.main()
