#!/usr/bin/env python3
import importlib.util
from pathlib import Path

W = Path("/home/robot/robot_ws_base_rl")
SOURCE = W / "src/tools/thesis_experiment/scripts/review_v3_03_c1_result.py"
V3 = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_03_c2.py"
ASSESSOR = W / "src/tools/thesis_experiment/scripts/assess_v3_03_c1_closed_loop.py"
RUNNER = W / "src/tools/thesis_experiment/scripts/run_v3_03_c1_closed_loop.py"
spec = importlib.util.spec_from_file_location("v3_c2_result_reviewer", SOURCE)
driver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(driver)
driver.V3_PATH = V3
driver.ASSESSOR_PATH = ASSESSOR
driver.RUNNER_PATH = RUNNER
driver.main()
