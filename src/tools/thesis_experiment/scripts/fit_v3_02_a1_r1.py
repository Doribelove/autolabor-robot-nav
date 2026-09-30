#!/usr/bin/env python3
import importlib.util
from pathlib import Path

W = Path("/home/robot/robot_ws_base_rl")
SOURCE = W / "src/tools/thesis_experiment/scripts/fit_v3_02_predictor.py"
REPAIR = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_a1_r1.py"
spec = importlib.util.spec_from_file_location("v3_02_fitter", SOURCE)
driver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(driver)
driver.M = REPAIR
driver.main()
