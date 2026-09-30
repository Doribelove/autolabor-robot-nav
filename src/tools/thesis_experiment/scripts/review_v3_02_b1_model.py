#!/usr/bin/env python3
import importlib.util
from pathlib import Path

W = Path("/home/robot/robot_ws_base_rl")
SOURCE = W / "src/tools/thesis_experiment/scripts/review_v3_02_model.py"
REPAIR = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_b1.py"
spec = importlib.util.spec_from_file_location("v3_02_model_reviewer", SOURCE)
driver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(driver)
driver.V3_PATH = REPAIR
driver.main()
