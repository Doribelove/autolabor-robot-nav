#!/usr/bin/env python3
import importlib.util
from pathlib import Path
W = Path("/home/robot/robot_ws_base_rl")
SOURCE = W / "src/tools/thesis_experiment/scripts/run_v3_03_c1_closed_loop.py"
V3 = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_06_c3.py"
vs = importlib.util.spec_from_file_location("v3_06_c3_selector", V3)
v = importlib.util.module_from_spec(vs); vs.loader.exec_module(v)
spec = importlib.util.spec_from_file_location("v3_06_c3_runner", SOURCE)
driver = importlib.util.module_from_spec(spec); spec.loader.exec_module(driver)
driver.M = V3; driver.apply_selection_method = v.apply_selection_method; driver.main()
