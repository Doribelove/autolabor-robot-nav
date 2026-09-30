#!/usr/bin/env python3
import importlib.util
from pathlib import Path
W = Path("/home/robot/robot_ws_base_rl")
import sys
PACKAGE = W / "src/application/v3_scope_planner/src"
if str(PACKAGE) not in sys.path:
    sys.path.insert(0, str(PACKAGE))
from v3_scope_planner import spatial_core_r1
BASE = W / "src/tools/thesis_experiment/scripts/run_v3_08_a1_integration.py"
MODULE = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_08_a1_r1.py"
spec = importlib.util.spec_from_file_location("v3_08_a1_r1_run", BASE)
driver = importlib.util.module_from_spec(spec); spec.loader.exec_module(driver)
driver.MODULE = MODULE
driver.spatial_core = spatial_core_r1
driver.main()
