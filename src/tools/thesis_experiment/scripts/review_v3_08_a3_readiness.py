#!/usr/bin/env python3
import importlib.util
from pathlib import Path
W=Path("/home/robot/robot_ws_base_rl"); BASE=W/"src/tools/thesis_experiment/scripts/review_v3_02_t1_readiness.py"; MODULE=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_08_a3.py"
spec=importlib.util.spec_from_file_location("v3_08_a3_review",BASE); driver=importlib.util.module_from_spec(spec); spec.loader.exec_module(driver); driver.M=MODULE; driver.main()
