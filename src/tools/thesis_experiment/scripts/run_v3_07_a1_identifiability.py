#!/usr/bin/env python3
import importlib.util
from pathlib import Path
W=Path("/home/robot/robot_ws_base_rl")
SOURCE=W/"src/tools/thesis_experiment/scripts/run_v3_01_r1_qualification.py"
V3=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_07_a1.py"
s=importlib.util.spec_from_file_location("v3_07_a1_runner",SOURCE); d=importlib.util.module_from_spec(s); s.loader.exec_module(d); d.M=V3; d.main()
