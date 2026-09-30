#!/usr/bin/env python3
import importlib.util
from pathlib import Path
W=Path("/home/robot/robot_ws_base_rl"); S=W/"src/tools/thesis_experiment/scripts/run_v3_01_r1_qualification.py"; V=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_07_a2.py"
s=importlib.util.spec_from_file_location("a2run",S); d=importlib.util.module_from_spec(s); s.loader.exec_module(d); d.M=V; d.main()
