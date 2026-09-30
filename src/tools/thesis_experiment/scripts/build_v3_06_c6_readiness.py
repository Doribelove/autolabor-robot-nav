#!/usr/bin/env python3
import importlib.util
from pathlib import Path
W=Path("/home/robot/robot_ws_base_rl")
S=W/"src/tools/thesis_experiment/scripts/build_v3_02_t1_readiness.py"
V=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_06_c6.py"
s=importlib.util.spec_from_file_location("c6b",S); d=importlib.util.module_from_spec(s); s.loader.exec_module(d)
d.M=V; d.main()
