#!/usr/bin/env python3
import importlib.util
from pathlib import Path
W=Path("/home/robot/robot_ws_base_rl"); S=W/"src/tools/thesis_experiment/scripts/assess_v3_07_a1_identifiability.py"; V=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_07_a2.py"
s=importlib.util.spec_from_file_location("a2assess",S); d=importlib.util.module_from_spec(s); s.loader.exec_module(d); d.MODULE=V
original=d.assess
def assess(module):
    value=original(module); value["stage"]="V3-07-A2"; return value
d.assess=assess; d.main()
