#!/usr/bin/env python3
import importlib.util
from pathlib import Path
W=Path("/home/robot/robot_ws_base_rl")
S=W/"src/tools/thesis_experiment/scripts/run_v3_03_c1_closed_loop.py"
V=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_06_c6.py"
C=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_06_cached_core_r1.py"
def load(p,n):
    s=importlib.util.spec_from_file_location(n,p); m=importlib.util.module_from_spec(s); s.loader.exec_module(m); return m
v=load(V,"c6v"); cache=load(C,"c6cache"); d=load(S,"c6driver")
d.M=V; d.core=cache; d.apply_selection_method=v.apply_selection_method; d.main()
