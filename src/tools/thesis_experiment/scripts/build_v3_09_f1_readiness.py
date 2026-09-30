#!/usr/bin/env python3
import importlib.util
from pathlib import Path
W=Path("/home/robot/robot_ws_base_rl");B=W/"src/tools/thesis_experiment/scripts/build_v3_02_t1_readiness.py";M=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_09_f1.py";s=importlib.util.spec_from_file_location("f1b",B);d=importlib.util.module_from_spec(s);s.loader.exec_module(d);d.M=M;d.main()
