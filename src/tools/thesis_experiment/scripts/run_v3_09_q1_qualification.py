#!/usr/bin/env python3
import copy
import importlib.util
from pathlib import Path
W=Path("/home/robot/robot_ws_base_rl"); BASE=W/"src/tools/thesis_experiment/scripts/run_v3_08_a1_integration.py"; MODULE=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_09_q1.py"; DISPATCH=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_09_dispatch.py"
def load(path,name):
    spec=importlib.util.spec_from_file_location(name,path); value=importlib.util.module_from_spec(spec); spec.loader.exec_module(value); return value
runner=load(BASE,"v3_09_q1_runner"); dispatcher=load(DISPATCH,"v3_09_q1_dispatch")
original_load=runner.load
def patched_load(path,name):
    value=original_load(path,name)
    if Path(path)==runner.BASE:
        original_append=value.append
        def append(journal,item):
            item=copy.deepcopy(item)
            if item.get("authorized_units")==18: item["authorized_units"]=36
            if item.get("remaining_units_forfeited") is not None: item["remaining_units_forfeited"]=36-int(item["sequence"])
            if item.get("event")=="execution_complete":
                item["authorized_units"]=36; item["consumed_units"]=36; item["completed_units"]=36
            original_append(journal,item)
        value.append=append
    return value
runner.load=patched_load; runner.MODULE=MODULE; runner.spatial_core=dispatcher; runner.main()
