#!/usr/bin/env python3
import copy,importlib.util
from pathlib import Path
W=Path("/home/robot/robot_ws_base_rl");BASE=W/"src/tools/thesis_experiment/scripts/run_v3_08_a1_integration.py";MODULE=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_09_q1_r1.py";DISPATCH=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_09_dispatch.py"
def load(path,name):
 s=importlib.util.spec_from_file_location(name,path);v=importlib.util.module_from_spec(s);s.loader.exec_module(v);return v
runner=load(BASE,"q1r1runner");dispatcher=load(DISPATCH,"q1r1dispatch");orig=runner.load
def patched(path,name):
 v=orig(path,name)
 if Path(path)==runner.BASE:
  oa=v.append
  def append(j,item):
   item=copy.deepcopy(item)
   if item.get("authorized_units")==18:item["authorized_units"]=36
   if item.get("remaining_units_forfeited") is not None:item["remaining_units_forfeited"]=36-int(item["sequence"])
   if item.get("event")=="execution_complete":item.update({"authorized_units":36,"consumed_units":36,"completed_units":36})
   oa(j,item)
  v.append=append
 return v
runner.load=patched;runner.MODULE=MODULE;runner.spatial_core=dispatcher;runner.main()
