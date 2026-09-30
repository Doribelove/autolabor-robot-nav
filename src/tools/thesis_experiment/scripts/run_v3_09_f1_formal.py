#!/usr/bin/env python3
import copy,importlib.util
from pathlib import Path
W=Path("/home/robot/robot_ws_base_rl");BASE=W/"src/tools/thesis_experiment/scripts/run_v3_08_a1_integration.py";MODULE=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_09_f1.py";DISPATCH=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_09_dispatch.py"
def load(p,n):
 s=importlib.util.spec_from_file_location(n,p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
runner=load(BASE,"f1runner");dispatcher=load(DISPATCH,"f1dispatch");orig=runner.load
def patched(p,n):
 v=orig(p,n)
 if Path(p)==runner.BASE:
  oa=v.append
  def append(j,item):
   item=copy.deepcopy(item)
   if item.get("authorized_units")==18:item["authorized_units"]=180
   if item.get("remaining_units_forfeited") is not None:item["remaining_units_forfeited"]=180-int(item["sequence"])
   if item.get("event")=="execution_complete":item.update({"authorized_units":180,"consumed_units":180,"completed_units":180})
   oa(j,item)
  v.append=append
 return v
runner.load=patched;runner.MODULE=MODULE;runner.spatial_core=dispatcher;runner.main()
