import importlib.util
from pathlib import Path
W=Path("/home/robot/robot_ws_base_rl")
P=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_06_c6.py"
s=importlib.util.spec_from_file_location("c6test",P); V=importlib.util.module_from_spec(s); s.loader.exec_module(V)
def test_schedule():
    x=V.exact_schedule(V.effective_contract())
    assert x["row_count"]==36
    assert x["seeds"]==[6531,6532,6533,6534,6535,6536]
    assert x["methods"]==["sparse_control","dense_v020_v040"]
