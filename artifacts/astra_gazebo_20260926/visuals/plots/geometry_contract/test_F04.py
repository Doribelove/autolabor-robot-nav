"""Schema-only fixtures. No figure, fake episode or replacement evidence output."""
import copy
import importlib.util
import math
from pathlib import Path
import pytest

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('F04_plot_code',HERE/'generate_F04.py')
plot=importlib.util.module_from_spec(spec);spec.loader.exec_module(plot)


def inputs():
    start=1_000_000_000;end=start+40_000_000
    profile={'margin_m':.33,'physics_step_ns':2_000_000,'coarse_stride_steps':10}
    report={'schema':'astra_p04_engineering_sweep_v1','scope':plot.EXPECTED_SCOPE,
        'numeric_profile':profile,'source_window_ns':[start,end],
        'coarse_intervals':[{'start_ns':start,'end_ns':start+20_000_000,'covered_exactly':True,'conditional_clearance_lower_bound_m':.6},
                            {'start_ns':start+20_000_000,'end_ns':end,'covered_exactly':True,'conditional_clearance_lower_bound_m':.5}]}
    endpoints=[dict(stamp_ns=t,physical_xy_m=1.,conservative_xy_m=.67) for t in range(start,end+1,20_000_000)]
    intervals=[dict(start_ns=t,end_ns=t+2_000_000,robot_point_budget_m=.001,
        actor_Begin_variation_budget_m=0.,other_phase_variation_budget_m=.0001,
        relative_clearance_variation_budget_m=.0011) for t in range(start,end,2_000_000)]
    return report,endpoints,intervals


def test_only_preselected_future_run_admitted_and_outcome_false_retained():
    prereg={'figure_id':'F04','selected_future_run':plot.SELECTED_RUN,'seed':plot.SELECTED_SEED}
    receipt={'run_id':plot.SELECTED_RUN,'seed':plot.SELECTED_SEED,'attempt':1,'pass':False}
    plot.check_selection(prereg,plot.STAGE/'runs'/plot.SELECTED_RUN,receipt)
    with pytest.raises(ValueError):plot.check_selection(prereg,plot.STAGE/'runs/passive_sweep_q04_static',receipt)
    for key,value in (('seed',260926029),('attempt',True),('attempt',2),('run_id','synthetic'),('pass',None)):
        bad=dict(receipt);bad[key]=value
        with pytest.raises(ValueError):plot.check_selection(prereg,plot.STAGE/'runs'/plot.SELECTED_RUN,bad)


def test_complete_window_signed_margin_and_budget_sum():
    report,endpoints,intervals=inputs();out=plot.scalar_data(report,endpoints,intervals)
    assert len(out['endpoints'])==3 and len(out['intervals'])==20
    assert out['duration_s']==.04 and not out['display_issues']
    assert out['intervals'][0]['actor_Begin_variation_budget_m']==0


@pytest.mark.parametrize('mutation',['wrong_scope','wrong_margin','wrong_step','shifted_window','endpoint_order','margin_sign','non_grid_interval','negative_budget','budget_sum'])
def test_invalid_semantics_rejected(mutation):
    report,endpoints,intervals=inputs()
    if mutation=='wrong_scope':report['scope']='real_continuous_safety'
    elif mutation=='wrong_margin':report['numeric_profile']['margin_m']=.3
    elif mutation=='wrong_step':report['numeric_profile']['physics_step_ns']=4_000_000
    elif mutation=='shifted_window':report['source_window_ns'][0]+=1
    elif mutation=='endpoint_order':endpoints.reverse()
    elif mutation=='margin_sign':endpoints[0]['conservative_xy_m']=1.33
    elif mutation=='non_grid_interval':intervals[0]['start_ns']+=1;intervals[0]['end_ns']+=1
    elif mutation=='negative_budget':intervals[0]['robot_point_budget_m']=-1
    else:intervals[0]['relative_clearance_variation_budget_m']+=.001
    with pytest.raises(ValueError):plot.scalar_data(report,endpoints,intervals)


def test_missing_sources_remain_gaps_without_repair_or_interpolation():
    report,endpoints,intervals=inputs();endpoints.pop(1);intervals.pop(5)
    report['coarse_intervals'][0]['covered_exactly']=False
    data=plot.scalar_data(report,endpoints,intervals)
    assert len(data['missing_endpoint_ns'])==1 and len(data['missing_interval_start_ns'])==1
    _,series=plot.gapped_series(data['endpoints'],'stamp_ns',data['expected_endpoint_ns'],'physical_xy_m',data['start_ns'])
    assert series[0]==series[2]==1. and math.isnan(series[1])
    assert math.isnan(data['bounds'][0]['lower_bound_m'])


def test_negative_signed_clearance_is_not_clipped_to_zero():
    report,endpoints,intervals=inputs();endpoints[0].update(physical_xy_m=-.01,conservative_xy_m=-.34)
    data=plot.scalar_data(report,endpoints,intervals)
    assert data['endpoints'][0]['physical_xy_m']==-.01
    assert data['endpoints'][0]['conservative_xy_m']==-.34


def test_unconfirmed_review_never_promoted(tmp_path):
    source={'sha256':'a'*64};path=tmp_path/'review.json'
    path.write_text('{"pass":true,"status":"great","source_bindings":{}}')
    label,_=plot.safe_review_status(path,source)
    assert '未确认' in label
    label,b=plot.safe_review_status(None,source)
    assert '待完成' in label and b is None
