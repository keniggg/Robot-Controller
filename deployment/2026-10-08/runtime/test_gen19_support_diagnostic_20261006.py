"""Pure provenance/continuity tests; never call the diagnostic main or ROS."""
import copy
import importlib.util
import json
from pathlib import Path

import pytest

P=Path(__file__).with_name('run_gen19_support_diagnostic_20261006.py')
spec=importlib.util.spec_from_file_location('gen19_diagnostic_under_test',P)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def context():
    return dict(selection=dict(mode='unknown',strategy='observe_return',generation=18,stamp_ns=100),
        epoch_ns=4,camera=dict(K=[1,2,3,4],runtime_profile_geometry=dict(geometry_activation_stamp_ns='50',device='fixed')),
        handeye={'translation':[1,2,3]},model_sha256='old_model')


def prior():
    selection=dict(mode='unknown',strategy='observe_return',generation=17,stamp_ns=100)
    next_selection=dict(selection,generation=18,stamp_ns=200)
    r=dict(confirmation_id=m.CONFIRMATION_ID,confirmation_source=dict(kind='user_message',message_id='actual-id',text=m.CONFIRMATION_TEXT),
           confirmed_ns='5',kind='explicit_user_target_alignment',target_aligned=True,selection=selection)
    o={k:copy.deepcopy(r[k]) for k in ('confirmation_source','confirmed_ns')}
    p=dict(response={'success':True},alignment_receipt=copy.deepcopy(r),previous_selection=selection,new_selection=next_selection)
    manifest=dict(state='complete',recorder_closed=True,result_evidence=dict(runner_returncode=3,observations=[]),
                  failure_reason='NO_FRESH_MODE_PREVIEW generation=18')
    command=dict(runner_finished=True,evidence=dict(runner_returncode=3))
    lines=['CONFIRMED_DISPATCH '+json.dumps(dict(phase=s,motion_authorized=False)) for s in
           ('metric_entry','metric_runner_start_requested','metric_runner_finished')]
    lines+=['CANDIDATE_COMPUTATION_STOP success=True']
    return r,o,p,manifest,command,'\n'.join(lines)


def test_valid_prior_preserves_the_actual_old_confirmation():
    values=prior();before=copy.deepcopy(values)
    assert m.validate_prior_values(*values)==values[2]['new_selection']
    assert values==before


@pytest.mark.parametrize('fault',['not_closed','not_complete','runner_live','runner_success','motion_observation',
    'confirmation_new_time','confirmation_new_text','wrong_parent','wrong_generation','worker_not_stopped','action_dispatch','unexpected_event'])
def test_rejects_missing_terminal_or_original_alignment_evidence(fault):
    r,o,p,f,c,log=prior()
    if fault=='not_closed':f['recorder_closed']=False
    elif fault=='not_complete':f['state']='active'
    elif fault=='runner_live':c['runner_finished']=False
    elif fault=='runner_success':c['evidence']['runner_returncode']=0
    elif fault=='motion_observation':f['result_evidence']['observations']=[{'motion':True}]
    elif fault=='confirmation_new_time':r['confirmed_ns']='9'
    elif fault=='confirmation_new_text':r['confirmation_source']['text']='generated'
    elif fault=='wrong_parent':p['alignment_receipt']['confirmed_ns']='7'
    elif fault=='wrong_generation':p['new_selection']['generation']=19
    elif fault=='worker_not_stopped':log=log.replace('CANDIDATE_COMPUTATION_STOP success=True','CANDIDATE_COMPUTATION_STOP success=False')
    elif fault=='action_dispatch':log=log.replace('"motion_authorized": false','"motion_authorized": true')
    elif fault=='unexpected_event':log+='\nEXECUTE_OBSERVATION success=True'
    with pytest.raises(ValueError):m.validate_prior_values(r,o,p,f,c,log)


def test_activation_change_keeps_every_other_context_value_and_does_not_mutate_inputs():
    before=context();after=copy.deepcopy(before);after['camera']['runtime_profile_geometry']['geometry_activation_stamp_ns']='60'
    original=copy.deepcopy((before,after))
    assert m.validate_activation_only(before,after)==('50','60')
    assert (before,after)==original


@pytest.mark.parametrize('fault',['same_activation','backwards_activation','float_activation','bool_activation','epoch','camera','handeye','model','selection'])
def test_rejects_any_geometry_or_authority_change_in_addition_to_activation(fault):
    before=context();after=copy.deepcopy(before);after['camera']['runtime_profile_geometry']['geometry_activation_stamp_ns']='60'
    g=after['camera']['runtime_profile_geometry']
    if fault=='same_activation':g['geometry_activation_stamp_ns']='50'
    elif fault=='backwards_activation':g['geometry_activation_stamp_ns']='49'
    elif fault=='float_activation':g['geometry_activation_stamp_ns']=60.1
    elif fault=='bool_activation':g['geometry_activation_stamp_ns']=True
    elif fault=='epoch':after['epoch_ns']=5
    elif fault=='camera':after['camera']['K'][0]=99
    elif fault=='handeye':after['handeye']['translation'][0]=99
    elif fault=='model':after['model_sha256']='new_model'
    elif fault=='selection':after['selection']['stamp_ns']=101
    with pytest.raises(ValueError):m.validate_activation_only(before,after)


def test_new_selection_is_exactly_one_fresh_generation():
    old=context()['selection'];new=dict(old,generation=19,stamp_ns=101)
    m.validate_selection_ack(old,new)


@pytest.mark.parametrize('new',[
    dict(mode='unknown',strategy='observe_return',generation=18,stamp_ns=101),
    dict(mode='unknown',strategy='observe_return',generation=20,stamp_ns=101),
    dict(mode='unknown',strategy='observe_return',generation=19,stamp_ns=100),
    dict(mode='carton',strategy='direct',generation=19,stamp_ns=101),
])
def test_wrong_ack_is_not_treated_as_continued_capture(new):
    with pytest.raises(ValueError):m.validate_selection_ack(context()['selection'],new)


def test_output_records_cannot_replace_an_existing_experiment(tmp_path):
    p=tmp_path/'run.json';p.write_text('existing')
    with pytest.raises(FileExistsError):m.write_new(p,dict(new=True))
    assert p.read_text()=='existing'


def good_capture():
    return dict(archive_closed=True,complete_without_writer_error=True,byte_budget_passed=True,
                directory_budget_passed=True,cooperative_total_deadline_passed=True)


def test_closed_within_original_capture_budgets_is_accepted():
    m.validate_capture_summary(good_capture())


@pytest.mark.parametrize('key',list(good_capture()))
def test_partial_overrun_or_unsealed_capture_is_not_complete(key):
    summary=good_capture();summary[key]=False
    with pytest.raises(ValueError):m.validate_capture_summary(summary)


def test_changed_source_revokes_complete_and_preserves_original_error():
    r=dict(complete=True,error='original_capture_failure')
    m.bind_final_source_status(r,False)
    assert r==dict(complete=False,error='original_capture_failure',source_unchanged=False)


def test_changed_source_alone_is_an_explicit_failure():
    r=dict(complete=True);m.bind_final_source_status(r,False)
    assert r['complete'] is False and r['source_unchanged'] is False
    assert r['error']=='diagnostic_source_changed_during_capture'
