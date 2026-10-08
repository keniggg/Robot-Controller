"""Offline generation handoff tests using immutable gen19 evidence; no ROS."""
import ast
import copy
import importlib.util
from pathlib import Path

import pytest

P=Path(__file__).with_name('run_gen20_support_diagnostic_20261006.py')
spec=importlib.util.spec_from_file_location('gen20_contract_under_test',P)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
PRIOR=m.EXT/'ros_log/gen19_support_measurement_20261006'


def values():
    return tuple(m.read(PRIOR/p) for p in ('run.json','alignment_receipt.json',
        'alignment_revalidation_event.json','new_target_selection.json','raw_support/summary.json'))


def synchronize(run,r,o,s,summary):
    s['alignment_receipt']=copy.deepcopy(r)
    run['selection_report']=copy.deepcopy(s)
    run['capture_summary']=copy.deepcopy(summary)


def test_real_gen19_complete_diagnostic_is_not_quality_pass_and_keeps_original_reply():
    v=values();original=copy.deepcopy(v)
    assert m.validate_prior_values(*v)['generation']==19
    assert v==original
    assert v[0]['original_confirmed_ns']=='1791267609469281435'
    assert m.read(PRIOR/'live_followup.json')['status']['target_lost'] is True


@pytest.mark.parametrize('fault', ['incomplete','changed_source','capture_failed','run_error','wrong_scope',
    'run_not_preview','selection_not_preview','planning','runner','motion','torque','new_confirmation',
    'wrong_script','summary_mismatch','summary_open','summary_writer_error','summary_bytes','summary_directory',
    'summary_deadline','receipt_kind','target_unaligned','confirmation_id','confirmation_time','confirmation_text',
    'event_confirmation','selection_failed','nested_receipt','nested_selection','wrong_previous','wrong_generation',
    'wrong_strategy','backwards_stamp','run_selection','event_motion','event_target','event_epoch',
    'event_selection','event_feedback','event_derivation'])
def test_rejects_changed_prior_authority_or_incomplete_diagnostic(fault):
    run,r,o,s,summary=values()
    if fault=='incomplete':run['complete']=False
    elif fault=='changed_source':run['source_unchanged']=False
    elif fault=='capture_failed':run['capture_returncode']=1
    elif fault=='run_error':run['error']='failed'
    elif fault=='wrong_scope':run['scope']='gen19_full_grasp'
    elif fault=='run_not_preview':run['preview_only']=False
    elif fault=='selection_not_preview':s['preview_only']=False
    elif fault in ('planning','runner','motion','torque','new_confirmation'):
        field={'planning':'planning_requested','runner':'runner_started','motion':'motion_commands_sent',
               'torque':'torque_commands_sent','new_confirmation':'new_user_confirmation_received'}[fault]
        run[field]=True;s[field]=True
    elif fault=='wrong_script':run['source_sha256'][str(m.FROZEN.relative_to(m.W))]='wrong'
    elif fault=='summary_mismatch':run['capture_summary']['archive_closed']=False
    elif fault.startswith('summary_'):
        field={'summary_open':'archive_closed','summary_writer_error':'complete_without_writer_error',
               'summary_bytes':'byte_budget_passed','summary_directory':'directory_budget_passed',
               'summary_deadline':'cooperative_total_deadline_passed'}[fault]
        summary[field]=False;run['capture_summary']=copy.deepcopy(summary)
    elif fault=='receipt_kind':r['kind']='generated'
    elif fault=='target_unaligned':r['target_aligned']=False
    elif fault=='confirmation_id':r['confirmation_id']='invented'
    elif fault=='confirmation_time':r['confirmed_ns']=o['confirmed_ns']=run['original_confirmed_ns']='1791286986393163442'
    elif fault=='confirmation_text':r['confirmation_source']['text']='new invented reply';o['confirmation_source']=copy.deepcopy(r['confirmation_source'])
    elif fault=='event_confirmation':o['confirmation_source']['message_id']='different'
    elif fault=='selection_failed':s['response']['success']=False
    elif fault=='nested_receipt':s['alignment_receipt']['epoch_ns']='9'
    elif fault=='nested_selection':run['selection_report']['previous_selection']['stamp_ns']=1
    elif fault=='wrong_previous':s['previous_selection']['generation']=17
    elif fault=='wrong_generation':s['new_selection']['generation']=20
    elif fault=='wrong_strategy':s['new_selection']['strategy']='legacy'
    elif fault=='backwards_stamp':s['new_selection']['stamp_ns']=s['previous_selection']['stamp_ns']
    elif fault=='run_selection':run['new_selection']['stamp_ns']+=1
    elif fault=='event_motion':o['robot_commands_sent']=True
    elif fault=='event_target':o['target_aligned']=False
    elif fault=='event_epoch':o['source_context']['epoch_ns']+=1
    elif fault=='event_selection':o['source_context']['selection']['generation']=19
    elif fault=='event_feedback':o['bound_feedback']['positions'][0]+=.01
    elif fault=='event_derivation':o['derivation']['new_user_confirmation_received']=True
    if fault not in ('nested_receipt','nested_selection','summary_mismatch'):
        synchronize(run,r,o,s,summary)
    with pytest.raises((ValueError,KeyError)):m.validate_prior_values(run,r,o,s,summary)


def test_new_ack_normalizes_ros_decimal_stamp_and_requires_exact_one_generation():
    old=values()[0]['new_selection'];new=dict(old,generation=20,stamp_ns=str(old['stamp_ns']+1))
    m.validate_selection_ack(old,new)


@pytest.mark.parametrize('change',[{'generation':19},{'generation':21},{'mode':'known'},
    {'strategy':'direct'},{'stamp_ns':1},{'generation':True}])
def test_new_ack_rejects_wrong_or_old_selection(change):
    old=values()[0]['new_selection'];new=dict(old,generation=20,stamp_ns=old['stamp_ns']+1);new.update(change)
    with pytest.raises((ValueError,TypeError)):m.validate_selection_ack(old,new)


def test_only_new_camera_activation_can_differ():
    before=values()[2]['source_context'];after=copy.deepcopy(before)
    after['camera']['runtime_profile_geometry']['geometry_activation_stamp_ns']=str(int(before['camera']['runtime_profile_geometry']['geometry_activation_stamp_ns'])+1)
    m.validate_activation_only(before,after)
    after['camera']['depth_scale']+=.000001
    with pytest.raises(ValueError):m.validate_activation_only(before,after)


def test_source_change_cannot_report_complete_or_hide_original_error():
    r=dict(complete=True,error='original_failure');m.bind_final_source_status(r,False)
    assert r==dict(complete=False,error='original_failure',source_unchanged=False)


def test_no_execution_or_ros_import_on_module_load_and_only_selection_mutation():
    root=ast.parse(P.read_text())
    imports=[n for n in root.body if isinstance(n,(ast.Import,ast.ImportFrom))]
    assert not any('rospy' in ast.dump(n) for n in imports)
    main=next(n for n in root.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    proxies=[n for n in ast.walk(main) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='ServiceProxy']
    assert len(proxies)==1 and ast.literal_eval(proxies[0].args[0])=='/grasp_mode/select'
    forbidden=('Publisher','set_param','start_streaming','request_plan')
    assert not any(isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr in forbidden for n in ast.walk(main))
    writes=[n for n in ast.walk(main) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='write_new']
    assert all('INDEX' not in ast.dump(n.args[0]) for n in writes)


def test_write_new_never_overwrites(tmp_path):
    p=tmp_path/'record.json';m.write_new(p,{'original':True})
    with pytest.raises(FileExistsError):m.write_new(p,{'replacement':True})
    assert m.read(p)=={'original':True}


@pytest.fixture
def deployment_artifacts(tmp_path,monkeypatch):
    run,r,o,s,summary=values()
    d=m.read(run['deployment'])
    d['current_selection']=dict(run['new_selection'],stamp_ns=str(run['new_selection']['stamp_ns']))
    d['alignment_receipt_path']=str(PRIOR/'alignment_receipt.json')
    d['alignment_receipt_sha256']=m.sha(PRIOR/'alignment_receipt.json')
    vp=tmp_path/'validation.json'
    relative='src/alicia_flexible_grasp_supervisor/scripts/camera_node.py'
    v=dict(tests_passed=True,reviewed=True,source_unchanged=True,files={relative:m.sha(m.W/relative)})
    m.write_new(vp,v);d.update(manifest_path=str(vp),manifest_sha256=m.sha(vp))
    dp=tmp_path/'deployment.json';m.write_new(dp,d)
    index=tmp_path/'index.json';grasp=tmp_path/'grasp.json'
    m.write_new(index,dict(receipt=r['derivation']['parent_receipt'],confirmation_id=m.CONFIRMATION_ID))
    m.write_new(grasp,dict(selection=r['selection']))
    original=m.external_path
    monkeypatch.setattr(m,'external_path',lambda p:Path(p).resolve() if tmp_path in Path(p).resolve().parents else original(p))
    monkeypatch.setattr(m,'INDEX',index);monkeypatch.setattr(m,'GRASP_INDEX',grasp)
    return dp,vp,index,grasp


def test_deployment_matches_diagnostic19_and_unchanged_real_legacy18_edge(deployment_artifacts):
    dp,_,_,_=deployment_artifacts
    before=(m.INDEX.read_bytes(),m.GRASP_INDEX.read_bytes())
    result=m.artifacts(dp,PRIOR)
    assert result['selection']['generation']==19
    assert result['parent']['selection']['generation']==18
    assert m.FROZEN_SHA==result['source_hashes'][str(m.FROZEN.relative_to(m.W))]
    assert result['artifact_hashes'][str(PRIOR/'run.json')]==m.sha(PRIOR/'run.json')
    assert before==(m.INDEX.read_bytes(),m.GRASP_INDEX.read_bytes())


@pytest.mark.parametrize('fault',['deployment_not_loaded','not_finished','robot_command','new_authority',
    'selection_changed','wrong_generation','wrong_parent','old_index_changed','grasp_index_changed',
    'unreviewed','source_changed','manifest_hash'])
def test_deployment_cannot_silently_repair_invalid_parent_or_source(deployment_artifacts,fault):
    dp,vp,index,grasp=deployment_artifacts
    d=m.read(dp);v=m.read(vp)
    if fault=='deployment_not_loaded':d['camera_loaded']=False
    elif fault=='not_finished':d['events'][-1]['event']='starting'
    elif fault=='robot_command':d['robot_commands_sent']=True
    elif fault=='new_authority':d['new_motion_authority_created']=True
    elif fault=='selection_changed':d['selection_changed']=True
    elif fault=='wrong_generation':d['current_selection']['generation']=18
    elif fault=='wrong_parent':d['alignment_receipt_path']=values()[1]['derivation']['parent_receipt']
    elif fault=='old_index_changed':index.write_text('{}')
    elif fault=='grasp_index_changed':grasp.write_text('{"selection":{"mode":"unknown","strategy":"observe_return","generation":19,"stamp_ns":999}}')
    elif fault=='unreviewed':v['reviewed']=False
    elif fault=='source_changed':v['files'][next(iter(v['files']))]='wrong'
    elif fault=='manifest_hash':d['manifest_sha256']='wrong'
    if fault in ('unreviewed','source_changed'):
        import json
        vp.write_text(json.dumps(v));d['manifest_sha256']=m.sha(vp)
    import json
    dp.write_text(json.dumps(d))
    with pytest.raises((ValueError,KeyError)):m.artifacts(dp,PRIOR)


def test_exact_recorded_new_deployment_binds_direct_gen19_receipt():
    deployment=m.EXT/'ros_log/regional_production_20261006T123748.120315Z/result.json'
    before=(m.INDEX.read_bytes(),m.GRASP_INDEX.read_bytes())
    result=m.artifacts(deployment,PRIOR)
    assert result['deployment']['alignment_receipt_path']==str(result['parent_path'])
    assert result['parent_path']==PRIOR/'alignment_receipt.json'
    assert result['deployment']['alignment_receipt_sha256']==m.sha(result['parent_path'])
    assert result['parent']['derivation']['parent_receipt']==m.read(m.INDEX)['receipt']
    assert before==(m.INDEX.read_bytes(),m.GRASP_INDEX.read_bytes())


def test_direct_gen19_parent_needs_its_exact_hash(deployment_artifacts):
    import json
    dp,_,_,_=deployment_artifacts;d=m.read(dp)
    d['alignment_receipt_sha256']=m.sha(Path(values()[1]['derivation']['parent_receipt']))
    dp.write_text(json.dumps(d))
    with pytest.raises(ValueError,match='deployment_direct_gen19_parent_changed'):m.artifacts(dp,PRIOR)
