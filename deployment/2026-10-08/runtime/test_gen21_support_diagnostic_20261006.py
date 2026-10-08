"""Offline gen21 preparation/recursive lineage tests; no ROS or hardware."""
import ast
import copy
import importlib.util
import json
from pathlib import Path

import pytest

P=Path(__file__).with_name('run_gen21_support_diagnostic_20261006.py')
spec=importlib.util.spec_from_file_location('gen21_contract_under_test',P)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
PRIOR=m.EXT/'ros_log/gen20_support_measurement_20261006'


def kwargs():
    index=m.read(m.INDEX)
    return dict(read_json=m.read,digest_file=m.sha,resolve_path=m.external_path,
        parse_selection=m.parse_selection,digest_context=m.context_digest,
        validate_activation_only=m.validate_activation_only,
        validate_capture_summary=m.validate_capture_summary,
        frozen_scripts={19:(str(m._base.FROZEN.relative_to(m.W)),m._base.FROZEN_SHA),
                        20:(str(m.FROZEN.relative_to(m.W)),m.FROZEN_SHA)},
        confirmation_id=m.CONFIRMATION_ID,
        confirmation_source=m.read(index['receipt'])['confirmation_source'],
        confirmed_ns=m.ORIGINAL_CONFIRMED_NS)


def validate(**changes):
    options=kwargs();options.update(changes)
    return m._chain.validate_diagnostic_chain(PRIOR/'alignment_receipt.json',
        m.read(m.INDEX)['receipt'],m.read(m.GRASP_INDEX)['selection'],**options)


@pytest.fixture
def records():
    original=validate()
    # Object-layer counterexamples are injected independently of SHA checks.
    # Separate tests exercise file binding and actual immutable file hashes.
    data={p:m.read(p) for p in original['artifact_hashes']}
    hashes=original['artifact_hashes'].copy()
    bygen={node['selected']['generation']:node for node in original['nodes']}
    def run(**changes):
        options=dict(read_json=lambda p:copy.deepcopy(data[str(p)]),
                     digest_file=lambda p:hashes[str(p)])
        options.update(changes)
        return validate(**options)
    def node(gen):
        n=bygen[gen]
        return {key:data[str(n[key])] for key in
                ('receipt_path','event_path','selection_path','run_path','summary_path') if key in n}
    def sync(gen):
        n=node(gen);r=n['receipt_path'];event=n['event_path'];s=n['selection_path']
        r['source_context_sha256']=m.context_digest(event['source_context'])
        s['alignment_receipt']=copy.deepcopy(r)
        if 'run_path' in n:
            n['run_path']['selection_report']=copy.deepcopy(s)
            n['run_path']['capture_summary']=copy.deepcopy(n['summary_path'])
    return data,hashes,bygen,run,node,sync


def test_actual_gen20_chain_reaches_original_gen18_index_without_mutation():
    before=(m.INDEX.read_bytes(),m.GRASP_INDEX.read_bytes())
    chain=validate()
    assert [(n['receipt']['selection']['generation'],n['selected']['generation'])
            for n in chain['nodes']]==[(17,18),(18,19),(19,20)]
    assert len(chain['artifact_hashes'])==15
    assert all(n['receipt']['confirmed_ns']==m.ORIGINAL_CONFIRMED_NS for n in chain['nodes'])
    assert before==(m.INDEX.read_bytes(),m.GRASP_INDEX.read_bytes())


def test_gen20_is_not_a_direct_child_of_indexed_receipt(records):
    data,hashes,bygen,run,node,sync=records
    child=node(20);root=bygen[18]['receipt_path']
    child['receipt_path']['derivation']['parent_receipt']=str(root)
    child['receipt_path']['derivation']['parent_receipt_sha256']=hashes[str(root)]
    child['event_path']['derivation']=copy.deepcopy(child['receipt_path']['derivation']);sync(20)
    with pytest.raises(ValueError,match='chain_parent_selection_changed'):run()


def test_newest_parent_cannot_be_renamed_the_global_anchor(records):
    data,hashes,bygen,run,node,sync=records
    options=kwargs();options.update(read_json=lambda p:copy.deepcopy(data[str(p)]),digest_file=lambda p:hashes[str(p)])
    with pytest.raises(ValueError,match='chain_indexed_selection_changed'):
        m._chain.validate_diagnostic_chain(PRIOR/'alignment_receipt.json',bygen[19]['receipt_path'],
            m.read(m.GRASP_INDEX)['selection'],**options)


def test_chain_cycle_is_rejected_without_unbounded_recursion(records):
    data,hashes,bygen,run,node,sync=records
    child=node(20);path=bygen[20]['receipt_path']
    child['receipt_path']['derivation']['parent_receipt']=str(path)
    child['receipt_path']['derivation']['parent_receipt_sha256']=hashes[str(path)]
    child['event_path']['derivation']=copy.deepcopy(child['receipt_path']['derivation']);sync(20)
    with pytest.raises(ValueError,match='chain_cycle'):run()


def test_depth_bound_cannot_silently_truncate_chain():
    with pytest.raises(ValueError,match='chain_depth_exceeded'):validate(max_depth=2)


@pytest.mark.parametrize('gen',[18,19,20])
@pytest.mark.parametrize('field',['confirmation_id','confirmed_ns','confirmation_source'])
def test_every_ancestor_keeps_actual_original_confirmation(records,gen,field):
    data,hashes,bygen,run,node,sync=records
    n=node(gen);n['receipt_path'][field]='invented';sync(gen)
    with pytest.raises(ValueError,match='chain_original_confirmation_changed'):run()


@pytest.mark.parametrize('fault',['parent_hash','event_hash','selection_hash','source_context_hash',
    'generation_skip','old_stamp','epoch','context_depth_scale','activation_binding','feedback',
    'feedback_frame','feedback_nan','feedback_backwards','event_motion','new_confirmation',
    'run_motion','run_incomplete','run_source_changed','run_capture_failed','run_error','run_script',
    'capture_open','capture_wrong_generation','prior_run_hash','prior_summary_hash','wrong_prior_directory',
    'deployment_parent','deployment_selection','deployment_motion','deployment_incomplete'])
def test_recursive_contract_rejects_corrupt_edges_and_incomplete_evidence(records,fault):
    data,hashes,bygen,run,node,sync=records
    n=node(20);r=n['receipt_path'];event=n['event_path'];selection=n['selection_path'];report=n['run_path'];derivation=r['derivation']
    if fault=='parent_hash':derivation['parent_receipt_sha256']='wrong'
    elif fault=='event_hash':r['event_file_sha256']='wrong'
    elif fault=='selection_hash':derivation['prior_selection_sha256']='wrong'
    elif fault=='source_context_hash':r['source_context_sha256']='wrong'
    elif fault=='generation_skip':selection['new_selection']['generation']=21;report['new_selection']=copy.deepcopy(selection['new_selection'])
    elif fault=='old_stamp':selection['new_selection']['stamp_ns']=selection['previous_selection']['stamp_ns'];report['new_selection']=copy.deepcopy(selection['new_selection'])
    elif fault=='epoch':r['epoch_ns']=str(int(r['epoch_ns'])+1);event['source_context']['epoch_ns']=int(r['epoch_ns'])
    elif fault=='context_depth_scale':event['source_context']['camera']['depth_scale']+=.000001
    elif fault=='activation_binding':derivation['new_activation_stamp_ns']=str(int(derivation['new_activation_stamp_ns'])+1)
    elif fault=='feedback':r['bound_feedback']['positions'][0]+=.1
    elif fault=='feedback_frame':r['bound_feedback']['frame_id']='commanded'
    elif fault=='feedback_nan':r['bound_feedback']['positions'][0]=float('nan')
    elif fault=='feedback_backwards':r['bound_feedback']['stamp_ns']='1'
    elif fault=='event_motion':event['robot_commands_sent']=True
    elif fault=='new_confirmation':derivation['new_user_confirmation_received']=True
    elif fault=='run_motion':report['motion_commands_sent']=True
    elif fault=='run_incomplete':report['complete']=False
    elif fault=='run_source_changed':report['source_unchanged']=False
    elif fault=='run_capture_failed':report['capture_returncode']=1
    elif fault=='run_error':report['error']='failed'
    elif fault=='run_script':report['source_sha256'][str(m.FROZEN.relative_to(m.W))]='wrong'
    elif fault=='capture_open':n['summary_path']['archive_closed']=False
    elif fault=='capture_wrong_generation':n['summary_path']['initial_generation']=18
    elif fault=='prior_run_hash':derivation['prior_diagnostic_run_sha256']='wrong'
    elif fault=='prior_summary_hash':derivation['prior_diagnostic_summary_sha256']='wrong'
    elif fault=='wrong_prior_directory':report['prior_diagnostic']=str(PRIOR)
    elif fault.startswith('deployment_'):
        deployment=data[report['deployment']]
        if fault=='deployment_parent':deployment['alignment_receipt_path']=str(bygen[18]['receipt_path'])
        elif fault=='deployment_selection':deployment['current_selection']['generation']=18
        elif fault=='deployment_motion':deployment['robot_commands_sent']=True
        elif fault=='deployment_incomplete':deployment['static_validation_passed']=False
    event['derivation']=copy.deepcopy(derivation);event['bound_feedback']=copy.deepcopy(r['bound_feedback'])
    if fault!='source_context_hash':sync(20)
    else:selection['alignment_receipt']=copy.deepcopy(r);report['selection_report']=copy.deepcopy(selection)
    with pytest.raises((ValueError,KeyError)):run()


def test_small_per_edge_pose_changes_cannot_accumulate_past_original_anchor(records):
    data,hashes,bygen,run,node,sync=records
    for gen,delta in ((19,.003),(20,.006)):
        n=node(gen)
        n['receipt_path']['bound_feedback']['positions'][0]=node(18)['receipt_path']['bound_feedback']['positions'][0]+delta
        n['event_path']['bound_feedback']=copy.deepcopy(n['receipt_path']['bound_feedback']);sync(gen)
    with pytest.raises(ValueError,match='chain_accumulated_alignment_changed'):run()


@pytest.fixture
def deployment_artifacts(tmp_path,monkeypatch):
    prior_run=m.read(PRIOR/'run.json');d=m.read(prior_run['deployment'])
    d['current_selection']=prior_run['new_selection']
    d['alignment_receipt_path']=str(PRIOR/'alignment_receipt.json')
    d['alignment_receipt_sha256']=m.sha(PRIOR/'alignment_receipt.json')
    vp=tmp_path/'validation.json';relative='src/alicia_flexible_grasp_supervisor/scripts/camera_node.py'
    m.write_new(vp,dict(tests_passed=True,reviewed=True,source_unchanged=True,files={relative:m.sha(m.W/relative)}))
    d.update(manifest_path=str(vp),manifest_sha256=m.sha(vp))
    dp=tmp_path/'deployment.json';m.write_new(dp,d)
    original=m.external_path
    monkeypatch.setattr(m,'external_path',lambda p:Path(p).resolve() if tmp_path in Path(p).resolve().parents else original(p))
    return dp,vp


def test_future_deployment_artifacts_bind_gen20_and_complete_older_chain(deployment_artifacts):
    result=m.artifacts(deployment_artifacts[0],PRIOR)
    assert result['selection']['generation']==20
    assert result['parent']['selection']['generation']==19
    assert str(m.CHAIN_PATH.relative_to(m.W)) in result['source_hashes']
    assert str(m.read(m.INDEX)['receipt']) in result['artifact_hashes']


@pytest.mark.parametrize('fault',['wrong_parent','wrong_parent_hash','wrong_generation','source_changed','unreviewed'])
def test_future_deployment_cannot_skip_newest_parent_or_validated_source(deployment_artifacts,fault):
    dp,vp=deployment_artifacts;d=m.read(dp);v=m.read(vp)
    if fault=='wrong_parent':d['alignment_receipt_path']=m.read(m.INDEX)['receipt']
    elif fault=='wrong_parent_hash':d['alignment_receipt_sha256']='wrong'
    elif fault=='wrong_generation':d['current_selection']['generation']=19
    elif fault=='source_changed':v['files'][next(iter(v['files']))]='wrong'
    elif fault=='unreviewed':v['reviewed']=False
    vp.write_text(json.dumps(v));d['manifest_sha256']=m.sha(vp);dp.write_text(json.dumps(d))
    with pytest.raises(ValueError):m.artifacts(dp,PRIOR)


@pytest.mark.parametrize('change',[{}, {'generation':20},{'generation':22},{'mode':'carton'},
    {'strategy':'direct'},{'stamp_ns':1},{'generation':True}])
def test_ack_requires_only20_to21(change):
    old=m.read(PRIOR/'run.json')['new_selection'];new=dict(old,generation=21,stamp_ns=str(old['stamp_ns']+1));new.update(change)
    if not change:m.validate_selection_ack(old,new)
    else:
        with pytest.raises((ValueError,TypeError)):m.validate_selection_ack(old,new)


def test_only_perception_select_and_bounded_capture_no_motion_or_global_index_write():
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
    code=P.read_text()
    assert "'--seconds','15'" in code and "ready['initial_generation']==20" in code
    assert "os.environ['ROS_LOG_DIR']=str(output/'ros')" in code
    assert "MOUNT.is_mount(),'SanStand_unavailable_no_fallback'" in code
    chain_tree=ast.parse(m.CHAIN_PATH.read_text())
    assert not any(isinstance(n,(ast.Import,ast.ImportFrom)) and 'rospy' in ast.dump(n) for n in ast.walk(chain_tree))
