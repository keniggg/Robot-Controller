from pathlib import Path
import hashlib,json,shutil
W=Path(__file__).resolve().parent.parent;C=W/'.runtime/gen16_height_recipe_policy_test_20261013';assert not C.exists()
rel='src/alicia_flexible_grasp_supervisor/tests/test_selected_contact_height_recipe.py';old=W/rel;new=C/rel;new.parent.mkdir(parents=True);shutil.copy2(old,new)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
(C/'baseline_sha256.json').write_text(json.dumps({rel:sha(old)},indent=2)+'\n')
s=new.read_text().replace("@pytest.mark.parametrize('defer', [False, True])\ndef test_observe_return_override_forwards_options_only_after_original_yield_policy(options, defer):", "@pytest.mark.parametrize('measured_opportunity', [False, True])\ndef test_observe_return_override_keeps_height_search_and_forwards_options(options, measured_opportunity):")
s=s.replace('node._contact_yield_measured_opportunity = lambda p: defer','node._contact_yield_measured_opportunity = lambda p: measured_opportunity')
start=s.index('    if defer:\n',s.index('def test_observe_return_override_keeps_height'))
s=s[:start]+'''    # The measured-view opportunity does not suppress a same-station height
    # retry. The original candidate deadline and option identities are retained.
    assert returned == 'fresh-super-check' and len(forwarded) == 1
    args, passed = forwarded[0]
    assert args == (plan, prepared, state, result, 123.)
    if options == 'ordinary': assert passed == {}
    else:
        assert passed['preferred_recipe'] is kwargs.get('preferred_recipe')
        assert passed['skip_proposal_indices'] == kwargs.get('skip_proposal_indices', ())
    assert 'contact_height_window_search' not in state
    assert node._contact_yield_search['deferred_height_families'] == []


@pytest.mark.parametrize('measured_opportunity', [False, True])
def test_observe_return_override_runs_real_height_search_and_fresh_publication_check(measured_opportunity):
    from alicia_grasp_modes.observe_return_remote import ObserveReturnRemoteMixin
    class Node(ObserveReturnRemoteMixin, remote.RemoteGrasp6DNode):
        pass
    values, _ = bound_fixture()
    node, source, prepared, runtime, _, _, calls, _, _ = values
    # Retain the existing real geometry/height-search fixture and its synthetic
    # strict transport; add the production mode override to its method chain.
    node.__class__ = Node
    node._contact_yield_search = dict(key='exact-request', incumbent_found=False,
        fallback=False, deferred_height_families=[])
    node._contact_yield_request_key = lambda p: 'exact-request'
    node._contact_yield_measured_opportunity = lambda p: measured_opportunity
    selected = select(values)
    assert selected.plan_id != source.plan_id
    assert len(calls) == 2
    assert [operation for _,operation,_ in calls] == [
        'final_registered_strict_sequence', 'final_registered_height_window_strict_sequence']
    rebuilt = deepcopy(source)
    node._strict_check_final_near_field_plan(rebuilt, prepared, runtime)
    assert rebuilt == selected and len(calls) == 3
    assert all(deadline == 123. for _,_,deadline in calls)
    assert runtime['contact_height_window_preferred_revalidation']['strict_result_reused'] is False
    assert node._contact_yield_search['deferred_height_families'] == []
'''
new.write_text(s);print(json.dumps(dict(candidate=str(new),baseline_sha256=sha(old),candidate_sha256=sha(new))))
