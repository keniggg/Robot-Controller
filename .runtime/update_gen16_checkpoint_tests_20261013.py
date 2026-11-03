from pathlib import Path
import shutil
W=Path(__file__).resolve().parent.parent;C=W/'.runtime/gen16_contact_budget_checkpoint_20261013';T=C/'src/alicia_grasp_modes/tests'
p=C/'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_remote.py';s=p.read_text().replace("minimum_completed_families=2, completed_family_count=len(state['original_families']),", "minimum_completed_families=2,\n                family_evidence_scope='at_least_one_bound_failed_precheck_not_exhaustive',\n                unchecked_count_scope='original_shortlist_candidates', height_search_exhaustiveness_claimed=False,\n                completed_family_count=len(state['original_families']),")
p.write_text(s)
p=C/'src/alicia_grasp_modes/src/alicia_grasp_modes/contact_search_yield.py';s=p.read_text().replace('This proof says only that two original candidate families were checked and a\nsame-request compensated grasp collided.', 'Legacy v1 keeps the exact two-family wire meaning. Version 2 waits for the\n30-second scheduling checkpoint and retains every listed family with a bound\nfailed precheck. Neither version claims all height alternatives were exhausted.\nBoth require a same-request compensated grasp collision.')
s=s.replace("if (evidence['policy'] != 'contact_search_budget_checkpoint_v2'", "if (evidence['policy'] != 'contact_search_budget_checkpoint_v2'\n                or evidence['family_evidence_scope'] != 'at_least_one_bound_failed_precheck_not_exhaustive'\n                or evidence['unchecked_count_scope'] != 'original_shortlist_candidates'\n                or evidence['height_search_exhaustiveness_claimed'] is not False")
p.write_text(s)
p=T/'test_contact_search_yield_remote.py';s=p.read_text()
s=s.replace('def test_two_complete_original_families_yield_to_real_measured_supplement(', 'def test_two_completed_families_at_checkpoint_yield_to_real_measured_supplement(')
s=s.replace("selection,calls,retries=search(node,p,clock,candidates,monkeypatch)\n    assert calls==[1,2] and retries==[]", "selection,calls,retries=search(node,p,clock,candidates,monkeypatch,seconds=[15.,15.])\n    assert calls==[1,2] and len(retries)==2",1)
s=s.replace("    assert proof['checked_count']==2 and proof['shortlist_count']==4", "    assert proof['proof_version']==2\n    assert proof['yield_evidence']['exhausted_by']=='budget_checkpoint'\n    assert proof['checked_count']==2 and proof['shortlist_count']==4",1)
s=s.replace('def test_incomplete_second_family_or_unbound_receipt_uses_original_remaining_phase(', 'def test_incomplete_second_family_or_unbound_receipt_exhausts_only_finite_shortlist(')
s=s.replace("    assert node._contact_yield_search['fallback']\n    assert calls[:4]==[1,2,3,4]  # Distinct unchecked families precede deferred retries.", "    assert not node._contact_yield_search['fallback']\n    assert calls==[1,2,3,4]  # No two-family stop or redundant completed-shortlist fallback.")
s=s.replace("    assert node._direct_near_field_candidate_deadline_sec(p,{})==p.near_field_phase_deadline_sec", "    assert node._direct_near_field_candidate_deadline_sec(p,{})<=p.near_field_phase_deadline_sec")
s=s.replace("    assert node._contact_yield_search['fallback'] and retries", "    assert not node._contact_yield_search['fallback'] and len(retries)==4")
s=s.replace("    search(node,p,clock,candidates,monkeypatch,results=['timeout']*8)\n    state=", "    search(node,p,clock,candidates,monkeypatch,results=['timeout']*8,seconds=[31.])\n    state=")
p.write_text(s)
p=T/'test_contact_search_yield_reached_gain.py';s=p.read_text().replace('selection,calls,retries=search(node,p,clock,candidates,monkeypatch)','selection,calls,retries=search(node,p,clock,candidates,monkeypatch,seconds=[15.,15.])').replace('assert calls==[1,2] and retries==[]','assert calls==[1,2] and len(retries)==2');p.write_text(s)
p=T/'test_gen17_wire_selection_service_ingress.py';s=p.read_text().replace('clock[0] += 2_000_000_000','clock[0] += 15_000_000_000');p.write_text(s)
p=T/'test_gen7_contact_yield_completed_budget.py';s=p.read_text()
start=s.index('    assert calls == [1, 2, 3] and retries == []',s.index('def test_recorded_margin'))
end=s.index('\n\n\n@pytest.mark.parametrize',start)
s=s[:start]+'''    assert calls == [1, 2, 3, 4] and len(retries) == 4
    assert len(state['attempted_families']) == 4 and len(state['original_families']) == 3
    assert [f['track_id'] for f in state['original_families']] == [2, 3, 4]
    assert not state['fallback'] and selection.termination_reason != CODE
    assert not selection.terminated_early and len(selection.checked) == 4
    assert node._observe_return_completed_contact_failure(p, CODE) is None
    assert state['budget_deadline_ns'] - state['started_ns'] == 30_000_000_000
    assert p.near_field_phase_deadline_sec == phase_deadline
''' +s[end:]
s=s.replace("kinds = early + ['collision', 'collision']", "kinds = early + ['collision', 'collision', 'timeout']")
s=s.replace("assert calls == list(range(1, len(kinds)+1)) and retries == []", "assert calls == list(range(1, len(kinds)+1)) and len(retries) == len(kinds)")
s=s.replace("    assert selection.termination_reason == CODE\n    assert contact_search_yield_proof_valid(node._observe_return_completed_contact_failure(p, CODE))\n\n\ndef test_duplicate", "    assert selection.termination_reason != CODE and not selection.terminated_early\n    assert node._observe_return_completed_contact_failure(p, CODE) is None\n\n\ndef test_duplicate")
s=s.replace("selection, calls, retries = search(node, p, clock, candidates, monkeypatch)\n    assert calls == [1, 2, 3] and retries == []", "selection, calls, retries = search(node, p, clock, candidates, monkeypatch, seconds=[10.,10.,10.])\n    assert calls == [1, 2, 3] and len(retries) == 3",1)
s=s.replace('assert calls == list(range(1, 8))*2','assert calls == list(range(1, 8))')
s=s.replace("assert node._contact_yield_search['fallback'] and len(retries) == 7", "assert not node._contact_yield_search['fallback'] and len(retries) == 7")
s=s.replace('assert calls == [1, 2, 3] and retries == []','assert calls == [1, 2, 3] and len(retries) == 3')
s=s.replace("assert len(state['original_families']) == 2 and not state['collision_receipts']\n    assert state['fallback'] and retries", "assert len(state['original_families']) == 4 and not state['collision_receipts']\n    assert not state['fallback'] and len(retries) == 4")
p.write_text(s)
fixture=T/'fixtures/gen7_contact_yield_completed_budget.json';fixture.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(W/'src/alicia_grasp_modes/tests/fixtures'/fixture.name,fixture)
print('updated intent tests; copied original fixture unchanged')
