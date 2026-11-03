from pathlib import Path
import hashlib, json, shutil
W=Path(__file__).resolve().parent.parent
C=W/'.runtime/gen16_contact_budget_checkpoint_20261013'
assert not C.exists()
files=['src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_remote.py',
       'src/alicia_grasp_modes/src/alicia_grasp_modes/contact_search_yield.py']
files += ['src/alicia_grasp_modes/tests/'+p for p in (
 'test_contact_search_yield_remote.py','test_contact_search_yield_reached_gain.py',
 'test_gen17_wire_selection_service_ingress.py','test_gen7_contact_yield_completed_budget.py')]
for rel in files:
 d=C/rel;d.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(W/rel,d)
(C/'baseline_sha256.json').write_text(json.dumps({f:hashlib.sha256((W/f).read_bytes()).hexdigest() for f in files},indent=2)+'\n')
p=C/files[0];s=p.read_text()
start=s.index('            # Only independently bound completed precheck failures consume')
end=s.index('\n        return checked, continuing',start)
s=s[:start]+'''            # Two completed families are a progress milestone, never an early
            # stop. Keep trying the same-station height alternatives and tail
            # until the fixed child scheduling checkpoint. A wall-only expiry
            # cannot fabricate the original ROS-time proof below.
            timed = now >= state['budget_deadline_ns'] or time.monotonic() >= state['wall_deadline']
            if (timed and now >= state['budget_deadline_ns']
                    and len(state['original_families']) >= 2 and state['collision_receipts']
                    and self._contact_yield_measured_opportunity(prepared)):
                state.update(stopped=True, exhausted_by='budget_checkpoint', yielded_ns=now)
            # No valid certificate means the existing original-phase fallback,
            # not a fake claim that the remaining candidates were rejected.
            return not timed''' + s[end:]
start=s.index("        state = getattr(self, '_contact_yield_search', None)", s.index('    def _contact_height_window_retry('))
end=s.index('        if preferred_recipe is None',start)
s=s[:start]+'''        # Height alternatives keep their ordinary strict revalidation and the
        # caller's unchanged deadline. A two-family heuristic cannot defer them.
''' +s[end:]
s=s.replace("proof.update(contact_search_code=CONTACT_VIEW_YIELD, selected=False, terminated_early=True,", "proof.update(proof_version=2, contact_search_code=CONTACT_VIEW_YIELD, selected=False, terminated_early=True,")
s=s.replace("'original_families','collision_receipts','incumbent_found')}","'original_families','attempted_families','collision_receipts','incumbent_found')}")
s=s.replace("proof['yield_evidence'].update(policy='bounded_contact_search_for_measured_view',\n                phase_id=prepared.near_field_phase_id, maximum_duration_sec=30.,maximum_original_families=2)", "proof['yield_evidence'].update(policy='contact_search_budget_checkpoint_v2',\n                phase_id=prepared.near_field_phase_id, maximum_duration_sec=30.,\n                minimum_completed_families=2, completed_family_count=len(state['original_families']),\n                attempted_family_count=len(state['attempted_families']),\n                unchecked_count=search['shortlist_count']-search['checked_count'])")
s=s.replace('and its deferred height alternatives under the remaining ORIGINAL', 'and its ordinary height alternatives under the remaining ORIGINAL')
p.write_text(s)
p=C/files[1];original=p.read_text();old=original[original.index('def contact_search_yield_proof_valid(proof):'):]
legacy=old.replace('def contact_search_yield_proof_valid(proof):','def _legacy_contact_search_yield_proof_valid(proof):',1)
v2=old.replace('def contact_search_yield_proof_valid(proof):','def _budget_checkpoint_proof_valid(proof):',1)
v2=v2.replace("evidence['policy'] != 'bounded_contact_search_for_measured_view'", "evidence['policy'] != 'contact_search_budget_checkpoint_v2'")
v2=v2.replace("or type(evidence['maximum_original_families']) is not int\n                or evidence['maximum_original_families'] != 2", "or 'maximum_original_families' in evidence\n                or type(evidence['minimum_completed_families']) is not int\n                or evidence['minimum_completed_families'] != 2")
v2=v2.replace("evidence['exhausted_by'] not in ('original_family_limit', 'shared_time_budget')", "evidence['exhausted_by'] != 'budget_checkpoint'")
v2=v2.replace('len(families) != 2','len(families) < 2')
v2=v2.replace("for family in families}) != 2", "for family in families}) != len(families)")
v2=v2.replace("or (evidence['exhausted_by'] == 'shared_time_budget'\n                    and evidence['yielded_ns'] < evidence['budget_deadline_ns'])", "or evidence['yielded_ns'] < evidence['budget_deadline_ns']")
needle="        receipts = evidence['collision_receipts']"
addition='''        attempted = evidence['attempted_families']
        if (any(not _integer(evidence[key]) for key in
                ('completed_family_count', 'attempted_family_count', 'unchecked_count'))
                or evidence['completed_family_count'] != len(families)
                or not isinstance(attempted, (list, tuple))
                or evidence['attempted_family_count'] != len(attempted)
                or not len(families) <= len(attempted) <= proof['checked_count']
                or evidence['unchecked_count'] != proof['shortlist_count'] - proof['checked_count']):
            return False
        if any(not _integer(f['track_id']) or not _integer(f['variant_index'])
               or not _hex(f['family_sha256'], 64) for f in attempted):
            return False
        if (len({f['family_sha256'] for f in attempted}) != len(attempted)
                or len({(f['track_id'], f['variant_index']) for f in attempted}) != len(attempted)
                or any(f not in attempted for f in families)):
            return False
'''
assert needle in v2;v2=v2.replace(needle,addition+needle)
wrapper='''def contact_search_yield_proof_valid(proof):
    """Accept preserved v1 history or explicit v2; unknown versions fail closed."""
    if not isinstance(proof, dict):
        return False
    version = proof.get('proof_version', 1)
    if type(version) is not int:
        return False
    if version == 1:
        return _legacy_contact_search_yield_proof_valid(proof)
    if version == 2:
        return _budget_checkpoint_proof_valid(proof)
    return False


'''
p.write_text(original[:original.index('def contact_search_yield_proof_valid(proof):')]+wrapper+legacy+'\n\n'+v2)
print(C)
