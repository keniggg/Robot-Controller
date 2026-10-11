"""Recorded GEN17 wire selection at the real bounded-search handoff.

The full failed receipts are recorded, but the geometry, request plan ids and
feedback clocks and selection value in this callback fixture are synthetic.
The actual string/integer selection representation is retained. Rebinding those
fields does not claim a replay of GEN17 trajectories.
"""
from copy import deepcopy
import json
from pathlib import Path

import pytest

from alicia_grasp_modes.contact_search_yield import CODE
from alicia_grasp_modes.selection import parse_selection
from test_remote_modes import configured
import test_contact_search_yield_remote as search_module
from test_contact_search_yield_reached_gain import reached_gain_context
from test_contact_search_yield_accept import (
    test_actual_failure_audit_commit_precedes_mode_terminal_handoff as audit_handoff)


def wire_context(monkeypatch, fault=None, local_wire=False):
    path=Path(__file__).parent/'fixtures/gen17_contact_yield_wire_receipts.json'
    recorded=json.loads(path.read_text())
    current=deepcopy(recorded['current_selection'])
    receipts=[row['recorded_diagnostic']['precheck']
              for row in recorded['recorded_receipts']]
    assert len(receipts)==2
    assert type(current['stamp_ns']) is int
    assert all(type(r['selection']['stamp_ns']) is str for r in receipts)
    assert all(r['selection']!=current for r in receipts)
    assert all(parse_selection(r['selection'])==parse_selection(current) for r in receipts)

    def receipt(node,prepared,clock,plan_id):
        # Preserve the full recorded rejection and actual gateway decimal-text
        # representation, bound to this synthetic registered request's owner.
        value=deepcopy(receipts[(int(plan_id,16)-1)%2])
        value['plan_id']=plan_id
        value['source_sample']['sdk_stamp_ns']=clock[0]-100_000_000
        value['source_sample']['accepted_stamp_ns']=clock[0]-90_000_000
        value['selection']=deepcopy(node._mode_selection)
        value['selection']['stamp_ns']=str(value['selection']['stamp_ns'])
        selection=value['selection']
        if fault=='generation':selection['generation']+=1
        elif fault=='stamp':selection['stamp_ns']=str(int(selection['stamp_ns'])+1)
        elif fault=='float_stamp':selection['stamp_ns']=float(selection['stamp_ns'])
        elif fault=='bool_stamp':selection['stamp_ns']=True
        elif fault=='invalid_stamp':selection['stamp_ns']='1791722318310076236.0'
        elif fault=='missing_stamp':selection.pop('stamp_ns')
        elif fault=='mode':selection.update(mode='carton',strategy='direct')
        elif fault=='strategy':selection['strategy']='two_stage'
        elif fault=='local_invalid':node._mode_selection['stamp_ns']=True
        if local_wire:
            selection['stamp_ns']=int(node._mode_selection['stamp_ns'])
            node._mode_selection['stamp_ns']=str(node._mode_selection['stamp_ns'])
        return value
    monkeypatch.setattr(search_module,'receipt',receipt)
    return recorded


@pytest.mark.parametrize('local_wire',[False,True])
def test_recorded_gateway_selection_normalizes_both_directions(configured,monkeypatch,local_wire):
    wire_context(monkeypatch,local_wire=local_wire)
    node,p,clock,_,candidates=reached_gain_context(configured,monkeypatch)
    selection,calls,retries=search_module.search(node,p,clock,candidates,monkeypatch)
    state=node._contact_yield_search
    assert calls==[1,2] and retries==[]
    assert len(state['original_families'])==len(state['collision_receipts'])==2
    assert not state['fallback']
    assert selection.termination_reason==CODE


@pytest.mark.parametrize('fault',['generation','stamp','float_stamp','bool_stamp',
    'invalid_stamp','missing_stamp','mode','strategy','local_invalid'])
def test_nonmatching_or_invalid_wire_selection_never_yields(configured,monkeypatch,fault):
    wire_context(monkeypatch,fault=fault)
    node,p,clock,_,candidates=reached_gain_context(configured,monkeypatch)
    selection,calls,retries=search_module.search(node,p,clock,candidates,monkeypatch)
    state=node._contact_yield_search
    assert state['fallback'] and retries
    assert not state['original_families'] and not state['collision_receipts']
    assert selection.termination_reason!=CODE
    assert not node._observe_return_prepare_supplement(p,CODE)


@pytest.mark.parametrize('expire_after_commit',[False,True])
def test_recorded_wire_runs_gain_search_atomic_audit_and_terminal(
        configured,monkeypatch,tmp_path,expire_after_commit):
    wire_context(monkeypatch)
    audit_handoff(configured,monkeypatch,tmp_path,expire_after_commit)
