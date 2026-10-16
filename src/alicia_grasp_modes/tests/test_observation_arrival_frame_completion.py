"""One timely normal frame can finish without renewing historical geometry."""
import numpy as np
import pytest
from test_observation_arrival_station import completed
from test_arrival_fragment_group import completed_fragment_arrival,split_depth
from test_locked_rgb_hold import custom_frame,latest_status,module,locked


def captured_frame(node,clock,joint,reference_ns,*,fragment=False):
    process=node.process
    node.process=lambda event:None
    clock[0]=reference_ns*1e-9+.494414091+.1
    source=custom_frame(node,clock,joint,target_depth=split_depth() if fragment else .35)
    node.process=process
    clock[0]=reference_ns*1e-9+.530324221
    return source


@pytest.mark.parametrize('fragment',[False,True],ids=['native_unique','measured_fragment_group'])
def test_qualified_arrival_timely_frame_finishes_then_next_frame_uses_new_measured_reference(monkeypatch,fragment):
    setup=completed_fragment_arrival if fragment else completed
    node,clock,feed,joint,params,marker=setup(monkeypatch)
    reference=node._locked_rgb_hold_reference;old_ns=reference['source_stamp_ns'];last=node.tracker.last
    source=captured_frame(node,clock,joint,old_ns,fragment=fragment)
    source_ns=source[0].header.stamp.to_nsec();tokens=[]
    name='recover_arrival_fragment_group' if fragment else 'refine_candidate'
    original=getattr(module,name)
    def costly(*args,**kwargs):
        result=original(*args,**kwargs)
        token=node._arrival_frame_completion;assert token is not None
        assert node.tracker.last is last  # Also true for the native raw>=1 path.
        assert token['reference'] is reference and token['marker'] is marker
        assert token['source_stamp_ns']==source_ns
        tokens.append((token,token['qualified_entry_ns'],token['completion_monotonic']))
        clock[0]=old_ns*1e-9+.828624487
        assert node._observation_arrival_retry_reference(source,params['/camera'],expected=reference) is reference
        assert (token['qualified_entry_ns'],token['completion_monotonic'])==tokens[-1][1:]
        return result
    monkeypatch.setattr(module,name,costly)
    node.process(None)
    assert len(tokens)==1 and node._arrival_frame_completion is None
    assert latest_status(node)['state']=='ready',latest_status(node)
    evidence=latest_status(node)['frame_timing']['arrival_frame_completion']
    assert evidence['used_past_geometry_deadline'] is True
    assert evidence['reference_refreshed'] is False
    assert evidence['completion_deadline_ns']==min(source_ns+800_000_000,old_ns+1_600_000_000,marker['session_deadline_ns'])
    assert reference['source_stamp_ns']==old_ns
    assert node.last_good_source_stamp_ns==source_ns
    assert node._locked_rgb_hold_reference is not reference
    assert node._locked_rgb_hold_reference['observation_arrival_retry'] is marker
    monkeypatch.setattr(module,name,original)
    clock[0]+=.2
    following=custom_frame(node,clock,joint,target_depth=split_depth() if fragment else .35)
    assert latest_status(node)['state']=='ready',latest_status(node)
    assert node.last_good_source_stamp_ns==following[0].header.stamp.to_nsec()
    assert node._arrival_frame_completion is None
    np.testing.assert_array_equal(marker['arrival_q'],np.zeros(6))


def test_ordinary_tracking_does_not_receive_arrival_completion(monkeypatch):
    node,clock,feed,joint,params=locked(monkeypatch)
    clock[0]+=.2;custom_frame(node,clock,joint,target_depth=.35)
    assert latest_status(node)['state']=='ready'
    assert node._arrival_frame_completion is None
    assert 'arrival_frame_completion' not in latest_status(node)['frame_timing']
