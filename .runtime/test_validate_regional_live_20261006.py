import importlib.util
from pathlib import Path
from copy import deepcopy
import json
import pytest

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('live_validation_tested', HERE/'validate_regional_live_20261006.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def test_new_publication_window_cannot_pass_with_old_fast_repeats_or_regression():
    p = dict(fps=20., frames=10, header_duplicates=0, header_regressions=0, unique_source_stamps=10)
    pubs = dict(rgb=deepcopy(p), depth=deepcopy(p))
    rows = {k: [dict(source_stamp_ns=i) for i in range(10)] for k in pubs}
    assert m.validate_publications(pubs, rows)
    for key, value in [('header_duplicates', 1), ('header_regressions', 1), ('unique_source_stamps', 2), ('fps', 15.)]:
        bad = deepcopy(pubs);bad['rgb'][key] = value
        assert not m.validate_publications(bad, rows)
    rows['depth'] = [dict(source_stamp_ns=i+100) for i in range(10)]
    assert not m.validate_publications(pubs, rows)


def test_full_seven_joint_trace_catches_finger_only_change_and_return():
    before = dict(stamp_ns=1, frame_id='sdk_measured', names=['Joint'+str(i) for i in range(6)]+['right_finger'],
        positions=[0.]*7, velocity=[], effort=[])
    during = deepcopy(before);during['stamp_ns']=2;during['positions'][6]=.001
    after = deepcopy(before);after['stamp_ns']=3
    assert m.changed_fields(before, during) == ['positions']
    assert m.changed_fields(during, after) == ['positions']
    summary = m.joint_summary([before,during,after])
    assert summary['per_joint_range'] == [0.]*6+[.001]
    assert m.changed_fields(after, before) == ['nonmonotonic_stamp']


def test_polled_regional_snapshot_never_accepts_global_active_or_unowned_output():
    names={'center','top_left','top_right','bottom_left','bottom_right'}
    p = dict(enabled=True, regional_enabled=True, active=False, whole_frame_stationarity_qualified=False,
        alignment_proven=True, owner_schema='test', alpha=.4, delta_depth_units=100, persistence=0,
        visual_stationarity_required=True, visual_window_sec=.35, visual_max_motion_full_resolution_px=.25,
        filtered_pixel_count=20, regional_output_active=True,
        regions=[dict(name=n,owned_pixels=20,qualified=n=='center',filter_applied=n=='center') for n in sorted(names)])
    assert m.validate_regional_profile(p,100,'test',names)
    for key,value in [('active',True),('alignment_proven',False),('filtered_pixel_count',40),('persistence',1)]:
        bad=deepcopy(p);bad[key]=value
        with pytest.raises(RuntimeError):m.validate_regional_profile(bad,100,'test',names)
    p.update(regions=[],filtered_pixel_count=0,regional_output_active=False)
    assert m.validate_regional_profile(p,100,'test',names) is False


def test_invalid_numeric_feedback_is_recordable_without_invented_finite_value():
    row = dict(positions=[float('nan'), float('inf')], source_stamp_ns=1791254000000000001)
    safe=m.json_safe(row)
    assert safe['positions'] == ['nan','inf']
    assert safe['source_stamp_ns'] == row['source_stamp_ns']
    assert json.loads(json.dumps(safe,allow_nan=False)) == safe


def test_strict_existing_guard_latches_finger_change_without_tolerance_relaxation():
    ab=m.load('live_existing_guard_test','.runtime/camera_preset_ab_observer.py')
    guard=ab.EvidenceGuard()
    guard.observe('epoch',1,None)
    guard.observe('sdk',100,[0.]*6)
    guard.observe('accepted',100,[0.]*7)
    guard.observe('task',100,dict(active=False))
    assert guard.check(100,freeze=True)
    guard.observe('accepted',101,[0.]*6+[.00001])
    guard.observe('accepted',102,[0.]*7)
    with pytest.raises(ValueError,match='accepted_changed'):guard.check(102,freeze=True)
