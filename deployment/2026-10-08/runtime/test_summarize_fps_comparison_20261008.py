import importlib.util
import json
from pathlib import Path

path=Path(__file__).with_name('summarize_fps_comparison_20261008.py')
spec=importlib.util.spec_from_file_location('fps_summary',path)
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)


def entry(t,*,owner=1,qualified=True,reset=None,total=.09,wait=.07):
    stream=dict(frame_number=int(t*10),device_timestamp_ms=t*1000,timestamp_domain='global_time',
                metadata_raw=dict(frame_timestamp=int(t*1e6)))
    return dict(source_stamp_ns=1791426000000000000+int(t*1e9),read_completed_monotonic=t,
        geometry_generation=owner,geometry_activation_stamp_ns='123',evidence=dict(result='complete',
            total_sec=total,stages_sec=dict(wait_for_frames=wait,regional_stationary_depth=.012),
            raw_frames=dict(color=stream,depth=stream.copy()),
            regional_stationary_evidence=dict(global_history_reset_reason=reset,regions=[dict(name='center',
                qualified=qualified,filter_applied=qualified,visual=dict(reason='stationary' if qualified else 'warming_up'))])))


def test_window_excludes_outside_and_wait_processing_are_separate():
    rows=[entry(9.9,reset='sdk_source_discontinuity'),entry(10),entry(11),entry(12)]
    report=a.successful_read_summary(rows,10,12,[])
    assert report['successful_reads_completed_inside_capture']==2
    assert report['excluded_outside']==2 and report['global_reset_count']==0
    assert abs(report['processing_excluding_wait_stage_seconds']['max']-.02)<1e-12
    assert report['successful_source_interval_seconds']['max']==1
    assert report['metadata_coverage']['tail_after_last_read_sec']==1


def test_missing_center_and_reset_not_relabelled_as_false_or_no_reset():
    rows=[entry(10,qualified=False,reset='sdk_source_discontinuity'),entry(11)]
    rows[1]['evidence']['regional_stationary_evidence']={}
    report=a.successful_read_summary(rows,10,12,[])
    assert report['global_reset_count']==1 and report['global_reset_field_unavailable']==1
    assert report['center']['known']==1 and report['center']['unavailable']==1
    assert report['center']['qualified_fraction_among_known']==0


def test_impossible_negative_processing_is_unavailable_not_clamped():
    report=a.successful_read_summary([entry(10,total=.01,wait=.02)],10,11,[])
    assert report['processing_unavailable']==1
    assert report['processing_excluding_wait_stage_seconds']['count']==0


def test_source_intervals_do_not_cross_owner_change():
    report=a.successful_read_summary([entry(10),entry(11,owner=2)],10,12,[])
    assert report['successful_source_interval_seconds']['count']==0
    assert report['raw_stream_deltas_between_visible_successes']=={'color':{},'depth':{}}


def test_stream_raw_timestamp_units_retained_and_different_domain_not_joined():
    rows=[entry(10),entry(11)]
    rows[1]['evidence']['raw_frames']['depth']['timestamp_domain']='hardware_clock'
    report=a.successful_read_summary(rows,10,12,[])
    assert report['raw_stream_deltas_between_visible_successes']['color']['frame_timestamp']['max']==1_000_000
    assert report['raw_stream_deltas_between_visible_successes']['depth']=={}


def test_rpc_copy_errors_and_zero_overlap_keep_coverage_limit_visible():
    coverage=[dict(valid=True,producer_copy_errors=2,previous_same_owner_overlap_count=0,
                   no_successful_history_gap_proven=False),dict(valid=False,error='timeout')]
    row=a.successful_read_summary([entry(10)],10,12,coverage)['metadata_coverage']
    assert row['copy_error_or_missing_count']==1 and row['explicit_zero_overlap_count']==1
    assert row['invalid_rpc_rows']==1 and not row['entire_capture_coverage_proven']


def test_missing_legs_are_pending_not_silent_success(tmp_path):
    report=a.summarize(tmp_path)
    assert report['pending_legs']==['baseline30','trial15','restore30']
    assert not report['all_three_legs_complete']
    assert report['config_comparability']=='pending'


def test_existing_bad_leg_is_invalid_not_pending_or_omitted(tmp_path,monkeypatch):
    for leg,passive in a.LEGS:
        for name in [passive+'/outcome.json',passive+'/SHA256SUMS',leg+'_rgbd_analysis/report.json',
                     leg+'_rgbd_analysis/SHA256SUMS',leg+'_rgbd/summary.json']:
            p=tmp_path/name;p.parent.mkdir(exist_ok=True);p.write_text('{}')
    def invalid(_): raise ValueError('checksum mismatch')
    monkeypatch.setattr(a,'passive_summary',invalid)
    report=a.summarize(tmp_path)
    assert not report['pending_legs'] and len(report['invalid_legs'])==3
    assert not report['all_three_legs_complete']


def test_quality_report_hash_conflict_rejected(tmp_path):
    folder=tmp_path/'report';folder.mkdir()
    (folder/'report.json').write_text('{}')
    (folder/'SHA256SUMS').write_text('0'*64+'  report.json\n')
    import pytest
    with pytest.raises(ValueError,match='quality_report_SHA_mismatch'):
        a.quality_summary(folder,tmp_path/'capture')
