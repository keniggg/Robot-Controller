import importlib.util
import json
from pathlib import Path

import pytest

s = importlib.util.spec_from_file_location('aba_analysis', Path(__file__).with_name('analyze_camera_pipeline_aba_20261007.py'))
m = importlib.util.module_from_spec(s)
s.loader.exec_module(m)


def row(phase, counter, stamp, host, accepted=True, processing=.02):
    raw = {stream:dict(present=True, frame_number=counter, device_timestamp_ms=stamp,
        timestamp_domain='global_time', stream_fps=30., profile_sha256='frozen',
        diagnostic_extra_metadata=dict(values={'actual_exposure':8500},
            unavailable={'gain_level':'unsupported','auto_exposure':'unsupported'}),
        metadata_raw={'actual_fps':30000}) for stream in ('color','depth')}
    return dict(kind='read',phase=phase,requested_phase=phase,elapsed_start_sec=host-.03,
        elapsed_end_sec=host,read_thread_cpu_sec=processing,
        diagnostic=dict(result='raw_complete' if phase=='B' else 'complete',raw_frames=raw,
            pair_timing=dict(accepted=accepted, reason='within_nominal_pair_tolerance' if accepted else 'timestamp_skew_exceeded'),
            total_sec=processing+.01,host_wait=dict(elapsed_sec=.01,exit=dict(monotonic_sec=host)),
            stages_sec={'wait_for_frames':.01}))


def test_raw_dedup_and_accepted_pair_are_distinct():
    rows=[row('B',1,1000.,30.1), row('B',1,1000.,30.11,False), row('B',2,1033.,30.133)]
    result=m.analyze(rows, {'acquisition':{}})['phases']['B']
    assert result['reads']==3 and result['pair_accepted']==2
    assert result['raw_streams']['color']['exact_repeats_removed']==1
    assert result['raw_streams']['color']['counts']['unique_sources']==2
    assert result['accepted_pair_streams']['color']['selected_returned_packets']==2
    assert result['metadata']['color']['unique_source_count']==2
    assert result['metadata']['color']['fields']['actual_exposure']['observed']['mean']==8500
    assert result['metadata']['color']['fields']['gain_level']['unavailable']=={'unsupported':2}


def test_plus_one_counter_can_have_invalid_device_interval_and_precursor_is_immediate_packet():
    rows=[row('A1',1,1000.,1.),row('A1',1,1000.,1.01,False,processing=.001),row('A1',2,1067.,1.067)]
    d=m.stream_analysis(rows,'A1','color')
    assert d['counts']['counter_delta_1']==1
    assert d['counts']['device_interval_outside_original_source_gate']==1
    a=d['anomalies'][0]
    assert a['preceding_read_processing_ms']==pytest.approx(1.)
    assert a['previous_unique_source_read']['elapsed_end_sec']==1.
    assert a['immediately_preceding_packet']['elapsed_end_sec']==1.01


def test_cross_stage_and_boundary_are_not_in_stage_gap_counts():
    rows=[row('A1',1,1000.,29.9),row('boundary',2,1033.,30.01),row('B',20,1500.,30.2),row('B',21,1533.,30.233)]
    d=m.stream_analysis(rows,'B','depth')
    assert d['cross_boundary_first_source_intervals_excluded']==1
    assert d['internal_interval_count']==1 and not d['anomalies']


def test_same_counter_changed_stamp_is_not_an_exact_duplicate():
    rows=[row('A1',1,1000.,1.),row('A1',1,1033.,1.033)]
    d=m.stream_analysis(rows,'A1','color')
    assert d['exact_repeats_removed']==0
    assert d['counts']['counter_not_plus_one']==1


def test_sealed_input_rejects_active_or_modified_records(tmp_path):
    path=tmp_path/'metadata.jsonl';path.write_text(json.dumps(row('A1',1,1000.,1.))+'\n')
    summary=dict(complete=False,pipeline_closed=True,writer=dict(drained=True,worker_alive=False,
        sha256=m.sha(path),rows_written=1))
    (tmp_path/'summary.json').write_text(json.dumps(summary))
    with pytest.raises(ValueError,match='sealed_complete'):
        m.sealed_input(tmp_path)
    summary['complete']=True;(tmp_path/'summary.json').write_text(json.dumps(summary))
    assert len(m.sealed_input(tmp_path)[0])==1
    path.write_text(path.read_text()+'{}\n')
    with pytest.raises(ValueError,match='hash_mismatch'):
        m.sealed_input(tmp_path)
