import importlib.util
from pathlib import Path

import pytest


spec = importlib.util.spec_from_file_location('offline_node_analysis', Path(__file__).with_name('analyze_node_timings_20261007.py'))
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def pair(call, stage, lo, hi, *, source=None, link=None, thread=1):
    common = dict(call_id=call, stage=stage, thread_id=thread, owner_id=10,
                  source_stamp_ns=source, read_call_id=link)
    return [dict(common, event='start', monotonic_ns=lo, thread_cpu_ns=100),
            dict(common, event='end', monotonic_ns=hi, thread_cpu_ns=200,
                 exception_type=None, crosses_window_end=False)]


def metadata(lo=1.001, hi=1.028):
    return dict(read_started_monotonic=lo, read_completed_monotonic=hi,
        existing_read_diagnostics=dict(result='complete', total_sec=hi-lo,
            host_wait=dict(elapsed_sec=.010, thread_cpu_elapsed_sec=.00001,
                           enter={'monotonic_sec': lo}, exit={'monotonic_sec': lo+.010}),
            stages_sec={'regional_stationary_depth': .009, 'depth_filter_0_spatial': .006},
            regional_stationary_evidence=dict(global_history_reset_reason=None,
                regions=[dict(visual={'reason': 'stationary'})])))


def test_exact_call_source_and_contained_raw_read_are_bound():
    events = pair(1, 'read', 1_000_000_000, 1_030_000_000)
    events += pair(2, 'remember', 1_031_000_000, 1_032_000_000, source=123, link=1)
    events += pair(3, 'publish', 1_010_000_000, 1_015_000_000, source=123, thread=2)
    report = m.analyze(events, [metadata()])
    assert report['bindings']['exact_contained_rows'] == 1
    row = report['reads'][0]
    assert row['source_stamp_ns'] == 123
    assert row['read_observer_tail_ms'] == 2
    assert row['processing_publish_overlap_ms'] == 4
    assert report['bindings']['unmatched_publication_sources']['publish'] == []


def test_nearby_but_not_contained_row_is_never_paired():
    report = m.analyze(pair(1, 'read', 1_000_000_000, 1_030_000_000), [metadata(1.001, 1.030001)])
    assert report['reads'] == []
    assert report['bindings']['unmatched_metadata_rows'] == [0]


def test_duplicate_or_changed_event_identity_is_rejected():
    events = pair(1, 'read', 1, 20)
    with pytest.raises(ValueError, match='duplicate'):
        m.pair_events(events+[events[0]])
    events[1]['owner_id'] += 1
    with pytest.raises(ValueError, match='identity changed'):
        m.pair_events(events)


def test_union_overlap_does_not_double_count_concurrent_spans():
    assert m.overlap_ms(0, 20_000_000, [dict(start_ns=0, end_ns=15_000_000),
        dict(start_ns=10_000_000, end_ns=30_000_000)]) == 20


def test_remember_from_wrong_thread_cannot_supply_source():
    events = pair(1, 'read', 1, 20)+pair(2, 'remember', 21, 22, source=123, link=1, thread=2)
    with pytest.raises(ValueError, match='same owner/thread'):
        m.analyze(events, [])


def test_two_metadata_reads_in_one_call_are_rejected():
    with pytest.raises(ValueError, match='multiple SDK reads'):
        m.analyze(pair(1, 'read', 1_000_000_000, 1_030_000_000), [metadata(), metadata()])


def test_native_timestamp_delta_is_independent_of_global_mapping_and_backend_is_raw():
    def frame(number, global_ms, native, backend):
        return dict(frame_number=number, device_timestamp_ms=global_ms,
            timestamp_domain='global_time', stream_fps=30,
            metadata_raw={'frame_timestamp': native, 'backend_timestamp': backend})
    before = {'raw_frames': {'color': frame(1, 1000., 3000000, 1000)}}
    after = {'raw_frames': {'color': frame(2, 1066.688, 3066688, 1067)}}
    delta = m.source_delta(before, after)['color']
    assert delta['native_frame_delta_ms'] == 66.688
    assert delta['global_minus_native_delta_ms'] == pytest.approx(0.)
    assert delta['metadata_raw_deltas']['backend_timestamp'] == 67


def test_native_clock_reversal_does_not_guess_wrap_or_convert_to_positive_interval():
    before = dict(raw_frames={'color': dict(frame_number=1, device_timestamp_ms=1000.,
        metadata_raw={'frame_timestamp': 2**32-100})})
    after = dict(raw_frames={'color': dict(frame_number=2, device_timestamp_ms=1033.,
        metadata_raw={'frame_timestamp': 33000})})
    delta = m.source_delta(before, after)['color']
    assert delta['native_delta_positive_no_wrap_adjustment'] is False
    assert 'native_frame_delta_ms' not in delta
    assert delta['metadata_raw_deltas']['frame_timestamp'] < 0
