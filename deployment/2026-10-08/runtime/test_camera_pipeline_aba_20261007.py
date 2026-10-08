"""Offline Fake-only contracts. No context/pipeline/device or ROS node is started."""
import copy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import threading

import pytest

PATH = Path(__file__).with_name('camera_pipeline_aba_20261007.py')
SPEC = importlib.util.spec_from_file_location('test_aba_inner', PATH)
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)


def contract():
    return dict(schema=1, kind='camera_pipeline_aba_inner', camera_diagnostic_only=True,
        motion_authorized=False, camera_config=dict(width=768, height=576,
            stream_width=640, stream_height=480, fps=30, simulate=False,
            fallback_to_simulation=False, align_depth_to_color=True,
            pair_rectification=dict(enabled=True), mode_depth_preset=dict(enabled=True,
                stationary_temporal_enabled=True, regional_stationary_enabled=True,
                unknown_preset='high_accuracy', baseline_path='/frozen/baseline.json')),
        perception_config={}, selection=dict(mode='unknown', generation=20), epoch_ns=10,
        expected_feedback=dict(epoch=10, sdk=dict(q=[0.]*6, source_stamp_ns=11),
            accepted=dict(q=[0.]*7, source_stamp_ns=11), task=dict(active=False)),
        source_sha256={})


class Clock:
    def __init__(self):
        self.t = 0.
    def __call__(self):
        return self.t


class Writer:
    def __init__(self):
        self.rows = []
    def check(self):
        pass
    def put(self, row):
        self.rows.append(copy.deepcopy(row))


class Evidence:
    def __init__(self, fail_after=None):
        self.calls, self.fail_after = 0, fail_after
    def check(self):
        self.calls += 1
        if self.fail_after is not None and self.calls >= self.fail_after:
            raise ValueError('accepted_feedback_stale')


class Pipeline:
    def __init__(self, owner):
        self.owner = owner
        self.wait_calls = 0
        self.stops = 0
    def wait_for_frames(self):
        self.wait_calls += 1
        self.owner.clock.t += self.owner.step
        return object()
    def stop(self):
        self.stops += 1


class Manager:
    def __init__(self, clock, step=10.):
        self.clock, self.step = clock, step
        self.pipeline = Pipeline(self)
        self.align = object()
        self.mode_depth_preset = SimpleNamespace(close=lambda:None)
        self.stationary_depth_provider = lambda:False
        self.reads = 0
        self.resets = []
        self.last_read_diagnostics = None
        self._previous_completed_raw_frames = {'old': True}
        self._stationary_pair_last_decision = {'old': True}
        self._diagnostic_clock = clock
    def _reset_stationary_history(self, reason):
        self.resets.append(reason)
    def _host_wait_snapshot(self):
        return dict(monotonic_sec=self.clock(), wall_time_sec=1000.+self.clock(), thread_cpu_sec=0.)
    def _diagnostic_elapsed(self, lo, hi):
        return hi-lo
    def _raw_frame_diagnostics(self, frames):
        return {stream:dict(present=True, frame_number=self.pipeline.wait_calls,
                    device_timestamp_ms=self.clock()*1000., timestamp_domain='hardware_clock',
                    stream_fps=30., metadata_raw={}, unavailable={},
                    profile_signature=dict(fps=30, stream=stream, width=640, height=480))
                for stream in ('color', 'depth')}
    def _pair_timing(self, raw):
        return dict(accepted=True, reason='within_nominal_pair_tolerance')
    def read(self):
        self.reads += 1
        m.raw_read(self)
        self.last_read_diagnostics.update(result='complete', raw_only=False,
            regional_stationary_evidence=dict(regions=[dict(name='center',
                qualified=self.stationary_depth_provider(), filter_applied=False,
                visual=dict(reason='encoder_or_mode_inactive'))]))
        return 'original-color', 'original-depth'


def test_native_stream_not_rectified_canvas_is_bound():
    c = contract()
    result = m.validate_contract(c)
    assert result == c and result is not c


@pytest.mark.parametrize('mutate', [
    lambda c:c.update(kind='target_alignment'),
    lambda c:c.update(motion_authorized=True),
    lambda c:c.update(camera_diagnostic_only=1),
    lambda c:c['camera_config'].update(simulate=True),
    lambda c:c['camera_config'].update(fallback_to_simulation=True),
    lambda c:c['camera_config'].update(align_depth_to_color=False),
    lambda c:c['camera_config'].update(stream_width=848),
    lambda c:c['camera_config'].update(fps=15),
    lambda c:c['camera_config']['pair_rectification'].update(enabled=False),
    lambda c:c['camera_config']['mode_depth_preset'].update(regional_stationary_enabled=1),
    lambda c:c['camera_config']['mode_depth_preset'].update(unknown_preset='saved_baseline'),
    lambda c:c['selection'].update(mode='carton'),
    lambda c:c.update(epoch_ns=True),
    lambda c:c['expected_feedback']['accepted'].update(q=[0.]*6),
    lambda c:c['expected_feedback']['sdk'].update(q=[float('nan')]*6),
    lambda c:c['expected_feedback']['sdk'].update(source_stamp_ns=9),
    lambda c:c['expected_feedback'].update(task={'active':True}),
])
def test_contract_rejects_scope_geometry_preset_and_feedback_mutation(mutate):
    c = contract()
    mutate(c)
    with pytest.raises(ValueError):
        m.validate_contract(c)


def test_same_pipeline_fixed_phases_and_raw_only_does_not_read_images():
    clock, writer = Clock(), Writer()
    manager = Manager(clock)
    pipeline = manager.pipeline
    result = m.acquisition(manager, Evidence(), writer, clock=clock)
    assert result['phase_counts'] == dict(A1=3, B=3, A2=3)
    assert result['boundary_reads'] == 0
    assert result['acquisition_elapsed_sec'] == 90.
    assert manager.pipeline is pipeline and pipeline.wait_calls == 9
    assert manager.reads == 6 and pipeline.stops == 0
    assert manager.resets == ['diagnostic_stage_transition_'+p for p in m.PHASES]
    assert manager._previous_completed_raw_frames is None
    assert manager._stationary_pair_last_decision is None
    reads = [r for r in writer.rows if r['kind'] == 'read']
    assert [r['diagnostic']['raw_frames']['color']['frame_number'] for r in reads] == list(range(1, 10))
    assert [r['diagnostic']['raw_frames']['color']['device_timestamp_ms'] for r in reads] == list(range(10000, 90001, 10000))
    assert all(r['diagnostic']['raw_only'] is True for r in reads if r['phase'] == 'B')
    assert all(r['diagnostic']['regional']['regions'][0]['qualified'] is False
               for r in reads if r['phase'] != 'B')  # No fake stationary=True gate.
    assert 'profile_signature' not in reads[0]['diagnostic']['raw_frames']['color']
    assert result['profiles']['color']['value']['width'] == 640


def test_boundary_reads_are_not_relabelled_to_next_phase_and_wait_can_overrun():
    clock, writer = Clock(), Writer()
    result = m.acquisition(Manager(clock, step=7.), Evidence(), writer, clock=clock)
    assert result['boundary_reads'] == 3
    assert result['acquisition_elapsed_sec'] == 91.
    rows = [r for r in writer.rows if r.get('phase') == 'boundary']
    assert [r['requested_phase'] for r in rows] == ['A1', 'B', 'A2']
    assert [r['elapsed_end_sec'] for r in rows] == [35., 63., 91.]
    assert result['hard_time_bound_claimed'] is False


def test_stale_original_feedback_aborts_without_another_read():
    clock, writer = Clock(), Writer()
    manager = Manager(clock)
    with pytest.raises(ValueError, match='accepted_feedback_stale'):
        m.acquisition(manager, Evidence(fail_after=2), writer, clock=clock)
    assert manager.reads == 1


def test_cancellation_never_synchronously_closes_inside_read_loop():
    clock, writer = Clock(), Writer()
    manager = Manager(clock)
    with pytest.raises(ValueError, match='cancelled'):
        m.acquisition(manager, Evidence(), writer, clock=clock, cancelled=lambda:clock() >= 20)
    assert manager.pipeline.wait_calls == 2
    assert manager.pipeline.stops == 0  # Owning finally does cleanup after the call unwinds.


def test_read_exception_retains_failed_diagnostic_then_aborts():
    clock, writer = Clock(), Writer()
    manager = Manager(clock)
    def fail():
        clock.t += .1
        manager.last_read_diagnostics = dict(result='error', failed_stage='align',
                                             raw_frames={'color':dict(frame_number=999)})
        raise RuntimeError('original_align_error')
    manager.read = fail
    with pytest.raises(RuntimeError, match='original_align_error'):
        m.acquisition(manager, Evidence(), writer, clock=clock)
    row = writer.rows[-1]
    assert row['diagnostic']['failed_stage'] == 'align'
    assert row['diagnostic']['raw_frames']['color']['frame_number'] == 999


def test_profile_change_refused_not_silently_compacted():
    profiles = {}
    original = dict(raw_frames={'color':dict(profile_signature={'uid':1}, frame_number=8)})
    m.compact_diagnostic(original, profiles)
    assert original['raw_frames']['color']['profile_signature'] == {'uid':1}
    with pytest.raises(ValueError, match='sdk_profile_changed'):
        m.compact_diagnostic(dict(raw_frames={'color':dict(profile_signature={'uid':2})}), profiles)


def test_extra_metadata_is_best_effort_original_raw_values_remain_exact():
    seen = []
    class Frame:
        def supports_frame_metadata(self, field):
            seen.append(('supports', field))
            if field == 'gain_level':
                raise RuntimeError('unavailable')
            return field == 'actual_exposure'
        def get_frame_metadata(self, field):
            seen.append(('get', field))
            return 999999999999
        def get_data(self):
            raise AssertionError('pixels must not be read')
    frames = SimpleNamespace(get_color_frame=lambda:Frame(), get_depth_frame=lambda:Frame())
    raw = dict(color=dict(present=True, frame_number=2**40, metadata_raw={'original':2**40}),
               depth=dict(present=False))
    rs = SimpleNamespace(frame_metadata_value=SimpleNamespace(**{x:x for x in m.EXTRA_METADATA}))
    result = m.metadata_adapter(lambda _:copy.deepcopy(raw), rs)(frames)
    assert result['color']['frame_number'] == 2**40
    assert result['color']['metadata_raw'] == raw['color']['metadata_raw']
    assert result['color']['diagnostic_extra_metadata'] == dict(values={'actual_exposure':999999999999},
        unavailable={'gain_level':'RuntimeError', 'auto_exposure':'unsupported'})
    assert result['depth'] == raw['depth']
    assert {field for _, field in seen} == set(m.EXTRA_METADATA)


def test_writer_detaches_rows_and_records_exact_large_integers(tmp_path):
    w = m.MetadataWriter(tmp_path/'metadata.jsonl')
    raw = dict(source=1791299999999999999, invalid=None, pixels='no actual pixels')
    w.put(raw)
    raw['source'] = 0
    end = w.close()
    assert end['drained'] and end['rows_written'] == 1
    assert json.loads((tmp_path/'metadata.jsonl').read_text())['source'] == 1791299999999999999
    assert m.sha(tmp_path/'metadata.jsonl') == end['sha256']


def test_writer_budget_exhaustion_does_not_hide_dropped_rows(tmp_path):
    w = m.MetadataWriter(tmp_path/'metadata.jsonl', maximum=20)
    w.put({'v':1})
    with pytest.raises(ValueError, match='budget_exhausted'):
        w.put({'too_big':'x'*30})
    end = w.close()
    assert end['rows_reserved'] == 1 and end['bytes_reserved'] <= 20


def test_writer_queue_full_fails_without_unbounded_buffer(tmp_path, monkeypatch):
    started, finish = threading.Event(), threading.Event()
    original = m.MetadataWriter._work
    def blocked(self):
        started.set()
        finish.wait(1.)
        original(self)
    monkeypatch.setattr(m.MetadataWriter, '_work', blocked)
    w = m.MetadataWriter(tmp_path/'metadata.jsonl', queue_size=1)
    assert started.wait(.5)
    try:
        w.put({'first':1})
        with pytest.raises(RuntimeError, match='queue_full'):
            w.put({'second':2})
    finally:
        finish.set()
    assert w.close()['drained']


def test_strict_cleanup_stops_even_when_preset_restore_and_history_fail():
    manager = Manager(Clock())
    pipeline = manager.pipeline
    def fail():
        raise RuntimeError('failure')
    manager.mode_depth_preset.close = fail
    manager._reset_stationary_history = lambda _:fail()
    result = m.close_manager_strict(manager)
    assert result['pipeline_closed'] and pipeline.stops == 1 and manager.pipeline is None
    assert len(result['errors']) == 2


def test_strict_cleanup_does_not_claim_device_released_on_sdk_stop_error():
    manager = Manager(Clock())
    def fail():
        raise RuntimeError('SDK stop failed')
    manager.pipeline.stop = fail
    result = m.close_manager_strict(manager)
    assert result['pipeline_closed'] is False and manager.pipeline is not None
    assert result['errors'] == ['pipeline_stop: SDK stop failed']


def test_sources_require_dependency_set_and_detect_changed_content(tmp_path, monkeypatch):
    source, baseline = tmp_path/'source.py', tmp_path/'baseline.json'
    source.write_text('original')
    baseline.write_text('{}')
    c = contract()
    c['camera_config']['mode_depth_preset']['baseline_path'] = str(baseline)
    c['source_sha256'] = {str(p):m.sha(p) for p in (source, baseline)}
    monkeypatch.setattr(m, 'required_sources', lambda:[source])
    assert m.verify_sources(c) == c['source_sha256']
    source.write_text('changed')
    with pytest.raises(ValueError, match='source_changed'):
        m.verify_sources(c)
    del c['source_sha256'][str(source)]
    with pytest.raises(ValueError, match='unfrozen_dependency'):
        m.verify_sources(c)


def test_runtime_validation_rejects_native_or_calibration_changes():
    c = contract()
    p = {k:{} for k in ('device', 'depth_to_color_extrinsics', 'alignment',
         'geometry_intrinsics', 'depth_filters', 'color_projection_correction',
         'rectification', 'mode_depth_preset')}
    p.update(sdk_version='2.56.5', depth_scale_m_per_unit=.0001,
             color_intrinsics=dict(width=640, height=480),
             native_depth_intrinsics=dict(width=640, height=480))
    c['camera_config']['runtime_profile'] = copy.deepcopy(p)
    manager = SimpleNamespace(runtime_profile=p)
    m.validate_runtime_profile(manager, c)
    p['depth_scale_m_per_unit'] = .001
    with pytest.raises(ValueError, match='active_runtime_profile_mismatch'):
        m.validate_runtime_profile(manager, c)


@pytest.mark.parametrize('version,path', [('2.55.1', str(m.SDK_SO)),
                                        ('2.56.5', '/wrong/pyrealsense2.so')])
def test_wrong_sdk_is_rejected_before_loading_or_starting_any_pipeline(version, path):
    rs = SimpleNamespace(__version__=version, __file__=path)
    with pytest.raises(ValueError, match='wrong_real_sdk_binding'):
        m.validate_loaded_sources(contract(), rs, None)


def feedback_fixture():
    # Real original gate implementations, with only ROS transport replaced.
    ab = m.load_module('aba_original_gate_test', m.AB)
    sw = m.load_module('aba_original_window_test', m.VISION/'stationary_depth.py')
    c = contract()
    now = [1_400_000_000]
    params = {'/camera':c['camera_config'], '/perception':c['perception_config'],
              '/grasp_mode/selection':c['selection']}
    ros = SimpleNamespace(Time=SimpleNamespace(now=lambda:SimpleNamespace(to_nsec=lambda:now[0])),
                          get_time=lambda:now[0]*1e-9,
                          get_param_cached=lambda key,*_:params[key])
    e = object.__new__(m.FeedbackEvidence)
    e.rospy, e.ab, e.contract = ros, ab, c
    e.guard, e.window = ab.EvidenceGuard(), sw.StationaryDepthWindow()
    e.last_context = float('-inf')
    e.guard.observe('epoch', c['epoch_ns'], None)
    e.guard.observe('task', now[0], c['expected_feedback']['task'])
    def message(kind, stamp, q=None):
        arm = ['Joint%d' % n for n in range(1, 7)]
        names = arm+(['right_finger'] if kind == 'accepted' else [])
        return SimpleNamespace(name=names, position=q or [0.]*len(names),
            header=SimpleNamespace(frame_id='sdk_measured' if kind == 'accepted' else 'sdk_transmitted',
                stamp=SimpleNamespace(to_nsec=lambda:stamp)))
    return e, now, message


def test_original_stationarity_needs_actual_history_and_keeps_unchanged_thresholds():
    e, now, msg = feedback_fixture()
    for kind in ('sdk', 'accepted'):
        e.joints(kind, msg(kind, now[0]))
    e.check(freeze=True)
    assert e.stationary() is False  # Fresh endpoints cannot fabricate .35 s history.
    for ns in (1_500_000_000, 1_600_000_000, 1_700_000_000, 1_800_000_000):
        now[0] = ns
        for kind in ('sdk', 'accepted'):
            e.joints(kind, msg(kind, ns))
        e.check()
    assert e.stationary() is True
    now[0] += 600_000_000
    assert e.stationary() is False
    with pytest.raises(ValueError, match='accepted_feedback_stale'):
        e.check()


def test_original_sdk_change_latches_even_if_command_returns_to_zero():
    e, now, msg = feedback_fixture()
    for kind in ('sdk', 'accepted'):
        e.joints(kind, msg(kind, now[0]))
    e.check(freeze=True)
    now[0] += 10_000_000
    e.joints('sdk', msg('sdk', now[0], [.007]+[0.]*5))
    now[0] += 10_000_000
    e.joints('sdk', msg('sdk', now[0]))
    with pytest.raises(ValueError, match='sdk_changed'):
        e.check()


def test_outer_feedback_binding_is_not_replaced_with_new_initial_pose():
    e, now, msg = feedback_fixture()
    e.joints('accepted', msg('accepted', now[0]))
    e.joints('sdk', msg('sdk', now[0], [.007]+[0.]*5))
    with pytest.raises(ValueError, match='sdk_differs_from_outer_contract'):
        e.check(freeze=True)


def test_logs_stay_in_output_and_recursive_budget_includes_ros_files(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', '/wrong/location')
    monkeypatch.setenv('ROS_PYTHON_LOG_CONFIG_FILE', '/wrong/config')
    m.configure_external_logging(tmp_path)
    assert m.os.environ['ROS_LOG_DIR'] == str(tmp_path/'ros')
    assert m.os.environ['ROS_PYTHON_LOG_CONFIG_FILE'] == str(tmp_path/'python_logging.conf')
    assert '131072, 1' in (tmp_path/'python_logging.conf').read_text()
    before = m.directory_bytes(tmp_path)
    (tmp_path/'ros/session').mkdir(parents=True)
    (tmp_path/'ros/session/node.log').write_bytes(b'x'*123)
    assert m.directory_bytes(tmp_path) == before+123
    assert m.METADATA_BYTES+2*131072+2*128*1024+4096 < m.MAX_BYTES


def test_compact_report_retains_exact_global_reset_cause():
    raw = dict(regional_stationary_evidence=dict(global_history_reset_reason='sdk_source_discontinuity',
                                                regions=[]))
    out = m.compact_diagnostic(raw, {})
    assert out['regional']['global_history_reset_reason'] == 'sdk_source_discontinuity'
