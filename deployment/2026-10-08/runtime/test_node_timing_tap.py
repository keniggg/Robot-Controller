"""Pure fakes: no ROS initialization, SDK device, image publication or control."""
import importlib.util
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace as NS
import threading
import time

import pytest

BASE = Path(__file__).parent/'gen4_root_cause'


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


m = module('pure_node_timing_tap', BASE/'node_timing_tap.py')
observer = module('pure_node_timing_observer', BASE/'camera_existing_read_observer.py')


class Clock:
    def __init__(self):
        self.now = 100_000_000_000
        self.cpu = 20_000_000_000
    def wall(self):
        return self.now
    def thread(self):
        return self.cpu
    def step(self, wall=10_000_000, cpu=2_000_000):
        self.now += wall
        self.cpu += cpu


def classes(clock):
    class Manager:
        def read(self):
            clock.step()
            return self.result
    class Publication:
        def __init__(self, callback):
            self._publish = callback
            self.calls = []
        def submit(self, *args, **kwargs):
            self.calls.append((args, kwargs))
            clock.step()
            return self.result
    class Camera:
        def __init__(self):
            self.cam = Manager()
            self.cam.result = (object(), object())
            self._publication_worker = Publication(self._publish_snapshot)
            self._publication_worker.result = object()
            self._runtime_status_timer = NS(_callback=self._publish_temporal_status)
            self.calls = []
        def _timed_stage(self, timings, stage, callback):
            self.calls.append((timings, stage, callback))
            return callback()
        def _remember_successful_read(self, *args, **kwargs):
            self.calls.append((args, kwargs))
            clock.step()
            return self.cam.result
        def _publish_temporal_status(self, *args, **kwargs):
            self.calls.append((args, kwargs))
            clock.step()
            return self.cam.result
        def _publish_snapshot(self, *args, **kwargs):
            self.calls.append((args, kwargs))
            clock.step()
            return self.cam.result
    return Camera, Publication


def fixture_setup(tmp_path, **kwargs):
    clock = Clock()
    camera, publication = classes(clock)
    originals = {name: getattr(camera, name) for name in (
        '_timed_stage', '_remember_successful_read', '_publish_temporal_status', '_publish_snapshot')}
    originals['submit'] = publication.submit
    tap = m.NodeTimingTap(camera, publication, tmp_path/'timings',
        clock_ns=clock.wall, cpu_ns=clock.thread, start_thread=False, **kwargs)
    return tap, camera(), clock, originals


def read(node):
    return node._timed_stage({}, 'read', node.cam.read)


def close(tap):
    tap.finish()
    tap._writer()
    return json.loads((tap.output/'summary.json').read_text())


def events(tap):
    return [json.loads(row) for row in (tap.output/'events.jsonl').read_text().splitlines()]


def test_original_objects_kwargs_source_and_actual_clocks_are_preserved(tmp_path):
    tap, node, clock, original = fixture_setup(tmp_path)
    pair = read(node)
    assert pair is node.cam.result
    stamp = 1791349999999999999
    options = {'camera': node.cam, 'geometry_generation': 7}
    assert node._remember_successful_read(stamp, 100., **options) is pair
    assert node.calls[-1][1] == options
    snapshot = NS(host_read_completed_stamp_ns=stamp, color=object(), depth=object())
    assert node._publication_worker._publish(snapshot) is pair
    assert node.calls[-1][0][0] is snapshot
    supplied = (pair[0], pair[1], 100., stamp)
    assert node._publication_worker.submit(*supplied) is node._publication_worker.result
    assert all(a is b for a, b in zip(node._publication_worker.calls[-1][0], supplied))
    assert node._runtime_status_timer._callback(None) is pair
    summary = close(tap)
    rows = events(tap)
    assert summary['complete'] and len(rows) == 10
    starts = {row['stage']:row for row in rows if row['event'] == 'start'}
    ends = {row['stage']:row for row in rows if row['event'] == 'end'}
    assert starts['read']['monotonic_ns'] == 100_000_000_000
    assert ends['read']['monotonic_ns']-starts['read']['monotonic_ns'] == 10_000_000
    assert ends['read']['thread_cpu_ns']-starts['read']['thread_cpu_ns'] == 2_000_000
    assert starts['remember']['source_stamp_ns'] == stamp
    assert starts['remember']['read_call_id'] == starts['read']['call_id']
    assert starts['publish']['source_stamp_ns'] == starts['submit']['source_stamp_ns'] == stamp
    assert starts['temporal_status']['source_stamp_ns'] is None
    assert not any(key in json.dumps(rows) for key in ('color', 'depth', 'array'))


def test_does_not_time_other_stage_or_pre_first_read_callbacks(tmp_path):
    tap, node, clock, _ = fixture_setup(tmp_path)
    expected = object()
    assert node._timed_stage({}, 'runtime_params', lambda:expected) is expected
    node._runtime_status_timer._callback(None)
    assert tap.start_ns is None and tap.queue.empty()
    assert not close(tap)['complete']  # A zero-frame diagnostic cannot pass.


def test_original_exception_instance_is_not_replaced(tmp_path):
    tap, node, clock, _ = fixture_setup(tmp_path)
    original = RuntimeError('original acquisition failure')
    def fail():
        clock.step()
        raise original
    with pytest.raises(RuntimeError) as caught:
        node._timed_stage({}, 'read', fail)
    assert caught.value is original
    summary = close(tap)
    assert summary['complete']
    assert events(tap)[1]['exception_type'] == 'RuntimeError'


def test_broken_diagnostic_clock_never_prevents_original_call(tmp_path):
    tap, node, _, _ = fixture_setup(tmp_path)
    def broken():
        raise RuntimeError('diagnostic clock broken')
    tap.cpu_ns = broken
    assert read(node) is node.cam.result
    summary = close(tap)
    assert not summary['complete'] and summary['errors']


def test_restore_class_and_captured_timer_and_publisher_callbacks(tmp_path):
    tap, node, _, originals = fixture_setup(tmp_path)
    read(node)
    assert node._publication_worker._publish.__func__ is not originals['_publish_snapshot']
    summary = close(tap)
    assert summary['class_hooks_restored'] and summary['observed_bound_callbacks_restored']
    assert node._runtime_status_timer._callback.__func__ is originals['_publish_temporal_status']
    assert node._publication_worker._publish.__func__ is originals['_publish_snapshot']
    before = summary['counts']['enqueued']
    node._runtime_status_timer._callback(None)
    read(node)
    assert tap.counts['enqueued'] == before


def test_expiration_restores_without_resetting_or_republishing(tmp_path):
    tap, node, clock, originals = fixture_setup(tmp_path)
    read(node)
    clock.step(wall=30_000_000_000)
    old_callback = node._runtime_status_timer._callback
    assert old_callback(None) is node.cam.result
    assert tap.closed and tap.reason == 'window_elapsed'
    assert node._runtime_status_timer._callback.__func__ is originals['_publish_temporal_status']
    assert node._publication_worker._publish.__func__ is originals['_publish_snapshot']
    assert close(tap)['counts']['started'] == 1


def test_callback_stored_after_finish_detaches_itself_without_new_events(tmp_path):
    tap, node, _, originals = fixture_setup(tmp_path)
    read(node)
    old_callback = node._runtime_status_timer._callback
    tap.finish()
    node._runtime_status_timer._callback = old_callback
    assert old_callback(None) is node.cam.result
    assert node._runtime_status_timer._callback.__func__ is originals['_publish_temporal_status']
    assert close(tap)['counts']['started'] == 1


def test_foreign_replacement_is_not_overwritten(tmp_path):
    tap, node, _, _ = fixture_setup(tmp_path)
    read(node)
    other = lambda self, event:None
    type(node)._publish_temporal_status = other
    node._runtime_status_timer._callback = other.__get__(node, type(node))
    summary = close(tap)
    assert type(node)._publish_temporal_status is other
    assert node._runtime_status_timer._callback.__func__ is other
    assert not summary['class_hooks_restored'] and not summary['complete']


def test_changed_read_owner_cannot_inherit_old_source_link(tmp_path):
    tap, node, _, _ = fixture_setup(tmp_path)
    read(node)
    node._remember_successful_read(123, 1., camera=object())
    close(tap)
    assert [r for r in events(tap) if r['stage']=='remember'][0]['read_call_id'] is None


def test_new_read_clears_previous_link_and_fault_does_not_reuse_it(tmp_path):
    tap, node, _, _ = fixture_setup(tmp_path)
    read(node)
    def fail():raise ValueError('bad')
    with pytest.raises(ValueError):node._timed_stage({}, 'read', fail)
    node._remember_successful_read(123, 1., camera=node.cam)
    close(tap)
    assert [r for r in events(tap) if r['stage']=='remember'][0]['read_call_id'] is None


def test_bounded_budget_disables_only_diagnostics(tmp_path):
    tap, node, _, _ = fixture_setup(tmp_path, max_bytes=m.NodeTimingTap.SUMMARY_BYTES+1024)
    assert read(node) is node.cam.result
    assert read(node) is node.cam.result
    summary = close(tap)
    assert not summary['complete'] and summary['counts']['budget_rejected'] == 1
    assert summary['counts']['started'] == 1
    assert sum(p.stat().st_size for p in tap.output.iterdir()) <= summary['max_directory_bytes']


def test_queue_full_never_drops_camera_operation(tmp_path):
    tap, node, _, _ = fixture_setup(tmp_path, queue_size=1)
    assert read(node) is node.cam.result
    assert read(node) is node.cam.result
    summary = close(tap)
    assert summary['counts']['queue_dropped'] and not summary['complete']


def test_existing_output_is_not_overwritten_and_camera_continues(tmp_path):
    tap, node, _, _ = fixture_setup(tmp_path)
    tap.output.mkdir()
    (tap.output/'events.jsonl').write_text('existing original')
    assert read(node) is node.cam.result
    tap._writer()
    assert (tap.output/'events.jsonl').read_text() == 'existing original'
    assert not json.loads((tap.output/'summary.json').read_text())['complete']
    assert read(node) is node.cam.result


def test_shutdown_during_inflight_read_keeps_original_result_and_restores(tmp_path):
    tap, node, clock, originals = fixture_setup(tmp_path)
    entered, released = threading.Event(), threading.Event()
    result = []
    def blocking():
        entered.set()
        assert released.wait(1.)
        clock.step()
        return node.cam.result
    worker = threading.Thread(target=lambda:result.append(node._timed_stage({}, 'read', blocking)))
    worker.start()
    assert entered.wait(.5)
    tap.finish('shutdown')
    assert type(node)._timed_stage is originals['_timed_stage']
    released.set();worker.join(1.)
    assert result[0] is node.cam.result
    summary = close(tap)
    assert summary['complete'] and not summary['unfinished_calls']
    assert type(node)._timed_stage is originals['_timed_stage']


def test_overlapping_callbacks_get_independent_thread_clocks_and_no_false_source(tmp_path):
    tap, node, _, _ = fixture_setup(tmp_path)
    read(node)
    barrier = threading.Barrier(3)
    original_hooks = {name:orig for _,name,orig,_ in tap.hooks}
    # Invoke both captured callbacks concurrently; each call has its own id and
    # thread clock. Their mere overlap must not fabricate a causal/source link.
    def run(callback, value):barrier.wait();callback(value)
    first=threading.Thread(target=run,args=(node._runtime_status_timer._callback,None))
    second=threading.Thread(target=run,args=(node._publication_worker._publish,NS(host_read_completed_stamp_ns=123)))
    first.start();second.start();barrier.wait();first.join();second.join()
    close(tap)
    rows = [r for r in events(tap) if r['event']=='start']
    assert len({r['call_id'] for r in rows}) == 3
    assert len({r['thread_id'] for r in rows}) == 3
    assert next(r for r in rows if r['stage']=='temporal_status')['source_stamp_ns'] is None


def test_worst_scalar_event_remains_within_reserved_budget(tmp_path):
    tap, node, _, _ = fixture_setup(tmp_path)
    read(node)
    node._remember_successful_read(2**63-1, 1., camera=node.cam)
    summary = close(tap)
    assert summary['complete']
    assert max(len(line)+1 for line in (tap.output/'events.jsonl').read_bytes().splitlines()) <= tap.EVENT_BYTES


def test_writer_deadline_restores_callbacks_even_without_another_read(tmp_path):
    tap, node, clock, originals = fixture_setup(tmp_path)
    tap.thread.start()
    read(node)
    clock.step(wall=30_000_000_000)
    tap.thread.join(1.)
    assert not tap.thread.is_alive()
    summary=json.loads((tap.output/'summary.json').read_text())
    assert summary['complete'] and summary['sealed'] and summary['window_complete']
    assert summary['closed_monotonic_ns']-summary['first_read_start_ns'] >= 30_000_000_000
    assert summary['events_sha256'] == hashlib.sha256((tap.output/'events.jsonl').read_bytes()).hexdigest()
    assert node._runtime_status_timer._callback.__func__ is originals['_publish_temporal_status']
    assert node._publication_worker._publish.__func__ is originals['_publish_snapshot']
    assert summary['event_bytes'] == (tap.output/'events.jsonl').stat().st_size
    assert sum(p.stat().st_size for p in tap.output.iterdir()) <= summary['max_directory_bytes']


def test_blocked_original_after_bounded_seal_is_explicitly_incomplete(tmp_path):
    tap, node, _, originals=fixture_setup(tmp_path)
    entered,released=threading.Event(),threading.Event();result=[]
    def blocked():
        entered.set();assert released.wait(2.);return node.cam.result
    worker=threading.Thread(target=lambda:result.append(node._timed_stage({},'read',blocked)))
    worker.start();assert entered.wait(.5)
    tap.finish('shutdown')
    # Inject elapsed tail allowance rather than making this fake wait a second.
    tap.finish_wall=time.monotonic()-2.
    tap._writer()
    summary=json.loads((tap.output/'summary.json').read_text())
    assert not summary['complete'] and not summary['window_complete']
    assert len(summary['unfinished_calls'])==1
    digest=hashlib.sha256((tap.output/'events.jsonl').read_bytes()).hexdigest()
    released.set();worker.join(1.)
    assert result[0] is node.cam.result
    assert hashlib.sha256((tap.output/'events.jsonl').read_bytes()).hexdigest()==digest
    assert type(node)._timed_stage is originals['_timed_stage']


def test_failed_read_preserves_base_exception_not_only_regular_exceptions(tmp_path):
    tap,node,_,_=fixture_setup(tmp_path)
    error=KeyboardInterrupt()
    def failed():raise error
    with pytest.raises(KeyboardInterrupt) as caught:node._timed_stage({},'read',failed)
    assert caught.value is error
    close(tap)
    assert events(tap)[1]['exception_type']=='KeyboardInterrupt'


def test_missing_method_validated_before_any_hook_is_installed(tmp_path):
    clock=Clock();camera,pub=classes(clock);original=camera._timed_stage
    del camera._publish_snapshot
    with pytest.raises(AttributeError):m.NodeTimingTap(camera,pub,tmp_path/'no',start_thread=False)
    assert camera._timed_stage is original


def test_default_observer_entry_remains_exact_main_runpy(monkeypatch):
    calls=[];result=object()
    monkeypatch.setattr(observer.runpy,'run_path',lambda path,run_name:calls.append((path,run_name)) or result)
    assert observer.run_camera_entry('/camera.py') is result
    assert calls==[('/camera.py','__main__')]


@pytest.mark.parametrize('failure',[None,RuntimeError('original main failed'),KeyboardInterrupt()])
def test_opt_in_executes_original_main_once_and_always_shuts_down(monkeypatch,tmp_path,failure):
    calls=[];expected=object()
    def main():
        calls.append('main')
        if failure is not None:raise failure
        return expected
    def namespace(path,run_name):
        calls.append(run_name)
        return {'CameraNode':object(),'main':main}
    class Tap:
        def __init__(self,*a,**k):calls.append('install')
        def shutdown(self):calls.append('shutdown')
    monkeypatch.setattr(observer.runpy,'run_path',namespace)
    monkeypatch.setattr(observer,'resolve_camera_timing_namespace',lambda ns,path:ns)
    monkeypatch.setattr(observer.importlib,'import_module',lambda name:NS(LatestRGBDPublication=object()))
    invoke=lambda:observer.run_camera_entry('/camera.py',NS(NodeTimingTap=Tap),output=tmp_path/'timings')
    if failure is None:assert invoke() is expected
    else:
        with pytest.raises(type(failure)) as caught:invoke()
        assert caught.value is failure
    assert calls==['__camera_node_timing_diagnostic__','install','main','shutdown']


def test_diagnostic_startup_failure_still_runs_original_main(monkeypatch,tmp_path):
    calls=[]
    monkeypatch.setattr(observer.runpy,'run_path',lambda *a,**k:{'CameraNode':object(),'main':lambda:calls.append('main')})
    monkeypatch.setattr(observer,'resolve_camera_timing_namespace',lambda ns,path:ns)
    monkeypatch.setattr(observer.importlib,'import_module',lambda name:NS(LatestRGBDPublication=object()))
    def fail(*a,**k):raise RuntimeError('diagnostic install')
    observer.run_camera_entry('/camera.py',NS(NodeTimingTap=fail),output=tmp_path/'timings')
    assert calls==['main']
    assert json.loads((tmp_path/'node_timing_startup_error.json').read_text())['diagnostic_only']


def test_diagnostic_cleanup_failure_cannot_mask_original_main_error(monkeypatch,tmp_path):
    original=ValueError('original main')
    def main():raise original
    class Tap:
        def __init__(self,*a,**k):pass
        def shutdown(self):raise RuntimeError('diagnostic shutdown')
    monkeypatch.setattr(observer.runpy,'run_path',lambda *a,**k:{'CameraNode':object(),'main':main})
    monkeypatch.setattr(observer,'resolve_camera_timing_namespace',lambda ns,path:ns)
    monkeypatch.setattr(observer.importlib,'import_module',lambda name:NS(LatestRGBDPublication=object()))
    with pytest.raises(ValueError) as caught:
        observer.run_camera_entry('/camera.py',NS(NodeTimingTap=Tap),output=tmp_path/'timings')
    assert caught.value is original


def camera_source_fixture(tmp_path, monkeypatch):
    source=tmp_path/'src/alicia_flexible_grasp_supervisor/scripts/camera_node.py'
    source.parent.mkdir(parents=True)
    source.write_text('''
class CameraNode:
    def _timed_stage(self,*a,**k): pass
    def _remember_successful_read(self,*a,**k): pass
    def _publish_temporal_status(self,*a,**k): pass
    def _publish_snapshot(self,*a,**k): pass
def main():
    return 'original main result'
if __name__ == '__main__':
    main()
''')
    relay=tmp_path/'devel/lib/alicia_flexible_grasp_supervisor/camera_node.py'
    relay.parent.mkdir(parents=True)
    relay.write_text("python_script = %r\n" % str(source)+'''
with open(python_script,'r') as fh:
    context={'__builtins__':__builtins__,'__doc__':None,'__file__':python_script,
             '__name__':__name__,'__package__':None}
    exec(compile(fh.read(),python_script,'exec'),context)
''')
    monkeypatch.setattr(observer,'CAMERA_TIMING_SOURCE',source)
    return source,relay


@pytest.mark.parametrize('entry_kind',['source','relay'])
def test_actual_catkin_relay_context_and_plain_source_run_original_main(tmp_path,monkeypatch,entry_kind):
    source,relay=camera_source_fixture(tmp_path,monkeypatch)
    seen=[]
    class Tap:
        def __init__(self,camera,*a,**k):
            seen.append(camera._timed_stage.__code__.co_filename)
        def shutdown(self):seen.append('shutdown')
    monkeypatch.setattr(observer.importlib,'import_module',lambda name:NS(LatestRGBDPublication=object()))
    result=observer.run_camera_entry(source if entry_kind=='source' else relay,
        NS(NodeTimingTap=Tap),output=tmp_path/'timings')
    assert result=='original main result'
    assert seen==[str(source),'shutdown']
    assert not (tmp_path/'node_timing_startup_error.json').exists()


@pytest.mark.parametrize('mutation',['python_script','context_file','context_name','foreign_function'])
def test_relay_rejects_mixed_or_foreign_context(tmp_path,monkeypatch,mutation):
    source,relay=camera_source_fixture(tmp_path,monkeypatch)
    namespace=observer.runpy.run_path(str(relay),run_name='__camera_node_timing_diagnostic__')
    if mutation=='python_script':namespace['python_script']='/foreign.py'
    elif mutation=='context_file':namespace['context']['__file__']='/foreign.py'
    elif mutation=='context_name':namespace['context']['__name__']='foreign'
    else:namespace['context']['main']=lambda:None
    with pytest.raises(ValueError):observer.resolve_camera_timing_namespace(namespace,relay)
