"""Pure real-OpenCV snapshot equivalence; no ROS, camera or device access."""
import importlib.util
import json
from pathlib import Path
import sys
import threading

import cv2
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/alicia_flexible_grasp_supervisor/src'))
sys.path.insert(0,str(ROOT/'src/alicia_flexible_grasp_supervisor/tests'))
from test_regional_stationary_depth import Factory,texture,run,source
from alicia_flexible_grasp.vision.regional_stationary_depth import RegionalStationaryDepth
from alicia_flexible_grasp.vision.visual_stationary import VisualStationaryWindow

spec=importlib.util.spec_from_file_location('visual_snapshot_unit',ROOT/'.runtime/gen4_root_cause/visual_snapshot_tap.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


@pytest.fixture(autouse=True)
def single_opencv_thread():
    previous=cv2.getNumThreads();cv2.setNumThreads(1)
    yield
    cv2.setNumThreads(previous)


def gray(rgb,bounds):
    x0,y0,x1,y1=bounds;crop=rgb[y0:y1,x0:x1]
    h,w=crop.shape[:2];ratio=min(1.,320./max(h,w))
    small=cv2.resize(crop,(max(1,round(w*ratio)),max(1,round(h*ratio))),interpolation=cv2.INTER_AREA) if ratio<1 else crop
    return cv2.cvtColor(small,cv2.COLOR_BGR2GRAY)


def test_actual_anchors_and_reports_replay_exactly_without_pixel_or_filter_change(tmp_path):
    rgb=texture();moved=cv2.warpAffine(rgb,np.float32([[1,0,.8],[0,1,0]]),(640,480),borderMode=cv2.BORDER_REFLECT)
    frames=[rgb]*12+[moved]
    baseline_factory=Factory();baseline=RegionalStationaryDepth(baseline_factory)
    expected=[run(baseline,frame,i) for i,frame in enumerate(frames)]
    factory=Factory();engine=RegionalStationaryDepth(factory)
    tap=m.VisualSnapshotTap(RegionalStationaryDepth,VisualStationaryWindow,tmp_path/'evidence',
        queue_size=4,start_thread=False,provenance={'actual_sha':'frozen'})
    try:
        actual=[run(engine,frame,i) for i,frame in enumerate(frames)]
        for (a,ar),(b,br) in zip(actual,expected):
            assert np.array_equal(a,b)
            assert {k:v for k,v in ar.items() if k!='nested_center_shadow'}=={
                k:v for k,v in br.items() if k!='nested_center_shadow'}
            assert br['nested_center_shadow']['reason']=='diagnostic_disabled'
        assert [f.calls for f in factory.filters]==[f.calls for f in baseline_factory.filters]
        tap.finish();tap._writer()
        summary=json.loads((tap.output/'summary.json').read_text())
        assert summary['counts']['written']==4 and summary['errors']==[]
        assert summary['retained_anchor_bytes']==0 and summary['original_reset_restored']
        categories=set()
        for path in tap.output.glob('*.npz'):
            with np.load(path,allow_pickle=False) as saved:
                meta=json.loads(saved['metadata_json'].tobytes());categories.add((meta['region'],meta['category']))
                assert np.array_equal(saved['anchor_raw_color'],rgb)
                assert np.array_equal(saved['anchor_gray'],gray(saved['anchor_raw_color'],meta['bounds_xyxy']))
                assert meta['pre_observe']['anchor_source']==source(0)
                v=VisualStationaryWindow()
                v._anchor=saved['anchor_gray'].copy();v._points=saved['seed_points'].copy()
                previous=meta['pre_observe']['previous_source_tuple'];previous[-1]=tuple(previous[-1]);v._previous=tuple(previous)
                v._anchor_stamp=meta['pre_observe']['anchor_device_timestamp_ms']
                v._anchor_host=meta['pre_observe']['anchor_host_monotonic']
                v._count=meta['pre_observe']['continuous_frame_count']
                x0,y0,x1,y1=meta['bounds_xyxy']
                report=v.observe(saved['raw_color'][y0:y1,x0:x1],meta['source'],meta['filter_host_monotonic'])
                assert report==meta['original_return']
                assert meta['source_implementation']=={'actual_sha':'frozen'}
        assert categories=={(r,s) for r in m.REGIONS for s in m.REASONS}
    finally:tap.finish()


def test_process_reset_discards_both_raw_anchors_immediately(tmp_path):
    engine=RegionalStationaryDepth(Factory());tap=m.VisualSnapshotTap(RegionalStationaryDepth,
        VisualStationaryWindow,tmp_path/'reset',start_thread=False)
    try:
        run(engine,texture(),0)
        assert set(tap.anchors)==set(m.REGIONS) and tap.anchor_bytes>0
        engine.reset()
        assert tap.anchors=={} and tap.anchor_bytes==0
    finally:tap.finish()


def test_byte_budget_rejection_does_not_create_partial_snapshot(tmp_path):
    engine=RegionalStationaryDepth(Factory());tap=m.VisualSnapshotTap(RegionalStationaryDepth,
        VisualStationaryWindow,tmp_path/'budget',start_thread=False,max_bytes=m.SIDECAR_RESERVE+100)
    try:
        for i in range(12):out,report=run(engine,texture(),i)
        assert report['fully_active'] and np.all(out==1001)
        tap.finish();tap._writer()
        summary=json.loads((tap.output/'summary.json').read_text())
        assert summary['counts']['byte_budget_rejected']==2 and summary['files']==[]
        assert sum(p.stat().st_size for p in tap.output.iterdir())<=tap.max_bytes
    finally:tap.finish()


def test_watchdog_restores_even_if_no_regional_call_follows_first_read(tmp_path):
    now=[10.];first=[9.]
    tap=m.VisualSnapshotTap(RegionalStationaryDepth,VisualStationaryWindow,tmp_path/'watchdog',
        seconds=1.,clock=lambda:now[0],start_time=lambda:first[0],start_thread=False)
    tap._writer()
    result=json.loads((tap.output/'summary.json').read_text())
    assert result['close_reason']=='window_expired' and result['original_observe_restored']
    assert result['first_read_monotonic']==9. and result['files']==[]


def test_only_explicit_observer_option_imports_pure_helper():
    p=ROOT/'.runtime/gen4_root_cause/camera_existing_read_observer.py'
    spec=importlib.util.spec_from_file_location('visual_tap_observer_unit',p)
    observer=importlib.util.module_from_spec(spec);spec.loader.exec_module(observer)
    loaded=observer.loaded_module_manifest()
    assert len(loaded)==5 and 'diagnostic.visual_snapshot_tap' not in loaded
    helper,entry=observer.prepare_visual_snapshot_helper()
    assert helper.VisualSnapshotTap and entry['path']==str((p.parent/'visual_snapshot_tap.py').resolve())
    assert len(entry['sha256'])==64


def test_five_observers_by_default_six_only_during_window_then_five_again(tmp_path,monkeypatch):
    calls=[];original=VisualStationaryWindow.observe
    def counted(visual,*args):
        calls.append(visual);return original(visual,*args)
    monkeypatch.setattr(VisualStationaryWindow,'observe',counted)
    engine=RegionalStationaryDepth(Factory());rgb=texture();now=[10.]
    _,report=run(engine,rgb,0)
    assert len(calls)==5 and report['nested_center_shadow']['reason']=='diagnostic_disabled'
    tap=m.VisualSnapshotTap(RegionalStationaryDepth,VisualStationaryWindow,tmp_path/'bounded',
        start_thread=False,clock=lambda:now[0],seconds=1.)
    try:
        calls.clear();_,report=run(engine,rgb,1)
        assert len(calls)==6 and engine._nested_shadow_visual is not None
        assert report['nested_center_shadow']['reason']=='warming_up'
        production=[(r['visual'],r['filter']) for r in engine._regions]
        now[0]=11.;calls.clear();_,report=run(engine,rgb,2)
        assert len(calls)==5 and report['nested_center_shadow']['reason']=='diagnostic_disabled'
        assert engine._nested_shadow_visual is None and engine._nested_shadow_bounds is None
        assert production==[(r['visual'],r['filter']) for r in engine._regions]
        assert not tap.anchors and tap.inflight_nested_shadows==0
        tap._writer()
        summary=json.loads((tap.output/'summary.json').read_text())
        assert summary['nested_shadow_default_disabled'] and summary['nested_shadow_retained_owners']==0
    finally:tap.finish()


def test_finish_during_inflight_shadow_cannot_restore_old_shadow_state(tmp_path,monkeypatch):
    engine=RegionalStationaryDepth(Factory());rgb=texture();entered=threading.Event();release=threading.Event()
    original=engine._observe_nested_center_shadow;results=[];failures=[]
    def blocked(*args):
        entered.set();assert release.wait(2.)
        return original(*args)
    monkeypatch.setattr(engine,'_observe_nested_center_shadow',blocked)
    tap=m.VisualSnapshotTap(RegionalStationaryDepth,VisualStationaryWindow,tmp_path/'inflight',start_thread=False)
    def work():
        try:results.append(run(engine,rgb,0))
        except Exception as exc:failures.append(exc)
    thread=threading.Thread(target=work)
    try:
        thread.start();assert entered.wait(2.)
        assert tap.inflight_nested_shadows==1
        production=[(r['visual'],r['filter']) for r in engine._regions]
        tap.finish();release.set();thread.join(2.)
        assert not thread.is_alive() and not failures
        assert tap.inflight_nested_shadows==0 and engine._nested_shadow_visual is None
        assert results[0][1]['nested_center_shadow']['reason']=='diagnostic_disabled'
        assert production==[(r['visual'],r['filter']) for r in engine._regions]
    finally:
        release.set();thread.join(2.);tap.finish()
