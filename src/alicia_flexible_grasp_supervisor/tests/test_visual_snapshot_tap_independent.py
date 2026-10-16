"""Independent pure tests of the temporary raw-LK evidence tap; no ROS/SDK."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[3]
P=ROOT/'.runtime/gen4_root_cause/visual_snapshot_tap.py'
spec=importlib.util.spec_from_file_location('snapshot_independent_module',P)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def source(n):
    return dict(frame_number=n,device_timestamp_ms=1000.+n*1000./30.,
                timestamp_domain='hardware_clock',stream_fps=30.)


def rig(tmp_path,reason='visual_motion',**kwargs):
    class Visual:
        def __init__(self):
            self._anchor=self._points=self._previous=self._anchor_stamp=self._anchor_host=None
            self._count=0;self.raise_next=False
        def observe(self,color,s,host):
            if self.raise_next:raise RuntimeError('original failure')
            self._previous=(s['frame_number'],s['device_timestamp_ms'],'hardware_clock',30.,host,tuple(color.shape))
            if self._anchor is None:
                self.seed(color,s,host);return dict(reason='warming_up',qualified=False)
            result=dict(reason=reason,qualified=reason=='stationary',point_count=len(self._points))
            if reason=='visual_motion':self.seed(color,s,host)
            else:self._count+=1
            return result
        def seed(self,color,s,host):
            self._anchor=color[:,:,0].copy();self._points=np.zeros((24,1,2),np.float32)
            self._anchor_stamp=s['device_timestamp_ms'];self._anchor_host=host;self._count=1
    class Regional:
        def __init__(self):
            self._regions=[dict(name='center',bounds=(2,2,30,30),visual=Visual())]
            self._nested_shadow_visual=None;self._nested_shadow_bounds=None
            self.returned=None
        def reset(self):
            v=self._regions[0]['visual'];v._anchor=v._points=None;v._count=0
            self._nested_shadow_visual=None;self._nested_shadow_bounds=None
        def process(self,frame,color,s,host,encoder_active,aligned_to_color=False):
            v=self._regions[0]['visual'].observe(color[2:30,2:30].copy(),s,host)
            self.returned=(frame,dict(original=v,enabled=encoder_active,aligned=aligned_to_color))
            return self.returned
    now=[10.]
    tap=m.VisualSnapshotTap(Regional,Visual,tmp_path/'capture',clock=lambda:now[0],start_thread=False,**kwargs)
    return tap,Regional(),now,Visual,Regional


def process(engine,n,value=10):
    pixels=np.full((32,32,3),value,np.uint8);frame=object();s=source(n)
    result=engine.process(frame,pixels,s,10.+n/30.,True,aligned_to_color=True)
    assert result is engine.returned and result[0] is frame
    assert np.all(pixels==value) and s==source(n)
    return result


def seal(tap):
    tap.finish();tap._writer()
    return json.loads((tap.output/'summary.json').read_text())


def unpack(tap):
    paths=sorted(tap.output.glob('snapshot_*.npz'));result=[]
    for p in paths:
        with np.load(p,allow_pickle=False) as a:
            result.append(({k:a[k].copy() for k in a},json.loads(a['metadata_json'].tobytes())))
    return result


def test_pre_anchor_is_saved_even_when_original_motion_call_reseeds(tmp_path):
    tap,engine,_,_,_=rig(tmp_path)
    try:
        process(engine,1,11);process(engine,2,22)
        summary=seal(tap);rows=unpack(tap)
        assert summary['counts']['written']==1
        arrays,meta=rows[0]
        assert np.all(arrays['anchor_gray']==11) and np.all(arrays['raw_color']==22)
        assert meta['pre_observe']['anchor_device_timestamp_ms']==source(1)['device_timestamp_ms']
        assert meta['source']==source(2)
        assert meta['original_return']['reason']=='visual_motion'
        assert meta['publication_outcome_unknown'] and meta['enclosing_read_outcome_unknown']
    finally:tap.finish()


def test_original_exception_propagates_and_does_not_create_a_success_snapshot(tmp_path):
    tap,engine,_,_,_=rig(tmp_path)
    try:
        process(engine,1)
        engine._regions[0]['visual'].raise_next=True
        with pytest.raises(RuntimeError,match='original failure'):process(engine,2)
        summary=seal(tap)
        assert not unpack(tap) and summary['queued_unwritten']==0
    finally:tap.finish()


def test_capture_fault_does_not_change_original_return_or_pixels(tmp_path,monkeypatch):
    tap,engine,_,_,_=rig(tmp_path)
    try:
        process(engine,1)
        def fail(*args):raise RuntimeError('diagnostic failure')
        monkeypatch.setattr(tap,'_capture',fail)
        result=process(engine,2)
        assert result[1]['original']['reason']=='visual_motion'
        summary=seal(tap)
        assert summary['counts']['errors']>=1 and not unpack(tap)
    finally:tap.finish()


def test_finish_restores_hooks_and_never_overwrites_subsequent_wrapper(tmp_path):
    tap,engine,_,V,R=rig(tmp_path)
    original_r,original_v=tap.original_process,tap.original_observe
    tap.finish();assert R.process is original_r and V.observe is original_v
    other=lambda *args:None;R.process=other
    tap.finish();assert R.process is other and V.observe is original_v
    R.process=original_r


def test_deadline_expires_without_new_snapshot_or_original_gate_change(tmp_path):
    tap,engine,clock,_,_=rig(tmp_path,seconds=1.)
    try:
        process(engine,1)
        clock[0]=11.01
        result=process(engine,2)
        assert result[1]['original']['reason']=='visual_motion'
        summary=seal(tap)
        assert summary['close_reason']=='window_expired' and not unpack(tap)
    finally:tap.finish()


def test_queue_bound_does_not_modify_the_original_calls(tmp_path):
    tap,engine,_,_,_=rig(tmp_path,queue_size=1)
    try:
        process(engine,1)
        for n in range(2,8):assert process(engine,n)[1]['original']['reason']=='visual_motion'
        summary=seal(tap)
        assert len(unpack(tap))==1 and summary['counts']['queue_full']>=1
        assert summary['reserved_bytes']<=summary['maximum_bytes']
    finally:tap.finish()


def test_anchor_full_raw_and_source_follow_actual_reseeds_not_capture_time(tmp_path):
    tap,engine,_,_,_=rig(tmp_path,queue_size=4)
    try:
        for n,value in [(1,11),(2,22),(3,33)]:process(engine,n,value)
        seal(tap);rows=unpack(tap)
        assert len(rows)==2
        for (arrays,meta),prior,current in zip(rows,[1,2],[2,3]):
            assert np.all(arrays['anchor_raw_color']==prior*11)
            assert np.all(arrays['anchor_gray']==prior*11)
            assert np.all(arrays['raw_color']==current*11)
            assert meta['pre_observe']['anchor_source']==source(prior)
            assert meta['pre_observe']['anchor_filter_host_monotonic']==10.+prior/30.
            assert meta['pre_observe']['anchor_bound_verified'] is True
            assert meta['source']==source(current)
    finally:tap.finish()


def test_stationary_snapshots_keep_original_full_anchor_across_new_sources(tmp_path):
    tap,engine,_,_,_=rig(tmp_path,reason='stationary',queue_size=4)
    try:
        for n,value in [(1,11),(2,22),(3,33)]:process(engine,n,value)
        seal(tap);rows=unpack(tap)
        assert len(rows)==2
        for arrays,meta in rows:
            assert np.all(arrays['anchor_raw_color']==11)
            assert meta['pre_observe']['anchor_source']==source(1)
            assert np.all(arrays['anchor_gray']==11)
    finally:tap.finish()


def test_engine_reset_drops_old_anchor_association_before_reacquiring(tmp_path):
    tap,engine,_,_,_=rig(tmp_path,queue_size=4)
    try:
        process(engine,1,11);process(engine,2,22)
        engine.reset();process(engine,3,33);process(engine,4,44)
        seal(tap);rows=unpack(tap)
        assert len(rows)==2
        arrays,meta=rows[-1]
        assert np.all(arrays['anchor_raw_color']==33) and np.all(arrays['anchor_gray']==33)
        assert meta['pre_observe']['anchor_source']==source(3)
    finally:tap.finish()


def test_new_visual_owner_cannot_inherit_an_old_raw_anchor(tmp_path):
    tap,engine,_,V,_=rig(tmp_path,queue_size=4)
    try:
        process(engine,1,11);process(engine,2,22)
        engine._regions[0]['visual']=V()
        process(engine,3,33);process(engine,4,44)
        seal(tap);rows=unpack(tap)
        assert len(rows)==2
        assert rows[-1][1]['pre_observe']['anchor_source']==source(3)
        assert np.all(rows[-1][0]['anchor_raw_color']==33)
    finally:tap.finish()


def test_uncaptured_existing_anchor_is_not_fabricated_from_current_rgb(tmp_path):
    tap,engine,_,_,_=rig(tmp_path,queue_size=4)
    try:
        v=engine._regions[0]['visual']
        v.seed(np.full((28,28,3),7,np.uint8),source(0),10.)
        process(engine,1,11);process(engine,2,22)
        summary=seal(tap);rows=unpack(tap)
        assert len(rows)==1 and summary['counts']['unbound_pre_anchor']==1
        assert rows[0][1]['pre_observe']['anchor_source']==source(1)
        assert np.all(rows[0][0]['anchor_raw_color']==11)
    finally:tap.finish()


def test_all_three_hooks_and_anchor_cache_are_cleared_on_finish(tmp_path):
    tap,engine,_,V,R=rig(tmp_path)
    process(engine,1,11)
    assert tap.anchor_bytes>0 and tap.anchors
    tap.finish()
    assert not tap.anchors and tap.anchor_bytes==0
    assert R.process is tap.original_process and V.observe is tap.original_observe
    assert R.reset is tap.original_reset
    summary=seal(tap)
    assert summary['original_reset_restored'] and summary['retained_anchor_bytes']==0


def test_byte_budget_declines_snapshot_without_modifying_original_results(tmp_path):
    tap,engine,_,_,_=rig(tmp_path,max_bytes=m.SIDECAR_RESERVE+5000)
    try:
        process(engine,1,11)
        assert process(engine,2,22)[1]['original']['reason']=='visual_motion'
        summary=seal(tap)
        assert not unpack(tap) and summary['counts']['byte_budget_rejected']==1
        total=sum(p.stat().st_size for p in tap.output.iterdir())
        assert total<=summary['maximum_bytes']
    finally:tap.finish()


def test_writer_failure_restores_hooks_and_reports_partial_evidence(tmp_path,monkeypatch):
    tap,engine,_,V,R=rig(tmp_path)
    try:
        process(engine,1);process(engine,2)
        def fail(*a,**kw):raise OSError('injected writer failure')
        monkeypatch.setattr(m.np,'savez',fail)
        summary=seal(tap)
        assert summary['errors'] and summary['counts']['errors']>=1
        assert not unpack(tap) and summary['queued_unwritten']==0
        assert V.observe is tap.original_observe and R.process is tap.original_process and R.reset is tap.original_reset
    finally:tap.finish()


def test_concurrent_capture_commit_rechecks_category_limit(tmp_path):
    import threading
    tap,engine,_,_,_=rig(tmp_path,queue_size=4)
    try:
        process(engine,1,11)
        v=engine._regions[0]['visual'];binding=tap.anchors['center'];barrier=threading.Barrier(2)
        class SimultaneousCopy(np.ndarray):
            def copy(self,*args,**kwargs):
                barrier.wait(timeout=2.)
                return np.asarray(self).copy(*args,**kwargs)
        full=np.full((32,32,3),22,np.uint8).view(SimultaneousCopy)
        ctx=dict(engine=engine,color=full,source=source(2),host=10.+2/30.,encoder_active=True,aligned_to_color=True)
        before=dict(region='center',visual=v,anchor=v._anchor,points=v._points,
            previous=v._previous,anchor_stamp=v._anchor_stamp,anchor_host=v._anchor_host,count=v._count,raw_anchor=binding)
        category='center/visual_motion';tap.selected[category]=m.MAX_PER_CATEGORY-1
        errors=[]
        def capture():
            try:tap._capture(ctx,before,np.asarray(full)[2:30,2:30],source(2),10.+2/30.,dict(reason='visual_motion',qualified=False))
            except Exception as exc:errors.append(exc)
        ts=[threading.Thread(target=capture) for _ in range(2)]
        for t in ts:t.start()
        for t in ts:t.join(3.)
        assert not any(t.is_alive() for t in ts) and not errors
        assert tap.selected[category]==m.MAX_PER_CATEGORY
        assert tap.queue.qsize()==1 and tap.counts['copy_commit_rejected']==1
        assert tap.inflight_copies==0 and tap.reserved_bytes==tap.queue_bytes
    finally:tap.finish()
