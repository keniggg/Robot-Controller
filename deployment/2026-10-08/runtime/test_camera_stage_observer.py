import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


path = Path(__file__).with_name('camera_stage_observer.py')
spec = importlib.util.spec_from_file_location('camera_stage_observer', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Clock:
    value = 0.
    def __call__(self):return self.value
    def step(self, amount=.1):self.value += amount


class Profile:
    def fps(self):return 30
    def as_video_stream_profile(self):return self
    def get_intrinsics(self):
        return SimpleNamespace(width=8,height=8,fx=4.,fy=4.,ppx=3.,ppy=3.,
                               model='inverse_brown_conrady',coeffs=[.01,0,0,0,0])
    def get_extrinsics_to(self, other):
        return SimpleNamespace(rotation=[1,0,0,0,1,0,0,0,1],translation=[.001,0,0])


class Frame:
    def __init__(self, data):self.data=data
    def get_data(self):return self.data
    def get_frame_number(self):return 17
    def get_timestamp(self):return 1234.
    def get_frame_timestamp_domain(self):return 'hardware_clock'
    def get_profile(self):return Profile()
    def supports_frame_metadata(self, field):return True
    def get_frame_metadata(self, field):return 123


class Frames:
    def __init__(self, depth=None, color=None):
        self.depth=Frame(np.full((8,8),2400,np.uint16) if depth is None else depth)
        self.color=Frame(np.full((8,8,3),80,np.uint8) if color is None else color)
    def get_depth_frame(self):return self.depth
    def get_color_frame(self):return self.color


class Pipeline:
    def __init__(self):self.frames=Frames();self.calls=0
    def wait_for_frames(self):self.calls+=1;return self.frames


class Align:
    def __init__(self):self.calls=0
    def process(self, frames):
        self.calls+=1
        return Frames(frames.depth.data+np.uint16(10),frames.color.data)


class Spatial:
    def __init__(self):self.calls=0
    def process(self, frame):self.calls+=1;return Frame(frame.data+np.uint16(20))


@pytest.fixture
def setup(tmp_path):
    observers=[]
    class Camera:
        def __init__(self):
            self.pipeline=Pipeline();self.align=Align();self.depth_filters=[Spatial()]
            self.applied_depth_filters=[{'name':'spatial'}]
            self.depth_scale=.0001;self.runtime_profile={'stationary_temporal':{'active':True}}
            self.rs=SimpleNamespace(frame_metadata_value=SimpleNamespace(actual_exposure='exposure'))
            self.last_read_diagnostics=None;self.calls=0;self.raise_original=False;self.complete=True
        def _filter_stationary_depth(self, frame, diagnostic=None):
            return Frame(frame.data+np.uint16(30))
        def read(self):
            self.calls+=1
            frames=self.pipeline.wait_for_frames()
            frames=self.align.process(frames)
            if self.raise_original:raise RuntimeError('production failure')
            depth=frames.get_depth_frame()
            for filt in self.depth_filters:depth=filt.process(depth)
            depth=self._filter_stationary_depth(depth, {})
            self.last_read_diagnostics={'result':'complete' if self.complete else 'rejected_pair'}
            self.last_result=(frames.color.data.copy(),depth.data.copy())
            return self.last_result
    original=Camera.read
    def make(**kwargs):
        camera=Camera();clock=Clock()
        options=dict(seconds=30.,warmup=0.,stable_reads=1,samples=1,
                     clock=clock,start_thread=False)
        options.update(kwargs)
        obs=module.CameraStageObserver(Camera,tmp_path/('out%d'%len(observers)),**options)
        observers.append(obs)
        return camera,obs,clock
    yield make,Camera,original
    for obs in observers:obs.shutdown()
    Camera.read=original


def selected(setup, **kwargs):
    make,cls,original=setup
    camera,observer,clock=make(**kwargs)
    camera.read();clock.step()
    return camera,observer,clock,cls,original


def test_same_frame_stages_independent_and_original_return(setup):
    cam,obs,clock,cls,original=selected(setup)
    pipeline,align,filters=cam.pipeline,cam.align,cam.depth_filters
    returned=cam.read()
    assert returned is cam.last_result
    assert cls.read is original
    assert cam.pipeline is pipeline and cam.align is align and cam.depth_filters is filters
    assert '_filter_stationary_depth' not in cam.__dict__
    row=obs.queue.get_nowait();arrays=row['arrays']
    assert row['complete'] and set(arrays)==set(module.REQUIRED)
    for key,value in [('raw_depth',2400),('aligned_depth',2410),('post_spatial_depth',2430),
                      ('pre_stationary_depth',2430),('post_stationary_depth',2460),('final_depth',2460)]:
        assert np.all(arrays[key]==value)
    cam.pipeline.frames.depth.data[:]=999;returned[0][:]=1;returned[1][:]=2
    assert np.all(arrays['raw_depth']==2400) and np.all(arrays['final_depth']==2460)
    assert np.all(arrays['raw_rgb']==80) and np.all(arrays['final_rgb']==80)
    assert row['sdk_frames']['depth']['metadata_raw']['actual_exposure']==123
    assert cam.calls==pipeline.calls==align.calls==filters[0].calls==2


def test_warmup_sample_spacing_and_sample_cap(setup):
    make,cls,original=setup
    cam,obs,clock=make(warmup=3.,stable_reads=2,samples=2,interval=2.)
    for t in (0.,1.,2.,2.99,3.,3.5,4.99,5.):
        clock.value=t;cam.read()
    assert obs.counts['selected']==2
    assert obs.queue.qsize()==2 and cls.read is original
    assert obs.close_reason=='sample_limit'


def test_deadline_without_ready_restores_read(setup):
    make,cls,original=setup
    cam,obs,clock=make()
    cam.complete=False
    cam.read();clock.value=30.;returned=cam.read()
    assert returned is cam.last_result and obs.closed
    assert obs.counts['selected']==0 and cls.read is original


def test_original_exception_preserved_and_proxies_restored(setup):
    cam,obs,clock,cls,original=selected(setup)
    pipeline,align,filters=cam.pipeline,cam.align,cam.depth_filters
    cam.raise_original=True
    with pytest.raises(RuntimeError,match='production failure'):cam.read()
    assert cam.pipeline is pipeline and cam.align is align and cam.depth_filters is filters
    assert '_filter_stationary_depth' not in cam.__dict__ and cls.read is original
    assert obs.counts['original_read_errors']==1


@pytest.mark.parametrize('where',['raw','array','metadata'])
def test_observer_failures_do_not_change_production_return(setup,monkeypatch,where):
    cam,obs,clock,cls,original=selected(setup)
    def fail(*args,**kwargs):raise RuntimeError('tap failure')
    if where=='raw':monkeypatch.setattr(obs,'_raw',fail)
    elif where=='array':monkeypatch.setattr(obs,'_array',fail)
    else:cam.runtime_profile={'uncopyable':object()};monkeypatch.setattr(module.copy,'deepcopy',fail)
    assert cam.read() is cam.last_result
    assert cls.read is original and obs.errors
    assert cam.calls==2 and obs.counts['original_read_errors']==0


def test_queue_full_drops_diagnostic_not_frame(setup):
    cam,obs,clock,cls,original=selected(setup,samples=3,interval=.5,queue_size=1)
    for _ in range(3):
        assert cam.read() is cam.last_result
        clock.step(.5)
    assert obs.queue.qsize()==1 and obs.counts['queue_dropped']==2
    assert cam.calls==4 and cls.read is original


def test_real_async_writer_roundtrip(setup):
    cam,obs,clock,cls,original=selected(setup,start_thread=True)
    cam.read();obs.thread.join(3.)
    assert not obs.thread.is_alive()
    summary=json.loads((obs.output/'summary.json').read_text())
    assert summary['capture_complete'] and summary['original_read_restored']
    assert summary['counts']['written']==1 and summary['robot_commands_sent'] is False
    with np.load(obs.output/'sample_01.npz',allow_pickle=False) as z:
        assert np.all(z['raw_depth']==2400) and np.all(z['final_depth']==2460)
    meta=json.loads((obs.output/'sample_01.json').read_text())
    assert meta['sdk_frames']['depth']['timestamp_ms']==1234.
    assert 'source_stamp_ns' not in meta
    assert sum(p.stat().st_size for p in obs.output.iterdir())<obs.maximum_bytes


@pytest.mark.parametrize('failure',['serialization','io','budget'])
def test_writer_failure_restores_camera_and_reports_incomplete(setup,monkeypatch,failure):
    cam,obs,clock,cls,original=selected(setup)
    cam.read()
    if failure=='serialization':
        row=obs.queue.get_nowait();row['invalid']=float('nan');obs.queue.put_nowait(row)
    elif failure=='io':
        def fail(row):raise OSError('disk full')
        monkeypatch.setattr(obs,'_write_row',fail)
    else:obs.maximum_bytes=1
    obs._writer()
    v=json.loads((obs.output/'summary.json').read_text())
    assert not v['capture_complete'] and v['counts']['write_errors']==1
    assert cls.read is original and cam.read() is cam.last_result


def test_foreign_replacements_not_overwritten(setup):
    cam,obs,clock,cls,original=selected(setup)
    foreign=Pipeline()
    old=obs._raw
    def mutate(row,camera,frames):
        old(row,camera,frames);camera.pipeline=foreign
    obs._raw=mutate
    cam.read()
    assert cam.pipeline is foreign
    replacement=lambda camera:None
    cls.read=replacement;obs.finish()
    assert cls.read is replacement


@pytest.mark.parametrize('options',[{'seconds':31},{'samples':13},{'queue_size':3},
                                    {'maximum_bytes':65*module.MIB},{'interval':.1},
                                    {'seconds':float('nan')}])
def test_invalid_resource_limits_rejected(setup,options):
    make,cls,original=setup
    with pytest.raises(ValueError):make(**options)
    assert cls.read is original


def test_real_manager_read_keeps_rgb_correction_and_depth_clip_semantics(tmp_path,monkeypatch):
    source=path.parent.parent/'src/alicia_flexible_grasp_supervisor/src'
    monkeypatch.syspath_prepend(str(source))
    from alicia_flexible_grasp.vision.realsense_manager import RealSenseManager
    camera=RealSenseManager()
    camera.pipeline=Pipeline();camera.align=Align()
    camera.pipeline.frames.depth.data[0,:4]=[0,100,500,30000]
    camera.depth_filters=[Spatial()];camera.applied_depth_filters=[{'name':'spatial'}]
    camera.color_projection_correction=SimpleNamespace(apply=lambda a:a+np.uint8(5))
    camera.rs=SimpleNamespace(frame_metadata_value=SimpleNamespace())
    original=RealSenseManager.read
    baseline=original(camera)
    clock=Clock()
    obs=module.CameraStageObserver(RealSenseManager,tmp_path/'actual',warmup=0.,stable_reads=1,
                                   samples=1,clock=clock,start_thread=False)
    try:
        camera.read();clock.step();observed=camera.read()
        np.testing.assert_array_equal(observed[0],baseline[0])
        np.testing.assert_array_equal(observed[1],baseline[1])
        row=obs.queue.get_nowait()
        assert row['complete']
        assert np.all(row['arrays']['raw_rgb']==80)
        assert np.all(row['arrays']['final_rgb']==85)
        assert row['arrays']['raw_depth'][0].tolist()[:4]==[0,100,500,30000]
        assert row['arrays']['post_stationary_depth'][0].tolist()[:4]==[30,130,530,30030]
        assert row['arrays']['final_depth'][0].tolist()[:4]==[0,0,530,0]
        assert RealSenseManager.read is original
    finally:obs.shutdown()


def test_default_twelve_full_resolution_pairs_fit_actual_npz_budget():
    import io
    arrays={key:np.zeros((480,640,3),np.uint8) if key.endswith('rgb')
            else np.zeros((480,640),np.uint16) for key in module.REQUIRED}
    stream=io.BytesIO();np.savez(stream,**arrays)
    per_pair=len(stream.getvalue())
    assert per_pair<8*module.MIB
    # Current complete metadata is <16 KiB, leave twice that per sample plus
    # the independent overall reserve for manifest/summary.
    assert 12*(per_pair+32*1024)+256*1024<64*module.MIB


def test_deadline_during_selected_read_restores_without_cancelling_original(setup):
    cam,obs,clock,cls,original=selected(setup)
    raw=obs._raw
    def expiration(row,camera,frames):
        raw(row,camera,frames)
        obs.finish('deadline')
    obs._raw=expiration
    pipeline,align,filters=cam.pipeline,cam.align,cam.depth_filters
    assert cam.read() is cam.last_result
    assert cam.pipeline is pipeline and cam.align is align and cam.depth_filters is filters
    assert cls.read is original and not obs.queue.qsize()
    assert obs.inflight==0 and obs.close_reason=='deadline'


def test_timer_failure_does_not_prevent_normal_read(setup,monkeypatch):
    make,cls,original=setup
    cam,obs,clock=make()
    def fail(*args,**kwargs):raise RuntimeError('thread resources exhausted')
    monkeypatch.setattr(module.threading.Timer,'start',fail)
    assert cam.read() is cam.last_result
    assert cls.read is original and obs.close_reason=='timer_error'
    assert obs.counts['original_read_errors']==0


def test_unexpected_queue_error_does_not_change_return(setup,monkeypatch):
    cam,obs,clock,cls,original=selected(setup)
    def fail(row):raise RuntimeError('queue failed')
    monkeypatch.setattr(obs.queue,'put_nowait',fail)
    assert cam.read() is cam.last_result
    assert cls.read is original and obs.close_reason=='queue_error'
    assert obs.counts['original_read_errors']==0


def test_writer_thread_creation_failure_leaves_class_untouched(setup,monkeypatch):
    make,cls,original=setup
    def fail(*args,**kwargs):raise RuntimeError('cannot create writer')
    monkeypatch.setattr(module.threading.Thread,'start',fail)
    with pytest.raises(RuntimeError,match='cannot create writer'):make(start_thread=True)
    assert cls.read is original


@pytest.mark.parametrize('failure',['thread','manifest'])
def test_optional_setup_failure_still_runs_original_camera_entry(setup,tmp_path,monkeypatch,failure):
    _,cls,original=setup
    args=SimpleNamespace(output=tmp_path/'entry',camera_script=tmp_path/'camera.py',
                         seconds=30.,samples=12,interval=2.,warmup=3.,ros_args=['--','__name:=supervisor_camera'])
    def fail(*a,**k):raise RuntimeError('diagnostic setup failed')
    if failure=='thread':monkeypatch.setattr(module.threading.Thread,'start',fail)
    else:monkeypatch.setattr(module,'loaded_modules',fail)
    calls=[]
    def run(path,run_name):
        calls.append((path,run_name))
        assert cls.read is original
    monkeypatch.setattr(module.runpy,'run_path',run)
    module.run_camera_entry(SimpleNamespace(RealSenseManager=cls),args)
    assert calls==[(str(args.camera_script),'__main__')]
    summary=json.loads((args.output/'summary.json').read_text())
    assert not summary['capture_complete'] and summary['original_read_restored']


def test_deadline_with_concurrent_caller_cannot_bypass_temporary_proxies(setup):
    import threading
    cam,obs,clock,cls,original=selected(setup)
    entered,release=threading.Event(),threading.Event()
    raw=obs._raw
    def blocked(row,camera,frames):
        raw(row,camera,frames);entered.set();assert release.wait(2.)
    obs._raw=blocked
    returned=[]
    first=threading.Thread(target=lambda:returned.append(cam.read()))
    second=threading.Thread(target=lambda:returned.append(cam.read()))
    first.start();assert entered.wait(2.)
    obs.finish('deadline')
    assert cls.read is obs.wrapper and obs.inflight==1
    second.start()
    assert cam.calls==2  # initial warmup and the single blocked selected read
    release.set();first.join(2.);second.join(2.)
    assert not first.is_alive() and not second.is_alive()
    assert len(returned)==2 and cam.calls==3
    assert cls.read is original and type(cam.pipeline) is Pipeline
