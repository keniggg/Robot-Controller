"""Finite raw-RGB/LK evidence capture. No ROS, SDK access or control calls.

The wrappers return the original objects and never alter source, images, visual
permission or depth. Archives are diagnostic visual-call snapshots, not proof
that the enclosing RGB-D read completed or that an image was published.
"""
from collections import Counter
import copy
import hashlib
import io
import json
import math
from pathlib import Path
import queue
import threading
import time
import weakref

import numpy as np

MAX_BYTES = 64*1024*1024
SIDECAR_RESERVE = 64*1024
MAX_QUEUE_BYTES = 8*1024*1024
MAX_ANCHOR_BYTES = 8*1024*1024
MAX_METADATA_BYTES = 16*1024
ARRAY_HEADER_RESERVE = 8*1024
MAX_PER_CATEGORY = 8
REGIONS = ('center', 'nested_center')
REASONS = ('visual_motion', 'stationary')


class VisualSnapshotTap:
    def __init__(self, regional_class, visual_class, output, seconds=30., *,
                 clock=time.monotonic, start_time=None, queue_size=2,
                 max_bytes=MAX_BYTES, start_thread=True, provenance=None):
        if (type(seconds) not in (int,float) or not math.isfinite(seconds)
                or not 0 < seconds <= 30 or type(queue_size) is not int or not 1 <= queue_size <= 4
                or type(max_bytes) is not int or not SIDECAR_RESERVE < max_bytes <= MAX_BYTES):
            raise ValueError('bounded snapshot duration, queue and byte budget required')
        self.regional_class,self.visual_class=regional_class,visual_class
        self.original_process,self.original_observe=regional_class.process,visual_class.observe
        self.original_reset=getattr(regional_class,'reset',None)
        self.output=Path(output);self.output.mkdir(exist_ok=False)
        self.seconds,self.clock,self.start_time=float(seconds),clock,start_time
        self.first_call=None;self.closed=False;self.close_reason=None
        self.lock=threading.RLock();self.local=threading.local()
        self.queue=queue.Queue(maxsize=queue_size);self.queue_bytes=0
        self.max_bytes=max_bytes;self.reserved_bytes=0;self.written_bytes=0
        self.inflight_copies=0;self.selected=Counter();self.counts=Counter()
        self.errors=[];self.files=[];self.thread=None
        self.anchors={};self.anchor_bytes=0;self.anchor_peak_bytes=0
        self.shadow_engines=weakref.WeakSet();self.inflight_nested_shadows=0
        self.provenance=copy.deepcopy(provenance or {})
        tap=self
        def observed_process(engine,frame,color,source,host,encoder_active,aligned_to_color=False):
            previous=getattr(tap.local,'context',None)
            try:
                if tap._within_window():
                    # References only; no SDK/depth reads and no image copy.
                    tap.local.context=dict(engine=engine,color=color,source=source,host=host,
                        encoder_active=encoder_active,aligned_to_color=aligned_to_color)
                else:tap.local.context=None
            except Exception as exc:
                tap.local.context=None;tap._error('context',exc)
            try:
                returned=tap.original_process(engine,frame,color,source,host,encoder_active,
                    aligned_to_color=aligned_to_color)
                # The sixth visual observer exists only inside this explicit
                # diagnostic window, after all five real owners have finished.
                try:tap._observe_shadow(engine,color,source,host,returned)
                except Exception as exc:tap._error('nested_shadow',exc)
                return returned
            finally:
                tap.local.context=previous
        def observed_visual(visual,color,source,host):
            context=getattr(tap.local,'context',None)
            before=None
            try:
                region=tap._region(context,visual)
                if region is not None and not tap.closed:
                    # Current implementation replaces anchors/points on seed;
                    # LK does not mutate these retained arrays in place.
                    before=dict(region=region,visual=visual,anchor=getattr(visual,'_anchor',None),
                        points=getattr(visual,'_points',None),previous=getattr(visual,'_previous',None),
                        anchor_stamp=getattr(visual,'_anchor_stamp',None),
                        anchor_host=getattr(visual,'_anchor_host',None),count=getattr(visual,'_count',None))
                    before['raw_anchor']=tap._anchor_binding(context,before)
            except Exception as exc:tap._error('pre_observe_reference',exc)
            returned=tap.original_observe(visual,color,source,host)
            if before is not None:
                try:tap._capture(context,before,color,source,host,returned)
                except Exception as exc:tap._error('capture',exc)
                # Motion may have seeded a NEW anchor. Capture above always
                # uses the old binding, then retain the actual new seed image.
                try:tap._remember_anchor(context,before,source,host)
                except Exception as exc:tap._error('anchor',exc)
            return returned
        def observed_reset(engine,*args,**kwargs):
            try:return tap.original_reset(engine,*args,**kwargs)
            finally:
                with tap.lock:
                    for region in list(tap.anchors):
                        if tap.anchors[region]['engine'] is engine:tap._drop_anchor(region)
        self.process_wrapper,self.observe_wrapper=observed_process,observed_visual
        self.reset_wrapper=observed_reset
        regional_class.process=observed_process;visual_class.observe=observed_visual
        if self.original_reset is not None:regional_class.reset=observed_reset
        try:
            if start_thread:
                self.thread=threading.Thread(target=self._writer,name='raw-lk-snapshots',daemon=True)
                self.thread.start()
        except Exception:
            self.finish('startup_error');raise

    def _error(self,stage,exc):
        with self.lock:
            self.counts['errors']+=1
            if len(self.errors)<8:self.errors.append(dict(stage=stage,error=str(exc)[:512]))

    def _within_window(self):
        if self.closed:return False
        now=self.clock()
        with self.lock:
            if self.first_call is None:
                start=self.start_time() if self.start_time is not None else now
                if start is None:return False
                self.first_call=float(start)
            expired=now-self.first_call>=self.seconds
        if expired:self.finish('window_expired')
        return not expired

    @staticmethod
    def _region(context,visual):
        if context is None:return None
        engine=context['engine'];regions=getattr(engine,'_regions',())
        if regions and regions[0].get('name')=='center' and regions[0]['visual']is visual:
            return 'center'
        if getattr(engine,'_nested_shadow_visual',None)is visual:return 'nested_center'
        return None

    @staticmethod
    def _clear_shadow(engine):
        # Never reset the five production visual windows or SDK filters.
        engine._nested_shadow_visual=None
        engine._nested_shadow_bounds=None

    def _observe_shadow(self,engine,color,source,host,returned):
        observer=getattr(engine,'_observe_nested_center_shadow',None)
        if (not callable(observer) or not self._within_window()
                or not isinstance(returned,tuple) or len(returned)!=2
                or not isinstance(returned[1],dict)
                or returned[1].get('reason')!='regional_observation'
                or len(returned[1].get('regions',[]))!=5):return
        with self.lock:
            if self.closed:return
            if engine not in self.shadow_engines:
                # An old diagnostic owner/anchor must not become this run's
                # first seed. Actual raw anchor capture starts with this call.
                self._clear_shadow(engine)
                self.shadow_engines.add(engine)
            self.inflight_nested_shadows+=1
            self.counts['nested_shadow_calls']+=1
        try:
            report=observer(color,source,host)
            if self._within_window():returned[1]['nested_center_shadow']=report
        finally:
            # finish() can run while OpenCV is in progress. It first clears
            # current state; repeat after the in-flight call to prevent revival.
            with self.lock:
                if self.closed:self._clear_shadow(engine)
                self.inflight_nested_shadows-=1

    def _drop_anchor(self,region):
        previous=self.anchors.pop(region,None)
        if previous is not None:self.anchor_bytes-=previous['raw'].nbytes

    def _anchor_binding(self,context,before):
        with self.lock:
            saved=self.anchors.get(before['region'])
            if saved is not None and (saved['engine'] is not context['engine']
                    or saved['visual'] is not before['visual'] or saved['anchor'] is not before['anchor']
                    or saved['points'] is not before['points']
                    or saved['stamp']!=before['anchor_stamp'] or saved['host']!=before['anchor_host']):
                self._drop_anchor(before['region']);saved=None
            return saved

    def _remember_anchor(self,context,before,source,host):
        if not self._within_window():return
        visual=before['visual'];anchor=getattr(visual,'_anchor',None)
        if anchor is None:
            with self.lock:self._drop_anchor(before['region'])
            return
        if anchor is before['anchor']:return
        full=context['color'];points=getattr(visual,'_points',None)
        if (not isinstance(full,np.ndarray) or full.dtype!=np.uint8 or full.ndim!=3 or full.shape[2]!=3
                or not isinstance(anchor,np.ndarray) or not isinstance(points,np.ndarray)
                or source!=context['source'] or host!=context['host']
                or getattr(visual,'_anchor_stamp',None)!=source.get('device_timestamp_ms')
                or getattr(visual,'_anchor_host',None)!=host):
            raise ValueError('new actual anchor cannot bind to raw SDK source')
        with self.lock:
            self._drop_anchor(before['region'])
            if self.closed:return
            if full.nbytes+self.anchor_bytes>MAX_ANCHOR_BYTES:
                self.counts['anchor_budget_rejected']+=1;return
            # Only two region anchors, never a time-indexed pixel history.
            raw=full.copy();raw.flags.writeable=False
            self.anchors[before['region']]=dict(engine=context['engine'],visual=visual,
                anchor=anchor,points=points,stamp=visual._anchor_stamp,host=host,
                raw=raw,source=copy.deepcopy(source),bounds=tuple(context['engine']._regions[0]['bounds']
                    if before['region']=='center' else context['engine']._nested_shadow_bounds))
            self.anchor_bytes+=raw.nbytes
            self.anchor_peak_bytes=max(self.anchor_peak_bytes,self.anchor_bytes)

    def _capture(self,context,before,color,source,host,returned):
        if not self._within_window():return
        reason=returned.get('reason') if isinstance(returned,dict) else None
        if reason not in REASONS:return
        category=before['region']+'/'+reason
        with self.lock:
            self.counts['eligible']+=1
            if self.selected[category]>=MAX_PER_CATEGORY:
                self.counts['category_full']+=1;return
            if self.queue.full():self.counts['queue_full']+=1;return
        full,anchor,points=context['color'],before['anchor'],before['points']
        binding=before.get('raw_anchor')
        if binding is None:
            with self.lock:self.counts['unbound_pre_anchor']+=1
            return
        if (not isinstance(full,np.ndarray) or full.dtype!=np.uint8 or full.ndim!=3 or full.shape[2]!=3
                or not isinstance(anchor,np.ndarray) or anchor.dtype!=np.uint8 or anchor.ndim!=2
                or not isinstance(points,np.ndarray) or points.dtype!=np.float32
                or points.ndim!=3 or points.shape[1:]!=(1,2) or len(points)>64):
            raise ValueError('raw or actual pre-anchor/seed evidence unavailable')
        engine=context['engine']
        bounds=(engine._regions[0]['bounds'] if before['region']=='center'
                else engine._nested_shadow_bounds)
        if bounds is None:raise ValueError('visual input bounds unavailable')
        if tuple(bounds)!=binding['bounds']:raise ValueError('anchor bounds changed')
        x0,y0,x1,y1=bounds
        if tuple(color.shape)!=((y1-y0),(x1-x0),3):raise ValueError('observed crop geometry mismatch')
        if source!=context['source'] or host!=context['host']:
            raise ValueError('visual source differs from enclosing regional source')
        metadata=dict(schema=1,region=before['region'],category=reason,
            diagnostic_only=True,motion_authorized=False,pixels_changed=False,
            coordinate_domain='raw_sdk_color_before_pair_rectification',
            source=copy.deepcopy(source),filter_host_monotonic=host,
            observed_monotonic=self.clock(),bounds_xyxy=list(bounds),
            raw_color_shape=list(full.shape),observed_crop_shape=list(color.shape),
            source_implementation=copy.deepcopy(self.provenance),
            encoder_active=context['encoder_active'],aligned_to_color=context['aligned_to_color'],
            pre_observe=dict(previous=before['previous'],anchor_device_timestamp_ms=before['anchor_stamp'],
                previous_source_tuple=before['previous'],continuous_frame_count=before['count'],
                anchor_host_monotonic=before['anchor_host'],continuous_frames=before['count'],
                anchor_source=copy.deepcopy(binding['source']),anchor_bound_verified=True,
                anchor_filter_host_monotonic=binding['host'],anchor_raw_color_shape=list(binding['raw'].shape),
                anchor_gray_shape=list(anchor.shape),seed_points_count=len(points)),
            scale_full_resolution_xy=[(x1-x0)/float(anchor.shape[1]),(y1-y0)/float(anchor.shape[0])],
            original_return=copy.deepcopy(returned),
            provenance='Actual pre-call gray anchor and feature points; independently retained actual seed full raw RGB and SDK source; actual current full raw SDK color; original returned report.',
            publication_outcome_unknown=True,enclosing_read_outcome_unknown=True)
        encoded=json.dumps(metadata,ensure_ascii=False,allow_nan=False,separators=(',',':')).encode()
        if len(encoded)>MAX_METADATA_BYTES:raise ValueError('snapshot metadata too large')
        reserve=full.nbytes+binding['raw'].nbytes+anchor.nbytes+points.nbytes+len(encoded)+ARRAY_HEADER_RESERVE
        with self.lock:
            if self.closed:return
            if self.reserved_bytes+reserve>self.max_bytes-SIDECAR_RESERVE:
                self.counts['byte_budget_rejected']+=1;return
            if self.queue_bytes+reserve>MAX_QUEUE_BYTES:
                self.counts['queue_bytes_rejected']+=1;return
            self.reserved_bytes+=reserve;self.queue_bytes+=reserve;self.inflight_copies+=1
        try:
            arrays=dict(raw_color=full.copy(),anchor_raw_color=binding['raw'].copy(),
                        anchor_gray=anchor.copy(),seed_points=points.copy(),
                        metadata_json=np.frombuffer(encoded,dtype=np.uint8).copy())
            for a in arrays.values():a.flags.writeable=False
            with self.lock:
                if self.closed or self.selected[category]>=MAX_PER_CATEGORY or self.queue.full():
                    self.counts['copy_commit_rejected']+=1
                    self.reserved_bytes-=reserve;self.queue_bytes-=reserve;return
                index=sum(self.selected.values())
                self.queue.put_nowait(dict(index=index,category=category,arrays=arrays,
                    reserve=reserve,source=copy.deepcopy(source)))
                self.selected[category]+=1;self.counts['enqueued']+=1
        except Exception:
            with self.lock:self.reserved_bytes-=reserve;self.queue_bytes-=reserve
            raise
        finally:
            with self.lock:self.inflight_copies-=1

    def finish(self,reason='explicit_finish'):
        with self.lock:
            if self.regional_class.process is self.process_wrapper:self.regional_class.process=self.original_process
            if self.visual_class.observe is self.observe_wrapper:self.visual_class.observe=self.original_observe
            if self.original_reset is not None and self.regional_class.reset is self.reset_wrapper:
                self.regional_class.reset=self.original_reset
            self.closed=True
            for engine in list(self.shadow_engines):self._clear_shadow(engine)
            self.shadow_engines.clear()
            self.anchors.clear();self.anchor_bytes=0
            if self.close_reason is None:self.close_reason=reason

    def _writer(self):
        try:
            while True:
                # Also restores hooks if acquisition stalls without another call.
                if not self.closed and (self.first_call is not None or self.start_time is not None):
                    self._within_window()
                with self.lock:
                    if (self.closed and self.queue.empty() and not self.inflight_copies
                            and not self.inflight_nested_shadows):break
                try:item=self.queue.get(timeout=.05)
                except queue.Empty:continue
                try:
                    buffer=io.BytesIO();np.savez(buffer,**item['arrays']);raw=buffer.getvalue()
                    if len(raw)>item['reserve']:raise ValueError('snapshot exceeded reserved byte bound')
                    name='snapshot_%02d.npz'%item['index']
                    with (self.output/name).open('xb')as f:
                        if f.write(raw)!=len(raw):raise OSError('short snapshot write')
                    with self.lock:
                        self.written_bytes+=len(raw);self.counts['written']+=1
                        self.files.append(dict(name=name,bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest(),
                            category=item['category'],source=item['source']))
                except Exception as exc:self._error('write',exc);self.finish('writer_error')
                finally:
                    with self.lock:self.queue_bytes-=item['reserve']
        except Exception as exc:self._error('writer',exc);self.finish('writer_error')
        finally:
            self.finish(self.close_reason or 'writer_exit')
            with self.lock:
                summary=dict(schema=1,sealed=True,diagnostic_only=True,pixels_changed=False,
                    motion_authorized=False,coordinate_domain='raw_sdk_color_before_pair_rectification',
                    maximum_bytes=self.max_bytes,directory_bytes_before_summary=self.written_bytes,
                    reserved_bytes=self.reserved_bytes,maximum_per_region_reason=MAX_PER_CATEGORY,
                    categories=dict(self.selected),counts=dict(self.counts),errors=list(self.errors),
                    files=list(self.files),queued_unwritten=self.queue.qsize(),
                    inflight_copies=self.inflight_copies,queue_bytes=self.queue_bytes,
                    maximum_anchor_bytes=MAX_ANCHOR_BYTES,anchor_peak_bytes=self.anchor_peak_bytes,
                    retained_anchor_bytes=self.anchor_bytes,source_implementation=self.provenance,
                    first_read_monotonic=self.first_call,finished_monotonic=self.clock(),
                    requested_window_sec=self.seconds,close_reason=self.close_reason,
                    original_process_restored=self.regional_class.process is self.original_process,
                    original_observe_restored=self.visual_class.observe is self.original_observe,
                    original_reset_restored=self.original_reset is None or self.regional_class.reset is self.original_reset,
                    nested_shadow_default_disabled=True,nested_shadow_retained_owners=len(self.shadow_engines),
                    inflight_nested_shadows=self.inflight_nested_shadows,
                    limitations=['Visual-call evidence only; enclosing read/publication success is not implied.',
                        'No depth copied or processed; no production permission depends on this capture.',
                        'Window and restoration use a monotonic watchdog; scheduling or storage stalls can delay final sealing.'])
            try:
                encoded=(json.dumps(summary,ensure_ascii=False,allow_nan=False,indent=2)+'\n').encode()
                if len(encoded)>SIDECAR_RESERVE:raise ValueError('summary exceeded reserved budget')
                with (self.output/'summary.json').open('xb')as f:f.write(encoded)
            except Exception as exc:
                self._error('summary',exc)
                print('VISUAL_SNAPSHOT_SUMMARY_ERROR '+repr(exc),flush=True)

    def shutdown(self):
        self.finish('shutdown')
        if self.thread is not None and self.thread is not threading.current_thread():self.thread.join(timeout=3.)
