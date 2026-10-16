"""Exact three-source evidence for a non-authorizing initial preflight wait.

This collector does not reset nodes, renew identity, or grant motion.
"""
from copy import deepcopy
from .feedback_bracket_gap import KIND as FEEDBACK_GAP
import hashlib
import math
import threading
import time

import numpy as np

from .unknown_pending_identity import (reference_binding,
    initial_station_template, validate_support_receipt, validate_feedback_processing,
    support_identity_deadline)
from .feedback_recovery_completion import FIELD as FEEDBACK_COMPLETION, STATE as FEEDBACK_PROCESSING
from .observation_recovery_history import station_binding
from .tabletop import Config


class InitialPreflightRecoveryCollector:
    def __init__(self, selection, maximum_sec=.8, support_policy=None):
        self.selection = deepcopy(selection)
        if (not isinstance(maximum_sec, (int,float)) or isinstance(maximum_sec,bool)
                or not math.isfinite(maximum_sec) or not 0 < maximum_sec <= .8):
            raise ValueError('invalid current measurement maximum')
        self.maximum_sec = maximum_sec
        defaults = Config()
        self.support_policy = dict(support_policy or {key: getattr(defaults,key) for key in (
            'support_max_normal_change_deg','support_max_local_shift_m',
            'support_threshold_m','support_min_ratio')})
        self.lock = threading.RLock()
        self.objects = {}; self.masks = {}; self.statuses = {}
        self.references = {}; self.gaps = {}; self.recoveries = {}
        self.template = None; self.anchor_binding = None
        self.latest_seen_ns = 0; self.latest_joined_ns = 0
        self.fatal = ''

    @staticmethod
    def _store(cache, ns, row):
        if ns in cache and {k:v for k,v in cache[ns].items() if k!='received_ns'} != {
                k:v for k,v in row.items() if k!='received_ns'}:
            # Same-source semantic replacement cannot complete a trusted join.
            raise ValueError('same source content changed')
        if ns not in cache:cache[ns] = row
        while len(cache) > 64:
            del cache[min(cache)]

    def object(self, msg, now_ns):
        try:
            ns = msg.header.stamp.to_nsec()
            point = msg.pose_base.pose.position
            row = dict(detected=msg.detected, frame=msg.header.frame_id,
                pose_frame=msg.pose_base.header.frame_id,
                pose_ns=msg.pose_base.header.stamp.to_nsec(),
                position=[float(point.x),float(point.y),float(point.z)], received_ns=now_ns)
            self._receive(self.objects, ns, row)
        except Exception as exc:
            with self.lock:self.fatal = 'object_input:' + str(exc)

    def mask(self, msg, now_ns):
        try:
            ns = msg.header.stamp.to_nsec()
            if (msg.encoding != 'mono8' or msg.height <= 0 or msg.width <= 0
                    or msg.step < msg.width or len(msg.data) != msg.step*msg.height):
                raise ValueError('exact mono8 mask required')
            array = np.frombuffer(bytes(msg.data),dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width]
            normalized = np.ascontiguousarray(array != 0,dtype=np.uint8)
            row = dict(nonempty=bool(np.any(normalized)), frame=msg.header.frame_id,
                sha256=hashlib.sha256(normalized.tobytes()).hexdigest(),
                shape=[msg.height,msg.width], received_ns=now_ns)
            self._receive(self.masks, ns, row)
        except Exception as exc:
            with self.lock:self.fatal = 'mask_input:' + str(exc)

    def status(self, status, now_ns):
        if (not isinstance(status,dict) or status.get('generation') != self.selection['generation']
                or status.get('selection_stamp_ns') != self.selection['stamp_ns']
                or status.get('mode') != 'unknown'):
            return
        try:
            ns = status.get('source_stamp_ns')
            if type(ns) is not int or ns <= self.selection['stamp_ns']:
                return
            if status.get('target_lost') is True:
                with self.lock:self.fatal = 'producer_target_lost'
            if (status.get('source_qualification') or {}).get('state') == FEEDBACK_PROCESSING:
                with self.lock:
                    reference_ns = status['source_qualification'].get('reference_source_stamp_ns')
                    matches = [gap for gap in self.gaps.values()
                               if gap['reference']['source_ns'] == reference_ns]
                    if not matches:
                        return
                    latest = max(matches, key=lambda gap: gap['status']['source_stamp_ns'])
                    completed = self.recoveries.get(ns)
                    if completed is not None and completed['reference_source_ns'] == reference_ns:
                        if status != latest['reference'].get('_completion_processing_status'):
                            raise ValueError('completed feedback processing receipt changed')
                        return
                    value = validate_feedback_processing(status, latest['reference'], now_ns)
                    for gap in matches:
                        reference = gap['reference']
                        if reference.get(FEEDBACK_COMPLETION) is None:
                            reference[FEEDBACK_COMPLETION] = deepcopy(value)
                            reference['_completion_monotonic'] = time.monotonic() + (
                                value['identity_completion_deadline_ns']-now_ns)*1e-9
                            reference['_completion_last_ros_ns'] = now_ns
                            reference['_completion_processing_status'] = deepcopy(status)
                        else:
                            support_identity_deadline(reference, now_ns, ns)
                return
            row = {key:deepcopy(value) for key,value in status.items()
                if key not in ('frame_timing','geometry_metrics','observation_interest')}
            self._receive(self.statuses, ns, dict(value=row,received_ns=now_ns))
        except Exception as exc:
            with self.lock:self.fatal = 'status_input:' + str(exc)

    def _receive(self, cache, ns, row):
        if type(ns) is not int or ns <= self.selection['stamp_ns']:
            return
        received = row['received_ns']
        if type(received) is not int or received < ns:
            raise ValueError('future or invalid receive timestamp')
        with self.lock:
            self._store(cache,ns,row)
            self.latest_seen_ns = max(self.latest_seen_ns,ns)
            self._join(ns)

    def _join(self, ns):
        if ns not in self.objects or ns not in self.masks or ns not in self.statuses:
            return
        if ns in self.gaps:
            # _store already rejects semantic changes. Repeated old gap
            # callbacks must not erase an established completion or its clocks.
            return
        obj,mask,srow = self.objects[ns],self.masks[ns],self.statuses[ns]
        status=srow['value'];now=max(obj['received_ns'],mask['received_ns'],srow['received_ns'])
        if not 0 <= now-ns < round(self.maximum_sec*1e9):
            return
        if self.template is None and not (status.get('state')=='ready' and status.get('detail')=='acquired'):
            return
        try:
            binding=reference_binding(status,self.selection)
            template=initial_station_template(status,self.selection)
            if template is None:
                raise ValueError('immutable initial RGB identity required')
            if self.template is None:
                if (status.get('state')!='ready' or status.get('detail')!='acquired'
                        or ns!=template['source_stamp_ns'] or not obj['detected']
                        or not mask['nonempty'] or mask['sha256']!=template['refined_mask_sha256']
                        or mask['shape']!=template['image_shape']):
                    return
                self.template=deepcopy(template);self.anchor_binding=binding
            if template != self.template or binding != self.anchor_binding:
                raise ValueError('immutable initial identity or anchor changed')
            if obj['frame']!='base_link' or mask['frame']!=status['color_frame_id']:
                raise ValueError('source frames changed')
            q=status.get('source_qualification',{})
            reference_ns=q.get('reference_source_stamp_ns')
            if status.get('state')=='unavailable':
                if obj['detected'] or mask['nonempty']:
                    raise ValueError('gap is not non-authorizing')
                reference=deepcopy(self.references.get(reference_ns))
                if reference is None:return
                source_id=validate_support_receipt(status,reference,now)
                reference['source_id']=source_id
                reference['gap_kind']=q['kind']
                if q['kind'] == FEEDBACK_GAP:
                    reference['feedback_gap_receipt'] = deepcopy(q)
                if q.get('identity_completion') is not None:
                    reference['identity_completion']=deepcopy(q['identity_completion'])
                self.gaps[ns]=dict(reference=reference,status=deepcopy(status),registered_ns=now)
            elif status.get('state')=='ready':
                if (not obj['detected'] or not mask['nonempty']
                        or obj['pose_frame']!='base_link' or obj['pose_ns']!=ns
                        or not all(math.isfinite(v) for v in obj['position'])
                        or len(status['position_base'])!=3
                        or any(abs(a-b)>1e-6 for a,b in zip(obj['position'],status['position_base']))):
                    raise ValueError('current measured ObjectPose mismatch')
                reference=dict(source_ns=ns,selection=deepcopy(self.selection),binding=binding,
                    maximum_sec=self.maximum_sec,deadline_ns=ns+round(self.maximum_sec*1e9),
                    _reference_monotonic=time.monotonic()+(ns+round(self.maximum_sec*1e9)-now)*1e-9,
                    _reference_last_ros_ns=now,
                    support_policy=deepcopy(self.support_policy),identity_template=deepcopy(template),
                    initial_station_measurement=deepcopy(status['initial_station_measurement']),
                    footprint_vertices=len(status['reference_support_footprint_base']),registered_ns=now)
                station_binding(status,reference,(mask['nonempty'],mask['frame'],mask['sha256'],mask['shape']),now)
                if q.get('kind') in ('support_measurement_gap', FEEDBACK_GAP):
                    matches=[gap for gap in self.gaps.values() if gap['reference']['source_ns']==reference_ns]
                    if not matches:return
                    gap=max(matches,key=lambda row:row['status']['source_stamp_ns'])
                    old=gap['reference']
                    validate_support_receipt(status,old,now,recovery=True)
                    deadline=support_identity_deadline(old,now,ns)
                    self.recoveries[ns]=dict(reference_source_ns=reference_ns,
                        pending_source_ns=gap['status']['source_stamp_ns'],source_ns=ns,
                        identity_deadline_ns=deadline,registered_ns=now,source_id=old['source_id'],
                        station=deepcopy(reference['initial_station_measurement']))
                self.references.setdefault(ns,reference)
                self.latest_joined_ns=max(self.latest_joined_ns,ns)
            else:return
            for cache in (self.references,self.gaps,self.recoveries):
                while len(cache)>64:del cache[min(cache)]
        except (ValueError,TypeError,KeyError,IndexError,OverflowError) as exc:
            self.fatal='source_join:'+str(exc)

