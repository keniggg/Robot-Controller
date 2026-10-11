"""Previously qualified samples are stability history, never old plan authority."""
from copy import deepcopy
import hashlib
import json
import math


def _integer(value):
    if type(value) is not int or value <= 0:
        raise ValueError('positive integer required')
    return value


def _sha(value):
    if (not isinstance(value, str) or len(value) != 64
            or any(c not in '0123456789abcdef' for c in value)):
        raise ValueError('measurement digest required')
    return value


def station_binding(status, reference, mask, now_ns):
    """Read the producer's fixed-station receipt after the ordinary ready join."""
    row = status['initial_station_measurement']
    ns = _integer(status['source_stamp_ns'])
    station = _integer(row['station_source_stamp_ns'])
    epoch = _integer(row['epoch_ns'])
    q = row['q_station_rad']
    template = reference['identity_template']
    if (row.get('version') != 1 or type(row['version']) is not int
            or row.get('state') != 'qualified' or row.get('source_stamp_ns') != ns
            or row.get('metric_geometry_authority') is not False
            or row.get('motion_authorized') is not False
            or status.get('state') != 'ready' or status.get('target_lost') is not False
            or status.get('exact_rgbd') is not True or status.get('target_locked') is not True
            or not epoch <= station <= ns or reference['source_ns'] != ns
            or station != status['anchor_source_stamp_ns']
            or station != template['source_stamp_ns'] or epoch != template['epoch_ns']
            or row['source_id'] != template['source_id']
            or not ns <= now_ns < reference['deadline_ns']
            or any(status.get(k) != ns for k in ('color_stamp_ns', 'depth_stamp_ns', 'tf_lookup_stamp_ns'))
            or not mask[0] or mask[1] != status['color_frame_id']
            or _sha(row['refined_mask_sha256']) != mask[2]
            or not isinstance(q, (tuple, list)) or len(q) != 6
            or any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not math.isfinite(v) for v in q)):
        raise ValueError('fixed-station ready measurement required')
    return json.dumps(dict(station_source_stamp_ns=station, epoch_ns=epoch,
        source_id=_sha(row['source_id']), q_station_rad=q,
        selection=reference['selection'], reference_binding=reference['binding']),
        sort_keys=True, allow_nan=False)


def qualify_history(snapshot, records, recovery, *, cutoff_ns, identity, runtime_key,
                    now_ns, maximum_sec, frame_count, maximum_span_sec):
    """Require all five actual ready receipts, plus a fresh post-recovery source.

    The caller already ran the unchanged stable-sample fusion. This additional
    check does not manufacture samples or alter any fused arrays.
    """
    stamps = tuple(snapshot.sample_stamp_ns)
    if (frame_count != 5 or len(stamps) != frame_count
            or any(type(ns) is not int or ns <= 0 for ns in stamps)
            or any(a >= b for a, b in zip(stamps, stamps[1:]))
            or stamps[-1] != snapshot.stamp_ns or snapshot.target_identity != identity
            or snapshot.quality.fused_frames != frame_count
            or not snapshot.ok or not snapshot.object_msg.detected
            or stamps[-1] <= cutoff_ns or not recovery
            or recovery['source_ns'] != cutoff_ns + 1
            or not 0 <= now_ns-stamps[-1] < round(maximum_sec*1e9)
            or not 0 < maximum_sec <= .8 or not 0 < maximum_span_sec <= 12.
            or stamps[-1]-stamps[0] > round(maximum_span_sec*1e9)):
        raise ValueError('new recovered observation snapshot required')
    rows = [records[ns] for ns in stamps]
    if tuple(getattr(snapshot, 'sample_mask_sha256', ())) != tuple(row['mask_sha256'] for row in rows):
        raise ValueError('actual fusion mask history does not match ready receipts')
    if any(row['identity'] != identity or row['station'] != recovery['station']
           or row['runtime_key'] != runtime_key
           or not ns <= row['admitted_ns'] < row['freshness_deadline_ns']
           or row['freshness_deadline_ns']-ns > 800_000_000
           for ns, row in zip(stamps, rows)):
        raise ValueError('history station, identity or configuration changed')
    return dict(version=1, kind='qualified_observation_stability_history',
        source_stamps_ns=list(stamps), recovery_source_stamp_ns=recovery['source_ns'],
        current_source_stamp_ns=stamps[-1], metric_geometry_authority=False,
        motion_authorized=False)


class ObservationRecoveryHistoryMixin:
    def _reset_observation_recovery_history(self):
        self._observation_history_ready = {}
        self._observation_history_gap = None
        self._observation_history_recovery = None
        self._observation_history_entered = None

    def _observation_history_request_key(self):
        return (self._stream_generation, self._current_stream_target_identity(),
                json.dumps(self._mode_selection, sort_keys=True, allow_nan=False))

    def _observation_history_runtime_key(self):
        import rospy
        from .rgb_observation_source import camera_geometry_snapshot
        # Dynamic camera diagnostics are deliberately excluded by the existing
        # camera geometry contract; model, calibration and perception stay exact.
        get = rospy.get_param_cached
        if get('/gui/joint_direct_mode', False) is True:
            raise ValueError('manual ownership cannot retain observation history')
        value = dict(model=get('/robot_description', ''),
            camera=camera_geometry_snapshot(get('/camera')),
            perception=get('/unknown_perception', {}), handeye=get('/handeye', {}))
        if not isinstance(value['model'], str) or not value['model'].strip():
            raise ValueError('robot model required')
        return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()

    def _capture_observation_history_ready(self, status, reference, mask, now_ns):
        records = self._observation_history_ready
        ns = status['source_stamp_ns']
        try:
            station = station_binding(status, reference, mask, now_ns)
            runtime_key = self._observation_history_runtime_key()
            if records and any(row['station'] != station or row['runtime_key'] != runtime_key
                               for row in records.values()):
                records.clear()
                self._observation_history_recovery = None
            previous = records.get(ns)
            if previous is not None and previous['mask_sha256'] != mask[2]:
                records.pop(ns)
                self._observation_history_recovery = None
                return
            if previous is None:
                records[ns] = dict(station=station, identity=reference['identity'],
                    runtime_key=runtime_key, admitted_ns=now_ns,
                    freshness_deadline_ns=reference['deadline_ns'], mask_sha256=mask[2])
            while len(records) > 64:
                del records[min(records)]
        except Exception:
            # This optional evidence cache cannot affect ordinary ready inputs.
            records.pop(ns, None)

    def _begin_observation_history_gap(self, reference):
        self._observation_history_gap = None
        self._observation_history_recovery = None
        row = self._observation_history_ready.get(reference['source_ns'])
        if (row is not None and self._joint_space_observation_active()
                and self._observation_history_entered == self._observation_history_request_key()):
            self._observation_history_gap = deepcopy(row)

    def _commit_observation_history_recovery(self, pending, status, ns):
        row = self._observation_history_gap
        self._observation_history_gap = None
        try:
            # Called only after validate_support_receipt(recovery=True), the
            # original object acceptance and both unchanged deadline checks.
            # This is the newly recovered measured frame, not a renewal of the
            # old gap reference or its fixed identity-completion deadline.
            temporary = dict(pending, source_ns=ns,
                deadline_ns=ns+round(pending['maximum_sec']*1e9))
            station = station_binding(status, temporary, self._unknown_identity_masks[ns],
                                      self._ros_source_clock_ns())
            if (row is None or row['station'] != station
                    or row['identity'] != self._current_stream_target_identity()
                    or row['runtime_key'] != self._observation_history_runtime_key()
                    or not self._joint_space_observation_active()
                    or self._observation_history_entered != self._observation_history_request_key()):
                return
            self._observation_history_recovery = dict(source_ns=ns, station=station)
        except Exception:
            self._observation_history_recovery = None

    def _admit_observation_recovery_history(self, snapshot):
        if (not self._joint_space_observation_active() or not self._unknown_identity_scope()
                or self._observation_history_entered != self._observation_history_request_key()):
            return False
        try:
            import rospy
            import numpy as np
            from alicia_flexible_grasp.vision.rgbd_snapshot import retained_exact_source
            raw = rospy.get_param_cached('/unknown_perception', {})
            for ns in snapshot.sample_stamp_ns:
                obj = self._unknown_identity_objects[ns]
                mask = self._unknown_identity_masks[ns]
                status = self._unknown_identity_status[ns]
                if (not obj.detected or obj.header.stamp.to_nsec() != ns
                        or not mask[0] or status.get('state') != 'ready'
                        or mask[2] != self._observation_history_ready[ns]['mask_sha256']):
                    return False
            current = retained_exact_source(snapshot)
            if current is None or hashlib.sha256(np.ascontiguousarray(
                    np.asarray(current.object_mask) > 0, dtype=np.uint8).tobytes()).hexdigest() != (
                    self._observation_history_ready[snapshot.stamp_ns]['mask_sha256']):
                return False
            qualify_history(snapshot, self._observation_history_ready,
                self._observation_history_recovery,
                cutoff_ns=self._unknown_identity_cancelled_through_ns,
                identity=self._current_stream_target_identity(),
                runtime_key=self._observation_history_runtime_key(), now_ns=self._ros_source_clock_ns(),
                maximum_sec=min(float(raw.get('max_frame_age_sec', .8)), .8),
                frame_count=self.planning_snapshot_frames,
                maximum_span_sec=self.planning_snapshot_max_span_sec)
            return True
        except Exception:
            return False
