#!/usr/bin/env python3
"""Finite read-only first-loss archive: exact detector-source RGB-D, no control."""
import argparse
from collections import Counter, OrderedDict
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import io
import json
import math
import os
from pathlib import Path
import sys
import threading
import time

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from bounded_async_bag_writer import BoundedAsyncBagWriter


class ExactDetectorSources:
    """Bounded image cache; every requested source has an explicit final result."""
    def __init__(self, started_ns, image_limit=48, request_limit=512):
        self.started_ns = int(started_ns)
        self.image_limit, self.request_limit = image_limit, request_limit
        self.images = {key: OrderedDict() for key in ('rgb', 'depth')}
        self.requests, self.complete = OrderedDict(), {}
        self.no_source_count = self.historical_source_count = 0
        self.evictions = Counter()

    def image(self, kind, source, message, receipt):
        self.images[kind][source] = (message, receipt)
        while len(self.images[kind]) > self.image_limit:
            self.images[kind].popitem(last=False)
            self.evictions[kind] += 1
        return self.ready()

    def status(self, payload, receipt):
        source = payload.get('source_stamp_ns')
        if type(source) is not int or source <= 0:
            self.no_source_count += 1
            return []
        if source <= self.started_ns:
            self.historical_source_count += 1
            return []
        if source not in self.requests:
            if len(self.requests) >= self.request_limit:
                raise RuntimeError('detector source request limit exceeded')
            self.requests[source] = dict(source_stamp_ns=source, status_receipt_ns=receipt,
                generation=payload.get('generation'), selection_stamp_ns=payload.get('selection_stamp_ns'),
                state=payload.get('state'), detail=payload.get('detail'),
                target_lost=payload.get('target_lost'), first_loss=payload.get('first_loss'))
        return self.ready()

    def ready(self):
        result = []
        for source, row in self.requests.items():
            if source in self.complete or not all(source in self.images[k] for k in self.images):
                continue
            rgb, rr = self.images['rgb'][source]
            depth, dr = self.images['depth'][source]
            evidence = dict(row, rgb_receipt_ns=rr, depth_receipt_ns=dr)
            self.complete[source] = evidence
            result.append((evidence, rgb, depth))
        return result

    def report(self):
        return dict(requested_count=len(self.requests), paired_count=len(self.complete),
            paired_sources=list(self.complete.values()),
            missing_sources=[dict(row, available_image_kinds=[k for k in self.images if source in self.images[k]])
                for source,row in self.requests.items() if source not in self.complete],
            no_source_status_count=self.no_source_count,
            historical_source_status_count=self.historical_source_count,
            image_cache_limit_per_kind=self.image_limit, request_limit=self.request_limit,
            image_evictions=dict(self.evictions))


PAIR_BUDGET_BYTES = 8 * 1024 * 1024
MAX_FEEDBACK_BRACKET_NS = 200_000_000
JOINT_NAMES = tuple('Joint%d' % index for index in range(1, 7))


def diagnostic_source_files():
    """Normalize the entrypoint even when Python was launched with a relative path."""
    return [Path(__file__).resolve(), HERE/'bounded_async_bag_writer.py',
        HERE.parent/'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py',
        HERE.parent/'src/alicia_grasp_modes/src/alicia_grasp_modes/loss_observation.py']


def diagnostic_source_sha256():
    return {str(path.relative_to(HERE.parent)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in diagnostic_source_files()}


def finalization_value(stage, read, errors):
    """A failed final context/provenance read must not discard captured images."""
    try:
        value = read()
        # Validate before report construction so an unusual ROS parameter type
        # cannot defer its failure until after all raw evidence is in memory.
        json.dumps(value, allow_nan=False)
        return value
    except Exception as error:
        errors.append(dict(stage=stage, error=str(error)[:512]))
        return None


def serialize_pair_records(slots):
    """Isolate each serialization failure; retain every independently valid message."""
    records, errors = [], []
    def add(topic, message, receipt):
        try:
            stream = io.BytesIO(); message.serialize(stream)
            records.append((topic, stream.getvalue(), receipt, message._type, message._md5sum, type(message)))
        except Exception as error:
            errors.append(dict(stage='message_serialization', topic=topic,
                receipt_ns=receipt, error=str(error)[:512]))
    for slot in slots:
        if len(slot['images']) == 2:
            for kind, (message, receipt) in slot['images'].items():
                add('/unknown_loss/exact_'+kind, message, receipt)
        if slot['status_message'] is not None:
            add('/perception/unknown/detector_status', slot['status_message'], slot['status_receipt_ns'])
        if slot['object'] is not None:
            add('/perception/unknown/object', *slot['object'])
        feedback_rows = {}
        for bracket in (slot.get('observed_bracket'), slot['accepted_bracket']):
            for source, message, receipt in bracket or []:
                feedback_rows[(source, receipt)] = (message, receipt)
        for message, receipt in feedback_rows.values():
            add('/alicia_d/accepted_joint_states', message, receipt)
        if slot['exact_tf'] is not None:
            add('/unknown_loss/exact_tf', slot['exact_tf'], slot['status_receipt_ns'])
        for topic, (message, receipt) in slot['context'].items():
            add(topic, message, receipt)
    return records, errors


def image_layout(message):
    """Use the actual native transport layout, including row padding."""
    width, height, step = int(message.width), int(message.height), int(message.step)
    if min(width, height, step) <= 0 or len(message.data) != height * step:
        raise ValueError('invalid actual image shape/step/data length')
    return dict(width=width, height=height, step=step, encoding=message.encoding,
                is_bigendian=int(message.is_bigendian), native_bytes=len(message.data))


class FirstLossPairSources:
    """Pin two source identities without saving the intervening image stream.

    Ready requires the source-stamped ObjectPose.detected=True as well as ready
    status; headerless Bool receipt order is deliberately not an identity proof.
    All methods are called under the owner's lock. ROS messages remain native.
    """
    def __init__(self, started_ns, image_limit=48):
        self.started_ns, self.image_limit = int(started_ns), int(image_limit)
        self.images = {kind: OrderedDict() for kind in ('rgb', 'depth')}
        self.objects, self.statuses, self.feedback = OrderedDict(), OrderedDict(), OrderedDict()
        self.context = {}
        self.last_ready = self.trigger = None
        self.slots, self.ignored_historical, self.ignored_unarmed = [], 0, 0
        self.rejected_feedback = Counter()

    def _bounded(self, cache, key, value, maximum):
        cache[key] = value
        while len(cache) > maximum:
            cache.popitem(last=False)

    def image(self, kind, source, message, receipt):
        self._bounded(self.images[kind], source, (message, receipt), self.image_limit)
        self.refresh()

    def object(self, source, message, receipt):
        self._bounded(self.objects, source, (message, receipt), self.image_limit)
        self._select()

    def status(self, payload, receipt, message=None):
        source = payload.get('source_stamp_ns')
        if type(source) is int and source > self.started_ns:
            self._bounded(self.statuses, source, (deepcopy(payload), receipt, message), self.image_limit)
        self._select()
        if self.trigger is None:
            first = payload.get('first_loss')
            if isinstance(first, dict):
                observed = first.get('observed_stamp_ns')
                if type(observed) is int and observed > self.started_ns:
                    self._freeze('first_loss', first.get('source_stamp_ns'),
                        first.get('last_good_source_stamp_ns'), payload, receipt, message)
                else:
                    self.ignored_historical += 1

    def accepted_feedback(self, source, message, receipt):
        try:
            epoch = self.context.get('/alicia_d/control_reference_epoch')
            epoch_ns = None if epoch is None else epoch[0].stamp.to_nsec()
            if message.header.frame_id != 'sdk_measured':
                raise ValueError('accepted_feedback_frame_not_sdk_measured')
            names, positions = message.name, message.position
            if (len(names) != len(positions) or len(names) > 32
                    or len(set(names)) != len(names) or any(name not in names for name in JOINT_NAMES)):
                raise ValueError('accepted_feedback_requires_six_unique_named_joints')
            values = dict(zip(names, positions))
            if not all(math.isfinite(float(values[name])) for name in JOINT_NAMES):
                raise ValueError('accepted_feedback_nonfinite_joints')
            if (type(source) is not int or source <= 0 or message.header.stamp.to_nsec() != source
                    or epoch_ns is None or epoch_ns <= 0 or source < epoch_ns):
                raise ValueError('accepted_feedback_missing_epoch_or_pre_epoch_stamp')
        except (AttributeError, TypeError, ValueError) as error:
            self.rejected_feedback[str(error)] += 1
            return
        self._bounded(self.feedback, source, (message, receipt), 256)
        self.refresh()

    def current_context(self, topic, message, receipt):
        self.context[topic] = (message, receipt)

    def _slot(self, source, payload, receipt, message, role):
        return dict(source_stamp_ns=source, role=role, status=deepcopy(payload),
            status_receipt_ns=receipt, status_message=message, images={},
            object=None, accepted_bracket=None, observed_bracket=None, exact_tf=None, tf_error=None,
            context=dict(self.context))

    def _select(self):
        if self.trigger is not None:
            self.refresh()
            return
        for source in sorted(self.statuses):
            payload, receipt, message = self.statuses[source]
            obj = self.objects.get(source)
            if obj is not None and payload.get('state') == 'ready' and obj[0].detected is True:
                previous = self.last_ready
                if previous is None or source > previous['source_stamp_ns']:
                    self.last_ready = self._slot(source, payload, receipt, message, 'last_good')
            hold = payload.get('rgb_identity_hold') or {}
            if (obj is not None and obj[0].detected is False and hold.get('state') == 'holding_identity_only'
                    and hold.get('source_stamp_ns') == source):
                self._freeze('holding_identity_only', source,
                    hold.get('reference_source_stamp_ns'), payload, receipt, message)
                if self.trigger is not None:
                    break
            qualification = payload.get('source_qualification') or {}
            if (obj is not None and obj[0].detected is False
                    and qualification.get('state') == 'pending_source_qualification'
                    and qualification.get('source_stamp_ns') == source):
                self._freeze('pending_source_qualification', source,
                    qualification.get('reference_source_stamp_ns'), payload, receipt, message)
                if self.trigger is not None:
                    break
            first = payload.get('first_loss') or {}
            observed = first.get('observed_stamp_ns')
            if (self.last_ready is not None and type(observed) is int
                    and observed > max(self.started_ns, self.last_ready['source_stamp_ns'])
                    and all(self.last_ready['status'].get(key) == first.get(key)
                            for key in ('generation', 'selection_stamp_ns'))):
                self._freeze('first_loss', first.get('source_stamp_ns'),
                    first.get('last_good_source_stamp_ns'), payload, receipt, message)
                if self.trigger is not None:
                    break
        self.refresh()

    def _freeze(self, kind, source, expected_good, payload, receipt, message):
        if self.trigger is not None:
            return
        candidate = self.last_ready
        first = payload.get('first_loss') if kind == 'first_loss' else payload
        same_scope = candidate is not None and all(
            candidate['status'].get(key) == first.get(key)
            for key in ('generation', 'selection_stamp_ns'))
        if not same_scope:
            # A latched old selection or loss before any observed detection
            # cannot terminate a capture armed before a perception reload.
            self.ignored_unarmed += 1
            return
        def epoch(context):
            item = context.get('/alicia_d/control_reference_epoch')
            return None if item is None else item[0].stamp.to_nsec()
        ready_epoch, event_epoch = epoch(candidate['context']), epoch(self.context)
        epoch_matches = bool(ready_epoch is not None and ready_epoch > 0 and ready_epoch == event_epoch)
        match = bool(same_scope and type(expected_good) is int and expected_good > 0
                     and candidate['source_stamp_ns'] == expected_good)
        valid_source = type(source) is int and source > self.started_ns
        self.trigger = dict(kind=kind, source_stamp_ns=source,
            last_good_source_stamp_ns=expected_good, last_good_source_matches=match,
            pinned_ready_source_stamp_ns=None if candidate is None else candidate['source_stamp_ns'],
            last_good_epoch_ns=ready_epoch, event_epoch_ns=event_epoch, epoch_matches=epoch_matches,
            observed_status_receipt_ns=receipt,
            depth_gap_kind=(payload.get('rgb_identity_hold') or {}).get('depth_gap_kind'),
            source_qualification=deepcopy(payload.get('source_qualification')))
        good = candidate if match else self._slot(expected_good, {}, receipt, None, 'last_good')
        if not match:
            good['unavailable_reason'] = 'asserted_last_good_source_does_not_match_pinned_detected_ready'
        elif not epoch_matches:
            good['unavailable_reason'] = 'control_reference_epoch_missing_or_changed'
        event = self._slot(source, payload, receipt, message, kind)
        if not valid_source:
            event['unavailable_reason'] = 'first_event_has_no_post_start_exact_source'
        elif source == expected_good:
            event['unavailable_reason'] = 'first_event_source_does_not_follow_last_good'
        elif type(expected_good) is int and source < expected_good:
            event['unavailable_reason'] = 'first_event_source_precedes_last_good'
        self.slots = [good, event]
        self.refresh()

    def pending_slots(self):
        return self.slots if self.trigger is not None else ([] if self.last_ready is None else [self.last_ready])

    def refresh(self):
        for slot in self.pending_slots():
            source = slot['source_stamp_ns']
            if slot.get('unavailable_reason'):
                continue
            for kind, cache in self.images.items():
                if kind not in slot['images'] and source in cache:
                    slot['images'][kind] = cache[source]
            if slot['object'] is None:
                slot['object'] = self.objects.get(source)
            if slot['accepted_bracket'] is None:
                epoch = slot['context'].get('/alicia_d/control_reference_epoch')
                if epoch is None:
                    continue
                epoch_ns = epoch[0].stamp.to_nsec()
                before = max((ns for ns in self.feedback if epoch_ns <= ns <= source), default=None)
                after = min((ns for ns in self.feedback if ns >= max(source, epoch_ns)), default=None)
                if before is not None and after is not None:
                    bracket = [(ns, *self.feedback[ns]) for ns in (before, after)]
                    # Retain the first actual surrounding SDK samples even
                    # when their interval cannot qualify this source. A late
                    # middle sample may qualify it separately, never rewrite
                    # the originally observed interval or widen its limit.
                    if slot['observed_bracket'] is None:
                        slot['observed_bracket'] = bracket
                    if after-before <= MAX_FEEDBACK_BRACKET_NS:
                        slot['accepted_bracket'] = bracket

    def report(self):
        rows = []
        for slot in self.slots:
            source = slot['source_stamp_ns']
            missing = []
            if slot.get('unavailable_reason'):
                missing.append(slot['unavailable_reason'])
            missing.extend('exact_'+kind+'_unavailable' for kind in self.images if kind not in slot['images'])
            if slot['status'].get('source_stamp_ns') != source or slot['status_message'] is None:
                missing.append('corresponding_detector_status_unavailable')
            obj = slot['object']
            expected_detected = slot['role'] == 'last_good'
            if obj is None or obj[0].detected is not expected_detected:
                missing.append('corresponding_detected_object_unavailable')
            bracket = slot['accepted_bracket']
            observed_bracket = slot['observed_bracket']
            if bracket is None:
                missing.append('accepted_feedback_bracket_within_200ms_unavailable')
            if slot['exact_tf'] is None:
                missing.append('exact_source_tf_unavailable')
            for topic in ('/grasp_mode/selection', '/alicia_d/control_reference_epoch'):
                if topic not in slot['context']:
                    missing.append('current_context_unavailable:'+topic)
            rows.append(dict(role=slot['role'], source_stamp_ns=source,
                available_image_kinds=list(slot['images']), unavailable=missing,
                accepted_feedback_source_ns=None if bracket is None else [r[0] for r in bracket],
                accepted_feedback_span_ns=None if bracket is None else bracket[1][0]-bracket[0][0],
                observed_feedback_source_ns=None if observed_bracket is None else [r[0] for r in observed_bracket],
                observed_feedback_span_ns=None if observed_bracket is None else observed_bracket[1][0]-observed_bracket[0][0],
                observed_feedback_bracket_qualified=bool(observed_bracket is not None
                    and observed_bracket[1][0]-observed_bracket[0][0] <= MAX_FEEDBACK_BRACKET_NS),
                exact_tf_source_stamp_ns=None if slot['exact_tf'] is None else source,
                exact_tf_error=slot['tf_error'], status=slot['status'],
                status_receipt_ns=slot['status_receipt_ns'],
                context_receipt_ns={topic: value[1] for topic, value in slot['context'].items()}))
        return dict(trigger=self.trigger, sources=rows, ignored_historical_first_loss=self.ignored_historical,
            ignored_unarmed_or_other_selection_events=self.ignored_unarmed,
            rejected_accepted_feedback=dict(self.rejected_feedback),
            paired_count=sum(len(slot['images']) == 2 for slot in self.slots),
            evidence_complete=bool(self.trigger is not None and len(rows) == 2
                                   and not any(row['unavailable'] for row in rows)))


def pair_preflight(slots, serialized_records, sidecar_bytes, maximum_bytes=PAIR_BUDGET_BYTES):
    """Reserve generous bag connection/index overhead before any image write."""
    if len(slots) > 2:
        raise ValueError('at most two exact RGB-D pairs are allowed')
    layouts, native_bytes = [], 0
    for slot in slots:
        if len(slot['images']) != 2:
            continue
        pair = {kind: image_layout(value[0]) for kind, value in slot['images'].items()}
        if any(value[0].header.stamp.to_nsec() != slot['source_stamp_ns'] for value in slot['images'].values()):
            raise ValueError('exact image stamp differs from pinned source')
        if (pair['rgb']['width'], pair['rgb']['height']) != (pair['depth']['width'], pair['depth']['height']):
            raise ValueError('RGB-D actual image shapes differ')
        layouts.append(dict(source_stamp_ns=slot['source_stamp_ns'], **pair))
        native_bytes += sum(row['native_bytes'] for row in pair.values())
    serialized_bytes = sum(len(payload) for _, payload, _, _, _, _ in serialized_records)
    # Includes footer, connections and index records; native ROS bags are uncompressed.
    overhead = 1024*1024 + len(serialized_records)*2048
    required = max(native_bytes, serialized_bytes) + sidecar_bytes + overhead
    result = dict(maximum_bytes=maximum_bytes, estimated_maximum_bytes=required,
        native_image_bytes=native_bytes, serialized_message_bytes=serialized_bytes,
        sidecar_reserved_bytes=sidecar_bytes, bag_overhead_reserved_bytes=overhead,
        actual_image_layouts=layouts, fits=required <= maximum_bytes)
    if not result['fits']:
        raise ValueError('8 MiB first-loss-pair preflight budget insufficient: '+json.dumps(result))
    return result


class CappedBagFile(io.FileIO):
    """Hard write cap in addition to preflight; seeking cannot bypass the cap."""
    def __init__(self, path, maximum_bytes):
        super().__init__(path, 'w+')
        self.maximum_bytes = int(maximum_bytes)

    def write(self, data):
        if self.tell()+len(data) > self.maximum_bytes:
            raise RuntimeError('first-loss-pair hard artifact budget exceeded')
        return super().write(data)

    def truncate(self, size=None):
        size = self.tell() if size is None else size
        if size > self.maximum_bytes:
            raise RuntimeError('first-loss-pair hard artifact budget exceeded')
        return super().truncate(size)


def collect_first_loss_pair(seconds):
    """Passive ROS capture; raw image writes only after bounded preflight."""
    mount = Path('/media/zhuyupei/SanStand')
    if not os.path.ismount(mount):
        raise RuntimeError('SanStand is not mounted; no fallback archive')
    out = mount/'实验记录/Robot-Controller/ros_log'/('unknown_first_loss_pair_'+
        datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+str(os.getpid()))
    out.mkdir(); (out/'ros').mkdir(); os.environ['ROS_LOG_DIR'] = str(out/'ros')
    import rospy, rosbag, tf2_ros
    from sensor_msgs.msg import Image, JointState
    from std_msgs.msg import String, Header, Bool
    from diagnostic_msgs.msg import DiagnosticArray
    from alicia_flexible_grasp_supervisor.msg import ObjectPose, ObjectGeometry
    rospy.init_node('unknown_first_loss_pair_readonly', anonymous=True, disable_signals=True)
    started = rospy.Time.now().to_nsec()
    join = FirstLossPairSources(started)
    condition = threading.Condition(threading.RLock())
    state = dict(closed=False, active=0, faults=[], status_count=0)
    camera = rospy.get_param('/camera', {})
    perception = rospy.get_param('/unknown_perception', {})
    handeye = rospy.get_param('/handeye', {})
    robot_description = rospy.get_param('/robot_description', '')
    base_frame = str(perception.get('base_frame', 'base_link'))
    camera_frame = str(perception.get('camera_frame', camera.get('frame_id', 'camera_link')))
    buffer = tf2_ros.Buffer(cache_time=rospy.Duration(60.))
    listener = tf2_ros.TransformListener(buffer)
    rgb_topic = camera.get('color_topic', '/supervisor/camera/color/image_raw')
    depth_topic = camera.get('depth_topic', '/supervisor/camera/depth/image_raw')
    status_topic, object_topic = '/perception/unknown/detector_status', '/perception/unknown/object'
    feedback_topic = '/alicia_d/accepted_joint_states'
    topics = [(rgb_topic, Image), (depth_topic, Image), (status_topic, String),
        (object_topic, ObjectPose), (feedback_topic, JointState),
        ('/grasp_mode/selection', String), ('/alicia_d/control_reference_epoch', Header),
        ('/alicia_d/control_reference', JointState), ('/alicia_d/sdk_command', JointState),
        ('/alicia_d/actuation_status', String), ('/alicia_d/sdk_diagnostics', DiagnosticArray),
        ('/alicia_d/feedback_ready', Bool), ('/grasp_6d/object_geometry', ObjectGeometry),
        ('/grasp_6d/pipeline_metrics', String), ('/grasp_6d/status', String)]

    def callback(topic, message):
        receipt = rospy.Time.now().to_nsec()
        with condition:
            if state['closed']:
                return
            state['active'] += 1
            try:
                if topic in (rgb_topic, depth_topic):
                    join.image('rgb' if topic == rgb_topic else 'depth',
                        message.header.stamp.to_nsec(), message, receipt)
                elif topic == object_topic:
                    join.object(message.header.stamp.to_nsec(), message, receipt)
                elif topic == status_topic:
                    state['status_count'] += 1
                    join.status(json.loads(message.data), receipt, message)
                elif topic == feedback_topic:
                    join.accepted_feedback(message.header.stamp.to_nsec(), message, receipt)
                else:
                    join.current_context(topic, message, receipt)
            except Exception as error:
                if len(state['faults']) < 8:
                    state['faults'].append(str(error))
            finally:
                state['active'] -= 1
                condition.notify_all()

    def exact_transforms():
        with condition:
            slots = list(join.pending_slots())
        for slot in slots:
            if slot['exact_tf'] is not None or slot.get('unavailable_reason'):
                continue
            ns = slot['source_stamp_ns']
            try:
                transform = buffer.lookup_transform(base_frame, camera_frame,
                    rospy.Time(ns//10**9, ns%10**9), rospy.Duration(0))
                if transform.header.stamp.to_nsec() != ns:
                    raise ValueError('TF result timestamp differs from exact requested source')
                with condition:
                    slot['exact_tf'], slot['tf_error'] = transform, None
            except Exception as error:
                with condition:
                    slot['tf_error'] = str(error)[:512]

    subs = [rospy.Subscriber(topic, kind, lambda msg, topic=topic: callback(topic, msg),
        queue_size=48, buff_size=2**24) for topic, kind in topics]
    print(json.dumps(dict(ready=True, session=str(out), duration_seconds=seconds,
        mode='first_loss_pair', maximum_artifact_bytes=PAIR_BUDGET_BYTES,
        maximum_rgbd_pairs=2, motion_authorized=False)), flush=True)
    run_error = None
    try:
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline and not rospy.is_shutdown():
            exact_transforms()
            with condition:
                if state['faults']:
                    raise RuntimeError(state['faults'][0])
                if join.report()['evidence_complete']:
                    break
            time.sleep(.05)
    except BaseException as error:
        run_error = str(error)
    finally:
        with condition:
            state['closed'] = True
        for sub in subs:
            try:
                sub.unregister()
            except Exception as error:
                state['faults'].append('subscriber_unregister: '+str(error)[:512])
        with condition:
            drained = condition.wait_for(lambda: state['active'] == 0, timeout=5.)
        exact_transforms()
        try:
            listener.unregister()
        except Exception as error:
            state['faults'].append('tf_listener_unregister: '+str(error)[:512])

    finalization_errors = []
    final_read = lambda name: finalization_value('current_parameter:'+name,
        lambda: rospy.get_param(name, '' if name == '/robot_description' else {}), finalization_errors)
    current_handeye = final_read('/handeye')
    current_robot_description = final_read('/robot_description')
    current_camera = final_read('/camera')
    current_perception = final_read('/unknown_perception')
    metadata = dict(diagnostic_only=True, motion_authorized=False, started_ns=started,
        mode='first_loss_pair', duration_seconds=seconds, maximum_rgbd_pairs=2,
        maximum_artifact_bytes=PAIR_BUDGET_BYTES, board_pose_used=False,
        raw_archive_policy='Only pinned exact last detected-ready and first source-pending/identity-hold/loss; missing stays unavailable.',
        accepted_feedback_maximum_bracket_ns=MAX_FEEDBACK_BRACKET_NS,
        exact_tf_parent=base_frame, exact_tf_child=camera_frame,
        camera_parameters=finalization_value('initial_camera', lambda: camera, finalization_errors),
        unknown_perception_parameters=finalization_value('initial_perception', lambda: perception, finalization_errors),
        handeye_parameters=finalization_value('initial_handeye', lambda: handeye, finalization_errors),
        current_handeye_parameters=current_handeye,
        robot_description_sha256=hashlib.sha256(robot_description.encode()).hexdigest(),
        current_robot_description_sha256=finalization_value('current_robot_description_sha256',
            lambda: hashlib.sha256(current_robot_description.encode()).hexdigest(), finalization_errors),
        current_camera_parameters=current_camera, current_unknown_perception_parameters=current_perception,
        source_sha256=finalization_value('source_sha256', diagnostic_source_sha256, finalization_errors),
        finalization_errors=finalization_errors, topics=[topic for topic, _ in topics])
    metadata['handeye_unchanged'] = metadata['handeye_parameters'] == metadata['current_handeye_parameters']
    metadata['robot_description_unchanged'] = metadata['robot_description_sha256'] == metadata['current_robot_description_sha256']
    metadata['source_context_unchanged'] = bool(metadata['handeye_unchanged']
        and metadata['robot_description_unchanged']
        and camera == metadata['current_camera_parameters']
        and perception == metadata['current_unknown_perception_parameters']
        and not finalization_errors)
    report = dict(metadata, source_evidence=join.report(), callbacks_drained=drained,
        status_count=state['status_count'], faults=state['faults'], run_error=run_error,
        evidence_complete=False, archive_bag_closed=False)
    records, serialization_errors = serialize_pair_records(join.slots)
    finalization_errors.extend(serialization_errors)
    if finalization_errors:
        report['run_error'] = run_error or 'finalization_incomplete: '+', '.join(
            sorted({row['stage'] for row in finalization_errors}))
    encode = lambda value: (json.dumps(value, indent=2, sort_keys=True)+'\n').encode()
    metadata_bytes = encode(metadata)
    urdf_bytes = robot_description.encode()
    report_reserve = len(encode(report))+128*1024
    existing_bytes = sum(path.stat().st_size for path in out.rglob('*') if path.is_file())
    sidecar_reserve = len(metadata_bytes)+len(urdf_bytes)+report_reserve+existing_bytes+128*1024
    try:
        report['budget_preflight'] = pair_preflight(join.slots, records, sidecar_reserve)
        capped = CappedBagFile(out/'evidence.bag', PAIR_BUDGET_BYTES-sidecar_reserve)
        bag = None
        try:
            bag = rosbag.Bag(capped, 'w')
            for topic, payload, receipt, datatype, md5sum, pytype in records:
                bag.write(topic, (datatype, payload, md5sum, pytype),
                    rospy.Time(receipt//10**9, receipt%10**9), raw=True)
            bag.close()
            report['archive_bag_closed'] = True
        finally:
            capped.close()
        (out/'metadata.json').write_bytes(metadata_bytes)
        (out/'robot_description.urdf').write_bytes(urdf_bytes)
        report['evidence_complete'] = bool(join.report()['evidence_complete'] and drained
            and not state['faults'] and run_error is None and report['archive_bag_closed']
            and metadata['source_context_unchanged'] and not finalization_errors)
    except Exception as error:
        report['run_error'] = str(error)
        report['evidence_complete'] = False
    report['artifact_bytes_before_report'] = sum(path.stat().st_size for path in out.rglob('*') if path.is_file())
    report_bytes = encode(report)
    if report['artifact_bytes_before_report']+len(report_bytes) > PAIR_BUDGET_BYTES:
        # Oversized input context must not make even the explicit refusal exceed the cap.
        report = dict(mode='first_loss_pair', evidence_complete=False,
            run_error='artifact context/report exceeds 8 MiB budget', trigger=join.trigger,
            maximum_artifact_bytes=PAIR_BUDGET_BYTES)
        report_bytes = encode(report)
    (out/'report.json').write_bytes(report_bytes)
    print(json.dumps(dict(session=str(out), evidence_complete=report['evidence_complete'],
        trigger=join.trigger, paired_count=join.report()['paired_count'],
        run_error=report.get('run_error'), actual_artifact_bytes=sum(
            path.stat().st_size for path in out.rglob('*') if path.is_file())), indent=2), flush=True)
    if not report['evidence_complete']:
        raise RuntimeError('first-loss-pair evidence unavailable/incomplete; see archived report')


def parse_arguments(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=40.)
    parser.add_argument('--first-loss-pair', action='store_true',
        help='Save at most the last detected-ready and first pending/hold/loss source; finite wait 30–180 seconds, 8 MiB total.')
    args=parser.parse_args(argv)
    maximum = 180. if args.first_loss_pair else 45.
    if not 30. <= args.seconds <= maximum:
        parser.error('duration must be 30–%d seconds for this mode' % maximum)
    return args


def main():
    args = parse_arguments()
    if args.first_loss_pair:
        return collect_first_loss_pair(args.seconds)
    mount=Path('/media/zhuyupei/SanStand')
    if not os.path.ismount(mount):
        raise RuntimeError('SanStand is not mounted; no fallback archive')
    out=mount/'实验记录/Robot-Controller/ros_log'/('unknown_first_loss_readonly_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+str(os.getpid()))
    out.mkdir(); (out/'ros').mkdir(); os.environ['ROS_LOG_DIR']=str(out/'ros')
    import rospy, rosbag
    from sensor_msgs.msg import Image, JointState
    from std_msgs.msg import String, Header, Bool
    from tf2_msgs.msg import TFMessage
    from diagnostic_msgs.msg import DiagnosticArray
    from alicia_flexible_grasp_supervisor.msg import ObjectPose, ObjectGeometry
    rospy.init_node('unknown_first_loss_readonly', anonymous=True, disable_signals=True)
    started=rospy.Time.now().to_nsec()
    join=ExactDetectorSources(started)
    bag=rosbag.Bag(str(out/'evidence.bag'),'w')
    condition=threading.Condition(threading.RLock())
    state=dict(closed=False, active=0, faults=[], status_count=0, first_losses={}, status_states=Counter())
    def failed(reason):
        with condition:
            state['faults'].append(str(reason))
    def raw_write(metadata,payload,pytype):
        ns=metadata['stamp_ns']
        bag.write(metadata['topic'],(metadata['datatype'],payload,metadata['md5sum'],pytype),
                  rospy.Time(ns//10**9, ns%10**9),raw=True)
    writer=BoundedAsyncBagWriter(raw_write,failed)
    def save(topic,msg,receipt):
        try:
            stream=io.BytesIO(); msg.serialize(stream)
            writer.submit(topic,stream.getvalue(),receipt,msg._type,msg._md5sum,type(msg))
        except BaseException as error:
            writer.latch_failure('readonly archive serialization/admission: '+str(error))
            raise
    def callback(topic,msg):
        receipt=rospy.Time.now().to_nsec()
        with condition:
            if state['closed']: return
            state['active']+=1
        try:
            with condition:
                if topic==rgb_topic:
                    pairs=join.image('rgb',msg.header.stamp.to_nsec(),msg,receipt)
                elif topic==depth_topic:
                    pairs=join.image('depth',msg.header.stamp.to_nsec(),msg,receipt)
                elif topic=='/perception/unknown/detector_status':
                    payload=json.loads(msg.data)
                    state['status_count']+=1
                    state['status_states'][str(payload.get('detail'))]+=1
                    first=payload.get('first_loss')
                    if first is not None:
                        key=str((first.get('generation'),first.get('selection_stamp_ns')))
                        state['first_losses'].setdefault(key,first)
                    pairs=join.status(payload,receipt)
                else:
                    pairs=[]
            if topic not in (rgb_topic,depth_topic):
                save(topic,msg,receipt)
            for row,rgb,depth in pairs:
                row.update(rgb_frame_id=rgb.header.frame_id,depth_frame_id=depth.header.frame_id,
                    rgb_seq=rgb.header.seq,depth_seq=depth.header.seq,
                    rgb_dimensions=[rgb.width,rgb.height],depth_dimensions=[depth.width,depth.height],
                    raw_pair_source_equal=rgb.header.stamp.to_nsec()==depth.header.stamp.to_nsec())
                save('/unknown_loss/exact_rgb',rgb,row['rgb_receipt_ns'])
                save('/unknown_loss/exact_depth',depth,row['depth_receipt_ns'])
                save('/unknown_loss/exact_pair_provenance',String(data=json.dumps(row,sort_keys=True)),receipt)
        except BaseException as error:
            writer.latch_failure('readonly evidence callback: '+str(error))
        finally:
            with condition:
                state['active']-=1; condition.notify_all()
    camera=rospy.get_param('/camera',{})
    rgb_topic=camera.get('color_topic','/supervisor/camera/color/image_raw')
    depth_topic=camera.get('depth_topic','/supervisor/camera/depth/image_raw')
    topics=[(rgb_topic,Image),(depth_topic,Image),
        ('/perception/unknown/detector_status',String),('/perception/unknown/object_mask',Image),
        ('/perception/unknown/object',ObjectPose),('/perception/unknown/object_detected',Bool),
        ('/grasp_mode/selection',String),('/alicia_d/accepted_joint_states',JointState),
        ('/alicia_d/control_reference',JointState),('/alicia_d/sdk_command',JointState),
        ('/alicia_d/control_reference_epoch',Header),('/alicia_d/actuation_status',String),
        ('/alicia_d/sdk_diagnostics',DiagnosticArray),('/alicia_d/feedback_ready',Bool),
        ('/tf',TFMessage),('/tf_static',TFMessage),('/grasp_6d/object_geometry',ObjectGeometry),
        ('/grasp_6d/pipeline_metrics',String),('/grasp_6d/status',String)]
    subs=[rospy.Subscriber(topic,kind,lambda msg,topic=topic:callback(topic,msg),
        queue_size=48,buff_size=2**24) for topic,kind in topics]
    metadata=dict(diagnostic_only=True,motion_authorized=False,started_ns=started,
        duration_seconds=args.seconds,raw_archive_policy='Every detector source, including unavailable; no detector sources are rate-thinned.',
        board_pose_used=False,topics=[t for t,_ in topics],camera_parameters=camera,
        unknown_perception_parameters=rospy.get_param('/unknown_perception',{}))
    (out/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    (out/'robot_description.urdf').write_text(rospy.get_param('/robot_description',''))
    for file in diagnostic_source_files():
        (out/file.name).write_bytes(file.read_bytes())
    (out/'source_sha256.json').write_text(json.dumps({file.name:hashlib.sha256(file.read_bytes()).hexdigest() for file in out.iterdir() if file.is_file()},indent=2)+'\n')
    print(json.dumps(dict(ready=True,session=str(out),duration_seconds=args.seconds,motion_authorized=False)),flush=True)
    run_error=None
    try:
        deadline=time.monotonic()+args.seconds
        while time.monotonic()<deadline and not rospy.is_shutdown():
            writer.require_healthy(); time.sleep(.05)
    except BaseException as error:
        run_error=str(error)
    finally:
        with condition:
            state['closed']=True
        for sub in subs: sub.unregister()
        with condition:
            callbacks_done=condition.wait_for(lambda:state['active']==0,timeout=5.)
        if not callbacks_done:
            writer.latch_failure('readonly callback drain deadline exhausted')
        drained=False
        try:
            summary=writer.close(); drained=True
        except BaseException as error:
            run_error=run_error or str(error); summary=writer.summary()
        if drained: bag.close()
        for index,(entry,payload,_type,_cost) in enumerate(writer.failed_payloads):
            file=out/('failed_payload_%03d'%index)
            file.with_suffix('.bin').write_bytes(payload)
            file.with_suffix('.json').write_text(json.dumps(entry,indent=2)+'\n')
        report=dict(metadata,writer=summary,callbacks_drained=callbacks_done,
            archive_bag_closed=drained,run_error=run_error,faults=state['faults'],
            status_count=state['status_count'],status_details=dict(state['status_states']),
            first_losses=list(state['first_losses'].values()),source_evidence=join.report())
        report['evidence_complete']=bool(summary['archive_complete'] and callbacks_done and drained and not report['source_evidence']['missing_sources'])
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(dict(session=str(out),evidence_complete=report['evidence_complete'],
            status_count=report['status_count'],first_losses=report['first_losses'],
            requested_count=len(join.requests),paired_count=len(join.complete),writer=summary),indent=2),flush=True)
    if not report['evidence_complete']:
        raise RuntimeError('read-only evidence incomplete; see archived report')


if __name__=='__main__': main()
