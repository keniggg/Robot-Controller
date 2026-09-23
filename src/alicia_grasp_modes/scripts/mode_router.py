#!/usr/bin/env python3
"""Select one perception stream and serialize mode changes with task starts.

Controls perception/planning state only. The original task owns all motion.
"""
import json
import sys
import threading
import time
from pathlib import Path

import message_filters
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, SetBoolResponse, Trigger
from alicia_flexible_grasp_supervisor.msg import ObjectPose, GraspState, Grasp6DPlan
from alicia_flexible_grasp_supervisor.srv import StartGrasp, StartGraspResponse, TriggerZero
from alicia_grasp_modes.policy import ModePolicy
from alicia_grasp_modes.srv import SetGraspMode, SetGraspModeResponse


class ModeRouter:
    def __init__(self):
        self.lock = threading.RLock()
        old = rospy.get_param('/grasp_mode/selection', {})
        self.policy = ModePolicy(generation=int(old.get('generation', 0)))
        self.target_receipt = 0.
        self.last_source_stamp = 0
        self.last_status = ''
        self.call_started_ns = 0
        self.service_uncertain = False
        self.completion_seen = False
        self.source_status = {}
        self.desired = str(rospy.get_param('~initial_mode', 'carton'))
        self.desired_strategy = str(rospy.get_param('~initial_strategy', 'two_stage'))
        if self.desired not in ('carton', 'unknown'):
            raise ValueError('invalid initial_mode')
        if self.desired_strategy not in ('two_stage', 'direct'):
            raise ValueError('invalid initial_strategy')
        self.obj_pub = rospy.Publisher('/perception/object', ObjectPose, queue_size=1)
        self.mask_pub = rospy.Publisher('/perception/object_mask', Image, queue_size=1)
        self.cam_pub = rospy.Publisher('/perception/object_pose_camera', PoseStamped, queue_size=1)
        self.base_pub = rospy.Publisher('/perception/object_pose_base', PoseStamped, queue_size=1)
        self.detected_pub = rospy.Publisher('/perception/object_detected', Bool, queue_size=1)
        self.raw_pub = rospy.Publisher('/perception/raw_object_detected', Bool, queue_size=1)
        self.detector_pub = rospy.Publisher('/perception/detector_status', String, queue_size=1, latch=True)
        self.selection_pub = rospy.Publisher('/grasp_mode/selection', String, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher('/grasp_mode/status', String, queue_size=1, latch=True)
        self.subscribers = []
        self.syncs = []
        for mode in ('carton', 'unknown'):
            prefix = '/perception/' + mode
            obj = message_filters.Subscriber(prefix + '/object', ObjectPose, queue_size=5)
            mask = message_filters.Subscriber(prefix + '/object_mask', Image, queue_size=5, buff_size=2**22)
            sync = message_filters.TimeSynchronizer([obj, mask], 10)
            sync.registerCallback(lambda o, m, source=mode: self.on_frame(source, o, m))
            self.subscribers.extend([obj, mask, rospy.Subscriber(prefix + '/detector_status', String,
                lambda msg, source=mode: self.on_status(source, msg), queue_size=1)])
            self.syncs.append(sync)
        self.subscribers.append(rospy.Subscriber('/grasp/state', GraspState, self.on_state, queue_size=10))
        for topic in ('/grasp_6d/preview_plan_enriched', '/grasp_6d/plan_enriched'):
            self.subscribers.append(rospy.Subscriber(topic, Grasp6DPlan,
                lambda msg, source=topic: self.on_plan(source, msg), queue_size=2))
        self.start_proxy = rospy.ServiceProxy('/grasp/internal_start', StartGrasp)
        self.request_plan = rospy.ServiceProxy('/grasp_6d/request_plan', TriggerZero)
        self.reset_remote = rospy.ServiceProxy('/grasp_mode/reset_remote', Trigger)
        self.reset_task = rospy.ServiceProxy('/grasp_mode/reset_task', Trigger)
        self.mode_service = rospy.Service('/grasp_mode/set_unknown', SetBool, self.set_unknown)
        self.selection_service = rospy.Service('/grasp_mode/select', SetGraspMode, self.select)
        self.start_service = rospy.Service('/grasp/start', StartGrasp, self.start)
        self.timer = rospy.Timer(rospy.Duration(.5), self.tick)
        self.emit('initializing')

    def emit(self, state, detail=''):
        source = self.source_status.get(self.policy.mode, '')
        if self.policy.mode == 'unknown' and source:
            try:
                parsed = json.loads(source)
                source = '%s:%s' % (parsed.get('state', ''), parsed.get('detail', ''))
            except (TypeError, ValueError):
                source = 'invalid detector status'
        message = dict(self.policy.selection(), state=state, detail=detail,
                       pending=self.policy.pending, pending_strategy=self.policy.pending_strategy,
                       active=self.policy.active,
                       start_reserved=self.policy.reserved, source_status=source)
        text = json.dumps(message, sort_keys=True)
        if text != self.last_status:
            self.status_pub.publish(String(data=text))
            self.last_status = text
            rospy.loginfo('grasp mode %s', text)

    def unavailable(self):
        stamp = rospy.Time.now()
        obj = ObjectPose()
        obj.header.stamp = stamp
        obj.header.frame_id = 'base_link'
        obj.label = 'unknown_small' if self.policy.mode == 'unknown' else 'carton'
        mask = Image()
        mask.header.stamp = stamp
        mask.header.frame_id = str(rospy.get_param('/camera/frame_id', 'camera_link'))
        mask.height = int(rospy.get_param('/camera/height', 480))
        mask.width = int(rospy.get_param('/camera/width', 640))
        mask.encoding = 'mono8'
        mask.step = mask.width
        mask.data = bytes(mask.height * mask.width)
        self.target_receipt = 0.
        self.mask_pub.publish(mask)
        self.obj_pub.publish(obj)
        self.raw_pub.publish(Bool(data=False))
        self.detected_pub.publish(Bool(data=False))

    def on_frame(self, source, obj, mask):
        with self.lock:
            stamp = obj.header.stamp.to_nsec()
            if not self.policy.admit_frame(source, stamp) or stamp <= self.last_source_stamp:
                return
            if mask.header.stamp.to_nsec() != stamp or mask.encoding != 'mono8':
                return
            if (mask.width != int(rospy.get_param('/camera/width', 640))
                    or mask.height != int(rospy.get_param('/camera/height', 480))):
                return
            age = (rospy.Time.now() - obj.header.stamp).to_sec()
            if not 0 <= age <= 5.:
                return
            self.last_source_stamp = stamp
            self.mask_pub.publish(mask)
            self.obj_pub.publish(obj)
            self.detected_pub.publish(Bool(data=bool(obj.detected)))
            self.raw_pub.publish(Bool(data=bool(obj.detected)))
            if obj.detected:
                self.cam_pub.publish(obj.pose_camera)
                self.base_pub.publish(obj.pose_base)
                self.target_receipt = time.monotonic()
                self.emit('ready')
            else:
                self.target_receipt = 0.
                self.emit('waiting_for_target')

    def on_status(self, source, msg):
        with self.lock:
            self.source_status[source] = msg.data
            if self.policy.accepting and source == self.policy.mode:
                if source == 'unknown':
                    # Keep the original GUI's state:choice:detail protocol.
                    # Full structured evidence stays on the independent topic.
                    try:
                        evidence = json.loads(msg.data)
                        state = 'ready' if evidence.get('state') == 'ready' else 'loading'
                        self.detector_pub.publish(String(data=state + ':未知小物体'))
                    except (TypeError, ValueError):
                        self.detector_pub.publish(String(data='error:未知小物体:状态消息无效'))
                else:
                    self.detector_pub.publish(msg)

    def on_state(self, msg):
        with self.lock:
            self.policy.active = bool(msg.active)
            self.policy.state_known = True
            if (self.call_started_ns and not msg.active
                    and msg.header.stamp.to_nsec() >= self.call_started_ns
                    and str(msg.message).startswith('execution slot released:')):
                self.completion_seen = True
                if self.service_uncertain:
                    self.policy.reserved = False
                    self.service_uncertain = False
                    self.emit('task_returned', 'completion observed after service transport failure')

    def on_plan(self, source, msg):
        with self.lock:
            expected = ('unknown_tabletop' if self.policy.mode == 'unknown'
                        else str(rospy.get_param('/perception/yolo_model_choice', 'carton_segment')))
            self.policy.observe_plan(source, bool(msg.valid), msg.plan_id,
                msg.header.stamp.to_nsec(), msg.model_choice, expected)

    def switch(self, mode, strategy=None):
        # Caller holds the lock; worker never overlaps a reserved start.
        outcome = self.policy.begin_switch(mode, rospy.Time.now().to_nsec(), strategy)
        if outcome == 'SWITCHING':
            self.target_receipt = 0.
            self.last_source_stamp = 0
            self.unavailable()
            self.emit('switching')
            threading.Thread(target=self.apply_switch, daemon=True).start()
        elif outcome == 'QUEUED':
            self.emit('switch_queued')
        return outcome

    def set_unknown(self, request):
        with self.lock:
            outcome = self.switch('unknown' if request.data else 'carton')
            return SetBoolResponse(outcome in ('SWITCHING', 'QUEUED'), outcome)

    def select(self, request):
        with self.lock:
            try:
                outcome = self.switch(str(request.mode), str(request.strategy))
            except ValueError as exc:
                return SetGraspModeResponse(False, str(exc))
            return SetGraspModeResponse(outcome in ('SWITCHING', 'QUEUED'), outcome)

    def apply_switch(self):
        try:
            rospy.wait_for_service('/grasp_6d/request_plan', timeout=30.)
            rospy.wait_for_service('/grasp_mode/reset_remote', timeout=30.)
            rospy.wait_for_service('/grasp_mode/reset_task', timeout=30.)
            response = self.request_plan(False)
            if not response.success:
                raise RuntimeError(response.message)
            with self.lock:
                selection = self.policy.selection()
                # ROS XML-RPC integers are signed int32; preserve ns exactly
                # as a decimal string on the parameter server.
                parameter = dict(selection, stamp_ns=str(selection['stamp_ns']))
                rospy.set_param('/grasp_mode/selection', parameter)
            for name, reset in (('remote', self.reset_remote), ('task', self.reset_task)):
                response = reset()
                if not response.success:
                    raise RuntimeError(name + ': ' + response.message)
                ack = json.loads(response.message)
                ack['stamp_ns'] = int(ack.get('stamp_ns', -1))
                if not ack.get('ready') or any(ack.get(k) != v for k, v in selection.items()):
                    raise RuntimeError(name + ' reset acknowledgement mismatch')
            with self.lock:
                self.policy.finish_switch(True)
                rospy.set_param('/grasp_mode/current', self.policy.mode)
                rospy.set_param('/grasp_mode/strategy', self.policy.strategy)
                self.selection_pub.publish(String(data=json.dumps(selection)))
                self.detector_pub.publish(String(data='loading:' + self.policy.mode))
                self.emit('waiting_for_target')
        except Exception as exc:
            with self.lock:
                self.policy.finish_switch(False)
                self.emit('error', str(exc))
                self.detector_pub.publish(String(data='error:grasp_mode:' + str(exc)))
            rospy.logerr('mode switch failed: %s', exc)

    def start(self, request):
        if not request.execute:
            return StartGraspResponse(False, 'execute=false')
        with self.lock:
            plan_id = str(request.plan_id).strip()
            if not plan_id:
                # GUI compatibility: bind its current enriched execution plan,
                # never a preview chosen silently from a previous mode.
                current = self.policy.plans.get('/grasp_6d/plan_enriched')
                plan_id = current[0] if current else ''
            fresh = self.target_receipt > 0 and time.monotonic() - self.target_receipt < 5.
            error = self.policy.reserve_start(plan_id, fresh)
            if error:
                return StartGraspResponse(False, error)
            self.call_started_ns = rospy.Time.now().to_nsec()
            self.service_uncertain = False
            self.completion_seen = False
            self.emit('executing', plan_id)
        returned = False
        try:
            response = self.start_proxy(True, plan_id)
            returned = True
            return response
        except Exception as exc:
            with self.lock:
                self.service_uncertain = True
                self.emit('execution_response_unknown', str(exc))
            return StartGraspResponse(False, str(exc))
        finally:
            with self.lock:
                if returned or self.completion_seen:
                    self.policy.reserved = False
                    self.service_uncertain = False
                    self.emit('task_returned')

    def tick(self, _event):
        with self.lock:
            if self.desired and self.policy.state_known:
                mode, self.desired = self.desired, ''
                self.switch(mode, self.desired_strategy)
            elif self.policy.pending and not (self.policy.active or self.policy.reserved or self.policy.switching):
                self.switch(self.policy.pending, self.policy.pending_strategy)
            if self.policy.accepting and not self.policy.reserved and not self.policy.active:
                if self.target_receipt and time.monotonic() - self.target_receipt > 5.:
                    self.unavailable()
                    self.emit('waiting_for_target', 'source timeout')
            if self.last_status:
                self.status_pub.publish(String(data=self.last_status))


if __name__ == '__main__':
    rospy.init_node('grasp_mode_router')
    ModeRouter()
    rospy.spin()
