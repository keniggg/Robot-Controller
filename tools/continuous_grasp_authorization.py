"""One invocation under recorded continuing authorization; never an auto-loop.

The operator supplies a new observation for every invocation. File control can
pause/revoke unreleased and future attempts; it is not an in-flight robot stop.
"""
from contextlib import AbstractContextManager
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import time
import uuid

from runtime_evidence import ReleaseEvidence, normalize_sha256
from experiment_storage import select as select_storage

REQUEST_TEXT = '你一直进行处理，直到成功抓取'


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    try:
        with temporary.open('x') as stream:
            stream.write(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary), str(path))
        descriptor = os.open(str(path.parent), os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
    finally:
        if temporary.exists():
            temporary.unlink()


def set_control(authorization, state):
    """Explicit operator control only. Revocation/completion cannot be resumed."""
    require(state in ('active', 'paused', 'revoked', 'completed'), 'CONTROL_STATE_INVALID')
    authorization = Path(authorization)
    auth = read(authorization)
    with (authorization.parent / 'control.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        path = authorization.parent / 'control.json'
        old = read(path)
        require(old['authorization_id'] == auth['authorization_id'], 'CONTROL_ID_CHANGED')
        require(old['state'] not in ('revoked', 'completed') or state == old['state'], 'CONTROL_TERMINAL')
        atomic_json(path, dict(old, state=state, revision=old['revision'] + 1))


class ContinuationAttempt(AbstractContextManager):
    def __init__(self, authorization, record, session, user_message, now_ns, *, storage=None):
        self.path = Path(authorization).resolve()
        self.record, self.session = Path(record).resolve(), Path(session).resolve()
        self.selected_storage = select_storage(path=self.record) if storage is None else None
        if self.selected_storage is not None:
            self.selected_storage.validate(self.record, subtree=self.selected_storage.archive_root)
        self.storage, self.lock = (self.selected_storage.base if storage is None else Path(storage).resolve()), None
        auth_bytes = self.path.read_bytes()
        self.auth_sha = hashlib.sha256(auth_bytes).hexdigest()
        self.auth = json.loads(auth_bytes)
        # Current boundary inputs stay separate from historical snapshots.
        # Additional source/config pins belong here and are never cached.
        self.pinned = {self.path: self.auth_sha}
        self._evidence_transaction = None
        for path in (self.path, self.record, self.session):
            require(self.storage in path.parents, 'CONTINUATION_OUTSIDE_STORAGE')
        auth = self.auth
        require(auth['version'] == 1 and auth['scope'] == 'observe_return_until_physical_grasp_success', 'AUTH_SCOPE_INVALID')
        source = auth['confirmation_source']
        require(source['source'] == 'actual_user_message' and source['text'] == REQUEST_TEXT
                and user_message == source['text'], 'ACTUAL_CONTINUING_REQUEST_REQUIRED')
        require(source.get('platform_message_id') is None and source.get('local_record_id')
                and source.get('received_at')
                and source.get('received_at_kind') == 'locally_recorded_receipt_time',
                'CONTINUING_RECEIPT_PROVENANCE_INVALID')
        original_hands = auth['original_hands_clear_receipt']
        self._historical_evidence = ReleaseEvidence(
            {original_hands['path']: original_hands['sha256']},
            current_paths=(self.path, self.path.parent / 'control.json',
                           self.session / 'operator_observation.json'))
        self.hands = self._evidence(original_hands, historical=True)
        require(self.hands['user_message'] in (
            '当前已对准且手已移开，允许执行一轮', '已对准且手已移开，允许执行一轮',
            '双手已移到机械臂活动范围之外，可以启动', '已对准，手已移开，可以继续自动抓取')
            and self.hands.get('message_id'), 'HANDS_CLEAR_RECEIPT_REQUIRED')
        observation_path = self.session / 'operator_observation.json'
        observation_bytes = observation_path.read_bytes()
        self.observation = json.loads(observation_bytes)
        self.pinned[observation_path] = hashlib.sha256(observation_bytes).hexdigest()
        observation = self.observation
        require(observation['version'] == 1 and observation['new_user_confirmation_claimed'] is False,
                'NEW_USER_CONFIRMATION_MUST_NOT_BE_CLAIMED')
        require(observation['same_target_observed'] is True and observation['target_visible'] is True,
                'CURRENT_SAME_TARGET_REVIEW_REQUIRED')
        hands = observation['hands_clear_basis']
        require(hands['prior_explicit_user_reply_id'] == self.hands['message_id']
                and hands['current_image_review_no_conflicting_hand_in_motion_area'] is True,
                'CURRENT_HANDS_CLEAR_BASIS_REQUIRED')
        self._evidence(observation['image'], as_json=False)
        self.preflight = self._evidence(observation['preflight'])
        self.readiness = self._evidence(observation['readiness'])
        require(Path(observation['preflight']['path']).resolve() == self.session / 'preflight.json', 'PREFLIGHT_PATH_MISMATCH')
        ready = self.readiness
        require(ready['passed'] is True and ready['deployment_processes_unchanged'] is True
                and ready['source_matches_operational_release'] is True
                and ready['motion_commands_sent'] is False and ready['temperature_queried'] is False,
                'CURRENT_READINESS_REQUIRED')
        require(ready['sample_count'] >= 20 and ready['epoch_ns'] == self.preflight['epoch_ns'], 'READINESS_EPOCH_INVALID')
        require(ready['free_bytes'] >= ready['min_free_bytes'], 'RECORDING_SPACE_INSUFFICIENT')
        require(self.preflight['receipt_ns'] == observation['captured_ns']
                and auth['issued_ns'] <= observation['captured_ns'] <= ready['recorded_ns']
                <= observation['assessed_ns'] <= now_ns, 'CURRENT_CAPTURE_ORDER_INVALID')
        require(observation['image']['source_stamp_ns'] == self.preflight['image_stamp_ns']
                and 0 <= observation['captured_ns'] - self.preflight['image_stamp_ns'] <= 800_000_000,
                'CAPTURE_IMAGE_NOT_FRESH')
        q = self.preflight['topics']['/alicia_d/accepted_joint_states']
        require(q['frame_id'] == 'sdk_measured' and 0 <= observation['captured_ns'] - q['stamp_ns'] <= 200_000_000,
                'CAPTURE_SDK_NOT_FRESH')
        require(len(q['names']) == len(q['positions']) and len(set(q['names'])) == len(q['names'])
                and all(math.isfinite(value) for value in q['positions'])
                and set('Joint%d' % i for i in range(1, 7)) <= set(q['names']), 'CAPTURE_JOINTS_INVALID')
        self.check_active(now_ns)

    def _history_transaction(self):
        # close() ends this snapshot. A later reuse starts a new transaction
        # and rehashes historical bytes; it cannot keep a prior cache alive.
        if self._evidence_transaction is None:
            self._evidence_transaction = self._historical_evidence.transaction()
        return self._evidence_transaction

    def _evidence(self, reference, as_json=True, *, historical=False):
        path = Path(reference['path']).resolve()
        require(self.storage in path.parents, 'EVIDENCE_HASH_MISMATCH')
        if historical:
            require(self._historical_evidence.historical_sha256.get(path) == reference['sha256'],
                    'HISTORICAL_EVIDENCE_NOT_DECLARED')
            transaction = self._history_transaction()
            if as_json:
                return transaction.read_json(path)
            transaction.read_bytes(path)
            return None
        else:
            require(path not in self._historical_evidence.historical_sha256,
                    'CURRENT_INPUT_CANNOT_BE_CACHED')
            pins = normalize_sha256(self.pinned, {path: reference['sha256']})
            raw = path.read_bytes()
            require(hashlib.sha256(raw).hexdigest() == reference['sha256'], 'EVIDENCE_HASH_MISMATCH')
            self.pinned = pins
        return json.loads(raw) if as_json else None

    def check_active(self, now_ns):
        if self.selected_storage is not None:
            self.selected_storage.validate(self.record, subtree=self.selected_storage.archive_root)
        current = normalize_sha256(self.pinned)
        require(not set(current).intersection(self._historical_evidence.historical_sha256),
                'CURRENT_INPUT_CANNOT_BE_CACHED')
        require(all(digest(path) == expected for path, expected in current.items()), 'PINNED_EVIDENCE_CHANGED')
        self._history_transaction().verify_all()
        control = read(self.path.parent / 'control.json')
        require(control['version'] == 1 and control['authorization_id'] == self.auth['authorization_id']
                and control['state'] == 'active', 'CONTINUATION_PAUSED_REVOKED_OR_COMPLETED')
        require(self.observation['assessed_ns'] <= now_ns < self.observation['valid_until_ns'], 'OPERATOR_OBSERVATION_EXPIRED')
        return control

    def __enter__(self):
        self.lock = (self.path.parent / 'attempt.lock').open('a')
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            ledger = self.path.parent / 'ledger'
            ledger.mkdir(exist_ok=True)
            for path in ledger.glob('*.json'):
                previous = read(path)
                require(previous['observation_id'] != self.observation['observation_id']
                        and previous['captured_ns'] < self.observation['captured_ns'], 'OBSERVATION_ALREADY_CONSUMED')
                previous_record = Path(previous['record_dir'])
                manifest = read(previous_record / 'manifest.json')
                outcome = read(previous_record / 'recorder_outcome.json')
                require(manifest['state'] == 'complete' and manifest['recorder_closed'] is True
                        and outcome['bag_closed'] is True, 'PREVIOUS_RECORD_NOT_CLOSED')
            require(not (self.record / 'authorization.json').exists(), 'RECORD_ALREADY_AUTHORIZED')
            self.attempt = dict(version=1, authorization_id=self.auth['authorization_id'],
                authorization_sha256=self.auth_sha, attempt_id=uuid.uuid4().hex,
                record_dir=str(self.record), session_dir=str(self.session),
                observation_id=self.observation['observation_id'], captured_ns=self.observation['captured_ns'],
                consumed=True, status='claimed', physical_success='unverified')
            require(not (self.record / 'continuation_attempt_claim.json').exists(), 'RECORD_ALREADY_CLAIMED')
            self.ledger_path = ledger / (self.attempt['attempt_id'] + '.json')
            # Consume durably before the record claim. Failure between these
            # writes remains consumed and cannot release any runner.
            atomic_json(self.ledger_path, self.attempt)
            # The record claim is never removed, including on early failure.
            with (self.record / 'continuation_attempt_claim.json').open('x') as stream:
                json.dump(self.attempt, stream, ensure_ascii=False)
                stream.flush()
                os.fsync(stream.fileno())
            return self
        except BaseException:
            self.close()
            raise

    def event(self, status, **values):
        self.attempt.update(status=status, **values)
        atomic_json(self.ledger_path, self.attempt)

    def commit_release(self, runtime, selection, commit_status, now_ns):
        """Linearization point for this attempt, immediately before pipe release.

        Control updates winning this short lock first prevent commitment. A
        pause after commitment affects future attempts; it is not cancellation
        of the committed attempt, even if its pipe write has not occurred yet.
        """
        with (self.path.parent / 'control.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            require('release_commit' not in self.attempt, 'RELEASE_ALREADY_COMMITTED')
            runtime.check(selection, commit_status=commit_status)
            control = read(self.path.parent / 'control.json')
            require(control['state'] == 'active', 'CONTINUATION_PAUSED_REVOKED_OR_COMPLETED')
            commit = dict(selection=selection, committed_ns=now_ns,
                control_revision=control['revision'],
                policy='final_checked_authorization_commit_before_pipe_release',
                pause_after_commit_cancels_this_attempt=False,
                physical_action_submitted=False)
            self.event('authorization_committed_for_runner_release', release_commit=commit)
            return commit

    def authorization_fields(self):
        return dict(confirmation_source=dict(self.auth['confirmation_source'],
            kind='continuing_user_authorization', user_alignment_claimed=False),
            continuing_authorization=dict(path=str(self.path), sha256=self.auth_sha,
                authorization_id=self.auth['authorization_id'], attempt_id=self.attempt['attempt_id'],
                original_hands_clear_receipt=self.auth['original_hands_clear_receipt']),
            operator_observation=self.observation, new_user_confirmation_claimed=False,
            target_basis='Fresh operator-reviewed same-target image and SDK/readiness evidence under continuing user authorization')

    def close(self):
        if self._evidence_transaction is not None:
            self._evidence_transaction.close()
            self._evidence_transaction = None
        if self.lock is not None:
            self.lock.close()
            self.lock = None

    def __exit__(self, kind, value, traceback):
        try:
            if kind is not None and hasattr(self, 'ledger_path'):
                self.event('launcher_error', error=str(value), authorization_not_refunded=True)
        finally:
            self.close()


class RuntimeGuard(AbstractContextManager):
    """Read-only subscriptions keep final release checks non-blocking."""
    def __init__(self, rospy, attempt):
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Header, String
        from actionlib_msgs.msg import GoalStatusArray
        from alicia_flexible_grasp_supervisor.msg import GraspState
        self.rospy, self.attempt = rospy, attempt
        self.lock, self.latest, self.received = threading.Lock(), {}, {}
        self.refresh_topics = []
        self.subscriptions = []
        for key, topic, kind in (
            ('q', '/alicia_d/accepted_joint_states', JointState),
            ('epoch', '/alicia_d/control_reference_epoch', Header),
            ('task', '/grasp/state', GraspState), ('mode', '/grasp_mode/status', String),
            ('arm', '/alicia_controller/follow_joint_trajectory/status', GoalStatusArray),
            ('hand', '/hand_controller/follow_joint_trajectory/status', GoalStatusArray)):
            if key in ('task', 'arm', 'hand'):
                self.refresh_topics.append((key, topic, kind))
            self.subscriptions.append(rospy.Subscriber(topic, kind,
                lambda message, name=key: self.receive(name, message), queue_size=4))

    def receive(self, key, message):
        with self.lock:
            self.latest[key] = message
            self.received[key] = time.monotonic()

    def refresh_idle(self):
        # Task state is latched and changes on transitions, not a heartbeat.
        # Request its current latch again before selection. Actionlib normally
        # publishes status at 5 Hz; require a recent source stamp as well.
        deadline = time.monotonic() + 5.
        for key, topic, kind in self.refresh_topics:
            remaining = deadline - time.monotonic()
            require(remaining > 0., 'CONTINUATION_IDLE_REFRESH_TIMEOUT')
            self.receive(key, self.rospy.wait_for_message(topic, kind, timeout=remaining))

    def wait_ready(self):
        deadline = time.monotonic() + 5.
        while time.monotonic() < deadline and not self.rospy.is_shutdown():
            with self.lock:
                if len(self.latest) == 6:
                    return
            time.sleep(.01)
        raise ValueError('CONTINUATION_LIVE_READINESS_INCOMPLETE')

    def check(self, selection, *, commit_status=None):
        from alicia_grasp_modes.selection import parse_selection
        with self.lock:
            state = dict(self.latest)
            received = dict(self.received)
        require(len(state) == 6 and not self.rospy.is_shutdown(), 'CONTINUATION_LIVE_READINESS_INCOMPLETE')
        now = self.rospy.Time.now().to_nsec()
        wall = time.monotonic()
        require(all(0. <= wall-received.get(key, -float('inf')) <= 2.
                    for key in ('task', 'arm', 'hand')), 'CONTINUATION_IDLE_CACHE_STALE')
        self.attempt.check_active(now)
        require(parse_selection(self.rospy.get_param('/grasp_mode/selection')) == selection, 'CONTINUATION_SELECTION_CHANGED')
        require(not state['task'].active, 'CONTINUATION_TASK_ACTIVE')
        # The router's exact completed transaction is stronger than a lagging
        # second subscriber cache. It is supplied only by the existing ACK wait.
        mode = commit_status if commit_status is not None else json.loads(state['mode'].data)
        if commit_status is not None:
            require(mode.get('state') in ('ready', 'waiting_for_target'), 'CONTINUATION_COMMIT_NOT_READY')
        require(parse_selection(mode) == selection and not any(mode.get(k) for k in ('active', 'start_reserved', 'pending')),
                'CONTINUATION_ROUTER_BUSY')
        for key in ('arm', 'hand'):
            require(0 <= now-state[key].header.stamp.to_nsec() <= 2_000_000_000,
                    'CONTINUATION_ACTION_HEARTBEAT_STALE')
            require(not any(goal.status in (0, 1, 6, 7) for goal in state[key].status_list), 'CONTINUATION_ACTION_ACTIVE')
        q = state['q']
        require(q.header.frame_id == 'sdk_measured' and 0 <= now-q.header.stamp.to_nsec() <= 200_000_000,
                'CONTINUATION_SDK_STALE')
        require(state['epoch'].stamp.to_nsec() == self.attempt.preflight['epoch_ns'], 'CONTINUATION_EPOCH_CHANGED')
        before = self.attempt.preflight['topics']['/alicia_d/accepted_joint_states']
        actual = dict(zip(q.name, q.position))
        require(all(name in actual and math.isfinite(actual[name]) and abs(actual[name]-value) <= .02
                    for name, value in zip(before['names'], before['positions'])), 'CONTINUATION_POSE_CHANGED')

    def __exit__(self, *_args):
        for subscription in self.subscriptions:
            subscription.unregister()
