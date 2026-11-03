"""Pure post-trajectory empty-LEGACY readiness, with exact transport history.

No ROS import, file access, QUERY, mutation, reference reset or motion exists
here. The adapter binds archive bytes and retained process identities. Historical
bag connections can omit callerid; that absence is reported, never invented.
The original startup validator remains unchanged and requires sequence zero.
"""
from dataclasses import dataclass, asdict
from types import SimpleNamespace
import math
import time

GOAL_TOPIC = '/alicia_controller/follow_joint_trajectory/goal'
RESULT_TOPIC = '/alicia_controller/follow_joint_trajectory/result'
DRIVER_TOPIC = '/alicia_d/physical_reference_state'
HW_TOPIC = '/bessica_d_hw_interface/physical_reference_state'
ACTION_TYPES = {
    GOAL_TOPIC: ('control_msgs/FollowJointTrajectoryActionGoal', 'cff5c1d533bf2f82dd0138d57f4304bb', '/motion_gateway'),
    RESULT_TOPIC: ('control_msgs/FollowJointTrajectoryActionResult', 'c4fb3b000dc9da4fd99699380efcc5d9', '/bessica_d_hw_interface'),
}
EMPTY_FIELDS = ('binding_id', 'previous_binding_id', 'owner_token', 'goal_token',
    'episode_id', 'action_goal_id', 'plan_sha256', 'model_sha256', 'proof_sha256',
    'physical_trajectory_sha256', 'wire_trajectory_sha256')
ZERO_FIELDS = ('action_goal_stamp_ns', 'action_transport_sequence', 'selection_generation',
    'request_sequence', 'last_command_sequence', 'prepared_ns', 'deadline_ns',
    'episode_deadline_ns', 'monotonic_remaining_sec', 'outer_monotonic_remaining_sec',
    'stop_duration_sec', 'goal_status', 'goal_error_code')


def require(condition, reason):
    if not condition:
        raise RuntimeError(reason)


@dataclass(frozen=True)
class TransportHistory:
    action_transport_sequence: int
    goal_id: str
    goal_stamp_ns: int
    terminal_stamp_ns: int
    record_epoch_ns: int
    recorded_goal_count: int
    connection_callerids_available: bool


def _bound_history(history):
    require(type(history) is TransportHistory
            and type(history.action_transport_sequence) is int
            and 0 <= history.action_transport_sequence <= 0xffffffff
            and type(history.goal_id) is str and 0 < len(history.goal_id) <= 1024
            and all(type(v) is int for v in (history.record_epoch_ns, history.goal_stamp_ns,
                                             history.terminal_stamp_ns, history.recorded_goal_count))
            and 0 < history.record_epoch_ns <= history.goal_stamp_ns <= history.terminal_stamp_ns
            and history.recorded_goal_count > 0
            and type(history.connection_callerids_available) is bool,
            'validated successful action transport history required')
    return history


def _empty_legacy_message(message):
    require(message['phase'] == 0 and message['code'] == 'LEGACY'
            and all(message[key] == '' for key in EMPTY_FIELDS)
            and all(message[key] == 0 for key in ZERO_FIELDS if key != 'action_transport_sequence')
            and len(message['bias_rad']) == 6 and all(v == 0. for v in message['bias_rad'])
            and all(message[key] is False for key in ('goal_observed', 'goal_terminal', 'output_gated')),
            'history contains a typed lease or nonempty LEGACY state')
    sequence = message['action_transport_sequence']
    require(type(sequence) is int and 0 <= sequence <= 0xffffffff,
            'invalid historical action transport sequence')


def validate_transport_history(history, actions):
    """Validate the actual closed-bag extraction shapes, without reading a bag.

    The sequence is the last observed goal header.seq, not a goal count. Every
    recorded goal must have its matching successful terminal; both endpoints'
    empty-LEGACY history must finish with that exact sequence. File SHA, bag
    completeness and process/graph provenance remain the adapter's obligation.
    """
    require(type(history) is dict and type(actions) is dict,
            'typed and action history mappings required')
    require(history.get('source_record_unchanged') is True and actions.get('source_record_unchanged') is True,
            'sealed historical sources required')
    require(all(history.get(key) == actions.get(key) and history.get(key) is not None
                for key in ('record', 'manifest_sha256', 'bag_sha256', 'bag_identity')),
            'action and typed histories refer to different recordings')
    records = actions['records']
    require(type(records) is list and bool(records), 'complete action records required')
    require(set(actions['topics']) == set(ACTION_TYPES), 'action topic scope differs')
    counts = {topic: sum(row['topic'] == topic for row in records) for topic in ACTION_TYPES}
    require(counts == actions['message_counts'] and counts[GOAL_TOPIC] > 0,
            'action extraction inventory differs')
    require(all(row['topic'] in ACTION_TYPES for row in records), 'unexpected action topic')
    callerids = True
    for row in records:
        topic = row['topic']; connection = row['connection']; kind, md5, owner = ACTION_TYPES[topic]
        require(connection.get('topic') == topic and connection.get('type') == kind
                and connection.get('md5sum') == md5, 'historical action transport type differs')
        if 'callerid' in connection:
            require(connection['callerid'] == owner, 'historical action caller differs')
        else:
            callerids = False
        require(type(row['bag_ns']) is int and row['bag_ns'] > 0,
                'historical action recording stamp invalid')
    goals = sorted((r for r in records if r['topic'] == GOAL_TOPIC), key=lambda r:r['bag_ns'])
    terminals = [r for r in records if r['topic'] == RESULT_TOPIC]
    require(len(goals) == len(terminals), 'unmatched historical action terminal')
    identities = []
    for row in goals:
        goal = row['message']; ident = goal['goal_id']; sequence = goal['header']['seq']
        require(type(ident['id']) is str and 0 < len(ident['id']) <= 1024
                and type(ident['stamp']) is int and 0 < ident['stamp'] <= row['bag_ns']
                and type(sequence) is int and 0 <= sequence <= 0xffffffff,
                'historical action goal identity invalid')
        identity = (ident['id'], ident['stamp']); identities.append(identity)
        matches = [r for r in terminals if r['message']['status']['goal_id'] == ident]
        require(len(matches) == 1, 'historical GoalID lacks one exact terminal')
        terminal = matches[0]; message = terminal['message']
        require(message['status']['status'] == 3 and message['result']['error_code'] == 0
                and type(message['header']['stamp']) is int
                and ident['stamp'] <= message['header']['stamp'] <= terminal['bag_ns']
                and row['bag_ns'] <= terminal['bag_ns'], 'historical action did not complete successfully')
    require(len(identities) == len(set(identities)), 'historical action GoalID replay')
    last = goals[-1]; goal = last['message']; sequence = goal['header']['seq']
    terminal = next(r for r in terminals if r['message']['status']['goal_id'] == goal['goal_id'])
    allowed_sequences = {0} | {r['message']['header']['seq'] for r in goals}
    epochs = set()
    for topic in (DRIVER_TOPIC, HW_TOPIC):
        first = history['first_messages'][topic]['message']
        final = history['last_messages'][topic]
        _empty_legacy_message(first); _empty_legacy_message(final['message'])
        require(first['action_transport_sequence'] == 0
                and final['message']['action_transport_sequence'] == sequence
                and final['bag_ns'] >= terminal['bag_ns'], 'endpoint sequence does not close the recorded goal history')
        for transition in history['typed_transitions'][topic]:
            for edge in ('first', 'last'):
                message = transition[edge]['message']; _empty_legacy_message(message)
                require(message['action_transport_sequence'] in allowed_sequences,
                        'typed history contains an unexplained action sequence')
                epochs.add(message['epoch_ns'])
        epochs.update((first['epoch_ns'], final['message']['epoch_ns']))
    require(len(epochs) == 1, 'historical action epoch changed')
    result = TransportHistory(sequence, goal['goal_id']['id'], goal['goal_id']['stamp'],
        terminal['message']['header']['stamp'], epochs.pop(), len(goals), callerids)
    return _bound_history(result)


def _startup_contract(startup):
    require(tuple(startup.EMPTY_FIELDS) == EMPTY_FIELDS and tuple(startup.ZERO_FIELDS) == ZERO_FIELDS,
            'original startup field contract changed')


def validate_state(state, epoch, now, rows, assets, history, owner=None, *, startup):
    _startup_contract(startup); history = _bound_history(history)
    fresh, decoded, ARM = startup.fresh, startup.decoded, startup.ARM
    require(epoch >= history.record_epoch_ns and now >= history.terminal_stamp_ns,
            "current evidence predates the recorded action history")
    if owner is not None:
        header = state._connection_header
        require(header.get('callerid') == owner and header.get('latching') == '0'
                and header.get('md5sum') == assets['type_md5']['alicia_d_driver/PhysicalReferenceState'],
                'fresh state transport owner/latch/MD5 invalid')
    require(state.header.frame_id == 'physical_reference_v1' and fresh(state.header.stamp.to_nsec(), now, epoch), 'typed state stale')
    require(state.phase == 0 and state.epoch_ns == epoch and all(getattr(state, key) == '' for key in EMPTY_FIELDS)
            and state.code == 'LEGACY'
            and all(getattr(state, key) == 0 for key in ZERO_FIELDS if key != 'action_transport_sequence')
            and type(state.action_transport_sequence) is int
            and state.action_transport_sequence == history.action_transport_sequence,
            'post-trajectory typed state is not the bound empty LEGACY')
    require(len(state.bias_rad) == 6 and all(x == 0. for x in state.bias_rad)
            and state.goal_observed is False and state.goal_terminal is False and state.output_gated is False,
            'initial typed goal/map state differs')
    require(all(getattr(state, name) is True for name in ('evidence_valid', 'pending_complete', 'stationary', 'manual_reference')),
            'typed_not_ready: fresh confirmed pending/stationary/manual evidence absent')
    for key, field in (('sdk', 'sdk_stamp_ns'), ('encoder', 'encoder_stamp_ns'), ('controller', 'controller_stamp_ns')):
        stamp = getattr(state, field)
        require(fresh(stamp, now, epoch) and any(m.header.stamp.to_nsec() == stamp for m in rows[key]), 'typed source stamp is stale/unseen: '+key)
    counts = list(state.sdk_counts); expected = decoded(counts)
    require(all(math.isfinite(v) and abs(v-q) <= 1e-12 for v, q in zip(state.wire_positions_rad, expected))
            and len(state.wire_positions_rad) == 6, 'typed wire words/decoded values differ')
    controller = next(m for m in rows['controller'] if m.header.stamp.to_nsec() == state.controller_stamp_ns)
    desired = controller.desired
    require(len(controller.joint_names) == len(set(controller.joint_names)) == 6 and set(controller.joint_names) == set(ARM), 'JTC joint layout differs')
    require(all(len(getattr(desired, key)) == 6 for key in ('positions', 'velocities', 'accelerations')), 'JTC desired vectors missing')
    ordered = [desired.positions[list(controller.joint_names).index(name)] for name in ARM]
    require(len(state.physical_reference_rad) == 6 and all(math.isfinite(v) and abs(v-q) <= 1e-12 for v, q in zip(state.physical_reference_rad, ordered)),
            'LEGACY physical_reference_rad does not match actual JTC desired')
    require(all(math.isfinite(v) and v == 0. for values in (desired.velocities, desired.accelerations) for v in values), 'JTC is not zero velocity/acceleration')
    return dict(phase=state.phase, code=state.code, epoch_ns=epoch, sdk_counts=counts,
                state_stamp_ns=state.header.stamp.to_nsec(), sdk_stamp_ns=state.sdk_stamp_ns,
                encoder_stamp_ns=state.encoder_stamp_ns, controller_stamp_ns=state.controller_stamp_ns,
                request_sequence=state.request_sequence, last_command_sequence=state.last_command_sequence,
                action_transport_sequence=state.action_transport_sequence,
                manual_reference=True, evidence_valid=True, pending_complete=True, stationary=True,
                physical_reference_rad=list(state.physical_reference_rad), wire_positions_rad=list(state.wire_positions_rad))


def observed_ready(observer, epoch, rospy, assets, history, query_states=None, *, startup):
    _startup_contract(startup); history = _bound_history(history)
    fresh, words, positions = startup.fresh, startup.words, startup.positions
    DRIVER, HW = startup.DRIVER, startup.HW
    rows = observer.snapshot(); require(all(rows.values()), 'typed/raw telemetry pending')
    now = rospy.Time.now().to_nsec(); require(epoch > 0, 'positive current epoch required')
    for key in ('sdk', 'encoder', 'reference', 'controller'):
        message = rows[key][-1]; owner = HW if key == 'controller' else DRIVER
        require(message._connection_header.get('callerid') == owner and fresh(message.header.stamp.to_nsec(), now, epoch), 'raw source owner/stamp differs: '+key)
    sdk, encoder, reference = (rows[key][-1] for key in ('sdk', 'encoder', 'reference'))
    require(sdk.header.frame_id == 'sdk_transmitted' and encoder.header.frame_id == 'sdk_measured', 'real SDK frame kinds differ')
    counts = words(positions(sdk))
    require(reference._connection_header.get('latching') == '0' and reference.header.frame_id == 'sdk_manual'
            and reference.header.stamp == sdk.header.stamp and words(positions(reference)) == counts,
            'newest actual control_reference is not paired sdk_manual')
    windows = {}
    for key in ('sdk', 'encoder'):
        last = rows[key][-1].header.stamp.to_nsec()
        window = [m for m in rows[key] if last-500000000 <= m.header.stamp.to_nsec() <= last]
        stamps = [m.header.stamp.to_nsec() for m in window]
        require(len(window) >= 3 and stamps[-1]-stamps[0] >= 300000000
                and all(a < b and b-a <= 250000000 for a, b in zip(stamps, stamps[1:])), 'stationary source window incomplete: '+key)
        require(all(m._connection_header.get('callerid') == DRIVER and m.header.frame_id == ('sdk_transmitted' if key == 'sdk' else 'sdk_measured') for m in window), 'stationary window source changed')
        q = [positions(m) for m in window]
        if key == 'sdk':
            require(all(words(row) == counts for row in q), 'successful SDK words changed during stationary window')
        else:
            require(all(max(row[j] for row in q)-min(row[j] for row in q) <= 2*math.pi/4096+1e-12 for j in range(6)), 'real encoder window is not stationary')
        windows[key] = dict(samples=len(window), start_ns=stamps[0], end_ns=stamps[-1])
    states = {}
    for key, owner in (('driver', DRIVER), ('hardware', HW)):
        states[key] = validate_state(rows[key][-1], epoch, now, rows, assets, history, owner, startup=startup)
        require(tuple(states[key]['sdk_counts']) == counts, 'typed endpoint disagrees with successful SDK words')
    queried = {}
    for key, state in (query_states or {}).items():
        queried[key] = validate_state(state, epoch, now, rows, assets, history, startup=startup)
        require(tuple(queried[key]['sdk_counts']) == counts, 'QUERY state differs from successful SDK words')
    return dict(states=states, query_states=queried, source_windows=windows,
                transport_history=asdict(history), initial_startup_claimed=False,
                sdk_counts=list(counts), accepted_positions_rad=positions(encoder),
                newest_reference_frame=reference.header.frame_id,
                newest_reference_stamp_ns=reference.header.stamp.to_nsec(), observed_ns=now)


def wait_for_fixed_query_sources(observer, epoch, rospy, assets, history, query_states, audit, *, startup):
    """Wait only for messages named by the two already returned QUERY states.

    Separate TCPROS connections can deliver a service receipt before the raw
    telemetry it references. Neither QUERY response nor any deadline is renewed.
    Once those exact messages arrive, every original readiness gate runs again.
    """
    _startup_contract(startup); history = _bound_history(history)
    fresh = startup.fresh
    require(set(query_states) == {'driver', 'hardware'}, 'both fixed QUERY receipts required')
    fields = (('sdk', 'sdk_stamp_ns'), ('encoder', 'encoder_stamp_ns'),
              ('controller', 'controller_stamp_ns'))
    fixed = {}
    for key, state in query_states.items():
        require(state.epoch_ns == epoch and epoch > 0, 'QUERY epoch differs before source wait')
        fixed[key] = dict(state_stamp_ns=state.header.stamp.to_nsec(),
                          **{field: getattr(state, field) for _source, field in fields})
    started_ros_ns = rospy.Time.now().to_nsec()
    started_monotonic = time.monotonic()
    stamps = [stamp for row in fixed.values() for stamp in row.values()]
    require(all(fresh(stamp, started_ros_ns, epoch) for stamp in stamps),
            'fixed QUERY receipt/source already stale or invalid; no retry')
    expires_ros_ns = min(stamps)+500000000
    expires_monotonic = started_monotonic+(expires_ros_ns-started_ros_ns)*1e-9
    audit.update(fixed_query_stamps=fixed, epoch_ns=epoch,
                 started_ros_ns=started_ros_ns, expires_ros_ns=expires_ros_ns,
                 original_wait_budget_sec=(expires_ros_ns-started_ros_ns)*1e-9,
                 query_reissued=False, receipt_refreshed=False, snapshots=0)
    try:
        while True:
            now = rospy.Time.now().to_nsec()
            require(started_ros_ns <= now <= expires_ros_ns
                    and time.monotonic() <= expires_monotonic
                    and all(fresh(stamp, now, epoch) for stamp in stamps),
                    'fixed QUERY source wait expired or clock moved backward; no retry')
            rows = observer.snapshot(); audit['snapshots'] += 1
            missing = []
            for key, state in query_states.items():
                require(state.header.stamp.to_nsec() == fixed[key]['state_stamp_ns']
                        and state.epoch_ns == epoch
                        and all(getattr(state, field) == fixed[key][field] for _source, field in fields),
                        'fixed QUERY response changed while awaiting its sources')
                for source, field in fields:
                    if not any(message.header.stamp.to_nsec() == fixed[key][field]
                               for message in rows.get(source, ())):
                        missing.append(dict(endpoint=key, source=source, stamp_ns=fixed[key][field]))
            audit['last_missing'] = missing
            if not missing:
                ready = observed_ready(observer, epoch, rospy, assets, history, query_states, startup=startup)
                finished_ros_ns = rospy.Time.now().to_nsec()
                require(started_ros_ns <= finished_ros_ns <= expires_ros_ns
                        and time.monotonic() <= expires_monotonic,
                        'fixed QUERY evidence expired during complete readiness check')
                audit['completed'] = True
                return ready
            remaining = min(expires_monotonic-time.monotonic(),
                            (expires_ros_ns-now)*1e-9)
            require(remaining > 0., 'fixed QUERY referenced telemetry did not arrive in its original window')
            time.sleep(min(.01, remaining))
    finally:
        audit['waited_sec'] = time.monotonic()-started_monotonic
        audit['finished_ros_ns'] = rospy.Time.now().to_nsec()

@dataclass(frozen=True)
class _Stamp:
    value: int
    def to_nsec(self):
        return self.value


def _json_message(value, key=None):
    if key == 'stamp':
        if type(value) is int:
            return _Stamp(value)
        require(type(value) is dict and set(value) == {'secs', 'nsecs'}
                and type(value['secs']) is int and type(value['nsecs']) is int
                and 0 <= value['nsecs'] < 1000000000, 'recorded ROS stamp invalid')
        return _Stamp(value['secs']*1000000000+value['nsecs'])
    if type(value) is dict:
        return SimpleNamespace(**{k:_json_message(v,k) for k,v in value.items()})
    if type(value) is list:
        return [_json_message(v) for v in value]
    return value


def recorded_rows(snapshot):
    """Lossless values/transport conversion only; no source timestamp repair."""
    def message(row):
        msg = _json_message(row['message']); msg._connection_header = dict(row['transport'])
        return msg
    rows = {key:[message(row) for row in snapshot['raw'][key]]
            for key in ('sdk', 'encoder', 'reference', 'controller')}
    rows.update({key:[message(snapshot['typed'][key])] for key in ('driver', 'hardware')})
    return rows


def validate_recorded_snapshot(snapshot, history, *, startup, assets):
    """Historical diagnosis at its recorded clock; never current readiness."""
    require(type(snapshot['finished_ns']) is int and snapshot['finished_ns'] >= snapshot['started_ns'],
            'recorded observation clock invalid')
    rows = recorded_rows(snapshot)
    observer = SimpleNamespace(snapshot=lambda:rows)
    rospy = SimpleNamespace(Time=SimpleNamespace(now=lambda:_Stamp(snapshot['finished_ns'])))
    result = observed_ready(observer,snapshot['state']['epoch_ns'],rospy,assets,history,startup=startup)
    result.update(historical_diagnostic_only=True,current_readiness_claimed=False)
    return result
