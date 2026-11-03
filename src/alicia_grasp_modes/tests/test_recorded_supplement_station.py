"""Recorded r11 post-arrival messages through the real task consumers.

Only transport, clocks and task entry state are supplied by the harness. This
does not simulate an arm, authorize a motion, or reproduce subscriber timing
which the recorder did not observe. The original frame/SDK values are intact.
"""
from base64 import b64decode
from collections import deque
from copy import deepcopy
import gzip
import json
from pathlib import Path
import threading

import roslib.message
from alicia_flexible_grasp.robot.observation_view_gateway import _camera_configuration_contract
from alicia_grasp_modes.rgb_observation_source import digest

from alicia_grasp_modes import station_compute_wait_task as wait
from alicia_grasp_modes.station_handoff_diagnostics import task_station_snapshot
from test_task_modes import configured, module
from test_observe_return_task import at_ns


def recorded_wire():
    path = Path(__file__).parent/'fixtures/supplement_station_r11_20261023.json.gz'
    return json.loads(gzip.decompress(path.read_bytes()))


def message(row):
    cls = roslib.message.get_message_class(row['type'])
    assert cls is not None and cls._md5sum == row['md5']
    return cls().deserialize(b64decode(row['serialized_b64']))


def replay(configured, monkeypatch, *, corrected_publication=False):
    # Exact historical wire contracts retain their original 5-second policy.
    # Current 10-second coverage is separate; the fixture bytes stay unchanged.
    from alicia_grasp_modes import station_compute_reacquisition as historical_policy
    monkeypatch.setattr(historical_policy, 'IDENTITY_WAIT_NS', 5_000_000_000)
    n, params = configured
    data = recorded_wire()
    phase_row = next(row for row in data['rows'] if row['kind'] == 'phase')
    phase = message(phase_row)
    before = dict(data['prior'])
    for row in data['rows']:
        if row['received_ns'] < phase_row['received_ns']:
            before[row['kind']] = row
    request = json.loads(message(before['request']).data)
    session = json.loads(message(before['session']).data)
    plan = message(before['plan'])
    assert request['state'] == 'qualified'
    assert plan.plan_id == session['view_plan_id'] == request['view_plan_id']
    assert plan.diagnostic == 'FAR_FIELD_OBSERVATION_PLAN'
    for namespace in ('grasp', 'perception'):
        prefix = '/'+namespace+'/'
        params['/'+namespace] = {k[len(prefix):]: v for k,v in
            data['startup_parameters'].items() if k.startswith(prefix)}
    params['/camera'] = deepcopy(json.loads(message(before['camera']).data)['parameters'])
    old_camera = deepcopy(params['/camera'])
    old_camera['runtime_profile'].pop('stationary_temporal')
    old_hash = digest(_camera_configuration_contract(old_camera))
    corrected_hash = digest(_camera_configuration_contract(params['/camera']))
    params['/gui/joint_direct_mode'] = False
    params[module.SELECTION_PARAM] = deepcopy(request['selection'])
    now = [data['start_ns']]
    monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda: at_ns(now[0])))
    monkeypatch.setattr(wait.time, 'monotonic', lambda: 1.+(now[0]-data['start_ns'])*1e-9)
    n._mode_selection = deepcopy(request['selection'])
    n._mode_bound_selection = deepcopy(request['selection'])
    n._mode_ready = True
    n.active = True
    n.stage = 2
    n._execution_authority_revoked = False
    n._bound_execution_plan = deepcopy(plan)
    n._bound_execution_plan_id = plan.plan_id
    n._bound_execution_plan_digest = n._execution_plan_digest(plan)
    n.latest_grasp6d_plan = deepcopy(plan)
    n._observe_return_session = session
    n._observation_measurement_request = request
    n._published_near_field_phase = phase
    n._near_field_phase_id = phase.phase_id
    n._near_field_active = True
    n._near_field_phase_deadline_sec = phase.deadline.to_sec()
    n._following_evidence_lock = threading.RLock()
    n._following_epoch_ns = request['control_epoch_ns']
    n._following_accepted_history = deque(maxlen=128)
    n._following_sdk_history = deque(maxlen=128)
    n.latest_target_geometry = message(before['geometry'])
    evidence = []
    registered = []
    requalification = []
    for row in data['rows']:
        now[0] = row['received_ns']
        kind = row['kind']
        if now[0] < phase_row['received_ns'] and kind != 'feedback':
            continue
        msg = message(row)
        if corrected_publication:
            # Counterfactual input, explicitly separate from historical replay:
            # the repaired camera publisher delivers the temporal configuration
            # to the producer before its unchanged validator freezes this hash.
            # No pose, object/mask pixel, timestamp, deadline or threshold changes.
            for slot in msg.__slots__:
                value = getattr(msg, slot)
                if type(value) is str:
                    setattr(msg, slot, value.replace(old_hash, corrected_hash))
        if kind == 'camera':
            params['/camera'] = deepcopy(json.loads(msg.data)['parameters'])
        elif kind == 'feedback':
            n.accepted_joint_cb(msg)
        elif kind == 'status':
            n._unknown_support_status_cb(msg)
        elif kind == 'mask':
            n._station_compute_task_mask_cb(msg)
        elif kind == 'object':
            n.obj_cb(msg)
        elif kind == 'geometry':
            n.target_geometry_cb(msg)
        elif kind == 'preview':
            n.grasp6d_preview_plan_cb(msg)
        if n._station_task_record is not None:
            source = n._station_task_record['contract']['source_ns']
            if not registered or source != registered[-1]:
                registered.append(source)
        if n._station_requalification_task is not None:
            requalification.append(now[0])
            if corrected_publication:
                break  # Later historical messages came from the failed handoff.
        if kind == 'status':
            status = json.loads(msg.data)
            evidence.append(dict(received_ns=now[0], snapshot=task_station_snapshot(n),
                camera_hash=digest(_camera_configuration_contract(params['/camera'])),
                expected_camera_hash=status.get('station_compute_wait',{}).get('binding',{}).get('camera_configuration_sha256'),
                receiving=n._station_task_receiving(), loss=deepcopy(n._unknown_target_loss)))
    return n, registered, requalification, evidence


def test_recorded_camera_mismatch_still_rejects_station(configured, monkeypatch):
    _, registered, recovery, evidence = replay(configured, monkeypatch)
    assert not registered and not recovery
    reasons = {event['reason'] for row in evidence for event in row['snapshot']['recent_join_events']}
    assert 'station_task_camera_changed' in reasons


def test_corrected_publication_registers_recorded_station_and_enrolls_remeasurement(configured, monkeypatch, tmp_path):
    n, registered, recovery, evidence = replay(configured, monkeypatch, corrected_publication=True)
    (tmp_path/'station_replay.json').write_text(json.dumps(dict(
        registered=registered, requalification_stamps=recovery, evidence=evidence), indent=2))
    assert registered, evidence[-1]
    assert recovery, evidence[-1]
