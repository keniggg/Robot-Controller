"""Real integration tests for the production close/support path.

Primitive fingers isolate the dynamics contract; they do not certify the Alicia
CAD or a whole-arm trajectory. Recorded full-model runs are separate evidence.
"""
import math

import numpy as np
import pytest

from test_mujoco_digital_twin_server_protocol import server_module as engine


def plant(size=(.030, .020, .020), yaw=0., offset_y=0., gap_above=.0017,
          duration=.8, target_gap=None, gravity=-9.81):
    mj = pytest.importorskip('mujoco')
    quat = [math.cos(yaw / 2), 0., 0., math.sin(yaw / 2)]
    half = np.asarray(size) / 2
    xml = f'''<mujoco><option timestep=".002" gravity="0 0 {gravity}"
        integrator="implicitfast" cone="elliptic"/>
      <worldbody>
        <body name="Link6">
          <body name="Link7" pos="0 -.028 .030">
            <joint name="left_finger" type="slide" axis="0 -1 0" range="-.025 0"/>
            <geom type="box" size=".030 .003 .024" mass=".050" friction=".1 .005 .0001"/>
          </body>
          <body name="Link8" pos="0 .028 .030">
            <joint name="right_finger" type="slide" axis="0 -1 0" range="0 .025"/>
            <geom type="box" size=".030 .003 .024" mass=".050" friction=".1 .005 .0001"/>
          </body>
        </body>
        <body name="target_object" pos="0 {offset_y} {half[2] + gap_above}"
              quat="{' '.join(map(str, quat))}">
          <freejoint name="target_object_joint"/>
          <geom type="box" size="{' '.join(map(str, half))}" mass=".020"
                friction=".1 .005 .0001"/>
        </body>
        <body name="detected_support" mocap="true">
          <geom type="plane" size="10 10 .125" friction="1.2 .08 .02"/>
        </body>
      </worldbody>
      <actuator>
        <position name="left_finger_act" joint="left_finger" kp="200" kv="7"
                  forcerange="-5 5"/>
        <position name="right_finger_act" joint="right_finger" kp="200" kv="7"
                  forcerange="-5 5"/>
      </actuator></mujoco>'''
    model = mj.MjModel.from_xml_string(xml)
    data = mj.MjData(model)
    backend = engine.MujocoDigitalTwinBackend(ros_sync_joint_states=False)
    backend._mujoco = mj
    bid = lambda name: mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, name)
    jid = lambda name: mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
    aid = lambda name: mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, name)
    meta = dict(arm_joints=[], arm_actuators=[],
                left_finger_joint=jid('left_finger'), right_finger_joint=jid('right_finger'),
                left_finger_actuator=aid('left_finger_act'), right_finger_actuator=aid('right_finger_act'),
                left_body=bid('Link7'), right_body=bid('Link8'), orientation_body=bid('Link6'),
                object_body=bid('target_object'), object_joint=jid('target_object_joint'),
                support_body=bid('detected_support'), support_mocap_id=0)
    mj.mj_forward(model, data)
    projected_width = abs(math.sin(yaw)) * size[0] + abs(math.cos(yaw)) * size[1]
    payload = dict(gripper=dict(close_settle_sec=duration,
        close_target_inner_gap_m=max(0., projected_width - .004) if target_gap is None else target_gap))
    return backend, model, data, meta, payload


def close(parts):
    backend, model, data, meta, payload = parts
    evidence = {}
    result = backend._simulate_close_contact(model, data, meta, [],
        payload=payload, close_evidence=evidence)
    return result, evidence


@pytest.mark.parametrize('size,yaw', [
    ((.030, .020, .020), 0.),
    ((.040, .025, .015), .15),
    ((.010, .035, .030), -.2),
])
def test_natural_preparation_and_continuous_closing_with_varied_geometry(size, yaw):
    parts = plant(size=size, yaw=yaw)
    before = parts[2].qpos.copy(), parts[2].qvel.copy()
    result, evidence = close(parts)
    assert result[0] and result[1], result[4]
    assert evidence['preparation_samples'] == evidence['close_samples'] == 400
    assert evidence['preparation_simulation_time_s'] == pytest.approx(.8)
    assert evidence['close_simulation_time_s'] == pytest.approx(.8)
    assert result[3].time == pytest.approx(1.6)
    assert evidence['preclose_state']['support_normal_force_n'] > 0
    assert evidence['close_final_state']['gripper']['left_object_normal_force_n'] > 0
    assert evidence['close_final_state']['gripper']['right_object_normal_force_n'] > 0
    # Natural gravity/support motion happened, and was never counted as a push.
    initial = np.array(evidence['initial_state']['object_position_m'])
    prep = np.array(evidence['preparation_trace'])[:, 1:4].astype(float)
    assert np.max(np.linalg.norm(prep - initial, axis=1)) > .002
    assert evidence['final_object_displacement_from_preclose_m'] < .002
    np.testing.assert_array_equal(parts[2].qpos, before[0])
    np.testing.assert_array_equal(parts[2].qvel, before[1])


def test_closing_only_commands_ctrl_and_records_post_integration_state(monkeypatch):
    backend, model, data, meta, payload = parts = plant()
    command = backend._command_gripper_inner_gap
    writes = []
    def observe(model, data, width, meta):
        qpos, qvel = data.qpos.copy(), data.qvel.copy()
        command(model, data, width, meta)
        np.testing.assert_array_equal(data.qpos, qpos)
        np.testing.assert_array_equal(data.qvel, qvel)
        writes.append(width)
    def forbidden(*args, **kwargs):
        pytest.fail('finger qpos initialization must not occur inside dynamic closure')
    monkeypatch.setattr(backend, '_command_gripper_inner_gap', observe)
    monkeypatch.setattr(backend, '_apply_gripper_inner_gap', forbidden)
    observe_state = backend._closure_dynamic_state
    def same_instant(model, data, meta):
        state = observe_state(model, data, meta)
        if state['finite_state']:
            np.testing.assert_array_equal(state['object_position_m'], state['object_qpos'][:3])
        return state
    monkeypatch.setattr(backend, '_closure_dynamic_state', same_instant)
    result, evidence = close(parts)
    assert result[0], result[4]
    target = payload['gripper']['close_target_inner_gap_m']
    assert writes.count(target) == 400
    assert evidence['first_finger_contact_sample'] > 1
    first = evidence['first_finger_contact_state']['gripper']
    assert abs(first['left_actuator_force_n']) + abs(first['right_actuator_force_n']) > 0


@pytest.mark.parametrize('duration,expected', [(.001, 0), (.051, 25), (.801, 400)])
def test_request_time_is_never_rounded_up_or_restarted(duration, expected):
    result, evidence = close(plant(duration=duration))
    assert evidence['requested_phase_samples'] == expected
    assert evidence['preparation_samples'] <= expected
    assert evidence['close_samples'] <= expected
    assert evidence.get('close_simulation_time_s', 0) <= duration
    if expected == 0:
        assert not result[0] and result[1]
        assert 'shorter than one' in result[4][0]


def test_actual_single_finger_push_over_two_mm_still_rejects():
    result, evidence = close(plant(offset_y=.004))
    assert not result[0] and result[1]
    assert 'single-finger' in result[4][0]
    assert evidence['final_object_displacement_from_preclose_m'] > .002
    assert evidence['close_samples'] < 400
    # The pre-close baseline stays fixed; the first contact never replaces it.
    baseline = np.array(evidence['preclose_state']['object_position_m'])
    last = np.array(evidence['close_final_state']['object_position_m'])
    assert np.linalg.norm(last - baseline) == pytest.approx(
        evidence['final_object_displacement_from_preclose_m'])


@pytest.mark.parametrize('gap,offset,expected', [
    (-.005, 0., 'disallowed collision'),
    (.0017, .016, 'before open-gripper support preparation'),
])
def test_initial_collision_or_finger_interference_never_becomes_baseline(gap, offset, expected):
    result, evidence = close(plant(gap_above=gap, offset_y=offset))
    assert not result[0]
    assert expected in result[4][0]
    assert evidence['close_samples'] == 0
    assert 'preclose_state' not in evidence


def test_free_object_without_support_cannot_enter_closing():
    result, evidence = close(plant(gap_above=.05, gravity=0.))
    assert not result[0] and result[1]
    assert 'no load-bearing support' in result[4][0]
    assert evidence['preparation_samples'] == 400 and evidence['close_samples'] == 0


def test_full_budget_without_two_sided_contact_remains_a_failure():
    result, evidence = close(plant(target_gap=.05))
    assert not result[0] and result[1]
    assert 'without geometrically opposed' in result[4][0]
    assert evidence['close_samples'] == 400


def test_nonfinite_scene_is_rejected_before_physics_steps():
    parts = plant()
    parts[2].qvel[-1] = float('nan')
    result, evidence = close(parts)
    assert not result[0] and 'non-finite' in result[4][0]
    assert evidence['preparation_samples'] == evidence['close_samples'] == 0


def test_persistent_free_drift_is_not_zeroed_by_support_preparation():
    backend, model, data, meta, payload = parts = plant(gap_above=.0001)
    # Independent negative plant: almost frictionless support and a real
    # initial free-body velocity. The production 20 g/.1 fixture is unchanged.
    model.geom_friction[:] = [1e-6, 1e-7, 1e-8]
    data.qvel[model.jnt_dofadr[meta['object_joint']]] = .05
    backend._mujoco.mj_forward(model, data)
    result, evidence = close(parts)
    assert not result[0]
    assert evidence['preclose_state']['object_qvel'][0] > .04
    assert evidence['preclose_state']['object_position_m'][0] > .03
    assert ('without geometrically opposed' in result[4][0]
            or 'single-finger' in result[4][0])


def test_sample_cap_reports_truncation_without_inventing_requested_time():
    result, evidence = close(plant(duration=8.1))
    assert result[0], result[4]
    assert evidence['requested_phase_samples'] > 4000
    assert evidence['budget_truncated'] is True
    assert evidence['preparation_samples'] == evidence['close_samples'] == 4000
    assert evidence['close_simulation_time_s'] == 8.0 < 8.1


def test_native_mujoco_automatic_reset_never_manufactures_supported_rest():
    backend, model, data, meta, payload = parts = plant()
    # Finite input deliberately trips MuJoCo's own BADQVEL check, which resets
    # the dynamics to a finite initial pose. No empirical speed limit is used.
    data.qvel[model.jnt_dofadr[meta['object_joint']]] = 1e12
    result, evidence = close(parts)
    assert not result[0]
    assert 'numerical instability' in result[4][0]
    assert any(event['warning'] == 'mjWARN_BADQVEL' and event['count'] > 0
               for event in evidence['numerical_instability_events'])
    assert evidence['preparation_samples'] == 1
    assert evidence['close_samples'] == 0
    assert 'preclose_state' not in evidence


def test_existing_native_warning_is_checked_before_seed_copy_clears_it():
    backend, model, data, meta, payload = parts = plant()
    data.qvel[model.jnt_dofadr[meta['object_joint']]] = 1e12
    backend._mujoco.mj_step(model, data)
    backend._mujoco.mj_forward(model, data)
    assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
    result, evidence = close(parts)
    assert not result[0] and 'numerical instability in input scene' in result[4][0]
    assert evidence['preparation_samples'] == evidence['close_samples'] == 0

@pytest.fixture(scope='module')
def recorded_backend(tmp_path_factory):
    """Original CAD/model and raw payload bytes; only relocate the mesh directory."""
    import gzip
    import hashlib
    import json
    import re
    from pathlib import Path
    from test_mujoco_digital_twin_server_protocol import ROOT
    mj = pytest.importorskip('mujoco')
    fixture = json.loads(gzip.decompress((Path(__file__).parent / 'fixtures' /
        'mujoco_precontact_settling_payloads_20261024.json.gz').read_bytes()))
    assert mj.__version__ == fixture['mujoco_version']
    model_text = fixture['model_xml_utf8']
    assert hashlib.sha256(model_text.encode()).hexdigest() == fixture['model_xml_sha256']
    # Mesh relocation is the sole XML edit and leaves all model physics intact.
    meshdir = str(ROOT / fixture['meshdir_relative_to_project'])
    model_text, count = re.subn(r'(<compiler\b[^>]*\bmeshdir=")[^"]+(")',
        lambda m: m[1] + meshdir + m[2], model_text)
    assert count == 1
    xml_path = tmp_path_factory.mktemp('original_mujoco_model') / 'loaded_candidate.xml'
    xml_path.write_text(model_text)

    class ObservedBackend(engine.MujocoDigitalTwinBackend):
        # Non-mutating observations verify no second preload consumes time.
        def _simulate_lift(self, *args, **kwargs):
            self.retained_time = float(kwargs['retained_data'].time)
            self.first_lift_reference_time = None
            return super()._simulate_lift(*args, **kwargs)
        def _apply_velocity_consistent_arm_reference(self, model, data, *args, **kwargs):
            if self.first_lift_reference_time is None:
                self.first_lift_reference_time = float(data.time)
            return super()._apply_velocity_consistent_arm_reference(model, data, *args, **kwargs)

    backend = ObservedBackend(model_xml=xml_path, max_snapshot_age_sec=120.,
                              ros_sync_joint_states=False)
    return fixture, backend


@pytest.mark.parametrize('case', [0, 1, 2])
def test_frozen_actual_requests_reach_loaded_closure_but_preserve_strict_lift_failure(
        recorded_backend, case):
    import copy
    import hashlib
    import json
    from test_mujoco_digital_twin_server_protocol import client_module
    fixture, backend = recorded_backend
    row = fixture['cases'][case]
    assert hashlib.sha256(row['request_utf8'].encode()).hexdigest() == row['request_sha256']
    assert hashlib.sha256(row['recorded_response_utf8'].encode()).hexdigest() == row['recorded_response_sha256']
    payload = json.loads(row['request_utf8'])
    historical = json.loads(row['recorded_response_utf8'])
    # Historical response is evidence of the old failure, not a simulation stub.
    assert historical['simulation_ok'] is False
    assert historical['failure_code'] == 'MUJOCO_CONTACT_FAILED'
    assert 'single-finger' in historical['failure_reason']
    assert 'lift_evidence' not in historical
    original = copy.deepcopy(payload)
    validated = engine._validate_v3_payload(payload,
        now_sec=row['received_ns'] / 1e9, max_snapshot_age_sec=120.)
    assert validated['object_model']['mass_kg'] == .020
    assert validated['object_model']['friction'] == [.1, .005, .0001]
    response = backend._simulate_validated_grasp(validated, payload['plan_id'])
    response = engine._normalize_http_simulation_result(validated, response, backend)
    response['ok'] = True
    assert payload == original
    close = response['close_evidence']
    assert close['preparation_samples'] == close['close_samples'] == 400
    assert close['requested_phase_samples'] == 400
    assert close['preparation_simulation_time_s'] == close['close_simulation_time_s'] == .8
    assert close['failure_reason'] == ''
    assert backend.retained_time == pytest.approx(1.6)
    assert backend.first_lift_reference_time == pytest.approx(backend.retained_time)
    baseline = np.array(close['preclose_state']['object_position_m'])
    final = np.array(close['close_final_state']['object_position_m'])
    assert np.linalg.norm(final - baseline) == pytest.approx(
        close['final_object_displacement_from_preclose_m'])
    lift = response['lift_evidence']
    assert lift['preload_settle_samples'] == 400
    state = lift['settled_gripper_state']
    assert state['left_object_normal_force_n'] > 0 and state['right_object_normal_force_n'] > 0
    assert lift['max_prescribed_joint_speed_rad_s'] <= .08
    # A real later failure stays a failure; do not tune friction/forces to pass.
    assert response['simulation_ok'] is False and response['lift_success'] is False
    assert response['failure_code'] == 'MUJOCO_CONTACT_FAILED'
    assert 'lost during lift' in response['failure_reason']
    assert lift['lost_contact_samples'] > lift['contact_loss_grace_samples']
    assert lift['object_lift_m'] < lift['minimum_lift_m']
    args = dict(expected_candidate_source=payload['candidate_source'],
        expected_candidate_source_lineage=payload['candidate_source_lineage'])
    actual = client_module.validate_mujoco_gate_response(
        response, payload['plan_id'], 80, require_lift_success=False, **args)
    strict = client_module.validate_mujoco_gate_response(
        response, payload['plan_id'], 80, require_lift_success=True, **args)
    assert actual.ok and actual.code == 'MUJOCO_LIFT_DIAGNOSTIC_ONLY'
    assert not strict.ok


