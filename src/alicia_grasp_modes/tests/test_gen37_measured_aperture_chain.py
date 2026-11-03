"""One-observation task replay with actual jaw-Y following errors.

The original GEN35 helper remains unchanged. Only its explicit FK/TF/device
simulators are wrapped; real source production, fusion, candidate generation,
plan admission, task execution, feedback gates and closing consumers still run.
No result here is evidence of a physical grasp or future controller following.
The narrow 40.5 mm scene and 3.5 mm bias come from bounded real-generation
fixture probes. They retain the original 2 mm fit/reserve configuration and
remain inside the original 6/10 mm endpoint tolerances.
"""
from copy import deepcopy
import hashlib
import inspect
from types import SimpleNamespace as NS

import numpy as np
import pytest

import test_gen35_minimal_chain_replay as chain


@pytest.fixture(autouse=True)
def forbid_unsimulated_external_boundaries(monkeypatch):
    import socket
    import rospy

    def forbidden(*args, **kwargs):
        raise chain.UnsimulatedExternalCall('unmodeled GEN37 boundary: '+repr(args))

    for name in ('connect', 'connect_ex', 'sendto', 'bind', 'listen'):
        monkeypatch.setattr(socket.socket, name, forbidden)
    monkeypatch.setattr(socket, 'create_connection', forbidden)
    for name in ('init_node', 'Publisher', 'Subscriber', 'Service', 'ServiceProxy',
                 'wait_for_service', 'set_param', 'delete_param'):
        monkeypatch.setattr(rospy, name, forbidden)


def _point_digest(c):
    return hashlib.sha256(np.ascontiguousarray(
        c.prepared.geometry.object_points_base).tobytes()).hexdigest()


def _plan_signature(c):
    from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
    return (compute_plan_id(c.contact), c.contact.plan_id,
            c.contact.contact_geometry_evidence, deepcopy(c.contact.object_geometry),
            c.contact.header.stamp.to_nsec(), c.contact.target_track_id,
            c.origin.plan_id, _point_digest(c))


class _ActualJawBias:
    """Wrap measured FK and matching TF, never alter a frozen source/plan.

    The original helper has private simulator closures. A monkeypatch proxy
    captures that simulator's execution frame when it installs SimulatedFk.
    Reading its q/nominal transform table preserves the original finite
    encoder-cell validation, while TF sees exactly the same biased transform.
    """
    def __init__(self, monkeypatch, delta_m, onset, terminal_fault=None):
        self.monkeypatch = monkeypatch
        self.delta_m = float(delta_m)
        self.onset = onset
        self.terminal_fault = terminal_fault
        self.execution_frame = None
        self.c = None
        self.late_active = False
        self.injections = []
        self.close_check_count = 0
        self.signature_before = None

    def contact_moves(self):
        if self.c is None or not hasattr(self.c, 'contact'):
            return []
        return [row for row in self.c.device_calls
                if row[0] == 'motion' and row[1] == self.c.contact.plan_id]

    def active(self):
        if self.onset == 'before_close':
            return self.late_active
        threshold = {'pregrasp': 1, 'approach': 2}[self.onset]
        return len(self.contact_moves()) >= threshold

    def biased(self, nominal):
        actual = np.asarray(nominal, dtype=float).copy()
        if self.active():
            actual[:3, 3] += actual[:3, 1] * self.delta_m
        return actual

    def sync_tf(self):
        if self.execution_frame is None:
            return
        local = self.execution_frame.f_locals
        if not {'actual', 'q', 'transforms'} <= set(local):
            return
        nominal = local['transforms'][tuple(local['q'])]
        measured = self.biased(nominal)
        for name, value in zip('xyz', measured[:3, 3]):
            setattr(local['actual'].pose.position, name, float(value))

    def note_fk(self, q, nominal, measured):
        self.sync_tf()
        if not self.active() or self.c is None or not hasattr(self.c, 'contact'):
            return
        if self.signature_before is None:
            self.signature_before = _plan_signature(self.c)
        local = self.execution_frame.f_locals
        if tuple(q) != tuple(local['q']):
            return
        # This is a measured-pose fixture fact, not a production gate bypass.
        vector = np.asarray(measured)[:3, 3] - np.asarray(nominal)[:3, 3]
        expected = np.asarray(nominal)[:3, 1] * self.delta_m
        np.testing.assert_allclose(vector, expected, rtol=0., atol=1e-12)
        self.injections.append(dict(contact_moves=len(self.contact_moves()),
            offset_m=vector.tolist(), accepted_stamp_ns=(
                self.c.task.latest_accepted_joint_state.header.stamp.to_nsec())))

    def setattr(self, *args, **kwargs):
        """Delegate all patches except the two explicit simulator providers."""
        if len(args) >= 3:
            target, name, value = args[:3]
            if name == 'SerialUrdfFk' and getattr(value, '__name__', '') == 'SimulatedFk':
                frame = inspect.currentframe().f_back
                assert frame.f_code.co_name == 'unified_replay'
                assert frame.f_code.co_filename == chain.__file__
                self.execution_frame = frame
                control = self

                class ActualBiasedFk:
                    def __init__(self, *constructor_args, **constructor_kwargs):
                        self.original = value(*constructor_args, **constructor_kwargs)
                        self.model_sha256 = self.original.model_sha256

                    def __call__(self, joints):
                        nominal = self.original(joints)
                        measured = control.biased(nominal)
                        control.note_fk(joints, nominal, measured)
                        return measured

                return self.monkeypatch.setattr(target, name, ActualBiasedFk, **kwargs)
            if name == 'sleep' and getattr(value, '__name__', '') == 'sleep':
                frame = inspect.currentframe().f_back
                if frame.f_code.co_name == 'unified_replay' and frame.f_code.co_filename == chain.__file__:
                    def sleep_and_update_tf(duration):
                        result = value(duration)
                        self.sync_tf()
                        return result
                    return self.monkeypatch.setattr(target, name, sleep_and_update_tf, **kwargs)
        return self.monkeypatch.setattr(*args, **kwargs)

    def __getattr__(self, name):
        return getattr(self.monkeypatch, name)

    def bind_task(self, c):
        self.c = c
        original_close_check = c.task._validate_measured_contact_before_close

        def check_current_contact(plan, config):
            self.close_check_count += 1
            if self.onset == 'before_close':
                self.signature_before = _plan_signature(c)
                self.late_active = True
                c.advance(.05)  # new accepted feedback, not a rewritten old stamp
                self.sync_tf()
            if self.terminal_fault == 'feedback_expired':
                # No new sample arrives during this bounded delay. Consumers
                # must age the original sample rather than reuse a prior pass.
                c.clock[0] += .6
            elif self.terminal_fault == 'epoch_changed':
                c.task._following_epoch_ns += 1
            return original_close_check(plan, config)

        self.monkeypatch.setattr(c.task, '_validate_measured_contact_before_close', check_current_contact)


def _replay(monkeypatch, tmp_path, *, width_m, delta_m, onset='pregrasp', terminal_fault=None):
    """Original task main loop, parameterized only at simulator boundaries."""
    control = _ActualJawBias(monkeypatch, delta_m, onset, terminal_fault)
    original_scene = chain.ray_scene
    original_arrival = chain.arrived_same_scene

    def scene_with_width(*args, **kwargs):
        kwargs['width'] = float(width_m)
        return original_scene(*args, **kwargs)

    def bind_arrival(*args, **kwargs):
        c = original_arrival(*args, **kwargs)
        control.bind_task(c)
        # Keep the historical 10 mm endpoint policy explicit. The new spatial
        # check must add geometry evidence rather than silently tighten it.
        c.cfg['measured_endpoint_position_tolerance_m'] = .010
        c.cfg['unknown_measured_endpoint_position_tolerance_m'] = .010
        return c

    monkeypatch.setattr(chain, 'ray_scene', scene_with_width)
    monkeypatch.setattr(chain, 'arrived_same_scene', bind_arrival)
    c = chain.unified_replay(control, tmp_path)
    control.sync_tf()
    assert control.execution_frame is not None
    assert c.remote_report['plan_id'] == c.contact.plan_id
    assert c.registration.ok and c.prepared.geometry.ok
    assert len(c.task._frozen_contact_pub.messages) == 1
    assert len(c.phases) == 1 and c.session['max_observations'] == 1
    assert sum(row[0] == 'motion' and row[1] == c.origin.plan_id for row in c.device_calls) == 1
    assert [row['state'] for row in c.station_requests] == ['prepare', 'retire']
    if control.signature_before is not None:
        assert control.signature_before == _plan_signature(c)
    return c, control


def _frozen_jaw_geometry(c, offset_m):
    """Read-only fixture assertions against real generated frozen geometry.

    Legacy patch/support calculations deliberately use their unchanged pure
    kernels. They show why a tool-Y displacement needs the additional check.
    The full OBB already includes the inference budget; no U is added again.
    """
    from alicia_flexible_grasp.grasp.contact_evidence import decode_contact_evidence
    from alicia_flexible_grasp.grasp.gripper_geometry import (
        ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M, ANALYTICAL_FINGER_SIZE_XYZ_M,
        ANALYTICAL_PALM_SIZE_XYZ_M, GripperGeometry, _box_corners, _stage_boxes,
        finger_contact_patch_overlap_m)
    from alicia_flexible_grasp.grasp.rich_plan_integrity import pose_values
    from tf.transformations import quaternion_matrix

    plan = c.contact
    evidence = decode_contact_evidence(plan, required=True)
    values = pose_values(plan.poses[2])
    planned = quaternion_matrix(values[3:])
    planned[:3, 3] = values[:3]
    actual = planned.copy()
    actual[:3, 3] += planned[:3, 1] * float(offset_m)
    geometry = plan.object_geometry
    obb_pose = pose_values(geometry.pose_base)
    rotation = quaternion_matrix(obb_pose[3:])[:3, :3]
    size = [geometry.size_xyz_m.x, geometry.size_xyz_m.y, geometry.size_xyz_m.z]
    vertices = _box_corners(np.asarray(obb_pose[:3]), rotation, np.asarray(size))
    opening = .05  # original helper's unchanged physical open command/feedback
    nominal_pair = planned[:3, 3] + planned[:3, :3] @ ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M
    actual_pair = actual[:3, 3] + actual[:3, :3] @ ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M
    nominal_y = (vertices-nominal_pair) @ planned[:3, 1]
    actual_y = (vertices-actual_pair) @ actual[:3, 1]
    nominal_clearances = np.asarray([opening/2.+min(nominal_y), opening/2.-max(nominal_y)])
    actual_clearances = np.asarray([opening/2.+min(actual_y), opening/2.-max(actual_y)])
    gripper = GripperGeometry(.05, .002, ANALYTICAL_FINGER_SIZE_XYZ_M,
                              ANALYTICAL_PALM_SIZE_XYZ_M, .003)
    overlap = finger_contact_patch_overlap_m(evidence['contact_center_base'], actual[:3, 3],
        actual[:3, :3], evidence['contact_height_axis_base'], *evidence['contact_height_bounds_m'])
    normal = np.asarray([geometry.support_normal_base.x, geometry.support_normal_base.y,
                         geometry.support_normal_base.z])
    clearance = min(float(np.min(_box_corners(center, actual[:3, :3], box_size) @ normal
        + geometry.support_offset_m)) for _name, center, box_size in _stage_boxes(
            actual, gripper, opening, 'y', 'z'))
    assert abs(np.linalg.norm(actual[:3, 3]-planned[:3, 3])-abs(offset_m)) < 1e-12
    assert abs(offset_m) < .010
    assert overlap+1e-9 >= evidence['minimum_contact_overlap_m']
    assert clearance+1e-9 >= gripper.support_clearance_m
    return NS(nominal_clearances=nominal_clearances, actual_clearances=actual_clearances,
              legacy_overlap_m=overlap, legacy_support_clearance_m=clearance,
              opening_m=opening, evidence=evidence)


def _assert_no_close(c):
    assert not any(row[0] == 'gripper' and row[2] < .05 for row in c.device_calls)
    assert c.states[-1].state == 'FAILED', c.states[-1]
    assert c.execution_result is False


@pytest.mark.parametrize('onset', ['pregrasp', 'approach'])
@pytest.mark.parametrize('delta_m', [-.0035, .0035])
def test_actual_jaw_error_blocks_next_contact_motion_before_close(monkeypatch, tmp_path, onset, delta_m):
    c, control = _replay(monkeypatch, tmp_path, width_m=.0405, delta_m=delta_m, onset=onset)
    geometry = _frozen_jaw_geometry(c, delta_m)
    assert 0. < min(geometry.nominal_clearances) < abs(delta_m)
    assert min(geometry.actual_clearances) < 0.
    assert control.injections
    assert len(control.contact_moves()) == {'pregrasp': 1, 'approach': 2}[onset]
    assert control.close_check_count == 0
    _assert_no_close(c)
    sample = c.task._last_measured_aperture_sample
    assert sample['ok'] is False
    assert sample['code'] in {'MEASURED_APERTURE_INSUFFICIENT', 'MEASURED_APERTURE_MODEL_REJECTED'}
    if sample['code'] == 'MEASURED_APERTURE_MODEL_REJECTED':
        from alicia_flexible_grasp.grasp.contact_evidence import decode_contact_evidence
        frozen = decode_contact_evidence(c.contact, required=True)
        assert sample['reason'] == 'BOX_INFERENCE_PRECISION_VIEW_REQUIRED'
        projection = sample['inferred_projection']
        assert projection['ok'] is False and projection['code'] == sample['reason']
        assert projection['model_sha256'] == frozen['box_model_evidence']['model_sha256']
    assert sample['stage_reference_pose_index'] == {'pregrasp': 0, 'approach': 1}[onset]
    assert sample['accepted_stamp_ns'] == c.task.latest_accepted_joint_state.header.stamp.to_nsec()
    assert sample['code'] in c.states[-1].message, c.states[-1]


@pytest.mark.parametrize('width_m,delta_m', [(.0405, 0.), (.0405, .00025), (.030, .0035)])
def test_sufficient_actual_aperture_keeps_one_observation_and_one_close(monkeypatch, tmp_path, width_m, delta_m):
    c, control = _replay(monkeypatch, tmp_path, width_m=width_m, delta_m=delta_m)
    geometry = _frozen_jaw_geometry(c, delta_m)
    assert min(geometry.actual_clearances) > .0005
    assert len(control.contact_moves()) == 3
    assert control.close_check_count == 1
    assert c.states[-1].state == 'HOLDING', c.states[-1]
    assert 'GRASP_HOLD_UNVERIFIED' in c.states[-1].message
    assert sum(row[0] == 'gripper' and row[2] < .05 for row in c.device_calls) == 1
    assert c.task._last_measured_contact_sample['ok'] is True
    assert c.task._last_measured_aperture_sample['ok'] is True
    assert c.task._last_measured_aperture_sample['stage_reference_pose_index'] == 2


@pytest.mark.parametrize('delta_m', [-.0035, .0035])
def test_new_actual_error_before_close_cannot_reuse_earlier_aperture_pass(monkeypatch, tmp_path, delta_m):
    c, control = _replay(monkeypatch, tmp_path, width_m=.0405, delta_m=delta_m, onset='before_close')
    geometry = _frozen_jaw_geometry(c, delta_m)
    assert 0. < min(geometry.nominal_clearances) < abs(delta_m)
    assert min(geometry.actual_clearances) < 0.
    assert len(control.contact_moves()) == 3
    assert control.close_check_count == 1
    assert control.injections
    _assert_no_close(c)
    assert 'APERTURE' in c.states[-1].message, c.states[-1]


@pytest.mark.parametrize('fault,reasons', [
    ('feedback_expired', ('STALE', 'EXPIRED')),
    ('epoch_changed', ('EPOCH', 'COMPLETED_ENDPOINT_CHANGED_OR_MISSING')),
])
def test_close_rechecks_feedback_age_and_epoch_after_earlier_aperture_pass(monkeypatch, tmp_path, fault, reasons):
    c, control = _replay(monkeypatch, tmp_path, width_m=.030, delta_m=0.,
        onset='before_close', terminal_fault=fault)
    assert len(control.contact_moves()) == 3
    assert control.close_check_count == 1
    _assert_no_close(c)
    assert any(reason in c.states[-1].message for reason in reasons), c.states[-1]


def test_atomic_close_new_sdk_z_sample_repeats_final_patch_on_same_fk(monkeypatch, tmp_path):
    """New encoder values after both earlier checks reach the real close gate.

    The original fixture leaves height stress disabled. This case preserves
    that policy and checks its explicit NOT_APPLICABLE receipt. A large fresh
    Z drift also fails aperture; its final-patch subreport proves the new
    same-sample consumer runs, not that patch is the sole necessary rejection.
    """
    import json
    original_control = _ActualJawBias

    class AtomicCloseZSample(original_control):
        def bind_task(self, c):
            super().bind_task(c)
            original_close = c.task._close_gripper
            self.atomic_close_count = 0

            def enter_close_with_new_sdk(*args, **kwargs):
                self.atomic_close_count += 1
                plan = kwargs['execution_plan']
                assert plan.plan_id == c.contact.plan_id
                self.prior_contact = deepcopy(c.task._last_measured_contact_sample)
                self.prior_aperture = deepcopy(c.task._last_measured_aperture_sample)
                assert self.prior_contact['ok'] is True
                assert self.prior_aperture['ok'] is True
                assert self.prior_aperture['stage_reference_pose_index'] == 2
                assert self.prior_aperture['height_registration_margin']['ok'] is True
                assert self.prior_aperture['height_registration_margin']['code'] == 'NOT_APPLICABLE'
                self.prior_sdk = deepcopy(c.task.latest_accepted_joint_state)

                # Create a NEW finite encoder cell in the existing explicit
                # simulator. Identical q never changes FK under the same model.
                local = self.execution_frame.f_locals
                old_q = list(local['q'])
                actual = local['transforms'][tuple(old_q)].copy()
                normal = plan.object_geometry.support_normal_base
                dz = .030 * np.asarray([normal.x, normal.y, normal.z])
                actual[:3, 3] += dz
                new_q = old_q.copy()
                new_q[2] += .030
                assert tuple(new_q) not in local['transforms']
                local['transforms'][tuple(new_q)] = actual.copy()
                local['q'][:] = new_q
                self.new_actual_T = actual.copy()
                c.advance(.05)  # the unchanged helper emits SDK and accepted q
                self.sync_tf()
                self.new_sdk = deepcopy(c.task.latest_accepted_joint_state)
                assert list(self.new_sdk.position[:6]) == new_q
                assert list(self.new_sdk.position[:6]) != list(self.prior_sdk.position[:6])
                assert self.new_sdk.header.stamp > self.prior_sdk.header.stamp
                assert self.new_sdk.header.frame_id == 'sdk_measured'
                observed = self.execution_frame.f_locals['actual'].pose.position
                np.testing.assert_allclose([observed.x, observed.y, observed.z],
                    self.new_actual_T[:3, 3], rtol=0., atol=1e-12)
                # Do not replace a geometry/authority result. The original
                # close implementation executes its real atomic action callback.
                return original_close(*args, **kwargs)

            self.monkeypatch.setattr(c.task, '_close_gripper', enter_close_with_new_sdk)

    monkeypatch.setitem(globals(), '_ActualJawBias', AtomicCloseZSample)
    c, control = _replay(monkeypatch, tmp_path, width_m=.030, delta_m=0.)
    report = c.task._last_measured_aperture_sample
    (tmp_path/'same_sample_atomic_close_result.json').write_text(json.dumps(dict(
        prior_contact=control.prior_contact, prior_aperture=control.prior_aperture,
        prior_sdk_stamp_ns=control.prior_sdk.header.stamp.to_nsec(),
        new_sdk_stamp_ns=control.new_sdk.header.stamp.to_nsec(),
        prior_sdk_positions=list(control.prior_sdk.position),
        new_sdk_positions=list(control.new_sdk.position),
        new_actual_T_base_tool0=control.new_actual_T.tolist(),
        atomic_close_report=report, atomic_close_entry_count=control.atomic_close_count,
        device_calls=c.device_calls, physical_grasp_verified=False,
        aperture_may_also_reject=True, height_stress_policy_enabled=False),
        default=str, indent=2)+'\n')
    assert len(control.contact_moves()) == 3
    assert control.close_check_count == 1 and control.atomic_close_count == 1
    _assert_no_close(c)
    assert report['ok'] is False
    assert report['stage_label'] == 'fixed gripper close submission'
    assert report['stage_reference_pose_index'] == 2
    assert report['accepted_stamp_ns'] == control.new_sdk.header.stamp.to_nsec()
    assert report['accepted_stamp_ns'] > control.prior_contact['accepted_stamp_ns']
    patch = report['final_contact_overlap']
    assert patch['ok'] is False and patch['code'] == 'MEASURED_CONTACT_PATCH_MISS'
    assert patch['actual_overlap_m'] < patch['minimum_contact_overlap_m']
    height = report['height_registration_margin']
    assert height['ok'] is True and height['code'] == 'NOT_APPLICABLE'
    for child in (patch, height):
        for name in ('accepted_stamp_ns', 'epoch_ns', 'model_sha256', 'measured_opening_width_m'):
            assert child[name] == report[name]
