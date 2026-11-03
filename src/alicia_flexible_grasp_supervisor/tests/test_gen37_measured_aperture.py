"""Pure actual/residual-projected aperture checks; no nodes or device calls."""
import json
from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
import pytest
from tf.transformations import rotation_matrix

from alicia_flexible_grasp.grasp.gripper_geometry import (
    ANALYTICAL_FINGER_SIZE_XYZ_M, ANALYTICAL_PALM_SIZE_XYZ_M,
    GripperGeometry)
from alicia_flexible_grasp.grasp.measured_contact import (
    evaluate_measured_aperture, evaluate_measured_contact)
from alicia_flexible_grasp.grasp.tabletop_geometry_candidates import (
    solve_tool0_translation_for_support_clearance)


@pytest.fixture(autouse=True)
def forbid_external_transport(monkeypatch):
    import socket
    import rospy
    class UnmodeledBoundary(BaseException):
        pass
    def forbidden(*args, **kwargs):
        raise UnmodeledBoundary('pure aperture test attempted an external boundary')
    for name in ('connect', 'connect_ex', 'sendto', 'bind', 'listen'):
        monkeypatch.setattr(socket.socket, name, forbidden)
    monkeypatch.setattr(socket, 'create_connection', forbidden)
    for name in ('init_node', 'Publisher', 'Subscriber', 'Service', 'ServiceProxy',
                 'wait_for_service', 'get_param', 'get_param_cached', 'set_param'):
        monkeypatch.setattr(rospy, name, forbidden)


def scene(index=2, width=.044, length=.065):
    gripper = GripperGeometry(.05, .002, ANALYTICAL_FINGER_SIZE_XYZ_M,
                              ANALYTICAL_PALM_SIZE_XYZ_M, .003)
    center = np.array([0., 0., .0125])
    contact = np.eye(4); contact[:3, :3] = np.diag([-1., 1., -1.])
    contact[:3, 3] = solve_tool0_translation_for_support_clearance(
        contact[:3, :3], center, np.zeros(3), np.array([0., 0., 1.]), .003, gripper)
    stage = contact.copy(); stage[2, 3] += {0:.08, 1:.02, 2:0.}[index]
    return dict(gripper=gripper, actual_T_base_tool0=stage.copy(),
        planned_stage_T_base_tool0=stage,
        planned_contact_T_base_tool0=contact,
        contact_center_base=center, obb_center_base=center.copy(),
        R_base_obb=np.eye(3), obb_size_xyz_m=np.array([length, width, .025]),
        support_normal_base=np.array([0., 0., 1.]), support_offset_m=0.,
        opening_width_m=.05, opening_fit_clearance_each_side_m=.0005,
        stage_reference_pose_index=index)


@pytest.mark.parametrize('index', [0, 1, 2])
def test_nominal_stage_uses_frozen_contact_and_does_not_demand_early_overlap(index):
    kw = scene(index)
    result = evaluate_measured_aperture(**kw)
    assert result['ok'], result
    np.testing.assert_allclose(result['projected_contact_T_base_tool0'],
                               kw['planned_contact_T_base_tool0'], atol=1e-12)
    assert result['projected_contact_is_measured'] is (index == 2)
    assert result['continuous_tracking_tube_checked'] is False
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize('index', [0, 1, 2])
@pytest.mark.parametrize('offset', [-.003, .003])
def test_jaw_error_inside_old_ten_mm_endpoint_limit_consumes_one_side(index, offset):
    kw = scene(index)
    kw['actual_T_base_tool0'][:3, 3] += kw['actual_T_base_tool0'][:3, 1]*offset
    assert abs(offset) < .01
    result = evaluate_measured_aperture(**kw)
    assert not result['ok'], result
    assert result['code'] == 'MEASURED_APERTURE_INSUFFICIENT'
    assert max(result['negative_reach_m'], result['positive_reach_m']) > .025
    # The old height-only gate still sees sufficient patch/support geometry.
    old = evaluate_measured_contact(gripper=kw['gripper'],
        actual_T_base_tool0=np.asarray(result['projected_contact_T_base_tool0']),
        planned_T_base_tool0=kw['planned_contact_T_base_tool0'],
        contact_center_base=kw['contact_center_base'],
        contact_height_axis_base=kw['planned_contact_T_base_tool0'][:3, 2],
        contact_height_bounds_m=[-.0125, .0125], minimum_contact_overlap_m=.0005,
        support_normal_base=kw['support_normal_base'], support_offset_m=0., opening_width_m=.05)
    assert old.ok, old


@pytest.mark.parametrize('offset,width', [(.0005, .044), (-.0005, .044), (.003, .030)])
def test_small_offset_or_wider_remaining_aperture_still_passes(offset, width):
    kw = scene(0, width=width)
    kw['actual_T_base_tool0'][1, 3] += offset
    assert evaluate_measured_aperture(**kw)['ok']


def test_measured_opening_is_used_instead_of_the_maximum_physical_gap():
    kw = scene()
    kw['opening_width_m'] = .0449
    result = evaluate_measured_aperture(**kw)
    assert not result['ok']
    assert result['code'] == 'MEASURED_APERTURE_INSUFFICIENT'
    assert result['required_open_width_m'] < .05
    assert result['required_open_width_m'] > kw['opening_width_m']


@pytest.mark.parametrize('index', [0, 1, 2])
def test_long_oblique_local_grasp_keeps_original_line_reach_and_finite_cad_policy(index):
    kw = scene(index, width=.026, length=.120)
    kw['R_base_obb'] = rotation_matrix(np.deg2rad(15.), [0, 0, 1])[:3, :3]
    result = evaluate_measured_aperture(**kw)
    assert result['ok'], result
    corners = result['union_corner_projection_m']
    assert corners[1]-corners[0] > .05  # Diagnostic is not a new whole-object gate.
    assert result['union_required_open_width_m'] < .05
    assert result['union_reach_policy'] == 'original_center_jaw_line_and_full_finite_CAD_SAT'


def test_asymmetric_union_offset_cannot_hide_in_projection_width_alone():
    kw = scene()
    kw['obb_center_base'][1] += .006
    result = evaluate_measured_aperture(**kw)
    assert not result['ok'] and result['code'] == 'MEASURED_APERTURE_INSUFFICIENT'
    assert kw['obb_size_xyz_m'][1]+2.*kw['opening_fit_clearance_each_side_m'] < .05


def test_frozen_contact_center_keeps_its_tool_offset_under_measured_rotation():
    kw = scene(0, width=.030)
    kw['contact_center_base'] = kw['contact_center_base']+np.array([.004, .001, 0.])
    delta = rotation_matrix(np.deg2rad(1.), [0, 0, 1])
    kw['actual_T_base_tool0'] = delta @ kw['planned_stage_T_base_tool0']
    result = evaluate_measured_aperture(**kw)
    expected = (delta @ np.r_[kw['contact_center_base'], 1.])[:3]
    np.testing.assert_allclose(result['projected_contact_center_base'], expected, atol=1e-12)
    assert not np.allclose(expected, np.asarray(result['projected_contact_T_base_tool0'])[:3, 3])


def test_inferred_model_and_union_share_u_and_report_clearance_preference():
    from test_gen26_regular_box_inference_independent import ray_scene, qualify
    from alicia_flexible_grasp.vision.regular_box_inference import enclose_inferred_box_geometry
    s = ray_scene(width=.030); qualified = qualify(s)
    assert qualified.ok, qualified.audit
    model = qualified.model
    union, _ = enclose_inferred_box_geometry(s.geometry, model)
    kw = scene(0, width=.030)
    rotation = np.column_stack((-model.axes_base[:, 0], model.axes_base[:, 1],
                                -model.support_normal_base))
    contact = np.eye(4); contact[:3, :3] = rotation
    contact[:3, 3] = solve_tool0_translation_for_support_clearance(rotation,
        model.center_base, np.zeros(3), model.support_normal_base, .003, kw['gripper'])
    stage = contact.copy(); stage[2, 3] += .08
    kw.update(actual_T_base_tool0=stage.copy(), planned_stage_T_base_tool0=stage,
        planned_contact_T_base_tool0=contact, contact_center_base=model.center_base,
        obb_center_base=union.center_base, R_base_obb=union.axes_base,
        obb_size_xyz_m=union.size_xyz_m, inferred_model=model)
    result = evaluate_measured_aperture(**kw)
    assert result['ok'], result
    assert result['effective_required_open_width_m'] == pytest.approx(max(
        result['union_required_open_width_m'],
        result['inferred_required_open_width_m']))
    assert result['required_open_width_m'] == pytest.approx(max(
        result['union_required_open_width_m'], result['inferred_required_open_width_m']))
    assert result['precision_reserve_m'] == .002
    assert result['precision_reserve_is_hard_requirement'] is False
    assert result['inferred_projection']['model_sha256'] == model.model_sha256


@pytest.mark.parametrize('field,value', [
    ('opening_width_m', .051), ('opening_width_m', float('nan')),
    ('opening_fit_clearance_each_side_m', -.001), ('stage_reference_pose_index', True),
    ('obb_size_xyz_m', [1., -.1, .02]), ('support_normal_base', [0., 0., 2.])])
def test_malformed_inputs_fail_without_nan_diagnostics(field, value):
    kw = scene(); kw[field] = value
    result = evaluate_measured_aperture(**kw)
    assert not result['ok'] and result['code'] == 'MEASURED_APERTURE_INPUT_INVALID'
    json.dumps(result, allow_nan=False)


def test_final_stage_cannot_label_a_residual_projection_as_measured():
    kw = scene(); kw['planned_stage_T_base_tool0'] = kw['planned_stage_T_base_tool0'].copy()
    kw['planned_stage_T_base_tool0'][2, 3] += .02
    result = evaluate_measured_aperture(**kw)
    assert not result['ok'] and result['code'] == 'MEASURED_APERTURE_INPUT_INVALID'


def task_consumer(monkeypatch):
    """Isolate consumer timing from geometry already covered above.

    The signed historical plan and real pure URDF FK come from the existing
    fixture. Only the geometry result is a deliberate immutable pass double,
    so a timing/context rejection cannot accidentally depend on target shape.
    """
    from test_contact_execution import make_task, module
    node, plan, clock, params = make_task(monkeypatch, hypothetical_contact=True)
    evaluate = Mock(return_value=dict(ok=True, code='MEASURED_APERTURE_OK',
        reason='explicit geometry double', physical_grip_verified=False))
    monkeypatch.setattr(module, 'plan_aperture_evaluator', lambda *args, **kwargs: evaluate)
    return node, plan, clock, params, module, evaluate


@pytest.mark.parametrize('change', ['epoch', 'model', 'gripper', 'authority', 'age'])
def test_context_is_rechecked_after_a_blocking_diagnostic_log(monkeypatch, change):
    node, plan, clock, params, module, evaluate = task_consumer(monkeypatch)
    changed = [False]
    def log(*args):
        if not str(args[0]).startswith('Measured contact aperture:'):
            return
        changed[0] = True
        if change == 'epoch': node._following_epoch_ns += 1
        elif change == 'model': params['/robot_description'] += ' '
        elif change == 'gripper': params['/grasp_6d/remote/gripper_geometry'] = {'changed': True}
        elif change == 'age': clock['now'] += .6
    if change == 'authority':
        node._validate_bound_plan_locked = lambda *args: NS(
            ok=not changed[0], code='EXECUTION_AUTHORITY_REVOKED')
    monkeypatch.setattr(module.rospy, 'loginfo', log)
    assert not node._validate_measured_aperture(plan, {}, 0, 'test stage')
    assert changed[0] and evaluate.call_count == 1
    assert node._last_measured_aperture_sample['code'] == 'MEASURED_APERTURE_EVIDENCE_INVALID'


def test_service_discovery_wait_cannot_spend_the_last_aperture_feedback_age(monkeypatch):
    node, plan, clock, _, module, _ = task_consumer(monkeypatch)
    proxy = Mock()
    monkeypatch.setattr(module.rospy, 'wait_for_service',
                        lambda *args, **kwargs: clock.update(now=21.))
    monkeypatch.setattr(module.rospy, 'ServiceProxy', lambda *args, **kwargs: proxy)
    from geometry_msgs.msg import PoseStamped
    pose = PoseStamped(); pose.header = plan.header; pose.pose = plan.poses[1]
    move = node._contact_linear_service(plan, 1, {})
    with pytest.raises(module._MeasuredApertureRejected):
        move(pose, True)
    proxy.assert_not_called()


def test_each_atomic_action_rechecks_feedback_instead_of_reusing_a_pass_report(monkeypatch):
    node, plan, clock, _, module, evaluate = task_consumer(monkeypatch)
    physical_action = Mock(return_value='explicit inert action')
    guarded = node._guard_measured_aperture_action(plan, {}, 2, 'close commit', physical_action)
    assert guarded() == 'explicit inert action'
    assert physical_action.call_count == 1 and evaluate.call_count == 1
    clock['now'] = 21.
    with pytest.raises(module._MeasuredApertureRejected):
        guarded()
    assert physical_action.call_count == 1


@pytest.mark.parametrize('new_geometry_ok', [True, False])
def test_changed_sdk_values_recompute_fk_and_use_the_new_geometry_result(monkeypatch, new_geometry_ok):
    node, plan, _, _, module, evaluate = task_consumer(monkeypatch)
    original_stamp = node.latest_accepted_joint_state.header.stamp.to_nsec()
    changed = [False]
    def log(*args):
        if not changed[0] and str(args[0]).startswith('Measured contact aperture:'):
            changed[0] = True
            current = node.latest_accepted_joint_state
            current.position[0] += 2.*np.pi/4096.  # One real encoder quantum.
            current.header.stamp = module.rospy.Time.from_sec(19.91)
    monkeypatch.setattr(module.rospy, 'loginfo', log)
    # The geometry consumer is an explicit result double in these timing
    # tests: prove the new FK/result is consumed, not that this toy plan fits.
    evaluate.side_effect = [dict(ok=True, code='MEASURED_APERTURE_OK', reason='old geometry'),
        dict(ok=new_geometry_ok,
             code='MEASURED_APERTURE_OK' if new_geometry_ok else 'MEASURED_APERTURE_INSUFFICIENT',
             reason='geometry re-evaluated at the new FK')]
    assert node._validate_measured_aperture(plan, {}, 2, 'close commit') is new_geometry_ok
    assert evaluate.call_count == 2
    assert not np.array_equal(evaluate.call_args_list[0].args[0], evaluate.call_args_list[1].args[0])
    report = node._last_measured_aperture_sample
    assert report['measurement_refresh_count'] == 1
    assert report['initial_accepted_stamp_ns'] == original_stamp
    assert report['accepted_stamp_ns'] == node.latest_accepted_joint_state.header.stamp.to_nsec()


def test_identical_values_with_a_new_stamp_do_not_refresh_the_old_measurement(monkeypatch):
    node, plan, clock, _, module, evaluate = task_consumer(monkeypatch)
    original_stamp = node.latest_accepted_joint_state.header.stamp.to_nsec()
    def log(*args):
        if str(args[0]).startswith('Measured contact aperture:'):
            node.latest_accepted_joint_state.header.stamp = module.rospy.Time.from_sec(20.5)
            clock['now'] = 20.5
    monkeypatch.setattr(module.rospy, 'loginfo', log)
    assert not node._validate_measured_aperture(plan, {}, 2, 'close commit')
    assert evaluate.call_count == 1
    assert node._last_measured_aperture_sample['accepted_stamp_ns'] == original_stamp
    assert 'STALE' in node._last_measured_aperture_sample['reason']


@pytest.mark.parametrize('late_change', ['z_patch_miss', 'healthy_opening'])
def test_close_recomputed_packet_repeats_final_contact_gates_on_the_same_fk(monkeypatch, late_change):
    from alicia_flexible_grasp.grasp.contact_execution import plan_contact_evaluator
    node, plan, _, params, module, aperture = task_consumer(monkeypatch)
    # This is the original measured contact/height validator, with its original
    # completed endpoint proof. Its earlier pass cannot authorize a later FK.
    assert node._validate_measured_contact_before_close(plan, {})
    old = node.latest_accepted_joint_state
    old_stamp = old.header.stamp.to_nsec()
    arm = np.asarray(old.position[:6], dtype=float)
    fk = module.SerialUrdfFk(params['/robot_description'], module.ARM_NAMES)
    old_actual = fk(arm)
    new_arm = None
    if late_change == 'z_patch_miss':
        evaluate = plan_contact_evaluator(plan, {}, old.position[-1])
        # Select a bounded hypothetical encoder perturbation using the real
        # fixture URDF/FK and original pure contact gate. No hardware commands,
        # source/plan mutations, or invented tool transform are involved.
        for joint in (1, 2, 4):
            for sign in (-1., 1.):
                for cells in range(1, 21):
                    q = arm.copy(); q[joint] += sign*cells*2.*np.pi/4096.
                    actual = fk(q)
                    if (.001 < actual[2, 3]-old_actual[2, 3]
                            and np.linalg.norm(actual[:3, 3]-old_actual[:3, 3]) < .01
                            and evaluate(actual)['code'] == 'MEASURED_CONTACT_PATCH_MISS'):
                        new_arm = q
                        break
                if new_arm is not None: break
            if new_arm is not None: break
        assert new_arm is not None, 'fixture must supply a real FK Z error within the old 10mm endpoint limit'
    changed = [False]
    def log(*args):
        if not changed[0] and str(args[0]).startswith('Measured contact aperture:'):
            changed[0] = True
            packet = node.latest_accepted_joint_state
            if new_arm is None: packet.position[-1] += .00005
            else: packet.position[:6] = new_arm.tolist()
            packet.header.stamp = module.rospy.Time.from_sec(19.91)
    monkeypatch.setattr(module.rospy, 'loginfo', log)
    close = Mock(return_value='explicit inert close')
    guarded = node._guard_measured_aperture_action(plan, {}, 2, 'atomic close', close)
    if late_change == 'z_patch_miss':
        with pytest.raises(module._MeasuredApertureRejected): guarded()
        close.assert_not_called()
    else:
        assert guarded() == 'explicit inert close'
        close.assert_called_once()
    assert changed[0] and aperture.call_count == 2
    report = node._last_measured_aperture_sample
    assert report['measurement_refresh_count'] == 1
    assert report['initial_accepted_stamp_ns'] == old_stamp
    for name in ('final_contact_overlap', 'height_registration_margin'):
        assert report[name]['accepted_stamp_ns'] == report['accepted_stamp_ns']
        assert report[name]['epoch_ns'] == report['epoch_ns']
        assert report[name]['model_sha256'] == report['model_sha256']
        assert report[name]['measured_opening_width_m'] == report['measured_opening_width_m']
    assert report['final_contact_overlap']['ok'] is (late_change != 'z_patch_miss')
