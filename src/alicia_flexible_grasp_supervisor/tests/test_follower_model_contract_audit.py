"""Coordinate relabeling must not be mistaken for physical joint sign changes."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest
from tf.transformations import rotation_matrix

from test_stationary_following import XML, NAMES
from alicia_flexible_grasp.robot.stationary_following import SerialUrdfFk

PATH = Path(__file__).resolve().parents[3] / 'tools/audit_follower_model_contract_readonly.py'
spec = importlib.util.spec_from_file_location('follower_model_audit', PATH)
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def with_tool_rotation():
    return XML.replace('link="tool0"', 'link="old_tool"').replace('</robot>',
        '<joint name="new_tool" type="fixed"><parent link="old_tool"/>'
        '<child link="tool0"/><origin rpy="3.141592653589793 0 0"/></joint></robot>')


def test_terminal_frame_flip_does_not_require_joint_sign_flip():
    old = SerialUrdfFk(XML, NAMES)
    new = SerialUrdfFk(with_tool_rotation(), NAMES)
    rows = np.random.default_rng(9).uniform(-1, 1, size=(30, 6))
    report = audit.compare(old, new, rows)
    stats = report['statistics']
    assert stats['raw']['rotation_deg']['maximum'] == pytest.approx(180)
    assert stats['raw']['translation_mm']['maximum'] < 1e-9
    assert stats['zero_anchored_frame']['rotation_deg']['maximum'] < 1e-10
    assert stats['zero_anchored_frame']['translation_mm']['maximum'] < 1e-9
    assert report['physical_calibration_or_global_equivalence_proven'] is False
    assert all(v['signed_direction_dot'] == pytest.approx(1.) for v in report['zero_axes'].values())


def test_joint_sign_change_cannot_be_hidden_by_zero_tool_alignment():
    old = SerialUrdfFk(XML, NAMES)
    new = SerialUrdfFk(XML.replace('xyz="0 1 0"', 'xyz="0 -1 0"', 1), NAMES)
    report = audit.compare(old, new, [[.1, 0, 0, 0, 0, 0]])
    assert report['zero_axes']['Joint1']['signed_direction_dot'] == pytest.approx(-1.)
    assert report['statistics']['zero_anchored_frame']['rotation_deg']['maximum'] > 10


@pytest.mark.parametrize('rows', [[], [[0]*5], [[float('nan')]*6], [[4]*6]])
def test_bad_shape_nonfinite_or_out_of_limits_samples_fail(rows):
    fk = SerialUrdfFk(XML, NAMES)
    with pytest.raises(ValueError):
        audit.compare(fk, fk, rows)


def test_audit_has_no_live_ros_control_or_fitted_offsets():
    source = PATH.read_text()
    for forbidden in ('rospy.init_node(', 'rospy.Publisher(', 'ServiceProxy(',
                      'least_squares(', 'minimize(', 'set_param('):
        assert forbidden not in source


def simple_mjcf():
    return ('<mujoco><compiler angle="radian"/><worldbody>'
            '<body name="base_link" quat="0 0 0 1">' + ''.join(
                '<body name="Link%d" pos="0.05 0 0"><joint name="Joint%d" '
                'axis="0 1 0" range="-3.14 3.14"/>' % (i, i) for i in range(1, 7)) +
            '<body name="Link7" pos="0 .02 .03"><joint type="slide"/></body>'
            '<body name="Link8" pos="0 -.02 .03"><joint type="slide"/></body>' +
            '</body>'*7 + '</worldbody></mujoco>')


def test_mjcf_uses_world_rotation_and_open_finger_midpoint():
    fk = audit.mjcf_arm_fk(simple_mjcf())
    q = np.array([.2, -.3, .1, .4, -.1, .2])
    expected = rotation_matrix(np.pi, [0, 0, 1])
    link = np.eye(4)
    link[0, 3] = .05
    for angle in q:
        expected = expected @ link @ rotation_matrix(angle, [0, 1, 0])
    suffix = np.eye(4)
    suffix[2, 3] = .03
    np.testing.assert_allclose(fk(q), expected @ suffix, atol=1e-14)
    assert fk(np.zeros(6))[:3, 3] == pytest.approx([-.3, 0, .03])


@pytest.mark.parametrize('before,after', [
    ('<mujoco>', '<mujoco><default/>'),
    ('<worldbody>', '<include file="other.xml"/><worldbody>'),
    ('<worldbody>', '<worldbody><frame/>'),
    ('angle="radian"', 'angle="degree"'),
    ('name="base_link"', 'name="base_link" mocap="true"'),
    ('name="Link1"', 'name="Link1" euler="0 0 0"'),
    ('name="Joint1"', 'name="Joint1" ref=".1"'),
    ('name="Joint1"', 'name="Joint1" pos="0 0 1"'),
    ('name="Joint1"', 'name="unrecognized"'),
    ('type="slide"', 'type="hinge"'),
    ('<body name="Link1"', '<freejoint/><body name="Link1"'),
])
def test_mjcf_audit_rejects_unimplemented_conventions(before, after):
    with pytest.raises(ValueError):
        audit.mjcf_arm_fk(simple_mjcf().replace(before, after, 1))


def test_actual_server_mjcf_is_not_the_same_named_local_urdf():
    root = PATH.parent.parent
    xml = (root / 'src/arm-mujoco/synriard/mjcf/Alicia_D_v5_6/Alicia_D_v5_6_gripper_50mm.xml').read_text()
    fk = audit.mjcf_arm_fk(xml)
    # Independent world-frame reference, not URDF base_link without its root rotation.
    assert fk(np.zeros(6))[:3, 3] == pytest.approx(
        [.313474451, .000249444299, .107336709], abs=1e-9)


def test_following_sensitivity_keeps_original_angles_and_model_provenance():
    old = SerialUrdfFk(XML, NAMES)
    candidate = SerialUrdfFk(with_tool_rotation(), NAMES)
    command, actual = [.1]*6, [.09]*6
    gap = audit.pose_error(old(command), old(actual))
    report = {'model_sha256': old.model_sha256, 'windows': [{
        'window': 'hold', 'sdk_positions_rad': command, 'accepted_positions_rad': actual,
        'position_error_m': gap['translation_mm']/1000}]}
    result = audit.following_sensitivity(old, candidate, report)
    assert result['execution_authority'] is False
    assert result['windows'][0]['candidate_reference_tcp'] == pytest.approx(gap)
    report['windows'][0]['position_error_m'] = 0
    with pytest.raises(ValueError, match='scalar'):
        audit.following_sensitivity(old, candidate, report)
    report['model_sha256'] = 'different'
    with pytest.raises(ValueError, match='different frozen model'):
        audit.following_sensitivity(old, candidate, report)
