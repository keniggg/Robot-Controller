"""Read-only SDK/encoder evidence must not treat a reused view as arrival."""
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest
import rospy

from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.robot.stationary_following import stationary_following_error


NAMES = ['Joint%d' % i for i in range(1, 7)]
XML = '<robot name="test">' + ''.join(
    '<joint name="Joint%d" type="revolute"><parent link="%s"/>'
    '<child link="%s"/><origin xyz="0.05 0 0"/><axis xyz="0 1 0"/>'
    '<limit lower="-3.14" upper="3.14"/></joint>' % (
        i, 'base_link' if i == 1 else 'L%d' % (i-1),
        'tool0' if i == 6 else 'L%d' % i) for i in range(1, 7)) + '</robot>'


def message(stamp, frame, q, measured=False):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=rospy.Time.from_sec(stamp), frame_id=frame),
        name=NAMES + (['right_finger'] if measured else []),
        position=list(q) + ([.05] if measured else []))


def fixture():
    q = np.zeros(6)
    actual = q.copy()
    actual[1] = -8 * (2*np.pi/4096)
    sdk = [message(t, 'sdk_transmitted', q) for t in [9.5, 9.6, 9.7, 9.8, 9.9, 10.]]
    measured = [message(t+.01, 'sdk_measured', actual, True)
                for t in [9.5, 9.6, 9.7, 9.8, 9.9]]
    return SerialUrdfFk(XML, NAMES), sdk, measured


def run(fk, sdk, measured, **kw):
    return stationary_following_error(fk, sdk, measured, now_sec=10.05,
                                     epoch_ns=9_000_000_000, **kw)


def test_reused_stationary_view_keeps_real_following_error_not_self_pose_zero():
    fk, sdk, measured = fixture()
    original = deepcopy((sdk, measured))
    report = run(fk, sdk, measured)
    expected = fk(measured[-1].position[:6])[:3, 3] - fk(sdk[-1].position)[:3, 3]
    assert report['position_error_m'] == pytest.approx(np.linalg.norm(expected))
    assert report['position_error_m'] > .002
    assert report['joint_error_sdk_counts'][1] == pytest.approx(-8)
    assert report['measurement_kind'] == 'stationary_sdk_following_not_executed_endpoint'
    assert report['certifies_command_response_or_calibration'] is False
    assert (sdk, measured) == original


def test_named_order_does_not_change_fk_or_error():
    fk, sdk, measured = fixture()
    baseline = run(fk, sdk, measured)
    for msg in sdk + measured:
        msg.name.reverse()
        msg.position.reverse()
    assert run(fk, sdk, measured) == baseline


@pytest.mark.parametrize('mutation', [
    lambda s, m: s.clear(),
    lambda s, m: m.clear(),
    lambda s, m: s.__delitem__(slice(None, -1)),
    lambda s, m: setattr(s[-1].header, 'frame_id', 'sdk_measured'),
    lambda s, m: setattr(m[-1].header, 'frame_id', ''),
    lambda s, m: s[-1].position.__setitem__(1, .001),
    lambda s, m: m[-1].position.__setitem__(1, .1),
    lambda s, m: m[-1].position.__setitem__(1, float('nan')),
    lambda s, m: m[-1].name.__setitem__(1, 'Joint1'),
    lambda s, m: m[-1].position.__setitem__(-1, .06),
    lambda s, m: setattr(s[-1].header, 'stamp', rospy.Time.from_sec(11)),
    lambda s, m: setattr(s[-1].header, 'stamp', rospy.Time.from_sec(9.5)),
    lambda s, m: s.__delitem__(slice(1, None)),
])
def test_missing_moving_or_corrupt_evidence_cannot_become_zero(mutation):
    fk, sdk, measured = fixture()
    mutation(sdk, measured)
    with pytest.raises(ValueError):
        run(fk, sdk, measured)


@pytest.mark.parametrize('maximum_age_sec', [0, -.1, float('nan'), 1.])
def test_freshness_contract_cannot_be_disabled(maximum_age_sec):
    with pytest.raises(ValueError):
        run(*fixture(), maximum_age_sec=maximum_age_sec)


def test_no_pre_power_cycle_window_is_reused():
    fk, sdk, measured = fixture()
    with pytest.raises(ValueError):
        stationary_following_error(fk, sdk, measured, now_sec=10.05,
                                   epoch_ns=9_850_000_000)


def test_one_encoder_count_jitter_is_not_reported_as_new_command_response():
    fk, sdk, measured = fixture()
    measured[-1].position[1] += 2*np.pi/4096
    report = run(fk, sdk, measured)
    assert report['joint_error_sdk_counts'][1] == pytest.approx(-7)
    assert not report['certifies_command_response_or_calibration']
