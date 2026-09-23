import json
from types import SimpleNamespace

import pytest

from alicia_flexible_grasp.vision.depth_preset import ModeDepthPreset


class Sensor:
    def __init__(self):
        self.scale = .0001
        self.preset = 0.
        self.fail = False
        self.alter_units = False
        self.writes = []

    def get_depth_scale(self):
        return self.scale

    def set_option(self, option, value):
        self.writes.append((option, value))
        self.preset = value
        if self.alter_units:
            self.scale = .001
        if self.fail:
            raise RuntimeError('sensor write failed')

    def get_option(self, option):
        return self.preset


class Advanced:
    def __init__(self, sensor):
        self.sensor = sensor
        self.parameters = {'param-depthunits': '100', 'param-secondpeakdelta': '325'}
        self.loads = []
        self.corrupt_restore = False

    def load_json(self, content):
        self.loads.append(content)
        self.parameters = json.loads(content)['parameters']
        self.sensor.scale = float(self.parameters['param-depthunits']) * 1e-6
        self.sensor.preset = 0.
        if self.corrupt_restore:
            self.parameters['param-secondpeakdelta'] = '0'

    def serialize_json(self):
        return json.dumps({'parameters': self.parameters})


def setup_policy(tmp_path, serial='test-camera', preset='high_accuracy'):
    sensor = Sensor()
    advanced = Advanced(sensor)
    path = tmp_path/'baseline.json'
    path.write_text(json.dumps({'serial_number': serial,
                               'advanced_settings': json.loads(advanced.serialize_json())}))
    device = SimpleNamespace(get_info=lambda _: 'test-camera', first_depth_sensor=lambda: sensor)
    rs = SimpleNamespace(camera_info=SimpleNamespace(serial_number='serial'),
                         option=SimpleNamespace(visual_preset='preset'),
                         rs400_advanced_mode=lambda _: advanced)
    policy = ModeDepthPreset(rs, device, dict(baseline_path=str(path), unknown_preset=preset), .0001)
    return policy, sensor, advanced


def test_unknown_mode_is_idempotent_and_carton_restores_exact_baseline(tmp_path):
    policy, sensor, advanced = setup_policy(tmp_path)
    assert policy.apply('unknown')
    assert sensor.preset == 3
    assert not policy.apply('unknown')
    assert len(sensor.writes) == 1
    assert policy.evidence()['invalid_depth_filling'] is False
    assert policy.apply('carton')
    assert sensor.preset == 0
    assert len(advanced.loads) == 2
    assert json.loads(advanced.loads[0]) == json.loads(advanced.loads[1])
    assert not policy.dirty


def test_camera_restart_uses_saved_baseline_not_current_sensor_preset(tmp_path):
    policy, sensor, advanced = setup_policy(tmp_path)
    sensor.preset = 3
    advanced.parameters['param-secondpeakdelta'] = '647'
    assert policy.apply('carton')
    assert advanced.parameters['param-secondpeakdelta'] == '325'
    assert sensor.preset == 0


def test_shutdown_restores_baseline_after_unknown(tmp_path):
    policy, sensor, advanced = setup_policy(tmp_path)
    policy.apply('unknown')
    policy.close()
    assert sensor.preset == 0
    assert policy.mode is None
    assert not policy.dirty


@pytest.mark.parametrize('failure', ['fail', 'alter_units'])
def test_partial_failure_restores_baseline_and_does_not_claim_applied(tmp_path, failure):
    policy, sensor, advanced = setup_policy(tmp_path)
    setattr(sensor, failure, True)
    with pytest.raises((RuntimeError, ValueError)):
        policy.apply('unknown')
    assert policy.mode is None
    assert sensor.get_depth_scale() == pytest.approx(.0001)
    assert sensor.preset == 0


def test_restore_mismatch_prevents_new_preset(tmp_path):
    policy, sensor, advanced = setup_policy(tmp_path)
    advanced.corrupt_restore = True
    with pytest.raises(ValueError, match='restoration mismatch'):
        policy.apply('unknown')
    assert not sensor.writes
    assert policy.mode is None


@pytest.mark.parametrize('mode', ['bad', None, '', 'Unknown'])
def test_invalid_mode_does_not_touch_sensor(tmp_path, mode):
    policy, sensor, advanced = setup_policy(tmp_path)
    with pytest.raises(ValueError, match='invalid depth preset task mode'):
        policy.apply(mode)
    assert not sensor.writes
    assert not advanced.loads


def test_other_camera_baseline_rejected_before_hardware_change(tmp_path):
    with pytest.raises(ValueError, match='another camera'):
        setup_policy(tmp_path, serial='other-camera')


def test_unqualified_preset_rejected(tmp_path):
    with pytest.raises(ValueError, match='qualified high_accuracy'):
        setup_policy(tmp_path, preset='high_density')


def test_manager_records_active_mode_and_restores_on_stop(tmp_path):
    from alicia_flexible_grasp.vision.realsense_manager import RealSenseManager
    policy, sensor, advanced = setup_policy(tmp_path)
    manager = RealSenseManager()
    manager.mode_depth_preset = policy
    stopped = []
    manager.pipeline = SimpleNamespace(stop=lambda: stopped.append(True))
    assert manager.set_depth_preset_mode('unknown')
    assert manager.runtime_profile['mode_depth_preset']['preset'] == 'high_accuracy'
    manager.stop()
    assert stopped == [True]
    assert sensor.preset == 0


def test_pipeline_stops_even_if_sensor_restore_fails(tmp_path):
    from alicia_flexible_grasp.vision.realsense_manager import RealSenseManager
    policy, sensor, advanced = setup_policy(tmp_path)
    manager = RealSenseManager()
    manager.mode_depth_preset = policy
    stopped = []
    manager.pipeline = SimpleNamespace(stop=lambda: stopped.append(True))
    manager.set_depth_preset_mode('unknown')
    advanced.corrupt_restore = True
    with pytest.raises(ValueError, match='restoration mismatch'):
        manager.stop()
    assert stopped == [True]


def test_camera_mode_sync_publishes_profile_before_reading_next_frame(monkeypatch):
    from test_camera_node_recovery import load_camera_node
    module = load_camera_node()
    node = module.CameraNode.__new__(module.CameraNode)
    node.cfg = {'mode_depth_preset': {'enabled': True}}
    events = []
    node.cam = SimpleNamespace(set_depth_preset_mode=lambda mode: events.append(mode) or True)
    node._publish_runtime_camera_params = lambda: events.append('profile_published')
    monkeypatch.setattr(module.rospy, 'get_param', lambda *args: {'mode': 'unknown'})
    node._sync_depth_preset()
    assert events == ['unknown', 'profile_published']
    events.clear()
    monkeypatch.setattr(module.rospy, 'get_param', lambda *args: {'mode': 'carton'})
    node._sync_depth_preset(publish=False)
    assert events == ['carton']


def test_disabled_camera_mode_policy_does_not_read_mode_or_touch_sensor(monkeypatch):
    from test_camera_node_recovery import load_camera_node
    module = load_camera_node()
    node = module.CameraNode.__new__(module.CameraNode)
    node.cfg = {}
    def forbidden(*args):
        raise AssertionError('disabled mode policy touched hardware or configuration')
    monkeypatch.setattr(module.rospy, 'get_param', forbidden)
    node.cam = SimpleNamespace(set_depth_preset_mode=forbidden)
    node._sync_depth_preset()
