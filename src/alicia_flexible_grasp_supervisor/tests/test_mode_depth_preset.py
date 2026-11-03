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


@pytest.mark.parametrize('preset', ['high_density', '', 'High_Accuracy', None, True, 3, {}, []])
def test_unsupported_preset_rejected_before_file_or_hardware_access(preset):
    with pytest.raises(ValueError, match='unsupported unknown depth preset'):
        ModeDepthPreset(None, None, dict(unknown_preset=preset), .0001)


def test_omitted_configuration_still_defaults_to_high_accuracy(tmp_path):
    previous, sensor, advanced = setup_policy(tmp_path)
    device = SimpleNamespace(get_info=lambda _: 'test-camera', first_depth_sensor=lambda: sensor)
    policy = ModeDepthPreset(previous.rs, device, dict(baseline_path=str(tmp_path/'baseline.json')), .0001)
    assert policy.evidence()['preset'] is None
    assert policy.evidence()['configured_unknown_preset'] == 'high_accuracy'
    policy.apply('unknown')
    assert sensor.preset == 3 and policy.mode == 'unknown'


def test_explicit_unknown_baseline_keeps_unknown_mode_and_restores_saved_values(tmp_path):
    policy, sensor, advanced = setup_policy(tmp_path, preset='saved_baseline')
    sensor.preset = 3
    advanced.parameters['param-secondpeakdelta'] = '647'
    assert policy.apply('unknown')
    assert policy.mode == 'unknown' and sensor.preset == 0
    assert advanced.parameters['param-secondpeakdelta'] == '325'
    assert not sensor.writes and len(advanced.loads) == 1 and not policy.dirty
    evidence = policy.evidence()
    assert evidence['mode'] == 'unknown' and evidence['preset'] == 'saved_baseline'
    assert evidence['configured_unknown_preset'] == 'saved_baseline'
    assert evidence['invalid_depth_filling'] is False
    assert not policy.apply('unknown') and len(advanced.loads) == 1
    # Changing task mode still applies and verifies the baseline anew, allowing
    # the manager to discard temporal history at the actual task boundary.
    assert policy.apply('carton') and policy.mode == 'carton'
    assert policy.apply('unknown') and policy.mode == 'unknown'
    assert len(advanced.loads) == 3 and not sensor.writes
    policy.close()
    assert policy.mode is None and policy.evidence()['preset'] is None
    assert len(advanced.loads) == 3  # Already restored; no redundant SDK write.


@pytest.mark.parametrize('failure', ['serial', 'parameters', 'units'])
def test_explicit_baseline_preserves_serial_parameter_and_unit_guards(tmp_path, failure):
    if failure == 'serial':
        with pytest.raises(ValueError, match='another camera'):
            setup_policy(tmp_path, serial='another', preset='saved_baseline')
        return
    policy, sensor, advanced = setup_policy(tmp_path, preset='saved_baseline')
    if failure == 'parameters':
        advanced.corrupt_restore = True
    else:
        saved = json.loads(policy.baseline)
        saved['parameters']['param-depthunits'] = '1000'
        policy.baseline = json.dumps(saved)
    with pytest.raises(ValueError, match='restoration mismatch|measurement units'):
        policy.apply('unknown')
    assert policy.mode is None and policy.evidence()['preset'] is None
    assert not sensor.writes


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
    monkeypatch.setattr(module.rospy, 'get_param_cached', lambda *args: {'mode': 'unknown'})
    node._sync_depth_preset()
    assert events == ['unknown', 'profile_published']
    events.clear()
    monkeypatch.setattr(module.rospy, 'get_param_cached', lambda *args: {'mode': 'carton'})
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
    monkeypatch.setattr(module.rospy, 'get_param_cached', forbidden)
    node.cam = SimpleNamespace(set_depth_preset_mode=forbidden)
    node._sync_depth_preset()


def test_cached_mode_update_applies_real_policy_and_restores_baseline(monkeypatch, tmp_path):
    from test_camera_node_recovery import load_camera_node
    module = load_camera_node()
    node = module.CameraNode.__new__(module.CameraNode)
    node.cfg = {'mode_depth_preset': {'enabled': True}}
    policy, sensor, advanced = setup_policy(tmp_path)
    node.cam = SimpleNamespace(set_depth_preset_mode=policy.apply)
    published = []
    node._publish_runtime_camera_params = lambda: published.append(policy.mode)
    cached_selection = {'mode': 'unknown'}
    monkeypatch.setattr(module.rospy, 'get_param_cached', lambda *args: cached_selection)
    def uncached_forbidden(*args):
        raise AssertionError('mode read must not make an uncached master request')
    monkeypatch.setattr(module.rospy, 'get_param', uncached_forbidden)

    node._sync_depth_preset()
    assert sensor.preset == 3
    node._sync_depth_preset()
    assert published == ['unknown']
    # Model a ROS paramUpdate after the initial cache subscription.
    cached_selection['mode'] = 'carton'
    node._sync_depth_preset()
    assert sensor.preset == 0
    assert published == ['unknown', 'carton']
    assert len(sensor.writes) == 1
    assert json.loads(advanced.loads[0]) == json.loads(advanced.loads[1])
