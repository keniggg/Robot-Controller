"""Actual ROS namespace notifications with inert transport and camera endpoints."""
from copy import deepcopy

from rosmaster.paramserver import compute_param_updates
from rospy.impl.paramserver import ParamServerCache

from alicia_flexible_grasp.robot.observation_view_gateway import _camera_configuration_contract
from test_camera_async_publication import camera


class Subscriptions(dict):
    def iterkeys(self):
        return iter(self)


def test_first_temporal_publication_reaches_existing_camera_namespace_subscribers(camera):
    node, ros, _ = camera
    early = ParamServerCache()
    early.set('/camera', deepcopy(ros.params['/camera']))
    assert 'stationary_temporal' not in early.get('/camera')['runtime_profile']
    subscriptions = Subscriptions({'/camera': [('perception', 'inert')]})
    rejected_updates = []
    original = ros.set_param

    def publish(key, value):
        original(key, value)
        for _, update_key, update in compute_param_updates(subscriptions, key, value):
            try:
                early.update(update_key, deepcopy(update))
            except KeyError:
                rejected_updates.append(update_key)

    ros.set_param = publish
    policy = dict(enabled=True, regional_enabled=True, active=False, alpha=.4,
                  delta_depth_units=100, persistence=0, stationary_window_sec=.35)
    node.cam.runtime_profile['stationary_temporal'] = deepcopy(policy)
    node._publish_temporal_status(None)
    expected = deepcopy(node.cfg)
    expected['runtime_profile']['stationary_temporal'] = deepcopy(policy)
    assert not rejected_updates, rejected_updates
    assert _camera_configuration_contract(early.get('/camera')) == _camera_configuration_contract(expected)
    # Established updates still use the existing child key, and changing real
    # policy remains visible to the consumer's strict configuration contract.
    node.cam.runtime_profile['stationary_temporal'].update(active=True, alpha=.5)
    node._publish_temporal_status(None)
    assert not rejected_updates
    assert early.get('/camera')['runtime_profile']['stationary_temporal']['alpha'] == .5
    assert _camera_configuration_contract(early.get('/camera')) != _camera_configuration_contract(expected)
